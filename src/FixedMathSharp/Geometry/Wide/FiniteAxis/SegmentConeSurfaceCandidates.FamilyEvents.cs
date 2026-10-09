//=======================================================================
// SegmentConeSurfaceCandidates.FamilyEvents.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Finite exact certificates for continuous families clipped by authored normal halfspaces.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    // Full Signed320 authored covectors become <452 bits in the relative
    // frame. Circle coefficients are <652, their discriminant <1306, and
    // homogeneous normals <1504. Rational rim bounds are <1320 and their
    // substituted normals <2220. Interval midpoint products and side feet
    // stay <3700. These 4096-bit fields retain complete coordinates; squared
    // metrics below use separately sized complete-product scratch.
    internal const int FamilyWords = 64;

    internal static int GetMaximumFamilyEventCount(SegmentConeSurfaceCandidate candidate, int halfspaceCount)
    {
        if (halfspaceCount < 0) throw new ArgumentOutOfRangeException(nameof(halfspaceCount));
        checked
        {
            int k = halfspaceCount + 4; // Segment polarity and finite-axis cap constraints.
            if (candidate.Input.Radius == Fixed64.Zero && candidate.Feature != ConeSurfaceFeature.Base)
                return (candidate.Family == ConeSurfaceFamily.SegmentInterval ? 3 : 1) * (6 + 6 * k + k * (k - 1));
            if (candidate.Family == ConeSurfaceFamily.SegmentInterval) return 3;
            if (candidate.Family == ConeSurfaceFamily.RotationCircle) return 4 + 2 * k;
            if (candidate.Family == ConeSurfaceFamily.NormalCone)
                return candidate.Feature == ConeSurfaceFeature.Apex ? 5 + 2 * k + k * (k - 1) / 2 : 2 + k;
            return 0;
        }
    }

    internal static bool AccumulateFamilyEvents(SegmentConeSurfaceCandidate candidate, scoped ReadOnlySpan<WideAxis3> fan,
        ConeSurfacePointLocation location, scoped ref ConeSurfaceFamilySelection selection)
    {
        if (location == ConeSurfacePointLocation.SegmentIntervalUnspecified)
            throw new ArgumentException("A family query requires Start, Interior, or End incident halfspaces.", nameof(location));
        if (candidate.Family == ConeSurfaceFamily.None
            || candidate.Family != ConeSurfaceFamily.SegmentInterval && candidate.PointLocation != location)
            return false;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        if (geometry.Edge.IsZero && location != ConeSurfacePointLocation.Start) return false;
        int before = selection.Count;
        int firstParameter = candidate.Family != ConeSurfaceFamily.SegmentInterval ? -1
            : location == ConeSurfacePointLocation.Start ? 3 : location == ConeSurfacePointLocation.End ? 4 : 0;
        int lastParameter = firstParameter == 0 ? 2 : firstParameter;
        int constraints = fan.Length + 4;
        for (int parameter = firstParameter; parameter <= lastParameter; parameter++)
        {
            if (candidate.Input.Radius == Fixed64.Zero && candidate.Feature != ConeSurfaceFeature.Base)
            {
                // A nonzero polyhedral cone has a coordinate direction, a
                // one-plane lineality direction, or a two-plane extreme ray.
                for (int axis = 0; axis < 3; axis++)
                for (int branch = -1; branch <= 1; branch += 2)
                    AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.Axis, axis, -1, branch, parameter, location), ref selection);
                for (int a = 0; a < constraints; a++)
                {
                    for (int axis = 0; axis < 3; axis++)
                    for (int branch = -1; branch <= 1; branch += 2)
                        AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.AxisPlane, a, axis, branch, parameter, location), ref selection);
                    for (int b = a + 1; b < constraints; b++)
                    for (int branch = -1; branch <= 1; branch += 2)
                        AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.AxisPair, a, b, branch, parameter, location), ref selection);
                }
            }
            else if (candidate.Family == ConeSurfaceFamily.SegmentInterval)
                AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.Interval, 0, 0, 0, parameter, location), ref selection);
            else if (candidate.Feature == ConeSurfaceFeature.Rim && candidate.Family == ConeSurfaceFamily.NormalCone)
            {
                // The generators and every linear-boundary intersection
                // contain the endpoints of every nonempty clipped interval.
                for (int bound = -2; bound < constraints; bound++)
                    AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.RimBound, bound, 0, 0, parameter, location), ref selection);
            }
            else
            {
                // Every nonempty proper closed subset of a circle cut by
                // halfplanes has a boundary root. Seams cover the full circle.
                for (int seam = 0; seam < 4; seam++)
                    AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.CircleSeam, seam, 0, 0, parameter, location), ref selection);
                for (int line = 0; line < constraints; line++)
                for (int branch = -1; branch <= 1; branch += 2)
                    AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.CircleBoundary, line, 0, branch, parameter, location), ref selection);
                if (candidate.Family == ConeSurfaceFamily.NormalCone)
                {
                    // A clipped disk also needs the center and polygon
                    // vertices: its entire feasible polygon may lie inside
                    // the circle, with no admissible circle boundary point.
                    AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.DiskCenter, 0, 0, 0, parameter, location), ref selection);
                    for (int a = 0; a < constraints; a++)
                    for (int b = a + 1; b < constraints; b++)
                        AddFamilyEvent(candidate, geometry, fan, new(ConeSurfaceFamilyEventKind.DiskPair, a, b, 0, parameter, location), ref selection);
                }
            }
        }
        return before != selection.Count;
    }

    private static void AddFamilyEvent(SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        scoped ReadOnlySpan<WideAxis3> fan, ConeSurfaceFamilyEvent item, scoped ref ConeSurfaceFamilySelection selection)
    {
        Span<ulong> data = stackalloc ulong[24 * FamilyWords], root = stackalloc ulong[FamilyWords];
        Span<int> signs = stackalloc int[24];
        var witness = new ConeSurfaceFamilyWitness(data, signs, root);
        if (!BuildFamilyEvent(candidate, geometry, fan, item, ref witness)) return;
        if (witness[0].Sign(root) == 0 && witness[1].Sign(root) == 0 && witness[2].Sign(root) == 0) return;
        for (int index = 0; index < fan.Length + 4; index++)
        {
            FamilyConstraint(geometry, fan, item, witness, index, out Signed576 x, out Signed576 y, out Signed576 z);
            if (QuadraticNormalDotSign(witness.Values, witness.Signs, witness.Root, x, y, z, FamilyWords) > 0) return;
        }
        selection.Add(item);
    }

    internal static bool TryGetFamilyWorldAnchors(SegmentConeSurfaceCandidate candidate, scoped ReadOnlySpan<WideAxis3> fan,
        ConeSurfaceFamilyEvent item, out Vector3d first, out Vector3d second)
    {
        Span<ulong> data = stackalloc ulong[24 * FamilyWords], root = stackalloc ulong[FamilyWords];
        Span<int> signs = stackalloc int[24];
        var witness = new ConeSurfaceFamilyWitness(data, signs, root);
        first = second = default;
        if (!BuildFamilyEvent(candidate, new SegmentConeSurfaceGeometry(candidate.Input), fan, item, ref witness)) return false;
        return ConePlaneRayPointMaterialization.TryGetWorldPoint(candidate.Input.Center, candidate.Input.ConeRotation,
                witness[3], witness[4], witness[5], witness[9], root, out first)
            && ConePlaneRayPointMaterialization.TryGetWorldPoint(candidate.Input.Center, candidate.Input.ConeRotation,
                witness[6], witness[7], witness[8], witness[9], root, out second);
    }

    internal static FixedContactAnchors GetFamilyContact(SegmentConeSurfaceCandidate candidate, scoped ReadOnlySpan<WideAxis3> fan,
        ConeSurfaceFamilyEvent item)
    {
        Span<ulong> data = stackalloc ulong[24 * FamilyWords], root = stackalloc ulong[FamilyWords];
        Span<int> signs = stackalloc int[24];
        var witness = new ConeSurfaceFamilyWitness(data, signs, root);
        if (!BuildFamilyEvent(candidate, new SegmentConeSurfaceGeometry(candidate.Input), fan, item, ref witness))
            throw new InvalidOperationException("The family certificate requires its original candidate and halfspaces.");
        Vector3d first = RoundSegmentPoint(candidate.Input.Segment, witness[10], witness[11], root), second = default;
        for (int axis = 0; axis < 3; axis++)
        {
            second[axis] = RoundRatio(witness[6 + axis], witness[9], root);
        }
        // The original candidate and this exact displacement retain the
        // family's constant depth; neither normalized normals nor rounded
        // anchors participate in depth admission or comparison.
        int squareWords = 2 * FamilyWords + WideArithmetic.GetActiveMagnitudeLength(root) + 3;
        Span<ulong> work = stackalloc ulong[14 * squareWords]; Span<int> workSigns = stackalloc int[14];
        ContactQuadratic delta = At(work, workSigns, 0, squareWords), square = At(work, workSigns, 1, squareWords);
        ContactQuadratic metric = At(work, workSigns, 2, squareWords), divisor = At(work, workSigns, 3, squareWords);
        metric.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            witness[3 + axis].CopyTo(delta); delta.Add(witness[6 + axis], -1);
            Multiply(delta, delta, root, square); metric.Add(square);
        }
        Multiply(witness[9], witness[9], root, divisor);
        bool represented = TryRoundSquareRootRatio(metric, divisor, root, out Fixed64 depth);
        Vector3d normal = FamilyWorldNormal(candidate.Input.ConeRotation, witness, work, workSigns, squareWords);
        return new FixedContactAnchors(new FixedPointAnchor(candidate.Input.Origin, candidate.Input.Rotation, first),
            new FixedPointAnchor(candidate.Input.Center, candidate.Input.ConeRotation, second), normal,
            represented ? depth : Fixed64.MaxValue, !represented);
    }

    private static Vector3d FamilyWorldNormal(FixedQuaternion rotation, scoped ConeSurfaceFamilyWitness witness,
        Span<ulong> work, Span<int> signs, int words)
    {
        var basis = new WideRationalBasis3d(rotation);
        ContactQuadratic metric = At(work, signs, 0, words), product = At(work, signs, 1, words);
        ContactQuadratic term = At(work, signs, 2, words), square = At(work, signs, 3, words);
        metric.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic normal = At(work, signs, 4 + axis, words);
            Scale(witness[0], ExtendFamily(axis == 0 ? basis.Xx : axis == 1 ? basis.Xy : basis.Xz), normal);
            Scale(witness[1], ExtendFamily(axis == 0 ? basis.Yx : axis == 1 ? basis.Yy : basis.Yz), term); normal.Add(term);
            Scale(witness[2], ExtendFamily(axis == 0 ? basis.Zx : axis == 1 ? basis.Zy : basis.Zz), term); normal.Add(term);
            Multiply(normal, normal, witness.Root, square); metric.Add(square);
        }
        Vector3d result = default;
        Signed576 rawSquared = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(Fixed64.One), Signed192.Raw(Fixed64.One)));
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic normal = At(work, signs, 4 + axis, words);
            Multiply(normal, normal, witness.Root, square); Scale(square, rawSquared, product);
            bool fits = TryRoundSquareRootRatio(product, metric, witness.Root, out Fixed64 coordinate);
            System.Diagnostics.Debug.Assert(fits);
            result[axis] = normal.Sign(witness.Root) < 0 ? -coordinate : coordinate;
        }
        return result;
    }

    private static Signed576 ExtendFamily(Signed192 value) => Signed576.ExtendValue(Signed320.ExtendValue(value));
}

internal enum ConeSurfaceFamilyEventKind { Interval, CircleSeam, CircleBoundary, DiskCenter, DiskPair, RimBound, Axis, AxisPlane, AxisPair }

/// <summary>Compact exact construction provenance; replay uses the original candidate and halfspace order.</summary>
internal readonly struct ConeSurfaceFamilyEvent
{
    internal readonly ConeSurfaceFamilyEventKind Kind;
    internal readonly int First, Second, Branch, Parameter;
    internal ConeSurfacePointLocation PointLocation { get; }
    internal ConeSurfaceFamilyEvent(ConeSurfaceFamilyEventKind kind, int first, int second, int branch, int parameter, ConeSurfacePointLocation location)
    { Kind = kind; First = first; Second = second; Branch = branch; Parameter = parameter; PointLocation = location; }
    internal FixedContactAnchors GetContact(SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> halfspaces) =>
        SegmentConeSurfaceCandidates.GetFamilyContact(candidate, halfspaces, this);
    internal bool TryGetWorldAnchors(SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> halfspaces, out Vector3d first, out Vector3d second) =>
        SegmentConeSurfaceCandidates.TryGetFamilyWorldAnchors(candidate, halfspaces, this, out first, out second);
}

/// <summary>Caller-owned finite pool of already admitted exact family events.</summary>
internal ref struct ConeSurfaceFamilySelection
{
    private readonly Span<ConeSurfaceFamilyEvent> events;
    internal int Count { get; private set; }
    internal ConeSurfaceFamilyEvent this[int index] => events[..Count][index];
    internal ConeSurfaceFamilySelection(Span<ConeSurfaceFamilyEvent> events) { this.events = events; Count = 0; }
    internal void Add(ConeSurfaceFamilyEvent item)
    {
        if (Count == events.Length) throw new InvalidOperationException("The caller-owned family event storage is full.");
        events[Count++] = item;
    }
}

// Slots 0..2 are a cone-local unnormalized normal; 3..5 and 6..8 are
// centered raw p/q numerators over common slot 9; 10..11 retain segment t.
internal readonly ref struct ConeSurfaceFamilyWitness
{
    internal readonly Span<ulong> Values, Root;
    internal readonly Span<int> Signs;
    internal ContactQuadratic this[int index] => ContactQuadratic.At(Values, Signs, index, SegmentConeSurfaceCandidates.FamilyWords);
    internal ConeSurfaceFamilyWitness(Span<ulong> values, Span<int> signs, Span<ulong> root)
    { Values = values; Signs = signs; Root = root; values.Clear(); signs.Clear(); root.Clear(); }
}
