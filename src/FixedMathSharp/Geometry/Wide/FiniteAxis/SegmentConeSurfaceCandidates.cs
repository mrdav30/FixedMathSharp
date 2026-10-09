//=======================================================================
// SegmentConeSurfaceCandidates.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>Exact finite-segment witnesses paired with finite-cone support features.</summary>
/// <content>Finite support strata, exact halfspace-clipped families, retained certificates, and caller-owned selection.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    // Four quartic charts contribute at most 16 roots, with two horizontal
    // principal feet, four endpoint rim branches, six side branches, three
    // apex feet and one base descriptor. Other strata replace these charts.
    internal const int MaximumCandidates = 32;

    /// <summary>Compares the exact nonnegative depths of certificates from the same input segment and cone frame.</summary>
    /// <remarks>Clipping a retained family changes its witnesses, but not its source depth. No rounded anchor or depth participates.</remarks>
    internal static int CompareDepths(SegmentConeSurfaceCandidate first, SegmentConeSurfaceCandidate second)
    {
        var geometry = new SegmentConeSurfaceGeometry(first.Input);
        bool firstRoot = first.Feature == ConeSurfaceFeature.Rim && first.Family == ConeSurfaceFamily.None && first.Chart >= 0;
        bool secondRoot = second.Feature == ConeSurfaceFeature.Rim && second.Family == ConeSurfaceFamily.None && second.Chart >= 0;
        if (!firstRoot && !secondRoot)
        {
            Span<ulong> a = stackalloc ulong[ConvexContactCandidate.Slots * Words], b = stackalloc ulong[ConvexContactCandidate.Slots * Words];
            Span<int> aSigns = stackalloc int[ConvexContactCandidate.Slots], bSigns = stackalloc int[ConvexContactCandidate.Slots];
            int aSign = BuildAnalyticSquaredDepth(geometry, first, a, aSigns), bSign = BuildAnalyticSquaredDepth(geometry, second, b, bSigns);
            return WideConvexPrismRelations.CompareConvexContactCandidates(new ConvexContactCandidate(a, aSigns, aSign), new ConvexContactCandidate(b, bSigns, bSign));
        }
        if (!firstRoot || !secondRoot)
            return firstRoot ? CompareRimToAnalyticDepth(geometry, first, second) : -CompareRimToAnalyticDepth(geometry, second, first);

        Span<ulong> firstValues = stackalloc ulong[5 * CircularRimContactAlgebra.ValueWords], secondValues = stackalloc ulong[5 * CircularRimContactAlgebra.ValueWords];
        Span<sbyte> firstSigns = stackalloc sbyte[5], secondSigns = stackalloc sbyte[5];
        Span<ulong> firstCell = stackalloc ulong[TriangleConeRimContacts.ValueCellWords], secondCell = stackalloc ulong[TriangleConeRimContacts.ValueCellWords];
        bool firstPositive = TryGetRimSquaredDepth(geometry, first, firstValues, firstSigns, firstCell, out FiniteAxisValueRoot firstValue);
        bool secondPositive = TryGetRimSquaredDepth(geometry, second, secondValues, secondSigns, secondCell, out FiniteAxisValueRoot secondValue);
        // A zero depth is outside the value isolator's (0,1] domain. The same
        // input precondition makes both positive roots' scale/shift identical.
        return !firstPositive || !secondPositive ? firstPositive.CompareTo(secondPositive)
            : WideFiniteAxisIntersection.CompareFiniteValueRoots(firstValue, secondValue);
    }

    private static int CompareRimToAnalyticDepth(in SegmentConeSurfaceGeometry geometry,
        SegmentConeSurfaceCandidate rim, SegmentConeSurfaceCandidate analytic)
    {
        Span<ulong> data = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        int gapSign = BuildAnalyticSquaredDepth(geometry, analytic, data, signs);
        Span<ulong> values = stackalloc ulong[5 * CircularRimContactAlgebra.ValueWords];
        Span<sbyte> valueSigns = stackalloc sbyte[5];
        Span<ulong> cell = stackalloc ulong[TriangleConeRimContacts.ValueCellWords];
        return TryGetRimSquaredDepth(geometry, rim, values, valueSigns, cell, out FiniteAxisValueRoot value)
            ? ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, ref value, new ConvexContactCandidate(data, signs, gapSign))
            : -gapSign;
    }

    private static int BuildAnalyticSquaredDepth(in SegmentConeSurfaceGeometry geometry,
        SegmentConeSurfaceCandidate candidate, Span<ulong> destination, Span<int> destinationSigns)
    {
        destination.Clear(); destinationSigns.Clear();
        ContactQuadratic numerator = new(destination.Slice(7 * Words, 2 * Words), destinationSigns.Slice(7, 2));
        Span<ulong> root = destination.Slice(9 * Words, Words);
        Span<ulong> scratch = stackalloc ulong[24 * Words];
        Span<int> scratchSigns = stackalloc int[24];
        ContactQuadratic denominator = At(scratch, scratchSigns, 10), product = At(scratch, scratchSigns, 11);
        Signed576 one = Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1)));
        denominator.Set(one);
        if (candidate.Family == ConeSurfaceFamily.NormalCone || candidate.Feature == ConeSurfaceFeature.Side && geometry.Radius.IsZero)
        {
            Slot(destination, 10)[0] = 1;
            return 0;
        }
        if (candidate.Feature == ConeSurfaceFeature.Apex)
        {
            bool admitted = TryBuildApex(geometry, candidate.RootOrdinal, out _, out Signed576 divisor, out Signed832 x, out Signed832 y, out Signed832 z);
            System.Diagnostics.Debug.Assert(admitted);
            ContactQuadratic coordinate = At(scratch, scratchSigns, 0);
            Span<Signed832> coordinates = stackalloc Signed832[3] { x, y, z };
            foreach (Signed832 component in coordinates)
            {
                coordinate.Clear(); WideArithmetic.GetMagnitude(component, coordinate.Rational[..13]); coordinate.Signs[0] = component.Sign;
                Multiply(coordinate, coordinate, root, product); numerator.Add(product);
            }
            coordinate.Set(divisor); Multiply(coordinate, coordinate, root, denominator);
        }
        else if (candidate.Feature == ConeSurfaceFeature.Base)
        {
            product.Set(Signed576.ExtendValue(WideArithmetic.AddSigned320(candidate.RootOrdinal == 1 ? geometry.B.Y : geometry.A.Y, geometry.Finite.ShapeFrame.Cap)));
            Multiply(product, product, root, numerator);
        }
        else if (candidate.Feature == ConeSurfaceFeature.Side && candidate.Family == ConeSurfaceFamily.RotationCircle)
        {
            product.Set(Signed576.ExtendValue(WideArithmetic.SubtractSigned320(geometry.Finite.ShapeFrame.Cap, candidate.RootOrdinal == 1 ? geometry.B.Y : geometry.A.Y)));
            ContactQuadratic square = At(scratch, scratchSigns, 0);
            Multiply(product, product, root, square);
            Signed192 h = Signed192.Raw(geometry.Input.Height), r = Signed192.Raw(geometry.Input.Radius);
            Scale(square, Signed576.ExtendValue(WideArithmetic.MultiplySigned192(r, r)), numerator);
            denominator.Set(Signed576.ExtendValue(WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(h, h), WideArithmetic.MultiplySigned192(r, r))));
        }
        else if (candidate.Feature == ConeSurfaceFeature.Side)
        {
            bool normal = BuildSideNormal(geometry, candidate.RootOrdinal, candidate.Chart, scratch, scratchSigns, root);
            System.Diagnostics.Debug.Assert(normal);
            BuildSideMetric(geometry, candidate.RootOrdinal, scratch, scratchSigns, root);
            Multiply(At(scratch, scratchSigns, 8), At(scratch, scratchSigns, 8), root, numerator);
            At(scratch, scratchSigns, 7).CopyTo(denominator);
        }
        else
        {
            if (candidate.RootOrdinal <= -3)
            {
                bool admitted = BuildMeridionalRim(geometry, candidate.RootOrdinal, candidate.Chart == -2 ? 1 : -1, scratch, scratchSigns, root);
                System.Diagnostics.Debug.Assert(admitted);
                Multiply(At(scratch, scratchSigns, 7), At(scratch, scratchSigns, 7), root, denominator);
            }
            else
            {
                bool admitted = TryGetRimPoint(geometry, candidate.RootOrdinal, out WideAxis3 radialAxis, out Signed576 coefficient, out Signed320 axial, out _, out _);
                System.Diagnostics.Debug.Assert(admitted);
                if (candidate.Family == ConeSurfaceFamily.RotationCircle)
                {
                    numerator.Set(WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius), WideArithmetic.MultiplySigned320(axial, axial)));
                }
                else
                {
                    Signed576 radialSquare = radialAxis.SquaredLength;
                    Import(radialSquare, root);
                    BuildRimPointNormal(geometry, radialAxis, coefficient, axial, radialSquare, candidate.Chart == -2 ? 1 : -1, scratch, scratchSigns);
                    WideArithmetic.MultiplyMagnitudes(root, root, denominator.Rational);
                    denominator.Signs[0] = 1;
                }
            }
            if (candidate.Family != ConeSurfaceFamily.RotationCircle)
            {
                for (int axis = 0; axis < 3; axis++)
                {
                    Multiply(At(scratch, scratchSigns, axis), At(scratch, scratchSigns, axis), root, product); numerator.Add(product);
                }
            }
        }
        // These are the same squared metrics used by each feature's depth
        // materializer. The largest meridional coefficients are <2000 bits;
        // denominator scaling adds <268 bits, within the existing 40 limbs.
        Span<ulong> scale = stackalloc ulong[Words], scaleSquared = stackalloc ulong[Words];
        Import(Signed320.ExtendValue(geometry.RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(scale, scale, scaleSquared);
        Scale(denominator, scaleSquared, 1, product);
        System.Diagnostics.Debug.Assert(product.Signs[1] == 0 && product.Signs[0] > 0);
        product.Rational.CopyTo(Slot(destination, 10));
        return numerator.Sign(root) == 0 ? 0 : 1;
    }

    internal static bool AccumulateSegmentConeSurfaceCandidates(FixedSegment segment,
        Vector3d origin, FixedQuaternion rotation, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        scoped ref ConeSurfaceSelection selection)
    {
        var input = new SegmentConeSurfaceInput(segment, origin, rotation, coneCenter, coneRotation, height, radius);
        var geometry = new SegmentConeSurfaceGeometry(input);
        if (!geometry.Finite.IntersectsTransformedSegment(geometry.Finite.Transform(segment.Start),
                geometry.Finite.Transform(segment.End)))
            return false;
        int first = selection.Count;
        AccumulateApex(geometry, ref selection);
        AccumulateBase(geometry, ref selection);
        AccumulateSide(geometry, ref selection);
        AccumulateAxis(geometry, ref selection);
        AccumulateRimPoints(geometry, ref selection);
        AccumulateMeridionalRim(geometry, ref selection);
        AccumulateRimRoots(geometry, ref selection);
        return selection.Count != first;
    }
}

internal enum ConeSurfaceFeature { Apex, Base, Side, Rim }
internal enum ConeSurfaceFamily { None, SegmentInterval, RotationCircle, NormalCone }
internal enum ConeSurfacePointLocation { SegmentIntervalUnspecified, Start, Interior, End }

/// <summary>One exact chart branch or a retained degenerate family.</summary>
internal readonly struct SegmentConeSurfaceCandidate
{
    internal int GetMaximumFamilyEventCount(int halfspaceCount) =>
        SegmentConeSurfaceCandidates.GetMaximumFamilyEventCount(this, halfspaceCount);

    internal bool AccumulateFamilyEvents(scoped ReadOnlySpan<WideAxis3> authoredHalfspaces,
        ConeSurfacePointLocation location, scoped ref ConeSurfaceFamilySelection selection) =>
        SegmentConeSurfaceCandidates.AccumulateFamilyEvents(this, authoredHalfspaces, location, ref selection);

    internal ConeSurfaceFeature Feature { get; }
    internal ConeSurfaceFamily Family { get; }
    internal ConeSurfacePointLocation PointLocation { get; }
    /// <summary>Whether a unique retained segment point is strictly interior; interval families have no unique point.</summary>
    internal bool IsSegmentInterior => PointLocation == ConeSurfacePointLocation.Interior;
    internal readonly SegmentConeSurfaceInput Input;
    internal readonly int Chart, RootOrdinal;

    internal SegmentConeSurfaceCandidate(SegmentConeSurfaceInput input, int chart, int rootOrdinal, ConeSurfacePointLocation location)
    {
        Input = input; Chart = chart; RootOrdinal = rootOrdinal;
        Feature = ConeSurfaceFeature.Rim; Family = ConeSurfaceFamily.None; PointLocation = location;
    }

    internal SegmentConeSurfaceCandidate(SegmentConeSurfaceInput input, ConeSurfaceFeature feature, ConeSurfaceFamily family)
    {
        Input = input; Feature = feature; Family = family;
        PointLocation = ConeSurfacePointLocation.SegmentIntervalUnspecified; Chart = RootOrdinal = 0;
    }

    internal SegmentConeSurfaceCandidate(SegmentConeSurfaceInput input, ConeSurfaceFeature feature,
        ConeSurfaceFamily family, int chart, int ordinal, ConeSurfacePointLocation location)
    {
        Input = input; Feature = feature; Family = family; Chart = chart;
        RootOrdinal = ordinal; PointLocation = location;
    }

    internal FixedContactAnchors GetContact() => Family == ConeSurfaceFamily.None
        ? Feature == ConeSurfaceFeature.Apex ? SegmentConeSurfaceCandidates.GetApexContact(this)
            : Feature == ConeSurfaceFeature.Base ? SegmentConeSurfaceCandidates.GetBaseContact(this)
            : Feature == ConeSurfaceFeature.Side ? SegmentConeSurfaceCandidates.GetSideContact(this)
            : Chart >= 0 ? SegmentConeSurfaceCandidates.GetRimRootContact(this)
                : SegmentConeSurfaceCandidates.GetRimPointContact(this)
        : throw new InvalidOperationException("A family requires an admitted exact parameter before materialization.");

    internal bool TryGetNormalDotSign(WideAxis3 authoredCovector, out int sign) =>
        SegmentConeSurfaceCandidates.TryGetNormalDotSign(this, authoredCovector, out sign);

}

/// <summary>Caller-owned retained finite-support branches and degenerate families.</summary>
internal ref struct ConeSurfaceSelection
{
    private readonly Span<SegmentConeSurfaceCandidate> candidates;
    internal int Count { get; private set; }
    internal SegmentConeSurfaceCandidate this[int index] => candidates[..Count][index];

    internal ConeSurfaceSelection(Span<SegmentConeSurfaceCandidate> candidates)
    {
        this.candidates = candidates; Count = 0;
    }

    internal void Add(SegmentConeSurfaceCandidate candidate)
    {
        if (Count == candidates.Length)
            throw new InvalidOperationException("The caller-owned surface candidate storage is full.");
        candidates[Count++] = candidate;
    }
}

/// <summary>Neutral source provenance sufficient to reconstruct one exact chart.</summary>
internal readonly struct SegmentConeSurfaceInput
{
    internal readonly FixedSegment Segment;
    internal readonly Vector3d Origin, Center;
    internal readonly FixedQuaternion Rotation, ConeRotation;
    internal readonly Fixed64 Height, Radius;

    internal SegmentConeSurfaceInput(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        Segment = segment; Origin = origin; Rotation = rotation; Center = center;
        ConeRotation = coneRotation; Height = height; Radius = radius;
    }
}

/// <summary>One exact centered frame shared by the segment's support charts.</summary>
internal readonly struct SegmentConeSurfaceGeometry
{
    internal readonly SegmentConeSurfaceInput Input;
    internal readonly ConeFiniteSectionFrame Finite;
    internal readonly WideAxis3 A, B, Edge, BaseOffset;
    internal readonly Signed320 Radius;
    internal readonly Signed192 RawScale;
    internal readonly int ValueShift;

    internal SegmentConeSurfaceGeometry(SegmentConeSurfaceInput input)
    {
        Input = input;
        Finite = new ConeFiniteSectionFrame(input.Origin, input.Rotation, input.Center,
            input.ConeRotation, input.Height, input.Radius);
        A = Finite.ShapeFrame.Transform(input.Segment.Start);
        B = Finite.ShapeFrame.Transform(input.Segment.End);
        Edge = new WideAxis3(WideArithmetic.SubtractSigned320(B.X, A.X),
            WideArithmetic.SubtractSigned320(B.Y, A.Y), WideArithmetic.SubtractSigned320(B.Z, A.Z));
        BaseOffset = new WideAxis3(A.X, WideArithmetic.AddSigned320(A.Y, Finite.ShapeFrame.Cap), A.Z);
        Radius = WideArithmetic.MultiplySigned192(Finite.ShapeFrame.Radius, Finite.ShapeFrame.Denominator);
        RawScale = WideArithmetic.AddSigned192(Finite.ShapeFrame.Denominator, Finite.ShapeFrame.Denominator);
        Span<Signed320> coordinates = stackalloc Signed320[8]
        {
            A.X, A.Y, A.Z, B.X, B.Y, B.Z, Finite.ShapeFrame.Cap, Radius
        };
        ValueShift = TriangleCircularGeometry.GetValueShift(coordinates, 4);
    }
}
