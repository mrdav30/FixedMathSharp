//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Exact matched finite segment/slab surface witnesses admitted by an authored normal fan.</summary>
/// <content>Finite caps, planar sides and rims share exact candidate admission before minimum-depth selection.</content>
internal static partial class SegmentSlabSurfaceContact
{
    // Authored rigid coordinates are <198 bits. Rational edge/edge minors
    // are <800 bits; their point products and quadratic radial branches are
    // <1100. Squared separations and scaled denominators remain <2400 bits,
    // within the existing forty-word contact candidate owner. Exact source
    // frames retain <306-bit coordinates and <385-bit local fan directions;
    // clip ratios can occupy 832 bits. Each primitive input fits three times
    // the original bit bound, so the unchanged fixed-degree formulas use
    // three times the borrowed fields. Ordinary rigid inputs keep 40 words.
    private const int Fields = 12;

    internal static bool TryGetContact(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
        Vector3d slabCenter, FixedQuaternion slabRotation, Fixed64 coreLength, Fixed64 radius,
        Fixed64 halfThickness, ReadOnlySpan<Vector2d> prismOffsets,
        ReadOnlySpan<WideAxis3> authoredHalfspaces, bool interiorOnly, out FixedContactAnchors contact, bool fullHeight = false)
        => TryGetContactCore(segment, origin, rotation, slabCenter, slabRotation, coreLength, radius,
            halfThickness, prismOffsets, authoredHalfspaces, interiorOnly, true, default, fullHeight, out contact);

    /// <summary>Restricts witnesses to the original segment's exact retained interval before selection.</summary>
    internal static bool TryGetContact(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
        Vector3d slabCenter, FixedQuaternion slabRotation, Fixed64 coreLength, Fixed64 radius,
        Fixed64 halfThickness, ReadOnlySpan<Vector2d> prismOffsets,
        ReadOnlySpan<WideAxis3> authoredHalfspaces, bool interiorOnly,
        in RationalSegmentInterval clip, out FixedContactAnchors contact, bool fullHeight = false)
    {
        if (!clip.IsClipped)
            return TryGetContact(segment, origin, rotation, slabCenter, slabRotation, coreLength, radius,
                halfThickness, prismOffsets, authoredHalfspaces, interiorOnly, out contact, fullHeight);
        Span<RationalSegmentInterval> interval = stackalloc RationalSegmentInterval[1] { clip };
        return TryGetContactCore(segment, origin, rotation, slabCenter, slabRotation, coreLength, radius,
            halfThickness, prismOffsets, authoredHalfspaces, interiorOnly, true, interval, fullHeight, out contact);
    }

    /// <summary>Returns a finite feature exit after the caller independently admits its owner's overlap.</summary>
    /// <remarks>The halfspaces must certify both the finite feature's normal fan and global support of its owner.
    /// The source witness may lie outside the original slab; its matched slab witness and outward cone remain exact.</remarks>
    internal static bool TryGetFeaturePenetration(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
        Vector3d slabCenter, FixedQuaternion slabRotation, Fixed64 coreLength, Fixed64 radius,
        Fixed64 halfThickness, ReadOnlySpan<Vector2d> prismOffsets,
        ReadOnlySpan<WideAxis3> authoredHalfspaces, bool interiorOnly, out FixedContactAnchors contact, bool fullHeight = false)
        => TryGetContactCore(segment, origin, rotation, slabCenter, slabRotation, coreLength, radius,
            halfThickness, prismOffsets, authoredHalfspaces, interiorOnly, false, default, fullHeight, out contact);

    private static bool TryGetContactCore(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
        Vector3d slabCenter, FixedQuaternion slabRotation, Fixed64 coreLength, Fixed64 radius,
        Fixed64 halfThickness, ReadOnlySpan<Vector2d> prismOffsets,
        ReadOnlySpan<WideAxis3> authoredHalfspaces, bool interiorOnly, bool requireSourceInside, ReadOnlySpan<RationalSegmentInterval> clip, bool fullHeight, out FixedContactAnchors contact)
    {
        contact = default;
        var frame = new FiniteSlabSectionFrame(origin, rotation, slabCenter, slabRotation,
            coreLength, radius, halfThickness, fullHeight);
        if (requireSourceInside && !frame.IntersectsSegment(segment, prismOffsets)) return false;
        Span<WideDirection3d> fan = stackalloc WideDirection3d[authoredHalfspaces.Length + (interiorOnly ? 2 : 0)];
        for (int index = 0; index < authoredHalfspaces.Length; index++)
            fan[index] = new WideDirection3d(WideRigidProjection.TransformLocalAxis(frame.RadialFrame.ShapeFrame.Basis,
                Signed192.NarrowProven(authoredHalfspaces[index].X),
                Signed192.NarrowProven(authoredHalfspaces[index].Y),
                Signed192.NarrowProven(authoredHalfspaces[index].Z)));
        WideDirection3d first = new(frame.Transform(segment.Start)), second = new(frame.Transform(segment.End));
        WideDirection3d edge = TriangleCircularGeometry.Subtract(second, first);
        if (interiorOnly)
        {
            if (edge.IsZero) return false;
            fan[^2] = edge; fan[^1] = -edge;
        }
        return TryGetContactCore(segment, origin, rotation, slabCenter, slabRotation, coreLength, radius,
            halfThickness, prismOffsets, fan, interiorOnly, requireSourceInside, clip, fullHeight, frame, first, edge, out contact);
    }

    internal static bool TryGetContact(FixedSegment segment, in ScaledRigidFrame3d sourceFrame,
        Vector3d slabCenter, FixedQuaternion slabRotation, Fixed64 coreLength, Fixed64 radius,
        Fixed64 halfThickness, ReadOnlySpan<Vector2d> prismOffsets,
        ReadOnlySpan<WideAxis3> worldHalfspaces, bool interiorOnly, out FixedContactAnchors contact, bool fullHeight = false) =>
        TryGetContactCore(segment, sourceFrame, slabCenter, slabRotation, coreLength, radius, halfThickness,
            prismOffsets, worldHalfspaces, interiorOnly, true, default, fullHeight, out contact);

    internal static bool TryGetContact(FixedSegment segment, in ScaledRigidFrame3d sourceFrame,
        Vector3d slabCenter, FixedQuaternion slabRotation, Fixed64 coreLength, Fixed64 radius,
        Fixed64 halfThickness, ReadOnlySpan<Vector2d> prismOffsets, ReadOnlySpan<WideAxis3> worldHalfspaces,
        bool interiorOnly, in RationalSegmentInterval clip, out FixedContactAnchors contact, bool fullHeight = false)
    {
        if (!clip.IsClipped)
            return TryGetContact(segment, sourceFrame, slabCenter, slabRotation, coreLength, radius,
                halfThickness, prismOffsets, worldHalfspaces, interiorOnly, out contact, fullHeight);
        Span<RationalSegmentInterval> intervals = stackalloc RationalSegmentInterval[1] { clip };
        return TryGetContactCore(segment, sourceFrame, slabCenter, slabRotation, coreLength, radius, halfThickness,
            prismOffsets, worldHalfspaces, interiorOnly, true, intervals, fullHeight, out contact);
    }

    internal static bool TryGetFeaturePenetration(FixedSegment segment, in ScaledRigidFrame3d sourceFrame,
        Vector3d slabCenter, FixedQuaternion slabRotation, Fixed64 coreLength, Fixed64 radius,
        Fixed64 halfThickness, ReadOnlySpan<Vector2d> prismOffsets,
        ReadOnlySpan<WideAxis3> worldHalfspaces, bool interiorOnly, out FixedContactAnchors contact, bool fullHeight = false) =>
        TryGetContactCore(segment, sourceFrame, slabCenter, slabRotation, coreLength, radius, halfThickness,
            prismOffsets, worldHalfspaces, interiorOnly, false, default, fullHeight, out contact);

    private static bool TryGetContactCore(FixedSegment segment, in ScaledRigidFrame3d sourceFrame,
        Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness,
        ReadOnlySpan<Vector2d> offsets, ReadOnlySpan<WideAxis3> worldHalfspaces, bool interiorOnly,
        bool requireSourceInside, ReadOnlySpan<RationalSegmentInterval> clip, bool fullHeight, out FixedContactAnchors contact)
    {
        contact = default;
        var frame = new FiniteSlabSectionFrame(sourceFrame, center, rotation, length, radius, halfThickness, fullHeight);
        if (requireSourceInside && !frame.IntersectsSegment(segment, offsets)) return false;
        Span<WideDirection3d> fan = stackalloc WideDirection3d[worldHalfspaces.Length + (interiorOnly ? 2 : 0)];
        var inverse = new WideRationalBasis3d(rotation.Conjugate());
        for (int index = 0; index < worldHalfspaces.Length; index++)
            fan[index] = new WideDirection3d(worldHalfspaces[index]).Transform(inverse);
        WideDirection3d first = new(frame.Transform(segment.Start)), edge = TriangleCircularGeometry.Subtract(new WideDirection3d(frame.Transform(segment.End)), first);
        if (interiorOnly)
        {
            if (edge.IsZero) return false;
            fan[^2] = edge; fan[^1] = -edge;
        }
        return TryGetContactCore(segment, sourceFrame.IsRigid ? sourceFrame.RigidOrigin : center,
            sourceFrame.IsRigid ? sourceFrame.Rotation : rotation, center, rotation, length, radius,
            halfThickness, offsets, fan, interiorOnly, requireSourceInside, clip, fullHeight, frame, first, edge, out contact);
    }

    private static bool TryGetContactCore(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
        Vector3d slabCenter, FixedQuaternion slabRotation, Fixed64 coreLength, Fixed64 radius,
        Fixed64 halfThickness, ReadOnlySpan<Vector2d> prismOffsets, ReadOnlySpan<WideDirection3d> fan,
        bool interiorOnly, bool requireSourceInside, ReadOnlySpan<RationalSegmentInterval> clip, bool fullHeight,
        in FiniteSlabSectionFrame frame, WideDirection3d first, WideDirection3d edge, out FixedContactAnchors contact)
    {
        contact = default;
        var input = new Input(segment, origin, rotation, slabCenter, slabRotation,
            coreLength, radius, halfThickness, frame, first, edge, prismOffsets, fan, interiorOnly, requireSourceInside, clip, fullHeight);
        int Words = input.Words;
        int StorageWords = Fields * 2 * Words;
        Span<ulong> best = stackalloc ulong[StorageWords], bestRoot = stackalloc ulong[Words];
        Span<int> bestSigns = stackalloc int[2 * Fields];
        Span<ulong> bestMetric = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> bestMetricSigns = stackalloc int[ConvexContactCandidate.Slots];
        var selection = new Selection(new Candidate(best, bestSigns, bestRoot), bestMetric, bestMetricSigns);
        if (prismOffsets.IsEmpty) AccumulateCurved(input, ref selection);
        else AccumulatePrism(input, ref selection);
        if (selection.HasValue) contact = Materialize(input, selection);
        if (prismOffsets.IsEmpty && TryImproveObliqueRims(input, selection, out FixedContactAnchors oblique))
        { contact = oblique; return true; }
        return selection.HasValue;
    }

    private readonly ref struct Input
    {
        internal readonly FixedSegment Segment;
        internal readonly Vector3d Origin, Center;
        internal readonly FixedQuaternion Rotation, SlabRotation;
        internal readonly Fixed64 Length, Radius, HalfThickness, FullHeight;
        internal readonly FiniteSlabSectionFrame Frame;
        internal readonly WideDirection3d First, Edge;
        internal readonly Signed320 Scale;
        internal readonly Signed576 Cap, Core, Radial;
        internal int Words => Frame.RadialFrame.ShapeFrame.HasExactScale || !Clip.IsEmpty && Clip[0].HasWideBounds ? 120 : CylinderContactAlgebra.Words;
        internal int ValueWords => Words == CylinderContactAlgebra.Words ? CircularRimContactAlgebra.ValueWords : 3 * CircularRimContactAlgebra.ValueWords;
        internal readonly ReadOnlySpan<Vector2d> Offsets;
        internal readonly ReadOnlySpan<WideDirection3d> Fan;
        internal readonly ReadOnlySpan<RationalSegmentInterval> Clip;
        internal readonly bool InteriorOnly, RequireSourceInside;

        internal Input(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
            Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness,
            FiniteSlabSectionFrame frame, WideDirection3d first, WideDirection3d edge,
            ReadOnlySpan<Vector2d> offsets, ReadOnlySpan<WideDirection3d> fan, bool interiorOnly, bool requireSourceInside,
            ReadOnlySpan<RationalSegmentInterval> clip, bool fullHeight)
        {
            Segment = segment; Origin = origin; Rotation = rotation; Center = center;
            SlabRotation = slabRotation; Length = length; Radius = radius; Frame = frame;
            HalfThickness = halfThickness; FullHeight = fullHeight ? halfThickness : Fixed64.Zero;
            First = first; Edge = edge; Offsets = offsets; Fan = fan; InteriorOnly = interiorOnly;
            RequireSourceInside = requireSourceInside; Clip = clip;
            Scale = WideArithmetic.AddSigned320(frame.RadialFrame.ShapeFrame.CoordinateDenominator,
                frame.RadialFrame.ShapeFrame.CoordinateDenominator);
            Cap = Signed576.ExtendValue(frame.Cap); Core = Signed576.ExtendValue(frame.RadialFrame.ShapeFrame.Cap);
            Radial = WideArithmetic.MultiplySigned320(frame.RadialFrame.ShapeFrame.CoordinateDenominator, frame.RadialFrame.ShapeFrame.Radius);
        }
    }

    private readonly ref struct Candidate
    {
        internal readonly Span<ulong> Values, Root;
        internal readonly Span<int> Signs;
        internal Candidate(Span<ulong> values, Span<int> signs, Span<ulong> root)
        { Values = values; Signs = signs; Root = root; }
        internal int Words => Values.Length / (Fields * 2);
        internal ContactQuadratic Field(int index) => ContactQuadratic.At(Values, Signs, index, Words);
        internal ContactQuadratic Point(int component) => Field(component);
        internal ContactQuadratic Boundary(int component) => Field(component + 3);
        internal ContactQuadratic Parameter => Field(6);
        internal ContactQuadratic ParameterDenominator => Field(7);
        internal ContactQuadratic Denominator => Field(8);
        internal ContactQuadratic Normal(int component) => Field(component + 9);
        internal void Clear()
        {
            Values.Clear(); Signs.Clear(); Root.Clear();
            ParameterDenominator.Set(One); Denominator.Set(One);
        }
        internal void CopyTo(scoped Candidate target)
        { Values.CopyTo(target.Values); Signs.CopyTo(target.Signs); Root.CopyTo(target.Root); }
    }

    private ref struct Selection
    {
        internal Candidate Best;
        internal Span<ulong> Metric;
        internal Span<int> MetricSigns;
        internal bool HasValue;
        internal Selection(Candidate best, Span<ulong> metric, Span<int> metricSigns)
        { Best = best; Metric = metric; MetricSigns = metricSigns; HasValue = false; }
    }

    // Homogeneous coefficients use integer unity, rather than Q32.32 raw unity.
    private static Signed832 One => Signed832.ExtendValue(Signed576.ExtendValue(Signed192.Signed(1)));
    private static Signed832 Extend(Signed576 value) => Signed832.ExtendValue(value);
    private static Signed832 Extend(Signed320 value) => Signed832.ExtendValue(value);

    private static void SetPoint(scoped Input input, scoped Candidate candidate)
    {
        int Words = input.Words;
        Span<ulong> values = stackalloc ulong[2 * Words]; Span<int> signs = stackalloc int[2];
        var term = new ContactQuadratic(values, signs);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic.Scale(candidate.ParameterDenominator, Extend(Component(input.First, axis)), candidate.Point(axis));
            ContactQuadratic.Scale(candidate.Parameter, Extend(Component(input.Edge, axis)), term);
            candidate.Point(axis).Add(term);
            candidate.Point(axis).CopyTo(candidate.Boundary(axis));
        }
        candidate.ParameterDenominator.CopyTo(candidate.Denominator);
    }

    private static void ScaleCoordinates(scoped Candidate candidate, scoped ContactQuadratic multiplier)
    {
        for (int field = 0; field < 6; field++)
            ContactQuadratic.Multiply(candidate.Field(field), multiplier, candidate.Root, candidate.Field(field));
        ContactQuadratic.Multiply(candidate.Denominator, multiplier, candidate.Root, candidate.Denominator);
    }

    private static bool Keep(scoped Input input, scoped Candidate candidate, scoped ref Selection selection)
    {
        int Words = input.Words;
        Span<ulong> values = stackalloc ulong[10 * Words]; Span<int> signs = stackalloc int[10];
        ContactQuadratic remainder = ContactQuadratic.At(values, signs, 0, Words), sum = ContactQuadratic.At(values, signs, 1, Words),
            term = ContactQuadratic.At(values, signs, 2, Words), delta = ContactQuadratic.At(values, signs, 3, Words),
            divisor = ContactQuadratic.At(values, signs, 4, Words);
        int lower, upper;
        if (input.Clip.IsEmpty)
        {
            candidate.ParameterDenominator.CopyTo(remainder); remainder.Add(candidate.Parameter, -1);
            lower = candidate.Parameter.Sign(candidate.Root); upper = remainder.Sign(candidate.Root);
        }
        else
        {
            ref readonly RationalSegmentInterval clip = ref input.Clip[0];
            // Unit bounds add at most 319 coefficient bits; the unchanged
            // quadratic field remains within the forty-word candidate bank.
            ContactQuadratic.Scale(candidate.Parameter, clip.WideLowerDenominator, remainder);
            ContactQuadratic.Scale(candidate.ParameterDenominator, clip.WideLowerNumerator, term); remainder.Add(term, -1);
            lower = remainder.Sign(candidate.Root);
            ContactQuadratic.Scale(candidate.ParameterDenominator, clip.WideUpperNumerator, remainder);
            ContactQuadratic.Scale(candidate.Parameter, clip.WideUpperDenominator, term); remainder.Add(term, -1);
            upper = remainder.Sign(candidate.Root);
        }
        if (lower < 0 || upper < 0 || input.InteriorOnly && (lower == 0 || upper == 0)) return false;
        if (input.RequireSourceInside && !Contains(input, candidate, boundary: false)
            || !Contains(input, candidate, boundary: true)) return false;
        sum.Clear();
        bool nonzero = false;
        for (int axis = 0; axis < 3; axis++)
        {
            candidate.Boundary(axis).CopyTo(delta); delta.Add(candidate.Point(axis), -1);
            ContactQuadratic.Multiply(delta, delta, candidate.Root, term); sum.Add(term);
            nonzero |= delta.Sign(candidate.Root) != 0;
        }
        if (nonzero)
            for (int axis = 0; axis < 3; axis++)
            { candidate.Boundary(axis).CopyTo(candidate.Normal(axis)); candidate.Normal(axis).Add(candidate.Point(axis), -1); }
        else if (candidate.Normal(0).Sign(candidate.Root) == 0 && candidate.Normal(1).Sign(candidate.Root) == 0
            && candidate.Normal(2).Sign(candidate.Root) == 0) return false;
        foreach (WideDirection3d constraint in input.Fan)
        {
            Dot(candidate, constraint, remainder, term);
            if (remainder.Sign(candidate.Root) > 0) return false;
        }
        Span<ulong> metric = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> metricSigns = stackalloc int[ConvexContactCandidate.Slots];
        metric.Clear(); metricSigns.Clear();
        sum.Rational.CopyTo(Slot(metric, 7, Words)); sum.Radical.CopyTo(Slot(metric, 8, Words));
        metricSigns[7] = sum.Signs[0]; metricSigns[8] = sum.Signs[1];
        candidate.Root.CopyTo(Slot(metric, 9, Words));
        ContactQuadratic.Multiply(candidate.Denominator, candidate.Denominator, candidate.Root, divisor);
        Signed832 scaleSquare = WideArithmetic.MultiplySigned576ToSigned832(Signed576.ExtendValue(input.Scale), Signed576.ExtendValue(input.Scale));
        ContactQuadratic.Scale(divisor, scaleSquare, term);
        System.Diagnostics.Debug.Assert(term.Signs[1] == 0 && term.Signs[0] > 0);
        term.Rational.CopyTo(Slot(metric, 10, Words));
        int gapSign = nonzero ? 1 : 0;
        var current = new ConvexContactCandidate(metric, metricSigns, gapSign, Words);
        if (selection.HasValue)
        {
            int comparison = WideConvexPrismRelations.CompareConvexContactCandidates(current,
                new ConvexContactCandidate(selection.Metric, selection.MetricSigns,
                    selection.MetricSigns[10], Words));
            if (comparison > 0 || comparison == 0 && CompareWitnesses(candidate, selection.Best) >= 0) return false;
        }
        metric.CopyTo(selection.Metric); metricSigns.CopyTo(selection.MetricSigns);
        selection.MetricSigns[10] = gapSign;
        candidate.CopyTo(selection.Best); selection.HasValue = true;
        return true;
    }

    private static void Dot(scoped Candidate candidate, WideDirection3d axis, scoped ContactQuadratic sum, scoped ContactQuadratic term)
    {
        sum.Clear();
        for (int component = 0; component < 3; component++)
        { ContactQuadratic.Scale(candidate.Normal(component), Extend(Component(axis, component)), term); sum.Add(term); }
    }

    private static int CompareWitnesses(scoped Candidate first, scoped Candidate second)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            int comparison = ContactQuadratic.CompareRatios(first.Boundary(axis), first.Denominator, first.Root,
                second.Boundary(axis), second.Denominator, second.Root);
            if (comparison != 0) return comparison;
        }
        return ContactQuadratic.CompareRatios(first.Parameter, first.ParameterDenominator, first.Root,
            second.Parameter, second.ParameterDenominator, second.Root);
    }

    private static bool Contains(scoped Input input, scoped Candidate candidate, bool boundary)
    {
        int Words = input.Words;
        Span<ulong> values = stackalloc ulong[12 * Words]; Span<int> signs = stackalloc int[12];
        ContactQuadratic bound = ContactQuadratic.At(values, signs, 0, Words), difference = ContactQuadratic.At(values, signs, 1, Words),
            x = ContactQuadratic.At(values, signs, 2, Words), z = ContactQuadratic.At(values, signs, 3, Words),
            square = ContactQuadratic.At(values, signs, 4, Words), term = ContactQuadratic.At(values, signs, 5, Words);
        ContactQuadratic y = candidate.Field(boundary ? 4 : 1);
        ContactQuadratic.Scale(candidate.Denominator, Extend(input.Cap), bound);
        bound.CopyTo(difference); difference.Add(y, -1);
        if (difference.Sign(candidate.Root) < 0) return false;
        bound.CopyTo(difference); difference.Add(y);
        if (difference.Sign(candidate.Root) < 0) return false;
        candidate.Field(boundary ? 3 : 0).CopyTo(x);
        candidate.Field(boundary ? 5 : 2).CopyTo(z);
        if (!input.Offsets.IsEmpty)
        {
            int winding = FiniteSlabSectionFrame.GetPrismWinding(input.Offsets);
            for (int edge = 0; edge < input.Offsets.Length; edge++)
            {
                Vector2d a = input.Offsets[edge], b = input.Offsets[(edge + 1) % input.Offsets.Length];
                Signed192 dx = WideArithmetic.SubtractSigned192(Signed192.Raw(b.X), Signed192.Raw(a.X));
                Signed192 dz = WideArithmetic.SubtractSigned192(Signed192.Raw(b.Y), Signed192.Raw(a.Y));
                ContactQuadratic.Scale(candidate.Denominator, Extend(WideArithmetic.MultiplySigned320(input.Scale, Signed192.Raw(a.X))), bound);
                x.CopyTo(difference); difference.Add(bound, -1);
                ContactQuadratic.Scale(difference, Signed832.ExtendValue(Signed576.ExtendValue(dz)), square);
                ContactQuadratic.Scale(candidate.Denominator, Extend(WideArithmetic.MultiplySigned320(input.Scale, Signed192.Raw(a.Y))), bound);
                z.CopyTo(difference); difference.Add(bound, -1);
                ContactQuadratic.Scale(difference, Signed832.ExtendValue(Signed576.ExtendValue(dx)), term); term.Add(square, -1);
                if (term.Sign(candidate.Root) * winding < 0) return false;
            }
            return true;
        }
        ContactQuadratic.Scale(candidate.Denominator, Extend(input.Core), bound);
        z.CopyTo(difference); difference.Add(bound, -1);
        if (difference.Sign(candidate.Root) >= 0) difference.CopyTo(z);
        else
        {
            z.Add(bound);
            if (z.Sign(candidate.Root) > 0) z.Clear();
        }
        ContactQuadratic.Multiply(x, x, candidate.Root, square);
        ContactQuadratic.Multiply(z, z, candidate.Root, term); square.Add(term);
        ContactQuadratic.Scale(candidate.Denominator, Extend(input.Radial), bound);
        ContactQuadratic.Multiply(bound, bound, candidate.Root, term); term.Add(square, -1);
        return term.Sign(candidate.Root) >= 0;
    }

    private static FixedContactAnchors Materialize(scoped Input input, scoped Selection selection)
    {
        int Words = input.Words;
        Candidate best = selection.Best;
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic coordinate = ContactQuadratic.At(values, signs, 0, Words), term = ContactQuadratic.At(values, signs, 1, Words),
            denominator = ContactQuadratic.At(values, signs, 2, Words), residual = ContactQuadratic.At(values, signs, 3, Words);
        Vector3d point = default;
        ContactQuadratic.Scale(best.Denominator, Extend(input.Scale), denominator);
        for (int axis = 0; axis < 3; axis++)
        {
            if (input.Frame.RadialFrame.ShapeFrame.HasExactScale)
            {
                point[axis] = ContactQuadratic.RoundRatio(best.Point(axis), denominator, best.Root);
                continue;
            }
            ContactQuadratic.Scale(best.ParameterDenominator, Extend(Signed576.ExtendValue(Signed192.Raw(input.Segment.Start[axis]))), coordinate);
            ContactQuadratic.Scale(best.Parameter, Extend(Signed576.ExtendValue(WideArithmetic.SubtractSigned192(
                Signed192.Raw(input.Segment.End[axis]), Signed192.Raw(input.Segment.Start[axis])))), term);
            coordinate.Add(term);
            point[axis] = ContactQuadratic.RoundRatio(coordinate, best.ParameterDenominator, best.Root,
                Math.Min(input.Segment.Start[axis].m_rawValue, input.Segment.End[axis].m_rawValue),
                Math.Max(input.Segment.Start[axis].m_rawValue, input.Segment.End[axis].m_rawValue));
        }
        ContactQuadratic.Scale(best.Denominator, Extend(input.Scale), denominator);
        Vector3d core = default;
        FixedPointAnchorTerm3d exactTerm = default;
        if (input.Offsets.IsEmpty && input.Length != Fixed64.Zero)
        {
            ContactQuadratic.Scale(best.Denominator, Extend(input.Core), term);
            best.Boundary(2).CopyTo(coordinate); coordinate.Add(term, -1);
            int region = coordinate.Sign(best.Root) > 0 ? 1 : 0;
            best.Boundary(2).CopyTo(coordinate); coordinate.Add(term);
            if (coordinate.Sign(best.Root) < 0) region = -1;
            if (region != 0)
                core = TriangleCapsuleSlabEndpointWitnesses.GetCore(Vector2d.Forward, input.Length, -region, out exactTerm);
        }
        Vector3d local = default;
        for (int axis = 0; axis < 3; axis++)
        {
            best.Boundary(axis).CopyTo(coordinate);
            ContactQuadratic.Scale(denominator, Extend(Signed576.ExtendValue(Signed192.Raw(core[axis]))), term);
            coordinate.Add(term, -1);
            // Retain an integral radial term with the exact odd half-core.
            // Otherwise round the complete sum once and absorb that residual.
            if (axis == 2 && exactTerm.Z != 0)
            {
                ContactQuadratic.Scale(best.Denominator, Extend(input.Core), term);
                term.MultiplySign(core.Z.m_rawValue < 0 ? -1 : 1);
                best.Boundary(axis).CopyTo(residual); residual.Add(term, -1);
                Fixed64 rounded = ContactQuadratic.RoundRatio(residual, denominator, best.Root);
                ContactQuadratic.Scale(denominator, Extend(Signed576.ExtendValue(Signed192.Raw(rounded))), term);
                residual.Add(term, -1);
                if (residual.Sign(best.Root) == 0)
                { local[axis] = rounded; continue; }
                exactTerm = default;
            }
            local[axis] = ContactQuadratic.RoundRatio(coordinate, denominator, best.Root, parityOffset: core[axis].m_rawValue);
        }
        if ((input.FullHeight.m_rawValue & 1L) != 0)
        {
            // An exact odd full-height cap lies halfway between local raw
            // coordinates. Retain that half-raw after radial/core rounding.
            ContactQuadratic.Scale(best.Denominator, Extend(input.Cap), term);
            best.Boundary(1).CopyTo(residual); residual.Add(term, -1);
            int cap = residual.Sign(best.Root) == 0 ? 1 : 0;
            if (cap == 0)
            {
                best.Boundary(1).CopyTo(residual); residual.Add(term);
                if (residual.Sign(best.Root) == 0) cap = -1;
            }
            if (cap != 0)
                exactTerm = new(exactTerm.X, FixedPointAnchorTerm3d.GetResidual(Fixed64.One,
                    cap < 0 ? -input.FullHeight : input.FullHeight, Fixed64.Zero, Fixed64.Zero,
                    core.Y, local.Y), exactTerm.Z);
        }
        Span<ulong> normals = stackalloc ulong[7 * Words]; Span<int> normalSigns = stackalloc int[7];
        Span<ulong> source = stackalloc ulong[6 * Words]; Span<int> sourceSigns = stackalloc int[6];
        normals.Clear(); normalSigns.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            best.Normal(axis).Rational.CopyTo(Slot(source, axis, Words));
            best.Normal(axis).Radical.CopyTo(Slot(source, axis + 3, Words));
            sourceSigns[axis] = best.Normal(axis).Signs[0]; sourceSigns[axis + 3] = best.Normal(axis).Signs[1];
        }
        var basis = new WideRationalBasis3d(input.SlabRotation);
        WriteWorldDirection(basis, source[..(3 * Words)], sourceSigns[..3], normals[..(3 * Words)], normalSigns[..3], -1);
        WriteWorldDirection(basis, source[(3 * Words)..], sourceSigns[3..], normals.Slice(3 * Words, 3 * Words), normalSigns.Slice(3, 3), -1);
        best.Root.CopyTo(Slot(normals, 6, Words));
        Vector3d normal = WideConvexPrismRelations.GetConvexContactCandidateNormal(new ConvexContactCandidate(normals, normalSigns, 0, Words));
        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(new ConvexContactCandidate(selection.Metric,
            selection.MetricSigns, selection.MetricSigns[10], Words), Fixed64.Zero, out Fixed64 depth, out bool clamped);
        return new FixedContactAnchors(new FixedPointAnchor(input.Origin, input.Rotation, point),
            new FixedPointAnchor(input.Center, input.SlabRotation, core, local, exactTerm), normal, depth, clamped);
    }
}
