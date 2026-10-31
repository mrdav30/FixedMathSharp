//=======================================================================
// SegmentConeSurfaceCandidates.Rim.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.CircularRimContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>All stationary rim roots with independent exact finite-segment admission.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    private const int RimSlots = 31;

    private static bool TryGetRimSquaredDepth(in SegmentConeSurfaceGeometry geometry,
        in SegmentConeSurfaceCandidate candidate, Span<ulong> values, Span<sbyte> valueSigns,
        Span<ulong> valueCell, out FiniteAxisValueRoot value)
    {
        int Words = geometry.Words;
        GetRimBasis(geometry.Edge, candidate.Chart, out WideDirection3d first, out WideDirection3d second);
        Span<ulong> data = stackalloc ulong[RimSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[RimSlots];
        TriangleConeRimContacts.BuildConeParameter(geometry.BaseOffset, geometry.Radius, geometry.RawScale,
            geometry.Input.Height, geometry.Input.Radius, first, second, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5], candidate.RootOrdinal, cell, out FiniteAxisValueRoot parameter);
        System.Diagnostics.Debug.Assert(found);
        value = default;
        if (Sign(ref parameter, data, signs, 13, 3) == 0)
            return false;
        // The cone parameter already retains the unsquared admitted gap and
        // its squared rational expression. Reuse its existing value polynomial
        // and isolator; squaring cannot reverse these nonnegative depths.
        BuildValues(geometry.Edge, geometry.BaseOffset, geometry.Radius, geometry.ValueShift, values, valueSigns);
        scoped FiniteAxisValueRoot mapped = ConvexContactValueRoot.MapSquaredValue(geometry.RawScale, geometry.ValueShift,
            ref parameter, data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5), data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5),
            values, valueSigns, valueCell);
        // Only the mapped polynomial and cell are retained. Rebuild the view
        // from caller storage explicitly; the local parameter chart cannot escape.
        value = new FiniteAxisValueRoot
        {
            Coefficients = values[..mapped.Coefficients.Length], Signs = valueSigns[..mapped.Signs.Length],
            LowerNumerator = valueCell, DenominatorShift = mapped.DenominatorShift,
            Ordinal = mapped.Ordinal, IsRational = mapped.IsRational
        };
        return true;
    }

    private static void AccumulateRimRoots(in SegmentConeSurfaceGeometry geometry, ref ConeSurfaceSelection selection)
    {

        // Horizontal and axial segments use the principal meridional/circular
        // charts. The quartic chart's K!=0 admission deliberately excludes them.
        if (geometry.Edge.Y.IsZero || geometry.Edge.X.IsZero && geometry.Edge.Z.IsZero || geometry.Radius.IsZero || IsMeridionalRim(geometry))
            return;
        for (int chart = 0; chart < 4; chart++)
            AccumulateRimChart(geometry, chart, ref selection);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void AccumulateRimChart(in SegmentConeSurfaceGeometry geometry, int chart, ref ConeSurfaceSelection selection)
    {
        int Words = geometry.Words;
        GetRimBasis(geometry.Edge, chart, out WideDirection3d first, out WideDirection3d second);
        Span<ulong> data = stackalloc ulong[RimSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[RimSlots];
        // C,R and the affine chart axes are <200 bits; scale<134.
        // M<403,P<402,Q<802,K<807,L<1207 give W<2421,N<2426,D<2288:
        // the shared forty-word cone parameter retains every coefficient.
        TriangleConeRimContacts.BuildConeParameter(geometry.BaseOffset, geometry.Radius, geometry.RawScale,
            geometry.Input.Height, geometry.Input.Radius, first, second, data, signs);
        Span<ulong> batchCells = stackalloc ulong[64];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(
            data[..(5 * Words)], signs[..5], batchCells, batchShifts);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, data[..(5 * Words)], signs[..5], cell);
            int kSign = Sign(ref root, data, signs, 8, 2);
            if (kSign == 0 || Sign(ref root, data, signs, 10, 3) != -kSign)
                continue;
            if (Sign(ref root, data, signs, 26, 2) < 0 && Sign(ref root, data, signs, 28, 3) <= 0)
                continue;
            // n.E=0 and the preflight supplies an intersecting segment point.
            // Its projection is at least the finite cone support minimum.
#if DEBUG
            System.Diagnostics.Debug.Assert(Sign(ref root, data, signs, 13, 3) * kSign >= 0);
#endif
            if (!AdmitRimParameter(geometry, first, second, ref root, out ConeSurfacePointLocation location))
                continue;
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, chart, ordinal, location));
        }
    }

    private static void GetRimBasis(WideDirection3d edge, int chart, out WideDirection3d first, out WideDirection3d second)
    {
        GetBasis(edge, out first, out second);
        // Consecutive (+u,+v,-u,-v) quadrant endpoints. The isolator's (0,1]
        // domain includes each basis axis as the preceding quadrant's end.
        if (chart == 1) (first, second) = (second, -first);
        else if (chart == 2) (first, second) = (-first, -second);
        else if (chart == 3) (first, second) = (-second, first);
        second = new WideDirection3d(WideArithmetic.SubtractSigned576(second.X, first.X),
            WideArithmetic.SubtractSigned576(second.Y, first.Y), WideArithmetic.SubtractSigned576(second.Z, first.Z));
    }

    private static void GetRimParameter(in SegmentConeSurfaceGeometry geometry, WideDirection3d first, WideDirection3d second,
        out Signed832 a0, out Signed832 a1, out Signed832 b0, out Signed832 b1, out Signed832 d0, out Signed832 d1)
    {

        WideDirection3d tangent0 = new(WideArithmetic.Negate(first.Z), default, first.X);
        WideDirection3d tangent1 = new(WideArithmetic.Negate(second.Z), default, second.X);
        a0 = WideDirection3d.Dot(geometry.A, tangent0); a1 = WideDirection3d.Dot(geometry.A, tangent1);
        b0 = WideDirection3d.Dot(geometry.B, tangent0); b1 = WideDirection3d.Dot(geometry.B, tangent1);
        d0 = WideArithmetic.SubtractSigned832(b0, a0); d1 = WideArithmetic.SubtractSigned832(b1, a1);
    }

    private static bool IsRimParameterInInterval(in SegmentConeSurfaceGeometry geometry,
        in SegmentConeSurfaceCandidate candidate, in RationalSegmentInterval clip, bool interiorOnly)
    {
        int Words = geometry.Words;
        GetRimBasis(geometry.Edge, candidate.Chart, out WideDirection3d first, out WideDirection3d second);
        Span<ulong> data = stackalloc ulong[RimSlots * Words]; Span<sbyte> signs = stackalloc sbyte[RimSlots];
        TriangleConeRimContacts.BuildConeParameter(geometry.BaseOffset, geometry.Radius, geometry.RawScale,
            geometry.Input.Height, geometry.Input.Radius, first, second, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5], candidate.RootOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        GetRimParameter(geometry, first, second, out Signed832 a0, out Signed832 a1,
            out _, out _, out Signed832 d0, out Signed832 d1);
        Span<ulong> query = stackalloc ulong[2 * Words]; Span<sbyte> querySigns = stackalloc sbyte[2];
        int denominatorSign = LinearSign(ref root, d0, d1, query, querySigns);
        for (int bound = 0; bound < 2; bound++)
        {
            Signed832 n = bound == 0 ? clip.WideLowerNumerator : clip.WideUpperNumerator;
            Signed832 d = bound == 0 ? clip.WideLowerDenominator : clip.WideUpperDenominator;
            // t=-a(u)/d(u). Full clip weights add at most 319 bits to
            // the <432-bit affine coefficients; Signed832 retains both products.
            for (int power = 0; power < 2; power++)
            {
                Span<ulong> destination = Slot(query, power, Words);
                CylinderContactAlgebra.DifferenceOfProducts(power == 0 ? a0 : a1, d,
                    WideArithmetic.SubtractSigned832(default, power == 0 ? d0 : d1), n, destination, out int coefficientSign);
                querySigns[power] = (sbyte)-coefficientSign;
            }
            int sign = denominatorSign * WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
            if (bound == 0 ? sign < 0 : sign > 0) return false;
            if (interiorOnly && sign == 0) return false;
        }
        return true;
    }

    private static bool AdmitRimParameter(in SegmentConeSurfaceGeometry geometry, WideDirection3d first, WideDirection3d second,
        ref FiniteAxisValueRoot root, out ConeSurfacePointLocation location)
    {
        int Words = geometry.Words;
        GetRimParameter(geometry, first, second, out Signed832 a0, out Signed832 a1,
            out Signed832 b0, out Signed832 b1, out Signed832 d0, out Signed832 d1);
        Span<ulong> query = stackalloc ulong[2 * Words];
        Span<sbyte> querySigns = stackalloc sbyte[2];
        int denominatorSign = LinearSign(ref root, d0, d1, query, querySigns);
        location = ConeSurfacePointLocation.SegmentIntervalUnspecified;
        // The unsquared stationary equation makes p-q parallel to n.
        // Its radial tangent recovers the same finite foot directly; a
        // zero denominator would be a separately handled principal stratum.
        System.Diagnostics.Debug.Assert(denominatorSign != 0);
        int lower = -denominatorSign * LinearSign(ref root, a0, a1, query, querySigns);
        int upper = denominatorSign * LinearSign(ref root, b0, b1, query, querySigns);
        if (lower < 0 || upper < 0)
            return false;
        location = lower == 0 ? ConeSurfacePointLocation.Start : upper == 0 ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Interior;
        return ContainsRimFoot(geometry, a0, a1, d0, d1, denominatorSign, ref root);
    }

    private static int LinearSign(ref FiniteAxisValueRoot root, Signed832 first, Signed832 second,
        scoped Span<ulong> values, scoped Span<sbyte> signs)
    {
        Write(values, signs, 0, first); Write(values, signs, 1, second);
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, values, signs);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool ContainsRimFoot(in SegmentConeSurfaceGeometry geometry, Signed832 a0, Signed832 a1,
        Signed832 d0, Signed832 d1, int denominatorSign, ref FiniteAxisValueRoot root)
    {
        int Words = geometry.Words;
        // t=-a/d, where both a,d are affine in the retained quartic root.
        // Source coordinates <198 bits and a,d <432 give point numerators
        // <631. The cone query coefficients are <1390 bits; forty words
        // suffice without introducing a second algebraic root or rounding t.
        Span<ulong> points = stackalloc ulong[8 * Words];
        Span<sbyte> pointSigns = stackalloc sbyte[8];
        WideDirection3d start = new(geometry.A.X, WideArithmetic.SubtractSigned576(geometry.Cap, geometry.A.Y), geometry.A.Z);
        WideDirection3d delta = new(geometry.Edge.X, WideArithmetic.Negate(geometry.Edge.Y), geometry.Edge.Z);
        for (int axis = 0; axis < 3; axis++)
        for (int power = 0; power < 2; power++)
        {
            Span<ulong> destination = Slot(points, 2 * axis + power, Words);
            CylinderContactAlgebra.DifferenceOfProducts(power == 0 ? d0 : d1, Signed832.ExtendValue(Component(start, axis)),
                power == 0 ? a0 : a1, Signed832.ExtendValue(Component(delta, axis)), destination, out int coefficientSign);
            pointSigns[2 * axis + power] = (sbyte)(denominatorSign * coefficientSign);
        }
        Write(points, pointSigns, 6, d0); Write(points, pointSigns, 7, d1);
        pointSigns[6] *= (sbyte)denominatorSign; pointSigns[7] *= (sbyte)denominatorSign;
        if (WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, points.Slice(2 * Words, 2 * Words), pointSigns.Slice(2, 2)) < 0)
            return false;
        Span<ulong> query = stackalloc ulong[3 * Words];
        Span<sbyte> querySigns = stackalloc sbyte[3];
        Span<ulong> product = stackalloc ulong[Words];
        Span<ulong> scale = stackalloc ulong[Words];
        Import(geometry.FullHeight, scale);
        for (int power = 0; power < 2; power++)
        {
            WideArithmetic.MultiplyMagnitudes(Slot(points, 6 + power, Words), scale, Slot(query, power, Words));
            int sign = pointSigns[6 + power];
            CylinderContactAlgebra.Add(Slot(points, 2 + power, Words), -pointSigns[2 + power], Slot(query, power, Words), ref sign);
            querySigns[power] = (sbyte)sign;
        }
        if (WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query[..(2 * Words)], querySigns[..2]) < 0)
            return false;
        query.Clear(); querySigns.Clear();
        Span<ulong> square = stackalloc ulong[3 * Words];
        Span<sbyte> squareSigns = stackalloc sbyte[3];
        for (int axis = 0; axis < 3; axis++)
        {
            ReadOnlySpan<ulong> coordinate = points.Slice(2 * axis * Words, 2 * Words);
            ReadOnlySpan<sbyte> coordinateSigns = pointSigns.Slice(2 * axis, 2);
            WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(coordinate, coordinateSigns,
                coordinate, coordinateSigns, square, squareSigns, product);
            Signed192 dimension = Signed192.Raw(axis == 1 ? geometry.Input.Radius : geometry.Input.Height);
            Import(WideArithmetic.MultiplySigned192(dimension, dimension), scale);
            for (int power = 0; power < 3; power++)
            {
                WideArithmetic.MultiplyMagnitudes(Slot(square, power, Words), scale, product);
                int sign = querySigns[power];
                CylinderContactAlgebra.Add(product, squareSigns[power] * (axis == 1 ? -1 : 1), Slot(query, power, Words), ref sign);
                querySigns[power] = (sbyte)sign;
            }
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns) <= 0;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static FixedContactAnchors GetRimRootContact(in SegmentConeSurfaceCandidate candidate)
    {
        int Words = candidate.Input.Words;
        int ValueWords = candidate.Input.ValueWords;
        int ValueCellWords = (16 * (ValueWords * 64 + 11) + 255) / 64;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        GetRimBasis(geometry.Edge, candidate.Chart, out WideDirection3d first, out WideDirection3d second);
        Span<ulong> data = stackalloc ulong[RimSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[RimSlots];
        TriangleConeRimContacts.BuildConeParameter(geometry.BaseOffset, geometry.Radius, geometry.RawScale,
            geometry.Input.Height, geometry.Input.Radius, first, second, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5], candidate.RootOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        GetRimParameter(geometry, first, second, out Signed832 a0, out Signed832 a1,
            out Signed832 b0, out Signed832 b1, out Signed832 d0, out Signed832 d1);
        Span<ulong> query = stackalloc ulong[2 * Words];
        Span<sbyte> querySigns = stackalloc sbyte[2];
        int denominatorSign = LinearSign(ref root, d0, d1, query, querySigns);
        FixedSegment segment = candidate.Input.Segment;
        Vector3d point = default;
        for (int axis = 0; axis < 3; axis++)
            point[axis] = geometry.Input.IsAffine
                ? TriangleCylinderRimWitnesses.RoundRootCoordinate(Component(geometry.A, axis), Component(geometry.B, axis),
                    geometry.RawScale, a0, a1, b0, b1, d0, d1, denominatorSign, ref root)
                : TriangleCylinderRimWitnesses.RoundRootCoordinate(segment.Start[axis], segment.End[axis],
                    a0, a1, b0, b1, d0, d1, denominatorSign, ref root);
        Span<ulong> gradient = stackalloc ulong[54];
        Span<sbyte> gradientSigns = stackalloc sbyte[6];
        WriteGradient(first, second, gradient, gradientSigns);
        gradient.Slice(18, 18).Clear(); gradientSigns.Slice(2, 2).Clear();
        Vector3d radialPoint = ConvexContactValueRoot.GetScaledNormalizedDirection(ref root, gradient, gradientSigns, candidate.Input.Radius, -1);
        var basis = new WideRationalBasis3d(candidate.Input.ConeRotation);
        WriteGradient(Transform(basis, first), Transform(basis, second), gradient, gradientSigns);
        Vector3d normal = ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, gradientSigns, 1);
        Fixed64 depth = default;
        bool clamped = false;
        if (Sign(ref root, data, signs, 13, 3) != 0)
        {
            Span<ulong> values = stackalloc ulong[5 * ValueWords];
            Span<sbyte> valueSigns = stackalloc sbyte[5];
            Span<ulong> valueCell = stackalloc ulong[ValueCellWords];
            BuildValues(geometry.Edge, geometry.BaseOffset, geometry.Radius, geometry.ValueShift, values, valueSigns);
            FiniteAxisValueRoot value = ConvexContactValueRoot.MapSquaredValue(geometry.RawScale, geometry.ValueShift,
                ref root, data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5), data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5),
                values, valueSigns, valueCell);
            ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift, ref value, out depth, out clamped);
        }
        return new FixedContactAnchors(new FixedPointAnchor(candidate.Input.Origin, candidate.Input.Rotation, point),
            TriangleCircularGeometry.GetSupport(candidate.Input.Center, candidate.Input.ConeRotation,
                Signed192.Raw(candidate.Input.Height), radialPoint, 1), normal, depth, clamped);
    }
}
