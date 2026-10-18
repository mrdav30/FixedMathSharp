//=======================================================================
// WideFiniteAxisIntersection.ClippedAxisSegment.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Closest finite-axis witnesses on exact rational subsegments.</content>
internal static partial class WideFiniteAxisIntersection
{
    // Rigid coverage ratios use <268 bits (<269 after a narrow-family mediant).
    // Normalized-axis projection adds <70 bits: denominators use <339 bits,
    // coordinates/separations <405, and squared separations <810. Existing
    // Signed576/Signed832 candidates and finite-ray bounds therefore suffice.
    private static AxisTriangleCandidate GetClosestClippedAxisSegment(Vector3d first, Vector3d second,
        Vector3d center, Vector3d axis, Fixed64 length, in RationalSegmentInterval clip, out int endpoint)
    {
        GetClosestCenteredAxisSegmentParameters(center, axis, length, first, second,
            out Signed320 axisN, out Signed320 edgeN, out Signed320 d);
        Signed320 lowerN = clip.LowerNumerator, lowerD = clip.LowerDenominator;
        Signed320 upperN = clip.UpperNumerator, upperD = clip.UpperDenominator;
        Signed192 aa = GetDot(axis, Vector3d.Zero, axis, Vector3d.Zero);
        Signed192 ae = GetDot(axis, Vector3d.Zero, second, first);
        Signed192 ee = GetDot(second, first, second, first);
        Signed192 aw = GetDot(center, first, axis, Vector3d.Zero);
        endpoint = 0;
        if (length != Fixed64.Zero && !ee.IsZero && WideArithmetic.MultiplySubtract(aa, ee, ae, ae).IsZero)
        {
            // Intersect the whole parallel closest family before choosing an
            // interior witness; clamping its old midpoint would invent a corner.
            Signed320 cap = WideArithmetic.MultiplySigned192(Signed192.Raw(length), aa);
            Signed320 start = WideArithmetic.MultiplySigned192(DoubleParameterScale, aw);
            Signed320 delta = WideArithmetic.MultiplySigned192(DoubleParameterScale, ae);
            Signed320 f0 = WideArithmetic.AddSigned320(cap, start);
            Signed320 f1 = WideArithmetic.SubtractSigned320(f0, delta);
            Signed320 g0 = WideArithmetic.SubtractSigned320(cap, start);
            Signed320 g1 = WideArithmetic.AddSigned320(g0, delta);
            if (RationalSegmentInterval.TryClipHalfPlane(f0, f1, ref lowerN, ref lowerD, ref upperN, ref upperD)
                && RationalSegmentInterval.TryClipHalfPlane(g0, g1, ref lowerN, ref lowerD, ref upperN, ref upperD)
                && RationalSegmentInterval.CompareParameters(lowerN, lowerD, upperN, upperD) < 0)
            {
                if (RationalSegmentInterval.CompareParameters(edgeN, d, lowerN, lowerD) <= 0
                    || RationalSegmentInterval.CompareParameters(edgeN, d, upperN, upperD) >= 0)
                {
                    SelectInteriorParameter(lowerN, lowerD, upperN, upperD, out edgeN, out d);
                    return AtParameter(edgeN, d);
                }
                return NativeCandidate();
            }
        }
        int low = RationalSegmentInterval.CompareParameters(edgeN, d, clip.LowerNumerator, clip.LowerDenominator);
        int high = RationalSegmentInterval.CompareParameters(edgeN, d, clip.UpperNumerator, clip.UpperDenominator);
        if (low <= 0) { endpoint = -1; return AtParameter(clip.LowerNumerator, clip.LowerDenominator); }
        if (high >= 0) { endpoint = 1; return AtParameter(clip.UpperNumerator, clip.UpperDenominator); }
        return NativeCandidate();

        AxisTriangleCandidate NativeCandidate()
        {
            GetRationalPoint(center, axis, axisN, d, out Signed576 x, out Signed576 y, out Signed576 z);
            GetRationalSegmentPoint(first, second, edgeN, d, out Signed576 px, out Signed576 py, out Signed576 pz);
            return CreateAxisTriangleCandidate(px, py, pz, x, y, z, Signed576.ExtendValue(d), 7);
        }

        AxisTriangleCandidate AtParameter(Signed320 n, Signed320 denominator)
        {
            Signed576 common = Signed576.ExtendValue(denominator), edge = Signed576.ExtendValue(n);
            Signed576 t = default;
            if (length != Fixed64.Zero)
            {
                t = WideArithmetic.SubtractSigned576(Multiply(n, ae), Multiply(denominator, aw));
                common = Multiply(denominator, aa);
                edge = Multiply(n, aa);
                Signed576 scaled = WideArithmetic.MultiplySigned576(t, DoubleParameterScale);
                Signed576 limit = WideArithmetic.MultiplySigned576(common, Signed192.Raw(length));
                if (WideArithmetic.SubtractSigned576(scaled, WideArithmetic.SubtractSigned576(default, limit)).Sign < 0
                    || WideArithmetic.SubtractSigned576(scaled, limit).Sign > 0)
                {
                    // Reproject the constrained point onto the finite core cap.
                    t = Multiply(denominator, scaled.Sign < 0 ? Signed192.Signed(-length.m_rawValue) : Signed192.Raw(length));
                    common = Multiply(denominator, DoubleParameterScale);
                    edge = Multiply(n, DoubleParameterScale);
                }
            }
            return CreateAxisTriangleCandidate(
                Coordinate(first.X, GetComponentDifference(second.X, first.X), edge, common),
                Coordinate(first.Y, GetComponentDifference(second.Y, first.Y), edge, common),
                Coordinate(first.Z, GetComponentDifference(second.Z, first.Z), edge, common),
                Coordinate(center.X, Signed192.Raw(axis.X), t, common),
                Coordinate(center.Y, Signed192.Raw(axis.Y), t, common),
                Coordinate(center.Z, Signed192.Raw(axis.Z), t, common), common, 7);
        }
    }

    private static Signed576 Multiply(Signed320 a, Signed192 b) =>
        WideArithmetic.MultiplySigned320(a, Signed320.ExtendValue(b));

    private static Signed576 Coordinate(Fixed64 origin, Signed192 direction, Signed576 n, Signed576 d) =>
        WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned576(d, Signed192.Raw(origin)),
            WideArithmetic.MultiplySigned576(n, direction));

    private static void SelectInteriorParameter(Signed320 ln, Signed320 ld, Signed320 un, Signed320 ud,
        out Signed320 n, out Signed320 d)
    {
        Signed192 grid = new(1, 0, 0); // 2^128; parameter precision, not a geometry tolerance.
        Signed192 lo = WideArithmetic.AddSigned192(WideArithmetic.DivideSigned576ToSigned192(Multiply(ln, grid), ld), Signed192.Signed(1));
        Signed192 hi = WideArithmetic.DivideSigned576ToSigned192(Multiply(un, grid), ud);
        if (RationalSegmentInterval.CompareParameters(Signed320.ExtendValue(hi), Signed320.ExtendValue(grid), un, ud) == 0)
            hi = WideArithmetic.SubtractSigned192(hi, Signed192.Signed(1));
        if (WideArithmetic.SubtractSigned192(hi, lo).Sign >= 0)
        {
            // The exact midpoint of the first/last interior grid indices is
            // reversal symmetric and within 2^-128 of the true family midpoint.
            n = Signed320.ExtendValue(WideArithmetic.AddSigned192(lo, hi));
            d = Signed320.ExtendValue(new Signed192(2, 0, 0));
            return;
        }
        // No grid point: the entire family is no wider than 2^-128. A reduced
        // mediant is strictly interior; its possible anchor shift is sub-raw
        // even for full-domain edges. Reduction removes representation weighting.
        Span<Signed320> lower = stackalloc Signed320[2] { ln, ld }, upper = stackalloc Signed320[2] { un, ud };
        WideArithmetic.ReduceCommonScale(lower); WideArithmetic.ReduceCommonScale(upper);
        n = WideArithmetic.AddSigned320(lower[0], upper[0]);
        d = WideArithmetic.AddSigned320(lower[1], upper[1]);
    }
}
