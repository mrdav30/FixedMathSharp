//=======================================================================
// WideConvex2dRelations.BoundaryIntervals.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Intersects retained nondegenerate native-yaw source edges in their original parameters.</content>
internal static partial class WideConvex2dRelations
{
    internal static bool TryIntersectBoundarySegments(Vector2d origin, Fixed64 yaw, FixedSegment2d edge,
        in RationalSegmentInterval clip, Vector2d otherOrigin, Fixed64 otherYaw, FixedSegment2d other,
        in RationalSegmentInterval otherClip, out RationalSegmentInterval intersection)
    {
        // The output may alias either retained input interval. Publish it only
        // after every input read, including on rejected intersections.
        GetRelativePointNumerators(origin, yaw, edge.Start, origin, out var ax, out var ay);
        GetRelativePointNumerators(origin, yaw, edge.End, origin, out var bx, out var by);
        GetRelativePointNumerators(otherOrigin, otherYaw, other.Start, origin, out var cx, out var cy);
        GetRelativePointNumerators(otherOrigin, otherYaw, other.End, origin, out var dx, out var dy);
        Signed192 ex = WideArithmetic.SubtractSigned192(bx, ax), ey = WideArithmetic.SubtractSigned192(by, ay),
            fx = WideArithmetic.SubtractSigned192(dx, cx), fy = WideArithmetic.SubtractSigned192(dy, cy),
            rx = WideArithmetic.SubtractSigned192(cx, ax), ry = WideArithmetic.SubtractSigned192(cy, ay);
        Signed320 denominator = WideArithmetic.MultiplySubtract(ex, fy, ey, fx);
        if (!denominator.IsZero)
        {
            Signed320 n = WideArithmetic.MultiplySubtract(rx, fy, ry, fx),
                otherN = WideArithmetic.MultiplySubtract(rx, ey, ry, ex);
            if (denominator.Sign < 0)
            { n = WideArithmetic.Negate(n); otherN = WideArithmetic.Negate(otherN); denominator = WideArithmetic.Negate(denominator); }
            if (!WithinBoundaryInterval(n, denominator, clip) || !WithinBoundaryInterval(otherN, denominator, otherClip))
            { intersection = default; return false; }
            intersection = new(n, denominator, n, denominator); return true;
        }
        if (!WideArithmetic.MultiplySubtract(rx, ey, ry, ex).IsZero) { intersection = default; return false; }
        // Native relative coordinates are <100 bits. Retained unit cuts <320
        // add at most 421 bits to these affine endpoint ratios; Signed832 keeps
        // the complete product before the interval owner reduces its content.
        Signed192 e = ex.IsZero ? ey : ex, f = ex.IsZero ? fy : fx, r = ex.IsZero ? ry : rx;
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
        Signed832 ln = otherClip.IsClipped ? otherClip.WideLowerNumerator : default,
            ld = otherClip.IsClipped ? otherClip.WideLowerDenominator : one,
            un = otherClip.IsClipped ? otherClip.WideUpperNumerator : one,
            ud = otherClip.IsClipped ? otherClip.WideUpperDenominator : one;
        ln = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned832(ld, r), WideArithmetic.MultiplySigned832(ln, f));
        un = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned832(ud, r), WideArithmetic.MultiplySigned832(un, f));
        ld = WideArithmetic.MultiplySigned832(ld, e); ud = WideArithmetic.MultiplySigned832(ud, e);
        if (e.Sign < 0)
        {
            ln = WideArithmetic.SubtractSigned832(default, ln); un = WideArithmetic.SubtractSigned832(default, un);
            ld = WideArithmetic.SubtractSigned832(default, ld); ud = WideArithmetic.SubtractSigned832(default, ud);
        }
        if (e.Sign * f.Sign < 0) { (ln, un) = (un, ln); (ld, ud) = (ud, ld); }
        Signed832 lower = clip.IsClipped ? clip.WideLowerNumerator : default,
            lowerD = clip.IsClipped ? clip.WideLowerDenominator : one,
            upper = clip.IsClipped ? clip.WideUpperNumerator : one,
            upperD = clip.IsClipped ? clip.WideUpperDenominator : one;
        // A translated collinear edge may begin before this source's unit
        // interval. The interval comparator requires nonnegative numerators;
        // clip those signed affine bounds before comparing retained ratios.
        if (un.Sign < 0) { intersection = default; return false; }
        if (ln.Sign < 0 || RationalSegmentInterval.CompareParameters(ln, ld, lower, lowerD) < 0) { ln = lower; ld = lowerD; }
        if (RationalSegmentInterval.CompareParameters(un, ud, upper, upperD) > 0) { un = upper; ud = upperD; }
        if (RationalSegmentInterval.CompareParameters(ln, ld, un, ud) > 0) { intersection = default; return false; }
        intersection = new(ln, ld, un, ud); return true;
    }

    private static bool WithinBoundaryInterval(Signed320 n, Signed320 d, in RationalSegmentInterval clip)
    {
        if (n.Sign < 0 || WideArithmetic.SubtractSigned320(n, d).Sign > 0) return false;
        return !clip.IsClipped || RationalSegmentInterval.CompareParameters(Signed832.ExtendValue(n), Signed832.ExtendValue(d),
                clip.WideLowerNumerator, clip.WideLowerDenominator) >= 0
            && RationalSegmentInterval.CompareParameters(Signed832.ExtendValue(n), Signed832.ExtendValue(d),
                clip.WideUpperNumerator, clip.WideUpperDenominator) <= 0;
    }
}
