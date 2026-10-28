//=======================================================================
// WideConvexPrismRelations.CapsuleDomain.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Retains exact capsule support intervals in bounded native-prism face domains.</content>
internal static partial class WideConvexPrismRelations
{
    /// <summary>Clips an already projected support segment to a retained native side rectangle or cap.</summary>
    /// <remarks>
    /// Endpoints are raw world-coordinate offsets from the prism origin over pointD. Admitted geometry
    /// has reach at most 4096; native tangent-cut bounds use fewer than 162 bits. A cap's complete
    /// footprint has already been admitted by coreClip, so cap clipping needs only the supplied Y plane.
    /// </remarks>
    internal static bool TryClipProjectedPrismFace(ReadOnlySpan<Signed832> projected, Signed832 pointD,
        Fixed64 yaw, Vector2d start, Vector2d end, in RationalSegmentInterval tangentClip,
        Fixed64 lowerY, Fixed64 upperY, in RationalSegmentInterval coreClip,
        out RationalSegmentInterval retained, bool cap = false)
    {
        retained = default;
        var rotation = new WideConvex2dRelations.RotationFrame2d(yaw);
        Signed320 metric = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(rotation.Cosine, rotation.Cosine),
            WideArithmetic.MultiplySigned192(rotation.Sine, rotation.Sine));
        Signed832 localD = WideArithmetic.MultiplySigned832(pointD, metric);
        Span<Signed832> local = stackalloc Signed832[6];
        for (int endpoint = 0; endpoint < 2; endpoint++)
        {
            int index = 3 * endpoint;
            local[index] = WideArithmetic.MultiplySigned832(WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned832(projected[index], rotation.Cosine),
                WideArithmetic.MultiplySigned832(projected[index + 2], rotation.Sine)), Scale);
            local[index + 1] = WideArithmetic.MultiplySigned832(projected[index + 1], metric);
            local[index + 2] = WideArithmetic.MultiplySigned832(WideArithmetic.SubtractSigned832(
                WideArithmetic.MultiplySigned832(projected[index + 2], rotation.Cosine),
                WideArithmetic.MultiplySigned832(projected[index], rotation.Sine)), Scale);
        }
        Signed832 lowerN = default, lowerD = Signed832.ExtendValue(Signed192.Signed(1)), upperN = lowerD, upperD = lowerD;
        if (coreClip.IsClipped)
        {
            lowerN = coreClip.WideLowerNumerator; lowerD = coreClip.WideLowerDenominator;
            upperN = coreClip.WideUpperNumerator; upperD = coreClip.WideUpperDenominator;
        }
        // Native inversion divides by the actual c²+s². Projected numerators <551
        // bits become <617; retained tangent constraints stay below 780 bits.
        if (!cap)
        {
            Signed192 dx = WideArithmetic.Difference(end.X, start.X), dz = WideArithmetic.Difference(end.Y, start.Y);
            if (dx.IsZero && dz.IsZero) return false;
            Span<Signed832> x = stackalloc Signed832[2], z = stackalloc Signed832[2], side = stackalloc Signed832[2];
            for (int endpoint = 0; endpoint < 2; endpoint++)
            {
                x[endpoint] = WideArithmetic.SubtractSigned832(local[3 * endpoint],
                    WideArithmetic.MultiplySigned832(localD, Signed192.Raw(start.X)));
                z[endpoint] = WideArithmetic.SubtractSigned832(local[3 * endpoint + 2],
                    WideArithmetic.MultiplySigned832(localD, Signed192.Raw(start.Y)));
                side[endpoint] = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(x[endpoint], dz),
                    WideArithmetic.MultiplySigned832(z[endpoint], dx));
            }
            if (!WideTriangleRelations.ClipScaledHalfPlane(side[0], side[1], ref lowerN, ref lowerD, ref upperN, ref upperD)
                || !WideTriangleRelations.ClipScaledHalfPlane(WideArithmetic.SubtractSigned832(default, side[0]),
                    WideArithmetic.SubtractSigned832(default, side[1]), ref lowerN, ref lowerD, ref upperN, ref upperD)) return false;

            Signed192 delta = dx.IsZero ? dz : dx;
            ReadOnlySpan<Signed832> coordinate = dx.IsZero ? z : x;
            Signed320 tangentLowerN = tangentClip.IsClipped ? tangentClip.LowerNumerator : default;
            Signed320 tangentLowerD = tangentClip.IsClipped ? tangentClip.LowerDenominator : Signed320.One;
            Signed320 tangentUpperN = tangentClip.IsClipped ? tangentClip.UpperNumerator : Signed320.One;
            Signed320 tangentUpperD = tangentClip.IsClipped ? tangentClip.UpperDenominator : Signed320.One;
            Signed832 edgeD = WideArithmetic.MultiplySigned832(localD, delta);
            Span<Signed832> low = stackalloc Signed832[2], high = stackalloc Signed832[2];
            for (int endpoint = 0; endpoint < 2; endpoint++)
            {
                low[endpoint] = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(coordinate[endpoint], tangentLowerD),
                    WideArithmetic.MultiplySigned832(edgeD, tangentLowerN));
                high[endpoint] = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(edgeD, tangentUpperN),
                    WideArithmetic.MultiplySigned832(coordinate[endpoint], tangentUpperD));
                if (delta.Sign < 0)
                {
                    low[endpoint] = WideArithmetic.SubtractSigned832(default, low[endpoint]);
                    high[endpoint] = WideArithmetic.SubtractSigned832(default, high[endpoint]);
                }
            }
            if (!WideTriangleRelations.ClipScaledHalfPlane(low[0], low[1], ref lowerN, ref lowerD, ref upperN, ref upperD)
                || !WideTriangleRelations.ClipScaledHalfPlane(high[0], high[1], ref lowerN, ref lowerD, ref upperN, ref upperD)) return false;
        }
        Signed832 bottom = WideArithmetic.MultiplySigned832(localD, Signed192.Raw(lowerY));
        Signed832 top = WideArithmetic.MultiplySigned832(localD, Signed192.Raw(upperY));
        if (!WideTriangleRelations.ClipScaledHalfPlane(WideArithmetic.SubtractSigned832(local[1], bottom),
                WideArithmetic.SubtractSigned832(local[4], bottom), ref lowerN, ref lowerD, ref upperN, ref upperD)
            || !WideTriangleRelations.ClipScaledHalfPlane(WideArithmetic.SubtractSigned832(top, local[1]),
                WideArithmetic.SubtractSigned832(top, local[4]), ref lowerN, ref lowerD, ref upperN, ref upperD)) return false;
        retained = new RationalSegmentInterval(lowerN, lowerD, upperN, upperD);
        return true;
    }

    private static void GetPrismInteriorParameter(in RationalSegmentInterval interval, out Signed832 n, out Signed832 d)
    {
        if (!interval.IsClipped) { n = Signed832.ExtendValue(Signed192.Signed(1)); d = Signed832.ExtendValue(Signed192.Signed(2)); return; }
        // Reduced-bound mediants are exact interior points independent of the
        // producer's coefficient scale. Native face bounds need <780 bits, so the sum fits.
        Span<ulong> values = stackalloc ulong[52];
        WideArithmetic.GetMagnitude(interval.WideLowerNumerator, values[..13]);
        WideArithmetic.GetMagnitude(interval.WideLowerDenominator, values.Slice(13, 13));
        WideArithmetic.GetMagnitude(interval.WideUpperNumerator, values.Slice(26, 13));
        WideArithmetic.GetMagnitude(interval.WideUpperDenominator, values.Slice(39, 13));
        RationalSegmentInterval.PackReducedParameter(values[..13], values.Slice(13, 13), out var ln, out var ld);
        RationalSegmentInterval.PackReducedParameter(values.Slice(26, 13), values.Slice(39, 13), out var un, out var ud);
        n = WideArithmetic.AddSigned832(ln, un); d = WideArithmetic.AddSigned832(ld, ud);
    }

    /// <summary>Tests a projected support point against the complete rotated capsule before physical/recovery classification.</summary>
    /// <remarks>relativeCenter is the capsule center minus the projected coordinates' prism origin.</remarks>
    internal static bool IsProjectedPrismPointWithinCapsule(ReadOnlySpan<Signed832> projected, Signed832 pointD,
        Signed832 tN, Signed832 tD, Vector3d relativeCenter, FixedQuaternion rotation, Fixed64 length, Fixed64 radius)
    {
        // Projected coefficients <551/503 bits and an 832-bit unit parameter
        // leave transformed doubled coordinates <1466 bits and their norm <2934.
        const int words = 48;
        Span<ulong> values = stackalloc ulong[28 * words]; Span<int> signs = stackalloc int[28];
        var dx = ContactQuadratic.At(values, signs, 0, words);
        var dy = ContactQuadratic.At(values, signs, 1, words);
        var dz = ContactQuadratic.At(values, signs, 2, words);
        var parameter = ContactQuadratic.At(values, signs, 3, words); parameter.Set(tN);
        var remaining = ContactQuadratic.At(values, signs, 4, words); remaining.Set(WideArithmetic.SubtractSigned832(tD, tN));
        var sourceD = ContactQuadratic.At(values, signs, 5, words); sourceD.Set(pointD);
        var denominator = ContactQuadratic.At(values, signs, 6, words);
        var localD = ContactQuadratic.At(values, signs, 7, words);
        var value = ContactQuadratic.At(values, signs, 8, words);
        var work = ContactQuadratic.At(values, signs, 9, words);
        var local = ContactQuadratic.At(values, signs, 10, words);
        var squared = ContactQuadratic.At(values, signs, 11, words); squared.Clear();
        var bound = ContactQuadratic.At(values, signs, 12, words);
        var term = ContactQuadratic.At(values, signs, 13, words);
        value.Set(tD); ContactQuadratic.Multiply(sourceD, value, ReadOnlySpan<ulong>.Empty, denominator);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic coordinate = axis == 0 ? dx : axis == 1 ? dy : dz;
            value.Set(projected[axis]); ContactQuadratic.Multiply(value, remaining, ReadOnlySpan<ulong>.Empty, coordinate);
            value.Set(projected[axis + 3]); ContactQuadratic.Multiply(value, parameter, ReadOnlySpan<ulong>.Empty, work); coordinate.Add(work);
            ContactQuadratic.Scale(denominator, Signed576.ExtendValue(Signed192.Raw(relativeCenter[axis])), work); coordinate.Add(work, -1);
        }
        var basis = new WideRationalBasis3d(rotation);
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(basis.Denominator), localD);
        ReadOnlySpan<Signed192> coefficients = stackalloc Signed192[9]
            { basis.Xx, basis.Xy, basis.Xz, basis.Yx, basis.Yy, basis.Yz, basis.Zx, basis.Zy, basis.Zz };
        Signed576 two = Signed576.ExtendValue(Signed192.Signed(2));
        for (int axis = 0; axis < 3; axis++)
        {
            local.Clear();
            for (int component = 0; component < 3; component++)
            {
                ContactQuadratic coordinate = component == 0 ? dx : component == 1 ? dy : dz;
                ContactQuadratic.Scale(coordinate, Signed576.ExtendValue(coefficients[3 * axis + component]), work); local.Add(work);
            }
            ContactQuadratic.Scale(local, two, work); work.CopyTo(local);
            if (axis == 1)
            {
                if (local.Signs[0] < 0) local.MultiplySign(-1);
                ContactQuadratic.Scale(localD, Signed576.ExtendValue(Signed192.Raw(length)), work); local.Add(work, -1);
                if (local.Signs[0] < 0) local.Clear();
            }
            ContactQuadratic.Multiply(local, local, ReadOnlySpan<ulong>.Empty, term); squared.Add(term);
        }
        ContactQuadratic.Scale(localD, Signed576.ExtendValue(Signed192.Raw(radius)), work);
        ContactQuadratic.Scale(work, two, bound);
        ContactQuadratic.Multiply(bound, bound, ReadOnlySpan<ulong>.Empty, term); squared.Add(term, -1);
        return squared.Signs[0] <= 0;
    }
}
