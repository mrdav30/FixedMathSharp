//=======================================================================
// WideRationalBasis3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Preserves a quaternion-derived rotation basis as exact rational
/// numerators over one shared denominator.
/// </summary>
internal readonly struct WideRationalBasis3d
{
    internal readonly Signed192 Denominator;
    internal readonly Signed192 Xx;
    internal readonly Signed192 Xy;
    internal readonly Signed192 Xz;
    internal readonly Signed192 Yx;
    internal readonly Signed192 Yy;
    internal readonly Signed192 Yz;
    internal readonly Signed192 Zx;
    internal readonly Signed192 Zy;
    internal readonly Signed192 Zz;

    internal WideRationalBasis3d(FixedQuaternion orientation)
    {
        Signed192 xx = Fixed64.GetExactRawProduct(
            orientation.X,
            orientation.X);
        Signed192 yy = Fixed64.GetExactRawProduct(
            orientation.Y,
            orientation.Y);
        Signed192 zz = Fixed64.GetExactRawProduct(
            orientation.Z,
            orientation.Z);
        Signed192 ww = Fixed64.GetExactRawProduct(
            orientation.W,
            orientation.W);
        Signed192 xy = Fixed64.GetExactRawProduct(
            orientation.X,
            orientation.Y);
        Signed192 xz = Fixed64.GetExactRawProduct(
            orientation.X,
            orientation.Z);
        Signed192 xw = Fixed64.GetExactRawProduct(
            orientation.X,
            orientation.W);
        Signed192 yz = Fixed64.GetExactRawProduct(
            orientation.Y,
            orientation.Z);
        Signed192 yw = Fixed64.GetExactRawProduct(
            orientation.Y,
            orientation.W);
        Signed192 zw = Fixed64.GetExactRawProduct(
            orientation.Z,
            orientation.W);
        Denominator = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(xx, yy),
            WideArithmetic.AddSigned192(zz, ww));

        Xx = WideArithmetic.AddSigned192(
            WideArithmetic.SubtractSigned192(
                WideArithmetic.SubtractSigned192(xx, yy),
                zz),
            ww);
        Xy = Double(WideArithmetic.AddSigned192(xy, zw));
        Xz = Double(WideArithmetic.SubtractSigned192(xz, yw));
        Yx = Double(WideArithmetic.SubtractSigned192(xy, zw));
        Yy = WideArithmetic.AddSigned192(
            WideArithmetic.SubtractSigned192(
                WideArithmetic.SubtractSigned192(yy, xx),
                zz),
            ww);
        Yz = Double(WideArithmetic.AddSigned192(yz, xw));
        Zx = Double(WideArithmetic.AddSigned192(xz, yw));
        Zy = Double(WideArithmetic.SubtractSigned192(yz, xw));
        Zz = WideArithmetic.AddSigned192(
            WideArithmetic.SubtractSigned192(
                WideArithmetic.SubtractSigned192(zz, xx),
                yy),
            ww);
    }

    private WideRationalBasis3d(
        Signed192 denominator,
        Signed192 xx,
        Signed192 xy,
        Signed192 xz,
        Signed192 yx,
        Signed192 yy,
        Signed192 yz,
        Signed192 zx,
        Signed192 zy,
        Signed192 zz)
    {
        Denominator = denominator;
        Xx = xx;
        Xy = xy;
        Xz = xz;
        Yx = yx;
        Yy = yy;
        Yz = yz;
        Zx = zx;
        Zy = zy;
        Zz = zz;
    }

    internal static WideRationalBasis3d CreateRelative(
        in WideRationalBasis3d target,
        in WideRationalBasis3d source) =>
        // Normalized quaternion-basis products and their three-term sums fit
        // Signed192, including the unreduced shared denominator.
        new(
            Signed192.NarrowProven(
                WideArithmetic.MultiplySigned192(
                    target.Denominator,
                    source.Denominator)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Xx, target.Xy, target.Xz,
                source.Xx, source.Xy, source.Xz)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Yx, target.Yy, target.Yz,
                source.Xx, source.Xy, source.Xz)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Zx, target.Zy, target.Zz,
                source.Xx, source.Xy, source.Xz)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Xx, target.Xy, target.Xz,
                source.Yx, source.Yy, source.Yz)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Yx, target.Yy, target.Yz,
                source.Yx, source.Yy, source.Yz)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Zx, target.Zy, target.Zz,
                source.Yx, source.Yy, source.Yz)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Xx, target.Xy, target.Xz,
                source.Zx, source.Zy, source.Zz)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Yx, target.Yy, target.Yz,
                source.Zx, source.Zy, source.Zz)),
            Signed192.NarrowProven(WideArithmetic.GetDotProduct3D(
                target.Zx, target.Zy, target.Zz,
                source.Zx, source.Zy, source.Zz)));

    internal WideAxis3 GetAxis(int index) =>
        index switch
        {
            0 => new WideAxis3(
                Signed320.ExtendValue(Xx),
                Signed320.ExtendValue(Xy),
                Signed320.ExtendValue(Xz)),
            1 => new WideAxis3(
                Signed320.ExtendValue(Yx),
                Signed320.ExtendValue(Yy),
                Signed320.ExtendValue(Yz)),
            _ => new WideAxis3(
                Signed320.ExtendValue(Zx),
                Signed320.ExtendValue(Zy),
                Signed320.ExtendValue(Zz)),
        };

    internal static Signed576 GetComposedScaledCoordinateNumerator(
        Fixed64 outerLocalPoint,
        Fixed64 outerScale,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 rotationDenominator,
        Fixed64 innerFrameOffset,
        Fixed64 innerFrameScale,
        Vector3d innerLocalDisplacement)
    {
        Signed576 outerScaled = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(outerLocalPoint),
                Signed192.Raw(outerScale)),
            Signed320.ExtendValue(rotationDenominator));
        Signed576 innerScaled = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(innerFrameOffset),
                Signed192.Raw(innerFrameScale)),
            Signed320.ExtendValue(rotationDenominator));
        Signed320 displacement = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(innerLocalDisplacement.X),
                    axisX),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(innerLocalDisplacement.Y),
                    axisY)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(innerLocalDisplacement.Z),
                axisZ));
        return WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                outerScaled,
                innerScaled),
            WideArithmetic.MultiplySigned320(
                displacement,
                Signed320.ExtendValue(Signed192.One)));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 Double(Signed192 value) =>
        WideArithmetic.AddSigned192(value, value);
}
