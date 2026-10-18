//=======================================================================
// WideAxis3.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Stores an exact three-component axis used by rigid wide geometry.
/// </summary>
internal readonly struct WideAxis3
{
    internal readonly Signed320 X;
    internal readonly Signed320 Y;
    internal readonly Signed320 Z;

    internal WideAxis3(Signed320 x, Signed320 y, Signed320 z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    internal bool IsZero => X.IsZero && Y.IsZero && Z.IsZero;

    internal Signed576 SquaredLength =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(X, X),
                WideArithmetic.MultiplySigned320(Y, Y)),
            WideArithmetic.MultiplySigned320(Z, Z));

    internal static WideAxis3 Cross(in WideAxis3 left, in WideAxis3 right) =>
        new(
            Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(left.Y, right.Z),
                WideArithmetic.MultiplySigned320(left.Z, right.Y))),
            Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(left.Z, right.X),
                WideArithmetic.MultiplySigned320(left.X, right.Z))),
            Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(left.X, right.Y),
                WideArithmetic.MultiplySigned320(left.Y, right.X))));

    // Equality needs the complete cross products, even when the resulting
    // direction would not fit this owner's Signed320 component storage.
    internal static bool AreParallel(in WideAxis3 left, in WideAxis3 right) =>
        WideArithmetic.SubtractSigned704(WideArithmetic.MultiplySigned576ToSigned704(Signed576.ExtendValue(left.Y), right.Z),
            WideArithmetic.MultiplySigned576ToSigned704(Signed576.ExtendValue(left.Z), right.Y)).IsZero
        && WideArithmetic.SubtractSigned704(WideArithmetic.MultiplySigned576ToSigned704(Signed576.ExtendValue(left.Z), right.X),
            WideArithmetic.MultiplySigned576ToSigned704(Signed576.ExtendValue(left.X), right.Z)).IsZero
        && WideArithmetic.SubtractSigned704(WideArithmetic.MultiplySigned576ToSigned704(Signed576.ExtendValue(left.X), right.Y),
            WideArithmetic.MultiplySigned576ToSigned704(Signed576.ExtendValue(left.Y), right.X)).IsZero;

    internal static Signed576 Dot(in WideAxis3 left, in WideAxis3 right) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(left.X, right.X),
                WideArithmetic.MultiplySigned320(left.Y, right.Y)),
            WideArithmetic.MultiplySigned320(left.Z, right.Z));

    public static WideAxis3 operator -(WideAxis3 value) =>
        new(
            WideArithmetic.SubtractSigned320(default, value.X),
            WideArithmetic.SubtractSigned320(default, value.Y),
            WideArithmetic.SubtractSigned320(default, value.Z));
}
