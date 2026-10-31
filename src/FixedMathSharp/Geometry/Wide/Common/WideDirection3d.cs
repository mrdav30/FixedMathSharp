//=======================================================================
// WideDirection3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>An exact scaled-hull SAT direction; its magnitude is deliberately arbitrary.</summary>
internal readonly struct WideDirection3d
{
    internal readonly Signed576 X, Y, Z;
    internal WideDirection3d(Signed576 x, Signed576 y, Signed576 z) { X = x; Y = y; Z = z; }
    internal WideDirection3d(Signed320 x, Signed320 y, Signed320 z)
        : this(Signed576.ExtendValue(x), Signed576.ExtendValue(y), Signed576.ExtendValue(z)) { }
    internal WideDirection3d(in WideAxis3 value)
        : this(Signed576.ExtendValue(value.X), Signed576.ExtendValue(value.Y), Signed576.ExtendValue(value.Z)) { }
    internal bool IsZero => X.IsZero && Y.IsZero && Z.IsZero;
    internal Signed832 SquaredLength => WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
        WideArithmetic.MultiplySigned576ToSigned832(X, X), WideArithmetic.MultiplySigned576ToSigned832(Y, Y)),
        WideArithmetic.MultiplySigned576ToSigned832(Z, Z));
    internal Vector3d GetNormal(bool negate) => WideNormalization.GetNormalized(
        negate ? WideArithmetic.SubtractSigned576(default, X) : X,
        negate ? WideArithmetic.SubtractSigned576(default, Y) : Y,
        negate ? WideArithmetic.SubtractSigned576(default, Z) : Z);

    internal static WideDirection3d Cross(in WideDirection3d a, in WideDirection3d b) => new(
        DifferenceProduct(a.Y, b.Z, a.Z, b.Y), DifferenceProduct(a.Z, b.X, a.X, b.Z),
        DifferenceProduct(a.X, b.Y, a.Y, b.X));
    internal static Signed832 Dot(in WideDirection3d axis, in WideAxis3 point) =>
        WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(axis.X, point.X),
            WideArithmetic.MultiplySigned576ToSigned832(axis.Y, point.Y)),
            WideArithmetic.MultiplySigned576ToSigned832(axis.Z, point.Z));
    internal static Signed832 Dot(in WideDirection3d first, in WideDirection3d second) =>
        WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(first.X, second.X),
            WideArithmetic.MultiplySigned576ToSigned832(first.Y, second.Y)),
            WideArithmetic.MultiplySigned576ToSigned832(first.Z, second.Z));
    internal static bool AreParallel(in WideDirection3d first, in WideDirection3d second) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(first.Y, second.Z),
            WideArithmetic.MultiplySigned576ToSigned832(first.Z, second.Y)).IsZero
        && WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(first.Z, second.X),
            WideArithmetic.MultiplySigned576ToSigned832(first.X, second.Z)).IsZero
        && WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(first.X, second.Y),
            WideArithmetic.MultiplySigned576ToSigned832(first.Y, second.X)).IsZero;
    public static WideDirection3d operator -(WideDirection3d value) => new(
        WideArithmetic.SubtractSigned576(default, value.X), WideArithmetic.SubtractSigned576(default, value.Y),
        WideArithmetic.SubtractSigned576(default, value.Z));
    private static Signed576 DifferenceProduct(Signed576 a, Signed576 b, Signed576 c, Signed576 d) =>
        Signed576.NarrowValue(WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(a, b),
            WideArithmetic.MultiplySigned576ToSigned832(c, d)));

    internal static WideDirection3d GetLocalEdge(in ScaledRigidFrame3d frame, Vector3d a, Vector3d b)
    {
        frame.GetScaledLocalPointNumerators(a, out Signed192 ax, out Signed192 ay, out Signed192 az);
        frame.GetScaledLocalPointNumerators(b, out Signed192 bx, out Signed192 by, out Signed192 bz);
        return new WideDirection3d(Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.SubtractSigned192(bx, ax))),
            Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.SubtractSigned192(by, ay))),
            Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.SubtractSigned192(bz, az))));
    }
    internal WideDirection3d Transform(in WideRationalBasis3d basis) => new(
        TransformComponent(basis.Xx, basis.Yx, basis.Zx), TransformComponent(basis.Xy, basis.Yy, basis.Zy),
        TransformComponent(basis.Xz, basis.Yz, basis.Zz));
    private Signed576 TransformComponent(Signed192 x, Signed192 y, Signed192 z) =>
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned576(X, x),
            WideArithmetic.MultiplySigned576(Y, y)), WideArithmetic.MultiplySigned576(Z, z));
}
