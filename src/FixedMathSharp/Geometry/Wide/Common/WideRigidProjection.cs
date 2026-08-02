//=======================================================================
// WideRigidProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns exact rigid-axis transforms and projection arithmetic shared by wide
/// geometry relations.
/// </summary>
internal static class WideRigidProjection
{
    internal static WideAxis3 TransformLocalAxis(
        in WideRationalBasis3d basis,
        Signed192 localX,
        Signed192 localY,
        Signed192 localZ) =>
        new(
            TransformLocalAxisComponent(
                localX, localY, localZ,
                basis.Xx, basis.Yx, basis.Zx),
            TransformLocalAxisComponent(
                localX, localY, localZ,
                basis.Xy, basis.Yy, basis.Zy),
            TransformLocalAxisComponent(
                localX, localY, localZ,
                basis.Xz, basis.Yz, basis.Zz));

    /// <summary>
    /// Transforms a local triangle normal-cross-edge axis. The local components
    /// must come from that exact construction; its proven rigid-triangle bound
    /// keeps each transformed component within <see cref="Signed320"/>.
    /// </summary>
    internal static WideAxis3 TransformLocalTriangleNormalCrossEdgeAxis(
        in WideRationalBasis3d basis,
        Signed320 localX,
        Signed320 localY,
        Signed320 localZ) =>
        new(
            TransformLocalTriangleNormalCrossEdgeComponent(
                localX, localY, localZ,
                basis.Xx, basis.Yx, basis.Zx),
            TransformLocalTriangleNormalCrossEdgeComponent(
                localX, localY, localZ,
                basis.Xy, basis.Yy, basis.Zy),
            TransformLocalTriangleNormalCrossEdgeComponent(
                localX, localY, localZ,
                basis.Xz, basis.Yz, basis.Zz));

    internal static void GetLocalAxisProjections(
        in WideAxis3 axis,
        in WideRationalBasis3d basis,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z)
    {
        x = GetBasisAxisProjection(
            axis,
            basis.Xx,
            basis.Xy,
            basis.Xz);
        y = GetBasisAxisProjection(
            axis,
            basis.Yx,
            basis.Yy,
            basis.Yz);
        z = GetBasisAxisProjection(
            axis,
            basis.Zx,
            basis.Zy,
            basis.Zz);
    }

    internal static Signed576 GetLocalOffsetProjection(
        Vector3d localOffset,
        in Signed576 axisX,
        in Signed576 axisY,
        in Signed576 axisZ) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    axisX,
                    localOffset.X.m_rawValue),
                WideArithmetic.MultiplySigned576(
                    axisY,
                    localOffset.Y.m_rawValue)),
            WideArithmetic.MultiplySigned576(
                axisZ,
                localOffset.Z.m_rawValue));

    internal static void TransformLocalAxis(
        in WideRationalBasis3d basis,
        in WideAxis3 localAxis,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z)
    {
        x = TransformLocalAxisComponent(
            localAxis,
            basis.Xx,
            basis.Yx,
            basis.Zx);
        y = TransformLocalAxisComponent(
            localAxis,
            basis.Xy,
            basis.Yy,
            basis.Zy);
        z = TransformLocalAxisComponent(
            localAxis,
            basis.Xz,
            basis.Yz,
            basis.Zz);
    }

    internal static Signed576 GetWorldOriginDifferenceProjection(
        Vector3d end,
        Vector3d start,
        in WideAxis3 axis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(WideArithmetic.SubtractSigned192(
                        Signed192.Raw(end.X),
                        Signed192.Raw(start.X))),
                    axis.X),
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(WideArithmetic.SubtractSigned192(
                        Signed192.Raw(end.Y),
                        Signed192.Raw(start.Y))),
                    axis.Y)),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.Z),
                    Signed192.Raw(start.Z))),
                axis.Z));

    internal static Signed576 GetBasisAxisProjection(
        in WideAxis3 axis,
        Signed192 basisX,
        Signed192 basisY,
        Signed192 basisZ) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(axis.X),
                    basisX),
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(axis.Y),
                    basisY)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(axis.Z),
                basisZ));

    internal static void IncludeProjection(
        in Signed576 candidate,
        ref Signed576 minimum,
        ref Signed576 maximum)
    {
        if (WideArithmetic.SubtractSigned576(candidate, minimum).Sign < 0)
            minimum = candidate;
        if (WideArithmetic.SubtractSigned576(candidate, maximum).Sign > 0)
            maximum = candidate;
    }

    private static Signed320 TransformLocalAxisComponent(
        Signed192 localX,
        Signed192 localY,
        Signed192 localZ,
        Signed192 basisX,
        Signed192 basisY,
        Signed192 basisZ) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(localX, basisX),
                WideArithmetic.MultiplySigned192(localY, basisY)),
            WideArithmetic.MultiplySigned192(localZ, basisZ));

    private static Signed320 TransformLocalTriangleNormalCrossEdgeComponent(
        Signed320 localX,
        Signed320 localY,
        Signed320 localZ,
        Signed192 basisX,
        Signed192 basisY,
        Signed192 basisZ)
    {
        Signed576 transformed = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(localX),
                    basisX),
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(localY),
                    basisY)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(localZ),
                basisZ));
        return Signed320.NarrowValue(transformed);
    }

    private static Signed576 TransformLocalAxisComponent(
        in WideAxis3 localAxis,
        Signed192 basisX,
        Signed192 basisY,
        Signed192 basisZ) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(localAxis.X),
                    basisX),
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(localAxis.Y),
                    basisY)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(localAxis.Z),
                basisZ));
}
