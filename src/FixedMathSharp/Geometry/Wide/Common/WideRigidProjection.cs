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
        WideRationalBasis3d basis,
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
        WideRationalBasis3d basis,
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

    internal static Signed576 GetTransformedLocalOffsetProjection(
        Vector3d localOffset,
        WideRationalBasis3d basis,
        WideAxis3 axis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    GetBasisAxisProjection(
                        axis, basis.Xx, basis.Xy, basis.Xz),
                    Signed192.Raw(localOffset.X)),
                WideArithmetic.MultiplySigned576(
                    GetBasisAxisProjection(
                        axis, basis.Yx, basis.Yy, basis.Yz),
                    Signed192.Raw(localOffset.Y))),
            WideArithmetic.MultiplySigned576(
                GetBasisAxisProjection(
                    axis, basis.Zx, basis.Zy, basis.Zz),
                Signed192.Raw(localOffset.Z)));

    internal static Signed576 GetWorldOriginDifferenceProjection(
        Vector3d end,
        Vector3d start,
        WideAxis3 axis) =>
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
        WideAxis3 axis,
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
        Signed576 candidate,
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
}
