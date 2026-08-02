//=======================================================================
// WideOrientedBox.ConvexPrism.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains exact oriented-box and convex-prism relation calculations.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct RationalAxis3
    {
        internal readonly Signed192 X;
        internal readonly Signed192 Y;
        internal readonly Signed192 Z;

        internal RationalAxis3(Signed192 x, Signed192 y, Signed192 z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        internal WideAxis3 ToWide() =>
            new(
                Signed320.ExtendValue(X),
                Signed320.ExtendValue(Y),
                Signed320.ExtendValue(Z));
    }

    #endregion

    internal static bool TryGetConvexPrismContact(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        WideRationalBasis3d basis = new(orientation);
        var best = default(WidePointSpanPenetration);
        WideAxis3 up = new(default, Signed320.One, default);
        if (!TryKeepConvexPrismAxis(
                up,
                center,
                halfExtents,
                basis,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best))
        {
            contact = default;
            return false;
        }

        Span<RationalAxis3> boxAxes = stackalloc RationalAxis3[3]
        {
            new(basis.Xx, basis.Xy, basis.Xz),
            new(basis.Yx, basis.Yy, basis.Yz),
            new(basis.Zx, basis.Zy, basis.Zz),
        };
        for (int axisIndex = 0; axisIndex < boxAxes.Length; axisIndex++)
        {
            WideAxis3 boxAxis = boxAxes[axisIndex].ToWide();
            if (!TryKeepConvexPrismAxis(
                    boxAxis,
                    center,
                    halfExtents,
                    basis,
                    prismOrigin,
                    prismRotation,
                    prismLocalOffsets,
                    prismHalfThickness,
                    ref best)
                || !TryKeepConvexPrismAxis(
                    CrossWithUp(boxAxis),
                    center,
                    halfExtents,
                    basis,
                    prismOrigin,
                    prismRotation,
                    prismLocalOffsets,
                    prismHalfThickness,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int edgeIndex = 0; edgeIndex < prismLocalOffsets.Length; edgeIndex++)
        {
            Vector2d start = prismLocalOffsets[edgeIndex];
            Vector2d end = prismLocalOffsets[
                edgeIndex + 1 == prismLocalOffsets.Length ? 0 : edgeIndex + 1];
            GetRotatedPlanarEdge(
                prismRotation,
                start,
                end,
                out Signed192 edgeX,
                out Signed192 edgeZ);
            WideAxis3 faceAxis = new(
                Signed320.ExtendValue(edgeZ),
                default,
                Signed320.ExtendValue(
                    WideArithmetic.SubtractSigned192(default, edgeX)));
            if (!TryKeepConvexPrismAxis(
                    faceAxis,
                    center,
                    halfExtents,
                    basis,
                    prismOrigin,
                    prismRotation,
                    prismLocalOffsets,
                    prismHalfThickness,
                    ref best))
            {
                contact = default;
                return false;
            }

            for (int boxAxisIndex = 0; boxAxisIndex < boxAxes.Length; boxAxisIndex++)
            {
                if (!TryKeepConvexPrismAxis(
                        CrossWithPlanarEdge(
                            boxAxes[boxAxisIndex],
                            edgeX,
                            edgeZ),
                        center,
                        halfExtents,
                        basis,
                        prismOrigin,
                        prismRotation,
                        prismLocalOffsets,
                        prismHalfThickness,
                        ref best))
                {
                    contact = default;
                    return false;
                }
            }
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideNormalization.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        Vector3d localBoxPoint = GetLocalAxisSupportPoint(
            basis,
            halfExtents,
            orientedAxis);
        Vector3d prismOffset = GetPrismSupportOffset(
            prismLocalOffsets,
            prismHalfThickness,
            prismRotation,
            orientedAxis);
        Fixed64 depth = best.GetRoundedDepth(out bool depthIsClamped);

        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                center,
                orientation,
                localBoxPoint),
            new FixedPointAnchor(
                prismOrigin,
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Up,
                    -prismRotation),
                prismOffset),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryKeepConvexPrismAxis(
        WideAxis3 axis,
        Vector3d boxCenter,
        Vector3d halfExtents,
        WideRationalBasis3d basis,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        ref WidePointSpanPenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 boxRadius = WideArithmetic.MultiplySigned576(
            GetBoxProjectionRadiusNumerator(
                axis,
                halfExtents,
                basis),
            Signed192.One);
        Signed576 originProjection = WideArithmetic.MultiplySigned576(
            WideRigidProjection.GetWorldOriginDifferenceProjection(
                prismOrigin,
                boxCenter,
                axis),
            Signed192.One);
        Signed576 minimumPlanar = default;
        Signed576 maximumPlanar = default;
        bool hasPlanarProjection = false;
        for (int index = 0; index < prismLocalOffsets.Length; index++)
        {
            Signed576 projection = GetRotatedPlanarOffsetProjection(
                prismLocalOffsets[index],
                prismRotation,
                axis);
            if (!hasPlanarProjection)
            {
                minimumPlanar = projection;
                maximumPlanar = projection;
                hasPlanarProjection = true;
            }
            else
            {
                if (CompareSigned(projection, minimumPlanar) < 0)
                    minimumPlanar = projection;
                if (CompareSigned(projection, maximumPlanar) > 0)
                    maximumPlanar = projection;
            }
        }

        Signed576 verticalRadius = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(GetMagnitude(axis.Y)),
                Signed192.Raw(prismHalfThickness)),
            Signed192.One);
        Signed576 minimum = WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(originProjection, minimumPlanar),
            verticalRadius);
        Signed576 maximum = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(originProjection, maximumPlanar),
            verticalRadius);
        minimum = WideArithmetic.MultiplySigned576(minimum, basis.Denominator);
        maximum = WideArithmetic.MultiplySigned576(maximum, basis.Denominator);
        Signed576 pushBoxNegative = WideArithmetic.SubtractSigned576(
            boxRadius,
            minimum);
        Signed576 pushBoxPositive = WideArithmetic.AddSigned576(
            maximum,
            boxRadius);
        if (pushBoxNegative.Sign < 0 || pushBoxPositive.Sign < 0)
            return false;

        int pushComparison = CompareSigned(
            pushBoxNegative,
            pushBoxPositive);
        bool negate = pushComparison > 0
            || (pushComparison == 0
                && ShouldNegateCanonicalAxis(axis));
        Signed576 overlap = pushComparison <= 0
            ? pushBoxNegative
            : pushBoxPositive;
        Signed320 commonDenominator =
            WideArithmetic.MultiplySigned192(
                basis.Denominator,
                Signed192.One);
        Signed576 squaredAxisLength = axis.SquaredLength;
        if (best.ShouldReplace(
            overlap,
            squaredAxisLength,
            commonDenominator))
        {
            best = new WidePointSpanPenetration(
                axis,
                negate,
                overlap,
                squaredAxisLength,
                commonDenominator);
        }

        return true;
    }

    private static Signed576 GetRotatedPlanarOffsetProjection(
        Vector2d offset,
        Fixed64 rotation,
        WideAxis3 axis)
    {
        WideConvex2dRelations.GetRelativePointNumerators(
            Vector2d.Zero,
            rotation,
            offset,
            Vector2d.Zero,
            out Signed192 x,
            out Signed192 z);
        return WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(x),
                axis.X),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(z),
                axis.Z));
    }

    private static void GetRotatedPlanarEdge(
        Fixed64 rotation,
        Vector2d start,
        Vector2d end,
        out Signed192 x,
        out Signed192 z)
    {
        WideConvex2dRelations.GetRelativePointNumerators(
            Vector2d.Zero,
            rotation,
            end,
            Vector2d.Zero,
            out Signed192 endX,
            out Signed192 endZ);
        WideConvex2dRelations.GetRelativePointNumerators(
            Vector2d.Zero,
            rotation,
            start,
            Vector2d.Zero,
            out Signed192 startX,
            out Signed192 startZ);
        x = WideArithmetic.SubtractSigned192(endX, startX);
        z = WideArithmetic.SubtractSigned192(endZ, startZ);
    }

    private static Vector3d GetLocalAxisSupportPoint(
        WideRationalBasis3d basis,
        Vector3d halfExtents,
        WideAxis3 axis)
    {
        return new Vector3d(
            WideRigidProjection.GetBasisAxisProjection(axis, basis.Xx, basis.Xy, basis.Xz).Sign > 0
                ? halfExtents.X
                : -halfExtents.X,
            WideRigidProjection.GetBasisAxisProjection(axis, basis.Yx, basis.Yy, basis.Yz).Sign > 0
                ? halfExtents.Y
                : -halfExtents.Y,
            WideRigidProjection.GetBasisAxisProjection(axis, basis.Zx, basis.Zy, basis.Zz).Sign > 0
                ? halfExtents.Z
                : -halfExtents.Z);
    }

    private static Vector3d GetPrismSupportOffset(
        ReadOnlySpan<Vector2d> offsets,
        Fixed64 halfThickness,
        Fixed64 rotation,
        WideAxis3 boxToPrismAxis)
    {
        int bestIndex = 0;
        Signed576 bestProjection = GetRotatedPlanarOffsetProjection(
            offsets[0],
            rotation,
            boxToPrismAxis);
        for (int index = 1; index < offsets.Length; index++)
        {
            Signed576 projection = GetRotatedPlanarOffsetProjection(
                offsets[index],
                rotation,
                boxToPrismAxis);
            int projectionComparison = CompareSigned(
                projection,
                bestProjection);
            if (projectionComparison < 0)
            {
                bestProjection = projection;
                bestIndex = index;
            }
            else if (projectionComparison == 0
                && IsLexicographicallyEarlier(offsets[index], offsets[bestIndex]))
            {
                bestIndex = index;
            }
        }

        Vector2d planar = offsets[bestIndex];
        return new Vector3d(
            planar.X,
            boxToPrismAxis.Y.Sign >= 0
                ? -halfThickness
                : halfThickness,
            planar.Y);
    }

    private static bool ShouldNegateCanonicalAxis(WideAxis3 axis)
    {
        if (!axis.X.IsZero)
            return axis.X.Sign < 0;
        if (!axis.Y.IsZero)
            return axis.Y.Sign < 0;
        return axis.Z.Sign < 0;
    }

    private static bool IsLexicographicallyEarlier(Vector2d candidate, Vector2d current) =>
        candidate.X < current.X
        || (candidate.X == current.X && candidate.Y < current.Y);

    private static WideAxis3 CrossWithUp(WideAxis3 axis) =>
        new(
            WideArithmetic.SubtractSigned320(default, axis.Z),
            default,
            axis.X);

    private static WideAxis3 CrossWithPlanarEdge(
        RationalAxis3 axis,
        Signed192 edgeX,
        Signed192 edgeZ) =>
        new(
            WideArithmetic.MultiplySigned192(axis.Y, edgeZ),
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(axis.Z, edgeX),
                WideArithmetic.MultiplySigned192(axis.X, edgeZ)),
            WideArithmetic.SubtractSigned320(
                default,
                WideArithmetic.MultiplySigned192(axis.Y, edgeX)));
}
