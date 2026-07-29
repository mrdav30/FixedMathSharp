//=======================================================================
// WideOrientedBox.CircleSlabProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains high-precision circle/slab projection logic used for continuous
/// collision sweep tests against oriented boxes, including the rational
/// arithmetic types used to represent spatial and planar constraints.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct SweepSpatialConstraint
    {
        internal readonly Signed192 A;
        internal readonly Signed192 B;
        internal readonly Signed192 C;
        internal readonly Signed320 K;

        internal SweepSpatialConstraint(
            Signed192 a,
            Signed192 b,
            Signed192 c,
            Signed320 k)
        {
            A = a;
            B = b;
            C = c;
            K = k;
        }
    }

    private readonly struct SweepPlanarConstraint
    {
        internal readonly Signed320 A;
        internal readonly Signed320 C;
        internal readonly Signed576 K;

        internal SweepPlanarConstraint(
            Signed320 a,
            Signed320 c,
            Signed576 k)
        {
            A = a;
            C = c;
            K = k;
        }
    }

    private readonly struct SweepBoxCorner
    {
        internal readonly Signed320 X;
        internal readonly Signed320 Y;
        internal readonly Signed320 Z;

        internal SweepBoxCorner(
            Signed320 x,
            Signed320 y,
            Signed320 z)
        {
            X = x;
            Y = y;
            Z = z;
        }
    }

    private readonly struct SweepRationalPoint
    {
        internal readonly Signed576 X;
        internal readonly Signed576 Z;
        internal readonly Signed576 Denominator;

        internal SweepRationalPoint(
            Signed576 x,
            Signed576 z,
            Signed576 denominator)
        {
            X = x;
            Z = z;
            Denominator = denominator;
        }
    }

    #endregion

    private static bool TryBuildCircleSlabProjectionConstraints(
        Vector3d center,
        Vector3d halfExtents,
        WideRationalBasis3d basis,
        Signed192 lowerY,
        Signed192 upperY,
        Span<SweepPlanarConstraint> constraints,
        out int count)
    {
        Span<SweepSpatialConstraint> spatial =
            stackalloc SweepSpatialConstraint[8];
        int spatialCount = 0;
        AddBoxAxisConstraints(
            center,
            halfExtents.X,
            basis.Denominator,
            basis.Xx,
            basis.Xy,
            basis.Xz,
            spatial,
            ref spatialCount);
        AddBoxAxisConstraints(
            center,
            halfExtents.Y,
            basis.Denominator,
            basis.Yx,
            basis.Yy,
            basis.Yz,
            spatial,
            ref spatialCount);
        AddBoxAxisConstraints(
            center,
            halfExtents.Z,
            basis.Denominator,
            basis.Zx,
            basis.Zy,
            basis.Zz,
            spatial,
            ref spatialCount);
        spatial[spatialCount++] = new SweepSpatialConstraint(
            default,
            SweepOneCoefficient,
            default,
            Signed320.ExtendValue(upperY));
        spatial[spatialCount++] = new SweepSpatialConstraint(
            default,
            WideArithmetic.SubtractSigned192(
                default,
                SweepOneCoefficient),
            default,
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(default, lowerY)));

        count = 0;
        for (int index = 0; index < spatialCount; index++)
        {
            if (!spatial[index].B.IsZero)
                continue;

            constraints[count++] = new SweepPlanarConstraint(
                Signed320.ExtendValue(spatial[index].A),
                Signed320.ExtendValue(spatial[index].C),
                Signed576.ExtendValue(spatial[index].K));
        }

        for (int positive = 0; positive < spatialCount; positive++)
        {
            if (spatial[positive].B.Sign <= 0)
                continue;

            for (int negative = 0; negative < spatialCount; negative++)
            {
                if (spatial[negative].B.Sign >= 0)
                    continue;

                Signed320 a = WideArithmetic.SubtractSigned320(
                    WideArithmetic.MultiplySigned192(
                        spatial[positive].B,
                        spatial[negative].A),
                    WideArithmetic.MultiplySigned192(
                        spatial[negative].B,
                        spatial[positive].A));
                Signed320 c = WideArithmetic.SubtractSigned320(
                    WideArithmetic.MultiplySigned192(
                        spatial[positive].B,
                        spatial[negative].C),
                    WideArithmetic.MultiplySigned192(
                        spatial[negative].B,
                        spatial[positive].C));
                Signed576 k = WideArithmetic.SubtractSigned576(
                    WideArithmetic.MultiplySigned320(
                        Signed320.ExtendValue(
                            spatial[positive].B),
                        spatial[negative].K),
                    WideArithmetic.MultiplySigned320(
                        Signed320.ExtendValue(
                            spatial[negative].B),
                        spatial[positive].K));
                if (!TryAddProjectionConstraint(
                        new SweepPlanarConstraint(a, c, k),
                        constraints,
                        ref count))
                {
                    return false;
                }
            }
        }

        return count > 0;
    }

    private static void AddBoxAxisConstraints(
        Vector3d center,
        Fixed64 extent,
        Signed192 denominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Span<SweepSpatialConstraint> constraints,
        ref int count)
    {
        Signed320 centerProjection = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(Signed192.Raw(center.X), axisX),
                WideArithmetic.MultiplySigned192(Signed192.Raw(center.Y), axisY)),
            WideArithmetic.MultiplySigned192(Signed192.Raw(center.Z), axisZ));
        Signed320 extentNumerator =
            WideArithmetic.MultiplySigned192(Signed192.Raw(extent), denominator);
        constraints[count++] = new SweepSpatialConstraint(
            axisX,
            axisY,
            axisZ,
            WideArithmetic.AddSigned320(
                extentNumerator,
                centerProjection));
        constraints[count++] = new SweepSpatialConstraint(
            WideArithmetic.SubtractSigned192(default, axisX),
            WideArithmetic.SubtractSigned192(default, axisY),
            WideArithmetic.SubtractSigned192(default, axisZ),
            WideArithmetic.SubtractSigned320(
                extentNumerator,
                centerProjection));
    }

    private static bool TryAddProjectionConstraint(
        SweepPlanarConstraint constraint,
        Span<SweepPlanarConstraint> constraints,
        ref int count)
    {
        if (constraint.A.IsZero && constraint.C.IsZero)
            return constraint.K.Sign >= 0;

        constraints[count++] = constraint;
        return true;
    }

    private static void BuildCircleSlabProjectionVertices(
        Vector3d center,
        Vector3d halfExtents,
        WideRationalBasis3d basis,
        Signed192 lowerY,
        Signed192 upperY,
        Span<SweepRationalPoint> vertices,
        out int count)
    {
        Span<SweepBoxCorner> corners = stackalloc SweepBoxCorner[8];
        Signed320 lowerNumerator =
            WideArithmetic.MultiplySigned192(lowerY, basis.Denominator);
        Signed320 upperNumerator =
            WideArithmetic.MultiplySigned192(upperY, basis.Denominator);
        count = 0;
        for (int index = 0; index < corners.Length; index++)
        {
            Vector3d local = new(
                (index & 1) == 0 ? -halfExtents.X : halfExtents.X,
                (index & 2) == 0 ? -halfExtents.Y : halfExtents.Y,
                (index & 4) == 0 ? -halfExtents.Z : halfExtents.Z);
            corners[index] = GetSweepBoxCorner(center, basis, local);
            if (CompareSigned(corners[index].Y, lowerNumerator) >= 0
                && CompareSigned(corners[index].Y, upperNumerator) <= 0)
            {
                AddUniqueSweepPoint(
                    new SweepRationalPoint(
                        Signed576.ExtendValue(corners[index].X),
                        Signed576.ExtendValue(corners[index].Z),
                        Signed576.ExtendValue(
                            Signed320.ExtendValue(
                                basis.Denominator))),
                    vertices,
                    ref count);
            }
        }

        for (int index = 0; index < corners.Length; index++)
        {
            for (int bit = 1; bit <= 4; bit <<= 1)
            {
                if ((index & bit) != 0)
                    continue;

                TryAddPlaneIntersection(
                    corners[index],
                    corners[index | bit],
                    lowerNumerator,
                    basis.Denominator,
                    vertices,
                    ref count);
                TryAddPlaneIntersection(
                    corners[index],
                    corners[index | bit],
                    upperNumerator,
                    basis.Denominator,
                    vertices,
                    ref count);
            }
        }
    }

    private static SweepBoxCorner GetSweepBoxCorner(
        Vector3d center,
        WideRationalBasis3d basis,
        Vector3d local) =>
        new(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(center.X),
                    basis.Denominator),
                GetLocalOffsetNumerator(
                    basis.Xx,
                    basis.Yx,
                    basis.Zx,
                    local)),
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(center.Y),
                    basis.Denominator),
                GetLocalOffsetNumerator(
                    basis.Xy,
                    basis.Yy,
                    basis.Zy,
                    local)),
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(center.Z),
                    basis.Denominator),
                GetLocalOffsetNumerator(
                    basis.Xz,
                    basis.Yz,
                    basis.Zz,
                    local)));

    private static void TryAddPlaneIntersection(
        SweepBoxCorner first,
        SweepBoxCorner second,
        Signed320 planeNumerator,
        Signed192 basisDenominator,
        Span<SweepRationalPoint> vertices,
        ref int count)
    {
        Signed320 minimum = CompareSigned(first.Y, second.Y) <= 0
            ? first.Y
            : second.Y;
        Signed320 maximum = CompareSigned(first.Y, second.Y) <= 0
            ? second.Y
            : first.Y;
        if (CompareSigned(planeNumerator, minimum) < 0
            || CompareSigned(planeNumerator, maximum) > 0)
        {
            return;
        }

        Signed320 deltaY =
            WideArithmetic.SubtractSigned320(second.Y, first.Y);
        if (deltaY.IsZero)
            return;

        Signed320 firstWeight =
            WideArithmetic.SubtractSigned320(second.Y, planeNumerator);
        Signed320 secondWeight =
            WideArithmetic.SubtractSigned320(planeNumerator, first.Y);
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(first.X, firstWeight),
            WideArithmetic.MultiplySigned320(second.X, secondWeight));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(first.Z, firstWeight),
            WideArithmetic.MultiplySigned320(second.Z, secondWeight));
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(basisDenominator),
            deltaY);
        if (denominator.Sign < 0)
        {
            x = WideArithmetic.SubtractSigned576(default, x);
            z = WideArithmetic.SubtractSigned576(default, z);
            denominator =
                WideArithmetic.SubtractSigned576(default, denominator);
        }

        AddUniqueSweepPoint(
            new SweepRationalPoint(x, z, denominator),
            vertices,
            ref count);
    }

    private static void AddUniqueSweepPoint(
        SweepRationalPoint point,
        Span<SweepRationalPoint> vertices,
        ref int count)
    {
        for (int index = 0; index < count; index++)
        {
            if (AreSameSweepPoint(vertices[index], point))
                return;
        }

        vertices[count++] = point;
    }

    private static bool AreSameSweepPoint(
        SweepRationalPoint first,
        SweepRationalPoint second) =>
        WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                first.X,
                second.Denominator),
            WideArithmetic.MultiplySigned576ToSigned832(
                second.X,
                first.Denominator)).IsZero
        && WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                first.Z,
                second.Denominator),
            WideArithmetic.MultiplySigned576ToSigned832(
                second.Z,
                first.Denominator)).IsZero;
}
