//=======================================================================
// WideConvexPrismRelations.SweptUprightCylinder.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact joint planar and vertical relations for swept upright cylinders.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private readonly struct SweepPlanarPath
    {
        internal readonly Signed192 StartX;
        internal readonly Signed192 StartZ;
        internal readonly Signed192 VelocityX;
        internal readonly Signed192 VelocityZ;

        internal SweepPlanarPath(
            Vector3d start,
            Vector3d end,
            Vector3d prismOrigin)
        {
            StartX = WideArithmetic.SubtractSigned192(
                WideArithmetic.Scale(start.X),
                WideArithmetic.Scale(prismOrigin.X));
            StartZ = WideArithmetic.SubtractSigned192(
                WideArithmetic.Scale(start.Z),
                WideArithmetic.Scale(prismOrigin.Z));
            Signed192 endX = WideArithmetic.SubtractSigned192(
                WideArithmetic.Scale(end.X),
                WideArithmetic.Scale(prismOrigin.X));
            Signed192 endZ = WideArithmetic.SubtractSigned192(
                WideArithmetic.Scale(end.Z),
                WideArithmetic.Scale(prismOrigin.Z));
            VelocityX = WideArithmetic.SubtractSigned192(endX, StartX);
            VelocityZ = WideArithmetic.SubtractSigned192(endZ, StartZ);
        }
    }

    private struct SweepParameterDomain
    {
        internal Signed320 LowerNumerator;
        internal Signed320 LowerDenominator;
        internal Signed320 UpperNumerator;
        internal Signed320 UpperDenominator;
        internal bool IsEmpty;

        internal static SweepParameterDomain Unit => new()
        {
            LowerDenominator = Signed320.One,
            UpperNumerator = Signed320.One,
            UpperDenominator = Signed320.One,
        };

        internal bool ClipStrictLinear(
            Signed320 constant,
            Signed320 velocity)
        {
            if (IsEmpty)
                return false;
            if (velocity.Sign > 0)
            {
                KeepLower(
                    WideArithmetic.Negate(constant),
                    velocity);
            }
            else if (velocity.Sign < 0)
            {
                KeepUpper(
                    constant,
                    WideArithmetic.Negate(velocity));
            }
            else
            {
                IsEmpty = constant.Sign <= 0;
            }

            return !IsEmpty;
        }

        private void KeepLower(
            Signed320 numerator,
            Signed320 denominator)
        {
            int comparison = CompareRatios(
                numerator,
                denominator,
                LowerNumerator,
                LowerDenominator);
            if (comparison > 0)
            {
                LowerNumerator = numerator;
                LowerDenominator = denominator;
            }

            ValidateOrder();
        }

        private void KeepUpper(
            Signed320 numerator,
            Signed320 denominator)
        {
            int comparison = CompareRatios(
                numerator,
                denominator,
                UpperNumerator,
                UpperDenominator);
            if (comparison < 0)
            {
                UpperNumerator = numerator;
                UpperDenominator = denominator;
            }

            ValidateOrder();
        }

        private void ValidateOrder()
        {
            int comparison = CompareRatios(
                LowerNumerator,
                LowerDenominator,
                UpperNumerator,
                UpperDenominator);
            IsEmpty = comparison >= 0;
        }
    }

    internal static bool IntersectsSweptUprightCylinderStrict(
        Vector3d bottomStart,
        Vector3d bottomEnd,
        Fixed64 radius,
        Fixed64 height,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness)
    {
        SweepParameterDomain verticalDomain = SweepParameterDomain.Unit;
        if (!ClipStrictVerticalOverlap(
                bottomStart.Y,
                bottomEnd.Y,
                height,
                prismOrigin.Y,
                prismHalfThickness,
                ref verticalDomain))
        {
            return false;
        }

        int winding = GetSweepFootprintWinding(prismLocalOffsets);
        if (winding == 0)
            return false;
        return IntersectsStrictPlanarSweep(
            new SweepPlanarPath(
                bottomStart,
                bottomEnd,
                prismOrigin),
            new WideConvex2dRelations.RotationFrame2d(prismRotation),
            radius,
            prismLocalOffsets,
            winding,
            verticalDomain);
    }

    private static bool ClipStrictVerticalOverlap(
        Fixed64 bottomStart,
        Fixed64 bottomEnd,
        Fixed64 height,
        Fixed64 prismCenter,
        Fixed64 prismHalfThickness,
        ref SweepParameterDomain domain)
    {
        Signed192 velocity = WideArithmetic.Difference(
            bottomEnd,
            bottomStart);
        Signed192 topAboveLower = WideArithmetic.AddSigned192(
            WideArithmetic.SubtractSigned192(
                WideArithmetic.AddSigned192(
                    Signed192.Raw(bottomStart),
                    Signed192.Raw(height)),
                Signed192.Raw(prismCenter)),
            Signed192.Raw(prismHalfThickness));
        if (!domain.ClipStrictLinear(
                Signed320.ExtendValue(topAboveLower),
                Signed320.ExtendValue(velocity)))
        {
            return false;
        }

        Signed192 upperAboveBottom = WideArithmetic.SubtractSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Raw(prismCenter),
                Signed192.Raw(prismHalfThickness)),
            Signed192.Raw(bottomStart));
        return domain.ClipStrictLinear(
            Signed320.ExtendValue(upperAboveBottom),
            Signed320.ExtendValue(WideArithmetic.Negate(velocity)));
    }

    private static bool IntersectsStrictPlanarSweep(
        SweepPlanarPath path,
        WideConvex2dRelations.RotationFrame2d rotation,
        Fixed64 radius,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        int winding,
        SweepParameterDomain verticalDomain)
    {
        Signed192 scaledRadius = WideArithmetic.Scale(radius);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            scaledRadius,
            scaledRadius);
        SweepParameterDomain centerlineDomain = verticalDomain;
        for (int index = 0; index < prismLocalOffsets.Length; index++)
        {
            GetSweepEdge(
                path,
                rotation,
                prismLocalOffsets[index],
                prismLocalOffsets[
                    index + 1 == prismLocalOffsets.Length ? 0 : index + 1],
                out Signed192 pointX,
                out Signed192 pointZ,
                out Signed192 velocityX,
                out Signed192 velocityZ,
                out Signed192 edgeX,
                out Signed192 edgeZ);
            if (radius > Fixed64.Zero
                && (IntersectsOpenCircle(
                        pointX,
                        pointZ,
                        velocityX,
                        velocityZ,
                        radiusSquared,
                        verticalDomain)
                    || (!(edgeX.IsZero && edgeZ.IsZero)
                        && IntersectsOpenEdgeStrip(
                            pointX,
                            pointZ,
                            velocityX,
                            velocityZ,
                            edgeX,
                            edgeZ,
                            radiusSquared,
                            verticalDomain))))
            {
                return true;
            }
            if (centerlineDomain.IsEmpty
                || (edgeX.IsZero && edgeZ.IsZero))
                continue;

            Signed320 constant = Cross(
                edgeX,
                edgeZ,
                pointX,
                pointZ);
            Signed320 velocity = Cross(
                edgeX,
                edgeZ,
                velocityX,
                velocityZ);
            if (winding < 0)
            {
                constant = WideArithmetic.Negate(constant);
                velocity = WideArithmetic.Negate(velocity);
            }
            _ = centerlineDomain.ClipStrictLinear(
                constant,
                velocity);
        }

        return !centerlineDomain.IsEmpty;
    }

    private static bool IntersectsOpenCircle(
        Signed192 pointX,
        Signed192 pointZ,
        Signed192 velocityX,
        Signed192 velocityZ,
        Signed320 radiusSquared,
        SweepParameterDomain domain)
    {
        Signed320 a = Dot(
            velocityX,
            velocityZ,
            velocityX,
            velocityZ);
        Signed320 b = Dot(
            pointX,
            pointZ,
            velocityX,
            velocityZ);
        Signed320 c = WideArithmetic.SubtractSigned320(
            Dot(pointX, pointZ, pointX, pointZ),
            radiusSquared);
        return HasStrictNegativeQuadratic(
            Signed576.ExtendValue(a),
            Signed576.ExtendValue(b),
            Signed576.ExtendValue(c),
            domain);
    }

    private static bool IntersectsOpenEdgeStrip(
        Signed192 pointX,
        Signed192 pointZ,
        Signed192 velocityX,
        Signed192 velocityZ,
        Signed192 edgeX,
        Signed192 edgeZ,
        Signed320 radiusSquared,
        SweepParameterDomain domain)
    {
        Signed320 edgeSquared = Dot(
            edgeX,
            edgeZ,
            edgeX,
            edgeZ);
        Signed320 projection = Dot(
            pointX,
            pointZ,
            edgeX,
            edgeZ);
        Signed320 projectionVelocity = Dot(
            velocityX,
            velocityZ,
            edgeX,
            edgeZ);
        if (!domain.ClipStrictLinear(projection, projectionVelocity)
            || !domain.ClipStrictLinear(
                WideArithmetic.SubtractSigned320(
                    edgeSquared,
                    projection),
                WideArithmetic.Negate(projectionVelocity)))
        {
            return false;
        }

        Signed320 cross = Cross(
            edgeX,
            edgeZ,
            pointX,
            pointZ);
        Signed320 crossVelocity = Cross(
            edgeX,
            edgeZ,
            velocityX,
            velocityZ);
        Signed576 a = WideArithmetic.MultiplySigned320(
            crossVelocity,
            crossVelocity);
        Signed576 b = WideArithmetic.MultiplySigned320(
            cross,
            crossVelocity);
        Signed576 c = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(cross, cross),
            WideArithmetic.MultiplySigned320(
                radiusSquared,
                edgeSquared));
        return HasStrictNegativeQuadratic(a, b, c, domain);
    }

    private static bool HasStrictNegativeQuadratic(
        Signed576 a,
        Signed576 b,
        Signed576 c,
        SweepParameterDomain domain)
    {
        if (domain.IsEmpty)
            return false;
        if (a.IsZero || b.Sign >= 0)
        {
            return EvaluateQuadratic(
                a,
                b,
                c,
                domain.LowerNumerator,
                domain.LowerDenominator).Sign < 0;
        }

        Signed576 vertexNumerator = WideArithmetic.SubtractSigned576(
            default,
            b);
        int lowerComparison = CompareRatioToBound(
            vertexNumerator,
            a,
            domain.LowerNumerator,
            domain.LowerDenominator);
        int upperComparison = CompareRatioToBound(
            vertexNumerator,
            a,
            domain.UpperNumerator,
            domain.UpperDenominator);
        if (lowerComparison > 0 && upperComparison < 0)
        {
            Signed832 discriminant = WideArithmetic.SubtractSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(b, b),
                WideArithmetic.MultiplySigned576ToSigned832(a, c));
            return discriminant.Sign > 0;
        }

        bool useLower = lowerComparison <= 0;
        return EvaluateQuadratic(
            a,
            b,
            c,
            useLower
                ? domain.LowerNumerator
                : domain.UpperNumerator,
            useLower
                ? domain.LowerDenominator
                : domain.UpperDenominator).Sign < 0;
    }

    private static Signed832 EvaluateQuadratic(
        Signed576 a,
        Signed576 b,
        Signed576 c,
        Signed320 numerator,
        Signed320 denominator)
    {
        // Reachable full-domain products and their signed three-term sum stay
        // below 793 bits, leaving more than 38 signed headroom bits in Signed832.
        Signed576 numeratorSquared = WideArithmetic.MultiplySigned320(
            numerator,
            numerator);
        Signed576 numeratorDenominator = WideArithmetic.MultiplySigned320(
            numerator,
            denominator);
        Signed576 denominatorSquared = WideArithmetic.MultiplySigned320(
            denominator,
            denominator);
        Signed832 linear = WideArithmetic.MultiplySigned576ToSigned832(
            b,
            numeratorDenominator);
        return WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(
                    a,
                    numeratorSquared),
                WideArithmetic.AddSigned832(linear, linear)),
            WideArithmetic.MultiplySigned576ToSigned832(
                c,
                denominatorSquared));
    }

    private static int CompareRatioToBound(
        Signed576 numerator,
        Signed576 denominator,
        Signed320 boundNumerator,
        Signed320 boundDenominator) =>
        WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                numerator,
                boundDenominator),
            WideArithmetic.MultiplySigned576ToSigned832(
                denominator,
                boundNumerator)).Sign;

    private static int CompareRatios(
        Signed320 firstNumerator,
        Signed320 firstDenominator,
        Signed320 secondNumerator,
        Signed320 secondDenominator) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                firstNumerator,
                secondDenominator),
            WideArithmetic.MultiplySigned320(
                secondNumerator,
                firstDenominator)).Sign;

    private static int GetSweepFootprintWinding(
        ReadOnlySpan<Vector2d> offsets)
    {
        for (int index = 0; index < offsets.Length; index++)
        {
            Vector2d first = offsets[index];
            Vector2d second = offsets[
                index + 1 == offsets.Length ? 0 : index + 1];
            Vector2d third = offsets[
                index + 2 >= offsets.Length
                    ? index + 2 - offsets.Length
                    : index + 2];
            Signed320 cross = Cross(
                WideArithmetic.Difference(second.X, first.X),
                WideArithmetic.Difference(second.Y, first.Y),
                WideArithmetic.Difference(third.X, second.X),
                WideArithmetic.Difference(third.Y, second.Y));
            if (!cross.IsZero)
                return cross.Sign;
        }

        return 0;
    }

    private static void GetSweepEdge(
        SweepPlanarPath path,
        WideConvex2dRelations.RotationFrame2d rotation,
        Vector2d edgeStart,
        Vector2d edgeEnd,
        out Signed192 pointX,
        out Signed192 pointZ,
        out Signed192 velocityX,
        out Signed192 velocityZ,
        out Signed192 edgeX,
        out Signed192 edgeZ)
    {
        WideConvex2dRelations.GetRotatedOffset(
            rotation,
            edgeStart,
            out Signed192 startX,
            out Signed192 startZ);
        WideConvex2dRelations.GetRotatedOffset(
            rotation,
            edgeEnd,
            out Signed192 endX,
            out Signed192 endZ);
        pointX = WideArithmetic.SubtractSigned192(path.StartX, startX);
        pointZ = WideArithmetic.SubtractSigned192(path.StartZ, startZ);
        velocityX = path.VelocityX;
        velocityZ = path.VelocityZ;
        edgeX = WideArithmetic.SubtractSigned192(
            endX,
            startX);
        edgeZ = WideArithmetic.SubtractSigned192(
            endZ,
            startZ);
    }

    private static Signed320 Cross(
        Signed192 firstX,
        Signed192 firstZ,
        Signed192 secondX,
        Signed192 secondZ) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(firstX, secondZ),
            WideArithmetic.MultiplySigned192(firstZ, secondX));

    private static Signed320 Dot(
        Signed192 firstX,
        Signed192 firstZ,
        Signed192 secondX,
        Signed192 secondZ) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(firstX, secondX),
            WideArithmetic.MultiplySigned192(firstZ, secondZ));
}
