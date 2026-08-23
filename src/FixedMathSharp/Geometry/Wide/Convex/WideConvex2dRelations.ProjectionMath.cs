//=======================================================================
// WideConvex2dRelations.ProjectionMath.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides high-precision projection math used for support feature
/// resolution and axis projection comparisons in convex 2D queries.
/// </content>
internal static partial class WideConvex2dRelations
{
    private static void GetSupportFeature(
        Vector2d origin,
        RotationFrame2d rotation,
        ReadOnlySpan<Vector2d> offsets,
        WideAxis2d axis,
        bool maximum,
        out Feature feature)
    {
        int bestIndex = 0;
        Signed320 bestProjection = GetProjection(
            origin,
            rotation,
            offsets[0],
            axis);
        for (int i = 1; i < offsets.Length; i++)
        {
            Signed320 projection = GetProjection(
                origin,
                rotation,
                offsets[i],
                axis);
            int comparison = WideArithmetic.SubtractSigned320(
                projection,
                bestProjection).Sign;
            if ((maximum && comparison > 0)
                || (!maximum && comparison < 0))
            {
                bestProjection = projection;
                bestIndex = i;
            }
        }

        int previousIndex =
            (bestIndex + offsets.Length - 1) % offsets.Length;
        if (WideArithmetic.SubtractSigned320(
                GetProjection(
                    origin,
                    rotation,
                    offsets[previousIndex],
                    axis),
                bestProjection).IsZero)
        {
            feature = new Feature(
                offsets[previousIndex],
                offsets[bestIndex]);
            return;
        }

        int nextIndex = bestIndex + 1 == offsets.Length ? 0 : bestIndex + 1;
        feature = WideArithmetic.SubtractSigned320(
                GetProjection(
                    origin,
                    rotation,
                    offsets[nextIndex],
                    axis),
                bestProjection).IsZero
            ? new Feature(offsets[bestIndex], offsets[nextIndex])
            : new Feature(offsets[bestIndex], offsets[bestIndex]);
    }

    private static void GetProjectionRange(
        Vector2d origin,
        RotationFrame2d rotation,
        ReadOnlySpan<Vector2d> offsets,
        WideAxis2d axis,
        out Signed320 minimum,
        out Signed320 maximum)
    {
        minimum = GetProjection(
            origin,
            rotation,
            offsets[0],
            axis);
        maximum = minimum;
        for (int i = 1; i < offsets.Length; i++)
        {
            Signed320 projection = GetProjection(
                origin,
                rotation,
                offsets[i],
                axis);
            if (WideArithmetic.SubtractSigned320(
                    projection,
                    minimum).Sign < 0)
            {
                minimum = projection;
            }
            if (WideArithmetic.SubtractSigned320(
                    projection,
                    maximum).Sign > 0)
            {
                maximum = projection;
            }
        }
    }

    private static Signed320 GetProjection(
        Vector2d origin,
        RotationFrame2d rotation,
        Vector2d offset,
        WideAxis2d axis)
    {
        GetWorldPoint(
            origin,
            rotation,
            offset,
            Vector2d.Zero,
            out Signed192 x,
            out Signed192 y);
        return WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(x, axis.X),
            WideArithmetic.MultiplySigned192(y, axis.Y));
    }

    private static Signed320 GetSquaredDistance(
        Vector2d firstOrigin,
        Vector2d firstOffset,
        Vector2d secondOrigin,
        RotationFrame2d secondRotation,
        Vector2d secondOffset)
    {
        GetWorldPoint(
            firstOrigin,
            RotationFrame2d.Identity,
            firstOffset,
            Vector2d.Zero,
            out Signed192 firstX,
            out Signed192 firstY);
        GetWorldPoint(
            secondOrigin,
            secondRotation,
            secondOffset,
            Vector2d.Zero,
            out Signed192 secondX,
            out Signed192 secondY);
        Signed192 x = WideArithmetic.SubtractSigned192(firstX, secondX);
        Signed192 y = WideArithmetic.SubtractSigned192(firstY, secondY);
        return WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(x, x),
            WideArithmetic.MultiplySigned192(y, y));
    }

    private static void AddProjectedEndpoint(
        Vector2d sourceOrigin,
        RotationFrame2d sourceRotation,
        Vector2d sourceOffset,
        Vector2d targetOrigin,
        RotationFrame2d targetRotation,
        Feature targetFeature,
        bool sourceIsFirst,
        Span<Vector2d> firstContactOffsets,
        Span<Vector2d> secondContactOffsets,
        ref int contactCount)
    {
        if (contactCount == 2
            || !TryProjectPointOntoFeature(
                sourceOrigin,
                sourceRotation,
                sourceOffset,
                Vector2d.Zero,
                Vector2d.Zero,
                targetOrigin,
                targetRotation,
                targetFeature.Start,
                targetFeature.End,
                requireInteriorProjection: true,
                out Vector2d targetOffset))
        {
            return;
        }

        Vector2d firstOffset = sourceIsFirst ? sourceOffset : targetOffset;
        Vector2d secondOffset = sourceIsFirst ? targetOffset : sourceOffset;
        firstContactOffsets[contactCount] = firstOffset;
        secondContactOffsets[contactCount] = secondOffset;
        contactCount++;
    }

    private static bool TryProjectPointOntoFeature(
        Vector2d sourceOrigin,
        Vector2d sourceOffset,
        Vector2d targetOrigin,
        RotationFrame2d targetRotation,
        Vector2d featureStart,
        Vector2d featureEnd,
        bool requireInteriorProjection,
        out Vector2d targetOffset)
    {
        return TryProjectPointOntoFeature(
            sourceOrigin,
            RotationFrame2d.Identity,
            sourceOffset,
            Vector2d.Zero,
            Vector2d.Zero,
            targetOrigin,
            targetRotation,
            featureStart,
            featureEnd,
            requireInteriorProjection,
            out targetOffset);
    }

    private static bool TryProjectPointOntoFeature(
        Vector2d sourceOrigin,
        RotationFrame2d sourceRotation,
        Vector2d firstSourceOffset,
        Vector2d secondSourceOffset,
        Vector2d sourceTranslationOffset,
        Vector2d targetOrigin,
        RotationFrame2d targetRotation,
        Vector2d featureStart,
        Vector2d featureEnd,
        bool requireInteriorProjection,
        out Vector2d targetOffset)
    {
        GetTransformedEdge(
            targetRotation,
            featureStart,
            featureEnd,
            out Signed192 edgeX,
            out Signed192 edgeY);
        Signed320 denominator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(edgeX, edgeX),
            WideArithmetic.MultiplySigned192(edgeY, edgeY));
        if (denominator.IsZero)
        {
            targetOffset = featureStart;
            return true;
        }

        GetWorldPoint(
            sourceOrigin,
            sourceTranslationOffset,
            sourceRotation,
            firstSourceOffset,
            secondSourceOffset,
            out Signed192 sourceWorldX,
            out Signed192 sourceWorldY);
        GetWorldPoint(
            targetOrigin,
            targetRotation,
            featureStart,
            Vector2d.Zero,
            out Signed192 featureWorldX,
            out Signed192 featureWorldY);
        Signed192 sourceX =
            WideArithmetic.SubtractSigned192(
                sourceWorldX,
                featureWorldX);
        Signed192 sourceY =
            WideArithmetic.SubtractSigned192(
                sourceWorldY,
                featureWorldY);
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(sourceX, edgeX),
            WideArithmetic.MultiplySigned192(sourceY, edgeY));
        if (numerator.Sign < 0)
        {
            if (requireInteriorProjection)
            {
                targetOffset = default;
                return false;
            }
            numerator = default;
        }
        else if (WideArithmetic.SubtractSigned320(
                     numerator,
                     denominator).Sign > 0)
        {
            if (requireInteriorProjection)
            {
                targetOffset = default;
                return false;
            }
            numerator = denominator;
        }

        targetOffset = new Vector2d(
            GetProjectedOffsetCoordinate(
                featureStart.X,
                WideArithmetic.Difference(featureEnd.X, featureStart.X),
                numerator,
                denominator),
            GetProjectedOffsetCoordinate(
                featureStart.Y,
                WideArithmetic.Difference(featureEnd.Y, featureStart.Y),
                numerator,
                denominator));
        return true;
    }

    private static Fixed64 GetProjectedOffsetCoordinate(
        Fixed64 featureStart,
        Signed192 edge,
        Signed320 parameterNumerator,
        Signed320 parameterDenominator)
    {
        Signed576 numerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(Signed192.Raw(featureStart)),
                parameterDenominator),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(edge),
                parameterNumerator));
        // A clamped interpolation of two representable local coordinates is
        // itself representable; only the wide intermediate needs protection.
        _ = Fixed64.TryGetSignedRawRatio(
            numerator,
            Signed576.ExtendValue(parameterDenominator),
            out Fixed64 coordinate);
        return coordinate;
    }

    private static void GetDepth(
        Signed320 overlap,
        Signed320 axisSquared,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Signed320 scaledLength =
            WideArithmetic.GetFloorSquareRootScaledByFixed64(
                Signed576.ExtendValue(axisSquared));
        if (!Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(overlap),
                Signed576.ExtendValue(scaledLength),
                out depth))
        {
            depth = Fixed64.MaxValue;
            depthIsClamped = true;
            return;
        }

        CorrectDepth(overlap, axisSquared, ref depth);
        depthIsClamped = false;
    }

    private static void CorrectDepth(
        Signed320 overlap,
        Signed320 axisSquared,
        ref Fixed64 depth)
    {
        if (depth == Fixed64.Zero)
            return;

        Signed192 lowerMidpoint = WideArithmetic.SubtractSigned192(
            WideArithmetic.AddSigned192(Signed192.Raw(depth), Signed192.Raw(depth)),
            Signed192.Signed(1L));
        int comparison = CompareDepthToTwiceRaw(
            overlap,
            axisSquared,
            lowerMidpoint);
        long adjustment = (uint)comparison >> 31;
        depth = Fixed64.FromRaw(depth.m_rawValue - adjustment);
    }

    private static int CompareDepthToTwiceRaw(
        Signed320 overlap,
        Signed320 axisSquared,
        Signed192 twiceRaw)
    {
        Signed320 twiceOverlap = WideArithmetic.AddSigned320(
            overlap,
            overlap);
        Signed576 left = WideArithmetic.MultiplySigned320(
            twiceOverlap,
            twiceOverlap);
        Signed320 thresholdSquared = WideArithmetic.MultiplySigned192(
            twiceRaw,
            twiceRaw);
        Signed576 unscaledRight = WideArithmetic.MultiplySigned320(
            thresholdSquared,
            axisSquared);
        Signed576 scaleSquared = WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(Signed192.One),
            Signed320.ExtendValue(Signed192.One));
        Signed832 right = WideArithmetic.MultiplySigned576ToSigned832(
            unscaledRight,
            scaleSquared);
        return WideArithmetic.SubtractSigned832(
            Signed832.ExtendValue(left),
            right).Sign;
    }

    private static void GetTransformedEdge(
        RotationFrame2d rotation,
        Vector2d start,
        Vector2d end,
        out Signed192 x,
        out Signed192 y)
    {
        GetRotatedOffset(
            rotation,
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(end.X),
                Signed192.Raw(start.X)),
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(end.Y),
                Signed192.Raw(start.Y)),
            out x,
            out y);
    }

    private static void GetWorldPoint(
        Vector2d origin,
        RotationFrame2d rotation,
        Vector2d firstLocalPoint,
        Vector2d secondLocalPoint,
        out Signed192 x,
        out Signed192 y) =>
        GetWorldPoint(
            origin,
            Vector2d.Zero,
            rotation,
            firstLocalPoint,
            secondLocalPoint,
            out x,
            out y);

    private static void GetWorldPoint(
        Vector2d origin,
        Vector2d translationOffset,
        RotationFrame2d rotation,
        Vector2d firstLocalPoint,
        Vector2d secondLocalPoint,
        out Signed192 x,
        out Signed192 y)
    {
        GetRotatedOffset(
            rotation,
            firstLocalPoint,
            out Signed192 firstRotatedX,
            out Signed192 firstRotatedY);
        GetRotatedOffset(
            rotation,
            secondLocalPoint,
            out Signed192 secondRotatedX,
            out Signed192 secondRotatedY);
        x = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                WideArithmetic.AddSigned192(
                    WideArithmetic.Scale(origin.X),
                    WideArithmetic.Scale(translationOffset.X)),
                firstRotatedX),
            secondRotatedX);
        y = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                WideArithmetic.AddSigned192(
                    WideArithmetic.Scale(origin.Y),
                    WideArithmetic.Scale(translationOffset.Y)),
                firstRotatedY),
            secondRotatedY);
    }

    internal static void GetRotatedOffset(
        RotationFrame2d rotation,
        Vector2d localPoint,
        out Signed192 x,
        out Signed192 y)
    {
        GetRotatedOffset(
            rotation,
            Signed192.Raw(localPoint.X),
            Signed192.Raw(localPoint.Y),
            out x,
            out y);
    }

    private static void GetRotatedOffset(
        RotationFrame2d rotation,
        Signed192 localX,
        Signed192 localY,
        out Signed192 x,
        out Signed192 y)
    {
        x = Signed192.NarrowValue(
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(
                    localX,
                    rotation.Cosine),
                WideArithmetic.MultiplySigned192(
                    localY,
                    rotation.Sine)));
        y = Signed192.NarrowValue(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    localX,
                    rotation.Sine),
                WideArithmetic.MultiplySigned192(
                    localY,
                    rotation.Cosine)));
    }

    private static void GetDirection(
        Vector2d direction,
        out WideAxis2d axis)
    {
        axis = new WideAxis2d(
            Signed192.Raw(direction.X),
            Signed192.Raw(direction.Y));
    }
}
