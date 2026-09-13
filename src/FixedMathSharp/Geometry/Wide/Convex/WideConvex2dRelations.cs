//=======================================================================
// WideConvex2dRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Contains methods for computing geometric relations between convex shapes in 2D space using wide fixed-point arithmetic.
/// </summary>
internal static partial class WideConvex2dRelations
{
    #region Nested Types

    private readonly struct WideAxis2d
    {
        internal readonly Signed192 X;
        internal readonly Signed192 Y;

        internal bool IsZero => X.IsZero && Y.IsZero;

        internal WideAxis2d(Signed192 x, Signed192 y)
        {
            X = x;
            Y = y;
        }

        public static WideAxis2d operator -(WideAxis2d value) =>
            new(WideArithmetic.Negate(value.X), WideArithmetic.Negate(value.Y));
    }

    internal readonly struct RotationFrame2d
    {
        internal static readonly RotationFrame2d Identity = new(
            Signed192.One,
            default);

        internal readonly Signed192 Cosine;
        internal readonly Signed192 Sine;

        internal RotationFrame2d(Fixed64 rotation)
            : this(
                Signed192.Raw(FixedMath.Cos(rotation)),
                Signed192.Raw(FixedMath.Sin(rotation)))
        {
        }

        private RotationFrame2d(
            Signed192 cosine,
            Signed192 sine)
        {
            Cosine = cosine;
            Sine = sine;
        }
    }

    private readonly struct Feature
    {
        internal readonly Vector2d Start;
        internal readonly Vector2d End;

        internal bool IsPoint => Start == End;

        internal Feature(Vector2d start, Vector2d end)
        {
            Start = start;
            End = end;
        }
    }

    private readonly struct PenetrationAxis
    {
        internal readonly WideAxis2d Axis;
        internal readonly Signed320 Overlap;
        internal readonly Signed576 OverlapSquared;
        internal readonly Signed320 AxisSquared;
        internal readonly bool Negate;
        internal readonly bool HasValue;

        internal PenetrationAxis(
            WideAxis2d axis,
            Signed320 overlap,
            Signed576 overlapSquared,
            Signed320 axisSquared,
            bool negate)
        {
            Axis = axis;
            Overlap = overlap;
            OverlapSquared = overlapSquared;
            AxisSquared = axisSquared;
            Negate = negate;
            HasValue = true;
        }
    }

    #endregion

    internal static FixedBoundArea GetBoundsClippedToDomain(
        Vector2d origin,
        Fixed64 rotation,
        ReadOnlySpan<Vector2d> vertexOffsets)
    {
        RotationFrame2d rotationFrame = new(rotation);
        GetWorldPoint(
            origin,
            rotationFrame,
            vertexOffsets[0],
            Vector2d.Zero,
            out Signed192 minimumX,
            out Signed192 minimumY);
        Signed192 maximumX = minimumX;
        Signed192 maximumY = minimumY;
        for (int i = 1; i < vertexOffsets.Length; i++)
        {
            GetWorldPoint(
                origin,
                rotationFrame,
                vertexOffsets[i],
                Vector2d.Zero,
                out Signed192 x,
                out Signed192 y);
            if (WideArithmetic.SubtractSigned192(x, minimumX).Sign < 0)
                minimumX = x;
            if (WideArithmetic.SubtractSigned192(x, maximumX).Sign > 0)
                maximumX = x;
            if (WideArithmetic.SubtractSigned192(y, minimumY).Sign < 0)
                minimumY = y;
            if (WideArithmetic.SubtractSigned192(y, maximumY).Sign > 0)
                maximumY = y;
        }

        Signed192 denominator = Signed192.One;
        return FixedBoundArea.FromMinMax(
            new Vector2d(
                WideGeometry.GetRationalBoundClippedToDomain(
                    Signed320.ExtendValue(minimumX),
                    denominator,
                    minimum: true),
                WideGeometry.GetRationalBoundClippedToDomain(
                    Signed320.ExtendValue(minimumY),
                    denominator,
                    minimum: true)),
            new Vector2d(
                WideGeometry.GetRationalBoundClippedToDomain(
                    Signed320.ExtendValue(maximumX),
                    denominator,
                    minimum: false),
                WideGeometry.GetRationalBoundClippedToDomain(
                    Signed320.ExtendValue(maximumY),
                    denominator,
                    minimum: false)));
    }

    internal static Vector2d GetTransformedEdgeDirection(
        Fixed64 rotation,
        Vector2d start,
        Vector2d end)
    {
        GetTransformedEdge(
            new RotationFrame2d(rotation),
            start,
            end,
            out Signed192 x,
            out Signed192 y);
        return WideNormalization.GetNormalized(x, y);
    }

    internal static void GetRelativePointNumerators(
        Vector2d pointOrigin,
        Fixed64 pointRotation,
        Vector2d pointLocalOffset,
        Vector2d referenceOrigin,
        out Signed192 x,
        out Signed192 y)
    {
        GetWorldPoint(
            pointOrigin,
            new RotationFrame2d(pointRotation),
            pointLocalOffset,
            Vector2d.Zero,
            out Signed192 pointX,
            out Signed192 pointY);
        GetWorldPoint(
            referenceOrigin,
            RotationFrame2d.Identity,
            Vector2d.Zero,
            Vector2d.Zero,
            out Signed192 referenceX,
            out Signed192 referenceY);
        x = WideArithmetic.SubtractSigned192(pointX, referenceX);
        y = WideArithmetic.SubtractSigned192(pointY, referenceY);
    }

    internal static bool IsStrictlyConvex(
        ReadOnlySpan<Vector2d> vertexOffsets)
    {
        int winding = 0;
        for (int index = 0; index < vertexOffsets.Length; index++)
        {
            Vector2d first = vertexOffsets[index];
            Vector2d second = vertexOffsets[
                index + 1 == vertexOffsets.Length ? 0 : index + 1];
            Vector2d third = vertexOffsets[
                index + 2 >= vertexOffsets.Length
                    ? index + 2 - vertexOffsets.Length
                    : index + 2];
            Signed192 firstEdgeX = WideArithmetic.Difference(second.X, first.X);
            Signed192 firstEdgeY = WideArithmetic.Difference(second.Y, first.Y);
            Signed192 secondEdgeX = WideArithmetic.Difference(third.X, second.X);
            Signed192 secondEdgeY = WideArithmetic.Difference(third.Y, second.Y);
            Signed320 cross = WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(firstEdgeX, secondEdgeY),
                WideArithmetic.MultiplySigned192(firstEdgeY, secondEdgeX));
            int sign = cross.Sign;
            if (sign == 0)
                return false;
            if (winding == 0)
                winding = sign;
            else if (sign != winding)
                return false;
        }

        return true;
    }

    internal static Vector2d GetSupportOffset(
        ReadOnlySpan<Vector2d> vertexOffsets,
        Vector2d direction) =>
        GetSupportOffset(
            Fixed64.Zero,
            vertexOffsets,
            direction);

    internal static Vector2d GetSupportOffset(
        Fixed64 rotation,
        ReadOnlySpan<Vector2d> vertexOffsets,
        Vector2d direction)
    {
        RotationFrame2d rotationFrame = new(rotation);
        GetDirection(direction, out WideAxis2d axis);
        int bestIndex = 0;
        Signed320 bestProjection = GetProjection(
            Vector2d.Zero,
            rotationFrame,
            vertexOffsets[0],
            axis);
        for (int i = 1; i < vertexOffsets.Length; i++)
        {
            Signed320 projection = GetProjection(
                Vector2d.Zero,
                rotationFrame,
                vertexOffsets[i],
                axis);
            if (WideArithmetic.SubtractSigned320(
                    projection,
                    bestProjection).Sign <= 0)
            {
                continue;
            }

            bestIndex = i;
            bestProjection = projection;
        }

        return vertexOffsets[bestIndex];
    }

    internal static Vector2d OrientTargetToSourceNormal(
        Vector2d normal,
        Vector2d sourceOrigin,
        Vector2d sourceTranslationOffset,
        Vector2d targetOrigin)
    {
        Signed192 x = WideArithmetic.AddSigned192(
            WideArithmetic.Difference(sourceOrigin.X, targetOrigin.X),
            Signed192.Raw(sourceTranslationOffset.X));
        Signed192 y = WideArithmetic.AddSigned192(
            WideArithmetic.Difference(sourceOrigin.Y, targetOrigin.Y),
            Signed192.Raw(sourceTranslationOffset.Y));
        Signed320 centerProjection = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(x, Signed192.Raw(normal.X)),
            WideArithmetic.MultiplySigned192(y, Signed192.Raw(normal.Y)));
        if (centerProjection.Sign > 0)
            return normal;
        if (centerProjection.Sign < 0)
            return -normal;

        // Sweep normals already face against motion. Preserve that orientation
        // when coincident centers provide no geometric preference.
        return normal;
    }

    internal static Vector2d GetClosestPointOffset(
        Vector2d pointOrigin,
        Vector2d pointOriginOffset,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets) =>
        GetClosestPointOffset(
            pointOrigin,
            pointOriginOffset,
            convexOrigin,
            Fixed64.Zero,
            convexVertexOffsets);

    internal static Vector2d GetClosestPointOffset(
        Vector2d pointOrigin,
        Vector2d pointOriginOffset,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        RotationFrame2d convexFrame = new(convexRotation);
        Vector2d start = convexVertexOffsets[0];
        Vector2d end = convexVertexOffsets[1];
        _ = TryProjectPointOntoFeature(
            pointOrigin,
            pointOriginOffset,
            convexOrigin,
            convexFrame,
            start,
            end,
            requireInteriorProjection: false,
            out Vector2d convexPointOffset);
        Signed320 bestSquaredDistance = GetSquaredDistance(
            pointOrigin,
            pointOriginOffset,
            convexOrigin,
            convexFrame,
            convexPointOffset);
        for (int i = 1; i < convexVertexOffsets.Length; i++)
        {
            start = convexVertexOffsets[i];
            end = convexVertexOffsets[
                i + 1 == convexVertexOffsets.Length ? 0 : i + 1];
            _ = TryProjectPointOntoFeature(
                pointOrigin,
                pointOriginOffset,
                convexOrigin,
                convexFrame,
                start,
                end,
                requireInteriorProjection: false,
                out Vector2d candidateOffset);

            Signed320 squaredDistance = GetSquaredDistance(
                pointOrigin,
                pointOriginOffset,
                convexOrigin,
                convexFrame,
                candidateOffset);
            if (WideArithmetic.SubtractSigned320(
                    squaredDistance,
                    bestSquaredDistance).Sign >= 0)
            {
                continue;
            }

            bestSquaredDistance = squaredDistance;
            convexPointOffset = candidateOffset;
        }

        return convexPointOffset;
    }

    internal static Vector2d GetSweptPointTargetContactOffset(
        Vector2d sourceOrigin,
        Vector2d sourceTranslationOffset,
        Vector2d sourceLocalOffset,
        Vector2d targetOrigin,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        Vector2d targetSupportDirection) =>
        GetSweptPointTargetContactOffset(
            sourceOrigin,
            sourceTranslationOffset,
            sourceLocalOffset,
            targetOrigin,
            Fixed64.Zero,
            targetVertexOffsets,
            targetSupportDirection);

    internal static Vector2d GetSweptPointTargetContactOffset(
        Vector2d sourceOrigin,
        Vector2d sourceTranslationOffset,
        Vector2d sourceLocalOffset,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        Vector2d targetSupportDirection)
    {
        RotationFrame2d targetFrame = new(targetRotation);
        GetDirection(targetSupportDirection, out WideAxis2d axis);
        GetSupportFeature(
            targetOrigin,
            targetFrame,
            targetVertexOffsets,
            axis,
            maximum: true,
            out Feature targetFeature);
        _ = TryProjectPointOntoFeature(
            sourceOrigin,
            RotationFrame2d.Identity,
            sourceLocalOffset,
            Vector2d.Zero,
            sourceTranslationOffset,
            targetOrigin,
            targetFrame,
            targetFeature.Start,
            targetFeature.End,
            requireInteriorProjection: false,
            out Vector2d targetContactOffset);
        return targetContactOffset;
    }

    internal static Vector2d GetSweptAnchorTargetContactOffset(
        in FixedPointAnchor2d source,
        Vector2d sourceTranslationOffset,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        Vector2d targetSupportDirection)
    {
        RotationFrame2d targetFrame = new(targetRotation);
        GetDirection(targetSupportDirection, out WideAxis2d axis);
        GetSupportFeature(
            targetOrigin,
            targetFrame,
            targetVertexOffsets,
            axis,
            maximum: true,
            out Feature targetFeature);
        _ = TryProjectPointOntoFeature(
            source.Origin,
            new RotationFrame2d(source.Rotation),
            source.LocalPoint,
            source.LocalDisplacement,
            sourceTranslationOffset,
            targetOrigin,
            targetFrame,
            targetFeature.Start,
            targetFeature.End,
            requireInteriorProjection: false,
            out Vector2d targetContactOffset);
        return targetContactOffset;
    }

    internal static Vector2d GetSweptConvexTargetContactOffset(
        Vector2d sourceOrigin,
        Fixed64 sourceRotation,
        ReadOnlySpan<Vector2d> sourceVertexOffsets,
        Vector2d sourceTranslationOffset,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        Vector2d targetSupportDirection)
    {
        RotationFrame2d sourceFrame = new(sourceRotation);
        RotationFrame2d targetFrame = new(targetRotation);
        GetDirection(targetSupportDirection, out WideAxis2d targetAxis);
        GetSupportFeature(
            targetOrigin,
            targetFrame,
            targetVertexOffsets,
            targetAxis,
            maximum: true,
            out Feature targetFeature);
        GetSupportFeature(
            sourceOrigin,
            sourceFrame,
            sourceVertexOffsets,
            -targetAxis,
            maximum: true,
            out Feature sourceFeature);
        _ = TryProjectPointOntoFeature(
            sourceOrigin,
            sourceFrame,
            sourceFeature.Start,
            Vector2d.Zero,
            sourceTranslationOffset,
            targetOrigin,
            targetFrame,
            targetFeature.Start,
            targetFeature.End,
            requireInteriorProjection: false,
            out Vector2d targetContactOffset);
        return targetContactOffset;
    }

    internal static bool ContainsPoint(
        Vector2d point,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        RotationFrame2d convexFrame = new(convexRotation);
        // Widen before subtracting: the point-to-origin displacement can exceed
        // the scalar domain. Keep it at the rotated vertices' product scale.
        Signed192 relativePointX = WideArithmetic.SubtractSigned192(
            WideArithmetic.Scale(point.X),
            WideArithmetic.Scale(convexOrigin.X));
        Signed192 relativePointY = WideArithmetic.SubtractSigned192(
            WideArithmetic.Scale(point.Y),
            WideArithmetic.Scale(convexOrigin.Y));
        GetRotatedOffset(
            convexFrame,
            convexVertexOffsets[0],
            out Signed192 startX,
            out Signed192 startY);
        bool hasPositive = false;
        bool hasNegative = false;
        for (int i = 0; i < convexVertexOffsets.Length; i++)
        {
            Vector2d end = convexVertexOffsets[
                i + 1 == convexVertexOffsets.Length ? 0 : i + 1];
            GetRotatedOffset(
                convexFrame,
                end,
                out Signed192 endX,
                out Signed192 endY);
            // Wide rotation is linear without rounding: R(end) - R(start)
            // equals R(end - start), and the next edge reuses this endpoint.
            Signed192 edgeX = WideArithmetic.SubtractSigned192(endX, startX);
            Signed192 edgeY = WideArithmetic.SubtractSigned192(endY, startY);
            Signed192 pointX = WideArithmetic.SubtractSigned192(relativePointX, startX);
            Signed192 pointY = WideArithmetic.SubtractSigned192(relativePointY, startY);
            int orientation = WideArithmetic.MultiplySubtract(
                edgeX,
                pointY,
                edgeY,
                pointX).Sign;
            if (orientation > 0)
                hasPositive = true;
            else if (orientation < 0)
                hasNegative = true;
            if (hasPositive && hasNegative)
                return false;

            startX = endX;
            startY = endY;
        }

        return true;
    }

    internal static bool TryProjectAnchorOntoFeature(
        in FixedPointAnchor2d source,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        Vector2d featureStart,
        Vector2d featureEnd,
        bool requireInteriorProjection,
        out Vector2d targetOffset) =>
        TryProjectPointOntoFeature(
            source.Origin,
            new RotationFrame2d(source.Rotation),
            source.LocalPoint,
            source.LocalDisplacement,
            Vector2d.Zero,
            targetOrigin,
            new RotationFrame2d(targetRotation),
            featureStart,
            featureEnd,
            requireInteriorProjection,
            out targetOffset);

    internal static bool TryGetContactOffsets(
        Vector2d firstOrigin,
        Fixed64 firstRotation,
        ReadOnlySpan<Vector2d> firstVertexOffsets,
        Vector2d secondOrigin,
        Fixed64 secondRotation,
        ReadOnlySpan<Vector2d> secondVertexOffsets,
        Span<Vector2d> firstContactOffsets,
        Span<Vector2d> secondContactOffsets,
        out int contactCount,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        RotationFrame2d firstFrame = new(firstRotation);
        RotationFrame2d secondFrame = new(secondRotation);
        var best = default(PenetrationAxis);
        if (!TryKeepAxes(
                firstOrigin,
                firstFrame,
                firstVertexOffsets,
                secondOrigin,
                secondFrame,
                secondVertexOffsets,
                sourceIsFirst: true,
                ref best)
            || !TryKeepAxes(
                secondOrigin,
                secondFrame,
                secondVertexOffsets,
                firstOrigin,
                firstFrame,
                firstVertexOffsets,
                sourceIsFirst: false,
                ref best))
        {
            contactCount = default;
            normal = default;
            depth = default;
            depthIsClamped = default;
            return false;
        }

        WideAxis2d orientedAxis = best.Negate ? -best.Axis : best.Axis;
        normal = WideNormalization.GetNormalized(
            orientedAxis.X,
            orientedAxis.Y);
        GetDepth(
            best.Overlap,
            best.AxisSquared,
            out depth,
            out depthIsClamped);
        GetSupportFeature(
            firstOrigin,
            firstFrame,
            firstVertexOffsets,
            orientedAxis,
            maximum: true,
            out Feature firstFeature);
        GetSupportFeature(
            secondOrigin,
            secondFrame,
            secondVertexOffsets,
            orientedAxis,
            maximum: false,
            out Feature secondFeature);
        contactCount = 0;

        if (firstFeature.IsPoint && secondFeature.IsPoint)
        {
            firstContactOffsets[0] = firstFeature.Start;
            secondContactOffsets[0] = secondFeature.Start;
            contactCount = 1;
            return true;
        }

        if (firstFeature.IsPoint)
        {
            _ = TryProjectPointOntoFeature(
                firstOrigin,
                firstFeature.Start,
                secondOrigin,
                secondFrame,
                secondFeature.Start,
                secondFeature.End,
                requireInteriorProjection: false,
                out Vector2d secondOffset);

            firstContactOffsets[0] = firstFeature.Start;
            secondContactOffsets[0] = secondOffset;
            contactCount = 1;
            return true;
        }

        if (secondFeature.IsPoint)
        {
            _ = TryProjectPointOntoFeature(
                secondOrigin,
                secondFeature.Start,
                firstOrigin,
                firstFrame,
                firstFeature.Start,
                firstFeature.End,
                requireInteriorProjection: false,
                out Vector2d firstOffset);

            firstContactOffsets[0] = firstOffset;
            secondContactOffsets[0] = secondFeature.Start;
            contactCount = 1;
            return true;
        }

        AddProjectedEndpoint(
            secondOrigin,
            secondFrame,
            secondFeature.Start,
            firstOrigin,
            firstFrame,
            firstFeature,
            sourceIsFirst: false,
            firstContactOffsets,
            secondContactOffsets,
            ref contactCount);
        AddProjectedEndpoint(
            secondOrigin,
            secondFrame,
            secondFeature.End,
            firstOrigin,
            firstFrame,
            firstFeature,
            sourceIsFirst: false,
            firstContactOffsets,
            secondContactOffsets,
            ref contactCount);
        AddProjectedEndpoint(
            firstOrigin,
            firstFrame,
            firstFeature.Start,
            secondOrigin,
            secondFrame,
            secondFeature,
            sourceIsFirst: true,
            firstContactOffsets,
            secondContactOffsets,
            ref contactCount);
        AddProjectedEndpoint(
            firstOrigin,
            firstFrame,
            firstFeature.End,
            secondOrigin,
            secondFrame,
            secondFeature,
            sourceIsFirst: true,
            firstContactOffsets,
            secondContactOffsets,
            ref contactCount);
        // Overlapping closed convex support segments have overlapping
        // tangential intervals, so at least one endpoint projection is
        // necessarily retained by the four exact checks above.
        return true;
    }

    internal static bool TryGetAreaAndCentroid(
        ReadOnlySpan<Vector2d> vertices,
        out Fixed64 area,
        out Vector2d centroid)
    {
        bool result = TryGetSignedDoubleAreaAndCentroid(
            vertices,
            out Signed320 signedDoubleArea,
            out centroid);
        Signed320 absoluteDoubleArea = signedDoubleArea.Sign < 0
            ? WideArithmetic.SubtractSigned320(
                default,
                signedDoubleArea)
            : signedDoubleArea;
        Signed192 doubledFixedScale =
            WideArithmetic.AddSigned192(
                Signed192.One,
                Signed192.One);
        if (!Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(absoluteDoubleArea),
                Signed576.ExtendValue(
                    Signed320.ExtendValue(doubledFixedScale)),
                out area))
        {
            area = Fixed64.MaxValue;
        }
        return result;
    }

    internal static bool TryGetSignedDoubleAreaAndCentroid(
        ReadOnlySpan<Vector2d> vertices,
        out Signed320 signedDoubleArea,
        out Vector2d centroid)
    {
        Vector2d anchor = vertices[0];
        signedDoubleArea = default;
        Signed576 weightedX = default;
        Signed576 weightedY = default;
        for (int i = 1; i < vertices.Length - 1; i++)
        {
            Signed192 ax = WideArithmetic.SubtractSigned192(
                Signed192.Raw(vertices[i].X),
                Signed192.Raw(anchor.X));
            Signed192 ay = WideArithmetic.SubtractSigned192(
                Signed192.Raw(vertices[i].Y),
                Signed192.Raw(anchor.Y));
            Signed192 bx = WideArithmetic.SubtractSigned192(
                Signed192.Raw(vertices[i + 1].X),
                Signed192.Raw(anchor.X));
            Signed192 by = WideArithmetic.SubtractSigned192(
                Signed192.Raw(vertices[i + 1].Y),
                Signed192.Raw(anchor.Y));
            Signed320 cross = WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(ax, by),
                WideArithmetic.MultiplySigned192(ay, bx));
            signedDoubleArea =
                WideArithmetic.AddSigned320(signedDoubleArea, cross);
            weightedX = WideArithmetic.AddSigned576(
                weightedX,
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(
                        WideArithmetic.AddSigned192(ax, bx)),
                    cross));
            weightedY = WideArithmetic.AddSigned576(
                weightedY,
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(
                        WideArithmetic.AddSigned192(ay, by)),
                    cross));
        }

        int areaSign = signedDoubleArea.Sign;
        if (areaSign == 0)
        {
            centroid = default;
            return false;
        }

        // The signed area can require more than one word. Multiplication by
        // three is therefore performed directly at the five-word width.
        Signed320 centroidDenominator = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                signedDoubleArea,
                signedDoubleArea),
            signedDoubleArea);
        Signed576 denominator =
            Signed576.ExtendValue(centroidDenominator);
        Signed576 centroidXNumerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(Signed192.Raw(anchor.X)),
                centroidDenominator),
            weightedX);
        Signed576 centroidYNumerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(Signed192.Raw(anchor.Y)),
                centroidDenominator),
            weightedY);
        // A centroid remains inside the convex hull of representable inputs.
        _ = Fixed64.TryGetSignedRawRatio(
            centroidXNumerator,
            denominator,
            out Fixed64 x);
        _ = Fixed64.TryGetSignedRawRatio(
            centroidYNumerator,
            denominator,
            out Fixed64 y);
        centroid = new Vector2d(x, y);
        return true;
    }

    private static bool TryKeepAxes(
        Vector2d axisSourceOrigin,
        RotationFrame2d axisSourceRotation,
        ReadOnlySpan<Vector2d> axisSourceOffsets,
        Vector2d otherOrigin,
        RotationFrame2d otherRotation,
        ReadOnlySpan<Vector2d> otherOffsets,
        bool sourceIsFirst,
        ref PenetrationAxis best)
    {
        for (int i = 0; i < axisSourceOffsets.Length; i++)
        {
            Vector2d start = axisSourceOffsets[i];
            Vector2d end = axisSourceOffsets[
                i + 1 == axisSourceOffsets.Length ? 0 : i + 1];
            GetTransformedEdge(
                axisSourceRotation,
                start,
                end,
                out Signed192 edgeX,
                out Signed192 edgeY);
            WideAxis2d axis = new(edgeY, WideArithmetic.Negate(edgeX));
            if (axis.IsZero)
                continue;
            if (!TryKeepAxis(
                    axis,
                    axisSourceOrigin,
                    axisSourceRotation,
                    axisSourceOffsets,
                    otherOrigin,
                    otherRotation,
                    otherOffsets,
                    sourceIsFirst,
                    ref best))
            {
                return false;
            }
        }

        return true;
    }

    private static bool TryKeepAxis(
        WideAxis2d axis,
        Vector2d axisSourceOrigin,
        RotationFrame2d axisSourceRotation,
        ReadOnlySpan<Vector2d> axisSourceOffsets,
        Vector2d otherOrigin,
        RotationFrame2d otherRotation,
        ReadOnlySpan<Vector2d> otherOffsets,
        bool sourceIsFirst,
        ref PenetrationAxis best)
    {
        Vector2d firstOrigin = sourceIsFirst
            ? axisSourceOrigin
            : otherOrigin;
        ReadOnlySpan<Vector2d> firstOffsets = sourceIsFirst
            ? axisSourceOffsets
            : otherOffsets;
        RotationFrame2d firstRotation = sourceIsFirst
            ? axisSourceRotation
            : otherRotation;
        Vector2d secondOrigin = sourceIsFirst
            ? otherOrigin
            : axisSourceOrigin;
        ReadOnlySpan<Vector2d> secondOffsets = sourceIsFirst
            ? otherOffsets
            : axisSourceOffsets;
        RotationFrame2d secondRotation = sourceIsFirst
            ? otherRotation
            : axisSourceRotation;
        GetProjectionRange(
            firstOrigin,
            firstRotation,
            firstOffsets,
            axis,
            out Signed320 firstMinimum,
            out Signed320 firstMaximum);
        GetProjectionRange(
            secondOrigin,
            secondRotation,
            secondOffsets,
            axis,
            out Signed320 secondMinimum,
            out Signed320 secondMaximum);
        Signed320 positive = WideArithmetic.SubtractSigned320(
            firstMaximum,
            secondMinimum);
        Signed320 negative = WideArithmetic.SubtractSigned320(
            secondMaximum,
            firstMinimum);
        if (positive.Sign < 0 || negative.Sign < 0)
            return false;

        Signed320 signedCenterDifference =
            WideArithmetic.SubtractSigned320(positive, negative);
        int signedOverlapComparison = signedCenterDifference.Sign;
        bool negate = signedOverlapComparison >= 0;
        Signed320 overlap = negate ? negative : positive;
        Signed320 axisSquared = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(axis.X, axis.X),
            WideArithmetic.MultiplySigned192(axis.Y, axis.Y));
        Signed576 overlapSquared = WideArithmetic.MultiplySigned320(
            overlap,
            overlap);
        if (best.HasValue)
        {
            Signed832 candidateScaled =
                WideArithmetic.MultiplyNonNegativeToSigned832(
                    Signed832.ExtendValue(overlapSquared),
                    best.AxisSquared);
            Signed832 bestScaled =
                WideArithmetic.MultiplyNonNegativeToSigned832(
                    Signed832.ExtendValue(best.OverlapSquared),
                    axisSquared);
            int depthComparison = WideArithmetic.SubtractSigned832(
                candidateScaled,
                bestScaled).Sign;
            if (depthComparison >= 0)
                return true;

            // Trigonometric frames are representable approximations. Two
            // mathematically equal axes can therefore differ below the
            // observable Fixed64 depth resolution. Preserve the first authored
            // axis when both exact candidates narrow to the same public depth;
            // a representable depth still wins over a clamped tie.
            GetDepth(
                overlap,
                axisSquared,
                out Fixed64 candidateDepth,
                out bool candidateDepthIsClamped);
            GetDepth(
                best.Overlap,
                best.AxisSquared,
                out Fixed64 bestDepth,
                out bool bestDepthIsClamped);
            if (candidateDepth == bestDepth
                && !(bestDepthIsClamped
                    && !candidateDepthIsClamped))
            {
                return true;
            }
        }

        best = new PenetrationAxis(
            axis,
            overlap,
            overlapSquared,
            axisSquared,
            negate);
        return true;
    }
}
