//=======================================================================
// FixedConvex2dRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Provides exact relations for convex 2D shapes represented by canonical
/// origins, scalar rotations, and local vertex offsets.
/// </summary>
/// <content>Owns exact planar convex contacts, queries and translated shape intersections.</content>
public static partial class FixedConvex2dRelations
{
    /// <summary>
    /// Returns whether boundary-ordered vertices form a strictly convex
    /// polygon in either winding direction.
    /// </summary>
    public static bool IsStrictlyConvex(
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.IsStrictlyConvex(convexVertexOffsets);
    }

    /// <summary>
    /// Returns whether a world-space point lies inside or on a closed convex
    /// polygon represented by an origin and origin-relative vertices.
    /// </summary>
    public static bool ContainsPoint(
        Vector2d point,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.ContainsPoint(
            point,
            convexOrigin,
            Fixed64.Zero,
            convexVertexOffsets);
    }

    /// <summary>
    /// Returns whether a world-space point lies inside or on a rotated closed
    /// convex polygon represented by a frame and local vertex offsets.
    /// </summary>
    public static bool ContainsPoint(
        Vector2d point,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.ContainsPoint(
            point,
            convexOrigin,
            convexRotation,
            convexVertexOffsets);
    }

    /// <summary>
    /// Finds the polygon-relative closest point to a world-space point without
    /// materializing polygon vertices.
    /// </summary>
    public static Vector2d GetClosestPointOffset(
        Vector2d point,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.GetClosestPointOffset(
            point,
            Vector2d.Zero,
            convexOrigin,
            convexVertexOffsets);
    }

    /// <summary>
    /// Finds the closest point on a rotated polygon while retaining that point
    /// in the polygon's supplied local frame.
    /// </summary>
    public static FixedPointAnchor2d GetClosestPointAnchor(
        Vector2d point,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            WideConvex2dRelations.GetClosestPointOffset(
                point,
                Vector2d.Zero,
                convexOrigin,
                convexRotation,
                convexVertexOffsets));
    }

    /// <summary>
    /// Returns the first authored polygon-relative vertex with the greatest
    /// exact projection along a direction.
    /// </summary>
    public static Vector2d GetSupportOffset(
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        Vector2d direction)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.GetSupportOffset(
            convexVertexOffsets,
            direction);
    }

    /// <summary>
    /// Returns the first local vertex with the greatest exact world-space
    /// projection along a direction.
    /// </summary>
    public static FixedPointAnchor2d GetSupportAnchor(
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        Vector2d direction)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            WideConvex2dRelations.GetSupportOffset(
                convexRotation,
                convexVertexOffsets,
                direction));
    }

    /// <summary>
    /// Attempts to derive the area and centroid of a boundary-ordered convex
    /// polygon without narrowing cross products or weighted-coordinate sums.
    /// </summary>
    /// <remarks>
    /// Area is nonnegative and clamps to <see cref="Fixed64.MaxValue"/> when
    /// the conceptual value exceeds the scalar domain. The centroid uses one
    /// final round-half-to-even conversion per component.
    /// </remarks>
    /// <returns>
    /// <see langword="false"/> only when the polygon has exact zero signed
    /// area or its final centroid is outside the scalar domain.
    /// </returns>
    public static bool TryGetAreaAndCentroid(
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 area,
        out Vector2d centroid)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.TryGetAreaAndCentroid(
            convexVertexOffsets,
            out area,
            out centroid);
    }

    /// <summary>
    /// Attempts to build up to two local-frame contact-anchor pairs between
    /// two rotated closed convex polygons.
    /// </summary>
    public static bool TryGetConvexContacts(
        Vector2d firstOrigin,
        Fixed64 firstRotation,
        ReadOnlySpan<Vector2d> firstVertexOffsets,
        Vector2d secondOrigin,
        Fixed64 secondRotation,
        ReadOnlySpan<Vector2d> secondVertexOffsets,
        Span<FixedPointAnchor2d> firstContacts,
        Span<FixedPointAnchor2d> secondContacts,
        out int contactCount,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateVertexOffsets(firstVertexOffsets);
        ValidateVertexOffsets(secondVertexOffsets);
        if (firstContacts.Length < 2)
        {
            throw new ArgumentException(
                "Contact output requires capacity for two anchors.",
                nameof(firstContacts));
        }
        if (secondContacts.Length < 2)
        {
            throw new ArgumentException(
                "Contact output requires capacity for two anchors.",
                nameof(secondContacts));
        }

        Span<Vector2d> firstOffsets = stackalloc Vector2d[2];
        Span<Vector2d> secondOffsets = stackalloc Vector2d[2];
        if (!WideConvex2dRelations.TryGetContactOffsets(
                firstOrigin,
                firstRotation,
                firstVertexOffsets,
                secondOrigin,
                secondRotation,
                secondVertexOffsets,
                firstOffsets,
                secondOffsets,
                out contactCount,
                out normal,
                out depth,
                out depthIsClamped))
        {
            return false;
        }

        for (int i = 0; i < contactCount; i++)
        {
            firstContacts[i] = new FixedPointAnchor2d(
                firstOrigin,
                firstRotation,
                firstOffsets[i]);
            secondContacts[i] = new FixedPointAnchor2d(
                secondOrigin,
                secondRotation,
                secondOffsets[i]);
        }
        return true;
    }

    /// <summary>
    /// Attempts to build a contact-anchor pair between a conceptual circle and
    /// a rotated closed convex polygon.
    /// </summary>
    /// <remarks>
    /// A circle's rotation does not change its geometry, but it defines the
    /// rigid local frame of the returned circle anchor.
    /// </remarks>
    public static bool TryGetCircleContact(
        Vector2d circleCenter,
        Fixed64 circleRotation,
        Fixed64 circleRadius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out FixedPointAnchor2d circleContact,
        out FixedPointAnchor2d convexContact,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        ValidateVertexOffsets(convexVertexOffsets);

        Span<FixedPointAnchor2d> circleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (!WideCenteredCapsule2dRelations.TryGetContacts(
                circleCenter,
                circleRotation,
                Vector2d.Right,
                Fixed64.Zero,
                circleRadius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                circleContacts,
                convexContacts,
                out _,
                out normal,
                out depth,
                out depthIsClamped))
        {
            circleContact = default;
            convexContact = default;
            return false;
        }

        circleContact = circleContacts[0];
        convexContact = convexContacts[0];
        return true;
    }

    private static void ValidateVertexOffsets(
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        if (convexVertexOffsets.Length < 3)
        {
            throw new ArgumentException(
                "A convex polygon requires at least three vertex offsets.",
                nameof(convexVertexOffsets));
        }
    }

    /// <summary>
    /// Finds the first distance where a finite directed segment reaches a
    /// closed convex polygon.
    /// </summary>
    public static bool TryGetSegmentFirstIntersectionDistance(
        Vector2d start,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset)
    {
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        if (WideConvex2dRelations.ContainsPoint(
                start,
                convexOrigin,
                Fixed64.Zero,
                convexVertexOffsets))
        {
            distance = Fixed64.Zero;
            normal = -direction;
            convexContactOffset =
                WideConvex2dRelations.GetSweptPointTargetContactOffset(
                start,
                Vector2d.Zero,
                Vector2d.Zero,
                convexOrigin,
                convexVertexOffsets,
                normal);
            return true;
        }

        return TryGetSweptCenteredCapsuleFirstDistanceCore(
            start,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            direction,
            maximumDistance,
            convexOrigin,
            convexVertexOffsets,
            out distance,
            out normal,
            out convexContactOffset);
    }

    /// <summary>
    /// Finds the first distance where a finite directed segment reaches a
    /// rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSegmentFirstIntersectionDistance(
        Vector2d start,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d convexContact)
    {
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        Vector2d contactOffset;
        if (WideConvex2dRelations.ContainsPoint(
                start,
                convexOrigin,
                convexRotation,
                convexVertexOffsets))
        {
            distance = Fixed64.Zero;
            normal = -direction;
            contactOffset =
                WideConvex2dRelations.GetSweptPointTargetContactOffset(
                    start,
                    Vector2d.Zero,
                    Vector2d.Zero,
                    convexOrigin,
                    convexRotation,
                    convexVertexOffsets,
                    normal);
        }
        else if (!TryGetSweptCenteredCapsuleFirstDistanceCore(
                     start,
                     Vector2d.Right,
                     Fixed64.Zero,
                     Fixed64.Zero,
                     direction,
                     maximumDistance,
                     convexOrigin,
                     convexRotation,
                     convexVertexOffsets,
                     out distance,
                     out normal,
                     out contactOffset))
        {
            convexContact = default;
            return false;
        }

        convexContact = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            contactOffset);
        return true;
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual circle reaches a
    /// closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCircleFirstDistance(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset)
    {
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        if (TryGetCircleContact(
                circleCenter,
                Fixed64.Zero,
                circleRadius,
                convexOrigin,
                Fixed64.Zero,
                convexVertexOffsets,
                out _,
                out FixedPointAnchor2d initialContact,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContactOffset = initialContact.LocalPoint;
            return true;
        }

        return TryGetSweptCenteredCapsuleFirstDistanceCore(
            circleCenter,
            Vector2d.Right,
            Fixed64.Zero,
            circleRadius,
            direction,
            maximumDistance,
            convexOrigin,
            convexVertexOffsets,
            out distance,
            out normal,
            out convexContactOffset);
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual circle reaches
    /// a rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCircleFirstDistance(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d convexContact)
    {
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        Vector2d contactOffset;
        if (TryGetCircleContact(
                circleCenter,
                Fixed64.Zero,
                circleRadius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out _,
                out FixedPointAnchor2d initialContact,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContact = initialContact;
            return true;
        }
        if (!TryGetSweptCenteredCapsuleFirstDistanceCore(
                circleCenter,
                Vector2d.Right,
                Fixed64.Zero,
                circleRadius,
                direction,
                maximumDistance,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out distance,
                out normal,
                out contactOffset))
        {
            convexContact = default;
            return false;
        }

        convexContact = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            contactOffset);
        return true;
    }

    /// <summary>
    /// Finds the first distance where a translated centered rotated capsule
    /// reaches a rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCenteredCapsuleFirstDistance(
        Vector2d capsuleCenter,
        Fixed64 capsuleRotation,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d convexContact)
    {
        if (capsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleAxisLength));
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);

        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                capsuleCenter,
                capsuleRotation,
                Vector2d.Forward,
                capsuleAxisLength,
                capsuleRadius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                capsuleContacts,
                convexContacts,
                out int contactCount,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContact = convexContacts[contactCount - 1];
            return true;
        }

        Vector2d capsuleAxis = new(
            -FixedMath.Sin(capsuleRotation),
            FixedMath.Cos(capsuleRotation));
        if (!TryGetSweptCenteredCapsuleFirstDistanceCore(
                capsuleCenter,
                capsuleAxis,
                capsuleAxisLength,
                capsuleRadius,
                direction,
                maximumDistance,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out distance,
                out normal,
                out Vector2d contactOffset))
        {
            convexContact = default;
            return false;
        }

        convexContact = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            contactOffset);
        return true;
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual centered
    /// capsule reaches a closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCenteredCapsuleFirstDistance(
        Vector2d capsuleCenter,
        Vector2d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset)
    {
        ValidateCenteredAxis(
            capsuleAxisDirection,
            capsuleAxisLength);
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (WideCenteredCapsule2dRelations.TryGetContacts(
                capsuleCenter,
                Fixed64.Zero,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                convexOrigin,
                Fixed64.Zero,
                convexVertexOffsets,
                capsuleContacts,
                convexContacts,
                out int contactCount,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContactOffset =
                convexContacts[contactCount - 1].LocalPoint;
            return true;
        }

        return TryGetSweptCenteredCapsuleFirstDistanceCore(
            capsuleCenter,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            direction,
            maximumDistance,
            convexOrigin,
            convexVertexOffsets,
            out distance,
            out normal,
            out convexContactOffset);
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual centered capsule
    /// reaches a rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCenteredCapsuleFirstDistance(
        Vector2d capsuleCenter,
        Vector2d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d convexContact)
    {
        ValidateCenteredAxis(capsuleAxisDirection, capsuleAxisLength);
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (WideCenteredCapsule2dRelations.TryGetContacts(
                capsuleCenter,
                Fixed64.Zero,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                capsuleContacts,
                convexContacts,
                out int contactCount,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContact = convexContacts[contactCount - 1];
            return true;
        }

        if (!TryGetSweptCenteredCapsuleFirstDistanceCore(
                capsuleCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                direction,
                maximumDistance,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out distance,
                out normal,
                out Vector2d contactOffset))
        {
            convexContact = default;
            return false;
        }

        convexContact = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            contactOffset);
        return true;
    }

    /// <summary>
    /// Finds the first distance where a translated closed convex polygon
    /// reaches another closed convex polygon.
    /// </summary>
    public static bool TryGetSweptConvexFirstDistance(
        Vector2d moverOrigin,
        ReadOnlySpan<Vector2d> moverVertexOffsets,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetOrigin,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d targetContactOffset) =>
        TryGetSweptConvexFirstDistanceCore(
            moverOrigin,
            Fixed64.Zero,
            moverVertexOffsets,
            direction,
            maximumDistance,
            targetOrigin,
            Fixed64.Zero,
            targetVertexOffsets,
            out distance,
            out normal,
            out targetContactOffset);

    /// <summary>
    /// Finds the first distance where a translated rotated convex polygon
    /// reaches another rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSweptConvexFirstDistance(
        Vector2d moverOrigin,
        Fixed64 moverRotation,
        ReadOnlySpan<Vector2d> moverVertexOffsets,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d targetContact)
    {
        if (!TryGetSweptConvexFirstDistanceCore(
                moverOrigin,
                moverRotation,
                moverVertexOffsets,
                direction,
                maximumDistance,
                targetOrigin,
                targetRotation,
                targetVertexOffsets,
                out distance,
                out normal,
                out Vector2d targetContactOffset))
        {
            targetContact = default;
            return false;
        }

        targetContact = new FixedPointAnchor2d(
            targetOrigin,
            targetRotation,
            targetContactOffset);
        return true;
    }

    // Authored mode consumes an already admitted displacement and its positive
    // representable length; public direction/distance entry points keep validation.
    internal static bool TryGetSweptConvexFirstDistanceCore(
        Vector2d moverOrigin,
        Fixed64 moverRotation,
        ReadOnlySpan<Vector2d> moverVertexOffsets,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d targetContactOffset,
        bool authoredDisplacement = false)
    {
        ValidateVertexOffsets(moverVertexOffsets);
        ValidateVertexOffsets(targetVertexOffsets);
        if (!authoredDisplacement)
            ValidateSweep(direction, maximumDistance);
        Span<Vector2d> moverContacts = stackalloc Vector2d[2];
        Span<Vector2d> targetContacts = stackalloc Vector2d[2];
        if (WideConvex2dRelations.TryGetContactOffsets(
                moverOrigin,
                moverRotation,
                moverVertexOffsets,
                targetOrigin,
                targetRotation,
                targetVertexOffsets,
                moverContacts,
                targetContacts,
                out int contactCount,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            targetContactOffset = targetContacts[contactCount - 1];
            return true;
        }

        bool found = false;
        distance = default;
        normal = default;
        targetContactOffset = default;
        for (int moverIndex = 0;
            moverIndex < moverVertexOffsets.Length;
            moverIndex++)
        {
            Vector2d moverStart = moverVertexOffsets[moverIndex];
            Vector2d moverEnd = moverVertexOffsets[
                moverIndex + 1 == moverVertexOffsets.Length
                    ? 0
                    : moverIndex + 1];
            for (int targetIndex = 0;
                targetIndex < targetVertexOffsets.Length;
                targetIndex++)
            {
                Vector2d targetStart = targetVertexOffsets[targetIndex];
                Vector2d targetEnd = targetVertexOffsets[
                    targetIndex + 1 == targetVertexOffsets.Length
                        ? 0
                        : targetIndex + 1];
                if (WideFiniteAxisIntersection
                    .TryGetSweptOriginOffsetSegmentsFirstDistance(
                        moverOrigin,
                        moverRotation,
                        moverStart,
                        moverEnd,
                        Fixed64.Zero,
                        direction,
                        maximumDistance,
                        targetOrigin,
                        targetRotation,
                        targetStart,
                        targetEnd,
                        Fixed64.Zero,
                        out Fixed64 candidateDistance,
                        out _,
                        authoredDisplacement))
                {
                    if (!found || candidateDistance < distance)
                    {
                        found = true;
                        distance = candidateDistance;
                    }
                }
            }
        }

        if (!found)
            return false;

        WideConvex2dRelations.GetSweptConvexContact(
            moverOrigin, moverRotation, moverVertexOffsets, targetOrigin, targetRotation, targetVertexOffsets,
            direction, out normal, out targetContactOffset);
        return true;
    }

    private static bool TryGetSweptCenteredCapsuleFirstDistanceCore(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset) =>
        TryGetSweptCenteredCapsuleFirstDistanceCore(
            center,
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            convexOrigin,
            Fixed64.Zero,
            convexVertexOffsets,
            out distance,
            out normal,
            out convexContactOffset);

    // Chord mode retains a physical-distance numerator over the admitted length,
    // so anchor reconstruction does not quantize time to a second Fixed64 fraction.
    internal static bool TryGetSweptCenteredCapsuleFirstDistanceCore(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset,
        bool authoredDisplacement = false)
    {
        bool found = false;
        distance = default;
        normal = default;
        convexContactOffset = default;
        for (int i = 0; i < convexVertexOffsets.Length; i++)
        {
            Vector2d start = convexVertexOffsets[i];
            Vector2d end = convexVertexOffsets[
                i + 1 == convexVertexOffsets.Length ? 0 : i + 1];
            if (WideFiniteAxisIntersection
                .TryGetSweptCenteredCapsuleOriginOffsetSegmentFirstDistance(
                    center,
                    axisDirection,
                    axisLength,
                    radius,
                    direction,
                    maximumDistance,
                    convexOrigin,
                    convexRotation,
                    start,
                    end,
                    Fixed64.Zero,
                    out Fixed64 candidateDistance,
                    out Vector2d candidateNormal,
                    authoredDisplacement))
            {
                // Positive radius preserves the geometric side; a rounded tangent normal's motion dot need not be zero.
                candidateNormal = radius == Fixed64.Zero
                    ? OrientZeroRadiusSweepNormal(candidateNormal, direction)
                    : -candidateNormal;
                if (!ShouldReplaceSweepCandidate(
                        candidateDistance,
                        candidateNormal,
                        direction,
                        found,
                        distance,
                        normal))
                {
                    continue;
                }
                found = true;
                distance = candidateDistance;
                normal = candidateNormal;
            }
        }

        if (!found)
            return false;

        if (radius == Fixed64.Zero && WideConvex2dRelations.TryGetSweptCenteredAxisContact(
                center, axisDirection, axisLength, convexOrigin, convexRotation, convexVertexOffsets,
                direction, out Vector2d entryNormal, out convexContactOffset))
        {
            normal = entryNormal;
            return true;
        }
        Vector2d translation = authoredDisplacement
            ? new FixedSegment2d(Vector2d.Zero, direction).GetPointAtDistance(distance, maximumDistance)
            : direction * distance;
        FixedPointAnchor2d sourceSupport =
            WideFiniteAxisIntersection.GetCenteredCapsuleSupportAnchor(
                center,
                Fixed64.Zero,
                axisDirection,
                axisLength,
                radius,
                -normal);
        convexContactOffset =
            WideConvex2dRelations.GetSweptAnchorTargetContactOffset(
            sourceSupport,
            translation,
            convexOrigin,
            convexRotation,
            convexVertexOffsets,
            normal);
        return true;
    }

    private static bool ShouldReplaceSweepCandidate(
        Fixed64 candidateDistance,
        Vector2d candidateNormal,
        Vector2d direction,
        bool found,
        Fixed64 bestDistance,
        Vector2d bestNormal) =>
        !found
        || candidateDistance < bestDistance
        || (candidateDistance == bestDistance
            && Vector2d.Dot(candidateNormal, direction)
                < Vector2d.Dot(bestNormal, direction));

    // A zero-radius crossing can round beyond the face, so its reconstructed normal needs motion orientation.
    private static Vector2d OrientZeroRadiusSweepNormal(Vector2d normal, Vector2d direction) =>
        Vector2d.Dot(normal, direction) > Fixed64.Zero ? -normal : normal;

    private static void ValidateSweep(
        Vector2d direction,
        Fixed64 maximumDistance)
    {
        if (!direction.IsNormalized())
            throw new ArgumentException(
                "Sweep direction must be normalized.",
                nameof(direction));
        if (maximumDistance < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(
                nameof(maximumDistance));
        }
    }

    private static void ValidateCenteredAxis(
        Vector2d axisDirection,
        Fixed64 axisLength)
    {
        if (!axisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Axis direction must be normalized.",
                nameof(axisDirection));
        }
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
    }
}
