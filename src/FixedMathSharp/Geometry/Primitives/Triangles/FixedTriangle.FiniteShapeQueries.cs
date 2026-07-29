//=======================================================================
// FixedTriangle.FiniteShapeQueries.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains finite-axis, finite-cone, and finite-slab contact and sweep
/// queries for rigidly transformed triangles.
/// </content>
public partial struct FixedTriangle
{
    /// <summary>
    /// Attempts to construct canonical contact anchors between this rigidly
    /// transformed triangle and one vertical circle slab.
    /// </summary>
    public readonly bool TryGetCircleSlabContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d slabCenter,
        Fixed64 circleFrameRotation,
        Fixed64 slabHalfThickness,
        Fixed64 circleRadius,
        out FixedContactAnchors contact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateCapsuleSlab(
            Vector2d.Right,
            Fixed64.Zero,
            circleRadius,
            slabHalfThickness);
        return WideOrientedBox.TryGetTriangleCapsuleSlabContact(
            triangleOrigin,
            triangleRotation,
            this,
            slabCenter,
            circleFrameRotation,
            Vector2d.Right,
            Fixed64.Zero,
            circleRadius,
            slabHalfThickness,
            out contact);
    }

    /// <summary>
    /// Attempts to construct canonical contact anchors between this rigidly
    /// transformed triangle and one vertical centered-capsule slab.
    /// </summary>
    public readonly bool TryGetCenteredCapsuleSlabContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d slabCenter,
        Fixed64 capsuleFrameRotation,
        Vector2d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness,
        out FixedContactAnchors contact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateCapsuleSlab(
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            slabHalfThickness);
        return WideOrientedBox.TryGetTriangleCapsuleSlabContact(
            triangleOrigin,
            triangleRotation,
            this,
            slabCenter,
            capsuleFrameRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            slabHalfThickness,
            out contact);
    }

    private static void ValidateCapsuleSlab(
        Vector2d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 halfThickness)
    {
        if (!localAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Capsule axis direction must be normalized.",
                nameof(localAxisDirection));
        }
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (halfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(halfThickness));
    }

    /// <summary>
    /// Attempts to return the closest points on this triangle and a conceptual
    /// centered finite axis without materializing its endpoints.
    /// </summary>
    public readonly bool TryGetClosestPointsToCenteredAxis(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        out Vector3d pointOnTriangle,
        out Vector3d pointOnAxis)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        return WideFiniteAxisIntersection.TryGetClosestPointsToCenteredAxis(
            this,
            center,
            axisDirection,
            axisLength,
            out pointOnTriangle,
            out pointOnAxis);
    }

    /// <summary>
    /// Returns whether this triangle inclusively overlaps a conceptual centered
    /// capsule without materializing either contact witness.
    /// </summary>
    public readonly bool DoesCenteredCapsuleOverlap(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.DoesCenteredCapsuleTriangleOverlap(
            this,
            center,
            axisDirection,
            axisLength,
            radius);
    }

    /// <summary>
    /// Attempts to create an inclusive contact between this triangle and a
    /// conceptual centered capsule.
    /// </summary>
    /// <remarks>
    /// The returned normal points from the triangle toward the capsule.
    /// <paramref name="fallbackNormal"/> is used only when the closest axis
    /// and triangle points coincide. A false result means that the contact is
    /// separated or that at least one final output is not representable; use
    /// <see cref="DoesCenteredCapsuleOverlap"/> when classification must remain
    /// independent of witness materialization.
    /// </remarks>
    public readonly bool TryGetCenteredCapsuleContact(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackNormal,
        out Vector3d pointOnTriangle,
        out Vector3d pointOnCapsule,
        out Vector3d normal,
        out Fixed64 depth)
    {
        ValidateCenteredCapsuleContact(
            axisDirection,
            axisLength,
            radius,
            fallbackNormal);

        return WideFiniteAxisIntersection.TryGetCenteredCapsuleTriangleContact(
            this,
            center,
            axisDirection,
            axisLength,
            radius,
            fallbackNormal,
            out pointOnTriangle,
            out pointOnCapsule,
            out normal,
            out depth);
    }

    private static void ValidateCenteredCapsuleContact(
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackNormal)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!fallbackNormal.IsNormalized())
            throw new ArgumentException("Fallback normal must be normalized.", nameof(fallbackNormal));
    }

    private static void ValidateCenteredAxis(
        Vector3d axisDirection,
        Fixed64 axisLength)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Axis direction must be normalized.", nameof(axisDirection));
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
    }

    /// <summary>
    /// Finds the intersecting point with the smallest axial parameter in an
    /// apex-authored finite cone.
    /// </summary>
    /// <remarks>
    /// Boundary candidates are visited in AB, BC, CA, face order. Triangle-face
    /// roots use a <see cref="Fixed64.MaxValue"/>-scaled parameter lattice;
    /// exact wide predicates select the witness before one final point rounding.
    /// </remarks>
    public bool TryGetFiniteConeIntersectionMinimumAxialPoint(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Vector3d point)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));

        bool found = false;
        Vector3d bestPoint = default;
        Fixed64 bestAxial = Fixed64.MaxValue;
        KeepEdgeCandidate(GetEdge(0), apex, apexToBaseDirection, height, baseRadius, ref found, ref bestPoint, ref bestAxial);
        KeepEdgeCandidate(GetEdge(1), apex, apexToBaseDirection, height, baseRadius, ref found, ref bestPoint, ref bestAxial);
        KeepEdgeCandidate(GetEdge(2), apex, apexToBaseDirection, height, baseRadius, ref found, ref bestPoint, ref bestAxial);

        GetExactNormal(out Signed192 normalX, out Signed192 normalY, out Signed192 normalZ, out Signed320 normalSquared);
        if (!WideGeometry.IsQ128MagnitudeAtMostEpsilon(normalSquared)
            && WideTriangleConeIntersection.TryGetFaceMinimumAxialPoint(
                this,
                normalX,
                normalY,
                normalZ,
                normalSquared,
                apex,
                apexToBaseDirection,
                height,
                baseRadius,
                found,
                out Vector3d facePoint,
                out Fixed64 faceAxial)
            && (!found || faceAxial < bestAxial))
        {
            found = true;
            bestPoint = facePoint;
        }

        point = bestPoint;
        return found;
    }

    private static void KeepEdgeCandidate(
        FixedSegment edge,
        Vector3d apex,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        ref bool found,
        ref Vector3d bestPoint,
        ref Fixed64 bestAxial)
    {
        if (!edge.TryGetFiniteConeIntersectionMinimumAxialPoint(
                apex,
                axisDirection,
                height,
                baseRadius,
                out Vector3d candidate))
        {
            return;
        }

        Fixed64 axial = FixedMath.Min(
            Vector3d.ProjectNonNegativeDifferenceParameter(candidate, apex, axisDirection),
            height);
        if (found && axial >= bestAxial)
            return;

        found = true;
        bestPoint = candidate;
        bestAxial = axial;
    }

    private static void ValidateFiniteCone(
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        string axisParameterName,
        string heightParameterName)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cone axis direction must be normalized.", axisParameterName);
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(heightParameterName);
        if (baseRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(baseRadius));
    }

    /// <summary>
    /// Attempts to find a triangle witness where a world-X/Z circle overlaps
    /// the triangle portion inside one finite world-Y slab.
    /// </summary>
    public readonly bool TryGetFiniteSlabProjectedCircleContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        out FixedPointAnchor triangleContact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        if (slabHalfThickness < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));

        return WideOrientedBox.TryGetFiniteSlabProjectedCircleSweep(
            this,
            triangleOrigin,
            triangleRotation,
            circleCenter,
            Vector2d.Right,
            Fixed64.Zero,
            circleRadius,
            slabCenterY,
            slabHalfThickness,
            out _,
            out triangleContact);
    }

    /// <summary>
    /// Finds the first distance where a circle swept in world X/Z reaches the
    /// triangle portion inside one finite world-Y slab.
    /// </summary>
    /// <remarks>
    /// Triangle transformation, slab clipping, and time-of-impact
    /// classification remain exact until the final Q32.32 distance and local
    /// triangle witness are rounded.
    /// </remarks>
    public readonly bool TryGetFiniteSlabProjectedCircleSweep(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector2d circleStart,
        Vector2d normalizedDirection,
        Fixed64 maximumDistance,
        Fixed64 circleRadius,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        out Fixed64 distance,
        out FixedPointAnchor triangleContact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        if (!normalizedDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Sweep direction must be normalized.",
                nameof(normalizedDirection));
        }
        if (maximumDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(maximumDistance));
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        if (slabHalfThickness < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));

        return WideOrientedBox.TryGetFiniteSlabProjectedCircleSweep(
            this,
            triangleOrigin,
            triangleRotation,
            circleStart,
            normalizedDirection,
            maximumDistance,
            circleRadius,
            slabCenterY,
            slabHalfThickness,
            out distance,
            out triangleContact);
    }
}
