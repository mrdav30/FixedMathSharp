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
    /// <param name="triangleOrigin">The triangle frame's origin in world coordinates.</param>
    /// <param name="triangleRotation">The triangle's normalized local-to-world rotation.</param>
    /// <param name="slabCenter">
    /// The slab center in world coordinates. X and Z locate its footprint and Y locates its midplane.
    /// </param>
    /// <param name="circleFrameRotation">
    /// The scalar rotation in radians defining the contact witness frame. The circular footprint is rotation
    /// invariant.
    /// </param>
    /// <param name="slabHalfThickness">The positive half-thickness along world Y.</param>
    /// <param name="circleRadius">The circle's nonnegative radius in coordinate units.</param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="triangleRotation"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="slabHalfThickness"/> is not positive or <paramref name="circleRadius"/> is negative.
    /// </exception>
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
        return TriangleCylinderContact.TryGetContact(this, triangleOrigin, triangleRotation,
            slabCenter, FixedQuaternion.FromAxisAngle(Vector3d.Up, -circleFrameRotation),
            WideArithmetic.AddSigned192(Signed192.Raw(slabHalfThickness), Signed192.Raw(slabHalfThickness)),
            circleRadius, out contact, out _);
    }

    /// <summary>
    /// Attempts to construct canonical contact anchors between this rigidly
    /// transformed triangle and one vertical centered-capsule slab.
    /// </summary>
    /// <param name="triangleOrigin">The triangle frame's origin in world coordinates.</param>
    /// <param name="triangleRotation">The triangle's normalized local-to-world rotation.</param>
    /// <param name="slabCenter">
    /// The slab center in world coordinates. X and Z locate its footprint and Y locates its midplane.
    /// </param>
    /// <param name="capsuleFrameRotation">
    /// The scalar rotation in radians applied to the local X/Z capsule axis and retained by the contact frame.
    /// </param>
    /// <param name="localCapsuleAxisDirection">The normalized capsule axis in local X/Z coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
    /// <param name="slabHalfThickness">The positive half-thickness along world Y.</param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="triangleRotation"/> is not normalized or <paramref name="localCapsuleAxisDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="capsuleAxisLength"/> is negative, <paramref name="capsuleRadius"/> is negative, or
    /// <paramref name="slabHalfThickness"/> is not positive.
    /// </exception>
    /// <remarks>
    /// The slab is a planar capsule extruded through flat Y caps, not a rounded
    /// 3D capsule. Closed contact includes exact tangency. The complete cap,
    /// straight-side and rounded-end feature set determines the minimum depth
    /// before contact coordinates are rounded. A zero core reduces to a circle
    /// slab; degenerate triangles return false.
    /// </remarks>
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
        return TriangleCylinderContact.TryGetContact(this, triangleOrigin, triangleRotation,
            slabCenter, FixedQuaternion.FromAxisAngle(Vector3d.Up, -capsuleFrameRotation),
            WideArithmetic.AddSigned192(Signed192.Raw(slabHalfThickness), Signed192.Raw(slabHalfThickness)),
            capsuleRadius, out contact, out _, localCapsuleAxisDirection, capsuleAxisLength);
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="pointOnTriangle">The selected point on the triangle in query coordinates on success.</param>
    /// <param name="pointOnAxis">The selected point on the central segment in query coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when both selected witnesses can be represented and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="axisLength"/> is negative.</exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the query finds closed overlap or tangency and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="fallbackNormal">The normalized world-space normal used when the closest features coincide.</param>
    /// <param name="pointOnTriangle">The selected point on the triangle in query coordinates when a contact is returned.</param>
    /// <param name="pointOnCapsule">The selected point on the capsule in query coordinates when a contact is returned.</param>
    /// <param name="normal">The world-space normal from the triangle toward the capsule when a contact is returned.</param>
    /// <param name="depth">The rounded penetration depth in coordinate units when a contact is returned.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="fallbackNormal"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The returned normal points from the triangle toward the capsule.
    /// <paramref name="fallbackNormal"/> is used only when the closest axis
    /// and triangle points coincide exactly. With a positive core length, the
    /// capsule witness is the first finite boundary exit from that witness
    /// opposite the selected normal. Its depth is this local ray distance,
    /// rather than a whole-triangle minimum translation. A false result means that the contact is
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
    /// <param name="apex">The cone apex in world coordinates.</param>
    /// <param name="apexToBaseDirection">The normalized world direction from the apex to the base center.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="point">The selected point on the intersection in query coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when a minimum-axial intersection witness can be returned and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="apexToBaseDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
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

        return WideTriangleConeIntersection.TryGetMinimumAxialPoint(this,
            Vector3d.Zero, FixedQuaternion.Identity, apex, apexToBaseDirection, height, baseRadius, out point);
    }

    /// <summary>
    /// Finds the minimum-axial cone intersection while retaining the triangle's
    /// rigid frame and the cone's world-space definition.
    /// </summary>
    /// <param name="triangleOrigin">The triangle frame's origin in world coordinates.</param>
    /// <param name="triangleRotation">The triangle's normalized local-to-world rotation.</param>
    /// <param name="apex">The cone apex in world coordinates.</param>
    /// <param name="apexToBaseDirection">The normalized world direction from the apex to the base center.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="point">The selected point on the intersection retained in the triangle's frame on success.</param>
    /// <returns>
    /// <see langword="true"/> when a minimum-axial intersection witness can be returned and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="triangleRotation"/> is not normalized or <paramref name="apexToBaseDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Neither the cone apex nor the triangle vertices must fit in the other's
    /// scalar coordinate frame. Classification uses exact rational rotation;
    /// only the selected triangle-local witness is rounded.
    /// </remarks>
    public bool TryGetFiniteConeIntersectionMinimumAxialPoint(
        Vector3d triangleOrigin, FixedQuaternion triangleRotation,
        Vector3d apex, Vector3d apexToBaseDirection, Fixed64 height, Fixed64 baseRadius,
        out FixedPointAnchor point)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateFiniteCone(apexToBaseDirection, height, baseRadius, nameof(apexToBaseDirection), nameof(height));
        bool found = WideTriangleConeIntersection.TryGetMinimumAxialPoint(this,
            triangleOrigin, triangleRotation, apex, apexToBaseDirection, height, baseRadius, out Vector3d localPoint);
        point = found ? FixedPointAnchor.FromValidatedFrame(triangleOrigin, triangleRotation, localPoint) : default;
        return found;
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
    /// <param name="triangleOrigin">The triangle frame's origin in world coordinates.</param>
    /// <param name="triangleRotation">The triangle's normalized local-to-world rotation.</param>
    /// <param name="circleCenter">The circle center in world X/Z coordinates.</param>
    /// <param name="circleRadius">The circle's nonnegative radius in coordinate units.</param>
    /// <param name="slabCenterY">The world Y coordinate of the slab midplane.</param>
    /// <param name="slabHalfThickness">The nonnegative half-thickness along world Y. Zero describes a planar slice.</param>
    /// <param name="triangleContact">The matched witness retained in the triangle frame when a contact is returned.</param>
    /// <returns>
    /// <see langword="true"/> when the circle overlaps the slab-clipped triangle projection and a witness is
    /// returned and <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="triangleRotation"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="circleRadius"/> is negative or <paramref name="slabHalfThickness"/> is negative.
    /// </exception>
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
    /// <param name="triangleOrigin">The triangle frame's origin in world coordinates.</param>
    /// <param name="triangleRotation">The triangle's normalized local-to-world rotation.</param>
    /// <param name="circleStart">The initial circle center in world X/Z coordinates.</param>
    /// <param name="normalizedDirection">The normalized direction of travel in world X/Z coordinates.</param>
    /// <param name="maximumDistance">The nonnegative maximum travel distance in coordinate units, including the endpoint.</param>
    /// <param name="circleRadius">The circle's nonnegative radius in coordinate units.</param>
    /// <param name="slabCenterY">The world Y coordinate of the slab midplane.</param>
    /// <param name="slabHalfThickness">The nonnegative half-thickness along world Y. Zero describes a planar slice.</param>
    /// <param name="distance">The first contact distance measured from the start on success.</param>
    /// <param name="triangleContact">The matched witness retained in the triangle frame on success.</param>
    /// <returns>
    /// <see langword="true"/> when a contact is found within the tested travel interval and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="triangleRotation"/> is not normalized or <paramref name="normalizedDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="maximumDistance"/> is negative, <paramref name="circleRadius"/> is negative, or
    /// <paramref name="slabHalfThickness"/> is negative.
    /// </exception>
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
    /// <summary>
    /// Finds the first distance along an authored X/Z circle-center segment
    /// that reaches this triangle portion inside one finite world-Y slab.
    /// </summary>
    /// <param name="triangleOrigin">The triangle frame's origin in world coordinates.</param>
    /// <param name="triangleRotation">The triangle's normalized local-to-world rotation.</param>
    /// <param name="segment">The authored center path in world X/Z coordinates.</param>
    /// <param name="totalDistance">
    /// The caller-supplied distance mapped to the full segment. Zero is allowed only for coincident endpoints.
    /// </param>
    /// <param name="circleRadius">The circle's nonnegative radius in coordinate units.</param>
    /// <param name="slabCenterY">The world Y coordinate of the slab midplane.</param>
    /// <param name="slabHalfThickness">The nonnegative half-thickness along world Y. Zero describes a planar slice.</param>
    /// <param name="distance">The first contact distance measured from the start on success.</param>
    /// <param name="triangleContact">The matched witness retained in the triangle frame on success.</param>
    /// <returns>
    /// <see langword="true"/> when a contact is found within the tested travel interval and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="triangleRotation"/> is not normalized or <paramref name="totalDistance"/> is zero for a
    /// nonzero-length segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="totalDistance"/> is negative, <paramref name="circleRadius"/> is negative, or
    /// <paramref name="slabHalfThickness"/> is negative.
    /// </exception>
    /// <remarks>
    /// Transformation, slab clipping and the authored segment remain exact;
    /// only the final physical distance and local triangle witness are rounded.
    /// </remarks>
    public readonly bool TryGetFiniteSlabProjectedCircleSweep(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedSegment2d segment,
        Fixed64 totalDistance,
        Fixed64 circleRadius,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        out Fixed64 distance,
        out FixedPointAnchor triangleContact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        _ = segment.GetPointAtDistance(Fixed64.Zero, totalDistance);
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        if (slabHalfThickness < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        return WideOrientedBox.TryGetFiniteSlabProjectedCircleSweep(
            this, triangleOrigin, triangleRotation, segment, totalDistance,
            circleRadius, slabCenterY, slabHalfThickness, out distance, out triangleContact);
    }

}
