//=======================================================================
// FixedOrientedBox.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Represents an immutable oriented box through canonical center, orientation,
/// and positive local half-extents.
/// </summary>
/// <remarks>
/// World corners are intentionally not stored or exposed. Use local feature
/// selection and <see cref="TryMaterializeLocalPoint"/> when a representable
/// world-space witness is required. Geometry is evaluated against the exact
/// scale-invariant rational basis of the stored quaternion; rounded axes are
/// presentation values rather than query inputs.
/// </remarks>
public readonly struct FixedOrientedBox : IEquatable<FixedOrientedBox>
{
    /// <summary>
    /// The number of stable local corners exposed by <see cref="GetLocalCorner"/>.
    /// </summary>
    public const int CornerCount = 8;

    /// <summary>
    /// Initializes an oriented box from canonical geometry.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="orientation">The normalized quaternion that maps box-local axes to world coordinates.</param>
    /// <param name="halfExtents">The strictly positive X, Y, and Z half-extents in box-local coordinate units.</param>
    /// <exception cref="ArgumentException">
    /// <paramref name="orientation"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// At least one half-extent is not positive.
    /// </exception>
    [JsonConstructor]
    public FixedOrientedBox(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents)
    {
        if (!orientation.IsNormalized())
            throw new ArgumentException("The oriented-box rotation must be normalized.", nameof(orientation));
        if (!HasPositiveHalfExtents(halfExtents))
        {
            throw new ArgumentOutOfRangeException(
                nameof(halfExtents),
                "Every oriented-box half-extent must be positive.");
        }

        Center = center;
        Orientation = orientation;
        HalfExtents = halfExtents;
    }

    /// <summary>
    /// The world-space center.
    /// </summary>
    /// <value>The box center in world coordinates.</value>
    [JsonInclude]
    public Vector3d Center { get; }

    /// <summary>
    /// The normalized local-to-world orientation.
    /// </summary>
    /// <value>The normalized quaternion mapping box-local directions to world coordinates.</value>
    [JsonInclude]
    public FixedQuaternion Orientation { get; }

    /// <summary>
    /// The positive local face distances.
    /// </summary>
    /// <value>The positive half-extents measured along the box's local axes.</value>
    [JsonInclude]
    public Vector3d HalfExtents { get; }

    /// <summary>
    /// Gets the nearest round-half-to-even <see cref="Fixed64"/> views of all
    /// three exact local-to-world axes.
    /// </summary>
    /// <param name="axisX">The local X axis in world coordinates, rounded per component.</param>
    /// <param name="axisY">The local Y axis in world coordinates, rounded per component.</param>
    /// <param name="axisZ">The local Z axis in world coordinates, rounded per component.</param>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    public void GetAxes(
        out Vector3d axisX,
        out Vector3d axisY,
        out Vector3d axisZ)
    {
        EnsureValid();
        GetAxesUnchecked(out axisX, out axisY, out axisZ);
    }

    /// <summary>
    /// Gets a stable center-relative local corner.
    /// </summary>
    /// <param name="index">The zero-based corner index from 0 through 7.</param>
    /// <returns>The selected corner offset in box-local coordinates.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> is outside the supported range.</exception>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// Index bits select positive X, Y, and Z respectively. The order is
    /// <c>---</c>, <c>+--</c>, <c>-+-</c>, <c>++-</c>, <c>--+</c>,
    /// <c>+-+</c>, <c>-++</c>, <c>+++</c>.
    /// </remarks>
    public Vector3d GetLocalCorner(int index)
    {
        EnsureValid();
        if ((uint)index >= CornerCount)
        {
            throw new ArgumentOutOfRangeException(
                nameof(index),
                $"Corner index must be between 0 and {CornerCount - 1}.");
        }

        return new Vector3d(
            (index & 1) == 0 ? -HalfExtents.X : HalfExtents.X,
            (index & 2) == 0 ? -HalfExtents.Y : HalfExtents.Y,
            (index & 4) == 0 ? -HalfExtents.Z : HalfExtents.Z);
    }

    /// <summary>
    /// Gets the center-relative local support point for a world-space direction.
    /// </summary>
    /// <param name="worldDirection">
    /// The direction in world coordinates used to choose a support. Normalization is unnecessary.
    /// </param>
    /// <returns>The selected support corner in box-local coordinates, relative to the center.</returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// An exact zero projection selects the negative extent so ties retain the
    /// lower local corner index.
    /// </remarks>
    public Vector3d GetLocalSupportPoint(Vector3d worldDirection)
    {
        EnsureValid();
        return WideOrientedBox.GetLocalSupportPoint(
            worldDirection,
            Orientation,
            HalfExtents);
    }

    /// <summary>
    /// Gets the least-outward analytical axis-aligned bounds, clipped only at
    /// the final representable scalar endpoints.
    /// </summary>
    /// <returns>The outward-rounded world-axis-aligned bounds clipped to the representable coordinate range.</returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    public FixedBoundBox GetBoundsClippedToDomain()
    {
        EnsureValid();
        return WideOrientedBox.GetBoundsClippedToDomain(
            Center,
            Orientation,
            HalfExtents);
    }

    /// <summary>
    /// Returns whether a world-space point lies inside or on every conceptual
    /// box face.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <returns><see langword="true"/> when the point is inside or on the boundary and <see langword="false"/> otherwise.</returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    public bool Contains(Vector3d point)
    {
        EnsureValid();
        return WideOrientedBox.Contains(
            point,
            Center,
            Orientation,
            HalfExtents);
    }

    /// <summary>
    /// Gets the conceptual closest surface point in this box's rigid frame.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <returns>The selected point retained in the shape's local frame.</returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// Outside points clamp independently to the local extents. Inside points
    /// select the nearest face with stable X, then Y, then Z ties. The returned
    /// anchor remains valid when the selected absolute world point is outside
    /// the representable scalar domain.
    /// </remarks>
    public FixedPointAnchor GetClosestPointAnchor(Vector3d point)
    {
        EnsureValid();
        return WideOrientedBox.GetClosestPointAnchor(
            point,
            Center,
            Orientation,
            HalfExtents);
    }

    /// <summary>
    /// Attempts to materialize the nearest lattice representation of the
    /// conceptual closest surface point.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="closestPoint">The closest point on the conceptual surface in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// Outside points clamp independently to the local extents. Inside points
    /// select the nearest face with stable X, then Y, then Z ties. The method
    /// returns <see langword="false"/> only when the selected final world point
    /// is not representable.
    /// </remarks>
    public bool TryGetClosestPointOnSurface(
        Vector3d point,
        out Vector3d closestPoint)
        => GetClosestPointAnchor(point).TryGetPoint(out closestPoint);

    /// <summary>
    /// Gets the nearest representable normal of the nearest conceptual face.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <returns>The selected face's outward normal in world coordinates, rounded per component.</returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// Outside points select the first violated axis in X, then Y, then Z
    /// order, matching their nearest clamped surface feature. For contained
    /// points, equal inward face distances use the same axis order. A zero
    /// local projection selects the positive face.
    /// </remarks>
    public Vector3d GetNearestFaceNormal(Vector3d point)
    {
        EnsureValid();
        return WideOrientedBox.GetNearestFaceNormal(
            point,
            Center,
            Orientation,
            HalfExtents);
    }

    /// <summary>
    /// Attempts to materialize a center-relative local point in world space.
    /// </summary>
    /// <param name="localPoint">The point relative to the center, expressed along the box's local axes.</param>
    /// <param name="worldPoint">The rounded world point on success, or the default value on failure.</param>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// Each coordinate retains the exact rational quaternion basis, all three
    /// products, and the center contribution until one final round-half-to-even
    /// conversion. The result is the nearest lattice representation of the
    /// conceptual point; a conceptual boundary point need not remain exactly on
    /// that boundary after quantization. Failure is atomic.
    /// </remarks>
    /// <returns><see langword="true"/> when every final coordinate is representable and <see langword="false"/> otherwise.</returns>
    public bool TryMaterializeLocalPoint(
        Vector3d localPoint,
        out Vector3d worldPoint)
    {
        EnsureValid();
        return WideOrientedBox.TryMaterializeLocalPoint(
            Center,
            Orientation,
            localPoint,
            out worldPoint);
    }

    /// <summary>
    /// Attempts to rotate a center-relative local offset into world-space
    /// coordinates without adding <see cref="Center"/>.
    /// </summary>
    /// <param name="localOffset">The offset in box-local axes.</param>
    /// <param name="worldOffset">The rounded offset expressed along world axes on success, or the default value on failure.</param>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// All three quaternion-basis products are retained until one final
    /// round-half-to-even conversion per component. This is the canonical
    /// admission path for relative box features whose absolute world points
    /// may lie outside the scalar coordinate domain.
    /// </remarks>
    /// <returns><see langword="true"/> when every final coordinate is representable and <see langword="false"/> otherwise.</returns>
    public bool TryTransformLocalOffset(
        Vector3d localOffset,
        out Vector3d worldOffset)
    {
        EnsureValid();
        return WideOrientedBox.TryTransformLocalOffset(
            Orientation,
            localOffset,
            out worldOffset);
    }

    /// <summary>
    /// Attempts to return the center-relative world-space support offset for a
    /// direction.
    /// </summary>
    /// <param name="worldDirection">
    /// The direction in world coordinates used to choose a support. Normalization is unnecessary.
    /// </param>
    /// <param name="centerOffset">The support offset relative to the center and expressed along world axes on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// Exact zero projections select the negative local extent so ties retain
    /// the stable lower corner index.
    /// </remarks>
    public bool TryGetSupportOffset(
        Vector3d worldDirection,
        out Vector3d centerOffset)
    {
        EnsureValid();
        return WideOrientedBox.TryGetSupportOffset(
            worldDirection,
            Orientation,
            HalfExtents,
            out centerOffset);
    }

    /// <summary>
    /// Attempts to construct one exact-final-narrowing Minkowski support
    /// difference between this box and another origin-relative support.
    /// </summary>
    /// <param name="otherOrigin">The origin of the opposing support point in world coordinates.</param>
    /// <param name="otherOriginSupportOffset">
    /// The opposing support offset relative to its own origin and expressed along world axes.
    /// </param>
    /// <param name="worldDirection">
    /// The direction in world coordinates used to choose a support. Normalization is unnecessary.
    /// </param>
    /// <param name="difference">This box's support point minus the opposing support point in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// The result is
    /// <c>(Center + boxSupport) - (otherOrigin + otherOriginSupportOffset)</c>.
    /// Neither absolute support point is materialized independently.
    /// </remarks>
    public bool TryGetSupportDifference(
        Vector3d otherOrigin,
        Vector3d otherOriginSupportOffset,
        Vector3d worldDirection,
        out Vector3d difference)
    {
        EnsureValid();
        return WideOrientedBox.TryGetSupportDifference(
            Center,
            Orientation,
            HalfExtents,
            otherOrigin,
            otherOriginSupportOffset,
            worldDirection,
            out difference);
    }

    /// <summary>
    /// Attempts to construct the exact Minkowski support difference between
    /// this box and <paramref name="other"/>.
    /// </summary>
    /// <param name="other">The second shape in the same coordinate space.</param>
    /// <param name="worldDirection">
    /// The direction in world coordinates used to choose a support. Normalization is unnecessary.
    /// </param>
    /// <param name="difference">This box's support point minus the opposing support point in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">Either box does not contain valid canonical geometry.</exception>
    /// <remarks>
    /// Both rotated support offsets and the center difference are retained as
    /// rational wide values until one final narrowing per component.
    /// </remarks>
    public bool TryGetSupportDifference(
        FixedOrientedBox other,
        Vector3d worldDirection,
        out Vector3d difference)
    {
        EnsureValid();
        other.EnsureValid();
        return WideOrientedBox.TryGetSupportDifference(
            Center,
            Orientation,
            HalfExtents,
            other.Center,
            other.Orientation,
            other.HalfExtents,
            worldDirection,
            out difference);
    }

    /// <summary>
    /// Finds the closed parameter interval where a bounded ray overlaps this
    /// oriented box.
    /// </summary>
    /// <param name="ray">The ray in world coordinates. Its direction is used as supplied.</param>
    /// <param name="maxParameter">The nonnegative inclusive upper bound on the ray parameter.</param>
    /// <param name="entry">The entry ray parameter in the bounded interval on success.</param>
    /// <param name="exit">The exit ray parameter in the bounded interval on success.</param>
    /// <returns>
    /// <see langword="true"/> when the path intersects the closed shape within the tested interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxParameter"/> is negative.</exception>
    /// <remarks>
    /// World-to-local projections and slab clipping retain the exact rational
    /// quaternion basis. The direction need not be normalized.
    /// </remarks>
    public bool TryGetRayIntersectionInterval(
        FixedRay ray,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        EnsureValid();
        if (maxParameter < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(maxParameter));

        return WideOrientedBox.TryGetRayIntersectionInterval(
            Center,
            Orientation,
            HalfExtents,
            ray.Position,
            ray.Direction,
            maxParameter,
            out entry,
            out exit);
    }

    /// <summary>Tests whether the stored geometry is equal.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns>
    /// <see langword="true"/> when the stored centers, quaternion components, and half-extents match and
    /// <see langword="false"/> otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedOrientedBox left, FixedOrientedBox right) =>
        left.Equals(right);

    /// <summary>Tests whether the stored geometry is different.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the stored values differ and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedOrientedBox left, FixedOrientedBox right) =>
        !left.Equals(right);

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="obj">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when <paramref name="obj"/> is a <see cref="FixedOrientedBox"/> and the stored
    /// centers, quaternion components, and half-extents match and <see langword="false"/> otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) =>
        obj is FixedOrientedBox other && Equals(other);

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="other">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when the stored centers, quaternion components, and half-extents match and
    /// <see langword="false"/> otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedOrientedBox other) =>
        Center.Equals(other.Center)
        && Orientation.Equals(other.Orientation)
        && HalfExtents.Equals(other.HalfExtents);

    /// <summary>Computes a hash code for the stored geometry.</summary>
    /// <returns>A hash code consistent with structural equality.</returns>
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + Center.GetHashCode();
            hash = (hash * 31) + Orientation.GetHashCode();
            hash = (hash * 31) + HalfExtents.GetHashCode();
            return hash;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void GetAxesUnchecked(
        out Vector3d axisX,
        out Vector3d axisY,
        out Vector3d axisZ) =>
        WideOrientedBox.GetAxes(Orientation, out axisX, out axisY, out axisZ);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void EnsureValid()
    {
        if (!HasPositiveHalfExtents(HalfExtents))
        {
            throw new InvalidOperationException("The oriented box does not contain valid canonical geometry.");
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasPositiveHalfExtents(Vector3d halfExtents) =>
        halfExtents.X > Fixed64.Zero
        && halfExtents.Y > Fixed64.Zero
        && halfExtents.Z > Fixed64.Zero;

    /// <summary>
    /// Attempts to construct an exact contact against another oriented box.
    /// </summary>
    /// <param name="other">The second shape in the same coordinate space.</param>
    /// <param name="contact">The primary box-to-shape contact on success. The first witness is retained in the box frame.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">Either box does not contain valid canonical geometry.</exception>
    public bool TryGetContact(
        FixedOrientedBox other,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        other.EnsureValid();
        return WideOrientedBox.TryGetContact(
            Center,
            Orientation,
            HalfExtents,
            other.Center,
            other.Orientation,
            other.HalfExtents,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a nondegenerate
    /// triangle.
    /// </summary>
    /// <param name="triangleOrigin">
    /// The triangle's rigid-frame origin.
    /// </param>
    /// <param name="triangleRotation">The triangle's local-to-world rotation.</param>
    /// <param name="triangle">The triangle in its rigid-frame coordinates.</param>
    /// <param name="contact">
    /// The box-center-relative and triangle-origin-relative contact relation.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the shapes overlap and both relative
    /// contact anchors are representable; otherwise <see langword="false"/>.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException"><paramref name="triangleRotation"/> is not normalized.</exception>
    public bool TryGetTriangleContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!triangleRotation.IsNormalized())
        {
            throw new ArgumentException(
                "The triangle rotation must be normalized.",
                nameof(triangleRotation));
        }
        return WideOrientedBox.TryGetTriangleContact(
            Center,
            Orientation,
            HalfExtents,
            triangleOrigin,
            triangleRotation,
            triangle,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact triangle contact and, for a parallel
    /// box-face/triangle-face feature, clips stable box corners to the
    /// triangle.
    /// </summary>
    /// <param name="triangleOrigin">The triangle frame's origin in world coordinates.</param>
    /// <param name="triangleRotation">The triangle's normalized local-to-world rotation.</param>
    /// <param name="triangle">The triangle's three ordered vertices in its local frame.</param>
    /// <param name="faceContacts">
    /// Caller-owned storage with capacity for 4 pairs. Valid entries overwrite the prefix on success. The unused
    /// tail and the buffer on a miss are preserved.
    /// </param>
    /// <param name="contact">The primary box-to-shape contact on success. The first witness is retained in the box frame.</param>
    /// <param name="faceContactCount">The number of valid additional face-contact pairs, from zero through four.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException">
    /// <paramref name="triangleRotation"/> is not normalized or <paramref name="faceContacts"/> has fewer than four
    /// elements.
    /// </exception>
    /// <remarks>
    /// <paramref name="faceContacts"/> must provide capacity for four
    /// contacts. <paramref name="contact"/> always contains the primary exact
    /// contact relation when the shapes overlap. <paramref name="faceContactCount"/>
    /// is zero for non-face features.
    /// </remarks>
    public bool TryGetTriangleContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        Span<FixedContactLocalPoints> faceContacts,
        out FixedContactAnchors contact,
        out int faceContactCount)
    {
        EnsureValid();
        if (faceContacts.Length < 4)
        {
            throw new ArgumentException(
                "Triangle face-contact output requires capacity for four contacts.",
                nameof(faceContacts));
        }
        if (!triangleRotation.IsNormalized())
        {
            throw new ArgumentException(
                "The triangle rotation must be normalized.",
                nameof(triangleRotation));
        }
        if (!WideOrientedBox.TryGetTriangleContact(
                Center,
                Orientation,
                HalfExtents,
                triangleOrigin,
                triangleRotation,
                triangle,
                out contact))
        {
            faceContactCount = 0;
            return false;
        }

        WideOrientedBox.GetTriangleFaceContacts(
            Center,
            Orientation,
            HalfExtents,
            triangleOrigin,
            triangleRotation,
            triangle,
            contact,
            faceContacts,
            out faceContactCount);
        return true;
    }

    /// <summary>
    /// Attempts to construct an exact contact against a convex point span.
    /// </summary>
    /// <param name="hullOrigin">The hull frame's origin in world coordinates.</param>
    /// <param name="hullOrientation">The normalized rotation from hull-local coordinates to world coordinates.</param>
    /// <param name="hullLocalOffsets">
    /// At least four origin-relative hull vertices in local coordinates. The span is read during the call.
    /// </param>
    /// <param name="triangleVertexIndices">
    /// A nonempty sequence of index triples into the corresponding hull vertex span, one triple per face triangle.
    /// </param>
    /// <param name="edgeVertexPairs">The hull edge topology as pairs of indices into the corresponding hull vertex span.</param>
    /// <param name="contact">The primary box-to-shape contact on success. The first witness is retained in the box frame.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException">
    /// <paramref name="hullOrientation"/> is not normalized, <paramref name="hullLocalOffsets"/> has fewer than
    /// four vertices, <paramref name="triangleVertexIndices"/> is empty or has an incomplete triple, or
    /// <paramref name="edgeVertexPairs"/> is empty or has an incomplete pair.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">A topology index is outside its vertex span.</exception>
    /// <remarks>
    /// Hull points are origin-relative offsets expressed in the hull's local
    /// orientation. Triangle and edge indices provide the complete convex SAT
    /// topology. The method performs no absolute world-point materialization.
    /// </remarks>
    public bool TryGetConvexHullContact(
        Vector3d hullOrigin,
        FixedQuaternion hullOrientation,
        ReadOnlySpan<Vector3d> hullLocalOffsets,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!hullOrientation.IsNormalized())
        {
            throw new ArgumentException(
                "The convex-hull rotation must be normalized.",
                nameof(hullOrientation));
        }
        if (hullLocalOffsets.Length < 4)
        {
            throw new ArgumentException(
                "A convex hull requires at least four origin-relative points.",
                nameof(hullLocalOffsets));
        }
        if (triangleVertexIndices.Length == 0
            || triangleVertexIndices.Length % FixedTriangle.VertexCount != 0)
        {
            throw new ArgumentException(
                "Convex-hull triangle topology must be a nonempty multiple of three.",
                nameof(triangleVertexIndices));
        }
        if (edgeVertexPairs.Length == 0
            || (edgeVertexPairs.Length & 1) != 0)
        {
            throw new ArgumentException(
                "Convex-hull edge topology must contain complete vertex-index pairs.",
                nameof(edgeVertexPairs));
        }
        ValidateConvexHullIndices(
            hullLocalOffsets.Length,
            triangleVertexIndices,
            nameof(triangleVertexIndices));
        ValidateConvexHullIndices(
            hullLocalOffsets.Length,
            edgeVertexPairs,
            nameof(edgeVertexPairs));
        return WideOrientedBox.TryGetConvexHullContact(
            Center,
            Orientation,
            HalfExtents,
            hullOrigin,
            hullOrientation,
            hullLocalOffsets,
            triangleVertexIndices,
            edgeVertexPairs,
            out contact);
    }

    private static void ValidateConvexHullIndices(
        int vertexCount,
        ReadOnlySpan<int> indices,
        string parameterName)
    {
        for (int index = 0; index < indices.Length; index++)
        {
            if ((uint)indices[index] >= (uint)vertexCount)
            {
                throw new ArgumentOutOfRangeException(
                    parameterName,
                    "Convex-hull topology contains an invalid vertex index.");
            }
        }
    }

    /// <summary>
    /// Attempts to construct an exact contact against a centered 3D capsule.
    /// </summary>
    /// <param name="capsuleCenter">The capsule center in the query coordinate space.</param>
    /// <param name="capsuleRotation">The capsule's normalized local-to-world rotation.</param>
    /// <param name="localCapsuleAxisDirection">The normalized capsule axis in shape-local coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
    /// <param name="contact">The primary box-to-shape contact on success. The first witness is retained in the box frame.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException">
    /// <paramref name="capsuleRotation"/> is not normalized or <paramref name="localCapsuleAxisDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="capsuleAxisLength"/> is negative or <paramref name="capsuleRadius"/> is negative.
    /// </exception>
    public bool TryGetCenteredCapsuleContact(
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!capsuleRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Capsule rotation must be normalized.",
                nameof(capsuleRotation));
        }
        if (!localCapsuleAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Local capsule axis direction must be normalized.",
                nameof(localCapsuleAxisDirection));
        }
        if (capsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleAxisLength));
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        return WideOrientedBox.TryGetCenteredCapsuleContact(
            Center,
            Orientation,
            HalfExtents,
            capsuleCenter,
            capsuleRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a centered finite
    /// cylinder.
    /// </summary>
    /// <param name="cylinderCenter">The cylinder center in the query coordinate space.</param>
    /// <param name="cylinderRotation">The cylinder's normalized local-to-world rotation.</param>
    /// <param name="localCylinderAxisDirection">The normalized cylinder axis in shape-local coordinates.</param>
    /// <param name="cylinderAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="cylinderRadius">The cylinder's nonnegative radius in coordinate units.</param>
    /// <param name="contact">The primary box-to-shape contact on success. The first witness is retained in the box frame.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException">
    /// <paramref name="cylinderRotation"/> is not normalized or <paramref name="localCylinderAxisDirection"/> is
    /// not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="cylinderAxisLength"/> is not positive or <paramref name="cylinderRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Includes cap, side and rim constraints when selecting the minimum
    /// translation. Tangency is a contact with zero depth. The selected normal
    /// and complete depth are rounded only after exact classification and
    /// ordering; depth uses nearest-even rounding. A zero radius is a finite
    /// axis segment. Oblique rims require more work than parallel cap contacts.
    /// </remarks>
    public bool TryGetCenteredCylinderContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        ValidateCenteredCylinder(
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius);
        return WideOrientedBox.TryGetCenteredCylinderContact(
            Center,
            Orientation,
            HalfExtents,
            cylinderCenter,
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a centered finite
    /// cylinder and, for a parallel cap-to-face feature, clips a stable
    /// allocation-free contact set from the disk/rectangle intersection.
    /// </summary>
    /// <param name="cylinderCenter">The cylinder center in the query coordinate space.</param>
    /// <param name="cylinderRotation">The cylinder's normalized local-to-world rotation.</param>
    /// <param name="localCylinderAxisDirection">The normalized cylinder axis in shape-local coordinates.</param>
    /// <param name="cylinderAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="cylinderRadius">The cylinder's nonnegative radius in coordinate units.</param>
    /// <param name="capFaceContacts">
    /// Caller-owned storage with capacity for 4 pairs. Valid entries overwrite the prefix on success. The unused
    /// tail and the buffer on a miss are preserved.
    /// </param>
    /// <param name="contact">The primary box-to-shape contact on success. The first witness is retained in the box frame.</param>
    /// <param name="capFaceContactCount">The number of valid cap-to-face pairs, from zero through four.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException">
    /// <paramref name="cylinderRotation"/> is not normalized, <paramref name="localCylinderAxisDirection"/> is not
    /// normalized, or <paramref name="capFaceContacts"/> has fewer than four elements.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="cylinderAxisLength"/> is not positive or <paramref name="cylinderRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// <paramref name="capFaceContacts"/> must provide capacity for four
    /// contacts. <paramref name="contact"/> always contains the primary exact
    /// contact relation when the shapes overlap. <paramref name="capFaceContactCount"/>
    /// is zero for non-cap-face features, so callers should use the primary
    /// contact in that case.
    /// </remarks>
    public bool TryGetCenteredCylinderContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Span<FixedContactLocalPoints> capFaceContacts,
        out FixedContactAnchors contact,
        out int capFaceContactCount)
    {
        EnsureValid();
        ValidateCenteredCylinder(
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius);
        if (capFaceContacts.Length < 4)
        {
            throw new ArgumentException(
                "Cylinder cap-face contact output requires capacity for four contacts.",
                nameof(capFaceContacts));
        }

        if (!WideOrientedBox.TryGetCenteredCylinderContact(
                Center,
                Orientation,
                HalfExtents,
                cylinderCenter,
                cylinderRotation,
                localCylinderAxisDirection,
                cylinderAxisLength,
                cylinderRadius,
                out contact,
                out CenteredCylinderContactFeature feature))
        {
            capFaceContactCount = 0;
            return false;
        }

        WideOrientedBox.GetCenteredCylinderCapFaceContacts(
            Center,
            Orientation,
            HalfExtents,
            cylinderCenter,
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            contact,
            feature,
            capFaceContacts,
            out capFaceContactCount);
        return true;
    }

    private static void ValidateCenteredCylinder(
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius)
    {
        if (!cylinderRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Cylinder rotation must be normalized.",
                nameof(cylinderRotation));
        }
        if (!localCylinderAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Local cylinder axis direction must be normalized.",
                nameof(localCylinderAxisDirection));
        }
        if (cylinderAxisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(cylinderAxisLength));
        if (cylinderRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(cylinderRadius));
    }

    /// <summary>
    /// Attempts to construct an exact box-to-sphere contact using only
    /// center-relative contact offsets.
    /// </summary>
    /// <param name="sphereCenter">The sphere center.</param>
    /// <param name="sphereRotation">
    /// The normalized rigid frame used to retain stable sphere-local contact
    /// feature identity.
    /// </param>
    /// <param name="sphereRadius">The nonnegative sphere radius.</param>
    /// <param name="contact">The relative contact relation when overlapping.</param>
    /// <returns>
    /// <see langword="true"/> when the conceptual shapes overlap and both
    /// required relative offsets are representable.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException"><paramref name="sphereRotation"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="sphereRadius"/> is negative.</exception>
    public bool TryGetSphereContact(
        Vector3d sphereCenter,
        FixedQuaternion sphereRotation,
        Fixed64 sphereRadius,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!sphereRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Sphere rotation must be normalized.",
                nameof(sphereRotation));
        }
        if (sphereRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphereRadius));
        return WideOrientedBox.TryGetSphereContact(
            Center,
            Orientation,
            HalfExtents,
            sphereCenter,
            sphereRotation,
            sphereRadius,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a rotated convex polygon
    /// extruded through a closed world-Y slab.
    /// </summary>
    /// <param name="prismOrigin">The representable prism origin.</param>
    /// <param name="prismRotation">
    /// The prism's counterclockwise X/Z rotation in radians.
    /// </param>
    /// <param name="prismLocalOffsets">
    /// Ordered clockwise or counterclockwise X/Z boundary offsets in the
    /// prism's local frame.
    /// </param>
    /// <param name="prismHalfThickness">The positive world-Y half-thickness.</param>
    /// <param name="contact">The frame-relative contact anchors when overlapping.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException"><paramref name="prismLocalOffsets"/> has fewer than three vertices.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="prismHalfThickness"/> is not positive.</exception>
    public bool TryGetConvexPrismContact(
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (prismLocalOffsets.Length < 3)
        {
            throw new ArgumentException(
                "A convex prism requires at least three ordered boundary offsets.",
                nameof(prismLocalOffsets));
        }
        if (prismHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(prismHalfThickness));
        return WideOrientedBox.TryGetConvexPrismContact(
            Center,
            Orientation,
            HalfExtents,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a vertical circular
    /// prism centered in X/Z and extruded through a closed world-Y slab.
    /// </summary>
    /// <param name="slabCenter">
    /// The slab center in world coordinates. X and Z locate its footprint and Y locates its midplane.
    /// </param>
    /// <param name="circleFrameRotation">
    /// The scalar rotation in radians defining the contact witness frame. The circular footprint is rotation
    /// invariant.
    /// </param>
    /// <param name="slabHalfThickness">The positive half-thickness along world Y.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="contact">The primary box-to-shape contact on success. The first witness is retained in the box frame.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="slabHalfThickness"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
    public bool TryGetCircleSlabContact(
        Vector3d slabCenter,
        Fixed64 circleFrameRotation,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        return WideOrientedBox.TryGetCircleSlabContact(
            Center,
            Orientation,
            HalfExtents,
            slabCenter,
            circleFrameRotation,
            slabHalfThickness,
            radius,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a centered planar
    /// capsule extruded through a closed world-Y slab.
    /// </summary>
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
    /// <param name="contact">The primary box-to-shape contact on success. The first witness is retained in the box frame.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException"><paramref name="localCapsuleAxisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="capsuleAxisLength"/> is negative, <paramref name="capsuleRadius"/> is negative, or
    /// <paramref name="slabHalfThickness"/> is not positive.
    /// </exception>
    public bool TryGetCenteredCapsuleSlabContact(
        Vector3d slabCenter,
        Fixed64 capsuleFrameRotation,
        Vector2d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!localCapsuleAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Local capsule axis direction must be normalized.",
                nameof(localCapsuleAxisDirection));
        }
        if (capsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleAxisLength));
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        return WideOrientedBox.TryGetCenteredCapsuleSlabContact(
            Center,
            Orientation,
            HalfExtents,
            slabCenter,
            capsuleFrameRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            slabHalfThickness,
            out contact);
    }

    /// <summary>
    /// Finds the first X/Z travel distance at which a circle extruded through
    /// a fixed world-Y slab intersects this oriented box.
    /// </summary>
    /// <param name="slabStartCenter">The initial slab center in world coordinates. The sweep keeps its Y coordinate fixed.</param>
    /// <param name="normalizedDirection">The normalized direction of travel in world X/Z coordinates.</param>
    /// <param name="maxDistance">The nonnegative maximum travel distance in coordinate units, including the endpoint.</param>
    /// <param name="slabHalfThickness">The positive half-thickness along world Y.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="distance">The first contact distance measured from the start on success.</param>
    /// <returns>
    /// <see langword="true"/> when a contact is found within the tested travel interval and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException"><paramref name="normalizedDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="maxDistance"/> is negative, <paramref name="slabHalfThickness"/> is zero or negative, or
    /// <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The slab-clipped box projection remains an internal wide rational
    /// relation. No box corner, slab intersection, or projected polygon must
    /// be representable. Tangency and start overlap are inclusive, and the
    /// returned distance uses round-half-to-even.
    /// </remarks>
    public bool TryGetCircleSlabSweepDistance(
        Vector3d slabStartCenter,
        Vector2d normalizedDirection,
        Fixed64 maxDistance,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        out Fixed64 distance)
    {
        EnsureValid();
        if (!normalizedDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Sweep direction must be normalized.",
                nameof(normalizedDirection));
        }
        if (maxDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(maxDistance));
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        return WideOrientedBox.TryGetCircleSlabSweepDistance(
            Center,
            Orientation,
            HalfExtents,
            slabStartCenter,
            normalizedDirection,
            maxDistance,
            slabHalfThickness,
            radius,
            out distance);
    }

    /// <summary>
    /// Finds the first distance along an authored X/Z segment where a circle
    /// extruded through a fixed world-Y slab intersects this box.
    /// </summary>
    /// <param name="segment">The authored center path in world X/Z coordinates.</param>
    /// <param name="totalDistance">
    /// The caller-supplied distance mapped to the full segment. Zero is allowed only for coincident endpoints.
    /// </param>
    /// <param name="slabCenterY">The world Y coordinate of the slab midplane.</param>
    /// <param name="slabHalfThickness">The positive half-thickness along world Y.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="distance">The first contact distance, expressed on the authored distance scale on success.</param>
    /// <returns>
    /// <see langword="true"/> when a contact is found within the tested travel interval and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentException"><paramref name="totalDistance"/> is zero for a nonzero-length segment.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="totalDistance"/> is negative, <paramref name="slabHalfThickness"/> is zero or negative, or
    /// <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The segment is retained exactly without normalizing its displacement.
    /// Its interval maps to [0, <paramref name="totalDistance"/>] with one
    /// final round-half-to-even distance conversion.
    /// </remarks>
    public bool TryGetCircleSlabSweepDistance(
        FixedSegment2d segment,
        Fixed64 totalDistance,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        out Fixed64 distance)
    {
        EnsureValid();
        _ = segment.GetPointAtDistance(Fixed64.Zero, totalDistance);
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        return WideOrientedBox.TryGetCircleSlabSweepDistance(
            Center, Orientation, HalfExtents, segment, totalDistance,
            slabCenterY, slabHalfThickness, radius, out distance);
    }

    /// <summary>
    /// Returns a conservative X/Z separation lower bound between this box and
    /// a circle extruded through a fixed world-Y slab.
    /// </summary>
    /// <param name="slabCenter">
    /// The slab center in world coordinates. X and Z locate its footprint and Y locates its midplane.
    /// </param>
    /// <param name="slabHalfThickness">The positive half-thickness along world Y.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <returns>A conservative lower bound in coordinate units. A zero bound does not establish contact.</returns>
    /// <exception cref="InvalidOperationException">This box does not contain valid canonical geometry.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="slabHalfThickness"/> is zero or negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Projected edge witnesses are certified against every rational
    /// half-plane. When the closest feature is a projected vertex, the result
    /// uses the full-domain Euclidean distance floor before subtracting the
    /// circle radius. The result never overestimates true projected
    /// separation. When the Y intervals are disjoint, it returns the
    /// conservative vertical interval gap instead.
    /// </remarks>
    public Fixed64 GetCircleSlabSeparationLowerBound(
        Vector3d slabCenter,
        Fixed64 slabHalfThickness,
        Fixed64 radius)
    {
        EnsureValid();
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        return WideOrientedBox.GetCircleSlabSeparationLowerBound(
            Center,
            Orientation,
            HalfExtents,
            slabCenter,
            slabHalfThickness,
            radius);
    }
}
