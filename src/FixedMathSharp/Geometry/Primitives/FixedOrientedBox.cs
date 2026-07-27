//=======================================================================
// FixedOrientedBox.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Bounds;

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
public readonly partial struct FixedOrientedBox : IEquatable<FixedOrientedBox>
{
    /// <summary>
    /// The number of stable local corners exposed by <see cref="GetLocalCorner"/>.
    /// </summary>
    public const int CornerCount = 8;

    /// <summary>
    /// Initializes an oriented box from canonical geometry.
    /// </summary>
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
    [JsonInclude]
    public Vector3d Center { get; }

    /// <summary>
    /// The normalized local-to-world orientation.
    /// </summary>
    [JsonInclude]
    public FixedQuaternion Orientation { get; }

    /// <summary>
    /// The positive local face distances.
    /// </summary>
    [JsonInclude]
    public Vector3d HalfExtents { get; }

    /// <summary>
    /// Gets the nearest round-half-to-even <see cref="Fixed64"/> views of all
    /// three exact local-to-world axes.
    /// </summary>
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
    /// <remarks>
    /// Each coordinate retains the exact rational quaternion basis, all three
    /// products, and the center contribution until one final round-half-to-even
    /// conversion. The result is the nearest lattice representation of the
    /// conceptual point; a conceptual boundary point need not remain exactly on
    /// that boundary after quantization. Failure is atomic.
    /// </remarks>
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
    /// <remarks>
    /// All three quaternion-basis products are retained until one final
    /// round-half-to-even conversion per component. This is the canonical
    /// admission path for relative box features whose absolute world points
    /// may lie outside the scalar coordinate domain.
    /// </remarks>
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

    /// <summary>
    /// Determines whether two boxes have identical canonical state.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedOrientedBox left, FixedOrientedBox right) =>
        left.Equals(right);

    /// <summary>
    /// Determines whether two boxes have different canonical state.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedOrientedBox left, FixedOrientedBox right) =>
        !left.Equals(right);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) =>
        obj is FixedOrientedBox other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedOrientedBox other) =>
        Center.Equals(other.Center)
        && Orientation.Equals(other.Orientation)
        && HalfExtents.Equals(other.HalfExtents);

    /// <inheritdoc/>
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
}
