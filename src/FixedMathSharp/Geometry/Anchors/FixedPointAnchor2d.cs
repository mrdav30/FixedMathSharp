//=======================================================================
// FixedPointAnchor2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Identifies a conceptual 2D point by an origin, scalar rotation, and local
/// point without requiring the transformed point to be representable.
/// </summary>
/// <remarks>
/// Relation-produced anchors can retain an exact sub-lattice feature term that
/// is intentionally hidden behind the narrow comparison and materialization
/// APIs. Anchors with identical rounded public local components can therefore
/// remain distinct when they identify different exact features.
/// </remarks>
public readonly struct FixedPointAnchor2d : IEquatable<FixedPointAnchor2d>
{
    /// <summary>
    /// Gets the point frame's world-space origin.
    /// </summary>
    /// <value>The stored origin in world coordinates.</value>
    public Vector2d Origin { get; }

    /// <summary>
    /// Gets the point frame's counterclockwise rotation in radians.
    /// </summary>
    /// <value>The counterclockwise local-to-world angle in radians.</value>
    public Fixed64 Rotation { get; }

    /// <summary>
    /// Gets the point in its supplied local frame.
    /// </summary>
    /// <value>The first point component in local coordinates.</value>
    public Vector2d LocalPoint { get; }

    /// <summary>
    /// An additional local-space feature displacement that exact operations
    /// add to <see cref="LocalPoint"/> without a scalar intermediate.
    /// </summary>
    /// <value>The second point component in local coordinates, applied additively.</value>
    public Vector2d LocalDisplacement { get; }

    /// <summary>
    /// Creates a transformed point anchor.
    /// </summary>
    /// <param name="origin">The frame origin in world coordinates.</param>
    /// <param name="rotation">The counterclockwise rotation from local coordinates to world coordinates, in radians.</param>
    /// <param name="localPoint">The first local component of the point.</param>
    public FixedPointAnchor2d(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d localPoint)
        : this(origin, rotation, localPoint, Vector2d.Zero)
    { }

    /// <summary>
    /// Creates a transformed point anchor from two additive local-space
    /// feature components.
    /// </summary>
    /// <param name="origin">The frame origin in world coordinates.</param>
    /// <param name="rotation">The counterclockwise rotation from local coordinates to world coordinates, in radians.</param>
    /// <param name="localPoint">The first local component of the point.</param>
    /// <param name="localDisplacement">An additional local displacement retained separately from the point.</param>
    public FixedPointAnchor2d(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d localPoint,
        Vector2d localDisplacement)
        : this(
            origin,
            rotation,
            localPoint,
            localDisplacement,
            default)
    { }

    internal FixedPointAnchor2d(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d localPoint,
        Vector2d localDisplacement,
        FixedPointAnchorTerm2d exactLocalTerm)
    {
        Origin = origin;
        Rotation = rotation;
        LocalPoint = localPoint;
        LocalDisplacement = localDisplacement;
        ExactLocalTerm = exactLocalTerm;
    }

    internal FixedPointAnchorTerm2d ExactLocalTerm { get; }

    /// <summary>
    /// Attempts to materialize the conceptual world-space point with one
    /// final round-half-to-even conversion per component.
    /// </summary>
    /// <param name="point">The rounded world point on success, or the default value on failure.</param>
    /// <returns>
    /// <see langword="true"/> when the final rounded coordinates are representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    public bool TryGetPoint(out Vector2d point) =>
        WidePointAnchor2d.TryGetPoint(
            Origin,
            LocalPoint,
            LocalDisplacement,
            ExactLocalTerm,
            Rotation,
            out point);

    /// <summary>
    /// Attempts to obtain this point's exact offset from another transformed
    /// point without materializing either world-space point.
    /// </summary>
    /// <param name="other">The anchor to subtract, with its own frame.</param>
    /// <param name="offset">The world-space difference on success, or the default value on failure.</param>
    /// <returns>
    /// <see langword="true"/> when the final rounded coordinates are representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    public bool TryGetOffsetFrom(
        in FixedPointAnchor2d other,
        out Vector2d offset) =>
        WidePointAnchor2d.TryGetRelativeOffset(
            Origin,
            LocalPoint,
            LocalDisplacement,
            ExactLocalTerm,
            Rotation,
            other.Origin,
            other.LocalPoint,
            other.LocalDisplacement,
            other.ExactLocalTerm,
            other.Rotation,
            out offset);

    /// <summary>
    /// Compares the exact squared distance from this point to two other
    /// anchored points.
    /// </summary>
    /// <returns>
    /// A negative value when <paramref name="first"/> is closer, zero when the
    /// distances are equal, or a positive value when
    /// <paramref name="second"/> is closer.
    /// </returns>
    /// <param name="first">The first candidate point.</param>
    /// <param name="second">The second candidate point.</param>
    public int CompareSquaredDistance(
        in FixedPointAnchor2d first,
        in FixedPointAnchor2d second) =>
        WidePointAnchor2d.CompareSquaredDistances(
            this,
            first,
            second);

    /// <summary>
    /// Attempts to express this conceptual point in another rotated frame
    /// without materializing the world-space point.
    /// </summary>
    /// <param name="frameOrigin">The new frame's origin in world coordinates.</param>
    /// <param name="frameRotation">The new frame's counterclockwise local-to-world rotation in radians.</param>
    /// <param name="localPoint">The rounded point in the target frame on success, or the default value on failure.</param>
    /// <returns>
    /// <see langword="true"/> when the final rounded coordinates are representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    public bool TryGetLocalPointIn(
        Vector2d frameOrigin,
        Fixed64 frameRotation,
        out Vector2d localPoint) =>
        WidePointAnchor2d.TryGetLocalPointIn(
            Origin,
            LocalPoint,
            LocalDisplacement,
            ExactLocalTerm,
            Rotation,
            frameOrigin,
            frameRotation,
            out localPoint);

    /// <summary>
    /// Attempts to express this exact conceptual point in another rotated
    /// frame without discarding sub-lattice feature information.
    /// </summary>
    /// <param name="frameOrigin">The new frame's origin in world coordinates.</param>
    /// <param name="frameRotation">The new frame's counterclockwise local-to-world rotation in radians.</param>
    /// <param name="anchor">The equivalent anchor in the new frame on success, or the default value on failure.</param>
    /// <returns>
    /// <see langword="true"/> when the same conceptual point can be retained exactly in the target frame and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>
    /// An identical frame returns this anchor with its retained residuals.
    /// For a different frame, a single Q32.32 local vector must represent the
    /// same conceptual point exactly.
    /// </remarks>
    public bool TryReframe(
        Vector2d frameOrigin,
        Fixed64 frameRotation,
        out FixedPointAnchor2d anchor)
    {
        if (Origin == frameOrigin && Rotation == frameRotation)
        {
            anchor = this;
            return true;
        }
        if (!TryGetLocalPointIn(
                frameOrigin,
                frameRotation,
                out Vector2d localPoint))
        {
            anchor = default;
            return false;
        }

        var candidate = new FixedPointAnchor2d(
            frameOrigin,
            frameRotation,
            localPoint);
        if (!WidePointAnchor2d.RepresentsSamePoint(
                this,
                candidate))
        {
            anchor = default;
            return false;
        }

        anchor = candidate;
        return true;
    }

    /// <summary>
    /// Compares the complete local feature identity of this anchor with
    /// <paramref name="other"/> in deterministic component order.
    /// </summary>
    /// <param name="other">The anchor whose local feature is compared.</param>
    /// <returns>A negative value, zero, or a positive value according to lexicographic order of the local feature terms.</returns>
    /// <remarks>
    /// Frame origin and rotation are intentionally excluded. Exact
    /// sub-lattice centered-axis residuals are included after the public local
    /// components.
    /// </remarks>
    public int CompareLocalFeature(in FixedPointAnchor2d other)
    {
        int comparison = CompareRaw(
            LocalPoint.X.m_rawValue,
            other.LocalPoint.X.m_rawValue);
        if (comparison != 0)
            return comparison;
        comparison = CompareRaw(
            LocalPoint.Y.m_rawValue,
            other.LocalPoint.Y.m_rawValue);
        if (comparison != 0)
            return comparison;
        comparison = CompareRaw(
            LocalDisplacement.X.m_rawValue,
            other.LocalDisplacement.X.m_rawValue);
        if (comparison != 0)
            return comparison;
        comparison = CompareRaw(
            LocalDisplacement.Y.m_rawValue,
            other.LocalDisplacement.Y.m_rawValue);
        if (comparison != 0)
            return comparison;
        comparison = CompareRaw(
            ExactLocalTerm.X,
            other.ExactLocalTerm.X);
        return comparison != 0
            ? comparison
            : CompareRaw(
                ExactLocalTerm.Y,
                other.ExactLocalTerm.Y);
    }

    /// <summary>
    /// Returns a stable 64-bit hash of the complete local feature identity.
    /// </summary>
    /// <returns>An unsigned deterministic 64-bit hash of the local feature terms.</returns>
    /// <remarks>Uses the same components as <see cref="CompareLocalFeature(in FixedPointAnchor2d)"/>.</remarks>
    public ulong GetLocalFeatureHash64()
    {
        ulong hash = 14695981039346656037UL;
        KeepHash(ref hash, LocalPoint.X.m_rawValue);
        KeepHash(ref hash, LocalPoint.Y.m_rawValue);
        KeepHash(ref hash, LocalDisplacement.X.m_rawValue);
        KeepHash(ref hash, LocalDisplacement.Y.m_rawValue);
        KeepHash(ref hash, ExactLocalTerm.X);
        KeepHash(ref hash, ExactLocalTerm.Y);
        return hash;
    }

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="other">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when the stored origins, rotations, local coordinates, and additive terms match and
    /// <see langword="false"/> otherwise.
    /// </returns>
    public bool Equals(FixedPointAnchor2d other) =>
        Origin == other.Origin
        && Rotation == other.Rotation
        && LocalPoint == other.LocalPoint
        && LocalDisplacement == other.LocalDisplacement
        && ExactLocalTerm.Equals(other.ExactLocalTerm);

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="obj">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when <paramref name="obj"/> is a <see cref="FixedPointAnchor2d"/> and the stored
    /// origins, rotations, local coordinates, and additive terms match and <see langword="false"/> otherwise.
    /// </returns>
    public override bool Equals(object? obj) =>
        obj is FixedPointAnchor2d other && Equals(other);

    /// <summary>Computes a hash code for the stored geometry.</summary>
    /// <returns>A hash code consistent with structural equality.</returns>
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + Origin.GetHashCode();
            hash = (hash * 31) + Rotation.GetHashCode();
            hash = (hash * 31) + LocalPoint.GetHashCode();
            hash = (hash * 31) + LocalDisplacement.GetHashCode();
            if (!ExactLocalTerm.IsZero)
                hash = (hash * 31) + ExactLocalTerm.GetHashCode();
            return hash;
        }
    }

    /// <summary>Tests whether the stored geometry is equal.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns>
    /// <see langword="true"/> when the stored origins, rotations, local coordinates, and additive terms match and
    /// <see langword="false"/> otherwise.
    /// </returns>
    public static bool operator ==(
        FixedPointAnchor2d left,
        FixedPointAnchor2d right) =>
        left.Equals(right);

    /// <summary>Tests whether the stored geometry is different.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the stored values differ and <see langword="false"/> otherwise.</returns>
    public static bool operator !=(
        FixedPointAnchor2d left,
        FixedPointAnchor2d right) =>
        !left.Equals(right);

    private static int CompareRaw(long left, long right) =>
        left < right ? -1 : left > right ? 1 : 0;

    private static void KeepHash(ref ulong hash, long value)
    {
        hash ^= unchecked((ulong)value);
        hash *= 1099511628211UL;
    }
}
