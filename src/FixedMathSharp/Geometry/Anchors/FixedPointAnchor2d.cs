//=======================================================================
// FixedPointAnchor2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
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
    public Vector2d Origin { get; }

    /// <summary>
    /// Gets the point frame's counterclockwise rotation in radians.
    /// </summary>
    public Fixed64 Rotation { get; }

    /// <summary>
    /// Gets the point in its supplied local frame.
    /// </summary>
    public Vector2d LocalPoint { get; }

    /// <summary>
    /// An additional local-space feature displacement that exact operations
    /// add to <see cref="LocalPoint"/> without a scalar intermediate.
    /// </summary>
    public Vector2d LocalDisplacement { get; }

    /// <summary>
    /// Creates a transformed point anchor.
    /// </summary>
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
    public bool TryGetPoint(out Vector2d point) =>
        WideVector2dTransform.TryTransformCompositePoint(
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
    public bool TryGetOffsetFrom(
        in FixedPointAnchor2d other,
        out Vector2d offset) =>
        WideVector2dTransform.TryGetRelativeOffset(
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
    /// Preserves this point minus <paramref name="other"/> as an exact semantic
    /// lever without narrowing it to <see cref="Vector2d"/>.
    /// </summary>
    public FixedLever2d GetLeverFrom(
        in FixedPointAnchor2d other) =>
        WideVector2dTransform.GetLever(
            this,
            other);

    /// <summary>
    /// Attempts to express this conceptual point in another rotated frame
    /// without materializing the world-space point.
    /// </summary>
    public bool TryGetLocalPointIn(
        Vector2d frameOrigin,
        Fixed64 frameRotation,
        out Vector2d localPoint) =>
        WideVector2dTransform.TryGetLocalPointIn(
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
    /// <remarks>
    /// The operation fails when the target frame would require a general
    /// rational local coordinate that a compact point anchor cannot retain.
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
        if (!WideVector2dTransform.RepresentsSamePoint(
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

    /// <summary>
    /// Returns whether both anchors have identical frames, rounded local
    /// components, and any retained exact feature identity.
    /// </summary>
    public bool Equals(FixedPointAnchor2d other) =>
        Origin == other.Origin
        && Rotation == other.Rotation
        && LocalPoint == other.LocalPoint
        && LocalDisplacement == other.LocalDisplacement
        && ExactLocalTerm.Equals(other.ExactLocalTerm);

    /// <summary>
    /// Returns a hash code for the complete frame and exact feature identity.
    /// </summary>
    public override bool Equals(object? obj) =>
        obj is FixedPointAnchor2d other && Equals(other);

    /// <inheritdoc />
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

    /// <summary>
    /// Returns whether two anchors have identical frame and local-point
    /// components.
    /// </summary>
    public static bool operator ==(
        FixedPointAnchor2d left,
        FixedPointAnchor2d right) =>
        left.Equals(right);

    /// <summary>
    /// Returns whether two anchors differ in any frame or local-point
    /// component.
    /// </summary>
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
