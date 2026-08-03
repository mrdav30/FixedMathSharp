//=======================================================================
// FixedPointAnchor.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Represents a 3D point as a local coordinate in one rigid world-space frame.
/// </summary>
/// <remarks>
/// Keeping the frame components separate allows full-domain geometry to retain
/// points whose rotated offset or absolute world coordinate is not representable
/// as an intermediate <see cref="Fixed64"/> value.
/// Relation-produced anchors can also retain an exact sub-lattice feature term
/// that is intentionally not exposed as a public wide-arithmetic primitive.
/// Consequently, anchors with identical rounded public local components can
/// still compare unequal when they identify distinct exact features.
/// </remarks>
public readonly struct FixedPointAnchor : IEquatable<FixedPointAnchor>
{
    /// <summary>
    /// Initializes a point anchor from a world origin, normalized local-to-world
    /// rotation, and local point.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// <paramref name="rotation"/> is not normalized.
    /// </exception>
    public FixedPointAnchor(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint)
        : this(origin, rotation, localPoint, Vector3d.Zero)
    { }

    /// <summary>
    /// Initializes a point anchor from a world origin, normalized local-to-world
    /// rotation, and two additive local-space feature components.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// <paramref name="rotation"/> is not normalized.
    /// </exception>
    public FixedPointAnchor(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d localDisplacement)
        : this(
            origin,
            rotation,
            localPoint,
            localDisplacement,
            Vector3d.Zero,
            default,
            validateRotation: true)
    { }

    internal FixedPointAnchor(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d localDisplacement,
        FixedPointAnchorTerm3d exactLocalTerm)
        : this(
            origin,
            rotation,
            localPoint,
            localDisplacement,
            Vector3d.Zero,
            exactLocalTerm,
            validateRotation: true)
    { }

    internal static FixedPointAnchor FromValidatedFrame(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint) =>
        new(
            origin,
            rotation,
            localPoint,
            Vector3d.Zero,
            Vector3d.Zero,
            default,
            validateRotation: false);

    private FixedPointAnchor(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d localDisplacement,
        Vector3d localTranslation,
        FixedPointAnchorTerm3d exactLocalTerm,
        bool validateRotation)
    {
        if (validateRotation && !rotation.IsNormalized())
            throw new ArgumentException("The point-anchor rotation must be normalized.", nameof(rotation));

        Origin = origin;
        Rotation = rotation;
        LocalPoint = localPoint;
        LocalDisplacement = localDisplacement;
        LocalTranslation = localTranslation;
        ExactLocalTerm = exactLocalTerm;
    }

    /// <summary>
    /// The world-space origin of the rigid frame.
    /// </summary>
    public Vector3d Origin { get; }

    /// <summary>
    /// The normalized local-to-world rotation of the rigid frame.
    /// </summary>
    public FixedQuaternion Rotation { get; }

    /// <summary>
    /// The point in the rigid frame's local coordinates.
    /// </summary>
    public Vector3d LocalPoint { get; }

    /// <summary>
    /// An additional local-space feature displacement that exact operations
    /// add to <see cref="LocalPoint"/> without a scalar intermediate.
    /// </summary>
    public Vector3d LocalDisplacement { get; }

    /// <summary>
    /// An independent local-space translation applied to the complete anchored
    /// point without merging it into either feature component.
    /// </summary>
    public Vector3d LocalTranslation { get; }

    internal FixedPointAnchorTerm3d ExactLocalTerm { get; }

    /// <summary>
    /// Returns an anchor for the same exact local feature with the specified
    /// independent local-space translation.
    /// </summary>
    public FixedPointAnchor WithLocalTranslation(
        Vector3d localTranslation) =>
        new(
            Origin,
            Rotation,
            LocalPoint,
            LocalDisplacement,
            localTranslation,
            ExactLocalTerm,
            validateRotation: true);

    /// <summary>
    /// Attempts to materialize the absolute world point.
    /// </summary>
    public bool TryGetPoint(out Vector3d point)
    {
        if (!Rotation.IsNormalized())
        {
            point = default;
            return false;
        }

        return WidePointAnchor3d.TryGetPoint(
            Origin,
            Rotation,
            LocalPoint,
            LocalDisplacement,
            LocalTranslation,
            ExactLocalTerm,
            out point);
    }

    /// <summary>
    /// Attempts to materialize this point's world-space offset from
    /// <paramref name="other"/>.
    /// </summary>
    public bool TryGetOffsetFrom(
        in FixedPointAnchor other,
        out Vector3d offset) =>
        TryGetScaledOffsetFrom(other, Fixed64.One, out offset);

    /// <summary>
    /// Attempts to materialize this point's world-space offset from
    /// <paramref name="other"/> after applying <paramref name="scale"/>.
    /// </summary>
    /// <remarks>
    /// The scale is applied to the complete conceptual difference before its
    /// final round-half-to-even conversion.
    /// </remarks>
    public bool TryGetScaledOffsetFrom(
        in FixedPointAnchor other,
        Fixed64 scale,
        out Vector3d offset)
    {
        if (!Rotation.IsNormalized() || !other.Rotation.IsNormalized())
        {
            offset = default;
            return false;
        }
        if (scale == Fixed64.Zero)
        {
            offset = Vector3d.Zero;
            return true;
        }

        return WidePointAnchor3d.TryGetRelativeOffset(
            Origin,
            Rotation,
            LocalPoint,
            LocalDisplacement,
            LocalTranslation,
            ExactLocalTerm,
            other.Origin,
            other.Rotation,
            other.LocalPoint,
            other.LocalDisplacement,
            other.LocalTranslation,
            other.ExactLocalTerm,
            scale,
            out offset);
    }

    /// <summary>
    /// Attempts to project this point minus <paramref name="other"/> onto
    /// <paramref name="direction"/> with one final round-half-to-even
    /// conversion.
    /// </summary>
    public bool TryGetProjectedOffsetFrom(
        in FixedPointAnchor other,
        Vector3d direction,
        out Fixed64 projection)
    {
        if (!Rotation.IsNormalized() || !other.Rotation.IsNormalized())
        {
            projection = default;
            return false;
        }

        return WidePointAnchor3d.TryGetProjectedOffset(
            Origin,
            Rotation,
            LocalPoint,
            LocalDisplacement,
            LocalTranslation,
            ExactLocalTerm,
            other.Origin,
            other.Rotation,
            other.LocalPoint,
            other.LocalDisplacement,
            other.LocalTranslation,
            other.ExactLocalTerm,
            direction,
            out projection);
    }

    /// <summary>
    /// Projects this point minus <paramref name="other"/> onto
    /// <paramref name="direction"/> and returns zero for nonpositive results,
    /// the positive projection floored to Q32.32, or
    /// <see cref="Fixed64.MaxValue"/> when only the final result is
    /// unrepresentable.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// This anchor does not contain a normalized rotation.
    /// </exception>
    /// <exception cref="ArgumentException">
    /// <paramref name="other"/> does not contain a normalized rotation.
    /// </exception>
    public Fixed64 ProjectNonNegativeOffsetFrom(
        in FixedPointAnchor other,
        Vector3d direction)
    {
        if (!Rotation.IsNormalized())
        {
            throw new InvalidOperationException(
                "The point anchor must have a normalized rotation.");
        }
        if (!other.Rotation.IsNormalized())
        {
            throw new ArgumentException(
                "The other point anchor must have a normalized rotation.",
                nameof(other));
        }

        return WidePointAnchor3d.ProjectNonNegativeOffset(
            Origin,
            Rotation,
            LocalPoint,
            LocalDisplacement,
            LocalTranslation,
            ExactLocalTerm,
            other.Origin,
            other.Rotation,
            other.LocalPoint,
            other.LocalDisplacement,
            other.LocalTranslation,
            other.ExactLocalTerm,
            direction);
    }

    /// <summary>
    /// Compares the exact squared distance from this point to two other
    /// anchored points.
    /// </summary>
    /// <returns>
    /// A negative value when <paramref name="first"/> is closer, zero when the
    /// distances are equal, or a positive value when
    /// <paramref name="second"/> is closer.
    /// </returns>
    public int CompareSquaredDistance(
        in FixedPointAnchor first,
        in FixedPointAnchor second)
    {
        if (!Rotation.IsNormalized())
        {
            throw new InvalidOperationException(
                "The reference point anchor must have a normalized rotation.");
        }
        if (!first.Rotation.IsNormalized())
        {
            throw new ArgumentException(
                "The first point anchor must have a normalized rotation.",
                nameof(first));
        }
        if (!second.Rotation.IsNormalized())
        {
            throw new ArgumentException(
                "The second point anchor must have a normalized rotation.",
                nameof(second));
        }

        return WidePointAnchor3d.CompareSquaredDistances(
            this,
            first,
            second);
    }

    /// <summary>
    /// Attempts to express this point in another rigid frame's local
    /// coordinates without first materializing its absolute world position.
    /// </summary>
    public bool TryGetLocalPointIn(
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out Vector3d localPoint)
    {
        if (!frameRotation.IsNormalized())
        {
            localPoint = default;
            return false;
        }
        if (!Rotation.IsNormalized())
        {
            localPoint = default;
            return false;
        }

        return WidePointAnchor3d.TryGetLocalPointIn(
            Origin,
            Rotation,
            LocalPoint,
            LocalDisplacement,
            LocalTranslation,
            ExactLocalTerm,
            frameOrigin,
            frameRotation,
            out localPoint);
    }

    /// <summary>
    /// Attempts to express this exact conceptual point in another rigid frame
    /// without discarding any sub-lattice feature information.
    /// </summary>
    /// <remarks>
    /// The operation fails when the target frame would require a general
    /// rational local coordinate that a compact point anchor cannot retain.
    /// </remarks>
    public bool TryReframe(
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out FixedPointAnchor anchor)
    {
        if (!Rotation.IsNormalized()
            || !frameRotation.IsNormalized())
        {
            anchor = default;
            return false;
        }
        if (Origin == frameOrigin && Rotation == frameRotation)
        {
            anchor = this;
            return true;
        }
        if (!TryGetLocalPointIn(
                frameOrigin,
                frameRotation,
                out Vector3d localPoint))
        {
            anchor = default;
            return false;
        }

        var candidate = new FixedPointAnchor(
            frameOrigin,
            frameRotation,
            localPoint);
        if (!WidePointAnchor3d.RepresentsSamePoint(this, candidate))
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
    /// Frame origin, rotation, and <see cref="LocalTranslation"/> are
    /// intentionally excluded because they move a feature without changing
    /// its local identity. Exact sub-lattice centered-axis residuals are
    /// included after the public feature components, so distinct features
    /// cannot collapse when their rounded local coordinates are equal.
    /// </remarks>
    public int CompareLocalFeature(in FixedPointAnchor other)
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
            LocalPoint.Z.m_rawValue,
            other.LocalPoint.Z.m_rawValue);
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
            LocalDisplacement.Z.m_rawValue,
            other.LocalDisplacement.Z.m_rawValue);
        if (comparison != 0)
            return comparison;
        comparison = CompareRaw(
            ExactLocalTerm.X,
            other.ExactLocalTerm.X);
        if (comparison != 0)
            return comparison;
        comparison = CompareRaw(
            ExactLocalTerm.Y,
            other.ExactLocalTerm.Y);
        return comparison != 0
            ? comparison
            : CompareRaw(
                ExactLocalTerm.Z,
                other.ExactLocalTerm.Z);
    }

    /// <summary>
    /// Returns a stable 64-bit hash of the complete local feature identity.
    /// </summary>
    public ulong GetLocalFeatureHash64()
    {
        ulong hash = 14695981039346656037UL;
        KeepHash(ref hash, LocalPoint.X.m_rawValue);
        KeepHash(ref hash, LocalPoint.Y.m_rawValue);
        KeepHash(ref hash, LocalPoint.Z.m_rawValue);
        KeepHash(ref hash, LocalDisplacement.X.m_rawValue);
        KeepHash(ref hash, LocalDisplacement.Y.m_rawValue);
        KeepHash(ref hash, LocalDisplacement.Z.m_rawValue);
        KeepHash(ref hash, ExactLocalTerm.X);
        KeepHash(ref hash, ExactLocalTerm.Y);
        KeepHash(ref hash, ExactLocalTerm.Z);
        return hash;
    }

    /// <summary>
    /// Returns whether both anchors have identical frames, rounded local
    /// components, and any retained exact feature identity.
    /// </summary>
    public bool Equals(FixedPointAnchor other) =>
        Origin.Equals(other.Origin)
        && Rotation.Equals(other.Rotation)
        && LocalPoint.Equals(other.LocalPoint)
        && LocalDisplacement.Equals(other.LocalDisplacement)
        && LocalTranslation.Equals(other.LocalTranslation)
        && ExactLocalTerm.Equals(other.ExactLocalTerm);

    /// <summary>
    /// Returns a hash code for the complete frame and exact feature identity.
    /// </summary>
    public override bool Equals(object? obj) =>
        obj is FixedPointAnchor other && Equals(other);

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
            hash = (hash * 31) + LocalTranslation.GetHashCode();
            if (!ExactLocalTerm.IsZero)
                hash = (hash * 31) + ExactLocalTerm.GetHashCode();
            return hash;
        }
    }

    /// <summary>
    /// Returns whether two point anchors have identical frame components.
    /// </summary>
    public static bool operator ==(
        FixedPointAnchor left,
        FixedPointAnchor right) =>
        left.Equals(right);

    /// <summary>
    /// Returns whether two point anchors have different frame components.
    /// </summary>
    public static bool operator !=(
        FixedPointAnchor left,
        FixedPointAnchor right) =>
        !left.Equals(right);

    private static int CompareRaw(long left, long right) =>
        left < right ? -1 : left > right ? 1 : 0;

    private static void KeepHash(ref ulong hash, long value)
    {
        hash ^= unchecked((ulong)value);
        hash *= 1099511628211UL;
    }
}
