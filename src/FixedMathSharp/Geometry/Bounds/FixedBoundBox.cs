//=======================================================================
// FixedBoundBox.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Represents a normalized three-dimensional axis-aligned bounding box.
/// </summary>
/// <remarks>
/// Use <see cref="FromMinMax"/>, <see cref="FromCenterAndSize"/>, or
/// <see cref="FromCenterAndScope"/> so construction intent is explicit at the call site.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct FixedBoundBox : IEquatable<FixedBoundBox>
{
    /// <summary>
    /// The number of stable corners exposed by <see cref="GetCorner"/> and <see cref="CopyCorners"/>.
    /// </summary>
    public const int CornerCount = 8;

    #region Nested Types

    /// <summary>
    /// Represents the state of a three-dimensional axis-aligned bounding box using its minimum and maximum coordinates.
    /// </summary>
    /// <remarks>
    /// The bounding box is defined by two points: the minimum and maximum corners in 3D space. This
    /// structure is immutable and can be used to describe spatial boundaries for geometric computations, collision
    /// detection, or spatial queries.
    /// </remarks>
    [Serializable]
    [MemoryPackable]
    public readonly partial struct BoundingBoxState
    {
        /// <inheritdoc cref="FixedBoundBox.Min"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector3d Min;

        /// <inheritdoc cref="FixedBoundBox.Max"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector3d Max;

        /// <summary>
        /// Initializes a new instance of the BoundingBoxState class with the specified minimum and maximum coordinates.
        /// </summary>
        [JsonConstructor]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingBoxState(Vector3d min, Vector3d max)
        {
            Min = Vector3d.Min(min, max);
            Max = Vector3d.Max(min, max);
        }
    }

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance of the FixedBoundBox class with the specified bounding box state.
    /// </summary>
    /// <param name="state">The state that defines the position, size, and orientation of the bounding box.</param>
    [JsonConstructor]
    public FixedBoundBox(BoundingBoxState state)
    {
        State = state;
    }

    #endregion

    #region Properties

    /// <summary>
    /// The minimum corner of the bounding box.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Min { get; private set; }

    /// <summary>
    /// The maximum corner of the bounding box.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Max { get; private set; }

    /// <summary>
    /// The center of the bounding box, rounded to the nearest-even Q32.32 lattice point.
    /// </summary>
    /// <remarks>
    /// Assigning a different center preserves a conservative half-extent. An
    /// odd raw-unit span can therefore expand by one raw unit so the assigned
    /// center remains exact and the previous box is not under-represented.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// An assigned center would place an endpoint outside the scalar domain.
    /// </exception>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Center
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Vector3d.Midpoint(Min, Max);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            if (value != Center)
                SetCenterAndHalfSize(value, Scope);
        }
    }

    /// <summary>
    /// The exact total size of the box.
    /// </summary>
    /// <remarks>
    /// Assigned values are normalized by absolute component value and divided
    /// outward. An odd raw-unit size therefore expands by one raw unit. Reading
    /// this property throws rather than returning a saturated value when an
    /// exact component span is not representable by <see cref="Fixed64"/>.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// A component span is not representable, or an assigned size would place
    /// an endpoint outside the scalar domain.
    /// </exception>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Proportions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(
            WideGeometry.GetIntervalSize(Min.X, Max.X),
            WideGeometry.GetIntervalSize(Min.Y, Max.Y),
            WideGeometry.GetIntervalSize(Min.Z, Max.Z));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            SetCenterAndHalfSize(Center, GetHalfSize(value));
        }
    }

    /// <summary>
    /// The smallest representable half-extent that conservatively contains the
    /// box around <see cref="Center"/>.
    /// </summary>
    /// <exception cref="OverflowException">
    /// A conservative half-extent is outside the representable scalar domain.
    /// </exception>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Scope
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(
            WideGeometry.GetIntervalScope(Min.X, Max.X),
            WideGeometry.GetIntervalScope(Min.Y, Max.Y),
            WideGeometry.GetIntervalScope(Min.Z, Max.Z));
    }

    /// <summary>
    /// Gets or sets the current normalized bounding box state, including its minimum and maximum coordinates.
    /// </summary>
    [JsonInclude]
    [MemoryPackInclude]
    public BoundingBoxState State
    {
        get => new(Min, Max);

        internal set
        {
            SetMinMax(value.Min, value.Max);
        }
    }

    #endregion

    #region Factories

    /// <summary>
    /// Creates a normalized bounding box from minimum and maximum corners.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundBox FromMinMax(Vector3d min, Vector3d max)
    {
        var box = default(FixedBoundBox);
        box.SetMinMax(min, max);
        return box;
    }

    /// <summary>
    /// Creates a bounding box from a center point and total size.
    /// </summary>
    /// <remarks>
    /// Negative size components are normalized by absolute value. Odd raw-unit
    /// sizes are divided outward and therefore expand by one raw unit.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// The centered box would place an endpoint outside the scalar domain.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundBox FromCenterAndSize(Vector3d center, Vector3d size)
    {
        var box = default(FixedBoundBox);
        box.SetCenterAndHalfSize(center, GetHalfSize(size));
        return box;
    }

    /// <summary>
    /// Creates a bounding box from a center point and half-size scope.
    /// </summary>
    /// <remarks>
    /// Negative scope components are normalized by absolute value.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// A scope magnitude is not representable, or the centered box would place
    /// an endpoint outside the scalar domain.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundBox FromCenterAndScope(Vector3d center, Vector3d scope)
    {
        var box = default(FixedBoundBox);
        box.SetCenterAndHalfSize(center, GetScopeMagnitude(scope));
        return box;
    }

    /// <summary>
    /// Creates the representable-domain intersection of a box described by a
    /// center point and total size.
    /// </summary>
    /// <remarks>
    /// Negative size components are normalized by absolute value and odd raw-
    /// unit sizes divide outward. Endpoints outside the scalar domain are
    /// explicitly clipped to <see cref="Fixed64.MinValue"/> or
    /// <see cref="Fixed64.MaxValue"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundBox FromCenterAndSizeClippedToDomain(Vector3d center, Vector3d size)
    {
        var box = default(FixedBoundBox);
        box.SetCenterAndHalfSizeClippedToDomain(center, GetHalfSize(size));
        return box;
    }

    /// <summary>
    /// Creates the representable-domain intersection of a box described by a
    /// center point and half-size scope.
    /// </summary>
    /// <remarks>
    /// Negative scope components are normalized by absolute value. Endpoints
    /// outside the scalar domain are explicitly clipped to
    /// <see cref="Fixed64.MinValue"/> or <see cref="Fixed64.MaxValue"/>.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// A scope magnitude is not representable.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundBox FromCenterAndScopeClippedToDomain(Vector3d center, Vector3d scope)
    {
        var box = default(FixedBoundBox);
        box.SetCenterAndHalfSizeClippedToDomain(center, GetScopeMagnitude(scope));
        return box;
    }

    /// <summary>
    /// Creates conservative target-frame bounds for a rotated source-frame
    /// box, clipping only final target-frame endpoints to the scalar domain.
    /// </summary>
    /// <remarks>
    /// Conceptually transforms every source local point as
    /// <c>targetRotation^-1 * (sourceOrigin + sourceRotation * point - targetOrigin)</c>.
    /// Source endpoints are normalized component-wise before the exact
    /// projection. Intermediate world points are never materialized.
    /// </remarks>
    public static FixedBoundBox FromRelativeRotatedBoundsClippedToDomain(
        Vector3d sourceOrigin,
        FixedQuaternion sourceRotation,
        Vector3d sourceLocalMin,
        Vector3d sourceLocalMax,
        Vector3d targetOrigin,
        FixedQuaternion targetRotation)
    {
        if (!sourceRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Source rotation must be normalized.",
                nameof(sourceRotation));
        }
        if (!targetRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Target rotation must be normalized.",
                nameof(targetRotation));
        }

        return WideOrientedBox.GetRelativeRotatedBoundsClippedToDomain(
            sourceOrigin,
            sourceRotation,
            sourceLocalMin,
            sourceLocalMax,
            targetOrigin,
            targetRotation);
    }

    #endregion

    #region Mutators

    /// <summary>
    /// Orients the bounding box with the given center and size.
    /// </summary>
    /// <exception cref="OverflowException">
    /// The requested bounds would place an endpoint outside the scalar domain.
    /// </exception>
    public void Orient(Vector3d center, Vector3d? size)
    {
        if (size.HasValue)
        {
            SetCenterAndHalfSize(center, GetHalfSize(size.Value));
            return;
        }

        Center = center;
    }

    /// <summary>
    /// Resizes the bounding box to the specified size, keeping the same center.
    /// </summary>
    /// <exception cref="OverflowException">
    /// The requested size would place an endpoint outside the scalar domain.
    /// </exception>
    public void Resize(Vector3d size)
    {
        SetCenterAndHalfSize(Center, GetHalfSize(size));
    }

    /// <summary>
    /// Sets the normalized bounds of the bounding box by specifying its minimum and maximum points.
    /// </summary>
    public void SetMinMax(Vector3d min, Vector3d max)
    {
        Min = Vector3d.Min(min, max);
        Max = Vector3d.Max(min, max);
    }

    /// <summary>
    /// Configures the bounding box with the specified center and scope (half-size).
    /// </summary>
    /// <remarks>
    /// Negative scope components are normalized by absolute value.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// A scope magnitude is not representable, or the requested bounds would
    /// place an endpoint outside the scalar domain.
    /// </exception>
    public void SetBoundingBox(Vector3d center, Vector3d scope)
    {
        SetCenterAndHalfSize(center, GetScopeMagnitude(scope));
    }

    /// <summary>
    /// Determines if a point is inside the bounding box (including boundaries).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector3d point)
    {
        return point.X >= Min.X && point.X <= Max.X
            && point.Y >= Min.Y && point.Y <= Max.Y
            && point.Z >= Min.Z && point.Z <= Max.Z;
    }

    /// <summary>
    /// Tests another bounding box against this bounding box.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundBox box) => ContainsBoxLike(box.Min, box.Max);

    /// <summary>
    /// Tests a bounding sphere against this bounding box.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundSphere sphere)
    {
        if (WideGeometry.ContainsCenteredExtent(Min.X, Max.X, sphere.Center.X, sphere.Radius)
            && WideGeometry.ContainsCenteredExtent(Min.Y, Max.Y, sphere.Center.Y, sphere.Radius)
            && WideGeometry.ContainsCenteredExtent(Min.Z, Max.Z, sphere.Center.Z, sphere.Radius))
            return FixedEnclosureType.Contains;

        return Intersects(sphere) ? FixedEnclosureType.Intersects : FixedEnclosureType.Disjoint;
    }

    /// <summary>
    /// Tests a bounding frustum against this bounding box.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundFrustum frustum)
    {
        if (Contains(frustum.Min) && Contains(frustum.Max))
            return FixedEnclosureType.Contains;

        return Intersects(frustum) ? FixedEnclosureType.Intersects : FixedEnclosureType.Disjoint;
    }

    /// <summary>
    /// Checks whether another bounding box intersects this bounding box, including boundary-only contact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundBox box) => IntersectsBoxLike(box.Min, box.Max);

    /// <summary>
    /// Checks whether a bounding sphere intersects this bounding box, including boundary-only contact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundSphere sphere) => IntersectsSphere(sphere);

    /// <summary>
    /// Checks whether another bounding box overlaps this bounding box with positive volume on every axis.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundBox box) => HasStrictAxisOverlap(box.Min, box.Max);

    /// <summary>
    /// Checks whether a bounding sphere overlaps this bounding box with positive volume.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundSphere sphere) => IntersectsSphereStrict(sphere);

    /// <summary>
    /// Checks whether a bounding frustum intersects this bounding box.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundFrustum frustum)
    {
        return frustum.Intersects(this);
    }

    /// <summary>
    /// Projects a point into the bounding box by clamping it to the box extents.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ProjectPoint(Vector3d point)
        => ClampPoint(point);

    /// <summary>
    /// Clamps a point to this bounding box, returning the point unchanged when it is already inside.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ClampPoint(Vector3d point)
    {
        return new Vector3d(
            FixedMath.Clamp(point.X, Min.X, Max.X),
            FixedMath.Clamp(point.Y, Min.Y, Max.Y),
            FixedMath.Clamp(point.Z, Min.Z, Max.Z));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private FixedEnclosureType ContainsBoxLike(Vector3d otherMin, Vector3d otherMax)
    {
        if (Contains(otherMin) && Contains(otherMax))
            return FixedEnclosureType.Contains;

        return IntersectsBoxLike(otherMin, otherMax)
            ? FixedEnclosureType.Intersects
            : FixedEnclosureType.Disjoint;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IntersectsBoxLike(Vector3d otherMin, Vector3d otherMax)
    {
        return Min.X <= otherMax.X && Max.X >= otherMin.X
            && Min.Y <= otherMax.Y && Max.Y >= otherMin.Y
            && Min.Z <= otherMax.Z && Max.Z >= otherMin.Z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IntersectsSphere(FixedBoundSphere sphere)
    {
        return WideGeometry.CompareDistanceToRadiusSum(
            sphere.Center,
            ClampPoint(sphere.Center),
            sphere.Radius,
            Fixed64.Zero) <= 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IntersectsSphereStrict(FixedBoundSphere sphere)
    {
        return HasPositiveVolume()
            && sphere.Radius > Fixed64.Zero
            && WideGeometry.CompareDistanceToRadiusSum(
                sphere.Center,
                ClampPoint(sphere.Center),
                sphere.Radius,
                Fixed64.Zero) < 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool HasPositiveVolume()
    {
        return Min.X < Max.X && Min.Y < Max.Y && Min.Z < Max.Z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool HasStrictAxisOverlap(Vector3d otherMin, Vector3d otherMax)
    {
        return HasPositiveVolume()
            && otherMin.X < otherMax.X
            && otherMin.Y < otherMax.Y
            && otherMin.Z < otherMax.Z
            && Min.X < otherMax.X && Max.X > otherMin.X
            && Min.Y < otherMax.Y && Max.Y > otherMin.Y
            && Min.Z < otherMax.Z && Max.Z > otherMin.Z;
    }

    /// <summary>
    /// Calculates the shortest distance from a given point to the surface of the bounding box.
    /// If the point lies inside the box, the distance is zero.
    /// </summary>
    /// <param name="point">The point from which to calculate the distance.</param>
    /// <returns>
    /// The shortest distance from the point to the surface of the bounding box.
    /// If the point is inside the box, the method returns zero.
    /// </returns>
    /// <remarks>
    /// The method finds the closest point on the box's surface by clamping the given point 
    /// to the box's bounds and returns the Euclidean distance between them. 
    /// This ensures accurate distance calculations, even near corners or edges.
    /// </remarks>
    public Fixed64 DistanceToSurface(Vector3d point)
    {
        // Clamp the point to the nearest point on the box's surface
        Vector3d clampedPoint = new(
            FixedMath.Clamp(point.X, Min.X, Max.X),
            FixedMath.Clamp(point.Y, Min.Y, Max.Y),
            FixedMath.Clamp(point.Z, Min.Z, Max.Z)
        );

        // If the point is inside the box, return 0
        if (Contains(point))
            return Fixed64.Zero;

        // Otherwise, return the Euclidean distance to the clamped point
        return Vector3d.Distance(point, clampedPoint);
    }

    /// <summary>
    /// Finds the closest point on the surface of the bounding box towards a specified object position.
    /// </summary>
    public Vector3d GetPointOnSurfaceTowardsObject(Vector3d objectPosition)
        => ClosestPointOnSurface(ProjectPoint(objectPosition));

    /// <summary>
    /// Finds the closest point on the surface of the bounding box to the specified point.
    /// </summary>
    public Vector3d ClosestPointOnSurface(Vector3d point)
    {
        if (Contains(point))
        {
            // Calculate distances to each face and return the closest face.
            Fixed64 distToMinX = point.X - Min.X;
            Fixed64 distToMaxX = Max.X - point.X;
            Fixed64 distToMinY = point.Y - Min.Y;
            Fixed64 distToMaxY = Max.Y - point.Y;
            Fixed64 distToMinZ = point.Z - Min.Z;
            Fixed64 distToMaxZ = Max.Z - point.Z;

            Fixed64 minDistToFace = FixedMath.Min(distToMinX,
                FixedMath.Min(distToMaxX,
                FixedMath.Min(distToMinY,
                FixedMath.Min(distToMaxY,
                FixedMath.Min(distToMinZ, distToMaxZ)))));

            // Adjust the closest point based on the face.
            if (minDistToFace == distToMinX) point.X = Min.X;
            else if (minDistToFace == distToMaxX) point.X = Max.X;

            if (minDistToFace == distToMinY) point.Y = Min.Y;
            else if (minDistToFace == distToMaxY) point.Y = Max.Y;

            if (minDistToFace == distToMinZ) point.Z = Min.Z;
            else if (minDistToFace == distToMaxZ) point.Z = Max.Z;

            return point;
        }

        // If the point is outside the box, clamp to the nearest surface.
        return new Vector3d(
            FixedMath.Clamp(point.X, Min.X, Max.X),
            FixedMath.Clamp(point.Y, Min.Y, Max.Y),
            FixedMath.Clamp(point.Z, Min.Z, Max.Z)
        );
    }

    /// <summary>
    /// Gets a stable corner without allocating.
    /// </summary>
    /// <remarks>
    /// Corner order is: min/min/min, max/min/min, min/max/min, max/max/min,
    /// min/min/max, max/min/max, min/max/max, max/max/max.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetCorner(int index)
    {
        return index switch
        {
            0 => new Vector3d(Min.X, Min.Y, Min.Z),
            1 => new Vector3d(Max.X, Min.Y, Min.Z),
            2 => new Vector3d(Min.X, Max.Y, Min.Z),
            3 => new Vector3d(Max.X, Max.Y, Min.Z),
            4 => new Vector3d(Min.X, Min.Y, Max.Z),
            5 => new Vector3d(Max.X, Min.Y, Max.Z),
            6 => new Vector3d(Min.X, Max.Y, Max.Z),
            7 => new Vector3d(Max.X, Max.Y, Max.Z),
            _ => throw new ArgumentOutOfRangeException(nameof(index), $"Corner index must be between 0 and {CornerCount - 1}."),
        };
    }

    /// <summary>
    /// Copies this box's corners into the destination span in <see cref="GetCorner"/> order.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void CopyCorners(Span<Vector3d> destination)
    {
        if (destination.Length < CornerCount)
            throw new ArgumentException($"The destination span must contain at least {CornerCount} elements.", nameof(destination));

        destination[0] = new Vector3d(Min.X, Min.Y, Min.Z);
        destination[1] = new Vector3d(Max.X, Min.Y, Min.Z);
        destination[2] = new Vector3d(Min.X, Max.Y, Min.Z);
        destination[3] = new Vector3d(Max.X, Max.Y, Min.Z);
        destination[4] = new Vector3d(Min.X, Min.Y, Max.Z);
        destination[5] = new Vector3d(Max.X, Min.Y, Max.Z);
        destination[6] = new Vector3d(Min.X, Max.Y, Max.Z);
        destination[7] = new Vector3d(Max.X, Max.Y, Max.Z);
    }

    /// <summary>
    /// Gets the exact additional volume required for this box's union with
    /// <paramref name="other"/>, floored to integer world units and clamped to
    /// <see cref="long.MaxValue"/>.
    /// </summary>
    /// <remarks>
    /// The three endpoint spans and both volumes remain in exact unsigned
    /// 192-bit arithmetic. This metric is suitable for spatial-index insertion
    /// heuristics even when either box has an unrepresentable
    /// <see cref="Proportions"/> component.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public long GetVolumeExpansionCost(FixedBoundBox other) =>
        WideGeometry.GetVolumeExpansionCost(Min, Max, other.Min, other.Max);

    #endregion

    #region Static Ops

    /// <summary>
    /// Creates a new bounding box that is the union of two bounding boxes.
    /// </summary>
    public static FixedBoundBox Union(FixedBoundBox a, FixedBoundBox b)
    {
        return FromMinMax(Vector3d.Min(a.Min, b.Min), Vector3d.Max(a.Max, b.Max));
    }

    /// <summary>
    /// Finds the closest points between two bounding boxes.
    /// </summary>
    public static Vector3d FindClosestPointsBetweenBoxes(FixedBoundBox a, FixedBoundBox b)
    {
        Vector3d closestPoint = Vector3d.Zero;
        Fixed64 minDistance = Fixed64.MaxValue;

        for (int i = 0; i < CornerCount; i++)
        {
            Vector3d corner = b.GetCorner(i);
            Vector3d point = a.ClosestPointOnSurface(corner);
            Fixed64 distance = Vector3d.Distance(point, corner);
            if (distance < minDistance)
            {
                closestPoint = point;
                minDistance = distance;
            }
        }

        return closestPoint;
    }

    #endregion

    #region Equality

    /// <summary>
    /// Determines whether two FixedBoundBox instances are equal.
    /// </summary>
    public static bool operator ==(FixedBoundBox left, FixedBoundBox right) => left.Equals(right);

    /// <summary>
    /// Determines whether two FixedBoundBox instances are not equal.
    /// </summary>
    public static bool operator !=(FixedBoundBox left, FixedBoundBox right) => !left.Equals(right);

    #endregion

    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is FixedBoundBox other && Equals(other);

    /// <inheritdoc/>
    public bool Equals(FixedBoundBox other)
        => Min.Equals(other.Min) && Max.Equals(other.Max);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + Min.StateHash;
            hash = (hash * 31) + Max.StateHash;
            return hash;
        }
    }

    #endregion

    #region Helpers

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetCenterAndHalfSize(Vector3d center, Vector3d halfSize)
    {
        if (!Vector3d.TrySubtract(center, halfSize, out Vector3d min)
            || !Vector3d.TryAdd(center, halfSize, out Vector3d max))
        {
            throw new OverflowException("The centered box places at least one endpoint outside the representable Fixed64 range.");
        }

        Min = min;
        Max = max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetCenterAndHalfSizeClippedToDomain(Vector3d center, Vector3d halfSize)
    {
        Vector3d min = center - halfSize;
        Vector3d max = center + halfSize;
        Min = min;
        Max = max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d GetHalfSize(Vector3d size) => new(
        WideGeometry.GetHalfSizeMagnitude(size.X),
        WideGeometry.GetHalfSizeMagnitude(size.Y),
        WideGeometry.GetHalfSizeMagnitude(size.Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d GetScopeMagnitude(Vector3d scope) => new(
        WideGeometry.GetExtentMagnitude(scope.X),
        WideGeometry.GetExtentMagnitude(scope.Y),
        WideGeometry.GetExtentMagnitude(scope.Z));

    #endregion
}
