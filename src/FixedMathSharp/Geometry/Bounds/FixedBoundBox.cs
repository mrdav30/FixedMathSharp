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

namespace FixedMathSharp.Bounds;

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
    /// The center of the bounding box.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Center
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (Min + Max) * Fixed64.Half;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            Vector3d half = (Max - Min) * Fixed64.Half;
            Min = value - half;
            Max = value + half;
        }
    }

    /// <summary>
    /// The total size of the box. This is always twice the scope.
    /// </summary>
    /// <remarks>
    /// Assigned values are normalized by absolute component value.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Proportions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Max - Min;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            Vector3d half = Vector3d.Abs(value) * Fixed64.Half;
            Vector3d center = (Min + Max) * Fixed64.Half;
            Min = center - half;
            Max = center + half;
        }
    }

    /// <summary>
    /// The range (half-size) of the bounding box in all directions. Always half of the total size.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Scope
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (Max - Min) * Fixed64.Half;
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
    /// Negative size components are normalized by absolute value.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundBox FromCenterAndSize(Vector3d center, Vector3d size)
    {
        return FromCenterAndScope(center, size * Fixed64.Half);
    }

    /// <summary>
    /// Creates a bounding box from a center point and half-size scope.
    /// </summary>
    /// <remarks>
    /// Negative scope components are normalized by absolute value.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundBox FromCenterAndScope(Vector3d center, Vector3d scope)
    {
        Vector3d normalizedScope = Vector3d.Abs(scope);
        return new FixedBoundBox
        {
            Min = center - normalizedScope,
            Max = center + normalizedScope
        };
    }

    #endregion

    #region Mutators

    /// <summary>
    /// Orients the bounding box with the given center and size.
    /// </summary>
    public void Orient(Vector3d center, Vector3d? size)
    {
        Vector3d half = size.HasValue
            ? Vector3d.Abs(size.Value) * Fixed64.Half
            : (Max - Min) * Fixed64.Half;

        Min = center - half;
        Max = center + half;
    }

    /// <summary>
    /// Resizes the bounding box to the specified size, keeping the same center.
    /// </summary>
    public void Resize(Vector3d size)
    {
        Vector3d half = Vector3d.Abs(size) * Fixed64.Half;
        Vector3d center = (Min + Max) * Fixed64.Half;

        Min = center - half;
        Max = center + half;
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
    public void SetBoundingBox(Vector3d center, Vector3d scope)
    {
        Vector3d normalizedScope = Vector3d.Abs(scope);
        Min = center - normalizedScope;
        Max = center + normalizedScope;
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
        if (Contains(sphere.Min) && Contains(sphere.Max))
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
        return Vector3d.DistanceSquared(sphere.Center, ClampPoint(sphere.Center)) <= sphere.RadiusSquared;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IntersectsSphereStrict(FixedBoundSphere sphere)
    {
        return HasPositiveVolume()
            && sphere.Radius > Fixed64.Zero
            && Vector3d.DistanceSquared(sphere.Center, ClampPoint(sphere.Center)) < sphere.RadiusSquared;
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
}
