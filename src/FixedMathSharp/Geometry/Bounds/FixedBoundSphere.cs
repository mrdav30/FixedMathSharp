//=======================================================================
// FixedBoundSphere.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Bounds;

/// <summary>
/// Represents a spherical bounding volume with fixed-point precision, optimized for fast, 
/// rotationally invariant spatial checks in 3D space.
/// </summary>
/// <remarks>
/// The FixedBoundSphere provides a simple yet effective way to represent the spatial extent of objects, 
/// especially when rotational invariance is required. 
/// Compared to FixedBoundBox, it offers faster intersection checks but is less precise in 
/// tightly fitting non-spherical objects.
/// 
/// Use Cases:
/// - Ideal for broad-phase collision detection, proximity checks, and culling in physics engines and rendering pipelines.
/// - Useful when fast, rotationally invariant checks are needed, such as detecting overlaps or distances between moving objects.
/// - Suitable for encapsulating objects with roughly spherical shapes or objects that rotate frequently, where the bounding box may need constant updates.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct FixedBoundSphere : IEquatable<FixedBoundSphere>
{
    #region Fields

    /// <summary>
    /// The center point of the sphere.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Vector3d Center;

    /// <summary>
    /// The radius of the sphere.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Fixed64 Radius;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance of the FixedBoundSphere struct with the specified center and radius.
    /// </summary>
    [JsonConstructor]
    public FixedBoundSphere(Vector3d center, Fixed64 radius)
    {
        Center = center;
        Radius = radius;
    }

    #endregion

    #region Properties

    /// <summary>
    /// Gets the coordinates of the minimum corner of the bounding box that contains the sphere.
    /// </summary>
    /// <remarks>
    /// The minimum corner is calculated by subtracting the radius from each component of the sphere's center. 
    /// This property is useful for spatial queries and bounding box calculations.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Min
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Center - new Vector3d(Radius, Radius, Radius);
    }

    /// <summary>
    /// Gets the coordinates of the maximum corner of the bounding box that contains the sphere.
    /// </summary>
    /// <remarks>
    /// The maximum corner is calculated as the center of the sphere plus the radius in each dimension. 
    /// This property is useful for spatial queries and bounding volume calculations.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Max
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Center + new Vector3d(Radius, Radius, Radius);
    }

    /// <summary>
    /// The squared radius of the sphere.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 SqrRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Radius * Radius;
    }

    #endregion

    #region Methods (Static)

    /// <summary>
    /// Creates a bounding sphere that contains the specified axis-aligned bounding box.
    /// </summary>
    public static FixedBoundSphere CreateFromBoundingBox(FixedBoundBox box)
    {
        Vector3d center = (box.Min + box.Max) * Fixed64.Half;
        Fixed64 radius = Vector3d.Distance(center, box.Max);

        return new FixedBoundSphere(center, radius);
    }

    /// <summary>
    /// Creates a bounding sphere that contains the specified frustum.
    /// </summary>
    public static FixedBoundSphere CreateFromFrustum(FixedBoundFrustum frustum)
    {
        return CreateFromFrustumCorners(frustum);
    }

    /// <summary>
    /// Creates a bounding sphere that contains the specified points.
    /// </summary>
    public static FixedBoundSphere CreateFromPoints(IEnumerable<Vector3d> points)
    {
        if (points is null)
            throw new ArgumentNullException(nameof(points), "Cannot create a bounding sphere from a null collection of points.");

        if (points is IReadOnlyList<Vector3d> pointList)
            return CreateFromPointList(pointList);

        var materialized = new List<Vector3d>();
        foreach (Vector3d point in points)
            materialized.Add(point);

        return CreateFromPointList(materialized);
    }

    /// <summary>
    /// Creates the smallest sphere that contains the two specified spheres.
    /// </summary>
    public static FixedBoundSphere CreateMerged(FixedBoundSphere original, FixedBoundSphere additional)
    {
        Vector3d centerOffset = additional.Center - original.Center;
        Fixed64 distance = centerOffset.Magnitude;

        if (distance + additional.Radius <= original.Radius)
            return original;

        if (distance + original.Radius <= additional.Radius)
            return additional;

        Fixed64 radius = (distance + original.Radius + additional.Radius) * Fixed64.Half;
        Vector3d center = original.Center + centerOffset * Fixed64.DivideByPositive(radius - original.Radius, distance);

        return new FixedBoundSphere(center, radius);
    }

    private static FixedBoundSphere CreateFromPointList(IReadOnlyList<Vector3d> points)
    {
        if (points.Count == 0)
            throw new ArgumentException("At least one point is required to create a bounding sphere.");

        Vector3d minX = points[0];
        Vector3d maxX = points[0];
        Vector3d minY = points[0];
        Vector3d maxY = points[0];
        Vector3d minZ = points[0];
        Vector3d maxZ = points[0];

        for (int i = 1; i < points.Count; i++)
        {
            Vector3d point = points[i];

            if (point.X < minX.X) minX = point;
            if (point.X > maxX.X) maxX = point;
            if (point.Y < minY.Y) minY = point;
            if (point.Y > maxY.Y) maxY = point;
            if (point.Z < minZ.Z) minZ = point;
            if (point.Z > maxZ.Z) maxZ = point;
        }

        Fixed64 sqDistX = Vector3d.SqrDistance(maxX, minX);
        Fixed64 sqDistY = Vector3d.SqrDistance(maxY, minY);
        Fixed64 sqDistZ = Vector3d.SqrDistance(maxZ, minZ);

        Vector3d min = minX;
        Vector3d max = maxX;
        Fixed64 largestDistance = sqDistX;

        if (sqDistY > largestDistance)
        {
            min = minY;
            max = maxY;
            largestDistance = sqDistY;
        }

        if (sqDistZ > largestDistance)
        {
            min = minZ;
            max = maxZ;
        }

        Vector3d center = (min + max) * Fixed64.Half;
        Fixed64 radius = Vector3d.Distance(max, center);
        Fixed64 sqRadius = radius * radius;

        for (int i = 0; i < points.Count; i++)
        {
            Vector3d diff = points[i] - center;
            Fixed64 sqDistance = diff.SqrMagnitude;
            if (sqDistance <= sqRadius)
                continue;

            Fixed64 distance = FixedMath.Sqrt(sqDistance);
            Fixed64 newRadius = (radius + distance) * Fixed64.Half;
            center += diff * Fixed64.DivideByPositive(distance - radius, Fixed64.Two * distance);
            radius = newRadius;
            sqRadius = EnsureRadiusContainsPoint(points[i], center, ref radius);
        }

        return new FixedBoundSphere(center, radius);
    }

    private static FixedBoundSphere CreateFromFrustumCorners(FixedBoundFrustum frustum)
    {
        Vector3d minX = frustum.GetCorner(0);
        Vector3d maxX = minX;
        Vector3d minY = minX;
        Vector3d maxY = minX;
        Vector3d minZ = minX;
        Vector3d maxZ = minX;

        for (int i = 1; i < FixedBoundFrustum.CornerCount; i++)
        {
            Vector3d point = frustum.GetCorner(i);

            if (point.X < minX.X) minX = point;
            if (point.X > maxX.X) maxX = point;
            if (point.Y < minY.Y) minY = point;
            if (point.Y > maxY.Y) maxY = point;
            if (point.Z < minZ.Z) minZ = point;
            if (point.Z > maxZ.Z) maxZ = point;
        }

        Fixed64 sqDistX = Vector3d.SqrDistance(maxX, minX);
        Fixed64 sqDistY = Vector3d.SqrDistance(maxY, minY);
        Fixed64 sqDistZ = Vector3d.SqrDistance(maxZ, minZ);

        Vector3d min = minX;
        Vector3d max = maxX;
        Fixed64 largestDistance = sqDistX;

        if (sqDistY > largestDistance)
        {
            min = minY;
            max = maxY;
            largestDistance = sqDistY;
        }

        if (sqDistZ > largestDistance)
        {
            min = minZ;
            max = maxZ;
        }

        Vector3d center = (min + max) * Fixed64.Half;
        Fixed64 radius = Vector3d.Distance(max, center);
        Fixed64 sqRadius = radius * radius;

        for (int i = 0; i < FixedBoundFrustum.CornerCount; i++)
        {
            Vector3d point = frustum.GetCorner(i);
            Vector3d diff = point - center;
            Fixed64 sqDistance = diff.SqrMagnitude;
            if (sqDistance <= sqRadius)
                continue;

            Fixed64 distance = FixedMath.Sqrt(sqDistance);
            Fixed64 newRadius = (radius + distance) * Fixed64.Half;
            center += diff * Fixed64.DivideByPositive(distance - radius, Fixed64.Two * distance);
            radius = newRadius;
            sqRadius = EnsureRadiusContainsPoint(point, center, ref radius);
        }

        return new FixedBoundSphere(center, radius);
    }

    private static Fixed64 EnsureRadiusContainsPoint(Vector3d point, Vector3d center, ref Fixed64 radius)
    {
        Fixed64 sqRadius = radius * radius;
        Fixed64 sqDistance = Vector3d.SqrDistance(point, center);

        if (sqDistance <= sqRadius)
            return sqRadius;

        radius = FixedMath.Sqrt(sqDistance);
        sqRadius = radius * radius;

        if (sqRadius < sqDistance)
        {
            radius += Fixed64.MinIncrement;
            sqRadius = radius * radius;
        }

        return sqRadius;
    }

    #endregion

    #region Methods (Instance)

    /// <summary>
    /// Checks if a point is inside the sphere.
    /// </summary>
    /// <param name="point">The point to check.</param>
    /// <returns>True if the point is inside the sphere, otherwise false.</returns>
    public bool Contains(Vector3d point)
    {
        return Vector3d.SqrDistance(Center, point) <= SqrRadius;
    }

    /// <summary>
    /// Tests a bounding box against this sphere.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundBox box)
    {
        return ContainsBoxLike(box.Min, box.Max);
    }

    /// <summary>
    /// Tests a bounding area against this sphere.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundArea area)
    {
        return ContainsBoxLike(area.Min, area.Max);
    }

    /// <summary>
    /// Tests another sphere against this sphere.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundSphere sphere)
    {
        Fixed64 sqDistance = Vector3d.SqrDistance(Center, sphere.Center);
        Fixed64 combinedRadius = Radius + sphere.Radius;

        if (sqDistance > combinedRadius * combinedRadius)
            return FixedEnclosureType.Disjoint;

        Fixed64 radiusDifference = Radius - sphere.Radius;
        if (radiusDifference >= Fixed64.Zero && sqDistance <= radiusDifference * radiusDifference)
            return FixedEnclosureType.Contains;

        return FixedEnclosureType.Intersects;
    }

    /// <summary>
    /// Tests a frustum against this sphere.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundFrustum frustum)
    {
        bool containsAllCorners = true;

        for (int i = 0; i < FixedBoundFrustum.CornerCount; i++)
        {
            if (!Contains(frustum.GetCorner(i)))
            {
                containsAllCorners = false;
                break;
            }
        }

        if (containsAllCorners)
            return FixedEnclosureType.Contains;

        return frustum.Intersects(this)
            ? FixedEnclosureType.Intersects
            : FixedEnclosureType.Disjoint;
    }

    /// <summary>
    /// Checks whether a bounding box intersects this sphere.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundBox box) => Contains(box) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Checks whether a bounding area intersects this sphere.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundArea area) => Contains(area) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Checks whether another sphere intersects this sphere.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundSphere sphere) => Contains(sphere) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Checks whether a frustum intersects this sphere.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundFrustum frustum)
    {
        return frustum.Intersects(this);
    }

    /// <summary>
    /// Classifies this sphere relative to a plane.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedPlaneIntersectionType Intersects(FixedPlane plane) => plane.Intersects(this);

    /// <summary>
    /// Finds the first forward ray intersection with this sphere.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64? Intersects(FixedRay ray) => ray.Intersects(this);

    /// <summary>
    /// Projects a point onto the bounding sphere. If the point is outside the sphere, it returns the closest point on the surface.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ProjectPoint(Vector3d point)
    {
        var direction = point - Center;
        if (direction.IsZero) return Center; // If the point is the center, return the center itself

        return Center + direction.Normalize() * Radius;
    }

    /// <summary>
    /// Clamps a point to this sphere, returning the point unchanged when it is already inside the sphere.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ClampPoint(Vector3d point)
    {
        if (Contains(point))
            return point;

        return ProjectPoint(point);
    }

    /// <summary>
    /// Calculates the distance from a point to the surface of the sphere.
    /// </summary>
    /// <param name="point">The point to calculate the distance from.</param>
    /// <returns>The distance from the point to the surface of the sphere.</returns>
    public Fixed64 DistanceToSurface(Vector3d point)
    {
        return Vector3d.Distance(Center, point) - Radius;
    }

    /// <summary>
    /// Creates a sphere that contains this sphere transformed by the specified matrix.
    /// </summary>
    public FixedBoundSphere Transform(Fixed4x4 matrix)
    {
        Vector3d center = Fixed4x4.TransformPoint(matrix, Center);
        Fixed64 scale = GetMaxBasisScale(matrix);

        return new FixedBoundSphere(center, Radius * scale);
    }

    /// <summary>
    /// Deconstructs this sphere into its center and radius.
    /// </summary>
    public void Deconstruct(out Vector3d center, out Fixed64 radius)
    {
        center = Center;
        radius = Radius;
    }

    private FixedEnclosureType ContainsBoxLike(Vector3d min, Vector3d max)
    {
        bool containsAllCorners =
            Contains(new Vector3d(min.X, min.Y, min.Z)) &&
            Contains(new Vector3d(max.X, min.Y, min.Z)) &&
            Contains(new Vector3d(min.X, max.Y, min.Z)) &&
            Contains(new Vector3d(max.X, max.Y, min.Z)) &&
            Contains(new Vector3d(min.X, min.Y, max.Z)) &&
            Contains(new Vector3d(max.X, min.Y, max.Z)) &&
            Contains(new Vector3d(min.X, max.Y, max.Z)) &&
            Contains(new Vector3d(max.X, max.Y, max.Z));

        if (containsAllCorners)
            return FixedEnclosureType.Contains;

        Vector3d closest = new(
            FixedMath.Clamp(Center.X, min.X, max.X),
            FixedMath.Clamp(Center.Y, min.Y, max.Y),
            FixedMath.Clamp(Center.Z, min.Z, max.Z));

        return Vector3d.SqrDistance(Center, closest) <= SqrRadius
            ? FixedEnclosureType.Intersects
            : FixedEnclosureType.Disjoint;
    }

    private static Fixed64 GetMaxBasisScale(Fixed4x4 matrix)
    {
        Fixed64 row0 = matrix.M11 * matrix.M11 + matrix.M12 * matrix.M12 + matrix.M13 * matrix.M13;
        Fixed64 row1 = matrix.M21 * matrix.M21 + matrix.M22 * matrix.M22 + matrix.M23 * matrix.M23;
        Fixed64 row2 = matrix.M31 * matrix.M31 + matrix.M32 * matrix.M32 + matrix.M33 * matrix.M33;

        return FixedMath.Sqrt(FixedMath.Max(row0, FixedMath.Max(row1, row2)));
    }

    #endregion

    #region Operators

    /// <summary>
    /// Determines whether two FixedBoundSphere instances are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedBoundSphere left, FixedBoundSphere right) => left.Equals(right);

    /// <summary>
    /// Determines whether two FixedBoundSphere instances are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedBoundSphere left, FixedBoundSphere right) => !left.Equals(right);

    #endregion

    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is FixedBoundSphere other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedBoundSphere other) => Center.Equals(other.Center) && Radius.Equals(other.Radius);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 23 + Center.GetHashCode();
            hash = hash * 23 + Radius.GetHashCode();
            return hash;
        }
    }

    /// <summary>
    /// Returns a string that represents the current FixedBoundSphere.
    /// </summary>
    public override string ToString()
    {
        return $"{{Center:{Center} Radius:{Radius}}}";
    }

    #endregion
}
