//=======================================================================
// FixedBoundSphere.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp.Geometry;

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
public partial struct FixedBoundSphere : IEquatable<FixedBoundSphere>, IFormattable
#if NET8_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Nested Types

    /// <summary>
    /// Represents the normalized serializable state of a three-dimensional spherical bound.
    /// </summary>
    [Serializable]
    [MemoryPackable]
    public readonly partial struct BoundingSphereState
    {
        /// <inheritdoc cref="FixedBoundSphere.Center"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector3d Center;

        /// <inheritdoc cref="FixedBoundSphere.Radius"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Fixed64 Radius;

        /// <summary>
        /// Initializes a normalized state from center and radius.
        /// </summary>
        [JsonConstructor]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingSphereState(Vector3d center, Fixed64 radius)
        {
            Center = center;
            Radius = NormalizeRadius(radius);
        }
    }

    #endregion

    #region Fields

    /// <summary>
    /// The radius backing field.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    private Fixed64 _radius;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance of the FixedBoundSphere struct with the specified center and radius.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedBoundSphere(Vector3d center, Fixed64 radius)
    {
        Center = center;
        _radius = NormalizeRadius(radius);
    }

    /// <summary>
    /// Initializes a new instance from serialized or caller-provided state.
    /// </summary>
    [JsonConstructor]
    public FixedBoundSphere(BoundingSphereState state)
    {
        Center = state.Center;
        _radius = state.Radius;
    }

    #endregion

    #region Properties

    /// <summary>
    /// The center point of the sphere.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Center { get; set; }

    /// <summary>
    /// The non-negative radius of the sphere. Assigned values are normalized by absolute value.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Radius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _radius;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _radius = NormalizeRadius(value);
    }

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
    /// Gets the current normalized state of the sphere.
    /// </summary>
    [JsonInclude]
    [MemoryPackInclude]
    public BoundingSphereState State => new(Center, Radius);

    #endregion

    #region Methods (Static)

    /// <summary>
    /// Creates a bounding sphere that contains the specified axis-aligned bounding box.
    /// </summary>
    /// <exception cref="OverflowException">
    /// Thrown when this construction requires an unrepresentable radius.
    /// </exception>
    public static FixedBoundSphere CreateFromBoundingBox(FixedBoundBox box)
    {
        Vector3d center = Vector3d.Midpoint(box.Min, box.Max);
        Vector3d farthestCorner = new(
            GetFartherEndpoint(box.Min.X, box.Max.X, center.X),
            GetFartherEndpoint(box.Min.Y, box.Max.Y, center.Y),
            GetFartherEndpoint(box.Min.Z, box.Max.Z, center.Z));
        if (!TryGetRequiredRadius(center, farthestCorner, Fixed64.Zero, out Fixed64 radius))
            throw CreateUnrepresentableRadiusException();

        return new FixedBoundSphere(center, radius);
    }

    /// <summary>
    /// Creates a bounding sphere that contains the specified frustum.
    /// </summary>
    /// <exception cref="OverflowException">
    /// Thrown when this construction requires an unrepresentable radius.
    /// </exception>
    public static FixedBoundSphere CreateFromFrustum(FixedBoundFrustum frustum)
    {
        return CreateFromFrustumCorners(frustum);
    }

    /// <summary>
    /// Creates a bounding sphere that contains the specified points.
    /// </summary>
    /// <exception cref="OverflowException">
    /// Thrown when this construction requires an unrepresentable radius.
    /// </exception>
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
    /// Creates a bounding sphere that contains the specified points.
    /// </summary>
    /// <exception cref="OverflowException">
    /// Thrown when this construction requires an unrepresentable radius.
    /// </exception>
    public static FixedBoundSphere CreateFromPoints(Vector3d[] points)
    {
        if (points is null)
            throw new ArgumentNullException(nameof(points), "Cannot create a bounding sphere from a null collection of points.");

        return CreateFromPoints(points.AsSpan());
    }

    /// <summary>
    /// Creates a bounding sphere that contains the specified points.
    /// </summary>
    /// <exception cref="OverflowException">
    /// Thrown when this construction requires an unrepresentable radius.
    /// </exception>
    public static FixedBoundSphere CreateFromPoints(ReadOnlySpan<Vector3d> points)
    {
        return CreateFromPointSpan(points);
    }

    /// <summary>
    /// Creates a deterministic sphere that contains the two specified spheres.
    /// </summary>
    /// <exception cref="OverflowException">
    /// Thrown when this construction cannot produce a containing sphere with a
    /// representable radius.
    /// </exception>
    public static FixedBoundSphere CreateMerged(FixedBoundSphere original, FixedBoundSphere additional)
    {
        if (ContainsSphere(original, additional))
            return original;

        if (ContainsSphere(additional, original))
            return additional;

        return MergeNonContaining(original, additional);
    }

    private static FixedBoundSphere MergeNonContaining(
        FixedBoundSphere original,
        FixedBoundSphere additional)
    {

        GetDistanceRoot(original.Center, additional.Center, out Signed192 distanceFloor, out Signed192 distanceRemainder);
        if (!TryGetMergedRadius(
                distanceFloor,
                distanceRemainder,
                original.Radius,
                additional.Radius,
                out Fixed64 radius))
        {
            throw CreateUnrepresentableRadiusException();
        }

        Signed192 firstGap = Signed192.Signed(
            (radius - original.Radius).m_rawValue);
        Signed192 secondGap = Signed192.Signed(
            (radius - additional.Radius).m_rawValue);
        Signed192 gapSum = WideArithmetic.AddSigned192(firstGap, secondGap);
        Vector3d center = new(
            WideGeometry.InterpolateCoordinate(
                original.Center.X,
                additional.Center.X,
                firstGap,
                gapSum),
            WideGeometry.InterpolateCoordinate(
                original.Center.Y,
                additional.Center.Y,
                firstGap,
                gapSum),
            WideGeometry.InterpolateCoordinate(
                original.Center.Z,
                additional.Center.Z,
                firstGap,
                gapSum));

        var merged = new FixedBoundSphere(center, radius);
        bool containsOriginal = ContainsSphere(merged, original);
        bool containsAdditional = ContainsSphere(merged, additional);
        if (containsOriginal & containsAdditional)
            return merged;

        bool originalRadiusRepresentable = TryGetRequiredRadius(
            center,
            original.Center,
            original.Radius,
            out Fixed64 originalRequired);
        bool additionalRadiusRepresentable = TryGetRequiredRadius(
            center,
            additional.Center,
            additional.Radius,
            out Fixed64 additionalRequired);
        if (!(originalRadiusRepresentable & additionalRadiusRepresentable))
        {
            throw CreateUnrepresentableRadiusException();
        }

        return new FixedBoundSphere(
            center,
            originalRequired >= additionalRequired ? originalRequired : additionalRequired);
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

        FixedBoundSphere sphere = CreateFromExtremePairs(minX, maxX, minY, maxY, minZ, maxZ);
        for (int i = 0; i < points.Count; i++)
            ExpandToContain(ref sphere, points[i]);

        return sphere;
    }

    private static FixedBoundSphere CreateFromPointSpan(ReadOnlySpan<Vector3d> points)
    {
        if (points.Length == 0)
            throw new ArgumentException("At least one point is required to create a bounding sphere.");

        Vector3d minX = points[0];
        Vector3d maxX = points[0];
        Vector3d minY = points[0];
        Vector3d maxY = points[0];
        Vector3d minZ = points[0];
        Vector3d maxZ = points[0];

        for (int i = 1; i < points.Length; i++)
        {
            Vector3d point = points[i];

            if (point.X < minX.X) minX = point;
            if (point.X > maxX.X) maxX = point;
            if (point.Y < minY.Y) minY = point;
            if (point.Y > maxY.Y) maxY = point;
            if (point.Z < minZ.Z) minZ = point;
            if (point.Z > maxZ.Z) maxZ = point;
        }

        FixedBoundSphere sphere = CreateFromExtremePairs(minX, maxX, minY, maxY, minZ, maxZ);
        for (int i = 0; i < points.Length; i++)
            ExpandToContain(ref sphere, points[i]);

        return sphere;
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

        FixedBoundSphere sphere = CreateFromExtremePairs(minX, maxX, minY, maxY, minZ, maxZ);
        for (int i = 0; i < FixedBoundFrustum.CornerCount; i++)
            ExpandToContain(ref sphere, frustum.GetCorner(i));

        return sphere;
    }

    private static FixedBoundSphere CreateFromExtremePairs(
        Vector3d minX,
        Vector3d maxX,
        Vector3d minY,
        Vector3d maxY,
        Vector3d minZ,
        Vector3d maxZ)
    {
        Vector3d min = minX;
        Vector3d max = maxX;
        if (Vector3d.CompareDistanceSquared(minY, maxY, min, max) > 0)
        {
            min = minY;
            max = maxY;
        }
        if (Vector3d.CompareDistanceSquared(minZ, maxZ, min, max) > 0)
        {
            min = minZ;
            max = maxZ;
        }

        Vector3d center = Vector3d.Midpoint(min, max);
        Vector3d farthest = Vector3d.CompareDistanceSquared(center, min, center, max) > 0
            ? min
            : max;
        if (!TryGetRequiredRadius(center, farthest, Fixed64.Zero, out Fixed64 radius))
        {
            throw CreateUnrepresentableRadiusException();
        }

        return new FixedBoundSphere(center, radius);
    }

    private static void ExpandToContain(ref FixedBoundSphere sphere, Vector3d point)
    {
        if (sphere.Contains(point))
            return;

        sphere = MergeNonContaining(sphere, new FixedBoundSphere(point, Fixed64.Zero));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ContainsSphere(FixedBoundSphere outer, FixedBoundSphere inner)
    {
        if (outer.Radius < inner.Radius)
            return false;

        return WideGeometry.CompareDistanceToRadiusSum(
            outer.Center,
            inner.Center,
            outer.Radius - inner.Radius,
            Fixed64.Zero) <= 0;
    }

    private static bool TryGetMergedRadius(
        Signed192 distanceFloor,
        Signed192 distanceRemainder,
        Fixed64 firstRadius,
        Fixed64 secondRadius,
        out Fixed64 radius)
    {
        Signed192 sum = WideArithmetic.AddSigned192(
            distanceFloor,
            Signed192.Signed(firstRadius.m_rawValue));
        sum = WideArithmetic.AddSigned192(
            sum,
            Signed192.Signed(secondRadius.m_rawValue));
        bool roundUp = (sum.Low & 1UL) != 0UL || !distanceRemainder.IsZero;
        WideArithmetic.GetMagnitude(sum, out ulong high, out ulong middle, out ulong low);
        WideArithmetic.ShiftRightOne(ref high, ref middle, ref low);
        if (roundUp)
        {
            Signed192 rounded = WideArithmetic.AddSigned192(
                new Signed192(high, middle, low),
                Signed192.Signed(1L));
            high = rounded.High;
            middle = rounded.Middle;
            low = rounded.Low;
        }

        return TryCreatePositiveRaw(high, middle, low, out radius);
    }

    private static bool TryGetRequiredRadius(
        Vector3d center,
        Vector3d enclosedCenter,
        Fixed64 enclosedRadius,
        out Fixed64 radius)
    {
        GetDistanceRoot(center, enclosedCenter, out Signed192 distanceFloor, out Signed192 distanceRemainder);
        if (!distanceRemainder.IsZero)
        {
            distanceFloor = WideArithmetic.AddSigned192(
                distanceFloor,
                Signed192.Signed(1L));
        }

        Signed192 required = WideArithmetic.AddSigned192(
            distanceFloor,
            Signed192.Signed(enclosedRadius.m_rawValue));
        WideArithmetic.GetMagnitude(required, out ulong high, out ulong middle, out ulong low);
        return TryCreatePositiveRaw(high, middle, low, out radius);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetDistanceRoot(
        Vector3d first,
        Vector3d second,
        out Signed192 floor,
        out Signed192 remainder)
    {
        Signed192 squaredDistance = WideGeometry.GetDifferenceDotProduct3D(
            first.X, second.X, first.Y, second.Y, first.Z, second.Z,
            first.X, second.X, first.Y, second.Y, first.Z, second.Z);
        floor = WideArithmetic.GetFloorSquareRoot(
            Signed320.ExtendValue(squaredDistance),
            out remainder);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryCreatePositiveRaw(
        ulong high,
        ulong middle,
        ulong low,
        out Fixed64 value)
    {
        if ((high | middle | (low >> 63)) != 0UL)
        {
            value = default;
            return false;
        }

        value = Fixed64.FromRaw((long)low);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetFartherEndpoint(Fixed64 first, Fixed64 second, Fixed64 center)
    {
        ulong firstDistance = GetRawDistance(first.m_rawValue, center.m_rawValue);
        ulong secondDistance = GetRawDistance(second.m_rawValue, center.m_rawValue);
        return firstDistance > secondDistance ? first : second;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong GetRawDistance(long first, long second) =>
        first >= second
            ? unchecked((ulong)first - (ulong)second)
            : unchecked((ulong)second - (ulong)first);

    private static OverflowException CreateUnrepresentableRadiusException() =>
        new("A containing sphere produced for the supplied geometry requires an unrepresentable radius.");

    #endregion

    #region Methods (Instance)

    /// <summary>
    /// Checks if a point is inside the sphere.
    /// </summary>
    /// <param name="point">The point to check.</param>
    /// <returns>True if the point is inside the sphere, otherwise false.</returns>
    public bool Contains(Vector3d point)
    {
        return WideGeometry.CompareDistanceToRadiusSum(Center, point, Radius, Fixed64.Zero) <= 0;
    }

    /// <summary>
    /// Returns whether the point lies strictly inside this sphere.
    /// </summary>
    /// <remarks>
    /// Boundary points and every point tested against a zero-radius sphere
    /// return <see langword="false"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ContainsStrict(Vector3d point) =>
        Radius > Fixed64.Zero
        && WideGeometry.CompareDistanceToRadiusSum(Center, point, Radius, Fixed64.Zero) < 0;

    /// <summary>
    /// Tests a bounding box against this sphere.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundBox box)
    {
        return ContainsBoxLike(box.Min, box.Max);
    }

    /// <summary>
    /// Tests another sphere against this sphere.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundSphere sphere)
    {
        if (WideGeometry.CompareDistanceToRadiusSum(Center, sphere.Center, Radius, sphere.Radius) > 0)
            return FixedEnclosureType.Disjoint;

        Fixed64 radiusDifference = Radius - sphere.Radius;
        if (radiusDifference >= Fixed64.Zero
            && WideGeometry.CompareDistanceToRadiusSum(
                Center,
                sphere.Center,
                radiusDifference,
                Fixed64.Zero) <= 0)
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
    /// Checks whether a bounding box intersects this sphere, including boundary-only contact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundBox box) => Contains(box) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Checks whether another sphere intersects this sphere, including boundary-only contact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundSphere sphere) => Contains(sphere) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Checks whether a bounding box overlaps this sphere with positive volume.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundBox box) => box.IntersectsStrict(this);

    /// <summary>
    /// Checks whether another sphere overlaps this sphere with positive volume.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundSphere sphere)
    {
        return Radius > Fixed64.Zero
            && sphere.Radius > Fixed64.Zero
            && WideGeometry.CompareDistanceToRadiusSum(Center, sphere.Center, Radius, sphere.Radius) < 0;
    }

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

        return Center + direction.NormalizeInPlace() * Radius;
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

        return WideGeometry.CompareDistanceToRadiusSum(Center, closest, Radius, Fixed64.Zero) <= 0
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 NormalizeRadius(Fixed64 radius) => FixedMath.Abs(radius);

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
    public override string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string that represents the current FixedBoundSphere.
    /// </summary>
    public string ToString(string? format, IFormatProvider? formatProvider)
    {
        FixedBoundSphere value = this;
        return FixedDiagnosticsFormatter.ToString((Span<char> destination, out int charsWritten) =>
            value.TryFormat(destination, out charsWritten, format.AsSpan(), formatProvider));
    }

    /// <summary>
    /// Formats this sphere into the provided destination buffer.
    /// </summary>
    public bool TryFormat(
        Span<char> destination,
        out int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        int written = 0;
        if (!FixedDiagnosticsFormatter.Append("{Center:", destination, ref written) ||
            !Center.TryFormat(destination[written..], out int centerChars, format, provider))
        {
            charsWritten = 0;
            return false;
        }

        written += centerChars;
        if (!FixedDiagnosticsFormatter.Append(" Radius:", destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(Radius, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append('}', destination, ref written))
        {
            charsWritten = 0;
            return false;
        }

        charsWritten = written;
        return true;
    }

    #endregion
}
