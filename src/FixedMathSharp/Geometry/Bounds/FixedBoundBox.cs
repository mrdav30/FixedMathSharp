//=======================================================================
// FixedBoundBox.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

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
    /// <value>The componentwise minimum corner in the bound's coordinate space.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Min { get; private set; }

    /// <summary>
    /// The maximum corner of the bounding box.
    /// </summary>
    /// <value>The componentwise maximum corner in the bound's coordinate space.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Max { get; private set; }

    /// <summary>
    /// The center of the bounding box, rounded to the nearest-even Q32.32 lattice point.
    /// </summary>
    /// <value>The midpoint of the stored endpoints. Setting it moves both endpoints.</value>
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
    /// <value>The full positive span along each axis. Setting it resizes the box about its current center.</value>
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
    /// <value>The outward-rounded half-extents around the current center.</value>
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
    /// <value>A copy of the normalized state. Assigning a state replaces the stored value.</value>
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
    /// <param name="min">One corner. The lesser coordinate on each axis becomes the minimum.</param>
    /// <param name="max">The opposite corner. The greater coordinate on each axis becomes the maximum.</param>
    /// <returns>The normalized box spanning the supplied corners.</returns>
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
    /// <param name="center">The center in the bound's coordinate space.</param>
    /// <param name="size">The full size along each axis. Negative components are treated as magnitudes.</param>
    /// <returns>The box with both endpoints representable.</returns>
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
    /// <param name="center">The center in the bound's coordinate space.</param>
    /// <param name="scope">The half-size along each axis. Negative components are treated as magnitudes.</param>
    /// <returns>The box with both endpoints representable.</returns>
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
    /// <param name="center">The center in the bound's coordinate space.</param>
    /// <param name="size">The full size along each axis. Negative components are treated as magnitudes.</param>
    /// <returns>The box with its final endpoints clipped to the representable coordinate range.</returns>
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
    /// <param name="center">The center in the bound's coordinate space.</param>
    /// <param name="scope">The half-size along each axis. Negative components are treated as magnitudes.</param>
    /// <returns>The box with its final endpoints clipped to the representable coordinate range.</returns>
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
    /// <param name="sourceOrigin">The source frame's origin in world coordinates.</param>
    /// <param name="sourceRotation">The normalized rotation from the source frame to world space.</param>
    /// <param name="sourceLocalMin">The componentwise minimum in source-local coordinates.</param>
    /// <param name="sourceLocalMax">The componentwise maximum in source-local coordinates.</param>
    /// <param name="targetOrigin">The target frame's origin in world coordinates.</param>
    /// <param name="targetRotation">The normalized rotation from the target frame to world space.</param>
    /// <returns>
    /// The conservative axis-aligned box in target-local coordinates with final endpoints clipped to the scalar
    /// domain.
    /// </returns>
    /// <remarks>
    /// Conceptually transforms every source local point as
    /// <c>targetRotation^-1 * (sourceOrigin + sourceRotation * point - targetOrigin)</c>.
    /// Source endpoints are normalized component-wise before the exact
    /// projection. Intermediate world points are never materialized.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="sourceRotation"/> or <paramref name="targetRotation"/> is not normalized.
    /// </exception>
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
    /// <param name="center">The center in the bound's coordinate space.</param>
    /// <param name="size">
    /// The new full size by axis, with negative components treated as magnitudes. Pass <see langword="null"/> to
    /// translate the current bounds without resizing them.
    /// </param>
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
    /// <param name="size">The full size along each axis. Negative components are treated as magnitudes.</param>
    /// <remarks>The bounds are updated only after both endpoints are representable.</remarks>
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
    /// <param name="min">One corner. The lesser coordinate on each axis becomes the minimum.</param>
    /// <param name="max">The opposite corner. The greater coordinate on each axis becomes the maximum.</param>
    public void SetMinMax(Vector3d min, Vector3d max)
    {
        Min = Vector3d.Min(min, max);
        Max = Vector3d.Max(min, max);
    }

    /// <summary>
    /// Configures the bounding box with the specified center and scope (half-size).
    /// </summary>
    /// <param name="center">The center in the bound's coordinate space.</param>
    /// <param name="scope">The half-size along each axis. Negative components are treated as magnitudes.</param>
    /// <remarks>The bounds are updated only after both endpoints are representable.</remarks>
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
    /// <param name="point">The point in the same coordinate space as this box.</param>
    /// <returns><see langword="true"/> when the point lies on the boundary or inside and <see langword="false"/> otherwise.</returns>
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
    /// <param name="box">The axis-aligned box in the same coordinate space as this box.</param>
    /// <returns>
    /// <see cref="FixedEnclosureType.Contains"/> if the supplied box is wholly inside, including the boundary,
    /// <see cref="FixedEnclosureType.Intersects"/> for other overlap or contact, or
    /// <see cref="FixedEnclosureType.Disjoint"/> for separation.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundBox box) => ContainsBoxLike(box.Min, box.Max);

    /// <summary>
    /// Tests a bounding sphere against this bounding box.
    /// </summary>
    /// <param name="sphere">The sphere in the same coordinate space as this box.</param>
    /// <returns>
    /// <see cref="FixedEnclosureType.Contains"/> if the supplied sphere is wholly inside, including the boundary,
    /// <see cref="FixedEnclosureType.Intersects"/> for other overlap or contact, or
    /// <see cref="FixedEnclosureType.Disjoint"/> for separation.
    /// </returns>
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
    /// <param name="frustum">The frustum in the same coordinate space as this box.</param>
    /// <returns>
    /// <see cref="FixedEnclosureType.Contains"/> if all frustum corners are contained, otherwise the result of the
    /// frustum's intersection test with this box as <see cref="FixedEnclosureType.Intersects"/> or
    /// <see cref="FixedEnclosureType.Disjoint"/>.
    /// </returns>
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
    /// <param name="box">The axis-aligned box in the same coordinate space as this box.</param>
    /// <returns><see langword="true"/> when the shapes overlap or touch and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundBox box) => IntersectsBoxLike(box.Min, box.Max);

    /// <summary>
    /// Checks whether a bounding sphere intersects this bounding box, including boundary-only contact.
    /// </summary>
    /// <param name="sphere">The sphere in the same coordinate space as this box.</param>
    /// <returns><see langword="true"/> when the shapes overlap or touch and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundSphere sphere) => IntersectsSphere(sphere);

    /// <summary>
    /// Checks whether another bounding box overlaps this bounding box with positive volume on every axis.
    /// </summary>
    /// <param name="box">The axis-aligned box in the same coordinate space as this box.</param>
    /// <returns><see langword="true"/> when the shared region has positive volume and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundBox box) => HasStrictAxisOverlap(box.Min, box.Max);

    /// <summary>
    /// Checks whether a bounding sphere overlaps this bounding box with positive volume.
    /// </summary>
    /// <param name="sphere">The sphere in the same coordinate space as this box.</param>
    /// <returns><see langword="true"/> when the shared region has positive volume and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundSphere sphere) => IntersectsSphereStrict(sphere);

    /// <summary>Tests this bound against all six frustum planes.</summary>
    /// <param name="frustum">The frustum in the same coordinate space as this box.</param>
    /// <returns><see langword="true"/> when no frustum plane excludes the entire box and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundFrustum frustum)
    {
        return frustum.Intersects(this);
    }

    /// <inheritdoc cref="ClampPoint(Vector3d)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ProjectPoint(Vector3d point)
        => ClampPoint(point);

    /// <summary>
    /// Clamps a point to this bounding box, returning the point unchanged when it is already inside.
    /// </summary>
    /// <param name="point">The point in the same coordinate space as this box.</param>
    /// <returns>The supplied point if it is contained, or its closest point on the boundary.</returns>
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

    /// <summary>Selects a box surface point associated with the object position.</summary>
    /// <param name="objectPosition">The object position in the box's coordinate space.</param>
    /// <returns>
    /// The result of projecting the position into the box and passing that projection to
    /// <see cref="ClosestPointOnSurface(Vector3d)"/>.
    /// </returns>
    public Vector3d GetPointOnSurfaceTowardsObject(Vector3d objectPosition)
        => ClosestPointOnSurface(ProjectPoint(objectPosition));

    /// <summary>Selects a point on the box surface using the supplied point.</summary>
    /// <param name="point">The point in the same coordinate space as this box.</param>
    /// <returns>A point on the box surface. Points outside the box are clamped per axis.</returns>
    /// <remarks>
    /// For a point inside the box, sets one coordinate to the nearest face. Ties prefer X, then Y, then Z,
    /// and the minimum face within the selected axis.
    /// </remarks>
    public Vector3d ClosestPointOnSurface(Vector3d point)
    {
        if (!Contains(point))
            return ClampPoint(point);

        Vector3d min = Min;
        Vector3d max = Max;
        // The distances are nonnegative and may span the entire unsigned raw range.
        ulong minX = unchecked((ulong)(point.X.m_rawValue - min.X.m_rawValue));
        ulong maxX = unchecked((ulong)(max.X.m_rawValue - point.X.m_rawValue));
        ulong minY = unchecked((ulong)(point.Y.m_rawValue - min.Y.m_rawValue));
        ulong maxY = unchecked((ulong)(max.Y.m_rawValue - point.Y.m_rawValue));
        ulong minZ = unchecked((ulong)(point.Z.m_rawValue - min.Z.m_rawValue));
        ulong maxZ = unchecked((ulong)(max.Z.m_rawValue - point.Z.m_rawValue));
        ulong x = Math.Min(minX, maxX);
        ulong y = Math.Min(minY, maxY);
        ulong z = Math.Min(minZ, maxZ);

        if (x <= y && x <= z)
            point.X = minX <= maxX ? min.X : max.X;
        else if (y <= z)
            point.Y = minY <= maxY ? min.Y : max.Y;
        else
            point.Z = minZ <= maxZ ? min.Z : max.Z;

        return point;
    }

    /// <summary>
    /// Gets a stable corner without allocating.
    /// </summary>
    /// <param name="index">The zero-based index from 0 through 7.</param>
    /// <returns>The selected corner in the bound's coordinate space.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> is outside the valid corner range.</exception>
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
    /// <param name="destination">
    /// Caller-owned storage with at least 8 elements. The first 8 elements are overwritten in
    /// <see cref="GetCorner(int)"/> order. Extra elements are preserved.
    /// </param>
    /// <exception cref="ArgumentException"><paramref name="destination"/> has fewer than 8 elements.</exception>
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
    /// <param name="other">The box to enclose along with this box.</param>
    /// <returns>The added volume in cubic coordinate units, rounded down and clamped to <see cref="long.MaxValue"/>.</returns>
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
    /// <param name="a">The first box.</param>
    /// <param name="b">The second box in the same coordinate space as <paramref name="a"/>.</param>
    /// <returns>The smallest axis-aligned box that contains both inputs.</returns>
    public static FixedBoundBox Union(FixedBoundBox a, FixedBoundBox b)
    {
        return FromMinMax(Vector3d.Min(a.Min, b.Min), Vector3d.Max(a.Max, b.Max));
    }

    /// <summary>Selects a surface point on the first box from projections of the second box's corners.</summary>
    /// <param name="a">The first box.</param>
    /// <param name="b">The second box in the same coordinate space as <paramref name="a"/>.</param>
    /// <returns>
    /// The surface candidate on <paramref name="a"/> selected from the corners of <paramref name="b"/>, or
    /// <see cref="Vector3d.Zero"/> if all computed distances equal <see cref="Fixed64.MaxValue"/>.
    /// </returns>
    /// <remarks>
    /// Candidates come from <see cref="ClosestPointOnSurface(Vector3d)"/> in the second box's
    /// <see cref="GetCorner(int)"/> order. Ties retain the first candidate.
    /// </remarks>
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

    /// <summary>Tests whether the stored geometry is equal.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the minimum and maximum corners match and <see langword="false"/> otherwise.</returns>
    public static bool operator ==(FixedBoundBox left, FixedBoundBox right) => left.Equals(right);

    /// <summary>Tests whether the stored geometry is different.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the stored values differ and <see langword="false"/> otherwise.</returns>
    public static bool operator !=(FixedBoundBox left, FixedBoundBox right) => !left.Equals(right);

    #endregion

    #region Equality and HashCode Overrides

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="obj">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when <paramref name="obj"/> is a <see cref="FixedBoundBox"/> and the minimum and
    /// maximum corners match and <see langword="false"/> otherwise.
    /// </returns>
    public override bool Equals(object? obj) => obj is FixedBoundBox other && Equals(other);

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="other">The value to compare with this instance.</param>
    /// <returns><see langword="true"/> when the minimum and maximum corners match and <see langword="false"/> otherwise.</returns>
    public bool Equals(FixedBoundBox other)
        => Min.Equals(other.Min) && Max.Equals(other.Max);

    /// <summary>Computes a hash code for the stored geometry.</summary>
    /// <returns>A hash code consistent with structural equality.</returns>
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
