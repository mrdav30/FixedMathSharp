//=======================================================================
// FixedBoundFrustum.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Bounds;

/// <summary>
/// Represents a frustum bounded by six clipping planes.
/// </summary>
public struct FixedBoundFrustum : IEquatable<FixedBoundFrustum>
{
    #region Constants

    /// <summary>
    /// The number of planes in a frustum.
    /// </summary>
    public const int PlaneCount = 6;

    /// <summary>
    /// The number of corner points in a frustum.
    /// </summary>
    public const int CornerCount = 8;

    #endregion

    #region Fields

    private const int EdgeCount = 12;
    private static readonly int[] s_edgeStarts = { 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3 };
    private static readonly int[] s_edgeEnds = { 1, 2, 3, 0, 5, 6, 7, 4, 4, 5, 6, 7 };

    private bool _hasMatrix;
    private Fixed4x4 _matrix;
    private FixedPlane _near;
    private FixedPlane _far;
    private FixedPlane _left;
    private FixedPlane _right;
    private FixedPlane _top;
    private FixedPlane _bottom;
    private Vector3d _nearTopLeft;
    private Vector3d _nearTopRight;
    private Vector3d _nearBottomRight;
    private Vector3d _nearBottomLeft;
    private Vector3d _farTopLeft;
    private Vector3d _farTopRight;
    private Vector3d _farBottomRight;
    private Vector3d _farBottomLeft;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new frustum by extracting the planes and corners from a combined view-projection matrix.
    /// </summary>
    public FixedBoundFrustum(Fixed4x4 matrix)
    {
        this = default;
        SetMatrix(matrix);
    }

    /// <summary>
    /// Initializes a new frustum from six clipping planes.
    /// </summary>
    public FixedBoundFrustum(
        FixedPlane near,
        FixedPlane far,
        FixedPlane left,
        FixedPlane right,
        FixedPlane top,
        FixedPlane bottom)
    {
        this = default;
        SetPlanes(near, far, left, right, top, bottom);
    }

    /// <summary>
    /// Initializes a new frustum from six clipping planes in near, far, left, right, top, bottom order.
    /// </summary>
    public FixedBoundFrustum(FixedPlane[] planes)
    {
        if (planes is null)
            throw new ArgumentNullException(nameof(planes), "Cannot create a frustum from a null plane array.");

        if (planes.Length != PlaneCount)
            throw new ArgumentException($"A frustum must be defined by exactly {PlaneCount} planes.");

        this = default;
        SetPlanes(planes);
    }

    #endregion

    #region Properties

    /// <summary>
    /// Gets a value indicating whether this frustum was created from a matrix source.
    /// </summary>
    public bool HasMatrix
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _hasMatrix;
    }

    /// <summary>
    /// Gets or sets the matrix source used to define this frustum.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// Thrown when reading the matrix from a frustum that was created from planes.
    /// </exception>
    public Fixed4x4 Matrix
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _hasMatrix ? _matrix : throw new InvalidOperationException("This frustum was not created from a matrix.");
        set => SetMatrix(value);
    }

    /// <summary>
    /// Gets the near clipping plane.
    /// </summary>
    public FixedPlane Near
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _near;
    }

    /// <summary>
    /// Gets the far clipping plane.
    /// </summary>
    public FixedPlane Far
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _far;
    }

    /// <summary>
    /// Gets the left clipping plane.
    /// </summary>
    public FixedPlane Left
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _left;
    }

    /// <summary>
    /// Gets the right clipping plane.
    /// </summary>
    public FixedPlane Right
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _right;
    }

    /// <summary>
    /// Gets the top clipping plane.
    /// </summary>
    public FixedPlane Top
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _top;
    }

    /// <summary>
    /// Gets the bottom clipping plane.
    /// </summary>
    public FixedPlane Bottom
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _bottom;
    }

    /// <summary>
    /// Gets the minimum corner of the axis-aligned box that encloses the frustum.
    /// </summary>
    public Vector3d Min { get; private set; }

    /// <summary>
    /// Gets the maximum corner of the axis-aligned box that encloses the frustum.
    /// </summary>
    public Vector3d Max { get; private set; }

    #endregion

    #region Methods

    /// <summary>
    /// Tests a point against this frustum.
    /// </summary>
    public FixedEnclosureType Contains(Vector3d point)
    {
        return _near.DotCoordinate(point) > Fixed64.Zero
            || _far.DotCoordinate(point) > Fixed64.Zero
            || _left.DotCoordinate(point) > Fixed64.Zero
            || _right.DotCoordinate(point) > Fixed64.Zero
            || _top.DotCoordinate(point) > Fixed64.Zero
            || _bottom.DotCoordinate(point) > Fixed64.Zero
                ? FixedEnclosureType.Disjoint
                : FixedEnclosureType.Contains;
    }

    /// <summary>
    /// Tests a bounding box against this frustum.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundBox box)
    {
        bool intersects = false;

        if (DisjointOrIntersects(_near, box, ref intersects)
            || DisjointOrIntersects(_far, box, ref intersects)
            || DisjointOrIntersects(_left, box, ref intersects)
            || DisjointOrIntersects(_right, box, ref intersects)
            || DisjointOrIntersects(_top, box, ref intersects)
            || DisjointOrIntersects(_bottom, box, ref intersects))
            return FixedEnclosureType.Disjoint;

        return intersects ? FixedEnclosureType.Intersects : FixedEnclosureType.Contains;
    }

    /// <summary>
    /// Tests a bounding area against this frustum.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundArea area)
    {
        bool intersects = false;

        if (DisjointOrIntersects(_near, area, ref intersects)
            || DisjointOrIntersects(_far, area, ref intersects)
            || DisjointOrIntersects(_left, area, ref intersects)
            || DisjointOrIntersects(_right, area, ref intersects)
            || DisjointOrIntersects(_top, area, ref intersects)
            || DisjointOrIntersects(_bottom, area, ref intersects))
            return FixedEnclosureType.Disjoint;

        return intersects ? FixedEnclosureType.Intersects : FixedEnclosureType.Contains;
    }

    /// <summary>
    /// Tests a bounding sphere against this frustum.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundSphere sphere)
    {
        bool intersects = false;

        if (DisjointOrIntersects(_near, sphere, ref intersects)
            || DisjointOrIntersects(_far, sphere, ref intersects)
            || DisjointOrIntersects(_left, sphere, ref intersects)
            || DisjointOrIntersects(_right, sphere, ref intersects)
            || DisjointOrIntersects(_top, sphere, ref intersects)
            || DisjointOrIntersects(_bottom, sphere, ref intersects))
            return FixedEnclosureType.Disjoint;

        return intersects ? FixedEnclosureType.Intersects : FixedEnclosureType.Contains;
    }

    /// <summary>
    /// Tests another frustum against this frustum.
    /// </summary>
    public FixedEnclosureType Contains(FixedBoundFrustum frustum)
    {
        if (Equals(frustum))
            return FixedEnclosureType.Contains;

        bool containsAllCorners = true;
        for (int i = 0; i < CornerCount; i++)
        {
            if (Contains(frustum.GetCorner(i)) == FixedEnclosureType.Disjoint)
            {
                containsAllCorners = false;
                break;
            }
        }

        if (containsAllCorners)
            return FixedEnclosureType.Contains;

        return IntersectsFrustum(frustum)
            ? FixedEnclosureType.Intersects
            : FixedEnclosureType.Disjoint;
    }

    /// <summary>
    /// Checks whether a bounding box intersects this frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundBox box) => Contains(box) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Checks whether a bounding area intersects this frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundArea area) => Contains(area) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Checks whether a bounding sphere intersects this frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundSphere sphere) => Contains(sphere) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Checks whether another frustum intersects this frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundFrustum frustum) => Contains(frustum) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Finds the first forward intersection between the specified ray and this frustum.
    /// </summary>
    public Fixed64? Intersects(FixedRay ray)
    {
        Fixed64 tEnter = Fixed64.Zero;
        Fixed64 tExit = Fixed64.MaxValue;

        if (!ClipRay(_near, ray, ref tEnter, ref tExit)
            || !ClipRay(_far, ray, ref tEnter, ref tExit)
            || !ClipRay(_left, ray, ref tEnter, ref tExit)
            || !ClipRay(_right, ray, ref tEnter, ref tExit)
            || !ClipRay(_top, ray, ref tEnter, ref tExit)
            || !ClipRay(_bottom, ray, ref tEnter, ref tExit))
            return null;

        return tEnter;
    }

    /// <summary>
    /// Classifies this frustum relative to a plane.
    /// </summary>
    public FixedPlaneIntersectionType Intersects(FixedPlane plane)
    {
        FixedPlaneIntersectionType result = plane.Intersects(_nearTopLeft);

        for (int i = 1; i < CornerCount; i++)
        {
            if (plane.Intersects(GetCorner(i)) != result)
                return FixedPlaneIntersectionType.Intersecting;
        }

        return result;
    }

    /// <summary>
    /// Clamps a point to this frustum, returning the point unchanged when it is already inside.
    /// </summary>
    public Vector3d ClampPoint(Vector3d point)
    {
        if (Contains(point) != FixedEnclosureType.Disjoint)
            return point;

        Vector3d best = _nearTopLeft;
        Fixed64 bestDistance = Vector3d.DistanceSquared(point, best);

        for (int i = 1; i < CornerCount; i++)
            UpdateNearestCandidate(point, GetCorner(i), ref best, ref bestDistance);

        for (int i = 0; i < PlaneCount; i++)
        {
            Vector3d candidate = Vector3d.ProjectOnPlane(point, GetPlane(i));
            if (IsInside(candidate))
                UpdateNearestCandidate(point, candidate, ref best, ref bestDistance);
        }

        for (int i = 0; i < EdgeCount; i++)
        {
            Vector3d candidate = Vector3d.ClosestPointOnLineSegment(point, GetCorner(GetEdgeStart(i)), GetCorner(GetEdgeEnd(i)));
            UpdateNearestCandidate(point, candidate, ref best, ref bestDistance);
        }

        return best;
    }

    /// <summary>
    /// Returns a copy of the frustum corner array.
    /// </summary>
    public Vector3d[] GetCorners()
    {
        var corners = new Vector3d[CornerCount];
        GetCorners(corners);
        return corners;
    }

    /// <summary>
    /// Copies this frustum's corners into the specified array.
    /// </summary>
    public void GetCorners(Vector3d[] corners)
    {
        if (corners is null)
            throw new ArgumentNullException(nameof(corners), "Cannot copy corners to a null array.");

        if (corners.Length < CornerCount)
            throw new ArgumentOutOfRangeException(nameof(corners), $"The destination array must have at least {CornerCount} elements to hold all corners.");

        corners[0] = _nearTopLeft;
        corners[1] = _nearTopRight;
        corners[2] = _nearBottomRight;
        corners[3] = _nearBottomLeft;
        corners[4] = _farTopLeft;
        corners[5] = _farTopRight;
        corners[6] = _farBottomRight;
        corners[7] = _farBottomLeft;
    }

    /// <summary>
    /// Returns a copy of the frustum plane array in near, far, left, right, top, bottom order.
    /// </summary>
    public FixedPlane[] GetPlanes()
    {
        var planes = new FixedPlane[PlaneCount];
        GetPlanes(planes);
        return planes;
    }

    /// <summary>
    /// Copies this frustum's planes into the specified array in near, far, left, right, top, bottom order.
    /// </summary>
    public void GetPlanes(FixedPlane[] planes)
    {
        if (planes is null)
            throw new ArgumentNullException(nameof(planes), "Cannot copy planes to a null array.");

        if (planes.Length < PlaneCount)
            throw new ArgumentOutOfRangeException(nameof(planes), $"The destination array must have at least {PlaneCount} elements to hold all planes.");

        planes[0] = _near;
        planes[1] = _far;
        planes[2] = _left;
        planes[3] = _right;
        planes[4] = _top;
        planes[5] = _bottom;
    }

    /// <summary>
    /// Gets the corner at the specified stable index without allocating.
    /// </summary>
    public Vector3d GetCorner(int index)
    {
        switch (index)
        {
            case 0: return _nearTopLeft;
            case 1: return _nearTopRight;
            case 2: return _nearBottomRight;
            case 3: return _nearBottomLeft;
            case 4: return _farTopLeft;
            case 5: return _farTopRight;
            case 6: return _farBottomRight;
            case 7: return _farBottomLeft;
            default:
                throw new ArgumentOutOfRangeException(nameof(index), $"Corner index must be between 0 and {CornerCount - 1}.");
        }
    }

    /// <summary>
    /// Gets the plane at the specified stable index in near, far, left, right, top, bottom order without allocating.
    /// </summary>
    public FixedPlane GetPlane(int index)
    {
        switch (index)
        {
            case 0: return _near;
            case 1: return _far;
            case 2: return _left;
            case 3: return _right;
            case 4: return _top;
            case 5: return _bottom;
            default:
                throw new ArgumentOutOfRangeException(nameof(index), $"Plane index must be between 0 and {PlaneCount - 1}.");
        }
    }

    private void SetMatrix(Fixed4x4 matrix)
    {
        _matrix = matrix;
        _hasMatrix = true;
        CreatePlanes(matrix);
        CreateCorners();
        UpdateBounds();
    }

    private void SetPlanes(FixedPlane[] planes)
    {
        _near = FixedPlane.GetNormalized(planes[0]);
        _far = FixedPlane.GetNormalized(planes[1]);
        _left = FixedPlane.GetNormalized(planes[2]);
        _right = FixedPlane.GetNormalized(planes[3]);
        _top = FixedPlane.GetNormalized(planes[4]);
        _bottom = FixedPlane.GetNormalized(planes[5]);

        _hasMatrix = false;
        _matrix = default;
        CreateCorners();
        UpdateBounds();
    }

    private void SetPlanes(
        FixedPlane near,
        FixedPlane far,
        FixedPlane left,
        FixedPlane right,
        FixedPlane top,
        FixedPlane bottom)
    {
        _near = FixedPlane.GetNormalized(near);
        _far = FixedPlane.GetNormalized(far);
        _left = FixedPlane.GetNormalized(left);
        _right = FixedPlane.GetNormalized(right);
        _top = FixedPlane.GetNormalized(top);
        _bottom = FixedPlane.GetNormalized(bottom);

        _hasMatrix = false;
        _matrix = default;
        CreateCorners();
        UpdateBounds();
    }

    private void CreatePlanes(Fixed4x4 matrix)
    {
        _near = FixedPlane.GetNormalized(new FixedPlane(-matrix.M13, -matrix.M23, -matrix.M33, -matrix.M43));
        _far = FixedPlane.GetNormalized(new FixedPlane(matrix.M13 - matrix.M14, matrix.M23 - matrix.M24, matrix.M33 - matrix.M34, matrix.M43 - matrix.M44));
        _left = FixedPlane.GetNormalized(new FixedPlane(-matrix.M14 - matrix.M11, -matrix.M24 - matrix.M21, -matrix.M34 - matrix.M31, -matrix.M44 - matrix.M41));
        _right = FixedPlane.GetNormalized(new FixedPlane(matrix.M11 - matrix.M14, matrix.M21 - matrix.M24, matrix.M31 - matrix.M34, matrix.M41 - matrix.M44));
        _top = FixedPlane.GetNormalized(new FixedPlane(matrix.M12 - matrix.M14, matrix.M22 - matrix.M24, matrix.M32 - matrix.M34, matrix.M42 - matrix.M44));
        _bottom = FixedPlane.GetNormalized(new FixedPlane(-matrix.M14 - matrix.M12, -matrix.M24 - matrix.M22, -matrix.M34 - matrix.M32, -matrix.M44 - matrix.M42));
    }

    private void CreateCorners()
    {
        _nearTopLeft = IntersectionPoint(_near, _left, _top);
        _nearTopRight = IntersectionPoint(_near, _right, _top);
        _nearBottomRight = IntersectionPoint(_near, _right, _bottom);
        _nearBottomLeft = IntersectionPoint(_near, _left, _bottom);
        _farTopLeft = IntersectionPoint(_far, _left, _top);
        _farTopRight = IntersectionPoint(_far, _right, _top);
        _farBottomRight = IntersectionPoint(_far, _right, _bottom);
        _farBottomLeft = IntersectionPoint(_far, _left, _bottom);
    }

    private void UpdateBounds()
    {
        Vector3d min = _nearTopLeft;
        Vector3d max = _nearTopLeft;

        for (int i = 1; i < CornerCount; i++)
        {
            Vector3d corner = GetCorner(i);
            min = Vector3d.Min(min, corner);
            max = Vector3d.Max(max, corner);
        }

        Min = min;
        Max = max;
    }

    private static Vector3d IntersectionPoint(FixedPlane a, FixedPlane b, FixedPlane c)
    {
        Vector3d cross = Vector3d.Cross(b.Normal, c.Normal);
        Fixed64 denominator = Vector3d.Dot(a.Normal, cross);

        if (denominator == Fixed64.Zero)
            throw new InvalidOperationException("Frustum planes do not intersect at a unique point.");

        Vector3d v1 = cross * a.D;
        Vector3d v2 = Vector3d.Cross(c.Normal, a.Normal) * b.D;
        Vector3d v3 = Vector3d.Cross(a.Normal, b.Normal) * c.D;

        return -(v1 + v2 + v3) / denominator;
    }

    private bool IsInside(Vector3d point)
    {
        for (int i = 0; i < PlaneCount; i++)
        {
            if (GetPlane(i).DotCoordinate(point) > Fixed64.Zero)
                return false;
        }

        return true;
    }

    private static void UpdateNearestCandidate(
        Vector3d point,
        Vector3d candidate,
        ref Vector3d best,
        ref Fixed64 bestDistance)
    {
        Fixed64 candidateDistance = Vector3d.DistanceSquared(point, candidate);
        if (candidateDistance >= bestDistance)
            return;

        best = candidate;
        bestDistance = candidateDistance;
    }

    private bool IntersectsFrustum(in FixedBoundFrustum other)
    {
        for (int i = 0; i < PlaneCount; i++)
        {
            if (Separates(GetPlane(i).Normal, this, other))
                return false;

            if (Separates(other.GetPlane(i).Normal, this, other))
                return false;
        }

        for (int i = 0; i < EdgeCount; i++)
        {
            Vector3d edgeA = GetCorner(GetEdgeEnd(i)) - GetCorner(GetEdgeStart(i));

            for (int j = 0; j < EdgeCount; j++)
            {
                Vector3d edgeB = other.GetCorner(GetEdgeEnd(j)) - other.GetCorner(GetEdgeStart(j));
                Vector3d axis = Vector3d.Cross(edgeA, edgeB);

                if (axis.MagnitudeSquared <= Fixed64.Epsilon)
                    continue;

                if (Separates(axis, this, other))
                    return false;
            }
        }

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool DisjointOrIntersects(FixedPlane plane, FixedBoundBox box, ref bool intersects)
    {
        switch (plane.Intersects(box))
        {
            case FixedPlaneIntersectionType.Front:
                return true;
            case FixedPlaneIntersectionType.Intersecting:
                intersects = true;
                break;
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool DisjointOrIntersects(FixedPlane plane, FixedBoundArea area, ref bool intersects)
    {
        switch (plane.Intersects(area))
        {
            case FixedPlaneIntersectionType.Front:
                return true;
            case FixedPlaneIntersectionType.Intersecting:
                intersects = true;
                break;
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool DisjointOrIntersects(FixedPlane plane, FixedBoundSphere sphere, ref bool intersects)
    {
        switch (plane.Intersects(sphere))
        {
            case FixedPlaneIntersectionType.Front:
                return true;
            case FixedPlaneIntersectionType.Intersecting:
                intersects = true;
                break;
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ClipRay(FixedPlane plane, FixedRay ray, ref Fixed64 tEnter, ref Fixed64 tExit)
    {
        Fixed64 distance = plane.DotCoordinate(ray.Position);
        Fixed64 denominator = plane.DotNormal(ray.Direction);

        if (FixedRay.IsNearlyZero(denominator))
            return distance <= Fixed64.Zero;

        Fixed64 t = -distance / denominator;
        if (denominator < Fixed64.Zero)
        {
            if (t > tEnter)
                tEnter = t;
        }
        else if (t < tExit)
        {
            tExit = t;
        }

        return tEnter <= tExit;
    }

    private static bool Separates(Vector3d axis, in FixedBoundFrustum a, in FixedBoundFrustum b)
    {
        Project(axis, a, out Fixed64 minA, out Fixed64 maxA);
        Project(axis, b, out Fixed64 minB, out Fixed64 maxB);

        return maxA < minB || maxB < minA;
    }

    private static void Project(Vector3d axis, in FixedBoundFrustum frustum, out Fixed64 min, out Fixed64 max)
    {
        min = Vector3d.Dot(axis, frustum._nearTopLeft);
        max = min;

        for (int i = 1; i < CornerCount; i++)
        {
            Fixed64 projected = Vector3d.Dot(axis, frustum.GetCorner(i));
            if (projected < min)
                min = projected;
            else if (projected > max)
                max = projected;
        }
    }

    private static int GetEdgeStart(int edge)
    {
        return s_edgeStarts[edge];
    }

    private static int GetEdgeEnd(int edge)
    {
        return s_edgeEnds[edge];
    }

    #endregion

    #region Equality

    /// <inheritdoc/>
    public bool Equals(FixedBoundFrustum other)
    {
        return _near == other._near
            && _far == other._far
            && _left == other._left
            && _right == other._right
            && _top == other._top
            && _bottom == other._bottom;
    }

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is FixedBoundFrustum other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        return HashCode.Combine(_near, _far, _left, _right, _top, _bottom);
    }

    #endregion
}
