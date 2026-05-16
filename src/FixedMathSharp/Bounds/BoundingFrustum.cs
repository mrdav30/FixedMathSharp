using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Represents a viewing frustum extracted from a combined view-projection matrix.
/// </summary>
public sealed class BoundingFrustum : IEquatable<BoundingFrustum>
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

    private Fixed4x4 _matrix;
    private readonly Vector3d[] _corners;
    private readonly FixedPlane[] _planes;

    private static readonly (int Start, int End)[] Edges =
    {
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    };

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new frustum by extracting the planes and corners from a combined view-projection matrix.
    /// </summary>
    public BoundingFrustum(Fixed4x4 matrix)
    {
        _corners = new Vector3d[CornerCount];
        _planes = new FixedPlane[PlaneCount];
        Matrix = matrix;
    }

    #endregion

    #region Properties

    /// <summary>
    /// Gets or sets the matrix used to define this frustum.
    /// </summary>
    public Fixed4x4 Matrix
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _matrix;
        set
        {
            _matrix = value;
            CreatePlanes();
            CreateCorners();
            UpdateBounds();
        }
    }

    /// <summary>
    /// Gets the near clipping plane.
    /// </summary>
    public FixedPlane Near
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _planes[0];
    }

    /// <summary>
    /// Gets the far clipping plane.
    /// </summary>
    public FixedPlane Far
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _planes[1];
    }

    /// <summary>
    /// Gets the left clipping plane.
    /// </summary>
    public FixedPlane Left
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _planes[2];
    }

    /// <summary>
    /// Gets the right clipping plane.
    /// </summary>
    public FixedPlane Right
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _planes[3];
    }

    /// <summary>
    /// Gets the top clipping plane.
    /// </summary>
    public FixedPlane Top
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _planes[4];
    }

    /// <summary>
    /// Gets the bottom clipping plane.
    /// </summary>
    public FixedPlane Bottom
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _planes[5];
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
    public ContainmentType Contains(Vector3d point)
    {
        for (int i = 0; i < PlaneCount; i++)
        {
            if (_planes[i].DotCoordinate(point) > Fixed64.Zero)
                return ContainmentType.Disjoint;
        }

        return ContainmentType.Contains;
    }

    /// <summary>
    /// Tests a bounding box against this frustum.
    /// </summary>
    public ContainmentType Contains(BoundingBox box)
    {
        bool intersects = false;

        for (int i = 0; i < PlaneCount; i++)
        {
            switch (_planes[i].Intersects(box))
            {
                case FixedPlaneIntersectionType.Front:
                    return ContainmentType.Disjoint;
                case FixedPlaneIntersectionType.Intersecting:
                    intersects = true;
                    break;
            }
        }

        return intersects ? ContainmentType.Intersects : ContainmentType.Contains;
    }

    /// <summary>
    /// Tests a bounding area against this frustum.
    /// </summary>
    public ContainmentType Contains(BoundingArea area)
    {
        bool intersects = false;

        for (int i = 0; i < PlaneCount; i++)
        {
            switch (_planes[i].Intersects(area))
            {
                case FixedPlaneIntersectionType.Front:
                    return ContainmentType.Disjoint;
                case FixedPlaneIntersectionType.Intersecting:
                    intersects = true;
                    break;
            }
        }

        return intersects ? ContainmentType.Intersects : ContainmentType.Contains;
    }

    /// <summary>
    /// Tests a bounding sphere against this frustum.
    /// </summary>
    public ContainmentType Contains(BoundingSphere sphere)
    {
        bool intersects = false;

        for (int i = 0; i < PlaneCount; i++)
        {
            switch (_planes[i].Intersects(sphere))
            {
                case FixedPlaneIntersectionType.Front:
                    return ContainmentType.Disjoint;
                case FixedPlaneIntersectionType.Intersecting:
                    intersects = true;
                    break;
            }
        }

        return intersects ? ContainmentType.Intersects : ContainmentType.Contains;
    }

    /// <summary>
    /// Tests another frustum against this frustum.
    /// </summary>
    public ContainmentType Contains(BoundingFrustum frustum)
    {
        if (frustum == null)
            throw new ArgumentNullException(nameof(frustum));

        if (Equals(frustum))
            return ContainmentType.Contains;

        bool containsAllCorners = true;
        for (int i = 0; i < CornerCount; i++)
        {
            if (Contains(frustum._corners[i]) == ContainmentType.Disjoint)
            {
                containsAllCorners = false;
                break;
            }
        }

        if (containsAllCorners)
            return ContainmentType.Contains;

        return IntersectsFrustum(frustum)
            ? ContainmentType.Intersects
            : ContainmentType.Disjoint;
    }

    /// <summary>
    /// Checks whether a bounding box intersects this frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(BoundingBox box) => Contains(box) != ContainmentType.Disjoint;

    /// <summary>
    /// Checks whether a bounding area intersects this frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(BoundingArea area) => Contains(area) != ContainmentType.Disjoint;

    /// <summary>
    /// Checks whether a bounding sphere intersects this frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(BoundingSphere sphere) => Contains(sphere) != ContainmentType.Disjoint;

    /// <summary>
    /// Checks whether another frustum intersects this frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(BoundingFrustum frustum) => Contains(frustum) != ContainmentType.Disjoint;

    /// <summary>
    /// Finds the first forward intersection between the specified ray and this frustum.
    /// </summary>
    public Fixed64? Intersects(FixedRay ray)
    {
        Fixed64 tEnter = Fixed64.Zero;
        Fixed64 tExit = Fixed64.MAX_VALUE;

        for (int i = 0; i < PlaneCount; i++)
        {
            FixedPlane plane = _planes[i];
            Fixed64 distance = plane.DotCoordinate(ray.Position);
            Fixed64 denominator = plane.DotNormal(ray.Direction);

            if (FixedRay.IsNearlyZero(denominator))
            {
                if (distance > Fixed64.Zero)
                    return null;

                continue;
            }

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

            if (tEnter > tExit)
                return null;
        }

        return tExit < Fixed64.Zero ? null : tEnter;
    }

    /// <summary>
    /// Classifies this frustum relative to a plane.
    /// </summary>
    public FixedPlaneIntersectionType Intersects(FixedPlane plane)
    {
        FixedPlaneIntersectionType result = plane.Intersects(_corners[0]);

        for (int i = 1; i < CornerCount; i++)
        {
            if (plane.Intersects(_corners[i]) != result)
                return FixedPlaneIntersectionType.Intersecting;
        }

        return result;
    }

    /// <summary>
    /// Returns a copy of the frustum corner array.
    /// </summary>
    public Vector3d[] GetCorners()
    {
        var corners = new Vector3d[CornerCount];
        Array.Copy(_corners, corners, CornerCount);
        return corners;
    }

    /// <summary>
    /// Copies this frustum's corners into the specified array.
    /// </summary>
    public void GetCorners(Vector3d[] corners)
    {
        if (corners == null)
            throw new ArgumentNullException(nameof(corners));

        if (corners.Length < CornerCount)
            throw new ArgumentOutOfRangeException(nameof(corners));

        Array.Copy(_corners, corners, CornerCount);
    }

    private void CreatePlanes()
    {
        _planes[0] = FixedPlane.Normalize(new FixedPlane(-_matrix.m02, -_matrix.m12, -_matrix.m22, -_matrix.m32));
        _planes[1] = FixedPlane.Normalize(new FixedPlane(_matrix.m02 - _matrix.m03, _matrix.m12 - _matrix.m13, _matrix.m22 - _matrix.m23, _matrix.m32 - _matrix.m33));
        _planes[2] = FixedPlane.Normalize(new FixedPlane(-_matrix.m03 - _matrix.m00, -_matrix.m13 - _matrix.m10, -_matrix.m23 - _matrix.m20, -_matrix.m33 - _matrix.m30));
        _planes[3] = FixedPlane.Normalize(new FixedPlane(_matrix.m00 - _matrix.m03, _matrix.m10 - _matrix.m13, _matrix.m20 - _matrix.m23, _matrix.m30 - _matrix.m33));
        _planes[4] = FixedPlane.Normalize(new FixedPlane(_matrix.m01 - _matrix.m03, _matrix.m11 - _matrix.m13, _matrix.m21 - _matrix.m23, _matrix.m31 - _matrix.m33));
        _planes[5] = FixedPlane.Normalize(new FixedPlane(-_matrix.m03 - _matrix.m01, -_matrix.m13 - _matrix.m11, -_matrix.m23 - _matrix.m21, -_matrix.m33 - _matrix.m31));
    }

    private void CreateCorners()
    {
        _corners[0] = IntersectionPoint(_planes[0], _planes[2], _planes[4]);
        _corners[1] = IntersectionPoint(_planes[0], _planes[3], _planes[4]);
        _corners[2] = IntersectionPoint(_planes[0], _planes[3], _planes[5]);
        _corners[3] = IntersectionPoint(_planes[0], _planes[2], _planes[5]);
        _corners[4] = IntersectionPoint(_planes[1], _planes[2], _planes[4]);
        _corners[5] = IntersectionPoint(_planes[1], _planes[3], _planes[4]);
        _corners[6] = IntersectionPoint(_planes[1], _planes[3], _planes[5]);
        _corners[7] = IntersectionPoint(_planes[1], _planes[2], _planes[5]);
    }

    private void UpdateBounds()
    {
        Vector3d min = _corners[0];
        Vector3d max = _corners[0];

        for (int i = 1; i < CornerCount; i++)
        {
            min = Vector3d.Min(min, _corners[i]);
            max = Vector3d.Max(max, _corners[i]);
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

    private bool IntersectsFrustum(BoundingFrustum other)
    {
        for (int i = 0; i < PlaneCount; i++)
        {
            if (Separates(_planes[i].Normal, _corners, other._corners))
                return false;

            if (Separates(other._planes[i].Normal, _corners, other._corners))
                return false;
        }

        for (int i = 0; i < Edges.Length; i++)
        {
            Vector3d edgeA = _corners[Edges[i].End] - _corners[Edges[i].Start];

            for (int j = 0; j < Edges.Length; j++)
            {
                Vector3d edgeB = other._corners[Edges[j].End] - other._corners[Edges[j].Start];
                Vector3d axis = Vector3d.Cross(edgeA, edgeB);

                if (axis.SqrMagnitude <= Fixed64.Epsilon)
                    continue;

                if (Separates(axis, _corners, other._corners))
                    return false;
            }
        }

        return true;
    }

    private static bool Separates(Vector3d axis, Vector3d[] a, Vector3d[] b)
    {
        Project(axis, a, out Fixed64 minA, out Fixed64 maxA);
        Project(axis, b, out Fixed64 minB, out Fixed64 maxB);

        return maxA < minB || maxB < minA;
    }

    private static void Project(Vector3d axis, Vector3d[] corners, out Fixed64 min, out Fixed64 max)
    {
        min = Vector3d.Dot(axis, corners[0]);
        max = min;

        for (int i = 1; i < CornerCount; i++)
        {
            Fixed64 projected = Vector3d.Dot(axis, corners[i]);
            if (projected < min)
                min = projected;
            else if (projected > max)
                max = projected;
        }
    }

    #endregion

    #region Equality

    /// <inheritdoc/>
    public bool Equals(BoundingFrustum? other)
    {
        return other != null && _matrix.Equals(other._matrix);
    }

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is BoundingFrustum other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => _matrix.GetHashCode();

    #endregion
}
