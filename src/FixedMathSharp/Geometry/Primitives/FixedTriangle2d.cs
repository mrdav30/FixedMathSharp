//=======================================================================
// FixedTriangle2d.cs
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
/// Represents a triangle in two-dimensional fixed-point space.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct FixedTriangle2d : IEquatable<FixedTriangle2d>
{
    #region Constants

    /// <summary>
    /// The number of vertices in a triangle.
    /// </summary>
    public const int VertexCount = 3;

    /// <summary>
    /// The number of edges in a triangle.
    /// </summary>
    public const int EdgeCount = 3;

    #endregion

    #region Fields

    /// <summary>
    /// The first vertex.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Vector2d A;

    /// <summary>
    /// The second vertex.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Vector2d B;

    /// <summary>
    /// The third vertex.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(2)]
    public Vector2d C;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a triangle from ordered vertices.
    /// </summary>
    [JsonConstructor]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedTriangle2d(Vector2d a, Vector2d b, Vector2d c)
    {
        A = a;
        B = b;
        C = c;
    }

    #endregion

    #region Properties

    /// <summary>
    /// The signed area of the triangle. Positive values indicate counter-clockwise winding.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 SignedArea
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Vector2d.CrossProduct(B - A, C - A) * Fixed64.Half;
    }

    /// <summary>
    /// The non-negative area of the triangle.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Area
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedMath.Abs(SignedArea);
    }

    /// <summary>
    /// The normalized axis-aligned area that contains all vertices.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedBoundArea Bounds
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedBoundArea.FromMinMax(ComponentMin(ComponentMin(A, B), C), ComponentMax(ComponentMax(A, B), C));
    }

    /// <summary>
    /// The arithmetic center of the three vertices.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Centroid
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (A + B + C) / (Fixed64)3;
    }

    /// <summary>
    /// Returns true when the triangle has no positive area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public bool IsDegenerate
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Area <= Fixed64.Epsilon;
    }

    #endregion

    #region Geometry Access

    /// <summary>
    /// Gets a vertex by stable index: 0 = A, 1 = B, 2 = C.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d GetVertex(int index) =>
        index switch
        {
            0 => A,
            1 => B,
            2 => C,
            _ => throw new ArgumentOutOfRangeException(nameof(index), $"Vertex index must be between 0 and {VertexCount - 1}."),
        };

    /// <summary>
    /// Gets an edge by stable index: 0 = AB, 1 = BC, 2 = CA.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedSegment2d GetEdge(int index) =>
        index switch
        {
            0 => new FixedSegment2d(A, B),
            1 => new FixedSegment2d(B, C),
            2 => new FixedSegment2d(C, A),
            _ => throw new ArgumentOutOfRangeException(nameof(index), $"Edge index must be between 0 and {EdgeCount - 1}."),
        };

    /// <summary>
    /// Gets the point represented by barycentric weights for vertices B and C.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d GetPoint(Fixed64 weightB, Fixed64 weightC)
    {
        return Vector2d.BarycentricCoordinates(A, B, C, weightB, weightC);
    }

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Computes barycentric weights for a point relative to this triangle.
    /// </summary>
    /// <returns>
    /// True when the triangle has non-degenerate area; false when weights cannot be solved.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetBarycentricWeights(Vector2d point, out Fixed64 weightA, out Fixed64 weightB, out Fixed64 weightC)
    {
        Vector2d ab = B - A;
        Vector2d ac = C - A;
        Fixed64 denominator = Vector2d.CrossProduct(ab, ac);
        if (FixedMath.Abs(denominator) <= Fixed64.Epsilon)
        {
            weightA = Fixed64.Zero;
            weightB = Fixed64.Zero;
            weightC = Fixed64.Zero;
            return false;
        }

        Vector2d ap = point - A;
        weightB = Vector2d.CrossProduct(ap, ac) / denominator;
        weightC = Vector2d.CrossProduct(ab, ap) / denominator;
        weightA = Fixed64.One - weightB - weightC;
        return true;
    }

    /// <summary>
    /// Determines whether the point is inside the triangle, including edges and vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector2d point)
    {
        Fixed64 signedArea = SignedArea;
        if (FixedMath.Abs(signedArea) <= Fixed64.Epsilon)
            return IsPointOnAnyEdge(point);

        Fixed64 ab = Vector2d.CrossProduct(B - A, point - A);
        Fixed64 bc = Vector2d.CrossProduct(C - B, point - B);
        Fixed64 ca = Vector2d.CrossProduct(A - C, point - C);

        return signedArea > Fixed64.Zero
            ? ab >= -Fixed64.Epsilon && bc >= -Fixed64.Epsilon && ca >= -Fixed64.Epsilon
            : ab <= Fixed64.Epsilon && bc <= Fixed64.Epsilon && ca <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Finds the closest point on or inside this triangle to the supplied point.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ClosestPoint(Vector2d point)
    {
        return Contains(point) ? point : ClosestPointOnEdges(point);
    }

    /// <summary>
    /// Computes the squared distance from the supplied point to this triangle.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector2d point)
    {
        return Vector2d.DistanceSquared(point, ClosestPoint(point));
    }

    #endregion

    #region Deconstruction

    /// <summary>
    /// Deconstructs the triangle into ordered vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector2d a, out Vector2d b, out Vector2d c)
    {
        a = A;
        b = B;
        c = C;
    }

    #endregion

    #region Operators

    /// <summary>
    /// Determines whether two triangles have the same ordered vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedTriangle2d left, FixedTriangle2d right) => left.Equals(right);

    /// <summary>
    /// Determines whether two triangles have different ordered vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedTriangle2d left, FixedTriangle2d right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedTriangle2d other)
    {
        return A == other.A && B == other.B && C == other.C;
    }

    /// <inheritdoc />
    public override bool Equals(object? obj)
    {
        return obj is FixedTriangle2d other && Equals(other);
    }

    /// <inheritdoc />
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + A.StateHash;
            hash = (hash * 31) + B.StateHash;
            hash = (hash * 31) + C.StateHash;
            return hash;
        }
    }

    #endregion

    #region Helpers

    private bool IsPointOnAnyEdge(Vector2d point)
    {
        return GetEdge(0).DistanceSquared(point) <= Fixed64.Epsilon
            || GetEdge(1).DistanceSquared(point) <= Fixed64.Epsilon
            || GetEdge(2).DistanceSquared(point) <= Fixed64.Epsilon;
    }

    private Vector2d ClosestPointOnEdges(Vector2d point)
    {
        Vector2d best = GetEdge(0).ClosestPoint(point);
        Fixed64 bestDistance = Vector2d.DistanceSquared(point, best);
        TrySetCloserPoint(GetEdge(1), point, ref best, ref bestDistance);
        TrySetCloserPoint(GetEdge(2), point, ref best, ref bestDistance);
        return best;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TrySetCloserPoint(FixedSegment2d edge, Vector2d point, ref Vector2d best, ref Fixed64 bestDistance)
    {
        Vector2d candidate = edge.ClosestPoint(point);
        Fixed64 distance = Vector2d.DistanceSquared(point, candidate);
        if (distance >= bestDistance)
            return;

        bestDistance = distance;
        best = candidate;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ComponentMin(Vector2d a, Vector2d b)
    {
        return new Vector2d(FixedMath.Min(a.X, b.X), FixedMath.Min(a.Y, b.Y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ComponentMax(Vector2d a, Vector2d b)
    {
        return new Vector2d(FixedMath.Max(a.X, b.X), FixedMath.Max(a.Y, b.Y));
    }

    #endregion
}
