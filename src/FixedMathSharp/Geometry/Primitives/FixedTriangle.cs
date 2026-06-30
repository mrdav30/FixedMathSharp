//=======================================================================
// FixedTriangle.cs
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
/// Represents a triangle in three-dimensional fixed-point space.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct FixedTriangle : IEquatable<FixedTriangle>
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
    public Vector3d A;

    /// <summary>
    /// The second vertex.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Vector3d B;

    /// <summary>
    /// The third vertex.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(2)]
    public Vector3d C;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a triangle from ordered vertices.
    /// </summary>
    [JsonConstructor]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedTriangle(Vector3d a, Vector3d b, Vector3d c)
    {
        A = a;
        B = b;
        C = c;
    }

    #endregion

    #region Properties

    /// <summary>
    /// The unnormalized triangle normal from <c>cross(B - A, C - A)</c>.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d UnnormalizedNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Vector3d.Cross(B - A, C - A);
    }

    /// <summary>
    /// The normalized triangle normal. Degenerate triangles return <see cref="Vector3d.Zero"/>.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Normal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            Vector3d normal = UnnormalizedNormal;
            Fixed64 magnitudeSquared = normal.MagnitudeSquared;
            return magnitudeSquared <= Fixed64.Epsilon
                ? Vector3d.Zero
                : normal / FixedMath.Sqrt(magnitudeSquared);
        }
    }

    /// <summary>
    /// The non-negative surface area of the triangle.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Area
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => UnnormalizedNormal.Magnitude * Fixed64.Half;
    }

    /// <summary>
    /// The normalized axis-aligned box that contains all vertices.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedBoundBox Bounds
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedBoundBox.FromMinMax(ComponentMin(ComponentMin(A, B), C), ComponentMax(ComponentMax(A, B), C));
    }

    /// <summary>
    /// The arithmetic center of the three vertices.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Centroid
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (A + B + C) / (Fixed64)3;
    }

    /// <summary>
    /// Returns true when the triangle has no positive surface area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public bool IsDegenerate
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => UnnormalizedNormal.MagnitudeSquared <= Fixed64.Epsilon;
    }

    #endregion

    #region Geometry Access

    /// <summary>
    /// Gets a vertex by stable index: 0 = A, 1 = B, 2 = C.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetVertex(int index) =>
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
    public FixedSegment GetEdge(int index) =>
        index switch
        {
            0 => new FixedSegment(A, B),
            1 => new FixedSegment(B, C),
            2 => new FixedSegment(C, A),
            _ => throw new ArgumentOutOfRangeException(nameof(index), $"Edge index must be between 0 and {EdgeCount - 1}."),
        };

    /// <summary>
    /// Gets the point represented by barycentric weights for vertices B and C.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetPoint(Fixed64 weightB, Fixed64 weightC)
    {
        return Vector3d.BarycentricCoordinates(A, B, C, weightB, weightC);
    }

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Computes barycentric weights for the point projected onto this triangle's plane.
    /// </summary>
    /// <returns>
    /// True when the triangle has non-degenerate area; false when weights cannot be solved.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetProjectedBarycentricWeights(Vector3d point, out Fixed64 weightA, out Fixed64 weightB, out Fixed64 weightC)
    {
        Vector3d ab = B - A;
        Vector3d ac = C - A;
        Vector3d ap = point - A;

        Fixed64 abAb = Vector3d.Dot(ab, ab);
        Fixed64 abAc = Vector3d.Dot(ab, ac);
        Fixed64 acAc = Vector3d.Dot(ac, ac);
        Fixed64 apAb = Vector3d.Dot(ap, ab);
        Fixed64 apAc = Vector3d.Dot(ap, ac);
        Fixed64 denominator = (abAb * acAc) - (abAc * abAc);

        if (FixedMath.Abs(denominator) <= Fixed64.Epsilon)
        {
            weightA = Fixed64.Zero;
            weightB = Fixed64.Zero;
            weightC = Fixed64.Zero;
            return false;
        }

        weightB = ((acAc * apAb) - (abAc * apAc)) / denominator;
        weightC = ((abAb * apAc) - (abAc * apAb)) / denominator;
        weightA = Fixed64.One - weightB - weightC;
        return true;
    }

    /// <summary>
    /// Determines whether the point lies on or inside this triangle.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector3d point)
    {
        return DistanceSquared(point) <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Finds the closest point on or inside this triangle to the supplied point.
    /// </summary>
    public Vector3d ClosestPoint(Vector3d point)
    {
        if (IsDegenerate)
            return ClosestPointOnEdges(point);

        Vector3d ab = B - A;
        Vector3d ac = C - A;
        Vector3d ap = point - A;

        Fixed64 d1 = Vector3d.Dot(ab, ap);
        Fixed64 d2 = Vector3d.Dot(ac, ap);
        if (d1 <= Fixed64.Zero && d2 <= Fixed64.Zero)
            return A;

        Vector3d bp = point - B;
        Fixed64 d3 = Vector3d.Dot(ab, bp);
        Fixed64 d4 = Vector3d.Dot(ac, bp);
        if (d3 >= Fixed64.Zero && d4 <= d3)
            return B;

        Fixed64 vc = (d1 * d4) - (d3 * d2);
        if (vc <= Fixed64.Zero && d1 >= Fixed64.Zero && d3 <= Fixed64.Zero)
        {
            Fixed64 v = d1 / (d1 - d3);
            return A + ab * v;
        }

        Vector3d cp = point - C;
        Fixed64 d5 = Vector3d.Dot(ab, cp);
        Fixed64 d6 = Vector3d.Dot(ac, cp);
        if (d6 >= Fixed64.Zero && d5 <= d6)
            return C;

        Fixed64 vb = (d5 * d2) - (d1 * d6);
        if (vb <= Fixed64.Zero && d2 >= Fixed64.Zero && d6 <= Fixed64.Zero)
        {
            Fixed64 w = d2 / (d2 - d6);
            return A + ac * w;
        }

        Fixed64 va = (d3 * d6) - (d5 * d4);
        if (va <= Fixed64.Zero && d4 - d3 >= Fixed64.Zero && d5 - d6 >= Fixed64.Zero)
        {
            Fixed64 w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return B + (C - B) * w;
        }

        Fixed64 denominator = va + vb + vc;
        if (denominator <= Fixed64.Zero)
            return ClosestPointOnEdges(point);

        Fixed64 vFace = vb / denominator;
        Fixed64 wFace = vc / denominator;
        return A + ab * vFace + ac * wFace;
    }

    /// <summary>
    /// Computes the squared distance from the supplied point to this triangle.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector3d point)
    {
        return Vector3d.DistanceSquared(point, ClosestPoint(point));
    }

    #endregion

    #region Deconstruction

    /// <summary>
    /// Deconstructs the triangle into ordered vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector3d a, out Vector3d b, out Vector3d c)
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
    public static bool operator ==(FixedTriangle left, FixedTriangle right) => left.Equals(right);

    /// <summary>
    /// Determines whether two triangles have different ordered vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedTriangle left, FixedTriangle right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedTriangle other)
    {
        return A == other.A && B == other.B && C == other.C;
    }

    /// <inheritdoc />
    public override bool Equals(object? obj)
    {
        return obj is FixedTriangle other && Equals(other);
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

    private Vector3d ClosestPointOnEdges(Vector3d point)
    {
        Vector3d best = GetEdge(0).ClosestPoint(point);
        Fixed64 bestDistance = Vector3d.DistanceSquared(point, best);
        TrySetCloserPoint(GetEdge(1), point, ref best, ref bestDistance);
        TrySetCloserPoint(GetEdge(2), point, ref best, ref bestDistance);
        return best;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TrySetCloserPoint(FixedSegment edge, Vector3d point, ref Vector3d best, ref Fixed64 bestDistance)
    {
        Vector3d candidate = edge.ClosestPoint(point);
        Fixed64 distance = Vector3d.DistanceSquared(point, candidate);
        if (distance >= bestDistance)
            return;

        bestDistance = distance;
        best = candidate;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ComponentMin(Vector3d a, Vector3d b)
    {
        return new Vector3d(FixedMath.Min(a.X, b.X), FixedMath.Min(a.Y, b.Y), FixedMath.Min(a.Z, b.Z));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ComponentMax(Vector3d a, Vector3d b)
    {
        return new Vector3d(FixedMath.Max(a.X, b.X), FixedMath.Max(a.Y, b.Y), FixedMath.Max(a.Z, b.Z));
    }

    #endregion
}
