//=======================================================================
// FixedTriangle2d.cs
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
    /// <param name="a">The first vertex in the triangle's coordinate space.</param>
    /// <param name="b">The second vertex in the triangle's coordinate space.</param>
    /// <param name="c">The third vertex in the triangle's coordinate space.</param>
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
    /// <value>The oriented area in squared coordinate units. Counterclockwise winding is positive.</value>
    /// <remarks>
    /// The exact endpoint-difference cross product is halved and converted once
    /// with round-half-to-even and signed saturation.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 SignedArea
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Fixed64.RoundSignedToFixed(GetDoubledArea(), FixedMath.SHIFT_AMOUNT_I + 1);
    }

    /// <summary>
    /// The non-negative saturating magnitude of <see cref="SignedArea"/>.
    /// </summary>
    /// <value>The nonnegative area in squared coordinate units, saturated to the scalar range.</value>
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
    /// <value>The axis-aligned bounds of the stored vertices or endpoints.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedBoundArea Bounds
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedBoundArea.FromMinMax(ComponentMin(ComponentMin(A, B), C), ComponentMax(ComponentMax(A, B), C));
    }

    /// <summary>
    /// The arithmetic center of the three vertices, averaged independently per component.
    /// </summary>
    /// <value>The arithmetic mean of the three vertices in the triangle's coordinate space.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Centroid
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(
            FixedMath.Average(A.X, B.X, C.X),
            FixedMath.Average(A.Y, B.Y, C.Y));
    }

    /// <summary>
    /// Returns true when the exact area magnitude is less than or equal to
    /// <see cref="Fixed64.Epsilon"/>.
    /// </summary>
    /// <value>
    /// <see langword="true"/> when the area is at or below <see cref="Fixed64.Epsilon"/> and
    /// <see langword="false"/> otherwise.
    /// </value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public bool IsDegenerate
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => WideArithmetic.IsMagnitudeAtMost(
            GetDoubledArea(),
            (ulong)Fixed64.Epsilon.m_rawValue,
            FixedMath.SHIFT_AMOUNT_I + 1);
    }

    #endregion

    #region Geometry Access

    /// <summary>
    /// Gets a vertex by stable index: 0 = A, 1 = B, 2 = C.
    /// </summary>
    /// <param name="index">The zero-based index from 0 through 2.</param>
    /// <returns>The selected vertex in the triangle's coordinate space.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> is outside the supported range.</exception>
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
    /// <param name="index">The zero-based index from 0 through 2.</param>
    /// <returns>The selected directed edge in vertex order.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> is outside the supported range.</exception>
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
    /// <param name="weightB">The signed barycentric weight for vertex B on success, or zero on failure.</param>
    /// <param name="weightC">The signed barycentric weight for vertex C on success, or zero on failure.</param>
    /// <returns>The point with the supplied weights in the triangle's coordinate space. Extrapolation is allowed.</returns>
    /// <remarks>
    /// Each component uses the shared full-domain barycentric interpolation
    /// contract with one final round-half-to-even/saturating conversion.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d GetPoint(Fixed64 weightB, Fixed64 weightC) =>
        Vector2d.BarycentricCoordinates(A, B, C, weightB, weightC);

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Computes barycentric weights for a point relative to this triangle.
    /// </summary>
    /// <returns>
    /// True when the exact doubled-area magnitude is greater than
    /// <see cref="Fixed64.Epsilon"/>; otherwise false with three zero outputs.
    /// </returns>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="weightA">The signed barycentric weight for vertex A on success, or zero on failure.</param>
    /// <param name="weightB">The signed barycentric weight for vertex B on success, or zero on failure.</param>
    /// <param name="weightC">The signed barycentric weight for vertex C on success, or zero on failure.</param>
    /// <remarks>
    /// The three weights use direct exact area numerators and are rounded and
    /// saturated independently. Reversing winding does not change the result.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetBarycentricWeights(Vector2d point, out Fixed64 weightA, out Fixed64 weightB, out Fixed64 weightC)
    {
        Signed192 denominator = GetDoubledArea();
        if (WideArithmetic.IsMagnitudeAtMost(
            denominator,
            (ulong)Fixed64.Epsilon.m_rawValue,
            FixedMath.SHIFT_AMOUNT_I))
        {
            weightA = Fixed64.Zero;
            weightB = Fixed64.Zero;
            weightC = Fixed64.Zero;
            return false;
        }

        weightA = Fixed64.GetSignedRatio(GetCrossProduct(point, B, C), denominator);
        weightB = Fixed64.GetSignedRatio(GetCrossProduct(point, C, A), denominator);
        weightC = Fixed64.GetSignedRatio(GetCrossProduct(point, A, B), denominator);
        return true;
    }

    /// <summary>
    /// Determines whether the point is inside the triangle, including epsilon-wide edges and vertices.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <returns>
    /// <see langword="true"/> when the point is inside or on the triangle within its edge tolerance and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>
    /// Exact wide orientations make the winding-independent decision before
    /// scalar saturation. Collapsed line and point triangles retain edge-distance behavior.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector2d point)
    {
        Signed192 doubledArea = GetDoubledArea();
        if (WideArithmetic.IsMagnitudeAtMost(
            doubledArea,
            (ulong)Fixed64.Epsilon.m_rawValue,
            FixedMath.SHIFT_AMOUNT_I + 1))
            return IsPointOnAnyEdge(point);

        Signed192 ab = GetCrossProduct(A, B, point);
        Signed192 bc = GetCrossProduct(B, C, point);
        Signed192 ca = GetCrossProduct(C, A, point);
        ulong epsilonRaw = (ulong)Fixed64.Epsilon.m_rawValue;

        return doubledArea.Sign > 0
            ? IsNonnegativeWithinEpsilon(ab, epsilonRaw)
                && IsNonnegativeWithinEpsilon(bc, epsilonRaw)
                && IsNonnegativeWithinEpsilon(ca, epsilonRaw)
            : IsNonpositiveWithinEpsilon(ab, epsilonRaw)
                && IsNonpositiveWithinEpsilon(bc, epsilonRaw)
                && IsNonpositiveWithinEpsilon(ca, epsilonRaw);
    }

    /// <summary>
    /// Finds the closest point on or inside this triangle to the supplied point.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <returns>The closest point in the closed triangle in the same coordinate space.</returns>
    /// <remarks>
    /// Edge candidates are compared in AB, BC, CA order with exact squared
    /// distances. Exact ties retain the first candidate.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ClosestPoint(Vector2d point) =>
        Contains(point) ? point : ClosestPointOnEdges(point);

    /// <summary>
    /// Computes the squared distance from the supplied point to this triangle.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <returns>The squared distance in squared coordinate units, saturated if it exceeds the scalar range.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector2d point) =>
        Vector2d.DistanceSquared(point, ClosestPoint(point));

    #endregion

    #region Deconstruction

    /// <summary>
    /// Deconstructs the triangle into ordered vertices.
    /// </summary>
    /// <param name="a">The first vertex in the triangle's coordinate space.</param>
    /// <param name="b">The second vertex in the triangle's coordinate space.</param>
    /// <param name="c">The third vertex in the triangle's coordinate space.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector2d a, out Vector2d b, out Vector2d c)
    {
        a = A;
        b = B;
        c = C;
    }

    #endregion

    #region Operators

    /// <summary>Tests whether the stored geometry is equal.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when all three vertices match in order and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedTriangle2d left, FixedTriangle2d right) => left.Equals(right);

    /// <summary>Tests whether the stored geometry is different.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the stored values differ and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedTriangle2d left, FixedTriangle2d right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="other">The value to compare with this instance.</param>
    /// <returns><see langword="true"/> when all three vertices match in order and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedTriangle2d other) => A == other.A && B == other.B && C == other.C;

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="obj">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when <paramref name="obj"/> is a <see cref="FixedTriangle2d"/> and all three vertices
    /// match in order and <see langword="false"/> otherwise.
    /// </returns>
    public override bool Equals(object? obj) => obj is FixedTriangle2d other && Equals(other);

    /// <summary>Computes a hash code for the stored geometry.</summary>
    /// <returns>A hash code consistent with structural equality.</returns>
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
        TrySetCloserPoint(GetEdge(1), point, ref best);
        TrySetCloserPoint(GetEdge(2), point, ref best);
        return best;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TrySetCloserPoint(FixedSegment2d edge, Vector2d point, ref Vector2d best)
    {
        Vector2d candidate = edge.ClosestPoint(point);
        if (Vector2d.CompareDistanceSquared(point, candidate, point, best) >= 0)
            return;

        best = candidate;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Signed192 GetDoubledArea() => GetCrossProduct(A, B, C);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetCrossProduct(Vector2d origin, Vector2d first, Vector2d second) =>
        WideGeometry.GetDifferenceCrossProduct2D(
            first.X, origin.X, first.Y, origin.Y,
            second.X, origin.X, second.Y, origin.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsNonnegativeWithinEpsilon(Signed192 value, ulong epsilonRaw) =>
        value.Sign >= 0
        || WideArithmetic.IsMagnitudeAtMost(value, epsilonRaw, FixedMath.SHIFT_AMOUNT_I);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsNonpositiveWithinEpsilon(Signed192 value, ulong epsilonRaw) =>
        value.Sign <= 0
        || WideArithmetic.IsMagnitudeAtMost(value, epsilonRaw, FixedMath.SHIFT_AMOUNT_I);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ComponentMin(Vector2d a, Vector2d b) =>
        new(FixedMath.Min(a.X, b.X), FixedMath.Min(a.Y, b.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ComponentMax(Vector2d a, Vector2d b) =>
        new(FixedMath.Max(a.X, b.X), FixedMath.Max(a.Y, b.Y));

    #endregion
}
