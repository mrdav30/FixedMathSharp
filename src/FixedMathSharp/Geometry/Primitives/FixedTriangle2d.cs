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
    /// The arithmetic center of the three vertices, averaged independently per component.
    /// </summary>
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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector2d point) =>
        Vector2d.DistanceSquared(point, ClosestPoint(point));

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
    public bool Equals(FixedTriangle2d other) => A == other.A && B == other.B && C == other.C;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is FixedTriangle2d other && Equals(other);

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
