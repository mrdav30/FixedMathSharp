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

namespace FixedMathSharp.Geometry;

/// <summary>
/// Represents an ordered triangle in three-dimensional fixed-point space.
/// </summary>
/// <remarks>
/// Triangle queries preserve complete raw-coordinate differences and exact wide
/// predicates until their final public <see cref="Fixed64"/> conversion.
/// </remarks>
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
    /// <remarks>
    /// Each exact cross component is rounded once, half to even, and saturates
    /// independently at the public Q32.32 boundary.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d UnnormalizedNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            GetExactNormalComponents(out Signed192 x, out Signed192 y, out Signed192 z);
            return new Vector3d(
                Fixed64.RoundSignedToFixed(x, FixedMath.SHIFT_AMOUNT_I),
                Fixed64.RoundSignedToFixed(y, FixedMath.SHIFT_AMOUNT_I),
                Fixed64.RoundSignedToFixed(z, FixedMath.SHIFT_AMOUNT_I));
        }
    }

    /// <summary>
    /// The normalized triangle normal derived from the exact cross product.
    /// </summary>
    /// <remarks>
    /// Components are rounded half to even from the exact squared magnitude.
    /// Triangles at or below the inclusive degeneracy threshold return
    /// <see cref="Vector3d.Zero"/>.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Normal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            GetExactNormalComponents(out Signed192 x, out Signed192 y, out Signed192 z);
            Signed320 squaredMagnitude = WideGeometry.GetSquaredMagnitude(
                x,
                y,
                z,
                out Signed320 xSquare,
                out Signed320 ySquare,
                out Signed320 zSquare);
            if (WideGeometry.IsQ128MagnitudeAtMostEpsilon(squaredMagnitude))
                return Vector3d.Zero;

            Signed192 magnitude = WideArithmetic.GetFloorSquareRoot(squaredMagnitude, out Signed192 remainder);
            Signed192 ceilingMagnitude = remainder.IsZero
                ? magnitude
                : WideArithmetic.AddSigned192(magnitude, Signed192.Signed(1L));
            return new Vector3d(
                Fixed64.NormalizeWideComponent(x, xSquare, ceilingMagnitude, squaredMagnitude),
                Fixed64.NormalizeWideComponent(y, ySquare, ceilingMagnitude, squaredMagnitude),
                Fixed64.NormalizeWideComponent(z, zSquare, ceilingMagnitude, squaredMagnitude));
        }
    }

    /// <summary>
    /// The non-negative surface area of the triangle.
    /// </summary>
    /// <remarks>
    /// The exact cross-product magnitude is halved and rounded once, half to
    /// even. Results beyond the positive Q32.32 range saturate to
    /// <see cref="Fixed64.MaxValue"/>.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Area
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            GetExactNormal(out _, out _, out _, out Signed320 squaredMagnitude);
            Signed192 root = WideArithmetic.GetFloorSquareRoot(squaredMagnitude, out Signed192 remainder);
            return Fixed64.RoundSquareRootToFixed(root, remainder, FixedMath.SHIFT_AMOUNT_I + 1);
        }
    }

    /// <summary>
    /// Gets the non-negative semantic surface-area weight with 32 fractional
    /// guard bits beyond the scalar area view.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedMassWeight AreaWeight
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            GetExactNormal(
                out _,
                out _,
                out _,
                out Signed320 squaredMagnitude);
            return WideMassProperties.CreateTriangleAreaWeight(
                squaredMagnitude);
        }
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
    /// The arithmetic center of the three vertices, rounded half to even per component.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Centroid
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(
            FixedMath.Average(A.X, B.X, C.X),
            FixedMath.Average(A.Y, B.Y, C.Y),
            FixedMath.Average(A.Z, B.Z, C.Z));
    }

    /// <summary>
    /// Attempts to calculate wide uniform thin-shell mass properties for an
    /// indexed triangle surface.
    /// </summary>
    /// <remarks>
    /// Semantic triangle-area weights retain 32 fractional guard bits. First
    /// and second moments remain wide until the final center and unit-mass
    /// tensor are rounded to Q32.32.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="triangleIndices"/> does not contain complete index
    /// triplets.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// An element of <paramref name="triangleIndices"/> is outside
    /// <paramref name="vertices"/>.
    /// </exception>
    public static bool TryGetUniformShellMassProperties(
        ReadOnlySpan<Vector3d> vertices,
        ReadOnlySpan<int> triangleIndices,
        out FixedMassWeight surfaceWeight,
        out Vector3d centerOfMass,
        out Fixed3x3 unitMassInertiaTensor) =>
        WideTriangleMassProperties.TryCreateUniformShell(
            vertices,
            triangleIndices,
            out surfaceWeight,
            out centerOfMass,
            out unitMassInertiaTensor);

    /// <summary>
    /// Returns true when the exact squared normal magnitude is at or below
    /// the inclusive <see cref="Fixed64.Epsilon"/> threshold.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public bool IsDegenerate
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            GetExactNormal(out _, out _, out _, out Signed320 squaredMagnitude);
            return WideGeometry.IsQ128MagnitudeAtMostEpsilon(squaredMagnitude);
        }
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
    /// Gets the point represented by barycentric weights for vertices B and C
    /// without saturating intermediate differences.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetPoint(Fixed64 weightB, Fixed64 weightC) =>
        Vector3d.BarycentricCoordinates(A, B, C, weightB, weightC);

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Computes barycentric weights for the point projected onto this triangle's plane.
    /// </summary>
    /// <returns>
    /// True when the exact Gram denominator is outside the inclusive
    /// <see cref="Fixed64.Epsilon"/> failure threshold; otherwise false with
    /// all three weights set to zero.
    /// </returns>
    /// <remarks>
    /// Successful weights are computed from independent exact numerators, then
    /// rounded half to even and saturated independently at the public boundary.
    /// The point is projected onto the triangle plane; it need not lie on it.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetProjectedBarycentricWeights(Vector3d point, out Fixed64 weightA, out Fixed64 weightB, out Fixed64 weightC)
    {
        Signed192 abAb = GetDifferenceDot(B, A, B, A);
        Signed192 abAc = GetDifferenceDot(B, A, C, A);
        Signed192 acAc = GetDifferenceDot(C, A, C, A);
        Signed192 apAb = GetDifferenceDot(point, A, B, A);
        Signed192 apAc = GetDifferenceDot(point, A, C, A);
        Signed320 denominator = WideArithmetic.MultiplySubtract(abAb, acAc, abAc, abAc);

        if (WideGeometry.IsQ128MagnitudeAtMostEpsilon(denominator))
        {
            weightA = Fixed64.Zero;
            weightB = Fixed64.Zero;
            weightC = Fixed64.Zero;
            return false;
        }

        Signed320 numeratorB = WideArithmetic.MultiplySubtract(acAc, apAb, abAc, apAc);
        Signed320 numeratorC = WideArithmetic.MultiplySubtract(abAb, apAc, abAc, apAb);
        Signed320 numeratorA = WideArithmetic.SubtractSigned320(
            WideArithmetic.SubtractSigned320(denominator, numeratorB),
            numeratorC);
        weightA = Fixed64.GetSignedRatio(numeratorA, denominator);
        weightB = Fixed64.GetSignedRatio(numeratorB, denominator);
        weightC = Fixed64.GetSignedRatio(numeratorC, denominator);
        return true;
    }

    /// <summary>
    /// Gets the closest point on this triangle to a point carried by another
    /// rigid frame without materializing either absolute world point.
    /// </summary>
    /// <remarks>
    /// The returned anchor remains in the triangle's rigid frame. Voronoi
    /// predicates retain the complete relative-frame displacement until the
    /// final local barycentric conversion.
    /// </remarks>
    public FixedPointAnchor GetClosestPointAnchor(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        in FixedPointAnchor point)
    {
        if (!triangleRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Triangle rotation must be normalized.",
                nameof(triangleRotation));
        }
        if (!point.Rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Point rotation must be normalized.",
                nameof(point));
        }

        return WideOrientedBox.GetClosestPointOnTriangle(
            this,
            triangleOrigin,
            triangleRotation,
            point);
    }

    /// <summary>
    /// Determines whether the supplied point projects within or onto this
    /// triangle without requiring the point to lie on its plane.
    /// </summary>
    /// <remarks>
    /// The projected barycentric signs are classified from exact wide
    /// numerators. Degenerate triangles have no projected face and return false.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ContainsProjection(Vector3d point)
    {
        Signed192 abAb = GetDifferenceDot(B, A, B, A);
        Signed192 abAc = GetDifferenceDot(B, A, C, A);
        Signed192 acAc = GetDifferenceDot(C, A, C, A);
        Signed320 denominator = WideArithmetic.MultiplySubtract(abAb, acAc, abAc, abAc);
        if (WideGeometry.IsQ128MagnitudeAtMostEpsilon(denominator))
            return false;

        Signed192 apAb = GetDifferenceDot(point, A, B, A);
        Signed192 apAc = GetDifferenceDot(point, A, C, A);
        Signed320 numeratorB = WideArithmetic.MultiplySubtract(acAc, apAb, abAc, apAc);
        if (numeratorB.Sign < 0)
            return false;

        Signed320 numeratorC = WideArithmetic.MultiplySubtract(abAb, apAc, abAc, apAb);
        if (numeratorC.Sign < 0)
            return false;

        return WideArithmetic.SubtractSigned320(
            WideArithmetic.SubtractSigned320(denominator, numeratorB),
            numeratorC).Sign >= 0;
    }

    /// <summary>
    /// Determines whether the point is within the inclusive squared-distance
    /// epsilon of this triangle.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector3d point)
    {
        return DistanceSquared(point) <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Finds the closest point on or inside this triangle to the supplied point.
    /// </summary>
    /// <remarks>
    /// Voronoi-region predicates and degenerate edge distances use exact wide
    /// intermediates. Degenerate candidates are visited in AB, BC, CA order,
    /// and an exact distance tie retains the first candidate.
    /// </remarks>
    public Vector3d ClosestPoint(Vector3d point)
    {
        Signed192 abAb = GetDifferenceDot(B, A, B, A);
        Signed192 abAc = GetDifferenceDot(B, A, C, A);
        Signed192 acAc = GetDifferenceDot(C, A, C, A);
        Signed320 denominator = WideArithmetic.MultiplySubtract(abAb, acAc, abAc, abAc);
        if (WideGeometry.IsQ128MagnitudeAtMostEpsilon(denominator))
            return ClosestPointOnEdges(point);

        Signed192 d1 = GetDifferenceDot(B, A, point, A);
        Signed192 d2 = GetDifferenceDot(C, A, point, A);
        if (d1.Sign <= 0 && d2.Sign <= 0)
            return A;

        Signed192 d3 = WideArithmetic.SubtractSigned192(d1, abAb);
        Signed192 d4 = WideArithmetic.SubtractSigned192(d2, abAc);
        if (d3.Sign >= 0 && WideArithmetic.SubtractSigned192(d4, d3).Sign <= 0)
            return B;

        Signed320 vc = WideArithmetic.MultiplySubtract(d1, d4, d3, d2);
        if (vc.Sign <= 0 && d1.Sign >= 0 && d3.Sign <= 0)
        {
            _ = Fixed64.TryGetUnitIntervalRatio(
                d1,
                WideArithmetic.SubtractSigned192(d1, d3),
                out Fixed64 parameter);
            return Vector3d.Lerp(A, B, parameter);
        }

        Signed192 d5 = WideArithmetic.SubtractSigned192(d1, abAc);
        Signed192 d6 = WideArithmetic.SubtractSigned192(d2, acAc);
        if (d6.Sign >= 0 && WideArithmetic.SubtractSigned192(d5, d6).Sign <= 0)
            return C;

        Signed320 vb = WideArithmetic.MultiplySubtract(d5, d2, d1, d6);
        if (vb.Sign <= 0 && d2.Sign >= 0 && d6.Sign <= 0)
        {
            _ = Fixed64.TryGetUnitIntervalRatio(
                d2,
                WideArithmetic.SubtractSigned192(d2, d6),
                out Fixed64 parameter);
            return Vector3d.Lerp(A, C, parameter);
        }

        Signed320 va = WideArithmetic.MultiplySubtract(d3, d6, d5, d4);
        Signed192 d4MinusD3 = WideArithmetic.SubtractSigned192(d4, d3);
        Signed192 d5MinusD6 = WideArithmetic.SubtractSigned192(d5, d6);
        if (va.Sign <= 0 && d4MinusD3.Sign >= 0 && d5MinusD6.Sign >= 0)
        {
            _ = Fixed64.TryGetUnitIntervalRatio(
                d4MinusD3,
                WideArithmetic.AddSigned192(d4MinusD3, d5MinusD6),
                out Fixed64 parameter);
            return Vector3d.Lerp(B, C, parameter);
        }

        _ = Fixed64.TryGetUnitIntervalRatio(vb, denominator, out Fixed64 weightB);
        _ = Fixed64.TryGetUnitIntervalRatio(vc, denominator, out Fixed64 weightC);
        return GetPoint(weightB, weightC);
    }

    /// <summary>
    /// Computes the squared distance from the supplied point to this triangle,
    /// rounded once and positively saturated at the public boundary.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector3d point)
    {
        Vector3d closest = ClosestPoint(point);
        Signed192 squaredDistance = GetDifferenceDot(point, closest, point, closest);
        return Fixed64.RoundSquaredDistance(squaredDistance);
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
        TrySetCloserPoint(GetEdge(1), point, ref best);
        TrySetCloserPoint(GetEdge(2), point, ref best);
        return best;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TrySetCloserPoint(FixedSegment edge, Vector3d point, ref Vector3d best)
    {
        Vector3d candidate = edge.ClosestPoint(point);
        if (WideGeometry.CompareSquaredDistance3D(
            point.X, candidate.X, point.Y, candidate.Y, point.Z, candidate.Z,
            point.X, best.X, point.Y, best.Y, point.Z, best.Z) >= 0)
            return;

        best = candidate;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void GetExactNormal(
        out Signed192 x,
        out Signed192 y,
        out Signed192 z,
        out Signed320 squaredMagnitude)
    {
        GetExactNormalComponents(out x, out y, out z);
        squaredMagnitude = WideGeometry.GetSquaredMagnitude(x, y, z, out _, out _, out _);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void GetExactNormalComponents(
        out Signed192 x,
        out Signed192 y,
        out Signed192 z)
    {
        WideGeometry.GetDifferenceCrossProduct3D(
            B.X, A.X, B.Y, A.Y, B.Z, A.Z,
            C.X, A.X, C.Y, A.Y, C.Z, A.Z,
            out x,
            out y,
            out z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetDifferenceDot(
        Vector3d leftEnd,
        Vector3d leftStart,
        Vector3d rightEnd,
        Vector3d rightStart) =>
        WideGeometry.GetDifferenceDotProduct3D(
            leftEnd.X, leftStart.X, leftEnd.Y, leftStart.Y, leftEnd.Z, leftStart.Z,
            rightEnd.X, rightStart.X, rightEnd.Y, rightStart.Y, rightEnd.Z, rightStart.Z);

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
