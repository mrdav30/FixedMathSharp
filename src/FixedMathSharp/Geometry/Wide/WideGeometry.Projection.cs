//=======================================================================
// WideGeometry.Projection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <content>
/// Wide-precision projection helpers: exact difference projections, dot-product based
/// projection parameters, and plane projections computed without narrowing intermediate results.
/// </content>
internal static partial class WideGeometry
{
    /// <summary>
    /// Compares the exact projection of three component differences.
    /// </summary>
    internal static int CompareDifferenceProjection(
        Fixed64 candidateX,
        Fixed64 currentX,
        Fixed64 directionX,
        Fixed64 candidateY,
        Fixed64 currentY,
        Fixed64 directionY,
        Fixed64 candidateZ,
        Fixed64 currentZ,
        Fixed64 directionZ)
    {
        GetDifferenceProjectionWords(
            candidateX,
            currentX,
            directionX,
            candidateY,
            currentY,
            directionY,
            candidateZ,
            currentZ,
            directionZ,
            out ulong sumHigh,
            out ulong sumMiddle,
            out ulong sumLow);

        if ((sumHigh & (1UL << 63)) != 0UL)
            return -1;

        return (sumHigh | sumMiddle | sumLow) == 0UL ? 0 : 1;
    }

    /// <summary>
    /// Returns the exact projection words of three component differences.
    /// </summary>
    internal static void GetDifferenceProjectionWords(
        Fixed64 candidateX,
        Fixed64 currentX,
        Fixed64 directionX,
        Fixed64 candidateY,
        Fixed64 currentY,
        Fixed64 directionY,
        Fixed64 candidateZ,
        Fixed64 currentZ,
        Fixed64 directionZ,
        out ulong sumHigh,
        out ulong sumMiddle,
        out ulong sumLow)
    {
        sumHigh = 0UL;
        sumMiddle = 0UL;
        sumLow = 0UL;
        AccumulateDifferenceProduct(
            candidateX.m_rawValue,
            currentX.m_rawValue,
            directionX.m_rawValue,
            0L,
            ref sumHigh,
            ref sumMiddle,
            ref sumLow);
        AccumulateDifferenceProduct(
            candidateY.m_rawValue,
            currentY.m_rawValue,
            directionY.m_rawValue,
            0L,
            ref sumHigh,
            ref sumMiddle,
            ref sumLow);
        AccumulateDifferenceProduct(
            candidateZ.m_rawValue,
            currentZ.m_rawValue,
            directionZ.m_rawValue,
            0L,
            ref sumHigh,
            ref sumMiddle,
            ref sumLow);
    }

    /// <summary>
    /// Returns max(0, dot(target - source, direction) / dot(direction, direction))
    /// without narrowing either dot product first.
    /// </summary>
    internal static Fixed64 GetNonNegativeDifferenceProjectionParameter(
        Vector3d target,
        Vector3d source,
        Vector3d direction)
    {
        Signed192 numerator = GetDifferenceDotProduct3D(
            target.X, source.X,
            target.Y, source.Y,
            target.Z, source.Z,
            direction.X, Fixed64.Zero,
            direction.Y, Fixed64.Zero,
            direction.Z, Fixed64.Zero);
        if (numerator.Sign <= 0)
            return Fixed64.Zero;

        Signed192 denominator = GetDifferenceDotProduct3D(
            direction.X, Fixed64.Zero,
            direction.Y, Fixed64.Zero,
            direction.Z, Fixed64.Zero,
            direction.X, Fixed64.Zero,
            direction.Y, Fixed64.Zero,
            direction.Z, Fixed64.Zero);
        return Fixed64.GetSignedRatio(numerator, denominator);
    }

    /// <summary>
    /// Returns normalize(q * vector - normal * dot(vector, normal)), where
    /// q is the exact squared length of the normal.
    /// </summary>
    internal static Vector3d GetNormalizedProjectionOnPlane(
        Vector3d vector,
        Vector3d normal)
    {
        Signed192 q = GetDifferenceDotProduct3D(
            normal.X, Fixed64.Zero,
            normal.Y, Fixed64.Zero,
            normal.Z, Fixed64.Zero,
            normal.X, Fixed64.Zero,
            normal.Y, Fixed64.Zero,
            normal.Z, Fixed64.Zero);
        if (q.Sign == 0)
            return GetNormalized(vector);

        Signed192 projection = GetDifferenceDotProduct3D(
            vector.X, Fixed64.Zero,
            vector.Y, Fixed64.Zero,
            vector.Z, Fixed64.Zero,
            normal.X, Fixed64.Zero,
            normal.Y, Fixed64.Zero,
            normal.Z, Fixed64.Zero);
        Signed192 x = Signed192.Signed(vector.X.m_rawValue);
        Signed192 y = Signed192.Signed(vector.Y.m_rawValue);
        Signed192 z = Signed192.Signed(vector.Z.m_rawValue);
        Signed192 nx = Signed192.Signed(normal.X.m_rawValue);
        Signed192 ny = Signed192.Signed(normal.Y.m_rawValue);
        Signed192 nz = Signed192.Signed(normal.Z.m_rawValue);
        return GetNormalized(
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(q, x),
                WideArithmetic.MultiplySigned192(nx, projection)),
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(q, y),
                WideArithmetic.MultiplySigned192(ny, projection)),
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(q, z),
                WideArithmetic.MultiplySigned192(nz, projection)));
    }
}
