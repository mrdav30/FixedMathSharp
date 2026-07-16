//=======================================================================
// WideGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Owns exact coordinate products and fixed-point geometry policy.
/// </summary>
internal static class WideGeometry
{
    /// <summary>
    /// Returns the exact dot product of two two-dimensional endpoint differences.
    /// </summary>
    internal static Signed192 GetDifferenceDotProduct2D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY)
    {
        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            leftEndX.m_rawValue,
            leftStartX.m_rawValue,
            rightEndX.m_rawValue,
            rightStartX.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndY.m_rawValue,
            leftStartY.m_rawValue,
            rightEndY.m_rawValue,
            rightStartY.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Returns the exact dot product of two three-dimensional endpoint differences.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 GetDifferenceDotProduct3D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 leftEndZ,
        Fixed64 leftStartZ,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY,
        Fixed64 rightEndZ,
        Fixed64 rightStartZ)
    {
        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            leftEndX.m_rawValue,
            leftStartX.m_rawValue,
            rightEndX.m_rawValue,
            rightStartX.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndY.m_rawValue,
            leftStartY.m_rawValue,
            rightEndY.m_rawValue,
            rightStartY.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndZ.m_rawValue,
            leftStartZ.m_rawValue,
            rightEndZ.m_rawValue,
            rightStartZ.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Returns whether an exact squared direction rounds to zero in Q32.32.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsSquaredLengthDegenerate(Signed192 value) =>
        value.High == 0UL
        && value.Middle == 0UL
        && value.Low <= (1UL << (FixedMath.SHIFT_AMOUNT_I - 1));

    /// <summary>
    /// Returns the exact 2D cross product of two endpoint differences.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 GetDifferenceCrossProduct2D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY)
    {
        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            leftEndX.m_rawValue,
            leftStartX.m_rawValue,
            rightEndY.m_rawValue,
            rightStartY.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndY.m_rawValue,
            leftStartY.m_rawValue,
            rightStartX.m_rawValue,
            rightEndX.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Returns the exact sign of the scalar triple product of three raw
    /// three-component vectors.
    /// </summary>
    internal static int GetTripleProductSign(
        Fixed64 firstX,
        Fixed64 firstY,
        Fixed64 firstZ,
        Fixed64 secondX,
        Fixed64 secondY,
        Fixed64 secondZ,
        Fixed64 thirdX,
        Fixed64 thirdY,
        Fixed64 thirdZ)
    {
        Signed192 minorX = GetDifferenceCrossProduct2D(
            secondY, Fixed64.Zero, secondZ, Fixed64.Zero,
            thirdY, Fixed64.Zero, thirdZ, Fixed64.Zero);
        Signed192 minorY = GetDifferenceCrossProduct2D(
            secondZ, Fixed64.Zero, secondX, Fixed64.Zero,
            thirdZ, Fixed64.Zero, thirdX, Fixed64.Zero);
        Signed192 minorZ = GetDifferenceCrossProduct2D(
            secondX, Fixed64.Zero, secondY, Fixed64.Zero,
            thirdX, Fixed64.Zero, thirdY, Fixed64.Zero);
        Signed320 tripleProduct = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(WideArithmetic.FromSignedRaw(firstX.m_rawValue), minorX),
                WideArithmetic.MultiplySigned192(WideArithmetic.FromSignedRaw(firstY.m_rawValue), minorY)),
            WideArithmetic.MultiplySigned192(WideArithmetic.FromSignedRaw(firstZ.m_rawValue), minorZ));
        return tripleProduct.Sign;
    }

    /// <summary>
    /// Applies the public 3D segment near-parallel threshold to an exact Q128.128 determinant.
    /// </summary>
    internal static bool IsSegmentDeterminantNearParallel(Signed320 determinant)
    {
        WideArithmetic.GetMagnitude(
            determinant,
            out ulong word4,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        ulong epsilonRaw = (ulong)Fixed64.Epsilon.m_rawValue;
        return WideArithmetic.CompareUnsigned(
            word4,
            word3,
            word2,
            word1,
            word0,
            0UL,
            0UL,
            epsilonRaw >> 32,
            epsilonRaw << 32,
            0UL) < 0;
    }

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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void AccumulateDifferenceProduct(
        long candidate,
        long current,
        long directionEnd,
        long directionStart,
        ref ulong sumHigh,
        ref ulong sumMiddle,
        ref ulong sumLow)
    {
        if (candidate == current || directionEnd == directionStart)
            return;

        bool negativeDifference = candidate < current;
        ulong differenceMagnitude = negativeDifference
            ? unchecked((ulong)current - (ulong)candidate)
            : unchecked((ulong)candidate - (ulong)current);
        bool negativeDirection = directionEnd < directionStart;
        ulong directionMagnitude = negativeDirection
            ? unchecked((ulong)directionStart - (ulong)directionEnd)
            : unchecked((ulong)directionEnd - (ulong)directionStart);
        bool negativeProduct = negativeDifference != negativeDirection;

        Fixed64.Multiply64To128(
            differenceMagnitude,
            directionMagnitude,
            out ulong productMiddle,
            out ulong productLow);

        ulong productHigh = 0UL;
        if (negativeProduct)
        {
            productLow = unchecked(~productLow + 1UL);
            productMiddle = unchecked(~productMiddle + (productLow == 0UL ? 1UL : 0UL));
            productHigh = ulong.MaxValue;
        }

        ulong previousLow = sumLow;
        sumLow = unchecked(sumLow + productLow);
        ulong carry = sumLow < previousLow ? 1UL : 0UL;

        ulong addMiddle = unchecked(productMiddle + carry);
        ulong carryHigh = addMiddle < productMiddle ? 1UL : 0UL;
        ulong previousMiddle = sumMiddle;
        sumMiddle = unchecked(sumMiddle + addMiddle);
        if (sumMiddle < previousMiddle)
            carryHigh = 1UL;

        sumHigh = unchecked(sumHigh + productHigh + carryHigh);
    }
}
