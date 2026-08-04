

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Arithmetic operations for <see cref="Fixed64"/>, including overflow-safe
/// try-add/try-subtract helpers and related combined operations.
/// </content>
public partial struct Fixed64
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAddOrSubtractResultExact(long left, long result, long overflowMask) =>
        (overflowMask & (left ^ result)) >= 0;

    /// <summary>
    /// Attempts to add two values without saturation.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <param name="result">
    /// The exact sum when representable; otherwise, <see langword="default"/>.
    /// </param>
    /// <returns><see langword="true"/> when the exact sum is representable; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryAdd(Fixed64 left, Fixed64 right, out Fixed64 result)
    {
        long leftRaw = left.m_rawValue;
        long rightRaw = right.m_rawValue;
        long rawResult = unchecked(leftRaw + rightRaw);
        if (!IsAddOrSubtractResultExact(leftRaw, rawResult, ~(leftRaw ^ rightRaw)))
        {
            result = default;
            return false;
        }

        result = new Fixed64(rawResult);
        return true;
    }

    /// <summary>
    /// Attempts to subtract two values without saturation.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <param name="result">
    /// The exact difference when representable; otherwise, <see langword="default"/>.
    /// </param>
    /// <returns><see langword="true"/> when the exact difference is representable; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TrySubtract(Fixed64 left, Fixed64 right, out Fixed64 result)
    {
        long leftRaw = left.m_rawValue;
        long rightRaw = right.m_rawValue;
        long rawResult = unchecked(leftRaw - rightRaw);
        if (!IsAddOrSubtractResultExact(leftRaw, rawResult, leftRaw ^ rightRaw))
        {
            result = default;
            return false;
        }

        result = new Fixed64(rawResult);
        return true;
    }

    /// <summary>
    /// Attempts to add two values and subtract a third without intermediate saturation.
    /// </summary>
    /// <param name="firstAddend">The first addend.</param>
    /// <param name="secondAddend">The second addend.</param>
    /// <param name="subtrahend">The value to subtract from the exact sum.</param>
    /// <param name="result">
    /// The exact result when representable; otherwise, <see langword="default"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the exact result is representable; otherwise, <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryAddSubtract(
        Fixed64 firstAddend,
        Fixed64 secondAddend,
        Fixed64 subtrahend,
        out Fixed64 result)
    {
        long firstRaw = firstAddend.m_rawValue;
        long secondRaw = secondAddend.m_rawValue;
        ulong firstLow = unchecked((ulong)firstRaw);
        ulong secondLow = unchecked((ulong)secondRaw);
        ulong low = unchecked(firstLow + secondLow);
        long high = (firstRaw >> 63) + (secondRaw >> 63);
        if (low < firstLow)
            high++;

        long subtrahendRaw = subtrahend.m_rawValue;
        ulong subtrahendLow = unchecked((ulong)subtrahendRaw);
        ulong finalLow = unchecked(low - subtrahendLow);
        if (low < subtrahendLow)
            high--;
        high -= subtrahendRaw >> 63;

        long finalRaw = unchecked((long)finalLow);
        if (high != finalRaw >> 63)
        {
            result = default;
            return false;
        }

        result = new Fixed64(finalRaw);
        return true;
    }

    /// <summary>
    /// Attempts to compute <c>(firstLeft + firstRight) -
    /// (secondLeft + secondRight)</c> without intermediate saturation.
    /// </summary>
    /// <param name="firstLeft">The first value in the minuend sum.</param>
    /// <param name="firstRight">The second value in the minuend sum.</param>
    /// <param name="secondLeft">The first value in the subtrahend sum.</param>
    /// <param name="secondRight">The second value in the subtrahend sum.</param>
    /// <param name="result">
    /// The exact difference when representable; otherwise,
    /// <see langword="default"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the exact result is representable;
    /// otherwise, <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TrySubtractSums(
        Fixed64 firstLeft,
        Fixed64 firstRight,
        Fixed64 secondLeft,
        Fixed64 secondRight,
        out Fixed64 result)
    {
        Signed192 exact = WideArithmetic.SubtractSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Signed(firstLeft.m_rawValue),
                Signed192.Signed(firstRight.m_rawValue)),
            WideArithmetic.AddSigned192(
                Signed192.Signed(secondLeft.m_rawValue),
                Signed192.Signed(secondRight.m_rawValue)));
        long raw = unchecked((long)exact.Low);
        ulong extension = raw < 0L ? ulong.MaxValue : 0UL;
        if (exact.High != extension || exact.Middle != extension)
        {
            result = default;
            return false;
        }

        result = new Fixed64(raw);
        return true;
    }

    /// <summary>
    /// Returns the absolute value of a signed 64-bit integer as an unsigned 64-bit magnitude,
    /// safely handling long.MinValue.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static ulong AbsToUInt64(long value)
    {
        return value < 0
            ? unchecked((ulong)(~value + 1))
            : (ulong)value;
    }

    /// <summary>
    /// Computes the exact unsigned 128-bit product of two 64-bit unsigned integers.
    /// The result is returned as hi:lo, where product = (hi &lt;&lt; 64) | lo.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Multiply64To128(ulong a, ulong b, out ulong hi, out ulong lo)
    {
#if NET8_0_OR_GREATER
        hi = Math.BigMul(a, b, out lo);
#else
        ulong aLo = (uint)a;
        ulong aHi = a >> 32;
        ulong bLo = (uint)b;
        ulong bHi = b >> 32;

        ulong p0 = aLo * bLo;
        ulong p1 = aLo * bHi;
        ulong p2 = aHi * bLo;
        ulong p3 = aHi * bHi;

        ulong middle = (p0 >> 32) + (uint)p1 + (uint)p2;

        lo = (p0 & 0xFFFFFFFFUL) | (middle << 32);
        hi = p3 + (p1 >> 32) + (p2 >> 32) + (middle >> 32);
#endif
    }

    /// <summary>
    /// Determines whether the Euclidean magnitude of up to four fixed-point components
    /// fits in the positive <see cref="Fixed64"/> range.
    /// </summary>
    internal static bool IsMagnitudeRepresentable(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 w)
    {
        GetMagnitudeSquaredWords(x, y, z, w, out ulong overflow, out ulong sumHi, out ulong sumLo);

        // (long.MaxValue)^2 = 0x3FFF_FFFF_FFFF_FFFF_0000_0000_0000_0001.
        return overflow == 0UL
            && (sumHi < 0x3FFF_FFFF_FFFF_FFFFUL
                || (sumHi == 0x3FFF_FFFF_FFFF_FFFFUL && sumLo <= 1UL));
    }

    /// <summary>
    /// Returns the exactly rounded Euclidean magnitude of up to four Q32.32
    /// components, saturating only when the rounded positive result does not fit.
    /// </summary>
    internal static Fixed64 GetRoundedMagnitude(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 w)
    {
        GetMagnitudeSquaredWords(x, y, z, w, out ulong overflow, out ulong sumHi, out ulong sumLo);
        if (overflow != 0UL)
            return MaxValue;
        if ((sumHi | sumLo) == 0UL)
            return Zero;

        ulong root = GetFloorSquareRoot(sumHi, sumLo, out _, out ulong remainderLo);
        if (root > long.MaxValue)
            return MaxValue;

        if (remainderLo > root)
        {
            if (root == long.MaxValue)
                return MaxValue;

            root++;
        }

        return FromRaw((long)root);
    }

    /// <summary>
    /// Compares exact squared magnitudes without projecting their sums back into Q32.32.
    /// </summary>
    internal static int CompareMagnitudeSquared(
        Fixed64 leftX,
        Fixed64 leftY,
        Fixed64 leftZ,
        Fixed64 leftW,
        Fixed64 rightX,
        Fixed64 rightY,
        Fixed64 rightZ,
        Fixed64 rightW)
    {
        GetMagnitudeSquaredWords(leftX, leftY, leftZ, leftW, out ulong leftOverflow, out ulong leftHi, out ulong leftLo);
        GetMagnitudeSquaredWords(rightX, rightY, rightZ, rightW, out ulong rightOverflow, out ulong rightHi, out ulong rightLo);

        if (leftOverflow != rightOverflow)
            return leftOverflow < rightOverflow ? -1 : 1;
        if (leftHi != rightHi)
            return leftHi < rightHi ? -1 : 1;
        if (leftLo != rightLo)
            return leftLo < rightLo ? -1 : 1;
        return 0;
    }

    private static void GetMagnitudeSquaredWords(
        Fixed64 x,
        Fixed64 y,
        Fixed64 z,
        Fixed64 w,
        out ulong overflow,
        out ulong sumHi,
        out ulong sumLo)
    {
        overflow = 0UL;
        sumHi = 0UL;
        sumLo = 0UL;
        AddMagnitudeSquare(x.m_rawValue, ref overflow, ref sumHi, ref sumLo);
        AddMagnitudeSquare(y.m_rawValue, ref overflow, ref sumHi, ref sumLo);
        AddMagnitudeSquare(z.m_rawValue, ref overflow, ref sumHi, ref sumLo);
        AddMagnitudeSquare(w.m_rawValue, ref overflow, ref sumHi, ref sumLo);
    }

    private static ulong GetFloorSquareRoot(
        ulong valueHi,
        ulong valueLo,
        out ulong remainderHi,
        out ulong remainderLo)
    {
        int highestBit = valueHi != 0UL
            ? 127 - CountLeadingZeroes(valueHi)
            : 63 - CountLeadingZeroes(valueLo);
        int pairIndex = highestBit >> 1;
        ulong root = 0UL;
        remainderHi = 0UL;
        remainderLo = 0UL;

        for (; pairIndex >= 0; pairIndex--)
        {
            ulong pair = pairIndex >= 32
                ? (valueHi >> ((pairIndex - 32) << 1)) & 3UL
                : (valueLo >> (pairIndex << 1)) & 3UL;

            remainderHi = (remainderHi << 2) | (remainderLo >> 62);
            remainderLo = (remainderLo << 2) | pair;
            root <<= 1;

            ulong candidateHi = root >> 63;
            ulong candidateLo = (root << 1) | 1UL;
            if (remainderHi < candidateHi
                || (remainderHi == candidateHi && remainderLo < candidateLo))
            {
                continue;
            }

            ulong previousRemainderLo = remainderLo;
            remainderLo -= candidateLo;
            remainderHi -= candidateHi + (previousRemainderLo < candidateLo ? 1UL : 0UL);
            root++;
        }

        return root;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddMagnitudeSquare(
        long component,
        ref ulong overflow,
        ref ulong sumHi,
        ref ulong sumLo)
    {
        ulong magnitude = AbsToUInt64(component);
        Multiply64To128(magnitude, magnitude, out ulong squareHi, out ulong squareLo);

        ulong previousLo = sumLo;
        sumLo += squareLo;
        ulong addHi = squareHi + (sumLo < previousLo ? 1UL : 0UL);
        ulong previousHi = sumHi;
        sumHi += addHi;
        if (sumHi < previousHi)
            overflow++;
    }

    /// <summary>
    /// Shifts the unsigned 128-bit value (hi:lo) right by <paramref name="shift"/> bits,
    /// applying round-half-to-even to the discarded bits.
    /// </summary>
    /// <param name="hi">Upper 64 bits of the 128-bit value.</param>
    /// <param name="lo">Lower 64 bits of the 128-bit value.</param>
    /// <param name="shift">Number of bits to shift right. Must be in the range 1..63.</param>
    /// <param name="overflowed">
    /// True if the shifted or rounded result exceeded 64 bits.
    /// </param>
    /// <returns>
    /// The rounded 64-bit result of ((hi:lo) >> shift).
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong ShiftRightRoundedToEven(ulong hi, ulong lo, int shift, out bool overflowed)
    {
        // Preconditions: 1 <= shift <= 63

        // Bits above the low 64 result bits must be preserved as saturation state
        // before the lower projection can wrap back into range.
        overflowed = (hi >> shift) != 0UL;

        // Integer part after shifting right by 'shift':
        // result = ((hi << (64 - shift)) | (lo >> shift))
        ulong result = (hi << (64 - shift)) | (lo >> shift);

        // Discarded remainder bits are the low 'shift' bits of lo.
        ulong remainderMask = (1UL << shift) - 1UL;
        ulong remainder = lo & remainderMask;

        // Halfway value among the discarded bits.
        ulong half = 1UL << (shift - 1);

        // Round-half-to-even:
        // - round up if remainder > half
        // - if exactly half, round so final result is even
        bool shouldRoundUp =
            remainder > half ||
            (remainder == half && (result & 1UL) != 0);

        if (shouldRoundUp)
        {
            ulong incremented = result + 1UL;
            overflowed |= incremented < result;
            result = incremented;
        }

        return result;
    }

    /// <summary>
    /// Divides unsigned raw magnitudes and applies the requested result sign.
    /// </summary>
    /// <remarks>
    /// The divisor magnitude must be in the range 1 through 2^63, inclusive, matching the
    /// unsigned magnitude of any signed raw value. This bound keeps remainder doubling within
    /// <see cref="ulong"/>. The quotient is calculated with one guard bit and rounded to the
    /// nearest even raw value before final saturation.
    /// </remarks>
    internal static Fixed64 DivideMagnitude(
        ulong dividendMagnitude,
        ulong divisorMagnitude,
        bool negative)
    {
        ulong remainder = dividendMagnitude;
        ulong divider = divisorMagnitude;
        ulong quotient = 0UL;
        int bitPos = FixedMath.SHIFT_AMOUNT_I + 1;

        // If the divider is divisible by 2^n, take advantage of it.
        while ((divider & 0xF) == 0 && bitPos >= 4)
        {
            divider >>= 4;
            bitPos -= 4;
        }

        while (remainder != 0 && bitPos >= 0)
        {
            int shift = CountLeadingZeroes(remainder);
            if (shift > bitPos)
                shift = bitPos;

            remainder <<= shift;
            bitPos -= shift;

            ulong div = remainder / divider;
            remainder %= divider;
            quotient += div << bitPos;

            if ((div & ~(ulong.MaxValue >> bitPos)) != 0)
                return negative ? MinValue : MaxValue;

            remainder <<= 1;
            --bitPos;
        }

        ulong magnitude = RoundGuardedQuotientToEven(
            quotient,
            remainder != 0UL,
            out bool roundedOverflow);

        if (roundedOverflow)
            return negative ? MinValue : MaxValue;

        long result = (long)magnitude;
        return new Fixed64(negative ? -result : result);
    }

    /// <summary>
    /// Removes the low guard bit and rounds the retained quotient to even.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static ulong RoundGuardedQuotientToEven(
        ulong guardedQuotient,
        bool hasTrailingRemainder,
        out bool overflowed)
    {
        bool shouldRoundUp = (guardedQuotient & 1UL) != 0UL
            && (hasTrailingRemainder || (guardedQuotient & 2UL) != 0UL);

        if (!shouldRoundUp)
        {
            overflowed = false;
            return guardedQuotient >> 1;
        }

        ulong roundedGuardedQuotient = guardedQuotient + 1UL;
        overflowed = roundedGuardedQuotient == 0UL;
        return overflowed ? 1UL << 63 : roundedGuardedQuotient >> 1;
    }
}
