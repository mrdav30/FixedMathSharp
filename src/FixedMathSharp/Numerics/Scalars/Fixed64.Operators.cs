//=======================================================================
// Fixed64.Operators.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed64
{
    #region Arithmetic Operators

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAddOrSubtractResultExact(long left, long result, long overflowMask) =>
        (overflowMask & (left ^ result)) >= 0;

    /// <summary>
    /// Adds two Fixed64 numbers, with saturating behavior in case of overflow.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;
        long sum = unchecked(xl + yl);
        if (!IsAddOrSubtractResultExact(xl, sum, ~(xl ^ yl)))
            sum = xl < 0 ? FixedMath.MIN_VALUE_L : FixedMath.MAX_VALUE_L;
        return new Fixed64(sum);
    }

    /// <summary>
    /// Adds an int to a Fixed64, with saturating behavior in case of overflow. 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(Fixed64 x, int y) => x + new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <inheritdoc cref="operator +(Fixed64, int)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(int x, Fixed64 y) => y + x;

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
    /// Subtracts one Fixed64 number from another, with saturating behavior in case of overflow.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;
        long diff = unchecked(xl - yl);
        if (!IsAddOrSubtractResultExact(xl, diff, xl ^ yl))
            diff = xl < 0 ? FixedMath.MIN_VALUE_L : FixedMath.MAX_VALUE_L;
        return new Fixed64(diff);
    }

    /// <summary>
    /// Subtracts an int from a Fixed64, with saturating behavior in case of overflow. 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x, int y) =>
        x - new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Subtracts a Fixed64 from an int, with saturating behavior in case of overflow.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(int x, Fixed64 y) =>
         new Fixed64((long)x << FixedMath.SHIFT_AMOUNT_I) - y;

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
    /// Multiplies two Fixed64 numbers, handling overflow and rounding.
    /// </summary>
    /// <summary>
    /// Multiplies two Fixed64 numbers using full-width 128-bit intermediate precision
    /// and round-half-to-even semantics on the discarded fractional bits.
    /// </summary>
    public static Fixed64 operator *(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;

        int shift = FixedMath.SHIFT_AMOUNT_I;

        // Determine sign of the final result.
        bool negative = ((xl ^ yl) < 0);

        // Convert to unsigned magnitudes safely, including long.MinValue.
        ulong ax = AbsToUInt64(xl);
        ulong ay = AbsToUInt64(yl);

        // Compute exact 128-bit unsigned product: (hi << 64) | lo
        Multiply64To128(ax, ay, out ulong hi, out ulong lo);

        // Shift-right with round-half-to-even using the FULL discarded remainder.
        ulong magnitude = ShiftRightRoundedToEven(hi, lo, shift, out bool roundedOverflow);

        // If rounding overflowed the shifted magnitude, carry it into saturation handling.
        if (!negative)
        {
            if (roundedOverflow || magnitude > long.MaxValue)
                return new Fixed64(FixedMath.MAX_VALUE_L);

            return new Fixed64((long)magnitude);
        }
        else
        {
            // For negative results, magnitude may be exactly 2^63, which maps to long.MinValue.
            const ulong minValueMagnitude = 0x8000000000000000UL;

            if (roundedOverflow || magnitude > minValueMagnitude)
                return new Fixed64(FixedMath.MIN_VALUE_L);

            if (magnitude == minValueMagnitude)
                return new Fixed64(FixedMath.MIN_VALUE_L);

            return new Fixed64(-(long)magnitude);
        }
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
    private static void Multiply64To128(ulong a, ulong b, out ulong hi, out ulong lo)
    {
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

    /// <summary>
    /// Compares the exact projection of three component differences without first
    /// converting those differences or their products back to <see cref="Fixed64"/>.
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
    /// Projects three component differences, clamps negative sums to zero, floors
    /// positive Q64.64 remainder, and saturates only the final Q32.32 result.
    /// </summary>
    internal static Fixed64 ProjectNonNegativeDifference(
        Fixed64 targetX,
        Fixed64 sourceX,
        Fixed64 directionX,
        Fixed64 targetY,
        Fixed64 sourceY,
        Fixed64 directionY,
        Fixed64 targetZ,
        Fixed64 sourceZ,
        Fixed64 directionZ)
    {
        GetDifferenceProjectionWords(
            targetX,
            sourceX,
            directionX,
            targetY,
            sourceY,
            directionY,
            targetZ,
            sourceZ,
            directionZ,
            out ulong sumHigh,
            out ulong sumMiddle,
            out ulong sumLow);

        if ((sumHigh & (1UL << 63)) != 0UL || (sumHigh | sumMiddle | sumLow) == 0UL)
            return Zero;

        // After the Q64.64-to-Q32.32 shift, the low 32 bits of this word become
        // the result's high 32 bits. long.MaxValue >> 32 is therefore the largest
        // positive middle word that can still produce a representable raw result.
        ulong positiveRawHighLimit = (ulong)(long.MaxValue >> FixedMath.SHIFT_AMOUNT_I);
        if (sumHigh != 0UL || sumMiddle > positiveRawHighLimit)
            return MaxValue;

        long rawResult = unchecked((long)(
            (sumMiddle << FixedMath.SHIFT_AMOUNT_I)
            | (sumLow >> FixedMath.SHIFT_AMOUNT_I)));
        return new Fixed64(rawResult);
    }

    private static void GetDifferenceProjectionWords(
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
    private static void AccumulateDifferenceProduct(
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

        Multiply64To128(
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
    /// Multiplies a Fixed64 by an integer, with overflow handling.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator *(Fixed64 x, int y) =>
        x * new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <inheritdoc cref="operator *(Fixed64, int)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator *(int x, Fixed64 y) => y * x;

    /// <summary>
    /// Divides one Fixed64 number by another, handling division by zero and overflow.
    /// </summary>
    public static Fixed64 operator /(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;

        if (yl == 0)
            throw new DivideByZeroException($"Attempted to divide {x} by zero.");

        return DivideMagnitude(
            AbsToUInt64(xl),
            AbsToUInt64(yl),
            (xl ^ yl) < 0);
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

    /// <summary>
    /// Divides a Fixed64 by an integer, handling division by zero and overflow.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator /(Fixed64 x, int y) =>
         x / new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Divides an integer by a Fixed64, handling division by zero and overflow.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator /(int y, Fixed64 x) =>
         new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I) / x;

    /// <summary>
    /// Computes the remainder of division of one Fixed64 number by another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator %(Fixed64 x, Fixed64 y)
    {
        if (x.m_rawValue == FixedMath.MIN_VALUE_L && y.m_rawValue == -1)
            return Zero;
        return new Fixed64(x.m_rawValue % y.m_rawValue);
    }

    /// <summary>
    /// Computes the remainder of division of a Fixed64 by an int.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator %(Fixed64 x, int y) => x % new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Computes the remainder of division of an int by a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator %(int x, Fixed64 y) => new Fixed64((long)x << FixedMath.SHIFT_AMOUNT_I) % y;

    /// <summary>
    /// Unary negation operator.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x) =>
        x.m_rawValue == FixedMath.MIN_VALUE_L
            ? new Fixed64(FixedMath.MAX_VALUE_L)
            : new Fixed64(-x.m_rawValue);

    /// <summary>
    /// Increments a Fixed64 number by one, with saturating behavior in case of overflow.
    /// </summary>
    /// <param name="a">The Fixed64 number to increment.</param>
    /// <returns>The incremented Fixed64 number.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator ++(Fixed64 a) => a + One;

    /// <summary>
    /// Decrements a Fixed64 number by one, with saturating behavior in case of overflow.
    /// </summary>
    /// <param name="a">The Fixed64 number to decrement.</param>
    /// <returns>The decremented Fixed64 number.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator --(Fixed64 a) => a - One;

    /// <summary>
    /// Bitwise left shift operator.
    /// </summary>
    /// <param name="a">Operand to shift.</param>
    /// <param name="shift">Number of bits to shift.</param>
    /// <returns>The shifted value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator <<(Fixed64 a, int shift) => new(a.m_rawValue << shift);

    /// <summary>
    /// Bitwise right shift operator.
    /// </summary>
    /// <param name="a">Operand to shift.</param>
    /// <param name="shift">Number of bits to shift.</param>
    /// <returns>The shifted value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator >>(Fixed64 a, int shift) => new(a.m_rawValue >> shift);

    #endregion
    #region Comparison Operators

    /// <summary>
    /// Determines whether one Fixed64 is greater than another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Fixed64 x, Fixed64 y) => x.m_rawValue > y.m_rawValue;

    /// <summary>
    /// Determines whether a Fixed64 is greater than an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Fixed64 x, int y) => x.m_rawValue > (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is greater than a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I > x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Fixed64 x, Fixed64 y) => x.m_rawValue < y.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Fixed64 x, int y) => x.m_rawValue < (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is less than a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I < x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is greater than or equal to another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Fixed64 x, Fixed64 y) => x.m_rawValue >= y.m_rawValue;

    /// <summary>
    /// Determines whether Fixed64 is greater than or equal to an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Fixed64 x, int y) => x.m_rawValue >= (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is greater than or equal to a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I >= x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than or equal to another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Fixed64 x, Fixed64 y) => x.m_rawValue <= y.m_rawValue;

    /// <summary>
    /// Determines whether a Fixed64 is less than or equal to an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Fixed64 x, int y) => x.m_rawValue <= (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is less than or equal to a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I <= x.m_rawValue;

    /// <summary>
    /// Determines whether two Fixed64 instances are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Fixed64 left, Fixed64 right) => left.Equals(right);

    /// <summary>
    /// Determines whether a Fixed64 instance is equal to an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Fixed64 left, int right) => left.m_rawValue == (long)right << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is equal to a Fixed64 instance.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(int left, Fixed64 right) => (long)left << FixedMath.SHIFT_AMOUNT_I == right.m_rawValue;

    /// <summary>
    /// Determines whether two Fixed64 instances are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Fixed64 left, Fixed64 right) => !left.Equals(right);

    /// <summary>
    /// Determines whether a Fixed64 instance is not equal to an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Fixed64 left, int right) => left.m_rawValue != (long)right << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is equal to a Fixed64 instance.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(int left, Fixed64 right) => (long)left << FixedMath.SHIFT_AMOUNT_I != right.m_rawValue;

    #endregion
}
