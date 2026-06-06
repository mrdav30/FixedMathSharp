//=======================================================================
// Fixed64.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a Q(64-SHIFT_AMOUNT).SHIFT_AMOUNT fixed-point number.
/// Provides high precision for fixed-point arithmetic where SHIFT_AMOUNT bits 
/// are used for the fractional part and (64 - SHIFT_AMOUNT) bits for the integer part.
/// The precision is determined by SHIFT_AMOUNT, which defines the resolution of fractional values.
/// </summary>
[Serializable]
[MemoryPackable]
public readonly partial struct Fixed64 : IEquatable<Fixed64>, IComparable<Fixed64>, IEqualityComparer<Fixed64>
{
    #region Static Fields

    /// <inheritdoc cref="FixedMath.MAX_VALUE_L" />
    public static Fixed64 MaxValue => new(FixedMath.MAX_VALUE_L);
    /// <inheritdoc cref="FixedMath.MIN_VALUE_L" />
    public static Fixed64 MinValue => new(FixedMath.MIN_VALUE_L);

    /// <inheritdoc cref="FixedMath.ONE_L" />
    public static Fixed64 One => new(FixedMath.ONE_L);
    /// <summary>
    /// Represents the value -1 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static Fixed64 NegOne => -One;
    /// <summary>
    /// Represents the value 2 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static Fixed64 Two => One * 2;
    /// <summary>
    /// Represents the value 3 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static Fixed64 Three => One * 3;
    /// <summary>
    /// Represents the value 0.5 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static Fixed64 Half => One / 2;
    /// <summary>
    /// Represents the value 0.25 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static Fixed64 Quarter => One / 4;
    /// <summary>
    /// Represents the value 0.125 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static Fixed64 Eighth => One / 8;
    /// <summary>
    /// Represents the value 0 as a fixed-point number.
    /// </summary>
    public static Fixed64 Zero => new(0);

    /// <inheritdoc cref="FixedMath.PI_LONG" />
    public static Fixed64 Pi => new(FixedMath.PI_LONG);
    /// <summary>
    /// Represents the mathematical constant 2π as a fixed-point value.
    /// </summary>
    /// <remarks>This value is commonly used to represent a full rotation in radians.</remarks>
    public static Fixed64 TwoPi => Pi * Two;
    /// <summary>
    /// Represents the mathematical constant π divided by 2 as a fixed-point value.
    /// </summary>
    /// <remarks>This value is used where π/2 is required.</remarks>
    public static Fixed64 HalfPi => Pi / new Fixed64(2);
    /// <summary>
    /// Represents the mathematical constant π divided by 3 as a fixed-point value.
    /// </summary>
    public static Fixed64 PiOver3 => Pi / new Fixed64(3);
    /// <summary>
    /// Represents the mathematical constant π divided by 4 as a fixed-point value.
    /// </summary>
    public static Fixed64 PiOver4 => Pi / new Fixed64(4);
    /// <summary>
    /// Represents the mathematical constant π divided by 6 as a fixed-point value.
    /// </summary>
    public static Fixed64 PiOver6 => Pi / new Fixed64(6);
    /// <summary>
    /// Represents the multiplicative inverse of the mathematical constant π (pi) as a fixed-point value.
    /// </summary>
    public static Fixed64 InvertedPi => One / Pi;
    /// <summary>
    /// Represents the fixed-point value for 180.
    /// </summary>
    public static Fixed64 OneEighty => new(180);
    /// <summary>
    /// Degrees to radians conversion factor (π / 180)
    /// </summary>
    public static Fixed64 Deg2Rad => Pi / OneEighty;
    /// <summary>
    /// Radians to degrees conversion factor (180 / π)
    /// </summary>
    public static Fixed64 Rad2Deg => OneEighty / Pi;

    /// <inheritdoc cref="FixedMath.LN2_LONG" />
    public static Fixed64 Ln2 => new(FixedMath.LN2_LONG);
    /// <summary>
    /// Represents the maximum value for the base-2 logarithm that can be represented by a Fixed64 instance.
    /// </summary>
    /// <remarks>
    /// This constant is useful when performing logarithmic calculations to ensure results 
    /// do not exceed the representable range of the Fixed64 type.
    /// </remarks>
    public static Fixed64 Log2Max => new(63L * FixedMath.ONE_L);
    /// <summary>
    /// Represents the minimum base-2 logarithm value supported by the Fixed64 type.
    /// </summary>
    /// <remarks>
    /// This constant can be used as a lower bound when performing logarithmic calculations
    /// with Fixed64 values to prevent underflow or invalid results.
    /// </remarks>
    public static Fixed64 Log2Min => new(-64L * FixedMath.ONE_L);

    internal static Fixed64 PadeA1 => new(FixedMath.PADE_A1_LONG);
    internal static Fixed64 PadeA2 => new(FixedMath.PADE_A2_LONG);

    internal static Fixed64 SinCoeff3 => new(FixedMath.SIN_COEFF_3_LONG); // 1/3!
    internal static Fixed64 SinCoeff5 => new(FixedMath.SIN_COEFF_5_LONG); // 1/5!
    internal static Fixed64 SinCoeff7 => new(FixedMath.SIN_COEFF_7_LONG); // 1/7!

    /// <inheritdoc cref="FixedMath.MIN_INCREMENT_L" />
    public static Fixed64 MinIncrement => new(FixedMath.MIN_INCREMENT_L);
    /// <inheritdoc cref="FixedMath.DEFAULT_TOLERANCE_L" />
    public static Fixed64 Epsilon => new(FixedMath.DEFAULT_TOLERANCE_L);

    #endregion

    #region Fields

    /// <summary>
    /// The underlying raw long value representing the fixed-point number.
    /// </summary>
    [JsonInclude]
    [MemoryPackInclude]
    public readonly long m_rawValue;

    #endregion

    #region Constructors

    /// <summary>
    /// Internal constructor for a Fixed64 from a raw long value.
    /// </summary>
    /// <param name="m_rawValue">Raw long value representing the fixed-point number.</param>
    [JsonConstructor]
    internal Fixed64(long m_rawValue) => this.m_rawValue = m_rawValue;

    /// <summary>
    /// Constructs a Fixed64 from an integer, with the fractional part set to zero.
    /// </summary>
    /// <param name="value">Integer value to convert to </param>
    public Fixed64(int value) : this((long)value << FixedMath.SHIFT_AMOUNT_I) { }

    #endregion

    #region Methods (Instance)

    /// <summary>
    /// Offsets the current Fixed64 by an integer value.
    /// </summary>
    /// <param name="x">The integer value to add.</param>
    /// <remarks>
    /// Unlike the <see cref="operator +(Fixed64, int)"/>, this method performs a simple addition of the integer value 
    /// to the raw fixed-point representation without any overflow checks or saturation behavior.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Offset(int x)
    {
        return new Fixed64(m_rawValue + ((long)x << FixedMath.SHIFT_AMOUNT_I));
    }

    /// <summary>
    /// Returns the raw value as a string.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string RawToString()
    {
        return m_rawValue.ToString();
    }

    #endregion

    #region Static Operations

    /// <summary>
    /// Counts the leading zeros in a 64-bit unsigned integer.
    /// </summary>
    /// <param name="x">The number to count leading zeros for.</param>
    /// <returns>The number of leading zeros.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CountLeadingZeroes(ulong x)
    {
        int result = 0;
        while ((x & 0xF000000000000000) == 0) { result += 4; x <<= 4; }
        while ((x & 0x8000000000000000) == 0) { result += 1; x <<= 1; }
        return result;
    }

    /// <summary>
    /// Returns a number indicating the sign of a Fix64 number.
    /// Returns 1 if the value is positive, 0 if is 0, and -1 if it is negative.
    /// </summary>
    /// <remarks>
    /// Optimized for branchless comparison.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Sign(Fixed64 value) =>
        value.m_rawValue < 0
                ? -1
                : (value.m_rawValue > 0 ? 1 : 0);

    /// <summary>
    /// Returns true if the number has no decimal part (i.e., if the number is equivalent to an integer) and False otherwise. 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsInteger(Fixed64 value)
    {
        return ((ulong)value.m_rawValue & FixedMath.MAX_SHIFTED_AMOUNT_UI) == 0;
    }

    /// <summary>
    /// Performs a smooth step interpolation using a cubic Hermite curve between two values.
    /// </summary>
    /// <remarks>
    /// The interpolation follows a cubic Hermite curve where the function starts at `a`,
    /// accelerates, and then decelerates towards `b`, ensuring smooth transitions.
    /// </remarks>
    /// <param name="a">The starting value.</param>
    /// <param name="b">The ending value.</param>
    /// <param name="t">A value between 0 and 1 that represents the interpolation factor.</param>
    /// <returns>The interpolated value between `a` and `b`.</returns>
    public static Fixed64 SmoothStep(Fixed64 a, Fixed64 b, Fixed64 t)
    {
        if (t.m_rawValue <= 0)
            return a;
        if (t.m_rawValue >= FixedMath.ONE_L)
            return b;
        // Hermite interpolation: f(t) = 3t^2 - 2t^3
        Fixed64 t2 = t * t;
        Fixed64 t3 = t2 * t;
        return a + (b - a) * (Three * t2 - Two * t3);
    }

    /// <summary>
    /// Performs cubic interpolation between two points `p0` and `p1` with tangents `m0` and `m1` at those points.
    /// </summary>
    /// <remarks>
    /// This method interpolates smoothly between `p0` and `p1` while considering the tangents `m0` and `m1`.
    /// It is useful for animation curves and smooth motion transitions.
    /// </remarks>
    /// <param name="p0">The first point.</param>
    /// <param name="p1">The second point.</param>
    /// <param name="m0">The tangent (slope) at `p0`.</param>
    /// <param name="m1">The tangent (slope) at `p1`.</param>
    /// <param name="t">A value between 0 and 1 that represents the interpolation factor.</param>
    /// <returns>The interpolated value between `p0` and `p1`.</returns>
    public static Fixed64 CubicInterpolate(Fixed64 p0, Fixed64 p1, Fixed64 m0, Fixed64 m1, Fixed64 t)
    {
        Fixed64 t2 = t * t;
        Fixed64 t3 = t2 * t;
        return (Two * p0 - Two * p1 + m0 + m1) * t3
             + (-Three * p0 + Three * p1 - Two * m0 - m1) * t2
             + m0 * t + p0;
    }

    /// <summary>
    /// Linearly interpolates between two fixed-point values based on a given interpolation factor `t`.
    /// </summary>
    /// <param name="from">The starting value.</param>
    /// <param name="to">The ending value.</param>
    /// <param name="t">A value between 0 and 1 that represents the interpolation factor.</param>
    /// <returns>The interpolated value between `from` and `to`.</returns>
    /// <remarks>
    /// The interpolation is clamped between `from` and `to` based on the value of `t`.
    /// If `t` is less than 0, the result is `from`. If `t` is greater than 1, the result is `to`.
    /// </remarks>
    public static Fixed64 Lerp(Fixed64 from, Fixed64 to, Fixed64 t)
    {
        if (t.m_rawValue >= FixedMath.ONE_L)
            return to;
        if (t.m_rawValue <= 0)
            return from;

        return from + (to - from) * t;
    }

    /// <summary>
    /// Computes the interpolated point along a Catmull-Rom spline given four control points.
    /// </summary>
    /// <param name="p0">The first control point.</param>
    /// <param name="p1">The second control point.</param>
    /// <param name="p2">The third control point.</param>
    /// <param name="p3">The fourth control point.</param>
    /// <param name="t">Interpolation factor between 0 and 1.</param>
    /// <returns>The interpolated point on the spline.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CatmullRom(Fixed64 p0, Fixed64 p1, Fixed64 p2, Fixed64 p3, Fixed64 t)
    {
        // Classic Catmull-Rom basis matrix
        Fixed64 t2 = t * t;
        Fixed64 t3 = t2 * t;
        return ((-t3 + 2 * t2 - t) * p0 +
             (3 * t3 - 5 * t2 + 2) * p1 +
             (-3 * t3 + 4 * t2 + t) * p2 +
             (t3 - t2) * p3) / 2;
    }

    /// <summary>
    /// Performs a Hermite interpolation between two Fixed64 values, using the specified tangents and interpolation amount.
    /// </summary>
    /// <param name="value1">The first value.</param>
    /// <param name="tangent1">The tangent at the first value.</param>
    /// <param name="value2">The second value.</param>
    /// <param name="tangent2">The tangent at the second value.</param>
    /// <param name="amount">The interpolation amount.</param>
    /// <returns>The Hermite spline interpolated value.</returns>
    public static Fixed64 HermiteSpline(
        Fixed64 value1,
        Fixed64 tangent1,
        Fixed64 value2,
        Fixed64 tangent2,
        Fixed64 amount)
    {
        if ((amount - Zero).LessThanEpsilon())
            return value1;

        if ((amount - One).LessThanEpsilon())
            return value2;

        Fixed64 v1 = value1, v2 = value2, t1 = tangent1, t2 = tangent2, s = amount;
        Fixed64 sCubed = s * s * s;
        Fixed64 sSquared = s * s;
        Fixed64 result = (
            ((2 * v1 - 2 * v2 + t2 + t1) * sCubed) +
            ((3 * v2 - 3 * v1 - 2 * t1 - t2) * sSquared) +
            (t1 * s) +
            v1
        );

        return result;
    }

    /// <summary>
    /// Performs a barycentric interpolation between three Fixed64 values that represent the coordinates of the vertices of a triangle.
    /// </summary>
    /// <param name="value1">The coordinate of the first vertex.</param>
    /// <param name="value2">The coordinate of the second vertex.</param>
    /// <param name="value3">The coordinate of the third vertex.</param>
    /// <param name="amount1">The normalized barycentric coordinate for the second vertex equal to the weight of the second vertex.</param>
    /// <param name="amount2">The normalized barycentric coordinate for the third vertex equal to the weight of the third vertex.</param>
    /// <returns>The cartesian coordinate for one axis based on the barycentric interpolation.</returns>
    public static Fixed64 BarycentricCoordinate(
        Fixed64 value1,
        Fixed64 value2,
        Fixed64 value3,
        Fixed64 amount1,
        Fixed64 amount2
    ) => value1 + (value2 - value1) * amount1 + (value3 - value1) * amount2;

    /// <summary>
    /// Divides by a known-positive fixed-point divisor using the same quotient and rounding
    /// semantics as <see cref="op_Division(Fixed64, Fixed64)"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 DivideByPositive(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;

        if (yl <= 0)
            return x / y;

        ulong remainder = (ulong)(xl < 0 ? -xl : xl);
        ulong divider = (ulong)yl;
        ulong quotient = 0UL;
        int bitPos = FixedMath.SHIFT_AMOUNT_I + 1;

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

            if ((div & ~(0xFFFFFFFFFFFFFFFF >> bitPos)) != 0)
                return xl >= 0
                    ? new Fixed64(FixedMath.MAX_VALUE_L)
                    : new Fixed64(FixedMath.MIN_VALUE_L);

            remainder <<= 1;
            --bitPos;
        }

        if ((quotient & 0x1) != 0)
            quotient += 1;

        long result = (long)(quotient >> 1);
        if (xl < 0)
            result = -result;

        return new Fixed64(result);
    }

    #endregion

    #region Explicit and Implicit Conversions

    /// <summary>
    /// Converts a 64-bit signed integer to a Fixed64 value using explicit casting.
    /// </summary>
    /// <remarks>
    /// The conversion interprets the input value as the integer part of the fixed-point number and saturates
    /// to <see cref="MinValue"/> or <see cref="MaxValue"/> when the integer cannot fit in Q32.32 value space.
    /// Use <see cref="FromRaw(long)"/> when a raw fixed-point payload is required.
    /// </remarks>
    /// <param name="value">The 64-bit signed integer to convert to a Fixed64 value.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(long value)
    {
        if (value > int.MaxValue)
            return MaxValue;

        if (value < int.MinValue)
            return MinValue;

        return FromRaw(value << FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Converts a Fixed64 value to a 64-bit signed integer by discarding the fractional part.
    /// </summary>
    /// <remarks>
    /// The conversion truncates any fractional component. 
    /// The result represents the integer portion of the Fixed64 value.</remarks>
    /// <param name="value">The Fixed64 value to convert to a long integer.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator long(Fixed64 value)
    {
        return value > Zero
            ? (long)(FixedMath.Floor(value).m_rawValue >> FixedMath.SHIFT_AMOUNT_I)
            : (long)(FixedMath.Ceil(value).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Defines an explicit conversion from a 32-bit signed integer to a Fixed64 value.
    /// </summary>
    /// <param name="value">The 32-bit signed integer to convert to a Fixed64 value.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(int value)
    {
        return new Fixed64(value);
    }

    /// <summary>
    /// Converts a Fixed64 value to a 32-bit signed integer by discarding the fractional part.
    /// </summary>
    /// <remarks>
    /// The conversion truncates any fractional component. 
    /// The result is equivalent to rounding toward zero.
    /// </remarks>
    /// <param name="value">The Fixed64 value to convert to an integer.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator int(Fixed64 value)
    {
        return value > Zero
            ? (int)(FixedMath.Floor(value).m_rawValue >> FixedMath.SHIFT_AMOUNT_I)
            : (int)(FixedMath.Ceil(value).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Defines an explicit conversion from a single-precision floating-point value to a Fixed64 instance.
    /// </summary>
    /// <param name="value">The single-precision floating-point value to convert to Fixed64.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(float value)
    {
        return FromFloatPoint(value);
    }

    /// <summary>
    /// Converts a Fixed64 value to its equivalent single-precision floating-point representation.
    /// </summary>
    /// <remarks>
    /// This conversion may result in a loss of precision if the Fixed64 value cannot be exactly  represented as a float.
    /// </remarks>
    /// <param name="value">The Fixed64 value to convert to a float.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator float(Fixed64 value)
    {
        return value.m_rawValue * FixedMath.SCALE_FACTOR_F;
    }

    /// <summary>
    /// Defines an explicit conversion from a double-precision floating-point number to a Fixed64 value.
    /// </summary>
    /// <remarks>
    /// This conversion may result in loss of precision if the double value cannot be exactly represented as a Fixed64.
    /// </remarks>
    /// <param name="value">The double-precision floating-point number to convert to a Fixed64 value.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(double value)
    {
        return FromFloatPoint(value);
    }

    /// <summary>
    /// Converts a Fixed64 value to its equivalent double-precision floating-point representation.
    /// </summary>
    /// <remarks>
    /// This conversion may result in a loss of precision if the Fixed64 value cannot be exactly represented as a double.
    /// </remarks>
    /// <param name="value">The Fixed64 value to convert to a double.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator double(Fixed64 value)
    {
        return value.m_rawValue * FixedMath.SCALE_FACTOR_D;
    }

    /// <summary>
    /// Defines an explicit conversion from a decimal value to a Fixed64 instance.
    /// </summary>
    /// <param name="value">The decimal value to convert to a Fixed64 instance.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(decimal value)
    {
        return FromFloatPoint((double)value);
    }

    /// <summary>
    /// Converts a Fixed64 value to its decimal representation.
    /// </summary>
    /// <remarks>
    /// This operator provides an explicit conversion from Fixed64 to decimal, preserving the numeric value as closely as possible. 
    /// Use this conversion when precise decimal arithmetic is required.
    /// </remarks>
    /// <param name="value">The Fixed64 value to convert to decimal.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator decimal(Fixed64 value)
    {
        return value.m_rawValue * FixedMath.SCALE_FACTOR_M;
    }

    #endregion

    #region Arithmetic Operators

    /// <summary>
    /// Adds two Fixed64 numbers, with saturating behavior in case of overflow.
    /// </summary>
    public static Fixed64 operator +(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;
        long sum = xl + yl;
        // Check for overflow, if signs of operands are equal and signs of sum and x are different
        if (((~(xl ^ yl) & (xl ^ sum)) & FixedMath.MIN_VALUE_L) != 0)
            sum = xl > 0 ? FixedMath.MAX_VALUE_L : FixedMath.MIN_VALUE_L;
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
    /// Subtracts one Fixed64 number from another, with saturating behavior in case of overflow.
    /// </summary>
    public static Fixed64 operator -(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;
        long diff = xl - yl;
        // Check for overflow, if signs of operands are different and signs of sum and x are different
        if ((((xl ^ yl) & (xl ^ diff)) & FixedMath.MIN_VALUE_L) != 0)
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
    private static ulong AbsToUInt64(long value)
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

        ulong remainder = (ulong)(xl < 0 ? -xl : xl);
        ulong divider = (ulong)(yl < 0 ? -yl : yl);
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

            // Detect overflow
            if ((div & ~(0xFFFFFFFFFFFFFFFF >> bitPos)) != 0)
                return ((xl ^ yl) & FixedMath.MIN_VALUE_L) == 0
                    ? new Fixed64(FixedMath.MAX_VALUE_L)
                    : new Fixed64(FixedMath.MIN_VALUE_L);

            remainder <<= 1;
            --bitPos;
        }

        // Rounding logic: "Round half to even" or "Banker's rounding"
        if ((quotient & 0x1) != 0)
            quotient += 1;

        long result = (long)(quotient >> 1);
        if (((xl ^ yl) & FixedMath.MIN_VALUE_L) != 0)
            result = -result;

        return new Fixed64(result);
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

    #region Conversion

    /// <summary>
    /// Returns the string representation of this Fixed64 instance.
    /// </summary>
    /// <remarks>
    /// Up to 10 decimal places.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() => ((double)this).ToString(CultureInfo.InvariantCulture);

    /// <summary>
    /// Converts the numeric value of the current Fixed64 object to its equivalent string representation.
    /// </summary>
    /// <param name="format">A format specification that governs how the current Fixed64 object is converted.</param>
    /// <returns>The string representation of the value of the current Fixed64 object.</returns>  
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string format) => ((double)this).ToString(format, CultureInfo.InvariantCulture);

    /// <summary>
    /// Parses a string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The string representation of the Fixed64 value.</param>
    /// <returns>The parsed Fixed64 value.</returns>
    public static Fixed64 Parse(string s)
    {
        if (string.IsNullOrEmpty(s)) throw new ArgumentNullException(nameof(s));

        // Check if the value is negative
        bool isNegative = false;
        if (s[0] == '-')
        {
            isNegative = true;
            s = s[1..];
        }

        if (!long.TryParse(s, out long rawValue))
            throw new FormatException($"Invalid format: {s}");

        // If the value was negative, negate the result
        if (isNegative)
            rawValue = -rawValue;

        return FromRaw(rawValue);
    }

    /// <summary>
    /// Tries to parse a string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The string representation of the Fixed64 value.</param>
    /// <param name="result">The parsed Fixed64 value.</param>
    /// <returns>True if parsing succeeded; otherwise, false.</returns>
    public static bool TryParse(string s, out Fixed64 result)
    {
        result = Zero;
        try
        {
            result = Parse(s);
            return true;
        }
        catch
        {
            return false;
        }
    }

    /// <summary>
    /// Creates a Fixed64 from a raw long value.
    /// </summary>
    /// <remarks>
    /// This method interprets <paramref name="rawValue"/> as an already-scaled Q32.32 payload.
    /// Use the integer constructors or explicit numeric conversions for user-facing integer values.
    /// </remarks>
    /// <param name="rawValue">The raw fixed-point payload.</param>
    /// <returns>A Fixed64 representing the raw value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromRaw(long rawValue) => new(rawValue);


    /// <summary>
    /// Converts a Fixed64's RawValue (Int64) into an integer by discarding the fractional part.
    /// </summary>
    /// <param name="value">The Fixed64 value to convert.</param>
    /// <returns>The integer representation of the Fixed64 value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int RawToInt(Fixed64 value) => (int)(value.m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Constructs a Fixed64 from a double-precision floating-point value.
    /// </summary>
    /// <remarks>
    /// The value is multiplied by the scaling factor (2^SHIFT_AMOUNT) and 
    /// rounded to the nearest integer to fit into the fixed-point representation.
    /// </remarks>
    /// <param name="value">Double value to convert to </param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromFloatPoint(double value) => new((long)Math.Round(value * FixedMath.ONE_L));

    /// <summary>
    /// Creates a Fixed64 from a fractional number.
    /// </summary>
    /// <param name="numerator">The numerator of the fraction.</param>
    /// <param name="denominator">The denominator of the fraction.</param>
    /// <returns>A Fixed64 representing the fraction.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromFraction(double numerator, double denominator) => FromFloatPoint(numerator / denominator);

    /// <summary>
    /// Converts a Fixed64s RawValue (Int64) into a double
    /// </summary>
    /// <param name="f1"></param>
    /// <returns></returns>
    public static double ToDouble(long f1) => f1 * FixedMath.SCALE_FACTOR_D;

    /// <summary>
    /// Converts a Fixed64s RawValue (Int64) into a float
    /// </summary>
    /// <param name="f1"></param>
    /// <returns></returns>
    public static float ToFloat(long f1) => f1 * FixedMath.SCALE_FACTOR_F;

    /// <summary>
    /// Converts a Fixed64s RawValue (Int64) into a decimal
    /// </summary>
    /// <param name="f1"></param>
    /// <returns></returns>
    public static decimal ToDecimal(long f1) => f1 * FixedMath.SCALE_FACTOR_M;

    #endregion

    #region Equality, HashCode, Comparable Overrides

    /// <summary>
    /// Determines whether this instance equals another object.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Fixed64 other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed64 other) => m_rawValue == other.m_rawValue;

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed64 x, Fixed64 y) => x.Equals(y);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => m_rawValue.GetHashCode();

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetHashCode(Fixed64 obj) => obj.GetHashCode();

    /// <summary>
    /// Compares this instance to another 
    /// </summary>
    /// <param name="other">The Fixed64 to compare with.</param>
    /// <returns>-1 if less than, 0 if equal, 1 if greater than other.</returns>
    public int CompareTo(Fixed64 other) => m_rawValue.CompareTo(other.m_rawValue);

    #endregion
}
