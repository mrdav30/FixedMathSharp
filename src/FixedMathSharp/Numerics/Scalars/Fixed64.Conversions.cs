//=======================================================================
// Fixed64.Conversions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public readonly partial struct Fixed64
{
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
        return FromDouble(value);
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
        return FromDouble(value);
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
        return FromDecimal(value);
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
    public string ToString(string? format) => ToString(format, CultureInfo.InvariantCulture);

    /// <summary>
    /// Converts the numeric value of the current Fixed64 object to its equivalent string representation.
    /// </summary>
    /// <param name="format">A format specification that governs how the current Fixed64 object is converted.</param>
    /// <param name="formatProvider">The provider to use for culture-specific formatting information.</param>
    /// <returns>The string representation of the value of the current Fixed64 object.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string? format, IFormatProvider? formatProvider) =>
        ((double)this).ToString(format, formatProvider ?? CultureInfo.InvariantCulture);

    /// <summary>
    /// Formats the value into the provided destination buffer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryFormat(
        Span<char> destination,
        out int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        return FixedDiagnosticsFormatter.TryFormat(this, destination, out charsWritten, format, provider);
    }

    /// <summary>
    /// Parses a decimal string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The string representation of the Fixed64 value.</param>
    /// <returns>The parsed Fixed64 value.</returns>
    public static Fixed64 Parse(string s) => Parse(s, CultureInfo.InvariantCulture);

    /// <summary>
    /// Parses a decimal string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The string representation of the Fixed64 value.</param>
    /// <param name="formatProvider">The provider to use for culture-specific parsing information.</param>
    /// <returns>The parsed Fixed64 value.</returns>
    public static Fixed64 Parse(string s, IFormatProvider? formatProvider)
    {
        if (s is null)
            throw new ArgumentNullException(nameof(s));

        if (!decimal.TryParse(s, NumberStyles.Float | NumberStyles.AllowThousands, formatProvider ?? CultureInfo.InvariantCulture, out decimal value))
            throw new FormatException($"Invalid format: {s}");

        return FromDecimal(value);
    }

    /// <summary>
    /// Parses a raw Q32.32 payload string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The raw fixed-point payload.</param>
    /// <returns>The parsed Fixed64 value.</returns>
    public static Fixed64 ParseRaw(string s)
    {
        if (s is null)
            throw new ArgumentNullException(nameof(s));

        if (!long.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out long rawValue))
            throw new FormatException($"Invalid format: {s}");

        return FromRaw(rawValue);
    }

    /// <summary>
    /// Tries to parse a decimal string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The string representation of the Fixed64 value.</param>
    /// <param name="result">The parsed Fixed64 value.</param>
    /// <returns>True if parsing succeeded; otherwise, false.</returns>
    public static bool TryParse(string? s, out Fixed64 result) => TryParse(s, CultureInfo.InvariantCulture, out result);

    /// <summary>
    /// Tries to parse a decimal string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The string representation of the Fixed64 value.</param>
    /// <param name="formatProvider">The provider to use for culture-specific parsing information.</param>
    /// <param name="result">The parsed Fixed64 value.</param>
    /// <returns>True if parsing succeeded; otherwise, false.</returns>
    public static bool TryParse(string? s, IFormatProvider? formatProvider, out Fixed64 result)
    {
        result = Zero;
        if (string.IsNullOrEmpty(s))
            return false;

        if (!decimal.TryParse(s, NumberStyles.Float | NumberStyles.AllowThousands, formatProvider ?? CultureInfo.InvariantCulture, out decimal value))
            return false;

        return TryFromDecimal(value, out result);
    }

    /// <summary>
    /// Tries to parse a raw Q32.32 payload string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The raw fixed-point payload.</param>
    /// <param name="result">The parsed Fixed64 value.</param>
    /// <returns>True if parsing succeeded; otherwise, false.</returns>
    public static bool TryParseRaw(string? s, out Fixed64 result)
    {
        result = Zero;
        if (string.IsNullOrEmpty(s))
            return false;

        if (!long.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out long rawValue))
            return false;

        result = FromRaw(rawValue);
        return true;
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
    public static int ToInt(Fixed64 value) => (int)(value.m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Constructs a Fixed64 from a double-precision floating-point value.
    /// </summary>
    /// <remarks>
    /// The value is multiplied by the scaling factor (2^SHIFT_AMOUNT) and 
    /// rounded to the nearest integer to fit into the fixed-point representation.
    /// </remarks>
    /// <param name="value">Double value to convert to </param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromDouble(double value) => new((long)Math.Round(value * FixedMath.ONE_L));

    /// <summary>
    /// Constructs a Fixed64 from a decimal value.
    /// </summary>
    /// <remarks>
    /// The value is multiplied by the scaling factor (2^SHIFT_AMOUNT) and rounded to the nearest
    /// raw integer using midpoint-to-even rounding. Values outside the Q32.32 range throw an
    /// <see cref="OverflowException"/>.
    /// </remarks>
    /// <param name="value">Decimal value to convert.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromDecimal(decimal value)
    {
        decimal rawValue = decimal.Round(value * FixedMath.ONE_L, 0, MidpointRounding.ToEven);

        if (rawValue < long.MinValue || rawValue > long.MaxValue)
            throw new OverflowException($"{value} is outside the representable range of {nameof(Fixed64)}.");

        return FromRaw((long)rawValue);
    }

    private static bool TryFromDecimal(decimal value, out Fixed64 result)
    {
        result = Zero;

        try
        {
            result = FromDecimal(value);
            return true;
        }
        catch (OverflowException)
        {
            return false;
        }
    }

    /// <summary>
    /// Creates a Fixed64 from a fractional number.
    /// </summary>
    /// <param name="numerator">The numerator of the fraction.</param>
    /// <param name="denominator">The denominator of the fraction.</param>
    /// <returns>A Fixed64 representing the fraction.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromFraction(double numerator, double denominator) => FromDouble(numerator / denominator);

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
}
