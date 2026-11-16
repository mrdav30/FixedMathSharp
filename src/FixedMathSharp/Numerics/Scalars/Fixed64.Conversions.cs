//=======================================================================
// Fixed64.Conversions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using Chronicler.Diagnostics;

namespace FixedMathSharp;

/// <content>
/// Conversion operators between Fixed64 and other numeric types (int, long, float, double, decimal, etc.).
/// </content>
public partial struct Fixed64
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
    /// <returns>The fixed-point integer, saturated at the representable endpoints.</returns>
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
    /// <returns>The value rounded toward zero.</returns>
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
    /// <returns>The exact integer represented in Q32.32.</returns>
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
    /// <returns>The value rounded toward zero.</returns>
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
    /// <returns>The nearest representable value, with ties to even.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="value"/> is NaN or infinity.</exception>
    /// <exception cref="OverflowException">The rounded value is outside the Q32.32 range.</exception>
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
    /// <returns>The floating-point approximation of the fixed-point value.</returns>
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
    /// <returns>The nearest representable value, with ties to even.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="value"/> is NaN or infinity.</exception>
    /// <exception cref="OverflowException">The rounded value is outside the Q32.32 range.</exception>
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
    /// <returns>The floating-point approximation of the fixed-point value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator double(Fixed64 value)
    {
        return value.m_rawValue * FixedMath.SCALE_FACTOR_D;
    }

    /// <summary>
    /// Defines an explicit conversion from a decimal value to a Fixed64 instance.
    /// </summary>
    /// <param name="value">The decimal value to convert to a Fixed64 instance.</param>
    /// <returns>The nearest representable value, with ties to even.</returns>
    /// <exception cref="OverflowException">The rounded value is outside the Q32.32 range.</exception>
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
    /// <returns>The equivalent decimal value, rounded if the decimal precision is insufficient.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator decimal(Fixed64 value)
    {
        return value.m_rawValue * FixedMath.SCALE_FACTOR_M;
    }

    #endregion
    #region Conversion

    /// <summary>Formats the value using the invariant culture.</summary>
    /// <returns>The default representation of the value converted to <see cref="double"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() => ((double)this).ToString(CultureInfo.InvariantCulture);

    /// <summary>
    /// Converts the numeric value of the current Fixed64 object to its equivalent string representation.
    /// </summary>
    /// <returns>The string representation of the value of the current Fixed64 object.</returns>  
    /// <param name="format">A numeric format string. A null or empty string uses the default format.</param>
    /// <returns>The formatted representation of the value converted to <see cref="double"/>.</returns>
    /// <exception cref="FormatException"><paramref name="format"/> is invalid.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string? format) => ToString(format, CultureInfo.InvariantCulture);

    /// <summary>
    /// Converts the numeric value of the current Fixed64 object to its equivalent string representation.
    /// </summary>
    /// <param name="format">A numeric format string. A null or empty string uses the default format.</param>
    /// <param name="formatProvider">The culture or format provider. Null selects the invariant culture.</param>
    /// <returns>The formatted representation of the value converted to <see cref="double"/>.</returns>
    /// <exception cref="FormatException"><paramref name="format"/> is invalid.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string? format, IFormatProvider? formatProvider) =>
        ((double)this).ToString(format, formatProvider ?? CultureInfo.InvariantCulture);

    /// <summary>
    /// Formats the value into the provided destination buffer.
    /// </summary>
    /// <param name="destination">Caller-owned storage. On success, the formatted value replaces the prefix of this span.</param>
    /// <param name="charsWritten">The number of characters written on success, or zero when the destination is too small.</param>
    /// <param name="format">The numeric format. An empty span selects the default format.</param>
    /// <param name="provider">The culture or format provider. Null selects the invariant culture.</param>
    /// <returns><see langword="true"/> if the formatted value fits, otherwise <see langword="false"/>.</returns>
    /// <remarks>Formatting uses the value converted to <see cref="double"/>.</remarks>
    /// <exception cref="FormatException"><paramref name="format"/> is invalid.</exception>
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
    /// <param name="s">A decimal number with optional sign, whitespace, exponent, and group separators. Parsing uses the invariant culture.</param>
    /// <returns>The parsed value rounded to the nearest representable value with ties to even.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="s"/> is null.</exception>
    /// <exception cref="FormatException">The text cannot be parsed as a decimal number.</exception>
    /// <exception cref="OverflowException">The parsed decimal rounds outside the Q32.32 range.</exception>
    public static Fixed64 Parse(string s) => Parse(s, CultureInfo.InvariantCulture);

    /// <summary>
    /// Parses a decimal string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">A decimal number with optional sign, whitespace, exponent, and group separators.</param>
    /// <param name="formatProvider">The provider for the text format. Null selects the invariant culture.</param>
    /// <returns>The parsed value rounded to the nearest representable value with ties to even.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="s"/> is null.</exception>
    /// <exception cref="FormatException">The text cannot be parsed as a decimal number.</exception>
    /// <exception cref="OverflowException">The parsed decimal rounds outside the Q32.32 range.</exception>
    public static Fixed64 Parse(string s, IFormatProvider? formatProvider)
    {
        ChronicleGuard.ThrowIfNull(s, nameof(s));

        if (!decimal.TryParse(s, NumberStyles.Float | NumberStyles.AllowThousands, formatProvider ?? CultureInfo.InvariantCulture, out decimal value))
            throw new FormatException($"Invalid format: {s}");

        return FromDecimal(value);
    }

    /// <summary>
    /// Parses a raw Q32.32 payload string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">An invariant-culture signed 64-bit integer encoding the raw Q32.32 payload.</param>
    /// <returns>The value encoded by the raw payload.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="s"/> is null.</exception>
    /// <exception cref="FormatException">The text does not encode a signed 64-bit integer.</exception>
    public static Fixed64 ParseRaw(string s)
    {
        ChronicleGuard.ThrowIfNull(s, nameof(s));

        if (!long.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out long rawValue))
            throw new FormatException($"Invalid format: {s}");

        return FromRaw(rawValue);
    }

    /// <summary>
    /// Tries to parse a decimal string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">A decimal number with optional sign, whitespace, exponent, and group separators. Parsing uses the invariant culture.</param>
    /// <param name="result">The parsed value on success, or <see cref="Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if the text can be parsed and rounded to a Q32.32 value, otherwise <see langword="false"/>.</returns>
    public static bool TryParse(string? s, out Fixed64 result) => TryParse(s, CultureInfo.InvariantCulture, out result);

    /// <summary>
    /// Tries to parse a decimal string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">A decimal number with optional sign, whitespace, exponent, and group separators.</param>
    /// <param name="formatProvider">The provider for the text format. Null selects the invariant culture.</param>
    /// <param name="result">The parsed value on success, or <see cref="Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if the text can be parsed and rounded to a Q32.32 value, otherwise <see langword="false"/>.</returns>
    public static bool TryParse(string? s, IFormatProvider? formatProvider, out Fixed64 result)
    {
        result = Zero;
        if (string.IsNullOrEmpty(s))
            return false;

        if (!decimal.TryParse(s, NumberStyles.Float | NumberStyles.AllowThousands, formatProvider ?? CultureInfo.InvariantCulture, out decimal value))
            return false;

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
    /// Tries to parse a raw Q32.32 payload string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">An invariant-culture signed 64-bit integer encoding the raw Q32.32 payload.</param>
    /// <param name="result">The parsed value on success, or <see cref="Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if the text encodes a signed 64-bit integer, otherwise <see langword="false"/>.</returns>
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


    /// <summary>Converts a fixed-point value to an integer by rounding down.</summary>
    /// <param name="value">The Fixed64 value to convert.</param>
    /// <returns>The largest integer less than or equal to <paramref name="value"/>.</returns>
    /// <remarks>The explicit conversion to <see cref="int"/> rounds toward zero.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int ToInt(Fixed64 value) => (int)(value.m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Constructs a Fixed64 from a double-precision floating-point value.
    /// </summary>
    /// <param name="value">Double value to convert.</param>
    /// <returns>The nearest representable fixed-point value, with ties to even.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="value"/> is NaN or infinity.</exception>
    /// <exception cref="OverflowException">The rounded value is outside the Q32.32 range.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromDouble(double value)
    {
        if (double.IsNaN(value) || double.IsInfinity(value))
            throw new ArgumentOutOfRangeException(nameof(value), value, "Value must be finite.");

        double rawValue = Math.Round(value * FixedMath.ONE_L, MidpointRounding.ToEven);
        if (rawValue < FixedMath.MIN_RAW_D || rawValue >= FixedMath.MAX_RAW_EXCLUSIVE_D)
            throw new OverflowException($"{value} is outside the representable range of {nameof(Fixed64)}.");

        return FromRaw((long)rawValue);
    }

    /// <summary>
    /// Constructs a Fixed64 from a decimal value.
    /// </summary>
    /// <param name="value">Decimal value to convert.</param>
    /// <returns>The nearest representable fixed-point value, with ties to even.</returns>
    /// <exception cref="OverflowException">The rounded value is outside the Q32.32 range.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromDecimal(decimal value)
    {
        decimal rawValue = decimal.Round(value * FixedMath.ONE_L, 0, MidpointRounding.ToEven);

        if (rawValue < long.MinValue || rawValue > long.MaxValue)
            throw new OverflowException($"{value} is outside the representable range of {nameof(Fixed64)}.");

        return FromRaw((long)rawValue);
    }

    /// <summary>
    /// Creates a Fixed64 from a fractional number.
    /// </summary>
    /// <param name="numerator">The numerator of the fraction.</param>
    /// <param name="denominator">The denominator of the fraction.</param>
    /// <returns>The double-precision quotient rounded to the nearest representable value, with ties to even.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The floating-point quotient is NaN or infinity.</exception>
    /// <exception cref="OverflowException">The finite quotient rounds outside the Q32.32 range.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromFraction(double numerator, double denominator) => FromDouble(numerator / denominator);

    /// <summary>Converts a raw Q32.32 payload to its numeric value.</summary>
    /// <param name="f1">The signed 64-bit raw payload.</param>
    /// <returns>The payload divided by 2³², rounded to <see cref="double"/> precision.</returns>
    public static double ToDouble(long f1) => f1 * FixedMath.SCALE_FACTOR_D;

    /// <summary>Converts a raw Q32.32 payload to its numeric value.</summary>
    /// <param name="f1">The signed 64-bit raw payload.</param>
    /// <returns>The payload divided by 2³², rounded to <see cref="float"/> precision.</returns>
    public static float ToFloat(long f1) => f1 * FixedMath.SCALE_FACTOR_F;

    /// <summary>Converts a raw Q32.32 payload to its numeric value.</summary>
    /// <param name="f1">The signed 64-bit raw payload.</param>
    /// <returns>The payload divided by 2³², rounded to <see cref="decimal"/> precision.</returns>
    public static decimal ToDecimal(long f1) => f1 * FixedMath.SCALE_FACTOR_M;

    #endregion
}
