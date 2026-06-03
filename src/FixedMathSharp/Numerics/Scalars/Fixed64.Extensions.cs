//=======================================================================
// Fixed64.Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for the Fixed64 type, enabling additional mathematical, conversion, and comparison operations using a fluent syntax.
/// </summary>
public static class Fixed64Extensions
{
    #region Fixed64 Operations

    /// <inheritdoc cref="Fixed64.Sign(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Sign(this Fixed64 value) => Fixed64.Sign(value);

    /// <inheritdoc cref="Fixed64.IsInteger(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsInteger(this Fixed64 value) => Fixed64.IsInteger(value);

    /// <inheritdoc cref="FixedMath.Squared(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Squared(this Fixed64 value) => FixedMath.Squared(value);

    /// <inheritdoc cref="FixedMath.Round(Fixed64, MidpointRounding)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Round(this Fixed64 value, MidpointRounding mode = MidpointRounding.ToEven) =>
        FixedMath.Round(value, mode);

    /// <inheritdoc cref="FixedMath.RoundToPrecision(Fixed64, int, MidpointRounding)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 RoundToPrecision(this Fixed64 value, int places, MidpointRounding mode = MidpointRounding.ToEven) =>
        FixedMath.RoundToPrecision(value, places, mode);

    /// <inheritdoc cref="FixedMath.Clamp(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Clamp(this Fixed64 value, Fixed64 min, Fixed64 max) => FixedMath.Clamp(value, min, max);

    /// <inheritdoc cref="FixedMath.ClampOne(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 ClampOne(this Fixed64 f1) => FixedMath.ClampOne(f1);

    /// <inheritdoc cref="FixedMath.Clamp01(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Clamp01(this Fixed64 f1) => FixedMath.Clamp01(f1);

    /// <inheritdoc cref="FixedMath.Abs(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Abs(this Fixed64 value) => FixedMath.Abs(value);

    /// <summary>
    /// Checks if the absolute value of x is less than y.
    /// </summary>
    /// <param name="x">The value to compare.</param>
    /// <param name="y">The comparison threshold.</param>
    /// <returns>True if |x| &lt; y; otherwise false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool AbsLessThan(this Fixed64 x, Fixed64 y) => Abs(x) < y;

    /// <inheritdoc cref="FixedMath.FastAdd(Fixed64, Fixed64)" />
    public static Fixed64 FastAdd(this Fixed64 a, Fixed64 b) => FixedMath.FastAdd(a, b);

    /// <inheritdoc cref="FixedMath.FastSub(Fixed64, Fixed64)" />
    public static Fixed64 FastSub(this Fixed64 a, Fixed64 b) => FixedMath.FastSub(a, b);

    /// <inheritdoc cref="FixedMath.FastMul(Fixed64, Fixed64)" />
    public static Fixed64 FastMul(this Fixed64 a, Fixed64 b) => FixedMath.FastMul(a, b);

    /// <inheritdoc cref="FixedMath.FastMod(Fixed64, Fixed64)" />
    public static Fixed64 FastMod(this Fixed64 a, Fixed64 b) => FixedMath.FastMod(a, b);

    /// <inheritdoc cref="FixedMath.Lerp(Fixed64, Fixed64, Fixed64)" />
    public static Fixed64 Lerp(this Fixed64 a, Fixed64 b, Fixed64 t) => Fixed64.Lerp(a, b, t);

    /// <inheritdoc cref="FixedMath.Floor(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Floor(this Fixed64 value) => FixedMath.Floor(value);

    /// <inheritdoc cref="FixedMath.Ceil(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Ceil(this Fixed64 value) => FixedMath.Ceil(value);

    /// <summary>
    /// Rounds the Fixed64 value to the nearest integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int RoundToInt(this Fixed64 x) => (int)FixedMath.Round(x);

    /// <summary>
    /// Rounds the Fixed64 value to the nearest long.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static long RoundToLong(this Fixed64 x) => (long)FixedMath.Round(x);

    /// <summary>
    /// Rounds up the Fixed64 value to the nearest integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CeilToInt(this Fixed64 x) =>
        (int)(FixedMath.Ceil(x).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Rounds up the Fixed64 value to the nearest long.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static long CeilToLong(this Fixed64 x) =>
        (long)(FixedMath.Ceil(x).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Rounds down the Fixed64 value to the nearest integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int FloorToInt(this Fixed64 x) =>
        (int)(FixedMath.Floor(x).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Rounds down the Fixed64 value to the nearest long.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static long FloorToLong(this Fixed64 x) =>
        (long)(FixedMath.Floor(x).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    #endregion

    #region Fixed64 Trigonometry

    /// <inheritdoc cref="FixedMath.Sqrt(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Sqrt(this Fixed64 f) => FixedMath.Sqrt(f);

    /// <inheritdoc cref="FixedMath.Acos(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Acos(this Fixed64 f) => FixedMath.Acos(f);

    /// <inheritdoc cref="FixedMath.Atan2(Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Atan2(this Fixed64 a, Fixed64 b) => FixedMath.Atan2(a, b);

    /// <inheritdoc cref="FixedMath.Sin(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Sin(Fixed64 f) => FixedMath.Sin(f);

    /// <inheritdoc cref="FixedMath.Cos(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Cos(this Fixed64 f) => FixedMath.Cos(f);

    /// <inheritdoc cref="FixedMath.GetHypotenuse(Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Hypot(this Fixed64 a, Fixed64 b) => FixedMath.GetHypotenuse(a, b);

    #endregion

    #region Conversion

    /// <summary>
    /// Converts the Fixed64 value to a string formatted to 2 decimal places.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static string ToFormattedString(this Fixed64 f1) => f1.ToPreciseFloat().ToString("0.##");

    /// <summary>
    /// Converts the Fixed64 value to a double with specified decimal precision.
    /// </summary>
    /// <param name="f1">The Fixed64 value to convert.</param>
    /// <param name="precision">The number of decimal places to round to.</param>
    /// <returns>The formatted double value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double ToFormattedDouble(this Fixed64 f1, int precision = 2) =>
        Math.Round((double)f1, precision, MidpointRounding.AwayFromZero);

    /// <summary>
    /// Converts the Fixed64 value to a float with 2 decimal points of precision.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float ToFormattedFloat(this Fixed64 f1) => (float)ToFormattedDouble(f1);

    /// <summary>
    /// Converts the Fixed64 value to a precise float representation (without rounding).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float ToPreciseFloat(this Fixed64 f1) => (float)(double)f1;

    /// <summary>
    /// Converts the angle in degrees to radians.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 ToRadians(this Fixed64 angleInDegrees) => FixedMath.DegToRad(angleInDegrees);

    /// <summary>
    /// Converts the angle in radians to degree.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 ToDegree(this Fixed64 angleInRadians) => FixedMath.RadToDeg(angleInRadians);

    #endregion

    #region Equality

    /// <summary>
    /// Checks if the value is greater than epsilon (positive or negative).
    /// Useful for determining if a value is effectively non-zero with a given precision.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool MoreThanEpsilon(this Fixed64 d) => d.Abs() > Fixed64.Epsilon;

    /// <summary>
    /// Checks if the value is less than epsilon (i.e., effectively zero).
    /// Useful for determining if a value is close enough to zero with a given precision.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool LessThanEpsilon(this Fixed64 d) => (d.Abs() < Fixed64.Epsilon);

    /// <summary>
    /// Helper method to compare individual vector components for approximate equality, allowing a fractional difference.
    /// Handles zero components by only using the allowed percentage difference.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyComponentEqual(this Fixed64 a, Fixed64 b, Fixed64 percentage)
    {
        var diff = (a - b).Abs();
        var allowedErr = a.Abs() * percentage;
        // Compare directly to percentage if a is zero
        // Otherwise, use percentage of a's magnitude
        return a.LessThanEpsilon() ? diff <= percentage : diff <= allowedErr;
    }

    #endregion
}