//=======================================================================
// Fixed64Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
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

    /// <summary>
    /// Rounds a fixed-point number to a specific number of decimal places.
    /// </summary>
    /// <param name="value">The value to round. Use a magnitude small enough that scaling by ten to the requested power fits in <see cref="Fixed64"/>.</param>
    /// <param name="places">The number of decimal places, from zero through nine.</param>
    /// <param name="mode">The tie rule passed to <see cref="FixedMath.Round(Fixed64, MidpointRounding)"/>. The default is midpoint-to-even.</param>
    /// <returns>The result of scaling, rounding to an integer, and converting back to the original scale.</returns>
    /// <remarks>The method adds half the integer scale factor in raw units before the final fixed-point division.
    /// This adjustment can shift a decimal-grid value by a raw unit. Intermediate saturation or
    /// raw wraparound can occur near the limits of the Q32.32 range.</remarks>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="places"/> is outside the supported range.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 RoundToPrecision(this Fixed64 value, int places, MidpointRounding mode = MidpointRounding.ToEven) =>
        FixedMath.RoundToPrecision(value, places, mode);

    /// <summary>
    /// Clamps a fixed-point value between the given minimum and maximum values.
    /// </summary>
    /// <param name="value">The value to clamp.</param>
    /// <param name="min">The inclusive minimum. Use a value less than or equal to <paramref name="max"/>.</param>
    /// <param name="max">The inclusive maximum.</param>
    /// <returns>The minimum if the input is below it, the maximum if the input is above it, or the input otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Clamp(this Fixed64 value, Fixed64 min, Fixed64 max) => FixedMath.Clamp(value, min, max);

    /// <inheritdoc cref="FixedMath.ClampOne(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 ClampOne(this Fixed64 f1) => FixedMath.ClampOne(f1);

    /// <summary>
    /// Clamps value between 0 and 1 and returns value.
    /// </summary>
    /// <param name="f1">The value to clamp.</param>
    /// <returns>The value limited to the inclusive interval [0, 1].</returns>
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

    /// <summary>
    /// Adds two fixed-point numbers by adding their raw Q32.32 payloads without saturation.
    /// </summary>
    /// <param name="a">The first addend.</param>
    /// <param name="b">The second addend.</param>
    /// <returns>The raw sum interpreted as a Q32.32 value.</returns>
    /// <remarks>
    /// This is an unchecked hot-path helper. Use it only when the raw sum is known to fit in
    /// <see cref="long"/> or raw wraparound is an intentional part of the algorithm. Use
    /// <see cref="Fixed64.op_Addition(Fixed64, Fixed64)"/> for the public saturating add contract.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FastAdd(this Fixed64 a, Fixed64 b) => FixedMath.FastAdd(a, b);

    /// <summary>
    /// Subtracts two fixed-point numbers by subtracting their raw Q32.32 payloads without saturation.
    /// </summary>
    /// <param name="a">The minuend.</param>
    /// <param name="b">The subtrahend.</param>
    /// <returns>The raw difference interpreted as a Q32.32 value.</returns>
    /// <remarks>
    /// This is an unchecked hot-path helper. Use it only when the raw difference is known to fit in
    /// <see cref="long"/> or raw wraparound is an intentional part of the algorithm. Use
    /// <see cref="Fixed64.op_Subtraction(Fixed64, Fixed64)"/> for the public saturating subtract contract.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FastSub(this Fixed64 a, Fixed64 b) => FixedMath.FastSub(a, b);

    /// <summary>
    /// Multiplies two fixed-point numbers using unchecked Q32.32 partial products.
    /// </summary>
    /// <param name="a">The first factor.</param>
    /// <param name="b">The second factor.</param>
    /// <returns>The product with the low fractional bits discarded. A product outside the representable range wraps.</returns>
    /// <remarks>
    /// This is an unchecked hot-path helper. It skips the full-width overflow/saturation path used by
    /// <see cref="Fixed64.op_Multiply(Fixed64, Fixed64)"/> and truncates discarded fractional bits instead
    /// of applying the operator's round-half-to-even behavior. Use it only when inputs are constrained and
    /// that precision tradeoff is acceptable. Integral operands take a direct raw multiplication path.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FastMul(this Fixed64 a, Fixed64 b) => FixedMath.FastMul(a, b);

    /// <summary>
    /// Divides two fixed-point numbers with an optimized path for known-positive divisors.
    /// </summary>
    /// <param name="a">The dividend.</param>
    /// <param name="b">The divisor.</param>
    /// <returns>The quotient rounded to the nearest representable value with ties to even and saturated at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException"><paramref name="b"/> is zero.</exception>
    /// <remarks>
    /// This helper preserves the same deterministic rounding, divide-by-zero, and saturation semantics
    /// as <see cref="Fixed64.op_Division(Fixed64, Fixed64)"/>. The fast path is only used when
    /// <paramref name="b"/> is positive; non-positive divisors fall back to the guarded division operator.
    /// Prefer the operator unless the divisor positivity invariant is already proven by the caller.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FastDiv(this Fixed64 a, Fixed64 b) => FixedMath.FastDiv(a, b);

    /// <summary>
    /// Computes the raw remainder of two fixed-point numbers without special-case guards.
    /// </summary>
    /// <param name="a">The dividend.</param>
    /// <param name="b">The divisor.</param>
    /// <returns>The raw signed remainder interpreted as a Q32.32 value.</returns>
    /// <exception cref="DivideByZeroException"><paramref name="b"/> is zero.</exception>
    /// <exception cref="OverflowException"><paramref name="a"/> is <see cref="Fixed64.MinValue"/> and <paramref name="b"/> is negative one raw unit.</exception>
    /// <remarks>
    /// This is an unchecked hot-path helper. It delegates directly to the raw <see cref="long"/> remainder
    /// operation, so raw zero divisors and integer edge cases follow runtime integer remainder behavior.
    /// Use <see cref="Fixed64.op_Modulus(Fixed64, Fixed64)"/> for the public guarded remainder contract.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FastMod(this Fixed64 a, Fixed64 b) => FixedMath.FastMod(a, b);

    /// <summary>
    /// Linearly interpolates between two fixed-point values based on a given interpolation factor.
    /// </summary>
    /// <param name="a">The starting value.</param>
    /// <param name="b">The ending value.</param>
    /// <param name="t">A value between 0 and 1 that represents the interpolation factor.</param>
    /// <returns>The interpolated value between <paramref name="a"/> and <paramref name="b"/>.</returns>
    /// <remarks>
    /// The interpolation is clamped between <paramref name="a"/> and <paramref name="b"/> based on the value of <paramref name="t"/>.
    /// If <paramref name="t"/> is less than 0, the result is <paramref name="a"/>. If <paramref name="t"/> is greater than 1, the result is <paramref name="b"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Lerp(this Fixed64 a, Fixed64 b, Fixed64 t) => FixedMath.Lerp(a, b, t);

    /// <inheritdoc cref="FixedMath.SmoothStep(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 SmoothStep(this Fixed64 a, Fixed64 b, Fixed64 t) => FixedMath.SmoothStep(a, b, t);

    /// <inheritdoc cref="FixedMath.CubicInterpolate(Fixed64, Fixed64, Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CubicInterpolate(this Fixed64 p0, Fixed64 p1, Fixed64 m0, Fixed64 m1, Fixed64 t) =>
        FixedMath.CubicInterpolate(p0, p1, m0, m1, t);

    /// <inheritdoc cref="FixedMath.Floor(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Floor(this Fixed64 value) => FixedMath.Floor(value);

    /// <inheritdoc cref="FixedMath.Ceil(Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Ceil(this Fixed64 value) => FixedMath.Ceil(value);

    /// <summary>
    /// Rounds the Fixed64 value to the nearest integer.
    /// </summary>
    /// <param name="x">The value to round.</param>
    /// <returns>The nearest integer, with ties to even. A positive result above the representable range returns <see cref="int.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int RoundToInt(this Fixed64 x) => (int)FixedMath.Round(x);

    /// <summary>
    /// Rounds the Fixed64 value to the nearest long.
    /// </summary>
    /// <param name="x">The value to round.</param>
    /// <returns>The nearest integer, with ties to even. A positive result above the <see cref="Fixed64"/> range returns 2³¹ − 1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static long RoundToLong(this Fixed64 x) => (long)FixedMath.Round(x);

    /// <summary>
    /// Rounds up the Fixed64 value to the nearest integer.
    /// </summary>
    /// <param name="x">The value to round.</param>
    /// <returns>The ceiling, limited to <see cref="int.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CeilToInt(this Fixed64 x) =>
        (int)(FixedMath.Ceil(x).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Rounds up the Fixed64 value to the nearest long.
    /// </summary>
    /// <param name="x">The value to round.</param>
    /// <returns>The ceiling, limited to 2³¹ − 1 by the fixed-point intermediate.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static long CeilToLong(this Fixed64 x) =>
        (long)(FixedMath.Ceil(x).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Rounds down the Fixed64 value to the nearest integer.
    /// </summary>
    /// <param name="x">The value to round.</param>
    /// <returns>The greatest integer less than or equal to the input.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int FloorToInt(this Fixed64 x) =>
        (int)(FixedMath.Floor(x).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Rounds down the Fixed64 value to the nearest long.
    /// </summary>
    /// <param name="x">The value to round.</param>
    /// <returns>The greatest integer less than or equal to the input.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static long FloorToLong(this Fixed64 x) =>
        (long)(FixedMath.Floor(x).m_rawValue >> FixedMath.SHIFT_AMOUNT_I);

    #endregion

    #region Fixed64 Trigonometry

    /// <summary>
    /// Returns the square root of a specified fixed-point number.
    /// </summary>
    /// <param name="f">The nonnegative radicand.</param>
    /// <returns>The nonnegative fixed-point square root.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="f"/> is negative.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Sqrt(this Fixed64 f) => FixedMath.Sqrt(f);

    /// <summary>
    /// Raises a nonnegative fixed-point base to a fixed-point power.
    /// </summary>
    /// <param name="f">The base. A negative base is accepted only with a zero exponent.</param>
    /// <param name="exponent">The exponent.</param>
    /// <returns>
    /// One for a zero exponent, zero for a zero base with a positive exponent,
    /// and the fixed-point power otherwise. Positive bases use <see cref="FixedMath.Log2(Fixed64)"/> and <see cref="FixedMath.Pow2(Fixed64)"/>.
    /// </returns>
    /// <exception cref="DivideByZeroException"><paramref name="f"/> is zero and <paramref name="exponent"/> is negative.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="f"/> is negative and <paramref name="exponent"/> is nonzero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Pow(this Fixed64 f, Fixed64 exponent) => FixedMath.Pow(f, exponent);

    /// <summary>
    /// Raises two to a fixed-point power.
    /// </summary>
    /// <param name="f">The exponent.</param>
    /// <returns>The approximate power. Exponents at least 31 return <see cref="Fixed64.MaxValue"/> and exponents at most −32 return <see cref="Fixed64.MinIncrement"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Pow2(this Fixed64 f) => FixedMath.Pow2(f);

    /// <summary>
    /// Computes the base-2 logarithm.
    /// </summary>
    /// <param name="f">The positive value whose logarithm is computed.</param>
    /// <returns>The approximate base-2 logarithm of <paramref name="f"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="f"/> is zero or negative.</exception>
    /// <remarks>
    /// This implementation is based on Clay. S. Turner's fast binary logarithm algorithm
    /// (C. S. Turner,  "A Fast Binary Logarithm Algorithm", IEEE Signal Processing Mag., pp. 124,140, Sep. 2010.)
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Log2(this Fixed64 f) => FixedMath.Log2(f);

    /// <summary>
    /// Computes the natural logarithm.
    /// </summary>
    /// <param name="f">The positive value whose logarithm is computed.</param>
    /// <returns>The approximate natural logarithm of <paramref name="f"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="f"/> is zero or negative.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Ln(this Fixed64 f) => FixedMath.Ln(f);

    /// <summary>
    /// Computes the principal arccosine.
    /// </summary>
    /// <param name="f">The input value whose arccosine is to be computed. Should be in the range [-1, 1].</param>
    /// <returns>The arccosine in radians, in the interval [0, π].</returns>
    /// <exception cref="ArithmeticException"><paramref name="f"/> is outside [−1, 1].</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Acos(this Fixed64 f) => FixedMath.Acos(f);

    /// <summary>
    /// Computes the principal arcsine.
    /// </summary>
    /// <param name="f">The input value (sine) whose arcsine is to be computed. Should be in the range [-1, 1].</param>
    /// <returns>The arcsine in radians, in the interval [−π/2, π/2].</returns>
    /// <exception cref="ArithmeticException"><paramref name="f"/> is outside [−1, 1].</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Asin(this Fixed64 f) => FixedMath.Asin(f);

    /// <summary>
    /// Computes the principal arctangent.
    /// </summary>
    /// <param name="f">The tangent of the angle.</param>
    /// <returns>The approximate arctangent in radians, in the interval [−π/2, π/2].</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Atan(this Fixed64 f) => FixedMath.Atan(f);

    /// <summary>
    /// Computes the angle of a 2D point from the positive X axis.
    /// </summary>
    /// <remarks>
    /// Uses a fixed-point arithmetic approximation for the arc tangent function, which is more efficient than using floating-point arithmetic,
    /// especially on systems where floating-point operations are expensive.
    /// </remarks>
    /// <param name="a">The y-coordinate of the point to which the angle is measured.</param>
    /// <param name="b">The x-coordinate of the point to which the angle is measured.</param>
    /// <returns>The angle in radians in the interval [−π, π], with positive angles toward positive Y. The origin returns zero.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Atan2(this Fixed64 a, Fixed64 b) => FixedMath.Atan2(a, b);

    /// <summary>
    /// Computes the sine of a given angle in radians using complementary
    /// reduced-range polynomial approximations.
    /// </summary>
    /// <param name="f">The angle in radians.</param>
    /// <returns>The sine of the given angle, in fixed-point format.</returns>
    /// <remarks>
    /// The input is normalized to [-π, π], reflected into [0, π/2], and
    /// evaluated as sine on [0, π/4] or cosine on [0, π/4]. Exact quadrant
    /// anchors remain exact without introducing a discontinuity beside them.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Sin(this Fixed64 f) => FixedMath.Sin(f);

    /// <summary>
    /// Computes the cosine of a given angle in radians using a sine-based identity transformation.
    /// </summary>
    /// <param name="f">The angle in radians.</param>
    /// <returns>The cosine of the given angle, in fixed-point format.</returns>
    /// <remarks>
    /// - Instead of directly approximating cosine, this function derives <c>cos(x)</c> using
    ///   the identity <c>cos(x) = sin(x + π/2)</c>.
    /// - The underlying sine function uses complementary reduced-range
    ///   sine and cosine polynomials so quadrant anchors remain continuous.
    /// - The function automatically normalizes input values to the range [-π, π] for stability.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Cos(this Fixed64 f) => FixedMath.Cos(f);

    /// <summary>
    /// Computes the tangent of an angle.
    /// </summary>
    /// <param name="f">The angle in radians.</param>
    /// <returns>The ratio of the reduced-angle sine and cosine approximations, saturated when the quotient exceeds the representable range.</returns>
    /// <remarks>
    /// The angle is reduced modulo <see cref="Fixed64.Pi"/>. Results near odd multiples of π/2 are sensitive to fixed-point rounding.
    /// </remarks>
    /// <exception cref="DivideByZeroException">The reduced-angle cosine is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Tan(this Fixed64 f) => FixedMath.Tan(f);

    /// <inheritdoc cref="FixedMath.GetHypotenuse(Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Hypot(this Fixed64 a, Fixed64 b) => FixedMath.GetHypotenuse(a, b);

    #endregion

    #region Conversion

    /// <summary>
    /// Converts the angle in degrees to radians.
    /// </summary>
    /// <param name="angleInDegrees">The angle in degrees.</param>
    /// <returns>The equivalent angle in radians, saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 ToRadians(this Fixed64 angleInDegrees) => FixedMath.DegToRad(angleInDegrees);

    /// <summary>
    /// Converts the angle in radians to degrees.
    /// </summary>
    /// <param name="angleInRadians">The angle in radians.</param>
    /// <returns>The equivalent angle in degrees, saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 ToDegrees(this Fixed64 angleInRadians) => FixedMath.RadToDeg(angleInRadians);

    #endregion

    #region Equality

    /// <summary>
    /// Checks if the value is greater than epsilon (positive or negative).
    /// Useful for determining if a value is effectively non-zero with a given precision.
    /// </summary>
    /// <param name="d">The signed value to test.</param>
    /// <returns><see langword="true"/> if the magnitude is strictly greater than <see cref="Fixed64.Epsilon"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool MoreThanEpsilon(this Fixed64 d) => d.Abs() > Fixed64.Epsilon;

    /// <summary>
    /// Checks if the value is less than epsilon (i.e., effectively zero).
    /// Useful for determining if a value is close enough to zero with a given precision.
    /// </summary>
    /// <param name="d">The signed value to test.</param>
    /// <returns><see langword="true"/> if the magnitude is strictly less than <see cref="Fixed64.Epsilon"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool LessThanEpsilon(this Fixed64 d) => (d.Abs() < Fixed64.Epsilon);

    /// <summary>Tests approximate equality using the first component as the reference scale.</summary>
    /// <param name="a">The reference component.</param>
    /// <param name="b">The component to compare with the reference.</param>
    /// <param name="percentage">The allowed relative error as a fraction, such as 0.01 for 1%. Use a nonnegative value.</param>
    /// <returns><see langword="true"/> when the absolute difference is within the tolerance, including the boundary, otherwise <see langword="false"/>.</returns>
    /// <remarks>When the magnitude of <paramref name="a"/> is below <see cref="Fixed64.Epsilon"/>, the fraction is used as an absolute tolerance. Otherwise, the fraction is multiplied by that magnitude.</remarks>
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
