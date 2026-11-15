//=======================================================================
// Fixed64.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp;

/// <summary>
/// Represents a signed Q32.32 fixed-point number.
/// </summary>
/// <remarks>
/// The numeric value is the signed 64-bit raw payload divided by 2³². The range is −2³¹
/// through 2³¹ minus one raw unit. One raw unit is 2⁻³².
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Fixed64 : IEquatable<Fixed64>, IComparable<Fixed64>, IEqualityComparer<Fixed64>, IFormattable
#if NET10_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Fields

    /// <summary>Gets the largest representable number.</summary>
    /// <value>2³¹ minus one raw unit.</value>
    public static Fixed64 MaxValue => new(FixedMath.MAX_VALUE_L);
    /// <summary>Gets the smallest representable number.</summary>
    /// <value>−2³¹.</value>
    public static Fixed64 MinValue => new(FixedMath.MIN_VALUE_L);

    /// <summary>Gets the multiplicative identity.</summary>
    /// <value>Exactly one.</value>
    public static Fixed64 One => new(FixedMath.ONE_L);
    /// <summary>Gets negative one.</summary>
    /// <value>Exactly −1.</value>
    public static Fixed64 NegOne => -One;
    /// <summary>Gets the number two.</summary>
    /// <value>Exactly two.</value>
    public static Fixed64 Two => One * 2;
    /// <summary>Gets the number three.</summary>
    /// <value>Exactly three.</value>
    public static Fixed64 Three => One * 3;
    /// <summary>Gets one half.</summary>
    /// <value>Exactly 0.5.</value>
    public static Fixed64 Half => One / 2;
    /// <summary>Gets one quarter.</summary>
    /// <value>Exactly 0.25.</value>
    public static Fixed64 Quarter => One / 4;
    /// <summary>Gets one eighth.</summary>
    /// <value>Exactly 0.125.</value>
    public static Fixed64 Eighth => One / 8;
    /// <summary>Gets the additive identity.</summary>
    /// <value>Exactly zero.</value>
    public static Fixed64 Zero => new(0);

    /// <summary>Gets the fixed-point approximation of π.</summary>
    /// <value>The canonical angle for half a turn in radians.</value>
    public static Fixed64 Pi => new(FixedMath.PI_LONG);
    /// <summary>Gets the fixed-point approximation of 2π.</summary>
    /// <value>The canonical angle for one turn in radians.</value>
    public static Fixed64 TwoPi => Pi * Two;
    /// <summary>Gets the fixed-point approximation of π/2.</summary>
    /// <value>The canonical angle for one quarter turn in radians.</value>
    public static Fixed64 HalfPi => Pi / new Fixed64(2);
    /// <summary>Gets the fixed-point approximation of π/3.</summary>
    /// <value>An angle of 60 degrees expressed in radians.</value>
    public static Fixed64 PiOver3 => Pi / new Fixed64(3);
    /// <summary>Gets the fixed-point approximation of π/4.</summary>
    /// <value>An angle of 45 degrees expressed in radians.</value>
    public static Fixed64 PiOver4 => Pi / new Fixed64(4);
    /// <summary>Gets the fixed-point approximation of π/6.</summary>
    /// <value>An angle of 30 degrees expressed in radians.</value>
    public static Fixed64 PiOver6 => Pi / new Fixed64(6);
    /// <summary>Gets the fixed-point approximation of 1/π.</summary>
    /// <value>The fixed-point reciprocal of <see cref="Pi"/>.</value>
    public static Fixed64 InvertedPi => One / Pi;
    /// <summary>Gets the number 180.</summary>
    /// <value>The number of degrees in half a turn.</value>
    public static Fixed64 OneEighty => new(180);
    /// <summary>Gets the degrees-to-radians scale factor.</summary>
    /// <value>The rounded ratio of <see cref="Pi"/> to <see cref="OneEighty"/>.</value>
    public static Fixed64 Deg2Rad => Pi / OneEighty;
    /// <summary>Gets the radians-to-degrees scale factor.</summary>
    /// <value>The rounded ratio of <see cref="OneEighty"/> to <see cref="Pi"/>.</value>
    public static Fixed64 Rad2Deg => OneEighty / Pi;

    /// <summary>Gets the fixed-point approximation of the natural logarithm of two.</summary>
    /// <value>The fixed-point approximation of ln(2).</value>
    public static Fixed64 Ln2 => new(FixedMath.LN2_LONG);
    /// <summary>Gets the positive logarithm-related constant.</summary>
    /// <value>Exactly 63. <see cref="FixedMath.Pow2(Fixed64)"/> returns <see cref="MaxValue"/> for this exponent.</value>
    public static Fixed64 Log2Max => new(63L * FixedMath.ONE_L);
    /// <summary>Gets the negative logarithm-related constant.</summary>
    /// <value>Exactly −64. <see cref="FixedMath.Pow2(Fixed64)"/> returns <see cref="MinIncrement"/> for this exponent.</value>
    public static Fixed64 Log2Min => new(-64L * FixedMath.ONE_L);

    internal static Fixed64 PadeA1 => new(FixedMath.PADE_A1_LONG);
    internal static Fixed64 PadeA2 => new(FixedMath.PADE_A2_LONG);

    internal static Fixed64 SinCoeff3 => new(FixedMath.SIN_COEFF_3_LONG); // Tuned minimax coefficient.
    internal static Fixed64 SinCoeff5 => new(FixedMath.SIN_COEFF_5_LONG); // Tuned minimax coefficient.
    internal static Fixed64 SinCoeff7 => new(FixedMath.SIN_COEFF_7_LONG); // Tuned minimax coefficient.
    internal static Fixed64 CosCoeff2 => new(FixedMath.COS_COEFF_2_LONG); // 1/2!
    internal static Fixed64 CosCoeff4 => new(FixedMath.COS_COEFF_4_LONG); // 1/4!
    internal static Fixed64 CosCoeff6 => new(FixedMath.COS_COEFF_6_LONG); // 1/6!
    internal static Fixed64 CosCoeff8 => new(FixedMath.COS_COEFF_8_LONG); // 1/8!

    /// <summary>Gets the smallest positive value.</summary>
    /// <value>2⁻³², or one raw unit.</value>
    public static Fixed64 MinIncrement => new(FixedMath.MIN_INCREMENT_L);
    /// <summary>Gets the default tolerance for approximate comparisons.</summary>
    /// <value>2⁻²⁴, or 256 raw units.</value>
    public static Fixed64 Epsilon => new(FixedMath.DEFAULT_TOLERANCE_L);

    #endregion
    #region Fields

    /// <summary>
    /// The underlying raw long value representing the fixed-point number.
    /// </summary>
    [JsonInclude]
    [MemoryPackInclude]
    public long m_rawValue;

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
    /// <param name="value">The signed integer in ordinary numeric units.</param>
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
    /// <returns>The adjusted copy. The raw sum wraps on overflow.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Offset(int x)
    {
        return new Fixed64(m_rawValue + ((long)x << FixedMath.SHIFT_AMOUNT_I));
    }

    /// <summary>
    /// Returns the raw value as a string.
    /// </summary>
    /// <returns>The signed raw Q32.32 payload formatted as a decimal integer using the invariant culture.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToRawString() => m_rawValue.ToString(CultureInfo.InvariantCulture);

    #endregion
}
