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
public partial struct Fixed64 : IEquatable<Fixed64>, IComparable<Fixed64>, IEqualityComparer<Fixed64>, IFormattable
#if NET8_0_OR_GREATER
    , ISpanFormattable
#endif
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
    public string ToRawString() => m_rawValue.ToString(CultureInfo.InvariantCulture);

    #endregion
}
