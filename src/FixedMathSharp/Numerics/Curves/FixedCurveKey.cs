//=======================================================================
// FixedCurveKey.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp;

/// <summary>
/// Represents a keyframe in a <see cref="FixedCurve"/>, defining a value at a specific time.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct FixedCurveKey : IEquatable<FixedCurveKey>
{
    #region Fields

    /// <summary>The time at which this keyframe occurs.</summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Fixed64 Time;

    /// <summary>The value of the curve at this keyframe.</summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Fixed64 Value;

    /// <summary>The incoming tangent for cubic interpolation.</summary>
    [JsonInclude]
    [MemoryPackOrder(2)]
    public Fixed64 InTangent;

    /// <summary>The outgoing tangent for cubic interpolation.</summary>
    [JsonInclude]
    [MemoryPackOrder(3)]
    public Fixed64 OutTangent;

    #endregion

    #region Constructors

    /// <summary>Creates a keyframe with zero tangents.</summary>
    /// <param name="time">The key time in the curve’s chosen time units.</param>
    /// <param name="value">The curve’s sampled value at this time.</param>
    public FixedCurveKey(Fixed64 time, Fixed64 value)
        : this(time, value, Fixed64.Zero, Fixed64.Zero) { }

    /// <summary>Creates a keyframe with zero tangents.</summary>
    /// <param name="time">The key time in the curve’s chosen time units.</param>
    /// <param name="value">The curve’s sampled value at this time.</param>
    public FixedCurveKey(int time, int value)
        : this(new(time), new(value), Fixed64.Zero, Fixed64.Zero) { }

    /// <summary>Creates a keyframe with tangents for cubic interpolation.</summary>
    /// <param name="time">The key time in the curve’s chosen time units.</param>
    /// <param name="value">The curve’s sampled value at this time.</param>
    /// <param name="inTangent">The incoming slope per unit of normalized segment parameter.</param>
    /// <param name="outTangent">The outgoing slope per unit of normalized segment parameter.</param>
    [JsonConstructor]
    public FixedCurveKey(Fixed64 time, Fixed64 value, Fixed64 inTangent, Fixed64 outTangent)
    {
        Time = time;
        Value = value;
        InTangent = inTangent;
        OutTangent = outTangent;
    }

    /// <summary>Creates a keyframe with tangents for cubic interpolation.</summary>
    /// <param name="time">The key time in the curve’s chosen time units.</param>
    /// <param name="value">The curve’s sampled value at this time.</param>
    /// <param name="inTangent">The incoming slope per unit of normalized segment parameter.</param>
    /// <param name="outTangent">The outgoing slope per unit of normalized segment parameter.</param>
    public FixedCurveKey(int time, int value, int inTangent, int outTangent)
        : this(new Fixed64(time), new Fixed64(value), new Fixed64(inTangent), new Fixed64(outTangent)) { }

    /// <summary>Creates a keyframe with zero tangents.</summary>
    /// <param name="time">The key time in the curve’s chosen time units.</param>
    /// <param name="value">The curve’s sampled value at this time.</param>
    /// <returns>The keyframe with numeric inputs rounded to Q32.32.</returns>
    /// <exception cref="ArgumentOutOfRangeException">An input is NaN or infinity.</exception>
    /// <exception cref="OverflowException">An input rounds outside the Q32.32 range.</exception>
    public static FixedCurveKey FromDouble(double time, double value) =>
        new(Fixed64.FromDouble(time), Fixed64.FromDouble(value));

    /// <summary>Creates a keyframe with tangents for cubic interpolation.</summary>
    /// <param name="time">The key time in the curve’s chosen time units.</param>
    /// <param name="value">The curve’s sampled value at this time.</param>
    /// <param name="inTangent">The incoming slope per unit of normalized segment parameter.</param>
    /// <param name="outTangent">The outgoing slope per unit of normalized segment parameter.</param>
    /// <returns>The keyframe with numeric inputs rounded to Q32.32.</returns>
    /// <exception cref="ArgumentOutOfRangeException">An input is NaN or infinity.</exception>
    /// <exception cref="OverflowException">An input rounds outside the Q32.32 range.</exception>
    public static FixedCurveKey FromDouble(double time, double value, double inTangent, double outTangent) =>
        new(Fixed64.FromDouble(time),
            Fixed64.FromDouble(value),
            Fixed64.FromDouble(inTangent),
            Fixed64.FromDouble(outTangent));

    #endregion

    #region Equality

    /// <summary>Compares this value with another instance.</summary>
    /// <param name="other">The value to compare.</param>
    /// <returns><see langword="true"/> if the time, value, and both tangents match, otherwise <see langword="false"/>.</returns>
    public bool Equals(FixedCurveKey other) =>
        Time == other.Time &&
        Value == other.Value &&
        InTangent == other.InTangent &&
        OutTangent == other.OutTangent;

    /// <summary>Compares this value with an object.</summary>
    /// <param name="obj">The object to compare.</param>
    /// <returns><see langword="true"/> if <paramref name="obj"/> is a <see cref="FixedCurveKey"/> and the time, value, and both tangents match, otherwise <see langword="false"/>.</returns>
    public override bool Equals(object? obj) => obj is FixedCurveKey other && Equals(other);

    /// <summary>Computes the hash code for this value.</summary>
    /// <returns>A hash combining the time, value, incoming tangent, and outgoing tangent in that order.</returns>
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + Time.GetHashCode();
            hash = (hash * 31) + Value.GetHashCode();
            hash = (hash * 31) + InTangent.GetHashCode();
            hash = (hash * 31) + OutTangent.GetHashCode();
            return hash;
        }
    }

    /// <summary>Tests two values for equality.</summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if the time, value, and both tangents match, otherwise <see langword="false"/>.</returns>
    public static bool operator ==(FixedCurveKey left, FixedCurveKey right) => left.Equals(right);

    /// <summary>Tests two values for inequality.</summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if the time, value, or a corresponding tangent differs, otherwise <see langword="false"/>.</returns>
    public static bool operator !=(FixedCurveKey left, FixedCurveKey right) => !(left == right);

    #endregion
}
