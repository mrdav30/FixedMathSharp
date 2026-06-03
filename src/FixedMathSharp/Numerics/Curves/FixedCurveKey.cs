//=======================================================================
// FixedCurveKey.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Text.Json.Serialization;

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

    /// <summary>
    /// Creates a keyframe with a specified time and value.
    /// </summary>
    public FixedCurveKey(Fixed64 time, Fixed64 value)
        : this(time, value, Fixed64.Zero, Fixed64.Zero) { }

    /// <summary>
    /// Creates a keyframe with a specified time and value, using integer parameters for convenience.
    /// </summary>
    public FixedCurveKey(int time, int value)
        : this(new(time), new(value), Fixed64.Zero, Fixed64.Zero) { }

    /// <summary>
    /// Creates a keyframe with optional tangents for cubic interpolation.
    /// </summary>
    [JsonConstructor]
    public FixedCurveKey(Fixed64 time, Fixed64 value, Fixed64 inTangent, Fixed64 outTangent)
    {
        Time = time;
        Value = value;
        InTangent = inTangent;
        OutTangent = outTangent;
    }

    /// <summary>
    /// Creates a keyframe with optional tangents for cubic interpolation, using integer parameters for convenience.
    /// </summary>
    public FixedCurveKey(int time, int value, int inTangent, int outTangent)
        : this(new Fixed64(time), new Fixed64(value), new Fixed64(inTangent), new Fixed64(outTangent)) { }

    /// <summary>
    /// Creates a keyframe with a specified time and value.
    /// </summary>
    public static FixedCurveKey FromFloat(double time, double value) =>
        new(Fixed64.FromFloatPoint(time), Fixed64.FromFloatPoint(value));

    /// <summary>
    /// Creates a keyframe with optional tangents for cubic interpolation.
    /// </summary>
    public static FixedCurveKey FromFloatPoint(double time, double value, double inTangent, double outTangent) =>
        new(Fixed64.FromFloatPoint(time),
            Fixed64.FromFloatPoint(value),
            Fixed64.FromFloatPoint(inTangent),
            Fixed64.FromFloatPoint(outTangent));

    #endregion

    #region Equality

    /// <inheritdoc/>
    public bool Equals(FixedCurveKey other) =>
        Time == other.Time &&
        Value == other.Value &&
        InTangent == other.InTangent &&
        OutTangent == other.OutTangent;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is FixedCurveKey other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() =>
        HashCode.Combine(Time, Value, InTangent, OutTangent);

    /// <summary>
    /// Determines whether two FixedCurveKey instances are equal.
    /// </summary>
    public static bool operator ==(FixedCurveKey left, FixedCurveKey right) => left.Equals(right);

    /// <summary>
    /// Determines whether two FixedCurveKey instances are not equal.
    /// </summary>
    public static bool operator !=(FixedCurveKey left, FixedCurveKey right) => !(left == right);

    #endregion
}