using MemoryPack;
using System;

#if NET8_0_OR_GREATER
using System.Text.Json.Serialization;
#endif
#if !NET8_0_OR_GREATER
using System.Text.Json.Serialization.Shim;
#endif

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
    public FixedCurveKey(double time, double value)
        : this(new Fixed64(time), new Fixed64(value)) { }

    /// <summary>
    /// Creates a keyframe with optional tangents for cubic interpolation.
    /// </summary>
    public FixedCurveKey(double time, double value, double inTangent, double outTangent)
        : this(new Fixed64(time), new Fixed64(value), new Fixed64(inTangent), new Fixed64(outTangent)) { }

    /// <summary>
    /// Creates a keyframe with a specified time and value.
    /// </summary>
    public FixedCurveKey(Fixed64 time, Fixed64 value)
        : this(time, value, Fixed64.Zero, Fixed64.Zero) { }

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

    #endregion

    #region Equality

    public bool Equals(FixedCurveKey other)
    {
        return Time == other.Time &&
               Value == other.Value &&
               InTangent == other.InTangent &&
               OutTangent == other.OutTangent;
    }

    public override bool Equals(object? obj) => obj is FixedCurveKey other && Equals(other);

    public override int GetHashCode()
    {
        return HashCode.Combine(Time, Value, InTangent, OutTangent);
    }

    public static bool operator ==(FixedCurveKey left, FixedCurveKey right) => left.Equals(right);

    public static bool operator !=(FixedCurveKey left, FixedCurveKey right) => !(left == right);

    #endregion
}