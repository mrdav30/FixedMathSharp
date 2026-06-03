//=======================================================================
// FixedCurve.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Linq;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// A deterministic fixed-point curve that interpolates values between keyframes.
/// Used for animations, physics calculations, and procedural data.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct FixedCurve : IEquatable<FixedCurve>
{
    #region Constructors

    /// <summary>
    /// Initializes a new instance of the <see cref="FixedCurve"/> with a default linear interpolation mode.
    /// </summary>
    /// <param name="keyframes">The keyframes defining the curve.</param>
    public FixedCurve(params FixedCurveKey[] keyframes)
        : this(FixedCurveMode.Linear, keyframes) { }

    /// <summary>
    /// Initializes a new instance of the <see cref="FixedCurve"/> with a specified interpolation mode.
    /// </summary>
    /// <param name="mode">The interpolation method to use.</param>
    /// <param name="keyframes">The keyframes defining the curve.</param>
    [JsonConstructor]
    [MemoryPackConstructor]
    public FixedCurve(FixedCurveMode mode, params FixedCurveKey[] keyframes)
    {
        Keyframes = keyframes?.Length > 1
            ? keyframes.OrderBy(k => k.Time).ToArray()
            : keyframes?.Clone() as FixedCurveKey[] ?? Array.Empty<FixedCurveKey>();
        Mode = mode;
    }

    #endregion

    #region Properties 

    /// <summary>
    /// Gets the mode used for the fixed curve calculation.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    [MemoryPackAllowSerialize]
    public FixedCurveMode Mode { get; private set; }

    /// <summary>
    /// Gets the collection of keyframes that define the curve.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public FixedCurveKey[] Keyframes { get; private set; }

    #endregion

    #region Methods

    /// <summary>
    /// Evaluates the curve at a given time using the specified interpolation mode.
    /// </summary>
    /// <param name="time">The time at which to evaluate the curve.</param>
    /// <returns>The interpolated value at the given time.</returns>
    public Fixed64 Evaluate(Fixed64 time)
    {
        if (Keyframes.Length == 0) return Fixed64.One;

        // Clamp input within the keyframe range
        if (time <= Keyframes[0].Time) return Keyframes[0].Value;
        if (time >= Keyframes[^1].Time) return Keyframes[^1].Value;

        // Find the surrounding keyframes. The constructor sorts keyframes, and the range checks above guarantee
        // that an interior time belongs to one of these segments.
        int segmentIndex = Keyframes.Length - 2;
        for (int i = 0; i < Keyframes.Length - 1; i++)
        {
            if (time < Keyframes[i + 1].Time)
            {
                segmentIndex = i;
                break;
            }
        }

        FixedCurveKey current = Keyframes[segmentIndex];
        FixedCurveKey next = Keyframes[segmentIndex + 1];
        Fixed64 t = (time - current.Time) / (next.Time - current.Time);

        return Mode switch
        {
            FixedCurveMode.Step => current.Value,
            FixedCurveMode.Smooth => Fixed64.SmoothStep(current.Value, next.Value, t),
            FixedCurveMode.Cubic => Fixed64.CubicInterpolate(current.Value, next.Value, current.OutTangent, next.InTangent, t),
            _ => Fixed64.Lerp(current.Value, next.Value, t),
        };
    }

    #endregion

    #region Equality

    /// <inheritdoc/>
    public bool Equals(FixedCurve other) =>
        Mode == other.Mode && Keyframes.SequenceEqual(other.Keyframes);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is FixedCurve other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = (int)Mode;
            foreach (var key in Keyframes)
                hash = (hash * 31) ^ key.GetHashCode();
            return hash;
        }
    }

    /// <summary>
    /// Determines whether two FixedCurve instances are equal.
    /// </summary>
    public static bool operator ==(FixedCurve left, FixedCurve right) => left.Equals(right);

    /// <summary>
    /// Determines whether two FixedCurve instances are not equal.
    /// </summary>
    public static bool operator !=(FixedCurve left, FixedCurve right) => !(left == right);

    #endregion
}
