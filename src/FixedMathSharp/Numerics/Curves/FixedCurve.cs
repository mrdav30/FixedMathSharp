//=======================================================================
// FixedCurve.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp;

/// <summary>
/// Interpolates fixed-point values between time-ordered keyframes.
/// </summary>
/// <remarks>
/// Both <see langword="default"/> and <c>new FixedCurve()</c> leave the keyframe array null.
/// Pass a mode or a keyframe array to construct an initialized curve.
/// Struct copies share the keyframe array. Preserve ascending time order when editing it.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct FixedCurve : IEquatable<FixedCurve>
{
    private static readonly Comparison<FixedCurveKey> CompareKeyframesByTime = (left, right) => left.Time.CompareTo(right.Time);

    #region Constructors

    /// <summary>
    /// Initializes a new instance of the <see cref="FixedCurve"/> with a default linear interpolation mode.
    /// </summary>
    /// <param name="keyframes">The array of keyframes. The constructor sorts it in place by ascending time and retains it. Null selects an empty array.</param>
    /// <remarks>Use distinct times when the order of equal-time keys matters. The sort is unstable.</remarks>
    public FixedCurve(params FixedCurveKey[] keyframes)
        : this(FixedCurveMode.Linear, keyframes) { }

    /// <summary>
    /// Initializes a new instance of the <see cref="FixedCurve"/> with a specified interpolation mode.
    /// </summary>
    /// <param name="mode">The interpolation rule. An unrecognized value uses linear interpolation.</param>
    /// <param name="keyframes">The array of keyframes. The constructor sorts it in place by ascending time and retains it. Null selects an empty array.</param>
    /// <remarks>Use distinct times when the order of equal-time keys matters. The sort is unstable.</remarks>
    [JsonConstructor]
    [MemoryPackConstructor]
    public FixedCurve(FixedCurveMode mode, params FixedCurveKey[] keyframes)
    {
        if (keyframes is null)
            Keyframes = Array.Empty<FixedCurveKey>();
        else
        {
            if (keyframes.Length > 1)
                Array.Sort(keyframes, CompareKeyframesByTime);

            Keyframes = keyframes;
        }

        Mode = mode;
    }

    #endregion

    #region Properties

    /// <summary>
    /// Gets the mode used for the fixed curve calculation.
    /// </summary>
    /// <value>The interpolation rule between successive keys. Unrecognized values are evaluated as linear interpolation.</value>
    [JsonInclude]
    [MemoryPackOrder(0)]
    [MemoryPackAllowSerialize]
    public FixedCurveMode Mode { get; private set; }

    /// <summary>
    /// Gets the collection of keyframes that define the curve.
    /// </summary>
    /// <value>The mutable array retained by the constructor, ordered by ascending time. The default struct exposes a null array.</value>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public FixedCurveKey[] Keyframes { get; private set; }

    #endregion

    #region Methods

    /// <summary>
    /// Evaluates the curve at a given time using the specified interpolation mode.
    /// </summary>
    /// <param name="time">The evaluation time in the same units as the keyframe times.</param>
    /// <returns>The sampled value. Times outside the key range use the nearest endpoint value, and an empty curve returns <see cref="Fixed64.One"/>.</returns>
    /// <remarks>Step interpolation holds the current key value until the next key time. Cubic tangents are slopes with respect to the normalized segment parameter. Evaluation reads the current keyframe array.</remarks>
    /// <exception cref="NullReferenceException">This curve is the default struct and has no keyframe array.</exception>
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
            FixedCurveMode.Smooth => FixedMath.SmoothStep(current.Value, next.Value, t),
            FixedCurveMode.Cubic => FixedMath.CubicInterpolate(current.Value, next.Value, current.OutTangent, next.InTangent, t),
            _ => FixedMath.Lerp(current.Value, next.Value, t),
        };
    }

    #endregion

    #region Equality

    /// <summary>Compares this value with another instance.</summary>
    /// <param name="other">The value to compare.</param>
    /// <returns><see langword="true"/> if the stored mode and ordered sequence of keyframes match, otherwise <see langword="false"/>.</returns>
    /// <exception cref="NullReferenceException">An operand is a default curve whose keyframe array is read.</exception>
    public bool Equals(FixedCurve other)
    {
        if (Mode != other.Mode || Keyframes.Length != other.Keyframes.Length)
            return false;

        for (int i = 0; i < Keyframes.Length; i++)
        {
            if (Keyframes[i] != other.Keyframes[i])
                return false;
        }

        return true;
    }

    /// <summary>Compares this value with an object.</summary>
    /// <param name="obj">The object to compare.</param>
    /// <returns><see langword="true"/> if <paramref name="obj"/> is a <see cref="FixedCurve"/> and the stored mode and ordered sequence of keyframes match, otherwise <see langword="false"/>.</returns>
    /// <exception cref="NullReferenceException">An operand is a default curve whose keyframe array is read.</exception>
    public override bool Equals(object? obj) => obj is FixedCurve other && Equals(other);

    /// <summary>Computes the hash code for this value.</summary>
    /// <returns>A hash based on the stored mode and ordered sequence of keyframes.</returns>
    /// <exception cref="NullReferenceException">An operand is a default curve whose keyframe array is read.</exception>
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

    /// <summary>Tests two values for equality.</summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if the stored mode and ordered sequence of keyframes match, otherwise <see langword="false"/>.</returns>
    /// <exception cref="NullReferenceException">An operand is a default curve whose keyframe array is read.</exception>
    public static bool operator ==(FixedCurve left, FixedCurve right) => left.Equals(right);

    /// <summary>Tests two values for inequality.</summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if the stored modes differ or the ordered keyframe sequences differ, otherwise <see langword="false"/>.</returns>
    /// <exception cref="NullReferenceException">An operand is a default curve whose keyframe array is read.</exception>
    public static bool operator !=(FixedCurve left, FixedCurve right) => !(left == right);

    #endregion
}
