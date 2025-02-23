using System;
using System.Linq;
using System.Runtime.Serialization;


#if NET8_0_OR_GREATER
using System.Text.Json.Serialization;
using System.Xml.Linq;
#endif

namespace FixedMathSharp
{
    /// <summary>
    /// Specifies the interpolation method used when evaluating a <see cref="FixedCurve"/>.
    /// </summary>
    public enum FixedCurveMode
    {
        /// <summary>Linear interpolation between keyframes.</summary>
        Linear,

        /// <summary>Step interpolation, instantly jumping between keyframe values.</summary>
        Step,

        /// <summary>Smooth interpolation using a cosine function (SmoothStep).</summary>
        Smooth,

        /// <summary>Cubic interpolation for smoother curves using tangents.</summary>
        Cubic
    }

    /// <summary>
    /// Represents a keyframe in a <see cref="FixedCurve"/>, defining a value at a specific time.
    /// </summary>
    [Serializable]
    public struct FixedCurveKey : IEquatable<FixedCurveKey>
    {
        /// <summary>The time at which this keyframe occurs.</summary>
        public Fixed64 Time;

        /// <summary>The value of the curve at this keyframe.</summary>
        public Fixed64 Value;

        /// <summary>The incoming tangent for cubic interpolation.</summary>
        public Fixed64 InTangent;

        /// <summary>The outgoing tangent for cubic interpolation.</summary>
        public Fixed64 OutTangent;

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
#if NET8_0_OR_GREATER
        [JsonConstructor]
#endif
        public FixedCurveKey(Fixed64 time, Fixed64 value, Fixed64 inTangent, Fixed64 outTangent)
        {
            Time = time;
            Value = value;
            InTangent = inTangent;
            OutTangent = outTangent;
        }

        public bool Equals(FixedCurveKey other)
        {
            return Time == other.Time &&
                   Value == other.Value &&
                   InTangent == other.InTangent &&
                   OutTangent == other.OutTangent;
        }

        public override bool Equals(object obj) => obj is FixedCurveKey other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = Time.GetHashCode();
                hash = (hash * 31) ^ Value.GetHashCode();
                hash = (hash * 31) ^ InTangent.GetHashCode();
                hash = (hash * 31) ^ OutTangent.GetHashCode();
                return hash;
            }
        }

        public static bool operator ==(FixedCurveKey left, FixedCurveKey right) => left.Equals(right);

        public static bool operator !=(FixedCurveKey left, FixedCurveKey right) => !(left == right);
    }

    /// <summary>
    /// A deterministic fixed-point curve that interpolates values between keyframes.
    /// Used for animations, physics calculations, and procedural data.
    /// </summary>
    [Serializable]
    public class FixedCurve : IEquatable<FixedCurve>
    {
        public FixedCurveKey[] Keyframes { get; private set; }

        public FixedCurveMode Mode { get; private set; }

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
#if NET8_0_OR_GREATER
        [JsonConstructor]
#endif
        public FixedCurve(FixedCurveMode mode, params FixedCurveKey[] keyframes)
        {
            Keyframes = keyframes.OrderBy(k => k.Time).ToArray();
            Mode = mode;
        }

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
            if (time >= Keyframes[Keyframes.Length - 1].Time) return Keyframes[Keyframes.Length - 1].Value;

            // Find the surrounding keyframes
            for (int i = 0; i < Keyframes.Length - 1; i++)
            {
                if (time >= Keyframes[i].Time && time < Keyframes[i + 1].Time)
                {
                    // Compute interpolation factor
                    Fixed64 t = (time - Keyframes[i].Time) / (Keyframes[i + 1].Time - Keyframes[i].Time);

                    // Choose interpolation method
                    return Mode switch
                    {
                        FixedCurveMode.Step => Keyframes[i].Value,// Immediate transition
                        FixedCurveMode.Smooth => FixedMath.SmoothStep(Keyframes[i].Value, Keyframes[i + 1].Value, t),
                        FixedCurveMode.Cubic => FixedMath.CubicInterpolate(
                                                        Keyframes[i].Value, Keyframes[i + 1].Value,
                                                        Keyframes[i].OutTangent, Keyframes[i + 1].InTangent, t),
                        _ => FixedMath.LinearInterpolate(Keyframes[i].Value, Keyframes[i + 1].Value, t),
                    };
                }
            }

            return Fixed64.One; // Fallback (should never be hit)
        }

        public bool Equals(FixedCurve other)
        {
            if (other is null) return false;
            if (ReferenceEquals(this, other)) return true;
            return Mode == other.Mode && Keyframes.SequenceEqual(other.Keyframes);
        }

        public override bool Equals(object obj) => obj is FixedCurve other && Equals(other);

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

        public static bool operator ==(FixedCurve left, FixedCurve right) => left?.Equals(right) ?? right is null;

        public static bool operator !=(FixedCurve left, FixedCurve right) => !(left == right);
    }
}