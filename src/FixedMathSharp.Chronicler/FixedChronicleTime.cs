//=======================================================================
// FixedChronicleTime.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using Chronicler.Timing;

namespace FixedMathSharp.Chronicler;

/// <summary>Converts between fixed-point seconds and Chronicler durations and counts complete simulation steps.</summary>
/// <remarks>
/// Both representations use 32 fractional bits. For long-running timelines, subtract
/// timestamps in Chronicler's wide representation before converting their difference.
/// </remarks>
public static class FixedChronicleTime
{
    /// <summary>Widens Fixed64 seconds without rounding or losing fractional bits.</summary>
    /// <param name="value">A signed duration in seconds. The full <see cref="Fixed64"/> range is accepted.</param>
    /// <returns>The exact Chronicler duration.</returns>
    public static ChronicleDuration FromFixed64(Fixed64 value)
    {
        long raw = value.m_rawValue;
        return new ChronicleDuration(raw >> 32, unchecked((uint)raw));
    }

    /// <summary>Attempts to convert a Chronicler duration to fixed-point seconds.</summary>
    /// <param name="value">The signed duration to convert.</param>
    /// <param name="result">The exact duration in seconds on success, or <see cref="Fixed64.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if the duration fits in <see cref="Fixed64"/>, otherwise <see langword="false"/>.</returns>
    public static bool TryToFixed64(ChronicleDuration value, out Fixed64 result)
    {
        if (value.WholeSeconds < int.MinValue || value.WholeSeconds > int.MaxValue)
        {
            result = Fixed64.Zero;
            return false;
        }

        result = Fixed64.FromRaw(unchecked((value.WholeSeconds << 32) | value.FractionalSecond));
        return true;
    }

    /// <summary>Converts a duration to Fixed64 seconds exactly, without saturation.</summary>
    /// <param name="value">The signed duration to convert.</param>
    /// <returns>The exact duration in seconds.</returns>
    /// <exception cref="OverflowException">The duration is outside the Fixed64 range.</exception>
    public static Fixed64 ToFixed64(ChronicleDuration value)
    {
        if (!TryToFixed64(value, out Fixed64 result))
            throw new OverflowException("The duration is outside the Fixed64 range.");

        return result;
    }

    /// <summary>Counts complete fixed-duration steps that fit in the supplied duration.</summary>
    /// <param name="duration">The total duration in seconds, from zero through <see cref="Fixed64.MaxValue"/>.</param>
    /// <param name="stepDuration">The duration in seconds of one step. This must be positive.</param>
    /// <returns>The number of complete steps, rounded down.</returns>
    /// <remarks>The quotient uses the raw fixed-point values, including any rounding already present in the inputs.</remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// The duration is negative or the step duration is not positive.
    /// </exception>
    public static long GetFrameCountForDuration(Fixed64 duration, Fixed64 stepDuration)
    {
        if (duration.m_rawValue < 0)
            throw new ArgumentOutOfRangeException(nameof(duration), "Duration must be nonnegative.");
        if (stepDuration.m_rawValue <= 0)
            throw new ArgumentOutOfRangeException(nameof(stepDuration), "Step duration must be positive.");

        return duration.m_rawValue / stepDuration.m_rawValue;
    }
}
