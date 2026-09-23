using System;
using Chronicler.Timing;

namespace FixedMathSharp.Chronicler;

/// <summary>
/// Exact conversions between Fixed64 seconds and Chronicler durations.
/// Subtract wide timestamps before narrowing their difference to Fixed64.
/// </summary>
public static class FixedChronicleTime
{
    /// <summary>Widens Fixed64 seconds without rounding or losing fractional bits.</summary>
    public static ChronicleDuration FromFixed64(Fixed64 value)
    {
        long raw = value.m_rawValue;
        return new ChronicleDuration(raw >> 32, unchecked((uint)raw));
    }

    /// <summary>
    /// Converts a representable duration exactly; otherwise returns false and a zero result.
    /// </summary>
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
    /// <exception cref="OverflowException">The duration is outside the Fixed64 range.</exception>
    public static Fixed64 ToFixed64(ChronicleDuration value)
    {
        if (!TryToFixed64(value, out Fixed64 result))
            throw new OverflowException("The duration is outside the Fixed64 range.");

        return result;
    }

    /// <summary>
    /// Counts complete steps in a nonnegative duration using exact raw integer division.
    /// This floors the result; it is neither a ceiling-based wait nor a historical frame lookup.
    /// </summary>
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
