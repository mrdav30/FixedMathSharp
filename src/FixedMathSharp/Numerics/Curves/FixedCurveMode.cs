//=======================================================================
// FixedCurveMode.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// Specifies the interpolation method used when evaluating a <see cref="FixedCurve"/>.
/// </summary>
public enum FixedCurveMode : byte
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
