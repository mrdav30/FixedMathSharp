//=======================================================================
// Axis3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// Identifies a signed axis in a 3D coordinate convention.
/// </summary>
/// <remarks>
/// FixedMathSharp's canonical 3D convention is <see cref="PositiveX"/> right,
/// <see cref="PositiveY"/> up, and <see cref="PositiveZ"/> forward. Use signed
/// axes to describe adapter-boundary conventions without changing core math
/// semantics.
/// </remarks>
public enum Axis3d
{
    /// <summary>
    /// Positive X.
    /// </summary>
    PositiveX = 0,

    /// <summary>
    /// Negative X.
    /// </summary>
    NegativeX = 1,

    /// <summary>
    /// Positive Y.
    /// </summary>
    PositiveY = 2,

    /// <summary>
    /// Negative Y.
    /// </summary>
    NegativeY = 3,

    /// <summary>
    /// Positive Z.
    /// </summary>
    PositiveZ = 4,

    /// <summary>
    /// Negative Z.
    /// </summary>
    NegativeZ = 5,
}
