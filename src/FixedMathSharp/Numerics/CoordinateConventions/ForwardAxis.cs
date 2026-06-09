//=======================================================================
// ForwardAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// Identifies the signed Z axis used as semantic forward by a 3D coordinate convention.
/// </summary>
/// <remarks>
/// FixedMathSharp's canonical 3D convention is <see cref="ForwardAxis.PositiveZ"/> forward with <c>+X</c>
/// right and <c>+Y</c> up. Conventions that use a different up axis, a different right axis,
/// or additional matrix semantics should be handled by adapter-level basis conversions.
/// </remarks>
public enum ForwardAxis
{
    /// <summary>
    /// Positive Z is semantic forward.
    /// </summary>
    PositiveZ = 0,

    /// <summary>
    /// Negative Z is semantic forward.
    /// </summary>
    NegativeZ = 1,
}
