//=======================================================================
// ContainmentType.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <summary>
/// Describes whether one volume contains, intersects, or is disjoint from another volume.
/// </summary>
public enum FixedEnclosureType
{
    /// <summary>
    /// The volumes do not overlap.
    /// </summary>
    Disjoint,

    /// <summary>
    /// One volume completely contains the tested volume.
    /// </summary>
    Contains,

    /// <summary>
    /// The volumes overlap without complete containment.
    /// </summary>
    Intersects
}
