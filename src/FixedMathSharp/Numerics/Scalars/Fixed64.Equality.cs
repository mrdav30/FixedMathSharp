//=======================================================================
// Fixed64.Equality.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public readonly partial struct Fixed64
{
    #region Equality, HashCode, Comparable Overrides

    /// <summary>
    /// Determines whether this instance equals another object.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Fixed64 other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed64 other) => m_rawValue == other.m_rawValue;

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed64 x, Fixed64 y) => x.Equals(y);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => m_rawValue.GetHashCode();

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetHashCode(Fixed64 obj) => obj.GetHashCode();

    /// <summary>
    /// Compares this instance to another 
    /// </summary>
    /// <param name="other">The Fixed64 to compare with.</param>
    /// <returns>-1 if less than, 0 if equal, 1 if greater than other.</returns>
    public int CompareTo(Fixed64 other) => m_rawValue.CompareTo(other.m_rawValue);

    #endregion
}
