//=======================================================================
// Fixed64.Equality.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Provides equality comparison, hash code generation, and ordering logic for <see cref="Fixed64"/>.
/// </content>
public partial struct Fixed64
{
    /// <summary>Compares this value with an object.</summary>
    /// <param name="obj">The object to compare.</param>
    /// <returns><see langword="true"/> when the object is a <see cref="Fixed64"/> with the same raw payload, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Fixed64 other && Equals(other);

    /// <summary>Compares this value with another fixed-point value.</summary>
    /// <param name="other">The value to compare.</param>
    /// <returns><see langword="true"/> if the raw payloads are equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed64 other) => m_rawValue == other.m_rawValue;

    /// <summary>Compares two fixed-point values.</summary>
    /// <param name="x">The first value.</param>
    /// <param name="y">The second value.</param>
    /// <returns><see langword="true"/> if the raw payloads are equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed64 x, Fixed64 y) => x.Equals(y);

    /// <summary>Gets the hash code of this fixed-point value.</summary>
    /// <returns>The hash code of the raw payload.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => m_rawValue.GetHashCode();

    /// <summary>Gets the hash code of a fixed-point value.</summary>
    /// <param name="obj">The value to hash.</param>
    /// <returns>The hash code of the supplied raw payload.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetHashCode(Fixed64 obj) => obj.GetHashCode();

    /// <summary>
    /// Compares this instance to another 
    /// </summary>
    /// <param name="other">The Fixed64 to compare with.</param>
    /// <returns>-1 if less than, 0 if equal, 1 if greater than other.</returns>
    public int CompareTo(Fixed64 other) => m_rawValue.CompareTo(other.m_rawValue);
}
