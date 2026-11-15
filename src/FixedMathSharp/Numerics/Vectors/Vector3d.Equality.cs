//=======================================================================
// Vector3d.Equality.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Equality, hashing, and comparison logic for <see cref="Vector3d"/>.
/// </content>
public partial struct Vector3d
{
    /// <summary>Compares vectors by corresponding components.</summary>
    /// <param name="obj">The object to compare.</param>
    /// <returns><see langword="true"/> if the object is a <see cref="Vector3d"/> with equal corresponding components, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Vector3d other && Equals(other);

    /// <summary>Compares vectors by corresponding components.</summary>
    /// <param name="other">The vector to compare.</param>
    /// <returns><see langword="true"/> if all corresponding components are exactly equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Vector3d other) => other.X == X && other.Y == Y && other.Z == Z;

    /// <summary>Compares vectors by corresponding components.</summary>
    /// <param name="x">The first vector.</param>
    /// <param name="y">The second vector.</param>
    /// <returns><see langword="true"/> if all corresponding components are exactly equal, otherwise <see langword="false"/>.</returns>
    public bool Equals(Vector3d x, Vector3d y) => x.Equals(y);

    /// <summary>Gets a hash code based on the raw components.</summary>
    /// <returns>The value of <see cref="StateHash"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => StateHash;

    /// <summary>Gets a hash code based on the raw components.</summary>
    /// <param name="obj">The vector to hash.</param>
    /// <returns>The supplied vector’s <see cref="StateHash"/>.</returns>
    public int GetHashCode(Vector3d obj) => obj.GetHashCode();

    /// <summary>
    /// Compares the current Vector3d instance with another Vector3d based on their squared magnitudes.
    /// </summary>
    /// <param name="other">The vector whose squared length is compared.</param>
    /// <returns>A negative value, zero, or a positive value when this rounded and saturated squared length compares less than, equal to, or greater than the other squared length.</returns>
    /// <remarks>Use <see cref="CompareMagnitudeSquared(Vector3d, Vector3d)"/> for an exact length comparison.</remarks>
    public int CompareTo(Vector3d other) => MagnitudeSquared.CompareTo(other.MagnitudeSquared);
}
