//=======================================================================
// Vector2d.Equality.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Vector2d
{
    #region Equality, HashCode, and Comparable Overrides

    /// <summary>
    /// Are all components of this vector equal to zero?
    /// </summary>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool EqualsZero() => Equals(Zero);

    /// <summary>
    /// Determines whether the current value is not equal to zero.
    /// </summary>
    /// <returns>true if the value is not zero; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool NotZero() => !EqualsZero();

    /// <summary>
    /// Checks whether all components are strictly greater than <see cref="Fixed64.Epsilon"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllComponentsGreaterThanEpsilon()
    {
        return X.Abs() > Fixed64.Epsilon && Y.Abs() > Fixed64.Epsilon;
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Vector2d other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Vector2d other) => other.X == X && other.Y == Y;

    /// <inheritdoc/>
    public bool Equals(Vector2d x, Vector2d y) => x.Equals(y);

    /// <summary>
    /// Returns a hash code for this instance, which is based on the combined hash codes of the X and Y components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => StateHash;

    /// <inheritdoc/>
    public int GetHashCode(Vector2d obj) => obj.GetHashCode();

    /// <summary>
    /// Compares the current Vector2d instance with another Vector2d based on their squared magnitudes.
    /// </summary>
    /// <remarks>
    /// This comparison uses the squared magnitude of each vector, which avoids the computational
    /// cost of calculating the actual magnitude. 
    /// Use this method when only relative vector lengths are
    /// important.
    /// </remarks>
    /// <param name="other">The Vector2d instance to compare with the current instance.</param>
    /// <returns>A value less than zero if this instance is less than <paramref name="other"/>; zero if this instance is equal to
    /// <paramref name="other"/>; or a value greater than zero if this instance is greater than <paramref
    /// name="other"/>, as determined by their squared magnitudes.</returns>
    public int CompareTo(Vector2d other) => MagnitudeSquared.CompareTo(other.MagnitudeSquared);

    #endregion
}
