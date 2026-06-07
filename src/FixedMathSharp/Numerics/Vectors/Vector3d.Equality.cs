//=======================================================================
// Vector3d.Equality.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Vector3d
{
    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Vector3d other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Vector3d other) => other.X == X && other.Y == Y && other.Z == Z;

    /// <inheritdoc/>
    public bool Equals(Vector3d x, Vector3d y) => x.Equals(y);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => StateHash;

    /// <inheritdoc/>
    public int GetHashCode(Vector3d obj) => obj.GetHashCode();

    /// <summary>
    /// Compares the current Vector3d instance with another Vector3d based on their squared magnitudes.
    /// </summary>
    /// <remarks>
    /// This comparison uses the squared magnitude of each vector, which avoids the computational
    /// cost of calculating the actual magnitude. 
    /// Use this method when only relative vector lengths are
    /// important.
    /// </remarks>
    /// <param name="other">The Vector3d instance to compare with the current instance.</param>
    /// <returns>A value less than zero if this instance is less than <paramref name="other"/>; zero if this instance is equal to
    /// <paramref name="other"/>; or a value greater than zero if this instance is greater than <paramref
    /// name="other"/>, as determined by their squared magnitudes.</returns>
    public int CompareTo(Vector3d other) => MagnitudeSquared.CompareTo(other.MagnitudeSquared);

    #endregion
}
