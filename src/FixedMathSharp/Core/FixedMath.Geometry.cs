//=======================================================================
// FixedMath.Geometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public static partial class FixedMath
{
    /// <summary>
    /// Attempts to get the radius of a circular sphere cross-section at the
    /// specified signed distance from the sphere center.
    /// </summary>
    /// <param name="radius">The non-negative sphere radius.</param>
    /// <param name="offset">The signed distance from the sphere center to the cross-section plane.</param>
    /// <param name="crossSectionRadius">The nearest-even cross-section radius, or zero when the plane misses the sphere.</param>
    /// <returns><see langword="true"/> when the plane intersects or is tangent to the sphere; otherwise, <see langword="false"/>.</returns>
    /// <remarks>
    /// The difference of squares and square root remain exact until the final
    /// deterministic <see cref="Fixed64"/> conversion.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="radius"/> is negative.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryGetCircleCrossSectionRadius(
        Fixed64 radius,
        Fixed64 offset,
        out Fixed64 crossSectionRadius)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius), "Radius must be non-negative.");

        return WideRadialGeometry.TryGetCircleCrossSectionRadius(
            radius,
            offset,
            out crossSectionRadius);
    }
}
