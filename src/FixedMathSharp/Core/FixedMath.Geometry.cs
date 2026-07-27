//=======================================================================
// FixedMath.Geometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Geometry helpers for barycentric interpolation and sphere/slab cross-section calculations.
/// </content>
public static partial class FixedMath
{
    /// <summary>
    /// Performs barycentric interpolation between three scalar coordinates from a triangle.
    /// </summary>
    /// <param name="coordA">The coordinate of the first vertex.</param>
    /// <param name="coordB">The coordinate of the second vertex.</param>
    /// <param name="coordC">The coordinate of the third vertex.</param>
    /// <param name="weightB">The barycentric weight for the second vertex.</param>
    /// <param name="weightC">The barycentric weight for the third vertex.</param>
    /// <returns>The interpolated scalar coordinate.</returns>
    /// <remarks>
    /// Endpoint differences, both weighted terms, and the base coordinate
    /// are accumulated before one final round-half-to-even conversion.
    /// Results outside the <see cref="Fixed64"/> range saturate.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 BarycentricCoordinate(
        Fixed64 coordA,
        Fixed64 coordB,
        Fixed64 coordC,
        Fixed64 weightB,
        Fixed64 weightC
    ) => Fixed64.BarycentricCoordinateFullDomain(coordA, coordB, coordC, weightB, weightC);

    /// <summary>
    /// Returns the second-order scalar product sum for three barycentric vertices.
    /// </summary>
    /// <remarks>
    /// Computes <c>a * a + b * b + c * c + a * b + a * c + b * c</c>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 SumSquaredBarycentricProducts(Fixed64 a, Fixed64 b, Fixed64 c) =>
        (a * a) + (b * b) + (c * c) + (a * b) + (a * c) + (b * c);

    /// <summary>
    /// Returns the cross scalar product sum for two sets of three barycentric vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 SumBarycentricProducts(
        Fixed64 firstA,
        Fixed64 firstB,
        Fixed64 firstC,
        Fixed64 secondA,
        Fixed64 secondB,
        Fixed64 secondC)
    {
        Fixed64 firstSum = firstA + firstB + firstC;
        Fixed64 secondSum = secondA + secondB + secondC;
        Fixed64 matchingProducts = (firstA * secondA) + (firstB * secondB) + (firstC * secondC);
        return firstSum * secondSum + matchingProducts;
    }

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

    /// <summary>
    /// Attempts to get the largest circular sphere cross-section that lies
    /// within a centered finite slab.
    /// </summary>
    /// <param name="sphereCenter">The sphere-center coordinate on the slab axis.</param>
    /// <param name="sphereRadius">The non-negative sphere radius.</param>
    /// <param name="slabCenter">The slab-center coordinate on the same axis.</param>
    /// <param name="slabHalfThickness">The non-negative slab half-thickness.</param>
    /// <param name="crossSectionRadius">
    /// The nearest-even radius at the slab plane closest to the sphere center,
    /// or zero when the slab and sphere do not intersect.
    /// </param>
    /// <returns><see langword="true"/> when the slab intersects or is tangent to the sphere; otherwise, <see langword="false"/>.</returns>
    /// <remarks>
    /// Center separation, slab projection, the difference of squares, and the
    /// square root remain exact until the final deterministic
    /// <see cref="Fixed64"/> conversion.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="sphereRadius"/> or <paramref name="slabHalfThickness"/> is negative.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryGetSphereSlabCrossSectionRadius(
        Fixed64 sphereCenter,
        Fixed64 sphereRadius,
        Fixed64 slabCenter,
        Fixed64 slabHalfThickness,
        out Fixed64 crossSectionRadius)
    {
        if (sphereRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphereRadius), "Sphere radius must be non-negative.");
        if (slabHalfThickness < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness), "Slab half-thickness must be non-negative.");

        return WideRadialGeometry.TryGetSphereSlabCrossSectionRadius(
            sphereCenter,
            sphereRadius,
            slabCenter,
            slabHalfThickness,
            out crossSectionRadius);
    }
}
