//=======================================================================
// FixedBoundArea.FiniteAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains methods for creating a representable-domain intersection of a centered capsule's tight axis-aligned bounds 
/// from its center-axis length and direction or rotation.
/// </content>
public partial struct FixedBoundArea
{
    /// <summary>
    /// Creates the representable-domain intersection of a centered capsule's
    /// tight axis-aligned bounds from its scalar frame rotation.
    /// </summary>
    /// <remarks>
    /// The capsule's local positive Y axis is its center axis. Sine and cosine
    /// remain authoritative through the exact finite-axis bound calculation;
    /// no rounded normalized world axis is fed back into the geometry.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="fullAxisLength"/> or <paramref name="radius"/> is negative.
    /// </exception>
    public static FixedBoundArea FromCenteredRotatedCapsuleClippedToDomain(
        Vector2d center,
        Fixed64 rotation,
        Fixed64 fullAxisLength,
        Fixed64 radius)
    {
        if (fullAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(fullAxisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        Fixed64 axisX = -FixedMath.Sin(rotation);
        Fixed64 axisY = FixedMath.Cos(rotation);
        return FromMinMax(
            new Vector2d(
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.X,
                    axisX,
                    fullAxisLength,
                    radius,
                    minimum: true),
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.Y,
                    axisY,
                    fullAxisLength,
                    radius,
                    minimum: true)),
            new Vector2d(
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.X,
                    axisX,
                    fullAxisLength,
                    radius,
                    minimum: false),
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.Y,
                    axisY,
                    fullAxisLength,
                    radius,
                    minimum: false)));
    }

    /// <summary>
    /// Creates the representable-domain intersection of a centered capsule's
    /// tight axis-aligned bounds from its full center-axis length.
    /// </summary>
    /// <param name="center">The center of the capsule's axis segment.</param>
    /// <param name="axisDirection">The normalized center-axis direction.</param>
    /// <param name="fullAxisLength">The nonnegative full center-axis length.</param>
    /// <param name="radius">The nonnegative capsule radius.</param>
    /// <remarks>
    /// Each exact endpoint is clipped to the scalar domain and rounded outward.
    /// Zero length and zero radius are valid degenerate capsules.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="fullAxisLength"/> or <paramref name="radius"/> is negative.
    /// </exception>
    public static FixedBoundArea FromCenteredCapsuleClippedToDomain(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 fullAxisLength,
        Fixed64 radius)
    {
        if (!axisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Finite-axis direction must be normalized.",
                nameof(axisDirection));
        }
        if (fullAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(fullAxisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return FromMinMax(
            new Vector2d(
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.X,
                    axisDirection.X,
                    fullAxisLength,
                    radius,
                    minimum: true),
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.Y,
                    axisDirection.Y,
                    fullAxisLength,
                    radius,
                    minimum: true)),
            new Vector2d(
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.X,
                    axisDirection.X,
                    fullAxisLength,
                    radius,
                    minimum: false),
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.Y,
                    axisDirection.Y,
                    fullAxisLength,
                    radius,
                    minimum: false)));
    }
}
