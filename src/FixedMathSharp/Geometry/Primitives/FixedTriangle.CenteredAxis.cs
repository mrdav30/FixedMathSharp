//=======================================================================
// FixedTriangle.CenteredAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides triangle-vs-centered-axis and triangle-vs-centered-capsule
/// intersection, overlap, and contact queries.
/// </content>
public partial struct FixedTriangle
{
    /// <summary>
    /// Attempts to return the closest points on this triangle and a conceptual
    /// centered finite axis without materializing its endpoints.
    /// </summary>
    public readonly bool TryGetClosestPointsToCenteredAxis(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        out Vector3d pointOnTriangle,
        out Vector3d pointOnAxis)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        return WideFiniteAxisIntersection.TryGetClosestPointsToCenteredAxis(
            this,
            center,
            axisDirection,
            axisLength,
            out pointOnTriangle,
            out pointOnAxis);
    }

    /// <summary>
    /// Returns whether this triangle inclusively overlaps a conceptual centered
    /// capsule without materializing either contact witness.
    /// </summary>
    public readonly bool DoesCenteredCapsuleOverlap(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.DoesCenteredCapsuleTriangleOverlap(
            this,
            center,
            axisDirection,
            axisLength,
            radius);
    }

    /// <summary>
    /// Attempts to create an inclusive contact between this triangle and a
    /// conceptual centered capsule.
    /// </summary>
    /// <remarks>
    /// The returned normal points from the triangle toward the capsule.
    /// <paramref name="fallbackNormal"/> is used only when the closest axis
    /// and triangle points coincide. A false result means that the contact is
    /// separated or that at least one final output is not representable; use
    /// <see cref="DoesCenteredCapsuleOverlap"/> when classification must remain
    /// independent of witness materialization.
    /// </remarks>
    public readonly bool TryGetCenteredCapsuleContact(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackNormal,
        out Vector3d pointOnTriangle,
        out Vector3d pointOnCapsule,
        out Vector3d normal,
        out Fixed64 depth)
    {
        ValidateCenteredCapsuleContact(
            axisDirection,
            axisLength,
            radius,
            fallbackNormal);

        return WideFiniteAxisIntersection.TryGetCenteredCapsuleTriangleContact(
            this,
            center,
            axisDirection,
            axisLength,
            radius,
            fallbackNormal,
            out pointOnTriangle,
            out pointOnCapsule,
            out normal,
            out depth);
    }

    private static void ValidateCenteredCapsuleContact(
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackNormal)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!fallbackNormal.IsNormalized())
            throw new ArgumentException("Fallback normal must be normalized.", nameof(fallbackNormal));
    }

    private static void ValidateCenteredAxis(
        Vector3d axisDirection,
        Fixed64 axisLength)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Axis direction must be normalized.", nameof(axisDirection));
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
    }
}
