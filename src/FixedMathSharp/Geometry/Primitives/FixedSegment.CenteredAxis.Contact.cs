//=======================================================================
// FixedSegment.CenteredAxis.Contact.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains methods for computing the contact relation between two centered 3D capsules without narrowing either finite axis.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Attempts to build the exact contact relation between two conceptual
    /// centered capsules without narrowing either finite axis.
    /// </summary>
    /// <remarks>
    /// Each capsule axis is expressed in its normalized rigid frame. The
    /// returned normal points from the first capsule toward the second.
    /// <paramref name="fallbackNormal"/> is used only when the closest axis
    /// points coincide. Axial and radial terms remain separate in each anchor,
    /// so classification does not depend on materializing a combined world
    /// point or center-relative offset.
    /// </remarks>
    /// <returns>
    /// <see langword="true"/> when the capsules overlap; otherwise
    /// <see langword="false"/>.
    /// </returns>
    public static bool TryGetCenteredCapsulesContact(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        Vector3d fallbackNormal,
        out FixedContactAnchors contact)
    {
        ValidateNormalizedRotation(firstRotation, nameof(firstRotation));
        ValidateNormalizedRotation(secondRotation, nameof(secondRotation));
        ValidateCenteredCapsulesContact(
            firstLocalAxisDirection,
            firstAxisLength,
            firstRadius,
            secondLocalAxisDirection,
            secondAxisLength,
            secondRadius,
            fallbackNormal);
        if (!WideOrientedBox.DoCenteredRigidCapsulesOverlap(
                firstCenter,
                firstRotation,
                firstLocalAxisDirection,
                firstAxisLength,
                firstRadius,
                secondCenter,
                secondRotation,
                secondLocalAxisDirection,
                secondAxisLength,
                secondRadius))
        {
            contact = default;
            return false;
        }
        _ = firstRotation.TryRotate(
            firstLocalAxisDirection,
            out Vector3d firstAxisDirection);
        _ = secondRotation.TryRotate(
            secondLocalAxisDirection,
            out Vector3d secondAxisDirection);
        firstAxisDirection = firstAxisDirection.Normalized;
        secondAxisDirection = secondAxisDirection.Normalized;

        return WideFiniteAxisIntersection.TryGetCenteredCapsulesContact(
            firstCenter,
            firstRotation,
            firstLocalAxisDirection,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            secondCenter,
            secondRotation,
            secondLocalAxisDirection,
            secondAxisDirection,
            secondAxisLength,
            secondRadius,
            fallbackNormal,
            out contact);
    }

    private static void ValidateNormalizedRotation(
        FixedQuaternion rotation,
        string parameterName)
    {
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Capsule rotation must be normalized.",
                parameterName);
        }
    }

    private static void ValidateCenteredCapsulesContact(
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        Vector3d fallbackNormal)
    {
        ValidateCenteredAxis(
            firstAxisDirection,
            firstAxisLength,
            nameof(firstAxisDirection),
            nameof(firstAxisLength));
        ValidateCenteredAxis(
            secondAxisDirection,
            secondAxisLength,
            nameof(secondAxisDirection),
            nameof(secondAxisLength));
        if (firstRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(firstRadius));
        if (secondRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(secondRadius));
        if (!fallbackNormal.IsNormalized())
            throw new ArgumentException("Fallback normal must be normalized.", nameof(fallbackNormal));
    }

    private static void ValidateCenteredAxis(
        Vector3d axisDirection,
        Fixed64 axisLength) =>
        ValidateCenteredAxis(
            axisDirection,
            axisLength,
            nameof(axisDirection),
            nameof(axisLength));

    private static void ValidateCenteredAxis(
        Vector3d axisDirection,
        Fixed64 axisLength,
        string directionParameterName,
        string lengthParameterName)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Axis direction must be normalized.", directionParameterName);
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(lengthParameterName);
    }
}
