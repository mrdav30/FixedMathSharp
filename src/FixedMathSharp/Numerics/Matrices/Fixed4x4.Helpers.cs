//=======================================================================
// Fixed4x4.Helpers.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp;

public partial struct Fixed4x4
{
    #region Private Helpers

    private static Fixed4x4 FromRotationMatrix(Fixed3x3 matrix) =>
        new(
            matrix.M11, matrix.M12, matrix.M13, Fixed64.Zero,
            matrix.M21, matrix.M22, matrix.M23, Fixed64.Zero,
            matrix.M31, matrix.M32, matrix.M33, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

    private static void ValidateDepthRange(Fixed64 nearPlaneDistance, Fixed64 farPlaneDistance)
    {
        if (nearPlaneDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(nearPlaneDistance), "Near plane distance must be greater than or equal to zero.");

        if (farPlaneDistance <= nearPlaneDistance)
            throw new ArgumentOutOfRangeException(nameof(farPlaneDistance), "Far plane distance must be greater than or equal to zero.");
    }

    private static void ValidatePerspectiveDepthRange(Fixed64 nearPlaneDistance, Fixed64 farPlaneDistance)
    {
        if (nearPlaneDistance <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(nearPlaneDistance), "Near plane distance must be greater than zero.");

        if (farPlaneDistance <= nearPlaneDistance)
            throw new ArgumentOutOfRangeException(nameof(farPlaneDistance), "Far plane distance must be greater than near plane distance.");
    }

    #endregion
}
