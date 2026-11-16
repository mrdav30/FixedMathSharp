//=======================================================================
// Fixed4x4.Helpers.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp;

/// <content>
/// Internal helper methods for constructing and validating <see cref="Fixed4x4"/> matrices.
/// </content>
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

    private static Fixed4x4 BuildPerspective(
        Fixed64 xScale, Fixed64 yScale, Fixed64 xOffset, Fixed64 yOffset,
        Fixed64 nearPlaneDistance, Fixed64 farPlaneDistance)
    {
        Fixed64 depth = farPlaneDistance - nearPlaneDistance;
        Fixed64 depthOffset = Fixed64.MultiplyDivide(-nearPlaneDistance, farPlaneDistance, depth, out _);
        return new Fixed4x4(
            xScale, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, yScale, Fixed64.Zero, Fixed64.Zero,
            xOffset, yOffset, farPlaneDistance / depth, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, depthOffset, Fixed64.Zero);
    }

    private static void GetPerspectiveAxis(
        Fixed64 start, Fixed64 end, Signed192 twoNear, out Fixed64 scale, out Fixed64 offset)
    {
        Signed192 span = WideArithmetic.Difference(end, start);
        Signed192 sum = WideArithmetic.AddSigned192(Signed192.Raw(start), Signed192.Raw(end));
        scale = Fixed64.GetSignedRatio(twoNear, span);
        offset = Fixed64.GetSignedRatio(WideArithmetic.Negate(sum), span);
    }

    #endregion
}
