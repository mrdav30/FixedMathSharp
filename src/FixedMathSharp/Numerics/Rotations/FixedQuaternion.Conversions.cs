//=======================================================================
// FixedQuaternion.Conversions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Conversion methods for <see cref="FixedQuaternion"/>, including Euler angles,
/// direction vectors, 3x3 rotation matrices, and component deconstruction.
/// </content>
public partial struct FixedQuaternion
{
    #region Conversion

    /// <summary>Converts this rotation to YXZ Euler angles.</summary>
    /// <returns>Pitch about X, yaw about Y, and roll about Z, in degrees and stored in X, Y, Z order.</returns>
    /// <remarks>The extraction corresponds to the composition <c>qy * qx * qz</c>. At gimbal lock, the method sets roll to zero and chooses a corresponding yaw.</remarks>
    public Vector3d ToEulerAngles()
    {
        Fixed3x3 m = ToMatrix3x3();

        Fixed64 pitch;
        Fixed64 yaw;
        Fixed64 roll;

        // For YXZ:
        // m32 = -sin(pitch)
        // m31 =  sin(yaw) * cos(pitch)
        // m33 =  cos(yaw) * cos(pitch)
        // m12 =  sin(roll) * cos(pitch)
        // m22 =  cos(roll) * cos(pitch)

        Fixed64 sinPitch = -m.M32;

        if (sinPitch.Abs() >= Fixed64.One)
        {
            // Gimbal lock: pitch is ±90°, yaw/roll are coupled.
            pitch = FixedMath.CopySign(Fixed64.HalfPi, sinPitch);

            // Choose roll = 0 and solve remaining yaw from matrix.
            roll = Fixed64.Zero;
            yaw = FixedMath.Atan2(-m.M13, m.M11);
        }
        else
        {
            pitch = FixedMath.Asin(sinPitch);
            yaw = FixedMath.Atan2(m.M31, m.M33);
            roll = FixedMath.Atan2(m.M12, m.M22);
        }

        return new Vector3d(
            FixedMath.RadToDeg(pitch),
            FixedMath.RadToDeg(yaw),
            FixedMath.RadToDeg(roll));
    }

    /// <summary>Gets this rotation's local forward axis in world coordinates.</summary>
    /// <returns>The rotated <see cref="Vector3d.Forward"/> direction.</returns>
    /// <remarks>Uses <see cref="Rotate(Vector3d)"/>. A zero quaternion returns canonical forward.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetForwardDirection() => Rotate(Vector3d.Forward);

    /// <summary>Expresses canonical world forward in this rotation's local axes.</summary>
    /// <returns>For a unit quaternion, the inverse-rotated <see cref="Vector3d.Forward"/> direction.</returns>
    /// <remarks>The formula uses the supplied components directly. Normalize nonunit quaternions before using the result as a rotation direction. Zero returns canonical forward.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetWorldForwardInLocalFrame() =>
        new(2 * (X * Z - W * Y),
            2 * (Y * Z + W * X),
            Fixed64.One - 2 * (X * X + Y * Y));

    /// <inheritdoc cref="GetWorldForwardInLocalFrame()"/>
    [Obsolete("Use " + nameof(GetWorldForwardInLocalFrame) + "() instead.")]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ToDirection() => GetWorldForwardInLocalFrame();

    /// <summary>
    /// Converts the quaternion into a 3x3 rotation matrix.
    /// </summary>
    /// <remarks>
    /// Every nonzero scalar multiple represents the same rotation. The zero
    /// quaternion converts to <see cref="Fixed3x3.Identity"/>.
    /// </remarks>
    /// <returns>A FixedMatrix3x3 representing the same rotation as the quaternion.</returns>
    public Fixed3x3 ToMatrix3x3()
    {
        Fixed64 componentScale = FixedMath.Max(
            FixedMath.Max(X.Abs(), Y.Abs()),
            FixedMath.Max(Z.Abs(), W.Abs()));
        if (componentScale == Fixed64.Zero)
            return Fixed3x3.Identity;

        Fixed64 x = X;
        Fixed64 y = Y;
        Fixed64 z = Z;
        Fixed64 w = W;

        // Ordinary rotation inputs can use their original coordinates without
        // square underflow, sum saturation, or material factor quantization.
        if (componentScale < Fixed64.Half
            || componentScale > Fixed64.Two)
        {
            x /= componentScale;
            y /= componentScale;
            z /= componentScale;
            w /= componentScale;
        }

        Fixed64 x2 = x * x;
        Fixed64 y2 = y * y;
        Fixed64 z2 = z * z;
        Fixed64 xy = x * y;
        Fixed64 xz = x * z;
        Fixed64 yz = y * z;
        Fixed64 xw = x * w;
        Fixed64 yw = y * w;
        Fixed64 zw = z * w;

        Fixed3x3 result = new();
        Fixed64 factor = Fixed64.Two / (x2 + y2 + z2 + (w * w));

        result.M11 = Fixed64.One - factor * (y2 + z2);
        result.M12 = factor * (xy + zw);
        result.M13 = factor * (xz - yw);

        result.M21 = factor * (xy - zw);
        result.M22 = Fixed64.One - factor * (x2 + z2);
        result.M23 = factor * (yz + xw);

        result.M31 = factor * (xz + yw);
        result.M32 = factor * (yz - xw);
        result.M33 = Fixed64.One - factor * (x2 + y2);

        return result;
    }

    /// <summary>
    /// Deconstructs the quaternion into its four Fixed64 components.
    /// </summary>
    /// <param name="x">The fixed-point X component.</param>
    /// <param name="y">The fixed-point Y component.</param>
    /// <param name="z">The fixed-point Z component.</param>
    /// <param name="w">The fixed-point W component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Fixed64 x, out Fixed64 y, out Fixed64 z, out Fixed64 w)
    {
        x = X;
        y = Y;
        z = Z;
        w = W;
    }

    /// <summary>
    /// Deconstructs the quaternion into its four int components.
    /// </summary>
    /// <param name="x">The X component rounded to the nearest integer with ties to even.</param>
    /// <param name="y">The Y component rounded to the nearest integer with ties to even.</param>
    /// <param name="z">The Z component rounded to the nearest integer with ties to even.</param>
    /// <param name="w">The W component rounded to the nearest integer with ties to even.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out int x, out int y, out int z, out int w)
    {
        x = X.RoundToInt();
        y = Y.RoundToInt();
        z = Z.RoundToInt();
        w = W.RoundToInt();
    }

    /// <summary>
    /// Deconstructs the quaternion into its four long components.
    /// </summary>
    /// <param name="x">The signed 64-bit raw Q32.32 payload of the X component.</param>
    /// <param name="y">The signed 64-bit raw Q32.32 payload of the Y component.</param>
    /// <param name="z">The signed 64-bit raw Q32.32 payload of the Z component.</param>
    /// <param name="w">The signed 64-bit raw Q32.32 payload of the W component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out long x, out long y, out long z, out long w)
    {
        x = X.m_rawValue;
        y = Y.m_rawValue;
        z = Z.m_rawValue;
        w = W.m_rawValue;
    }

    /// <summary>
    /// Deconstructs the quaternion into its four double components.
    /// </summary>
    /// <param name="x">The double-precision approximation of the X component.</param>
    /// <param name="y">The double-precision approximation of the Y component.</param>
    /// <param name="z">The double-precision approximation of the Z component.</param>
    /// <param name="w">The double-precision approximation of the W component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out double x, out double y, out double z, out double w)
    {
        x = (double)X;
        y = (double)Y;
        z = (double)Z;
        w = (double)W;
    }

    #endregion
}
