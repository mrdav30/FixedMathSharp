//=======================================================================
// FixedQuaternion.Conversions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Conversion methods for <see cref="FixedQuaternion"/>, including Euler angles,
/// direction vectors, 3x3 rotation matrices, and component deconstruction.
/// </content>
public partial struct FixedQuaternion
{
    #region Conversion

    /// <summary>
    /// Converts this quaternion to Euler angles in degrees.
    /// Returns angles as (pitch, yaw, roll), where:
    /// pitch = rotation around X
    /// yaw   = rotation around Y
    /// roll  = rotation around Z
    /// 
    /// The extraction matches FromEulerAngles(), which composes rotations in YXZ order:
    /// q = qy * qx * qz
    /// </summary>
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

    /// <summary>
    /// Converts this FixedQuaternion to the rotated canonical forward direction.
    /// </summary>
    /// <remarks>
    /// The identity quaternion returns <see cref="Vector3d.Forward"/> because FixedMathSharp's
    /// canonical 3D forward direction is <c>+Z</c>.
    /// </remarks>
    /// <returns>A Vector3d representing the rotated canonical forward direction.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ToDirection() =>
        new(2 * (X * Z - W * Y),
            2 * (Y * Z + W * X),
            Fixed64.One - 2 * (X * X + Y * Y));


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
