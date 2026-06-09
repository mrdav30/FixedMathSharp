//=======================================================================
// FixedQuaternion.Conversions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

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
    /// <returns>A FixedMatrix3x3 representing the same rotation as the quaternion.</returns>
    public Fixed3x3 ToMatrix3x3()
    {
        Fixed64 x2 = X * X;
        Fixed64 y2 = Y * Y;
        Fixed64 z2 = Z * Z;
        Fixed64 xy = X * Y;
        Fixed64 xz = X * Z;
        Fixed64 yz = Y * Z;
        Fixed64 xw = X * W;
        Fixed64 yw = Y * W;
        Fixed64 zw = Z * W;

        Fixed3x3 result = new();
        Fixed64 scale = Fixed64.One * 2;

        result.M11 = Fixed64.One - scale * (y2 + z2);
        result.M12 = scale * (xy + zw);
        result.M13 = scale * (xz - yw);

        result.M21 = scale * (xy - zw);
        result.M22 = Fixed64.One - scale * (x2 + z2);
        result.M23 = scale * (yz + xw);

        result.M31 = scale * (xz + yw);
        result.M32 = scale * (yz - xw);
        result.M33 = Fixed64.One - scale * (x2 + y2);

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
