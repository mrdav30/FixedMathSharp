//=======================================================================
// FixedQuaternion.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct FixedQuaternion
{
    #region Quaternion Operations

    /// <summary>
    /// Checks if this vector has been normalized by checking if the magnitude is close to 1.
    /// </summary>
    public bool IsNormalized()
    {
        Fixed64 mag = GetMagnitude(this);
        return FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Calculates the magnitude (or length) of the specified quaternion.
    /// </summary>
    /// <remarks>
    /// If rounding errors cause the computed magnitude to be slightly greater than 1 but within epsilon, the result is clamped to 1. 
    /// This helps maintain numerical stability when working with normalized quaternions.</remarks>
    /// <param name="q">The quaternion for which to compute the magnitude.</param>
    /// <returns>The magnitude of the quaternion as a Fixed64 value. Returns 0 if the quaternion is the zero quaternion.</returns>
    public static Fixed64 GetMagnitude(FixedQuaternion q)
    {
        Fixed64 mag = (q.X * q.X) + (q.Y * q.Y) + (q.Z * q.Z) + (q.W * q.W);
        // If rounding error caused the final magnitude to be slightly above 1, clamp it
        if (mag > Fixed64.One && mag <= Fixed64.One + Fixed64.Epsilon)
            return Fixed64.One;

        return mag != Fixed64.Zero ? FixedMath.Sqrt(mag) : Fixed64.Zero;
    }

    /// <summary>
    /// Normalizes the quaternion to a unit quaternion.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion GetNormalized(FixedQuaternion q)
    {
        Fixed64 mag = GetMagnitude(q);

        // If magnitude is zero, return identity quaternion (to avoid divide by zero)
        if (mag == Fixed64.Zero)
            return new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        // If already normalized, return as-is
        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return q;

        // Normalize it exactly
        return new FixedQuaternion(
            q.X / mag,
            q.Y / mag,
            q.Z / mag,
            q.W / mag
        );
    }

    /// <summary>
    /// Divides one quaternion by another using inverse quaternion multiplication.
    /// </summary>
    /// <remarks>
    /// This is equivalent to <c>dividend * Inverse(divisor)</c>.
    /// </remarks>
    /// <exception cref="InvalidOperationException">
    /// Thrown when <paramref name="divisor"/> is not invertible.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion Divide(FixedQuaternion dividend, FixedQuaternion divisor)
    {
        Fixed64 divisorSqrMagnitude = divisor.SqrMagnitude;

        if (divisorSqrMagnitude == Fixed64.Zero)
            throw new InvalidOperationException("Quaternion divisor is not invertible.");

        Fixed64 invNorm = Fixed64.One / divisorSqrMagnitude;
        FixedQuaternion inverseDivisor = new(
            -divisor.X * invNorm,
            -divisor.Y * invNorm,
            -divisor.Z * invNorm,
            divisor.W * invNorm);

        return dividend * inverseDivisor;
    }

    /// <summary>
    /// Creates a quaternion that rotates one vector to align with another.
    /// </summary>
    /// <param name="forward">The forward direction vector.</param>
    /// <param name="upwards">The upwards direction vector (optional, default: Vector3d.Up).</param>
    /// <returns>A quaternion representing the rotation from one direction to another.</returns>
    public static FixedQuaternion LookRotation(Vector3d forward, Vector3d? upwards = null)
    {
        Vector3d up = upwards ?? Vector3d.Up;

        Vector3d forwardNormalized = forward.Normal;
        Vector3d right = Vector3d.Cross(up.Normal, forwardNormalized);
        up = Vector3d.Cross(forwardNormalized, right);

        return FromMatrix(new Fixed3x3(
            right.X, right.Y, right.Z,
            up.X, up.Y, up.Z,
            forwardNormalized.X, forwardNormalized.Y, forwardNormalized.Z));
    }

    /// <summary>
    /// Converts a rotation matrix into a quaternion representation.
    /// </summary>
    /// <param name="matrix">The rotation matrix to convert.</param>
    /// <returns>A quaternion representing the same rotation as the matrix.</returns>
    public static FixedQuaternion FromMatrix(Fixed3x3 matrix)
    {
        Fixed64 trace = matrix.M11 + matrix.M22 + matrix.M33;

        Fixed64 w, x, y, z;

        if (trace > Fixed64.Zero)
        {
            Fixed64 s = FixedMath.Sqrt(trace + Fixed64.One);
            w = s * Fixed64.Half;
            s = Fixed64.Half / s;
            x = (matrix.M23 - matrix.M32) * s;
            y = (matrix.M31 - matrix.M13) * s;
            z = (matrix.M12 - matrix.M21) * s;
        }
        else if (matrix.M11 > matrix.M22 && matrix.M11 > matrix.M33)
        {
            Fixed64 s = FixedMath.Sqrt(Fixed64.One + matrix.M11 - matrix.M22 - matrix.M33);
            x = s * Fixed64.Half;
            s = Fixed64.Half / s;
            y = (matrix.M21 + matrix.M12) * s;
            z = (matrix.M13 + matrix.M31) * s;
            w = (matrix.M23 - matrix.M32) * s;
        }
        else if (matrix.M22 > matrix.M33)
        {
            Fixed64 s = FixedMath.Sqrt(Fixed64.One + matrix.M22 - matrix.M11 - matrix.M33);
            y = s * Fixed64.Half;
            s = Fixed64.Half / s;
            z = (matrix.M32 + matrix.M23) * s;
            x = (matrix.M21 + matrix.M12) * s;
            w = (matrix.M31 - matrix.M13) * s;
        }
        else
        {
            Fixed64 s = FixedMath.Sqrt(Fixed64.One + matrix.M33 - matrix.M11 - matrix.M22);
            z = s * Fixed64.Half;
            s = Fixed64.Half / s;
            x = (matrix.M13 + matrix.M31) * s;
            y = (matrix.M32 + matrix.M23) * s;
            w = (matrix.M12 - matrix.M21) * s;
        }

        return new FixedQuaternion(x, y, z, w);
    }

    /// <summary>
    /// Converts a rotation matrix (upper-left 3x3 part of a 4x4 matrix) into a quaternion representation.
    /// </summary>
    /// <param name="matrix">The 4x4 matrix containing the rotation component.</param>
    /// <remarks>Extracts the upper-left 3x3 rotation part of the 4x4</remarks>
    /// <returns>A quaternion representing the same rotation as the matrix.</returns>
    public static FixedQuaternion FromMatrix(Fixed4x4 matrix)
    {
        Fixed3x3 rotationMatrix = new(
            matrix.M11, matrix.M12, matrix.M13,
            matrix.M21, matrix.M22, matrix.M23,
            matrix.M31, matrix.M32, matrix.M33
        );

        return FromMatrix(rotationMatrix);
    }

    /// <summary>
    /// Creates a quaternion representing the rotation needed to align the forward vector with the given direction.
    /// </summary>
    /// <param name="direction">The target direction vector.</param>
    /// <returns>A quaternion representing the rotation to align with the direction.</returns>
    public static FixedQuaternion FromDirection(Vector3d direction)
    {
        Fixed64 directionSqrMagnitude = direction.SqrMagnitude;
        if (directionSqrMagnitude == Fixed64.Zero)
            return Identity;

        if (FixedMath.Abs(directionSqrMagnitude - Fixed64.One) > Fixed64.Epsilon)
        {
            Fixed64 directionMagnitude = FixedMath.Sqrt(directionSqrMagnitude);
            direction = new Vector3d(
                direction.X / directionMagnitude,
                direction.Y / directionMagnitude,
                direction.Z / directionMagnitude);
        }

        Fixed64 dot = direction.Z;
        if (dot <= -Fixed64.One + Fixed64.Epsilon)
            return FromAxisAngle(Vector3d.Up, Fixed64.Pi);

        if (dot >= Fixed64.One - Fixed64.Epsilon)
            return Identity;

        if (dot < Fixed64.FromRaw(NearOppositeDirectionDotRaw))
        {
            Vector3d axis = new(-direction.Y, direction.X, Fixed64.Zero);
            Fixed64 axisMagnitude = axis.Magnitude;
            if (axisMagnitude == Fixed64.Zero)
                return FromAxisAngle(Vector3d.Up, Fixed64.Pi);

            axis = new Vector3d(
                axis.X / axisMagnitude,
                axis.Y / axisMagnitude,
                Fixed64.Zero);

            return FromAxisAngle(axis, FixedMath.Acos(dot));
        }

        Fixed64 scale = FixedMath.Sqrt((Fixed64.One + dot) * Fixed64.Two);
        return new FixedQuaternion(
            -direction.Y / scale,
            direction.X / scale,
            Fixed64.Zero,
            scale * Fixed64.Half);
    }

    /// <summary>
    /// Creates a quaternion representing a rotation around a specified axis by a given angle.
    /// </summary>
    /// <param name="axis">The axis to rotate around (must be normalized).</param>
    /// <param name="angle">The rotation angle in radians.</param>
    /// <returns>A quaternion representing the rotation.</returns>
    public static FixedQuaternion FromAxisAngle(Vector3d axis, Fixed64 angle)
    {
        // Check if the angle is in a valid range (-pi, pi)
        if (angle < -Fixed64.Pi || angle > Fixed64.Pi)
            throw new ArgumentOutOfRangeException(nameof(angle), $"Angle must be in the range ({-Fixed64.Pi}, {Fixed64.Pi}), but was {angle}");

        Fixed64 axisSqrMagnitude = axis.SqrMagnitude;
        if (axisSqrMagnitude == Fixed64.Zero)
            return Identity;

        if (FixedMath.Abs(axisSqrMagnitude - Fixed64.One) > Fixed64.Epsilon)
        {
            Fixed64 axisMagnitude = FixedMath.Sqrt(axisSqrMagnitude);
            axis = new Vector3d(
                axis.X / axisMagnitude,
                axis.Y / axisMagnitude,
                axis.Z / axisMagnitude);
        }

        Fixed64 halfAngle = angle / Fixed64.Two;  // Half-angle formula
        Fixed64 sinHalfAngle = FixedMath.Sin(halfAngle);
        Fixed64 cosHalfAngle = FixedMath.Cos(halfAngle);

        return new FixedQuaternion(
            axis.X * sinHalfAngle,
            axis.Y * sinHalfAngle,
            axis.Z * sinHalfAngle,
            cosHalfAngle);
    }

    /// <summary>
    /// Assume the input angles are in degrees and converts them to radians before calling <see cref="FromEulerAngles"/> 
    /// </summary>
    /// <param name="pitch"></param>
    /// <param name="yaw"></param>
    /// <param name="roll"></param>
    /// <returns></returns>
    public static FixedQuaternion FromEulerAnglesInDegrees(Fixed64 pitch, Fixed64 yaw, Fixed64 roll)
    {
        // Convert input angles from degrees to radians
        pitch = FixedMath.DegToRad(pitch);
        yaw = FixedMath.DegToRad(yaw);
        roll = FixedMath.DegToRad(roll);

        // Call the original method that expects angles in radians
        return FromEulerAngles(pitch, yaw, roll);
    }

    /// <summary>
    /// Converts Euler angles (pitch, yaw, roll) to a quaternion and normalizes the result afterwards. 
    /// Assumes the input angles are in radians.
    /// </summary>
    /// <remarks>
    /// The order of operations is YXZ or yaw-pitch-roll
    /// </remarks>
    public static FixedQuaternion FromEulerAngles(Fixed64 pitch, Fixed64 yaw, Fixed64 roll)
    {
        // Check if the angles are in a valid range (-pi, pi)
        if (pitch < -Fixed64.Pi || pitch > Fixed64.Pi)
            throw new ArgumentOutOfRangeException(nameof(pitch), $"Pitch must be in the range ({-Fixed64.Pi}, {Fixed64.Pi}), but was {pitch}");

        if (yaw < -Fixed64.Pi || yaw > Fixed64.Pi)
            throw new ArgumentOutOfRangeException(nameof(yaw), $"Yaw must be in the range ({-Fixed64.Pi}, {Fixed64.Pi}), but was {yaw}");

        if (roll < -Fixed64.Pi || roll > Fixed64.Pi)
            throw new ArgumentOutOfRangeException(nameof(roll), $"Roll must be in the range ({-Fixed64.Pi}, {Fixed64.Pi}), but was {roll}");

        Fixed64 halfPitch = pitch / Fixed64.Two;
        Fixed64 halfYaw = yaw / Fixed64.Two;
        Fixed64 halfRoll = roll / Fixed64.Two;

        Fixed64 sx = FixedMath.Sin(halfPitch);
        Fixed64 cx = FixedMath.Cos(halfPitch);
        Fixed64 sy = FixedMath.Sin(halfYaw);
        Fixed64 cy = FixedMath.Cos(halfYaw);
        Fixed64 sz = FixedMath.Sin(halfRoll);
        Fixed64 cz = FixedMath.Cos(halfRoll);

        // q = qy * qx * qz
        Fixed64 x = (cx * sy * sz) + (cy * cz * sx);
        Fixed64 y = (cx * cz * sy) - (cy * sx * sz);
        Fixed64 z = (cx * cy * sz) - (cz * sx * sy);
        Fixed64 w = (cx * cy * cz) + (sx * sy * sz);

        return GetNormalized(new FixedQuaternion(x, y, z, w));
    }

    /// <summary>
    /// Computes the logarithm of a quaternion, which represents the rotational displacement.
    /// This is useful for interpolation and angular velocity calculations.
    /// </summary>
    /// <param name="q">The quaternion to compute the logarithm of.</param>
    /// <returns>A Vector3d representing the logarithm of the quaternion (axis-angle representation).</returns>
    /// <remarks>
    /// The logarithm of a unit quaternion is given by:
    /// log(q) = (θ * v̂), where:
    /// - θ = 2 * acos(w) is the rotation angle.
    /// - v̂ = (x, y, z) / ||(x, y, z)|| is the unit vector representing the axis of rotation.
    /// If the quaternion is close to identity, the function returns a zero vector to avoid numerical instability.
    /// </remarks>
    public static Vector3d QuaternionLog(FixedQuaternion q)
    {
        // Ensure the quaternion is normalized
        q = q.Normal;

        // Extract vector part
        Vector3d v = new(q.X, q.Y, q.Z);
        Fixed64 vLength = v.Magnitude;

        // If rotation is very small, avoid division by zero
        if (vLength < Fixed64.FromRaw(0x00001000L)) // Small epsilon
            return Vector3d.Zero;

        // Compute angle (theta = 2 * acos(w))
        Fixed64 theta = Fixed64.Two * FixedMath.Acos(q.W);

        // Convert to angular velocity
        return (v / vLength) * theta;
    }

    /// <summary>
    /// Computes the angular velocity required to move from `previousRotation` to `currentRotation` over a given time step.
    /// </summary>
    /// <param name="currentRotation">The current orientation as a quaternion.</param>
    /// <param name="previousRotation">The previous orientation as a quaternion.</param>
    /// <param name="deltaTime">The time step over which the rotation occurs.</param>
    /// <returns>A Vector3d representing the angular velocity (in radians per second).</returns>
    /// <remarks>
    /// This function calculates the change in rotation over `deltaTime` and converts it into angular velocity.
    /// - First, it computes the relative rotation: `rotationDelta = currentRotation * previousRotation.Inverse()`.
    /// - Then, it applies `QuaternionLog(rotationDelta)` to extract the axis-angle representation.
    /// - Finally, it divides by `deltaTime` to compute the angular velocity.
    /// </remarks>
    public static Vector3d ToAngularVelocity(
        FixedQuaternion currentRotation,
        FixedQuaternion previousRotation,
        Fixed64 deltaTime)
    {
        FixedQuaternion rotationDelta = currentRotation * previousRotation.Inverse();
        Vector3d angularDisplacement = QuaternionLog(rotationDelta);

        return angularDisplacement / deltaTime; // Convert to angular velocity
    }

    /// <summary>
    /// Performs a simple linear interpolation between the components of the input quaternions
    /// </summary>
    public static FixedQuaternion Lerp(FixedQuaternion a, FixedQuaternion b, Fixed64 t)
    {
        t = FixedMath.Clamp01(t);

        if (Dot(a, b) < Fixed64.Zero)
            b = -b;

        FixedQuaternion result;
        Fixed64 oneMinusT = Fixed64.One - t;
        result.X = a.X * oneMinusT + b.X * t;
        result.Y = a.Y * oneMinusT + b.Y * t;
        result.Z = a.Z * oneMinusT + b.Z * t;
        result.W = a.W * oneMinusT + b.W * t;

        result.Normalize();

        return result;
    }

    /// <summary>
    ///  Calculates the spherical linear interpolation, which results in a smoother and more accurate rotation interpolation
    /// </summary>
    public static FixedQuaternion Slerp(FixedQuaternion a, FixedQuaternion b, Fixed64 t)
    {
        t = FixedMath.Clamp01(t);

        Fixed64 cosOmega = a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;

        // If the dot product is negative, negate one of the input quaternions.
        // This ensures that the interpolation takes the shortest path around the sphere.
        if (cosOmega < Fixed64.Zero)
        {
            b.X = -b.X;
            b.Y = -b.Y;
            b.Z = -b.Z;
            b.W = -b.W;
            cosOmega = -cosOmega;
        }

        Fixed64 k0, k1;

        // If the quaternions are close, use linear interpolation
        if (cosOmega > Fixed64.One - Fixed64.Epsilon)
        {
            k0 = Fixed64.One - t;
            k1 = t;
        }
        else
        {
            // Otherwise, use spherical linear interpolation
            Fixed64 sinOmega = FixedMath.Sqrt(Fixed64.One - cosOmega * cosOmega);
            Fixed64 omega = FixedMath.Atan2(sinOmega, cosOmega);

            k0 = FixedMath.Sin((Fixed64.One - t) * omega) / sinOmega;
            k1 = FixedMath.Sin(t * omega) / sinOmega;
        }

        FixedQuaternion result;
        result.X = a.X * k0 + b.X * k1;
        result.Y = a.Y * k0 + b.Y * k1;
        result.Z = a.Z * k0 + b.Z * k1;
        result.W = a.W * k0 + b.W * k1;

        return result;
    }

    /// <summary>
    /// Returns the angle in degrees between two rotations a and b.
    /// </summary>
    /// <param name="a">The first rotation.</param>
    /// <param name="b">The second rotation.</param>
    /// <returns>The angle in degrees between the two rotations.</returns>
    public static Fixed64 Angle(FixedQuaternion a, FixedQuaternion b)
    {
        // Calculate the dot product of the two quaternions
        Fixed64 dot = Dot(a, b);

        // Ensure the dot product is in the range of [-1, 1] to avoid floating-point inaccuracies
        dot = FixedMath.Clamp(dot, -Fixed64.One, Fixed64.One);

        // Calculate the angle between the two quaternions using the inverse cosine (arccos)
        // arccos(dot(a, b)) gives us the angle in radians, so we convert it to degrees
        Fixed64 angleInRadians = FixedMath.Acos(dot);

        // Convert the angle from radians to degrees
        Fixed64 angleInDegrees = FixedMath.RadToDeg(angleInRadians);

        return angleInDegrees;
    }

    /// <summary>
    /// Creates a quaternion from an angle and axis.
    /// </summary>
    /// <param name="angle">The angle in degrees.</param>
    /// <param name="axis">The axis to rotate around (must be normalized).</param>
    /// <returns>A quaternion representing the rotation.</returns>
    public static FixedQuaternion AngleAxis(Fixed64 angle, Vector3d axis)
    {
        // Convert the angle to radians
        angle = angle.ToRadians();

        // Normalize the axis
        axis = axis.Normal;

        // Use the half-angle formula (sin(theta / 2), cos(theta / 2))
        Fixed64 halfAngle = angle / Fixed64.Two;
        Fixed64 sinHalfAngle = FixedMath.Sin(halfAngle);
        Fixed64 cosHalfAngle = FixedMath.Cos(halfAngle);

        return new FixedQuaternion(
            axis.X * sinHalfAngle,
            axis.Y * sinHalfAngle,
            axis.Z * sinHalfAngle,
            cosHalfAngle
        );
    }

    /// <summary>
    /// Calculates the dot product of two quaternions.
    /// </summary>
    /// <param name="a">The first quaternion.</param>
    /// <param name="b">The second quaternion.</param>
    /// <returns>The dot product of the two quaternions.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(FixedQuaternion a, FixedQuaternion b) => a.W * b.W + a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    #endregion
}
