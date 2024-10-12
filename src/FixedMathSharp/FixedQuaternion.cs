using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a quaternion (x, y, z, w) with fixed-point numbers.
    /// Quaternions are useful for representing rotations and can be used to perform smooth rotations and avoid gimbal lock.
    /// </summary>
    [Serializable]
    public partial struct FixedQuaternion : IEquatable<FixedQuaternion>
    {
        public Fixed64 x, y, z, w;

        /// <summary>
        /// Normalized version of this quaternion.
        /// </summary>
        public FixedQuaternion MyNormalized => Normalize(this);

        /// <summary>
        /// Creates a new FixedQuaternion with the specified components.
        /// </summary>
        public FixedQuaternion(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        /// <summary>
        /// Identity quaternion (0, 0, 0, 1).
        /// </summary>
        public static FixedQuaternion Identity => new FixedQuaternion(FixedMath.Zero, FixedMath.Zero, FixedMath.Zero, FixedMath.One);

        /// <summary>
        /// Returns the Euler angles (in degrees) of this quaternion.
        /// </summary>
        public Vector3d EulerAngles
        {
            get => ToEulerAngles();
            set => this = FromEulerAnglesInDegrees(value.x, value.y, value.z);
        }

        /// <summary>
        /// Normalizes the quaternion to a unit quaternion.
        /// </summary>
        public static FixedQuaternion Normalize(FixedQuaternion q)
        {
            Fixed64 magnitudeSqr = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
            if (magnitudeSqr <= FixedMath.Epsilon)
                return Identity;

            Fixed64 invMagnitude = FixedMath.One / FixedMath.Sqrt(magnitudeSqr);
            return new FixedQuaternion(q.x * invMagnitude, q.y * invMagnitude, q.z * invMagnitude, q.w * invMagnitude);

        }

        /// <summary>
        /// Normalizes this quaternion in place.
        /// </summary>
        public void Normalize()
        {
            this = Normalize(this);
        }

        /// <summary>
        /// Returns the conjugate of this quaternion (inverses the rotational effect).
        /// </summary>
        public FixedQuaternion Conjugate()
        {
            return new FixedQuaternion(-x, -y, -z, w);
        }

        /// <summary>
        /// Returns the inverse of this quaternion.
        /// </summary>
        public FixedQuaternion Inverse()
        {
            if (this == Identity) return Identity;
            Fixed64 norm = x * x + y * y + z * z + w * w;
            if (norm == FixedMath.Zero) return this; // Handle division by zero by returning the same quaternion

            Fixed64 invNorm = FixedMath.One / norm;
            return new FixedQuaternion(x * -invNorm, y * -invNorm, z * -invNorm, w * invNorm);

        }

        /// <summary>
        /// Rotates a vector by this quaternion.
        /// </summary>
        public Vector3d Rotate(Vector3d v)
        {
            FixedQuaternion vQuat = new FixedQuaternion(v.x, v.y, v.z, FixedMath.Zero);
            FixedQuaternion invQuat = Inverse();
            FixedQuaternion rotatedVQuat = this * vQuat * invQuat;
            return new Vector3d(rotatedVQuat.x, rotatedVQuat.y, rotatedVQuat.z);
        }

        public FixedQuaternion Rotated(Fixed64 sin, Fixed64 cos)
        {
            return Rotated(sin, cos, Vector3d.Up);
        }

        public FixedQuaternion Rotated(Fixed64 sin, Fixed64 cos, Vector3d axis)
        {
            // The rotation angle is the arc tangent of sin and cos
            Fixed64 angle = FixedMath.Atan2(sin, cos);

            // Construct a quaternion representing a rotation around the axis (default is y aka Vector3d.up)
            FixedQuaternion rotationQuat = FromAxisAngle(axis, angle);

            // Apply the rotation and return the result
            return rotationQuat * this;
        }

        public static FixedQuaternion LookRotation(Vector3d forward)
        {
            return LookRotation(forward, Vector3d.Up);
        }

        public static FixedQuaternion LookRotation(Vector3d forward, Vector3d upwards)
        {
            Vector3d forwardNormalized = forward.MyNormalized;
            Vector3d right = Vector3d.Cross(upwards.MyNormalized, forwardNormalized);
            Vector3d up = Vector3d.Cross(forwardNormalized, right);

            return FromMatrix(new FixedMatrix3x3(right.x, up.x, forwardNormalized.x,
                                            right.y, up.y, forwardNormalized.y,
                                            right.z, up.z, forwardNormalized.z));
        }

        public static FixedQuaternion FromMatrix(FixedMatrix3x3 matrix)
        {
            Fixed64 trace = matrix.m00 + matrix.m11 + matrix.m22;

            Fixed64 w, x, y, z;

            if (trace > FixedMath.Zero)
            {
                Fixed64 s = FixedMath.Sqrt(trace + FixedMath.One);
                w = s * FixedMath.Half;
                s = FixedMath.Half / s;
                x = (matrix.m21 - matrix.m12) * s;
                y = (matrix.m02 - matrix.m20) * s;
                z = (matrix.m10 - matrix.m01) * s;
            }
            else if (matrix.m00 > matrix.m11 && matrix.m00 > matrix.m22)
            {
                Fixed64 s = FixedMath.Sqrt(FixedMath.One + matrix.m00 - matrix.m11 - matrix.m22);
                x = s * FixedMath.Half;
                s = FixedMath.Half / s;
                y = (matrix.m10 + matrix.m01) * s;
                z = (matrix.m02 + matrix.m20) * s;
                w = (matrix.m21 - matrix.m12) * s;
            }
            else if (matrix.m11 > matrix.m22)
            {
                Fixed64 s = FixedMath.Sqrt(FixedMath.One + matrix.m11 - matrix.m00 - matrix.m22);
                y = s * FixedMath.Half;
                s = FixedMath.Half / s;
                z = (matrix.m21 + matrix.m12) * s;
                x = (matrix.m10 + matrix.m01) * s;
                w = (matrix.m02 - matrix.m20) * s;
            }
            else
            {
                Fixed64 s = FixedMath.Sqrt(FixedMath.One + matrix.m22 - matrix.m00 - matrix.m11);
                z = s * FixedMath.Half;
                s = FixedMath.Half / s;
                x = (matrix.m02 + matrix.m20) * s;
                y = (matrix.m21 + matrix.m12) * s;
                w = (matrix.m10 - matrix.m01) * s;
            }

            return new FixedQuaternion(x, y, z, w);
        }

        public static FixedQuaternion FromDirection(Vector3d direction)
        {
            // Compute the rotation axis as the cross product of the standard forward vector and the desired direction
            Vector3d axis = Vector3d.Cross(Vector3d.Forward, direction);
            Fixed64 axisLength = axis.MyMagnitude;

            // If the axis length is very close to zero, it means that the desired direction is almost equal to the standard forward vector
            // In this case, we can return the identity quaternion as the rotation
            if (axisLength < FixedMath.Epsilon)
            {
                return Identity;
            }

            // Normalize the rotation axis
            axis = (axis / axisLength).MyNormalized;

            // Compute the angle between the standard forward vector and the desired direction
            Fixed64 angle = FixedMath.Acos(Vector3d.Dot(Vector3d.Forward, direction));

            // Compute the rotation quaternion from the axis and angle
            return FromAxisAngle(axis, angle);
        }

        public static FixedQuaternion FromAxisAngle(Vector3d axis, Fixed64 angle)
        {
            // Check if the axis is a unit vector
            if (!axis.IsNormalized())
            {
                throw new ArgumentException("Axis must be a unit vector");
            }

            // Check if the angle is in a valid range (-pi, pi)
            if (angle < -FixedMath.PI || angle > FixedMath.PI)
            {
                throw new ArgumentOutOfRangeException("Angle must be in the range (-pi, pi)");
            }

            Fixed64 halfAngle = angle / FixedMath.Double;
            Fixed64 s = FixedMath.Sin(halfAngle);
            Fixed64 c = FixedMath.Cos(halfAngle);

            return new FixedQuaternion(axis.x * s, axis.y * s, axis.z * s, c);
        }

        /// <summary>
        /// Assume the input angles are in degrees and convert them to radians before calling <see cref="FromEulerAngles"/> 
        /// </summary>
        /// <param name="pitch"></param>
        /// <param name="yaw"></param>
        /// <param name="roll"></param>
        /// <returns></returns>
        public static FixedQuaternion FromEulerAnglesInDegrees(Fixed64 pitch, Fixed64 yaw, Fixed64 roll)
        {
            // Convert input angles from degrees to radians
            pitch *= FixedMath.Deg2Rad;
            yaw *= FixedMath.Deg2Rad;
            roll *= FixedMath.Deg2Rad;

            // Call the original method that expects angles in radians
            return FromEulerAngles(pitch, yaw, roll);
        }

        /// <summary>
        /// Converts Euler angles (pitch, yaw, roll) to a quaternion. Assumes the input angles are in radians.
        /// </summary>
        /// <remarks>
        /// The order of operations is YZX or yaw-roll-pitch, commonly used in applications such as robotics.
        /// </remarks>
        public static FixedQuaternion FromEulerAngles(Fixed64 pitch, Fixed64 yaw, Fixed64 roll)
        {
            // Check if the angles are in a valid range (-pi, pi)
            if (pitch < -FixedMath.PI || pitch > FixedMath.PI ||
                yaw < -FixedMath.PI || yaw > FixedMath.PI ||
                roll < -FixedMath.PI || roll > FixedMath.PI)
            {
                throw new ArgumentOutOfRangeException("Euler angles must be in the range (-pi, pi)");
            }

            Fixed64 c1 = FixedMath.Cos(yaw / FixedMath.Double);
            Fixed64 s1 = FixedMath.Sin(yaw / FixedMath.Double);
            Fixed64 c2 = FixedMath.Cos(roll / FixedMath.Double);
            Fixed64 s2 = FixedMath.Sin(roll / FixedMath.Double);
            Fixed64 c3 = FixedMath.Cos(pitch / FixedMath.Double);
            Fixed64 s3 = FixedMath.Sin(pitch / FixedMath.Double);

            Fixed64 c1c2 = c1 * c2;
            Fixed64 s1s2 = s1 * s2;

            Fixed64 w = c1c2 * c3 - s1s2 * s3;
            Fixed64 x = c1c2 * s3 + s1s2 * c3;
            Fixed64 y = s1 * c2 * c3 + c1 * s2 * s3;
            Fixed64 z = c1 * s2 * c3 - s1 * c2 * s3;

            return new FixedQuaternion(x, y, z, w);
        }

        /// <summary>
        /// Converts this FixedQuaternion to Euler angles (pitch, yaw, roll).
        /// </summary>
        /// <remarks>
        /// Handles the case where the pitch angle (asin of sinp) would be out of the range -π/2 to π/2. 
        /// This is known as the gimbal lock situation, where the pitch angle reaches ±90 degrees and we lose one degree of freedom in our rotation (we can't distinguish between yaw and roll). 
        /// In this case, we simply set the pitch to ±90 degrees depending on the sign of sinp.
        /// </remarks>
        /// <returns>A Vector3d representing the Euler angles (in degrees) equivalent to this FixedQuaternion in YZX order (yaw, pitch, roll).</returns>
        public Vector3d ToEulerAngles()
        {
            // roll (x-axis rotation)
            Fixed64 sinr_cosp = 2 * (w * x + y * z);
            Fixed64 cosr_cosp = FixedMath.One - 2 * (x * x + y * y);
            Fixed64 roll = FixedMath.Atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            Fixed64 sinp = 2 * (w * y - z * x);
            Fixed64 pitch;
            if (sinp.Abs() >= FixedMath.One)
                pitch = FixedMath.CopySign(FixedMath.PiOver2, sinp); // use 90 degrees if out of range
            else
                pitch = FixedMath.Asin(sinp);

            // yaw (z-axis rotation)
            Fixed64 siny_cosp = 2 * (w * z + x * y);
            Fixed64 cosy_cosp = FixedMath.One - 2 * (y * y + z * z);
            Fixed64 yaw = FixedMath.Atan2(siny_cosp, cosy_cosp);

            // Convert radians to degrees
            roll *= FixedMath.Rad2Deg;
            pitch *= FixedMath.Rad2Deg;
            yaw *= FixedMath.Rad2Deg;

            return new Vector3d(roll, pitch, yaw);
        }

        /// <summary>
        /// Converts this FixedQuaternion to a direction vector.
        /// </summary>
        /// <returns>A Vector3d representing the direction equivalent to this FixedQuaternion.</returns>
        public Vector3d ToDirection()
        {
            return new Vector3d(
                2 * (x * z - w * y),
                2 * (y * z + w * x),
                FixedMath.One - 2 * (x * x + y * y)
            );
        }

        public FixedMatrix3x3 ToMatrix()
        {
            Fixed64 x2 = x * x;
            Fixed64 y2 = y * y;
            Fixed64 z2 = z * z;
            Fixed64 xy = x * y;
            Fixed64 xz = x * z;
            Fixed64 yz = y * z;
            Fixed64 xw = x * w;
            Fixed64 yw = y * w;
            Fixed64 zw = z * w;

            FixedMatrix3x3 result = new FixedMatrix3x3();
            Fixed64 scale = FixedMath.One * 2;

            result.m00 = FixedMath.One - scale * (y2 + z2);
            result.m01 = scale * (xy - zw);
            result.m02 = scale * (xz + yw);

            result.m10 = scale * (xy + zw);
            result.m11 = FixedMath.One - scale * (x2 + z2);
            result.m12 = scale * (yz - xw);

            result.m20 = scale * (xz - yw);
            result.m21 = scale * (yz + xw);
            result.m22 = FixedMath.One - scale * (x2 + y2);

            return result;
        }

        /// <summary>
        /// Performs a simple linear interpolation between the components of the input quaternions
        /// </summary>
        public static FixedQuaternion Lerp(FixedQuaternion a, FixedQuaternion b, Fixed64 t)
        {
            t = FixedMath.Clamp01(t);

            FixedQuaternion result;
            Fixed64 oneMinusT = FixedMath.One - t;
            result.x = a.x * oneMinusT + b.x * t;
            result.y = a.y * oneMinusT + b.y * t;
            result.z = a.z * oneMinusT + b.z * t;
            result.w = a.w * oneMinusT + b.w * t;

            result.Normalize();

            return result;
        }

        /// <summary>
        ///  Calculates the spherical linear interpolation, which results in a smoother and more accurate rotation interpolation
        /// </summary>
        public static FixedQuaternion Slerp(FixedQuaternion a, FixedQuaternion b, Fixed64 t)
        {
            t = FixedMath.Clamp01(t);

            Fixed64 cosOmega = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

            // If the dot product is negative, negate one of the input quaternions.
            // This ensures that the interpolation takes the shortest path around the sphere.
            if (cosOmega < FixedMath.Zero)
            {
                b.x = -b.x;
                b.y = -b.y;
                b.z = -b.z;
                b.w = -b.w;
                cosOmega = -cosOmega;
            }

            Fixed64 k0, k1;

            // If the quaternions are close, use linear interpolation
            if (cosOmega > FixedMath.One - FixedMath.Epsilon)
            {
                k0 = FixedMath.One - t;
                k1 = t;
            }
            else
            {
                // Otherwise, use spherical linear interpolation
                Fixed64 sinOmega = FixedMath.Sqrt(FixedMath.One - cosOmega * cosOmega);
                Fixed64 omega = FixedMath.Atan2(sinOmega, cosOmega);

                k0 = FixedMath.Sin((FixedMath.One - t) * omega) / sinOmega;
                k1 = FixedMath.Sin(t * omega) / sinOmega;
            }

            FixedQuaternion result;
            result.x = a.x * k0 + b.x * k1;
            result.y = a.y * k0 + b.y * k1;
            result.z = a.z * k0 + b.z * k1;
            result.w = a.w * k0 + b.w * k1;

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
            dot = FixedMath.Clamp(dot, -FixedMath.One, FixedMath.One);

            // Calculate the angle between the two quaternions using the inverse cosine (arccos)
            // arccos(dot(a, b)) gives us the angle in radians, so we convert it to degrees
            Fixed64 angleInRadians = FixedMath.Acos(dot);

            // Convert the angle from radians to degrees
            Fixed64 angleInDegrees = angleInRadians * FixedMath.Rad2Deg;

            return angleInDegrees;
        }

        public static FixedQuaternion AngleAxis(Fixed64 angle, Vector3d axis)
        {
            // Convert the angle to radians
            angle = angle.ToRadians();

            // Normalize the axis
            axis = axis.MyNormalized;

            // Use the half-angle formula (sin(theta / 2), cos(theta / 2))
            Fixed64 halfAngle = angle / FixedMath.Double;
            Fixed64 sinHalfAngle = FixedMath.Sin(halfAngle);
            Fixed64 cosHalfAngle = FixedMath.Cos(halfAngle);

            return new FixedQuaternion(
                axis.x * sinHalfAngle,
                axis.y * sinHalfAngle,
                axis.z * sinHalfAngle,
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
        public static Fixed64 Dot(FixedQuaternion a, FixedQuaternion b)
        {
            return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        }

        #region Operators

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedQuaternion operator *(FixedQuaternion a, FixedQuaternion b)
        {
            return new FixedQuaternion(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedQuaternion operator *(FixedQuaternion q, Fixed64 scalar)
        {
            return new FixedQuaternion(q.x * scalar, q.y * scalar, q.z * scalar, q.w * scalar);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedQuaternion operator *(Fixed64 scalar, FixedQuaternion q)
        {
            return new FixedQuaternion(q.x * scalar, q.y * scalar, q.z * scalar, q.w * scalar);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedQuaternion operator /(FixedQuaternion q, Fixed64 scalar)
        {
            return new FixedQuaternion(q.x / scalar, q.y / scalar, q.z / scalar, q.w / scalar);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedQuaternion operator /(Fixed64 scalar, FixedQuaternion q)
        {
            return new FixedQuaternion(q.x / scalar, q.y / scalar, q.z / scalar, q.w / scalar);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedQuaternion operator +(FixedQuaternion q1, FixedQuaternion q2)
        {
            return new FixedQuaternion(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(FixedQuaternion left, FixedQuaternion right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(FixedQuaternion left, FixedQuaternion right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is FixedQuaternion other && Equals(other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(FixedQuaternion other)
        {
            return other.x == x && other.y == y && other.z == z && other.w == w;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return x.GetHashCode() ^ y.GetHashCode() << 2 ^ z.GetHashCode() >> 2 ^ w.GetHashCode();
        }

        #endregion
    }
}