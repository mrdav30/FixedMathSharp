using MessagePack;
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a quaternion (x, y, z, w) with fixed-point numbers.
    /// Quaternions are useful for representing rotations and can be used to perform smooth rotations and avoid gimbal lock.
    /// </summary>
    [Serializable]
    [MessagePackObject]
    public struct FixedQuaternion : IEquatable<FixedQuaternion>
    {
        #region Fields and Constants

        [Key(0)]
        public Fixed64 x;

        [Key(1)]
        public Fixed64 y;

        [Key(2)]
        public Fixed64 z;

        [Key(3)]
        public Fixed64 w;

        /// <summary>
        /// Identity quaternion (0, 0, 0, 1).
        /// </summary>
        public static readonly FixedQuaternion Identity = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        /// <summary>
        /// Empty quaternion (0, 0, 0, 0).
        /// </summary>
        public static readonly FixedQuaternion Zero = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        #endregion

        #region Constructors

        /// <summary>
        /// Creates a new FixedQuaternion with the specified components.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FixedQuaternion(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        #endregion

        #region Properties and Methods (Instance)

        /// <summary>
        /// Normalized version of this quaternion.
        /// </summary>
        [IgnoreMember]
        public FixedQuaternion Normal
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => GetNormalized(this);
        }

        /// <summary>
        /// Returns the Euler angles (in degrees) of this quaternion.
        /// </summary>
        [IgnoreMember]
        public Vector3d EulerAngles
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => ToEulerAngles();
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set => this = FromEulerAnglesInDegrees(value.x, value.y, value.z);
        }

        [IgnoreMember]
        public Fixed64 this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return index switch
                {
                    0 => x,
                    1 => y,
                    2 => z,
                    3 => w,
                    _ => throw new IndexOutOfRangeException("Invalid FixedQuaternion index!"),
                };
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                switch (index)
                {
                    case 0:
                        x = value;
                        break;
                    case 1:
                        y = value;
                        break;
                    case 2:
                        z = value;
                        break;
                    case 3:
                        w = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException("Invalid FixedQuaternion index!");
                }
            }
        }

        /// <summary>
        /// Set x, y, z and w components of an existing Quaternion.
        /// </summary>
        /// <param name="newX"></param>
        /// <param name="newY"></param>
        /// <param name="newZ"></param>
        /// <param name="newW"></param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Set(Fixed64 newX, Fixed64 newY, Fixed64 newZ, Fixed64 newW)
        {
            x = newX;
            y = newY;
            z = newZ;
            w = newW;
        }

        /// <summary>
        /// Normalizes this quaternion in place.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FixedQuaternion Normalize()
        {
            return this = GetNormalized(this);
        }

        /// <summary>
        /// Returns the conjugate of this quaternion (inverses the rotational effect).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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
            if (norm == Fixed64.Zero) return this; // Handle division by zero by returning the same quaternion

            Fixed64 invNorm = Fixed64.One / norm;
            return new FixedQuaternion(x * -invNorm, y * -invNorm, z * -invNorm, w * invNorm);
        }

        /// <summary>
        /// Rotates a vector by this quaternion.
        /// </summary>
        public Vector3d Rotate(Vector3d v)
        {
            FixedQuaternion normalizedQuat = Normal;
            FixedQuaternion vQuat = new FixedQuaternion(v.x, v.y, v.z, Fixed64.Zero);
            FixedQuaternion invQuat = normalizedQuat.Inverse();
            FixedQuaternion rotatedVQuat = normalizedQuat * vQuat * invQuat;
            return new Vector3d(rotatedVQuat.x, rotatedVQuat.y, rotatedVQuat.z).Normalize();
        }

        /// <summary>
        /// Rotates this quaternion by a given angle around a specified axis (default: Y-axis).
        /// </summary>
        /// <param name="sin">Sine of the rotation angle.</param>
        /// <param name="cos">Cosine of the rotation angle.</param>
        /// <param name="axis">The axis to rotate around (default: Vector3d.Up).</param>
        /// <returns>A new quaternion representing the rotated result.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FixedQuaternion Rotated(Fixed64 sin, Fixed64 cos, Vector3d? axis = null)
        {
            Vector3d rotateAxis = axis ?? Vector3d.Up;

            // The rotation angle is the arc tangent of sin and cos
            Fixed64 angle = FixedMath.Atan2(sin, cos);

            // Construct a quaternion representing a rotation around the axis (default is y aka Vector3d.up)
            FixedQuaternion rotationQuat = FromAxisAngle(rotateAxis, angle);

            // Apply the rotation and return the result
            return rotationQuat * this;
        }

        #endregion

        #region Quaternion Operations

        /// <summary>
        /// Checks if this vector has been normalized by checking if the magnitude is close to 1.
        /// </summary>
        public bool IsNormalized()
        {
            Fixed64 mag = GetMagnitude(this);
            return FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon;
        }

        public static Fixed64 GetMagnitude(FixedQuaternion q)
        {
            Fixed64 mag = (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
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
            if (mag == Fixed64.One)
                return q;

            // Normalize it exactly
            return new FixedQuaternion(
                q.x / mag,
                q.y / mag,
                q.z / mag,
                q.w / mag
            );
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

            return FromMatrix(new Fixed3x3(right.x, up.x, forwardNormalized.x,
                                            right.y, up.y, forwardNormalized.y,
                                            right.z, up.z, forwardNormalized.z));
        }

        /// <summary>
        /// Converts a rotation matrix into a quaternion representation.
        /// </summary>
        /// <param name="matrix">The rotation matrix to convert.</param>
        /// <returns>A quaternion representing the same rotation as the matrix.</returns>
        public static FixedQuaternion FromMatrix(Fixed3x3 matrix)
        {
            Fixed64 trace = matrix.m00 + matrix.m11 + matrix.m22;

            Fixed64 w, x, y, z;

            if (trace > Fixed64.Zero)
            {
                Fixed64 s = FixedMath.Sqrt(trace + Fixed64.One);
                w = s * Fixed64.Half;
                s = Fixed64.Half / s;
                x = (matrix.m21 - matrix.m12) * s;
                y = (matrix.m02 - matrix.m20) * s;
                z = (matrix.m10 - matrix.m01) * s;
            }
            else if (matrix.m00 > matrix.m11 && matrix.m00 > matrix.m22)
            {
                Fixed64 s = FixedMath.Sqrt(Fixed64.One + matrix.m00 - matrix.m11 - matrix.m22);
                x = s * Fixed64.Half;
                s = Fixed64.Half / s;
                y = (matrix.m10 + matrix.m01) * s;
                z = (matrix.m02 + matrix.m20) * s;
                w = (matrix.m21 - matrix.m12) * s;
            }
            else if (matrix.m11 > matrix.m22)
            {
                Fixed64 s = FixedMath.Sqrt(Fixed64.One + matrix.m11 - matrix.m00 - matrix.m22);
                y = s * Fixed64.Half;
                s = Fixed64.Half / s;
                z = (matrix.m21 + matrix.m12) * s;
                x = (matrix.m10 + matrix.m01) * s;
                w = (matrix.m02 - matrix.m20) * s;
            }
            else
            {
                Fixed64 s = FixedMath.Sqrt(Fixed64.One + matrix.m22 - matrix.m00 - matrix.m11);
                z = s * Fixed64.Half;
                s = Fixed64.Half / s;
                x = (matrix.m02 + matrix.m20) * s;
                y = (matrix.m21 + matrix.m12) * s;
                w = (matrix.m10 - matrix.m01) * s;
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

            var rotationMatrix = new Fixed3x3(
                matrix.m00, matrix.m01, matrix.m02,
                matrix.m10, matrix.m11, matrix.m12,
                matrix.m20, matrix.m21, matrix.m22
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
            // Compute the rotation axis as the cross product of the standard forward vector and the desired direction
            Vector3d axis = Vector3d.Cross(Vector3d.Forward, direction);
            Fixed64 axisLength = axis.Magnitude;

            // If the axis length is very close to zero, it means that the desired direction is almost equal to the standard forward vector
            if (axisLength.Abs() == Fixed64.Zero)
                return Identity;  // Return the identity quaternion if no rotation is needed

            // Normalize the rotation axis
            axis = (axis / axisLength).Normal;

            // Compute the angle between the standard forward vector and the desired direction
            Fixed64 angle = FixedMath.Acos(Vector3d.Dot(Vector3d.Forward, direction));

            // Compute the rotation quaternion from the axis and angle
            return FromAxisAngle(axis, angle);
        }

        /// <summary>
        /// Creates a quaternion representing a rotation around a specified axis by a given angle.
        /// </summary>
        /// <param name="axis">The axis to rotate around (must be normalized).</param>
        /// <param name="angle">The rotation angle in radians.</param>
        /// <returns>A quaternion representing the rotation.</returns>
        public static FixedQuaternion FromAxisAngle(Vector3d axis, Fixed64 angle)
        {
            // Check if the axis is a unit vector
            if (!axis.IsNormalized())
                axis = axis.Normalize();

            // Check if the angle is in a valid range (-pi, pi)
            if (angle < -FixedMath.PI || angle > FixedMath.PI)
                throw new ArgumentOutOfRangeException("Angle must be in the range (-pi, pi)");

            Fixed64 halfAngle = angle / Fixed64.Two;  // Half-angle formula
            Fixed64 sinHalfAngle = FixedMath.Sin(halfAngle);
            Fixed64 cosHalfAngle = FixedMath.Cos(halfAngle);

            return new FixedQuaternion(axis.x * sinHalfAngle, axis.y * sinHalfAngle, axis.z * sinHalfAngle, cosHalfAngle);
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
            return FromEulerAngles(pitch, yaw, roll).Normalize();
        }

        /// <summary>
        /// Converts Euler angles (pitch, yaw, roll) to a quaternion and normalizes the result afterwards. Assumes the input angles are in radians.
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

            Fixed64 c1 = FixedMath.Cos(yaw / Fixed64.Two);
            Fixed64 s1 = FixedMath.Sin(yaw / Fixed64.Two);
            Fixed64 c2 = FixedMath.Cos(roll / Fixed64.Two);
            Fixed64 s2 = FixedMath.Sin(roll / Fixed64.Two);
            Fixed64 c3 = FixedMath.Cos(pitch / Fixed64.Two);
            Fixed64 s3 = FixedMath.Sin(pitch / Fixed64.Two);

            Fixed64 c1c2 = c1 * c2;
            Fixed64 s1s2 = s1 * s2;

            Fixed64 w = c1c2 * c3 - s1s2 * s3;
            Fixed64 x = c1c2 * s3 + s1s2 * c3;
            Fixed64 y = s1 * c2 * c3 + c1 * s2 * s3;
            Fixed64 z = c1 * s2 * c3 - s1 * c2 * s3;

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
            Vector3d v = new Vector3d(q.x, q.y, q.z);
            Fixed64 vLength = v.Magnitude;

            // If rotation is very small, avoid division by zero
            if (vLength < Fixed64.FromRaw(0x00001000L)) // Small epsilon
                return Vector3d.Zero;

            // Compute angle (theta = 2 * acos(w))
            Fixed64 theta = Fixed64.Two * FixedMath.Acos(q.w);

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

            FixedQuaternion result;
            Fixed64 oneMinusT = Fixed64.One - t;
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
            if (cosOmega < Fixed64.Zero)
            {
                b.x = -b.x;
                b.y = -b.y;
                b.z = -b.z;
                b.w = -b.w;
                cosOmega = -cosOmega;
            }

            Fixed64 k0, k1;

            // If the quaternions are close, use linear interpolation
            if (cosOmega > Fixed64.One - Fixed64.Precision)
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

        #endregion

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

        #endregion

        #region Conversion

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
            Fixed64 cosr_cosp = Fixed64.One - 2 * (x * x + y * y);
            Fixed64 roll = FixedMath.Atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            Fixed64 sinp = 2 * (w * y - z * x);
            Fixed64 pitch;
            if (sinp.Abs() >= Fixed64.One)
                pitch = FixedMath.CopySign(FixedMath.PiOver2, sinp); // use 90 degrees if out of range
            else
                pitch = FixedMath.Asin(sinp);

            // yaw (z-axis rotation)
            Fixed64 siny_cosp = 2 * (w * z + x * y);
            Fixed64 cosy_cosp = Fixed64.One - 2 * (y * y + z * z);
            Fixed64 yaw = FixedMath.Atan2(siny_cosp, cosy_cosp);

            // Convert radians to degrees
            roll = FixedMath.RadToDeg(roll);
            pitch = FixedMath.RadToDeg(pitch);
            yaw = FixedMath.RadToDeg(yaw);

            return new Vector3d(roll, pitch, yaw);
        }

        /// <summary>
        /// Converts this FixedQuaternion to a direction vector.
        /// </summary>
        /// <returns>A Vector3d representing the direction equivalent to this FixedQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d ToDirection()
        {
            return new Vector3d(
                2 * (x * z - w * y),
                2 * (y * z + w * x),
                Fixed64.One - 2 * (x * x + y * y)
            );
        }

        /// <summary>
        /// Converts the quaternion into a 3x3 rotation matrix.
        /// </summary>
        /// <returns>A FixedMatrix3x3 representing the same rotation as the quaternion.</returns>
        public Fixed3x3 ToMatrix3x3()
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

            Fixed3x3 result = new Fixed3x3();
            Fixed64 scale = Fixed64.One * 2;

            result.m00 = Fixed64.One - scale * (y2 + z2);
            result.m01 = scale * (xy - zw);
            result.m02 = scale * (xz + yw);

            result.m10 = scale * (xy + zw);
            result.m11 = Fixed64.One - scale * (x2 + z2);
            result.m12 = scale * (yz - xw);

            result.m20 = scale * (xz - yw);
            result.m21 = scale * (yz + xw);
            result.m22 = Fixed64.One - scale * (x2 + y2);

            return result;
        }

        #endregion

        #region Equality and HashCode Overrides

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

        /// <summary>
        /// Returns a formatted string for this quaternion.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"({x}, {y}, {z}, {w})";
        }

        #endregion
    }
}