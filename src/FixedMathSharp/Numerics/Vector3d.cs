using MessagePack;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a 3D vector with fixed-point precision, supporting a wide range of vector operations such as rotation, scaling, interpolation, and projection.
    /// </summary>
    /// <remarks>
    /// The Vector3d struct is designed for high-precision applications in 3D space, including games, simulations, and physics engines. 
    /// It offers essential operations like addition, subtraction, dot product, cross product, distance calculation, and normalization.
    /// 
    /// Use Cases:
    /// - Modeling 3D positions, directions, and velocities with fixed-point precision.
    /// - Performing vector transformations, including rotations using quaternions.
    /// - Calculating distances, angles, projections, and interpolation between vectors.
    /// - Essential for fixed-point math scenarios where floating-point precision isn't suitable.
    /// </remarks>
    [Serializable]
    [MessagePackObject]
    public partial struct Vector3d : IEquatable<Vector3d>, IComparable<Vector3d>, IEqualityComparer<Vector3d>
    {
        #region Fields and Constants

        /// <summary>
        /// The X component of the vector.
        /// </summary>
        [Key(0)]
        public Fixed64 x;

        /// <summary>
        /// The Y component of the vector.
        /// </summary>
        [Key(1)]
        public Fixed64 y;

        /// <summary>
        /// The Z component of the vector.
        /// </summary>
        [Key(2)]
        public Fixed64 z;

        /// <summary>
        /// The upward direction vector (0, 1, 0).
        /// </summary>
        public static readonly Vector3d Up = new Vector3d(0, 1, 0);

        /// <summary>
        /// (1, 0, 0)
        /// </summary>
        public static readonly Vector3d Right = new Vector3d(1, 0, 0);

        /// <summary>
        /// (0, -1, 0)
        /// </summary>
        public static readonly Vector3d Down = new Vector3d(0, -1, 0);

        /// <summary>
        /// (-1, 0, 0)
        /// </summary>
        public static readonly Vector3d Left = new Vector3d(-1, 0, 0);

        /// <summary>
        /// The forward direction vector (0, 0, 1).
        /// </summary>
        public static readonly Vector3d Forward = new Vector3d(0, 0, 1);

        /// <summary>
        /// (0, 0, -1)
        /// </summary>
        public static readonly Vector3d Backward = new Vector3d(0, 0, -1);

        /// <summary>
        /// (1, 1, 1)
        /// </summary>
        public static readonly Vector3d One = new Vector3d(1, 1, 1);

        /// <summary>
        /// (-1, -1, -1)
        /// </summary>
        public static readonly Vector3d Negative = new Vector3d(-1, -1, -1);

        /// <summary>
        /// (0, 0, 0)
        /// </summary>
        public static readonly Vector3d Zero = new Vector3d(0, 0, 0);

        #endregion

        #region Constructors

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d(int xInt, int yInt, int zInt) : this((Fixed64)xInt, (Fixed64)yInt, (Fixed64)zInt) { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d(double xDoub, double yDoub, double zDoub) : this((Fixed64)xDoub, (Fixed64)yDoub, (Fixed64)zDoub) { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d(Fixed64 xLong, Fixed64 yLong, Fixed64 zLong)
        {
            x = xLong;
            y = yLong;
            z = zLong;
        }

        #endregion

        #region Properties and Methods (Instance)

        /// <summary>
        ///  Provides a rotated version of the current vector, where rotation is a 90 degrees rotation around the Y axis in the counter-clockwise direction.
        /// </summary>
        /// <remarks>
        /// These operations rotate the vector 90 degrees around the Y-axis.
        /// Note that the positive direction of rotation is defined by the right-hand rule:
        /// If your right hand's thumb points in the positive Y direction, then your fingers curl in the positive direction of rotation.
        /// </remarks>
        [IgnoreMember]
        public Vector3d RightHandNormal
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => new Vector3d(z, y, -x);
        }

        /// <summary>
        /// Provides a rotated version of the current vector, where rotation is a 90 degrees rotation around the Y axis in the clockwise direction.
        /// </summary>
        [IgnoreMember]
        public Vector3d LeftHandNormal
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => new Vector3d(-z, y, x);
        }

        /// <inheritdoc cref="GetNormalized(Vector3d)"/>
        [IgnoreMember]
        public Vector3d Normal
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => GetNormalized(this);
        }

        /// <summary>
        /// Returns the actual length of this vector (RO).
        /// </summary>
        [IgnoreMember]
        public Fixed64 Magnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => GetMagnitude(this);
        }

        /// <summary>
        /// Calculates the forward direction vector based on the yaw (x) and pitch (y) angles.
        /// </summary>
        /// <remarks>
        /// This is commonly used to determine the direction an object is facing in 3D space,
        /// where 'x' represents the yaw (horizontal rotation) and 'y' represents the pitch (vertical rotation).
        /// </remarks>
        [IgnoreMember]
        public Vector3d Direction
        {
            get
            {
                Fixed64 temp1 = FixedMath.Cos(x) * FixedMath.Sin(y);
                Fixed64 temp2 = FixedMath.Sin(-x);
                Fixed64 temp3 = FixedMath.Cos(x) * FixedMath.Cos(y);
                return new Vector3d(temp1, temp2, temp3);
            }
        }

        /// <summary>
        /// Are all components of this vector equal to zero?
        /// </summary>
        /// <returns></returns>
        [IgnoreMember]
        public bool IsZero
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => this.Equals(Zero);
        }

        /// <summary>
        /// This vector's square magnitude.
        /// If you're doing distance checks, use SqrMagnitude and square the distance you're checking against
        /// If you need to know the actual distance, use MyMagnitude
        /// </summary>
        /// <returns>The magnitude.</returns>
        [IgnoreMember]
        public Fixed64 SqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (x * x) + (y * y) + (z * z);
        }

        /// <summary>
        /// Returns a long hash of the vector based on its x, y, and z values.
        /// </summary>
        [IgnoreMember]
        public long LongStateHash
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (x.m_rawValue * 31) + (y.m_rawValue * 7) + (z.m_rawValue * 11);
        }

        /// <summary>
        /// Returns a hash of the vector based on its state.
        /// </summary>
        [IgnoreMember]
        public int StateHash
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (int)(LongStateHash % int.MaxValue);
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
                    _ => throw new IndexOutOfRangeException("Invalid Vector3d index!"),
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
                    default:
                        throw new IndexOutOfRangeException("Invalid Vector3d index!");
                }
            }
        }

        /// <summary>
        /// Set x, y and z components of an existing Vector3.
        /// </summary>
        /// <param name="newX"></param>
        /// <param name="newY"></param>
        /// <param name="newZ"></param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d Set(Fixed64 newX, Fixed64 newY, Fixed64 newZ)
        {
            x = newX;
            y = newY;
            z = newZ;
            return this;
        }

        /// <summary>
        /// Adds the specified values to the components of the vector in place and returns the modified vector.
        /// </summary>
        /// <param name="amount">The amount to add to the components.</param>
        /// <returns>The modified vector after addition.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d AddInPlace(Fixed64 amount)
        {
            x += amount;
            y += amount;
            z += amount;
            return this;
        }

        /// <summary>
        /// Adds the specified values to the components of the vector in place and returns the modified vector.
        /// </summary>
        /// <param name="xAmount">The amount to add to the x component.</param>
        /// <param name="yAmount">The amount to add to the y component.</param>
        /// <param name="zAmount">The amount to add to the z component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d AddInPlace(Fixed64 xAmount, Fixed64 yAmount, Fixed64 zAmount)
        {
            x += xAmount;
            y += yAmount;
            z += zAmount;
            return this;
        }

        /// <summary>
        /// Adds the specified vector components to the corresponding components of the in place vector and returns the modified vector.
        /// </summary>
        /// <param name="other">The other vector to add the components.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d AddInPlace(Vector3d other)
        {
            AddInPlace(other.x, other.y, other.z);
            return this;
        }

        /// <summary>
        /// Subtracts the specified value from all components of the vector in place and returns the modified vector.
        /// </summary>
        /// <param name="amount">The amount to subtract from each component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d SubtractInPlace(Fixed64 amount)
        {
            x -= amount;
            y -= amount;
            z -= amount;
            return this;
        }

        /// <summary>
        /// Subtracts the specified values from the components of the vector in place and returns the modified vector.
        /// </summary>
        /// <param name="xAmount">The amount to subtract from the x component.</param>
        /// <param name="yAmount">The amount to subtract from the y component.</param>
        /// <param name="zAmount">The amount to subtract from the z component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d SubtractInPlace(Fixed64 xAmount, Fixed64 yAmount, Fixed64 zAmount)
        {
            x -= xAmount;
            y -= yAmount;
            z -= zAmount;
            return this;
        }

        /// <summary>
        /// Subtracts the specified vector from the components of the vector in place and returns the modified vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d SubtractInPlace(Vector3d other)
        {
            SubtractInPlace(other.x, other.y, other.z);
            return this;
        }

        /// <summary>
        /// Scales the components of the vector by the specified scalar factor in place and returns the modified vector.
        /// </summary>
        /// <param name="scaleFactor">The scalar factor to multiply each component by.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d ScaleInPlace(Fixed64 scaleFactor)
        {
            x *= scaleFactor;
            y *= scaleFactor;
            z *= scaleFactor;
            return this;
        }

        /// <summary>
        /// Scales each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
        /// </summary>
        /// <param name="scale">The vector containing the scale factors for each component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d ScaleInPlace(Vector3d scale)
        {
            x *= scale.x;
            y *= scale.y;
            z *= scale.z;
            return this;
        }

        /// <summary>
        /// Normalizes this vector in place, making its magnitude (length) equal to 1, and returns the modified vector.
        /// </summary>
        /// <remarks>
        /// If the vector is zero-length or already normalized, no operation is performed. 
        /// This method modifies the current vector in place and supports method chaining.
        /// </remarks>
        /// <returns>The normalized vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d Normalize()
        {
            return this = GetNormalized(this);
        }

        /// <summary>
        /// Normalizes this vector in place and outputs its original magnitude.
        /// </summary>
        /// <remarks>
        /// If the vector is zero-length or already normalized, no operation is performed, but the original magnitude will still be output.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d Normalize(out Fixed64 mag)
        {
            mag = GetMagnitude(this);

            // If magnitude is zero, return a zero vector to avoid divide-by-zero errors
            if (mag == Fixed64.Zero)
            {
                x = Fixed64.Zero;
                y = Fixed64.Zero;
                z = Fixed64.Zero;
                return this;
            }

            // If already normalized, return as-is
            if (mag == Fixed64.One)
                return this;

            x /= mag;
            y /= mag;
            z /= mag;

            return this;
        }

        /// <summary>
        /// Checks if this vector has been normalized by checking if the magnitude is close to 1.
        /// </summary>
        public bool IsNormalized()
        {
            return FixedMath.Abs(Magnitude - Fixed64.One) <= Fixed64.Epsilon;
        }

        /// <summary>
        /// Computes the distance between this vector and another vector.
        /// </summary>
        /// <param name="otherX">The x component of the other vector.</param>
        /// <param name="otherY">The y component of the other vector.</param>
        /// <param name="otherZ">The z component of the other vector.</param>
        /// <returns>The distance between the two vectors.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Distance(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ)
        {
            Fixed64 temp1 = x - otherX;
            temp1 *= temp1;
            Fixed64 temp2 = y - otherY;
            temp2 *= temp2;
            Fixed64 temp3 = z - otherZ;
            temp3 *= temp3;
            return FixedMath.Sqrt(temp1 + temp2 + temp3);
        }

        /// <summary>
        /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
        /// </summary>
        /// <returns>The squared distance between the two vectors.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 SqrDistance(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ)
        {
            Fixed64 temp1 = x - otherX;
            temp1 *= temp1;
            Fixed64 temp2 = y - otherY;
            temp2 *= temp2;
            Fixed64 temp3 = z - otherZ;
            temp3 *= temp3;
            return temp1 + temp2 + temp3;
        }

        /// <summary>
        /// Computes the dot product of this vector with another vector specified by its components.
        /// </summary>
        /// <param name="otherX">The x component of the other vector.</param>
        /// <param name="otherY">The y component of the other vector.</param>
        /// <param name="otherZ">The z component of the other vector.</param>
        /// <returns>The dot product of the two vectors.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Dot(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ)
        {
            return x * otherX + y * otherY + z * otherZ;
        }

        /// <summary>
        /// Computes the cross product magnitude of this vector with another vector.
        /// </summary>
        /// <param name="otherX">The X component of the other vector.</param>
        /// <param name="otherY">The Y component of the other vector.</param>
        /// <param name="otherZ">The Z component of the other vector.</param>
        /// <returns>The cross product magnitude.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 CrossProduct(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ)
        {
            return (y * otherZ - z * otherY) + (z * otherX - x * otherZ) + (x * otherY - y * otherX);
        }

        /// <summary>
        /// Returns the cross vector of this vector with another vector.
        /// </summary>
        /// <returns>A new vector representing the cross product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d Cross(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ)
        {
            return new Vector3d(
                y * otherZ - z * otherY,
                z * otherX - x * otherZ,
                x * otherY - y * otherX);
        }

        #endregion

        #region Vector3d Operations

        /// <summary>
        /// Linearly interpolates between two points.
        /// </summary>
        /// <param name="a">Start value, returned when t = 0.</param>
        /// <param name="b">End value, returned when t = 1.</param>
        /// <param name="mag">Value used to interpolate between a and b.</param>
        /// <returns> Interpolated value, equals to a + (b - a) * t.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Lerp(Vector3d a, Vector3d b, Fixed64 mag)
        {
            mag = FixedMath.Clamp01(mag);
            return new Vector3d(a.x + (b.x - a.x) * mag, a.y + (b.y - a.y) * mag, a.z + (b.z - a.z) * mag);
        }

        /// <summary>
        /// Linearly interpolates between two vectors without clamping the interpolation factor between 0 and 1.
        /// </summary>
        /// <param name="a">The start vector.</param>
        /// <param name="b">The end vector.</param>
        /// <param name="t">The interpolation factor. Values outside the range [0, 1] will cause the interpolation to go beyond the start or end points.</param>
        /// <returns>The interpolated vector.</returns>
        /// <remarks>
        /// Unlike traditional Lerp, this function allows interpolation factors greater than 1 or less than 0, 
        /// which means the resulting vector can extend beyond the endpoints.
        /// </remarks>
        public static Vector3d UnclampedLerp(Vector3d a, Vector3d b, Fixed64 t)
        {
            return (b - a) * t + a;
        }

        /// <summary>
        /// Moves from a to b at some speed dependent of a delta time with out passing b.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="speed"></param>
        /// <param name="dt"></param>
        /// <returns></returns>
        public static Vector3d SpeedLerp(Vector3d a, Vector3d b, Fixed64 speed, Fixed64 dt)
        {
            Vector3d v = b - a;
            Fixed64 dv = speed * dt;
            if (dv > v.Magnitude)
                return b;
            else
                return a + v.Normal * dv;
        }

        /// <summary>
        /// Spherically interpolates between two vectors, moving along the shortest arc on a unit sphere.
        /// </summary>
        /// <param name="start">The starting vector.</param>
        /// <param name="end">The ending vector.</param>
        /// <param name="percent">A value between 0 and 1 that represents the interpolation amount. 0 returns the start vector, and 1 returns the end vector.</param>
        /// <returns>The interpolated vector between the two input vectors.</returns>
        /// <remarks>
        /// Slerp is used to interpolate between two unit vectors on a sphere, providing smooth rotation.
        /// It can be more computationally expensive than linear interpolation (Lerp) but results in smoother, arc-like motion.
        /// </remarks>
        public static Vector3d Slerp(Vector3d start, Vector3d end, Fixed64 percent)
        {
            // Dot product - the cosine of the angle between 2 vectors.
            Fixed64 dot = Dot(start, end);
            // Clamp it to be in the range of Acos()
            // This may be unnecessary, but floating point
            // precision can be a fickle mistress.
            FixedMath.Clamp(dot, -Fixed64.One, Fixed64.One);
            // Acos(dot) returns the angle between start and end,
            // And multiplying that by percent returns the angle between
            // start and the final result.
            Fixed64 theta = FixedMath.Acos(dot) * percent;
            Vector3d RelativeVec = end - start * dot;
            RelativeVec.Normalize();
            // Orthonormal basis
            // The final result.
            return (start * FixedMath.Cos(theta)) + (RelativeVec * FixedMath.Sin(theta));
        }

        /// <summary>
        /// Normalizes the given vector, returning a unit vector with the same direction.
        /// </summary>
        /// <param name="value">The vector to normalize.</param>
        /// <returns>A normalized (unit) vector with the same direction.</returns>
        public static Vector3d GetNormalized(Vector3d value)
        {
            Fixed64 mag = GetMagnitude(value);

            // If magnitude is zero, return a zero vector to avoid divide-by-zero errors
            if (mag == Fixed64.Zero)
                return new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

            // If already normalized, return as-is
            if (mag == Fixed64.One)
                return value;

            // Normalize it exactly           
            return new Vector3d(
                value.x / mag,
                value.y / mag,
                value.z / mag
            );
        }

        /// <summary>
        /// Returns the magnitude (length) of this vector.
        /// </summary>
        /// <param name="vector">The vector whose magnitude is being calculated.</param>
        /// <returns>The magnitude of the vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 GetMagnitude(Vector3d vector)
        {
            Fixed64 mag = (vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z);

            // If rounding error pushed magnitude slightly above 1, clamp it
            if (mag > Fixed64.One && mag <= Fixed64.One + Fixed64.Epsilon)
                return Fixed64.One;

            return mag != Fixed64.Zero ? FixedMath.Sqrt(mag) : Fixed64.Zero;
        }

        /// <summary>
        /// Returns a new <see cref="Vector3d"/> where each component is the absolute value of the corresponding input component.
        /// </summary>
        /// <param name="value">The input vector.</param>
        /// <returns>A vector with absolute values for each component.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Abs(Vector3d value)
        {
            return new Vector3d(value.x.Abs(), value.y.Abs(), value.z.Abs());
        }

        /// <summary>
        /// Returns a new <see cref="Vector3d"/> where each component is the sign of the corresponding input component.
        /// </summary>
        /// <param name="value">The input vector.</param>
        /// <returns>A vector where each component is -1, 0, or 1 based on the sign of the input.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Sign(Vector3d value)
        {
            return new Vector3d(value.x.Sign(), value.y.Sign(), value.z.Sign());
        }

        /// <summary>
        /// Clamps each component of the given <see cref="Vector3d"/> within the specified min and max bounds.
        /// </summary>
        /// <param name="value">The vector to clamp.</param>
        /// <param name="min">The minimum bounds.</param>
        /// <param name="max">The maximum bounds.</param>
        /// <returns>A vector with each component clamped between min and max.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Clamp(Vector3d value, Vector3d min, Vector3d max)
        {
            return new Vector3d(
                FixedMath.Clamp(value.x, min.x, max.x),
                FixedMath.Clamp(value.y, min.y, max.y),
                FixedMath.Clamp(value.z, min.z, max.z)
            );
        }

        /// <summary>
        /// Clamps the given Vector3d within the specified magnitude.
        /// </summary>
        /// <param name="value"></param>
        /// <param name="maxMagnitude"></param>
        /// <returns></returns>
        public static Vector3d ClampMagnitude(Vector3d value, Fixed64 maxMagnitude)
        {
            if (value.SqrMagnitude > maxMagnitude * maxMagnitude)
                return value.Normal * maxMagnitude; // Scale vector to max magnitude

            return value;
        }

        /// <summary>
        /// Determines if two vectors are exactly parallel by checking if their cross product is zero.
        /// </summary>
        /// <param name="v1">The first vector.</param>
        /// <param name="v2">The second vector.</param>
        /// <returns>True if the vectors are exactly parallel, false otherwise.</returns>
        public static bool AreParallel(Vector3d v1, Vector3d v2)
        {
            return Cross(v1, v2).SqrMagnitude == Fixed64.Zero;
        }

        /// <summary>
        /// Determines if two vectors are approximately parallel based on a cosine similarity threshold.
        /// </summary>
        /// <param name="v1">The first normalized vector.</param>
        /// <param name="v2">The second normalized vector.</param>
        /// <param name="cosThreshold">The cosine similarity threshold for near-parallel vectors.</param>
        /// <returns>True if the vectors are nearly parallel, false otherwise.</returns>
        public static bool AreAlmostParallel(Vector3d v1, Vector3d v2, Fixed64 cosThreshold)
        {
            // Assuming v1 and v2 are already normalized
            Fixed64 dot = Dot(v1, v2);

            // Compare dot product directly to the cosine threshold
            return dot >= cosThreshold;
        }

        /// <summary>
        /// Computes the midpoint between two vectors.
        /// </summary>
        /// <param name="v1">The first vector.</param>
        /// <param name="v2">The second vector.</param>
        /// <returns>The midpoint vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Midpoint(Vector3d v1, Vector3d v2)
        {
            return new Vector3d((v1.x + v2.x) * Fixed64.Half, (v1.y + v2.y) * Fixed64.Half, (v1.z + v2.z) * Fixed64.Half);
        }

        /// <inheritdoc cref="Distance(Fixed64, Fixed64, Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Distance(Vector3d start, Vector3d end)
        {
            return start.Distance(end.x, end.y, end.z);
        }

        /// <inheritdoc cref="SqrDistance(Fixed64, Fixed64, Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 SqrDistance(Vector3d start, Vector3d end)
        {
            return start.SqrDistance(end.x, end.y, end.z);
        }

        /// <summary>
        /// Calculates the closest points on two line segments.
        /// </summary>
        /// <param name="line1Start">The starting point of the first line segment.</param>
        /// <param name="line1End">The ending point of the first line segment.</param>
        /// <param name="line2Start">The starting point of the second line segment.</param>
        /// <param name="line2End">The ending point of the second line segment.</param>
        /// <returns>
        /// A tuple containing two points representing the closest points on each line segment. The first item is the closest point on the first line,
        /// and the second item is the closest point on the second line.
        /// </returns>
        /// <remarks>
        /// This method considers the line segments, not the infinite lines they represent, ensuring that the returned points always lie within the provided segments.
        /// </remarks>
        public static (Vector3d, Vector3d) ClosestPointsOnTwoLines(Vector3d line1Start, Vector3d line1End, Vector3d line2Start, Vector3d line2End)
        {
            Vector3d u = line1End - line1Start;
            Vector3d v = line2End - line2Start;
            Vector3d w = line1Start - line2Start;

            Fixed64 a = Dot(u, u);
            Fixed64 b = Dot(u, v);
            Fixed64 c = Dot(v, v);
            Fixed64 d = Dot(u, w);
            Fixed64 e = Dot(v, w);
            Fixed64 D = a * c - b * b;

            Fixed64 sc, tc;

            // compute the line parameters of the two closest points
            if (D < Fixed64.Epsilon)
            {
                // the lines are almost parallel
                sc = Fixed64.Zero;
                tc = (b > c ? d / b : e / c); // use the largest denominator
            }
            else
            {
                sc = (b * e - c * d) / D;
                tc = (a * e - b * d) / D;
            }

            // recompute sc if it is outside [0,1]
            if (sc < Fixed64.Zero)
            {
                sc = Fixed64.Zero;
                tc = (e < Fixed64.Zero ? Fixed64.Zero : (e > c ? Fixed64.One : e / c));
            }
            else if (sc > Fixed64.One)
            {
                sc = Fixed64.One;
                tc = (e + b < Fixed64.Zero ? Fixed64.Zero : (e + b > c ? Fixed64.One : (e + b) / c));
            }

            // recompute tc if it is outside [0,1]
            if (tc < Fixed64.Zero)
            {
                tc = Fixed64.Zero;
                sc = (-d < Fixed64.Zero ? Fixed64.Zero : (-d > a ? Fixed64.One : -d / a));
            }
            else if (tc > Fixed64.One)
            {
                tc = Fixed64.One;
                sc = ((-d + b) < Fixed64.Zero ? Fixed64.Zero : ((-d + b) > a ? Fixed64.One : (-d + b) / a));
            }

            // get the difference of the two closest points
            Vector3d pointOnLine1 = line1Start + sc * u;
            Vector3d pointOnLine2 = line2Start + tc * v;

            return (pointOnLine1, pointOnLine2);
        }

        /// <summary>
        /// Finds the closest point on a line segment between points A and B to a given point P.
        /// </summary>
        /// <param name="a">The start of the line segment.</param>
        /// <param name="b">The end of the line segment.</param>
        /// <param name="p">The point to project onto the segment.</param>
        /// <returns>The closest point on the line segment to P.</returns>
        public static Vector3d ClosestPointOnLineSegment(Vector3d a, Vector3d b, Vector3d p)
        {
            Vector3d ab = b - a;
            Fixed64 t = Dot(p - a, ab) / Dot(ab, ab);
            t = FixedMath.Max(Fixed64.Zero, FixedMath.Min(Fixed64.One, t));
            return a + ab * t;
        }

        /// <summary>
        /// Dot Product of two vectors.
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Dot(Vector3d lhs, Vector3d rhs)
        {
            return lhs.Dot(rhs.x, rhs.y, rhs.z);
        }

        /// <summary>
        /// Multiplies two vectors component-wise.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static Vector3d Scale(Vector3d a, Vector3d b)
        {
            return new Vector3d(a.x * b.x, a.y * b.y, a.z * b.z);
        }

        /// <summary>
        /// Cross Product of two vectors.
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Cross(Vector3d lhs, Vector3d rhs)
        {
            return lhs.Cross(rhs.x, rhs.y, rhs.z);
        }

        /// <inheritdoc cref="CrossProduct(Fixed64, Fixed64, Fixed64)"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 CrossProduct(Vector3d lhs, Vector3d rhs)
        {
            return lhs.CrossProduct(rhs.x, rhs.y, rhs.z);
        }

        /// <summary>
        /// Projects a vector onto another vector.
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="onNormal"></param>
        /// <returns></returns>
        public static Vector3d Project(Vector3d vector, Vector3d onNormal)
        {
            Fixed64 sqrMag = Dot(onNormal, onNormal);
            if (sqrMag < Fixed64.Epsilon)
                return Zero;
            else
            {
                Fixed64 dot = Dot(vector, onNormal);
                return new Vector3d(onNormal.x * dot / sqrMag,
                    onNormal.y * dot / sqrMag,
                    onNormal.z * dot / sqrMag);
            }
        }

        /// <summary>
        /// Projects a vector onto a plane defined by a normal orthogonal to the plane.
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="planeNormal"></param>
        /// <returns></returns>
        public static Vector3d ProjectOnPlane(Vector3d vector, Vector3d planeNormal)
        {
            Fixed64 sqrMag = Dot(planeNormal, planeNormal);
            if (sqrMag < Fixed64.Epsilon)
                return vector;
            else
            {
                Fixed64 dot = Dot(vector, planeNormal);
                return new Vector3d(vector.x - planeNormal.x * dot / sqrMag,
                    vector.y - planeNormal.y * dot / sqrMag,
                    vector.z - planeNormal.z * dot / sqrMag);
            }
        }

        /// <summary>
        /// Computes the angle in degrees between two vectors.
        /// </summary>
        /// <param name="from">The starting vector.</param>
        /// <param name="to">The target vector.</param>
        /// <returns>The angle in degrees between the two vectors.</returns>
        /// <remarks>
        /// This method calculates the angle by using the dot product between the vectors and normalizing the result.
        /// The angle is always the smaller angle between the two vectors on a plane.
        /// </remarks>
        public static Fixed64 Angle(Vector3d from, Vector3d to)
        {
            Fixed64 denominator = FixedMath.Sqrt(from.SqrMagnitude * to.SqrMagnitude);

            if (denominator.Abs() < Fixed64.Epsilon)
                return Fixed64.Zero;

            Fixed64 dot = FixedMath.Clamp(Dot(from, to) / denominator, -Fixed64.One, Fixed64.One);

            return FixedMath.RadToDeg(FixedMath.Acos(dot));
        }

        /// <summary>
        ///  Returns a vector whose elements are the maximum of each of the pairs of elements in two specified vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The maximized vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Max(Vector3d value1, Vector3d value2)
        {
            return new Vector3d((value1.x > value2.x) ? value1.x : value2.x, (value1.y > value2.y) ? value1.y : value2.y, (value1.z > value2.z) ? value1.z : value2.z);
        }

        /// <summary>
        /// Returns a vector whose elements are the minimum of each of the pairs of elements in two specified vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The minimized vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Min(Vector3d value1, Vector3d value2)
        {
            return new Vector3d((value1.x < value2.x) ? value1.x : value2.x, (value1.y < value2.y) ? value1.y : value2.y, (value1.z < value2.z) ? value1.z : value2.z);
        }

        /// <summary>
        /// Rotates the vector around a given position using a specified quaternion rotation.
        /// </summary>
        /// <param name="source">The vector to rotate.</param>
        /// <param name="position">The position around which the vector is rotated.</param>
        /// <param name="rotation">The quaternion representing the rotation.</param>
        /// <returns>The rotated vector.</returns>
        public static Vector3d Rotate(Vector3d source, Vector3d position, FixedQuaternion rotation)
        {
            source -= position; // Translate the vector by the position
            var normalizedRotation = rotation.Normal;
            return (normalizedRotation * source) + position;
        }

        /// <summary>
        /// Applies the inverse of a specified quaternion rotation to the vector around a given position.
        /// </summary>
        /// <param name="source">The vector to rotate.</param>
        /// <param name="position">The position around which the vector is rotated.</param>
        /// <param name="rotation">The quaternion representing the inverse rotation.</param>
        /// <returns>The rotated vector.</returns>
        public static Vector3d InverseRotate(Vector3d source, Vector3d position, FixedQuaternion rotation)
        {
            source -= position; // Translate the vector by the position
            var normalizedRotation = rotation.Normal;
            // Undo the rotation
            source = normalizedRotation.Inverse() * source;
            // Add the original position back
            return source + position;
        }

        #endregion

        #region Operators

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +(Vector3d v1, Fixed64 mag)
        {
            return new Vector3d(v1.x + mag, v1.y + mag, v1.z + mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +(Fixed64 mag, Vector3d v1)
        {
            return v1 + mag;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +(Vector3d v1, (int x, int y, int z) v2)
        {
            return new Vector3d(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +((int x, int y, int z) v2, Vector3d v1)
        {
            return v1 + v2;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +(Vector3d v1, (float x, float y, float z) v2)
        {
            return new Vector3d(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +((float x, float y, float z) v1, Vector3d v2)
        {
            return v2 + v1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Vector3d v1, Fixed64 mag)
        {
            return new Vector3d(v1.x - mag, v1.y - mag, v1.z - mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Fixed64 mag, Vector3d v1)
        {
            return new Vector3d(mag - v1.x, mag - v1.y, mag - v1.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Vector3d v1, (int x, int y, int z) v2)
        {
            return new Vector3d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -((int x, int y, int z) v1, Vector3d v2)
        {
            return new Vector3d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Vector3d v1, (float x, float y, float z) v2)
        {
            return new Vector3d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -((float x, float y, float z) v1, Vector3d v2)
        {
            return new Vector3d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Vector3d v1)
        {
            return new Vector3d(v1.x * -Fixed64.One, v1.y * -Fixed64.One, v1.z * -Fixed64.One);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Vector3d v1, Fixed64 mag)
        {
            return new Vector3d(v1.x * mag, v1.y * mag, v1.z * mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Fixed64 mag, Vector3d v1)
        {
            return new Vector3d(v1.x * mag, v1.y * mag, v1.z * mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Vector3d v1, int mag)
        {
            return new Vector3d(v1.x * mag, v1.y * mag, v1.z * mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(int mag, Vector3d v1)
        {
            return new Vector3d(v1.x * mag, v1.y * mag, v1.z * mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Fixed3x3 matrix, Vector3d vector)
        {
            return new Vector3d(
                matrix.m00 * vector.x + matrix.m01 * vector.y + matrix.m02 * vector.z,
                matrix.m10 * vector.x + matrix.m11 * vector.y + matrix.m12 * vector.z,
                matrix.m20 * vector.x + matrix.m21 * vector.y + matrix.m22 * vector.z
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Vector3d vector, Fixed3x3 matrix)
        {
            return matrix * vector;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Fixed4x4 matrix, Vector3d point)
        {
            if (matrix.IsAffine)
            {
                return new Vector3d(
                    matrix.m00 * point.x + matrix.m01 * point.y + matrix.m02 * point.z + matrix.m03 + matrix.m30,
                    matrix.m10 * point.x + matrix.m11 * point.y + matrix.m12 * point.z + matrix.m13 + matrix.m31,
                    matrix.m20 * point.x + matrix.m21 * point.y + matrix.m22 * point.z + matrix.m23 + matrix.m32
                );
            }

            // Full 4×4 transformation
            Fixed64 w = matrix.m03 * point.x + matrix.m13 * point.y + matrix.m23 * point.z + matrix.m33;
            if (w == Fixed64.Zero) w = Fixed64.One;  // Prevent divide-by-zero

            return new Vector3d(
                (matrix.m00 * point.x + matrix.m01 * point.y + matrix.m02 * point.z + matrix.m03 + matrix.m30) / w,
                (matrix.m10 * point.x + matrix.m11 * point.y + matrix.m12 * point.z + matrix.m13 + matrix.m31) / w,
                (matrix.m20 * point.x + matrix.m21 * point.y + matrix.m22 * point.z + matrix.m23 + matrix.m32) / w
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Vector3d vector, Fixed4x4 matrix)
        {
            return matrix * vector;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator /(Vector3d v1, Fixed64 div)
        {
            return div == Fixed64.Zero ? Zero : new Vector3d(v1.x / div, v1.y / div, v1.z / div);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator /(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(
                v2.x == Fixed64.Zero ? Fixed64.Zero : v1.x / v2.x,
                v2.y == Fixed64.Zero ? Fixed64.Zero : v1.y / v2.y,
                v2.z == Fixed64.Zero ? Fixed64.Zero : v1.z / v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator /(Vector3d v1, int div)
        {
            return div == 0 ? Zero : new Vector3d(v1.x / div, v1.y / div, v1.z / div);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator /(Fixed64 div, Vector3d v1)
        {
            return new Vector3d(div / v1.x, div / v1.y, div / v1.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Vector3d point, FixedQuaternion rotation)
        {
            return rotation * point;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(FixedQuaternion rotation, Vector3d point)
        {
            Fixed64 num1 = rotation.x * 2;
            Fixed64 num2 = rotation.y * 2;
            Fixed64 num3 = rotation.z * 2;
            Fixed64 num4 = rotation.x * num1;
            Fixed64 num5 = rotation.y * num2;
            Fixed64 num6 = rotation.z * num3;
            Fixed64 num7 = rotation.x * num2;
            Fixed64 num8 = rotation.x * num3;
            Fixed64 num9 = rotation.y * num3;
            Fixed64 num10 = rotation.w * num1;
            Fixed64 num11 = rotation.w * num2;
            Fixed64 num12 = rotation.w * num3;
            Vector3d vector3 = new Vector3d(
                (Fixed64.One - (num5 + num6)) * point.x + (num7 - num12) * point.y + (num8 + num11) * point.z,
                (num7 + num12) * point.x + (Fixed64.One - (num4 + num6)) * point.y + (num9 - num10) * point.z,
                (num8 - num11) * point.x + (num9 + num10) * point.y + (Fixed64.One - (num4 + num5)) * point.z
            );
            return vector3;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Vector3d left, Vector3d right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Vector3d left, Vector3d right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(Vector3d left, Vector3d right)
        {
            return left.x > right.x
                && left.y > right.y
                && left.z > right.z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(Vector3d left, Vector3d right)
        {
            return left.x < right.x
                && left.y < right.y
                && left.z < right.z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(Vector3d left, Vector3d right)
        {
            return left.x >= right.x
                && left.y >= right.y
                && left.z >= right.z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(Vector3d left, Vector3d right)
        {
            return left.x <= right.x
                && left.y <= right.y
                && left.z <= right.z;
        }

        #endregion

        #region Conversion

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("({0}, {1}, {2})", x.ToFormattedDouble(), y.ToFormattedDouble(), z.ToFormattedDouble());
        }

        /// <summary>
        /// Converts this <see cref="Vector3d"/> to a <see cref="Vector2d"/>, 
        /// dropping the Y component (height) of this vector in the resulting vector.
        /// </summary>
        /// <returns>
        /// A new <see cref="Vector2d"/> where (X, Z) from this <see cref="Vector3d"/> 
        /// become (X, Y) in the resulting vector.
        /// </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d ToVector2d()
        {
            return new Vector2d(x, z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void Deconstruct(out float x, out float y, out float z)
        {
            x = this.x.ToPreciseFloat();
            y = this.y.ToPreciseFloat();
            z = this.z.ToPreciseFloat();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void Deconstruct(out int x, out int y, out int z)
        {
            x = this.x.RoundToInt();
            y = this.y.RoundToInt();
            z = this.z.RoundToInt();
        }

        /// <summary>
        /// Converts each component of the vector from radians to degrees.
        /// </summary>
        /// <param name="radians">The vector with components in radians.</param>
        /// <returns>A new vector with components converted to degrees.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ToDegrees(Vector3d radians)
        {
            return new Vector3d(
                FixedMath.RadToDeg(radians.x),
                FixedMath.RadToDeg(radians.y),
                FixedMath.RadToDeg(radians.z));
        }

        /// <summary>
        /// Converts each component of the vector from degrees to radians.
        /// </summary>
        /// <param name="degrees">The vector with components in degrees.</param>
        /// <returns>A new vector with components converted to radians.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ToRadians(Vector3d degrees)
        {
            return new Vector3d(
                FixedMath.DegToRad(degrees.x),
                FixedMath.DegToRad(degrees.y),
                FixedMath.DegToRad(degrees.z));
        }

        #endregion

        #region Equality and HashCode Overrides

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object? obj)
        {
            return obj is Vector3d other && Equals(other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Vector3d other)
        {
            return other.x == x && other.y == y && other.z == z;
        }

        public bool Equals(Vector3d x, Vector3d y)
        {
            return x.Equals(y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return StateHash;
        }

        public int GetHashCode(Vector3d obj)
        {
            return obj.GetHashCode();
        }

        public int CompareTo(Vector3d other)
        {
            return SqrMagnitude.CompareTo(other.SqrMagnitude);
        }

        #endregion
    }
}