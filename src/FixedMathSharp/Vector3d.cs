using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    [Serializable]
    public partial struct Vector3d : IEquatable<Vector3d>
    {
        #region Properties

        public Fixed64 x, y, z;

        /// <summary>
        ///  Provides a rotated version of the current vector, where rotation is a 90 degrees rotation around the Y axis in the counter-clockwise direction.
        /// </summary>
        /// <remarks>
        /// These operations rotate the vector 90 degrees around the Y-axis.
        /// Note that the positive direction of rotation is defined by the right-hand rule:
        /// If your right hand's thumb points in the positive Y direction, then your fingers curl in the positive direction of rotation.
        /// </remarks>
        public Vector3d RightHandNormal => new Vector3d(z, y, -x);

        /// <summary>
        /// Provides a rotated version of the current vector, where rotation is a 90 degrees rotation around the Y axis in the clockwise direction.
        /// </summary>
        public Vector3d LeftHandNormal => new Vector3d(-z, y, x);

        public Vector3d MyNormalized => Normalize(this);

        // Returns the actual length of this vector (RO).
        public Fixed64 MyMagnitude => Magnitude(this);

        public Vector3d MyForward
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
        /// This vector's square magnitude.
        /// If you're doing distance checks, use SqrMagnitude and square the distance you're checking against
        /// If you need to know the actual distance, use MyMagnitude
        /// </summary>
        /// <returns>The magnitude.</returns>
        public Fixed64 SqrMagnitude => (x * x) + (y * y) + (z * z);

        public long LongStateHash => (x.RawValue * 31) + (y.RawValue * 7) + (z.RawValue * 11);
        public int StateHash => (int)(LongStateHash % int.MaxValue);

        #endregion

        #region constructors

        public Vector3d(int xInt, int yInt, int zInt)
        {
            x = (Fixed64)xInt;
            y = (Fixed64)yInt;
            z = (Fixed64)zInt;
        }

        public Vector3d(Fixed64 xLong, Fixed64 yLong, Fixed64 zLong)
        {
            x = xLong;
            y = yLong;
            z = zLong;
        }

        public Vector3d(float xDoub, float yDoub, float zDoub)
        {
            x = (Fixed64)xDoub;
            y = (Fixed64)yDoub;
            z = (Fixed64)zDoub;
        }

        public Vector3d(double xDoub, double yDoub, double zDoub)
        {
            x = (Fixed64)xDoub;
            y = (Fixed64)yDoub;
            z = (Fixed64)zDoub;
        }

        #endregion

        #region Local Math

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
        public Vector3d NormalizeInPlace()
        {
            Fixed64 mag = MyMagnitude;
            if (mag > FixedMath.Zero && mag != FixedMath.One)
            {
                x /= mag;
                y /= mag;
                z /= mag;
            }
            return this;
        }

        /// <summary>
        /// Normalizes this vector in place and outputs its original magnitude.
        /// </summary>
        /// <param name="mag">The original magnitude of the vector before normalization.</param>
        /// <remarks>
        /// If the vector is zero-length or already normalized, no operation is performed, but the original magnitude will still be output.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d NormalizeInPlace(out Fixed64 mag)
        {
            mag = MyMagnitude;
            if (mag > FixedMath.Zero && mag != FixedMath.One)
            {
                x /= mag;
                y /= mag;
                z /= mag;
            }
            return this;
        }

        public bool IsNormalized()
        {
            return MyMagnitude.Round() - FixedMath.One == FixedMath.Zero;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Distance(Vector3d other)
        {
            return Distance(other.x, other.y, other.z);
        }

        /// <summary>
        /// Does not normalize the calculations to fixed-point numbers. Can be used for distance comparisons but scales quadratically.
        /// </summary>
        /// <returns>The FastDistance.</returns>
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 SqrDistance(Vector3d other)
        {
            return SqrDistance(other.x, other.y, other.z);
        }

        /// <summary>
        /// Are all components of this vector equal to zero?
        /// </summary>
        /// <returns></returns>
        public bool EqualsZero()
        {
            return x == FixedMath.Zero && y == FixedMath.Zero && z == FixedMath.Zero;
        }

        public bool GreaterThanTolerance(Fixed64 tolerance)
        {
            return x > tolerance && y > tolerance && z > tolerance;
        }

        /// <summary>
        /// Used for rotation vectors that aren't exact
        /// </summary>
        /// <returns></returns>
        public bool NotZero()
        {
            return x.MoreThanEpsilon() || y.MoreThanEpsilon() || z.MoreThanEpsilon();
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Dot(Vector3d other)
        {
            return Dot(other.x, other.y, other.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Cross(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ)
        {
            return (y * otherZ - z * otherY) + (z * otherX - x * otherZ) + (x * otherY - y * otherX);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Cross(Vector3d other)
        {
            return Cross(other.x, other.y, other.z);
        }

        /// <summary>
        /// Returns the cross product of this vector with another vector.
        /// </summary>
        /// <param name="rhs">The right-hand side vector to compute the cross product with.</param>
        /// <returns>A new vector representing the cross product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d CrossProduct(Vector3d rhs)
        {
            return new Vector3d(
                y * rhs.z - z * rhs.y,
                z * rhs.x - x * rhs.z,
                x * rhs.y - y * rhs.x);
        }

        #endregion

        #region Static Math

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
            if (dv > v.MyMagnitude)
                return b;
            else
                return a + v.MyNormalized * dv;
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
            FixedMath.Clamp(dot, -FixedMath.One, FixedMath.One);
            // Acos(dot) returns the angle between start and end,
            // And multiplying that by percent returns the angle between
            // start and the final result.
            Fixed64 theta = FixedMath.Acos(dot) * percent;
            Vector3d RelativeVec = end - start * dot;
            RelativeVec.NormalizeInPlace();
            // Orthonormal basis
            // The final result.
            return (start * FixedMath.Cos(theta)) + (RelativeVec * FixedMath.Sin(theta));
        }

        public static Vector3d Normalize(Vector3d value)
        {
            Fixed64 mag = Magnitude(value);
            if (mag > FixedMath.Epsilon && mag != FixedMath.One)
            {
                Fixed64 temp1 = (value.x / mag);
                Fixed64 temp2 = (value.y / mag);
                Fixed64 temp3 = (value.z / mag);

                return new Vector3d(temp1, temp2, temp3);
            }

            return value;
        }

        /// <summary>
        /// Returns the magnitude (length) of this vector.
        /// </summary>
        /// <param name="vector">The vector whose magnitude is being calculated.</param>
        /// <returns>The magnitude of the vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Magnitude(Vector3d vector)
        {
            Fixed64 temp1 = (vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z);
            return temp1 > FixedMath.Epsilon ? FixedMath.Sqrt(temp1) : FixedMath.Zero;
        }

        public static Vector3d Abs(Vector3d value)
        {
            return new Vector3d(value.x.Abs(), value.y.Abs(), value.z.Abs());
        }

        public static Vector3d Clamp(Vector3d value, Vector3d min, Vector3d max)
        {
            return new Vector3d(
                FixedMath.Clamp(value.x, min.x, max.x),
                FixedMath.Clamp(value.y, min.y, max.y),
                FixedMath.Clamp(value.z, min.z, max.z)
            );
        }

        public static bool AreParallel(Vector3d v1, Vector3d v2)
        {
            return Cross(v1, v2).SqrMagnitude.Abs() < FixedMath.Epsilon;
        }

        public static bool AreAlmostParallel(Vector3d v1, Vector3d v2, Fixed64 cosThreshold)
        {
            // Assuming v1 and v2 are already normalized
            Fixed64 dot = Dot(v1, v2);

            // Compare dot product directly to the cosine threshold
            return dot >= cosThreshold;
        }

        public static Vector3d Midpoint(Vector3d v1, Vector3d v2)
        {
            return new Vector3d((v1.x + v2.x) * FixedMath.Half, (v1.y + v2.y) * FixedMath.Half, (v1.z + v2.z) * FixedMath.Half);
        }

        /// <summary>
        /// Computes the distance between two vectors using the Euclidean distance formula.
        /// </summary>
        /// <param name="start">The starting vector.</param>
        /// <param name="end">The ending vector.</param>
        /// <returns>The Euclidean distance between the two vectors.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Distance(Vector3d start, Vector3d end)
        {
            Fixed64 tX = end.x - start.x;
            tX *= tX;
            Fixed64 tY = end.y - start.y;
            tY *= tY;
            Fixed64 tZ = end.z - start.z;
            tZ *= tZ;
            return FixedMath.Sqrt(tX + tY + tZ);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 SqrDistance(Vector3d start, Vector3d end)
        {
            Fixed64 tX = end.x - start.x;
            tX *= tX;
            Fixed64 tY = end.y - start.y;
            tY *= tY;
            Fixed64 tZ = end.z - start.z;
            tZ *= tZ;
            return tX + tY + tZ;
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
            if (D < FixedMath.Epsilon)
            {
                // the lines are almost parallel
                sc = FixedMath.Zero;
                tc = (b > c ? d / b : e / c); // use the largest denominator
            }
            else
            {
                sc = (b * e - c * d) / D;
                tc = (a * e - b * d) / D;
            }

            // recompute sc if it is outside [0,1]
            if (sc < FixedMath.Zero)
            {
                sc = FixedMath.Zero;
                tc = (e < FixedMath.Zero ? FixedMath.Zero : (e > c ? FixedMath.One : e / c));
            }
            else if (sc > FixedMath.One)
            {
                sc = FixedMath.One;
                tc = (e + b < FixedMath.Zero ? FixedMath.Zero : (e + b > c ? FixedMath.One : (e + b) / c));
            }

            // recompute tc if it is outside [0,1]
            if (tc < FixedMath.Zero)
            {
                tc = FixedMath.Zero;
                sc = (-d < FixedMath.Zero ? FixedMath.Zero : (-d > a ? FixedMath.One : -d / a));
            }
            else if (tc > FixedMath.One)
            {
                tc = FixedMath.One;
                sc = ((-d + b) < FixedMath.Zero ? FixedMath.Zero : ((-d + b) > a ? FixedMath.One : (-d + b) / a));
            }

            // get the difference of the two closest points
            Vector3d pointOnLine1 = line1Start + sc * u;
            Vector3d pointOnLine2 = line2Start + tc * v;

            return (pointOnLine1, pointOnLine2);
        }

        public static Vector3d ClosestPointOnLineSegment(Vector3d a, Vector3d b, Vector3d p)
        {
            Vector3d ab = b - a;
            Fixed64 t = Dot(p - a, ab) / Dot(ab, ab);
            t = FixedMath.Max(FixedMath.Zero, FixedMath.Min(FixedMath.One, t));
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
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
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
            return new Vector3d(
                lhs.y * rhs.z - lhs.z * rhs.y,
                lhs.z * rhs.x - lhs.x * rhs.z,
                lhs.x * rhs.y - lhs.y * rhs.x);
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
            if (sqrMag < FixedMath.Epsilon)
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
            if (sqrMag < FixedMath.Epsilon)
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

            if (denominator.Abs() < FixedMath.Epsilon)
                return FixedMath.Zero;

            Fixed64 dot = FixedMath.Clamp(Dot(from, to) / denominator, -FixedMath.One, FixedMath.One);

            return FixedMath.Acos(dot) * FixedMath.Rad2Deg;
        }

        /// <summary>
        ///  Returns a vector whose elements are the maximum of each of the pairs of elements in two specified vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The maximized vector.</returns>
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
        public static Vector3d Min(Vector3d value1, Vector3d value2)
        {
            return new Vector3d((value1.x < value2.x) ? value1.x : value2.x, (value1.y < value2.y) ? value1.y : value2.y, (value1.z < value2.z) ? value1.z : value2.z);
        }

        #endregion

        #region Convert

        public override string ToString()
        {
            return string.Format("({0}, {1}, {2})", x.ToFormattedDouble(), y.ToFormattedDouble(), z.ToFormattedDouble());
        }

        public Vector2d ToVector2d()
        {
            return new Vector2d(x, z);
        }

        #endregion

        #region Operators

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator +(Vector3d v1, Fixed64 add)
        {
            return new Vector3d(v1.x + add, v1.y + add, v1.z + add);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Vector3d v1, Fixed64 sub)
        {
            return new Vector3d(v1.x - sub, v1.y - sub, v1.z - sub);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator -(Vector3d v1)
        {
            return new Vector3d(v1.x * -FixedMath.One, v1.y * -FixedMath.One, v1.z * -FixedMath.One);
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
        public static Vector3d operator *(FixedMatrix3x3 matrix, Vector3d vector)
        {
            return new Vector3d(
                matrix.m00 * vector.x + matrix.m01 * vector.y + matrix.m02 * vector.z,
                matrix.m10 * vector.x + matrix.m11 * vector.y + matrix.m12 * vector.z,
                matrix.m20 * vector.x + matrix.m21 * vector.y + matrix.m22 * vector.z
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Vector3d vector, FixedMatrix3x3 matrix)
        {
            return matrix * vector;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(FixedMatrix4x4 matrix, Vector3d vector)
        {
            return new Vector3d(
                matrix.m00 * vector.x + matrix.m01 * vector.y + matrix.m02 * vector.z + matrix.m03,
                matrix.m10 * vector.x + matrix.m11 * vector.y + matrix.m12 * vector.z + matrix.m13,
                matrix.m20 * vector.x + matrix.m21 * vector.y + matrix.m22 * vector.z + matrix.m23);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator *(Vector3d vector, FixedMatrix4x4 matrix)
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
            return div == FixedMath.Zero ? Zero : new Vector3d(v1.x / div, v1.y / div, v1.z / div);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d operator /(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(
                v2.x == FixedMath.Zero ? FixedMath.Zero : v1.x / v2.x,
                v2.y == FixedMath.Zero ? FixedMath.Zero : v1.y / v2.y,
                v2.z == FixedMath.Zero ? FixedMath.Zero : v1.z / v2.z);
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
                (FixedMath.One - (num5 + num6)) * point.x + (num7 - num12) * point.y + (num8 + num11) * point.z,
                (num7 + num12) * point.x + (FixedMath.One - (num4 + num6)) * point.y + (num9 - num10) * point.z,
                (num8 - num11) * point.x + (num9 + num10) * point.y + (FixedMath.One - (num4 + num5)) * point.z
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
        public override bool Equals(object obj)
        {
            return obj is Vector3d other && Equals(other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Vector3d other)
        {
            return other.x == x && other.y == y && other.z == z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return StateHash;
        }

        #endregion
    }
}