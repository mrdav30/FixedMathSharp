using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    [Serializable]
    public partial struct Vector2d : IEquatable<Vector2d>
    {
        public Fixed64 x, y;

        #region Properties

        /// <summary>
        /// Rotates the vector to the right (90 degrees clockwise).
        /// </summary>
        public Vector2d RotatedRight => new Vector2d(y, -x);

        /// <summary>
        /// Rotates the vector to the left (90 degrees counterclockwise).
        /// </summary>
        public Vector2d RotatedLeft => new Vector2d(-y, x);

        /// <summary>
        /// Gets the right-hand (counter-clockwise) normal vector.
        /// </summary>
        public Vector2d RightHandNormal => new Vector2d(-y, x);

        /// <summary>
        /// Gets the left-hand (clockwise) normal vector.
        /// </summary>
        public Vector2d LeftHandNormal => new Vector2d(y, -x);

        public Vector2d MyNormalized => Normalize(this);
        public Fixed64 MyMagnitude => Magnitude(this);

        /// <summary>
        /// Returns the square magnitude of the vector (avoids calculating the square root).
        /// </summary>
        public Fixed64 SqrMagnitude => x * x + y * y;

        /// <summary>
        /// Returns a long hash of the vector based on its x and y values.
        /// </summary>
        public long LongStateHash => x.RawValue * 31 + y.RawValue * 7;

        /// <summary>
        /// Returns a hash of the vector based on its state.
        /// </summary>
        public int StateHash => (int)(LongStateHash % int.MaxValue);

        #endregion

        #region Constructors

        public Vector2d(Fixed64 xFixed, Fixed64 yFixed)
        {
            x = xFixed;
            y = yFixed;
        }

        public Vector2d(int xInt, int yInt)
        {
            x = (Fixed64)xInt;
            y = (Fixed64)yInt;
        }

        public Vector2d(float xFloat, float yFloat)
        {
            x = (Fixed64)xFloat;
            y = (Fixed64)yFloat;
        }

        public Vector2d(double xDoub, double yDoub)
        {
            x = (Fixed64)xDoub;
            y = (Fixed64)yDoub;
        }

        #endregion

        #region Local Math


        /// <summary>
        /// Adds the specified values to the components of the vector in place and returns the modified vector.
        /// </summary>
        /// <param name="amount">The amount to add to the components.</param>
        /// <returns>The modified vector after addition.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d AddInPlace(Fixed64 amount)
        {
            x += amount;
            y += amount;
            return this;
        }

        /// <summary>
        /// Adds the specified values to the components of the vector in place and returns the modified vector.
        /// </summary>
        /// <param name="xAmount">The amount to add to the x component.</param>
        /// <param name="yAmount">The amount to add to the y component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d AddInPlace(Fixed64 xAmount, Fixed64 yAmount)
        {
            x += xAmount;
            y += yAmount;
            return this;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d AddInPlace(Vector2d other)
        {
            AddInPlace(other.x, other.y);
            return this;
        }

        /// <summary>
        /// Subtracts the specified value from all components of the vector in place and returns the modified vector.
        /// </summary>
        /// <param name="amount">The amount to subtract from each component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d SubtractInPlace(Fixed64 amount)
        {
            x -= amount;
            y -= amount;
            return this;
        }

        /// <summary>
        /// Subtracts the specified values from the components of the vector in place and returns the modified vector.
        /// </summary>
        /// <param name="xAmount">The amount to subtract from the x component.</param>
        /// <param name="yAmount">The amount to subtract from the y component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d SubtractInPlace(Fixed64 xAmount, Fixed64 yAmount)
        {
            x -= xAmount;
            y -= yAmount;
            return this;
        }

        /// <summary>
        /// Subtracts the specified vector from the components of the vector in place and returns the modified vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d Subtract(Vector2d other)
        {
            SubtractInPlace(other.x, other.y);
            return this;
        }

        /// <summary>
        /// Scales the components of the vector by the specified scalar factor in place and returns the modified vector.
        /// </summary>
        /// <param name="scaleFactor">The scalar factor to multiply each component by.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d ScaleInPlace(Fixed64 scaleFactor)
        {
            x *= scaleFactor;
            y *= scaleFactor;
            return this;
        }

        /// <summary>
        /// Scales each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
        /// </summary>
        /// <param name="scale">The vector containing the scale factors for each component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d ScaleInPlace(Vector2d scale)
        {
            x *= scale.x;
            y *= scale.y;
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
        public Vector2d NormalizeInPlace()
        {
            Fixed64 mag = MyMagnitude;
            if (mag > FixedMath.Epsilon && mag != FixedMath.One)
            {
                x /= mag;
                y /= mag;
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
        public Vector2d NormalizeInPlace(out Fixed64 mag)
        {
            mag = MyMagnitude;
            if (mag > FixedMath.Epsilon && mag != FixedMath.One)
            {
                x /= mag;
                y /= mag;
            }
            return this;
        }

        /// <summary>
        /// Linearly interpolates this vector toward the target vector by the specified amount.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d LerpInPlace(Vector2d target, Fixed64 amount)
        {
            LerpInPlace(target.x, target.y, amount);
            return this;
        }

        /// <summary>
        /// Linearly interpolates this vector toward the target values by the specified amount.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2d LerpInPlace(Fixed64 targetx, Fixed64 targety, Fixed64 amount)
        {
            if (amount >= FixedMath.One)
            {
                x = targetx;
                y = targety;
            }
            else if (amount > FixedMath.Zero)
            {
                x = targetx * amount + x * (FixedMath.One - amount);
                y = targety * amount + y * (FixedMath.One - amount);
            }
            return this;
        }

        /// <summary>
        /// Linearly interpolates between two vectors.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d Lerp(Vector2d a, Vector2d b, Fixed64 amount)
        {
            amount = FixedMath.Clamp01(amount);
            return new Vector2d(a.x + (b.x - a.x) * amount, a.y + (b.y - a.y) * amount);
        }

        /// <summary>
        /// Returns a new vector that is the result of linear interpolation toward the target by the specified amount.
        /// </summary>
        public Vector2d Lerped(Vector2d target, Fixed64 amount)
        {
            Vector2d vec = this;
            vec.LerpInPlace(target.x, target.y, amount);
            return vec;
        }

        /// <summary>
        /// Rotates this vector by the specified cosine and sine values.
        /// </summary>
        public Vector2d RotateInPlace(Fixed64 cos, Fixed64 sin)
        {
            Fixed64 temp1 = x * cos + y * sin;
            y = x * -sin + y * cos;
            x = temp1;
            return this;
        }

        /// <summary>
        /// Returns a new vector that is the result of rotating this vector by the specified cosine and sine values.
        /// </summary>
        public Vector2d Rotated(Fixed64 cos, Fixed64 sin)
        {
            Vector2d vec = this;
            vec.RotateInPlace(cos, sin);
            return vec;
        }

        /// <summary>
        /// Rotates this vector using another vector representing the cosine and sine of the rotation angle.
        /// </summary>
        /// <param name="rotation">The vector containing the cosine and sine values for rotation.</param>
        /// <returns>A new vector representing the result of the rotation.</returns>

        public Vector2d Rotated(Vector2d rotation)
        {
            return Rotated(rotation.x, rotation.y);
        }

        /// <summary>
        /// Rotates this vector in the inverse direction using cosine and sine values.
        /// </summary>
        /// <param name="cos">The cosine of the rotation angle.</param>
        /// <param name="sin">The sine of the rotation angle.</param>
        public void RotateInverse(Fixed64 cos, Fixed64 sin)
        {
            RotateInPlace(cos, -sin);
        }


        /// <summary>
        /// Rotates this vector 90 degrees to the right (clockwise).
        /// </summary>
        public Vector2d RotateRightInPlace()
        {
            Fixed64 temp1 = x;
            x = y;
            y = -temp1;
            return this;
        }

        /// <summary>
        /// Rotates this vector 90 degrees to the left (counterclockwise).
        /// </summary>
        public Vector2d RotateLeftInPlace()
        {
            Fixed64 temp1 = x;
            x = -y;
            y = temp1;
            return this;
        }

        /// <summary>
        /// Reflects this vector across the specified axis.
        /// </summary>
        public Vector2d ReflectInPlace(Fixed64 axisX, Fixed64 axisY)
        {
            Fixed64 temp3 = Dot(axisX, axisY);
            Fixed64 temp1 = axisX * temp3;
            Fixed64 temp2 = axisY * temp3;
            x = temp1 + temp1 - x;
            y = temp2 + temp2 - y;
            return this;
        }

        /// <summary>
        /// Reflects this vector across the specified axis using the provided projection of this vector onto the axis.
        /// </summary>
        /// /// <param name="axisX">The x component of the axis to reflect across.</param>
        /// <param name="axisY">The y component of the axis to reflect across.</param>
        /// <param name="projection">The precomputed projection of this vector onto the reflection axis.</param>
        public Vector2d ReflectInPlace(Fixed64 axisX, Fixed64 axisY, Fixed64 projection)
        {
            Fixed64 temp1 = axisX * projection;
            Fixed64 temp2 = axisY * projection;
            x = temp1 + temp1 - x;
            y = temp2 + temp2 - y;
            return this;
        }

        /// <summary>
        /// Reflects this vector across the specified axis.
        /// </summary>
        /// <returns>A new vector representing the result of the reflection.</returns>
        public Vector2d Reflected(Fixed64 axisX, Fixed64 axisY)
        {
            Vector2d vec = this;
            vec.ReflectInPlace(axisX, axisY);
            return vec;
        }

        /// <summary>
        /// Returns the dot product of this vector with another vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Dot(Fixed64 otherX, Fixed64 otherY)
        {
            return x * otherX + y * otherY;
        }

        /// <summary>
        /// Returns the dot product of this vector with another vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Dot(Vector2d other)
        {
            return Dot(other.x, other.y);
        }

        /// <summary>
        /// Returns the cross product of this vector with another vector specified by its components.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Cross(Fixed64 otherX, Fixed64 otherY)
        {
            return x * otherY - y * otherX;
        }

        /// <summary>
        /// Returns the cross product of this vector with another 2D vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Cross(Vector2d vec)
        {
            return Cross(vec.x, vec.y);
        }

        /// <summary>
        /// Returns the distance between this vector and another vector specified by its components.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Distance(Fixed64 otherX, Fixed64 otherY)
        {
            Fixed64 temp1 = x - otherX;
            temp1 *= temp1;
            Fixed64 temp2 = y - otherY;
            temp2 *= temp2;
            return FixedMath.Sqrt(temp1 + temp2);
        }

        /// <summary>
        /// Returns the distance between this vector and another vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 Distance(Vector2d other)
        {
            return Distance(other.x, other.y);
        }

        /// <summary>
        /// Returns the squared distance between this vector and another vector (avoids calculating the square root).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 SqrDistance(Fixed64 otherX, Fixed64 otherY)
        {
            Fixed64 temp1 = x - otherX;
            temp1 *= temp1;
            Fixed64 temp2 = y - otherY;
            temp2 *= temp2;
            return temp1 + temp2;
        }

        /// <summary>
        /// Returns the squared distance between this vector and another vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 SqrDistance(Vector2d other)
        {
            return SqrDistance(other.x, other.y);
        }

        /// <summary>
        /// Checks whether the vector equals zero.
        /// </summary>
        public bool EqualsZero()
        {
            return x == FixedMath.Zero && y == FixedMath.Zero;
        }

        /// <summary>
        /// Checks if the vector is non-zero.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool NotZero()
        {
            return x.MoreThanEpsilon() || y.MoreThanEpsilon();
        }

        #endregion

        #region Static Math

        /// <summary>
        /// (1, 0)
        /// </summary>
        public static readonly Vector2d DefaultRotation = new Vector2d(1, 0);

        /// <summary>
        /// (0, 1)
        /// </summary>
        public static readonly Vector2d Forward = new Vector2d(0, 1);

        /// <summary>
        /// (1, 0)
        /// </summary>
        public static readonly Vector2d Right = new Vector2d(1, 0);

        /// <summary>
        /// (0, -1)
        /// </summary>
        public static readonly Vector2d Down = new Vector2d(0, -1);

        /// <summary>
        /// (-1, 0)
        /// </summary>
        public static readonly Vector2d Left = new Vector2d(-1, 0);

        /// <summary>
        /// (1, 1)
        /// </summary>
        public static readonly Vector2d One = new Vector2d(1, 1);

        /// <summary>
        /// (-1, -1)
        /// </summary>
        public static readonly Vector2d Negative = new Vector2d(-1, -1);

        /// <summary>
        /// (0, 0)
        /// </summary>
        public static readonly Vector2d Zero = new Vector2d(0, 0);

        /// <summary>
        /// Normalizes the given vector, returning a unit vector with the same direction.
        /// </summary>
        /// <param name="value">The vector to normalize.</param>
        /// <returns>A normalized (unit) vector with the same direction.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d Normalize(Vector2d value)
        {
            Fixed64 mag = Magnitude(value);
            if (mag > FixedMath.Epsilon && mag != FixedMath.One)
                return value / mag;
            return value;
        }

        /// <summary>
        /// Returns the magnitude (length) of the given vector.
        /// </summary>
        /// <param name="vector">The vector to compute the magnitude of.</param>
        /// <returns>The magnitude (length) of the vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Magnitude(Vector2d vector)
        {
            Fixed64 temp1 = vector.x * vector.x + vector.y * vector.y;
            if (temp1 == FixedMath.Zero)
                return FixedMath.Zero;

            return FixedMath.Sqrt(temp1);
        }

        /// <summary>
        /// Creates a vector from a given angle in radians.
        /// </summary>
        public static Vector2d CreateRotation(Fixed64 angle)
        {
            return new Vector2d(FixedMath.Cos(angle), FixedMath.Sin(angle));
        }

        /// <summary>
        /// Computes the distance between two vectors using the Euclidean distance formula.
        /// </summary>
        /// <param name="start">The starting vector.</param>
        /// <param name="end">The ending vector.</param>
        /// <returns>The Euclidean distance between the two vectors.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Distance(Vector2d start, Vector2d end)
        {
            Fixed64 tX = end.x - start.x;
            tX *= tX;
            Fixed64 tY = end.y - start.y;
            tY *= tY;
            return FixedMath.Sqrt(tX + tY);
        }

        #endregion

        #region Convert

        public override string ToString()
        {
            return $"({Math.Round((double)x, 2)}, {Math.Round((double)y, 2)})";
        }

        public Vector3d ToVector3d(Fixed64 z)
        {
            return new Vector3d(x, z, y);
        }

        #endregion

        #region Operators

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d operator +(Vector2d v1, Vector2d v2)
        {
            return new Vector2d(v1.x + v2.x, v1.y + v2.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d operator +(Vector2d v1, Fixed64 mag)
        {
            return new Vector2d(v1.x + mag, v1.y + mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d operator -(Vector2d v1, Vector2d v2)
        {
            return new Vector2d(v1.x - v2.x, v1.y - v2.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d operator -(Vector2d v1, Fixed64 mag)
        {
            return new Vector2d(v1.x - mag, v1.y - mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d operator -(Vector2d v1)
        {
            return new Vector2d(v1.x * -FixedMath.One, v1.y * -FixedMath.One);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d operator *(Vector2d v1, Fixed64 mag)
        {
            return new Vector2d(v1.x * mag, v1.y * mag);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d operator *(Vector2d v1, Vector2d v2)
        {
            return new Vector2d(v1.x * v2.x, v1.y * v2.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d operator /(Vector2d v1, Fixed64 div)
        {
            return new Vector2d(v1.x / div, v1.y / div);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Vector2d left, Vector2d right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Vector2d left, Vector2d right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is Vector2d other && Equals(other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Vector2d other)
        {
            return other.x == x && other.y == y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return StateHash;
        }

        #endregion
    }
}