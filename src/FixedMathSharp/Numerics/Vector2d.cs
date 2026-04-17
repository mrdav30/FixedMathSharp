using MemoryPack;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a 2D vector with fixed-point precision, offering a range of mathematical operations
/// and transformations such as rotation, scaling, reflection, and interpolation.
/// </summary>
/// <remarks>
/// The Vector2d struct is designed for applications that require precise numerical operations, 
/// such as games, simulations, or physics engines. It provides methods for common vector operations
/// like addition, subtraction, dot product, cross product, distance calculations, and rotation.
/// 
/// Use Cases:
/// - Modeling 2D positions, directions, and velocities in fixed-point math environments.
/// - Performing vector transformations, including rotations and reflections.
/// - Handling interpolation and distance calculations in physics or simulation systems.
/// - Useful for fixed-point math scenarios where floating-point precision is insufficient or not desired.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Vector2d : IEquatable<Vector2d>, IComparable<Vector2d>, IEqualityComparer<Vector2d>
{
    #region Static Readonly Fields

    /// <summary>
    /// (1, 0)
    /// </summary>
    public static readonly Vector2d DefaultRotation = new(1, 0);

    /// <summary>
    /// (0, 1)
    /// </summary>
    public static readonly Vector2d Forward = new(0, 1);

    /// <summary>
    /// (1, 0)
    /// </summary>
    public static readonly Vector2d Right = new(1, 0);

    /// <summary>
    /// (0, -1)
    /// </summary>
    public static readonly Vector2d Down = new(0, -1);

    /// <summary>
    /// (-1, 0)
    /// </summary>
    public static readonly Vector2d Left = new(-1, 0);

    /// <summary>
    /// (1, 1)
    /// </summary>
    public static readonly Vector2d One = new(1, 1);

    /// <summary>
    /// (-1, -1)
    /// </summary>
    public static readonly Vector2d Negative = new(-1, -1);

    /// <summary>
    /// (0, 0)
    /// </summary>
    public static readonly Vector2d Zero = new(0, 0);

    #endregion

    #region Fields

    /// <summary>
    /// The X component of the vector.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Fixed64 x;

    /// <summary>
    /// The Y component of the vector.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Fixed64 y;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance of the Vector2d structure using integer values for the X and Y components.
    /// </summary>
    /// <param name="xInt">The X component of the vector, specified as an integer.</param>
    /// <param name="yInt">The Y component of the vector, specified as an integer.</param>
    public Vector2d(int xInt, int yInt) : this((Fixed64)xInt, (Fixed64)yInt) { }

    /// <summary>
    /// Initializes a new instance of the Vector2d structure using the specified X and Y coordinates as double-precision
    /// floating-point values.
    /// </summary>
    /// <param name="xDoub">The X coordinate of the vector, specified as a double-precision floating-point value.</param>
    /// <param name="yDoub">The Y coordinate of the vector, specified as a double-precision floating-point value.</param>
    public Vector2d(double xDoub, double yDoub) : this((Fixed64)xDoub, (Fixed64)yDoub) { }

    /// <summary>
    /// Initializes a new instance of the Vector2d structure with the specified X and Y components.
    /// </summary>
    /// <param name="x">The value to assign to the X component of the vector.</param>
    /// <param name="y">The value to assign to the Y component of the vector.</param>
    [JsonConstructor]
    public Vector2d(Fixed64 x, Fixed64 y)
    {
        this.x = x;
        this.y = y;
    }

    #endregion

    #region Properties

    /// <summary>
    /// Rotates the vector to the right (90 degrees clockwise).
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d RotatedRight
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(y, -x);
    }

    /// <summary>
    /// Rotates the vector to the left (90 degrees counterclockwise).
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d RotatedLeft
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(-y, x);
    }

    /// <summary>
    /// Gets the right-hand (counter-clockwise) normal vector.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d RightHandNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(-y, x);
    }

    /// <summary>
    /// Gets the left-hand (clockwise) normal vector.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d LeftHandNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(y, -x);
    }

    /// <inheritdoc cref="GetNormalized(Vector2d)"/>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Normal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetNormalized(this);
    }

    /// <summary>
    /// Returns the actual length of this vector (RO).
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Magnitude
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetMagnitude(this);
    }

    /// <summary>
    /// Returns the square magnitude of the vector (avoids calculating the square root).
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 SqrMagnitude
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => x * x + y * y;
    }

    /// <summary>
    /// Returns a long hash of the vector based on its x and y values.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public long LongStateHash
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => x.m_rawValue * 31 + y.m_rawValue * 7;
    }

    /// <summary>
    /// Returns a hash of the vector based on its state.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public int StateHash
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (int)(LongStateHash % int.MaxValue);
    }

    /// <summary>
    /// Gets or sets the vector component at the specified index.
    /// </summary>
    /// <remarks>
    /// This indexer provides array-like access to the vector's components. 
    /// Index 0 corresponds to the X component, and index 1 corresponds to the Y component.
    /// </remarks>
    /// <param name="index">The zero-based index of the component to access. Use 0 for the X component and 1 for the Y component.</param>
    /// <returns>The vector component at the specified index.</returns>
    /// <exception cref="IndexOutOfRangeException">Thrown when the index is less than 0 or greater than 1.</exception>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            return index switch
            {
                0 => x,
                1 => y,
                _ => throw new IndexOutOfRangeException("Invalid Vector2d index!"),
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
                default:
                    throw new IndexOutOfRangeException("Invalid Vector2d index!");
            }
        }
    }

    #endregion

    #region Methods (Instance)

    /// <summary>
    /// Set x, y and z components of an existing Vector3.
    /// </summary>
    /// <param name="newX"></param>
    /// <param name="newY"></param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Set(Fixed64 newX, Fixed64 newY)
    {
        x = newX;
        y = newY;
    }

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

    /// <inheritdoc cref="AddInPlace(Fixed64, Fixed64)"/>
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
    public Vector2d SubtractInPlace(Vector2d other)
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
    public Vector2d Normalize()
    {
        return this = GetNormalized(this);
    }

    /// <summary>
    /// Normalizes this vector in place and outputs its original magnitude.
    /// </summary>
    /// <param name="mag">The original magnitude of the vector before normalization.</param>
    /// <remarks>
    /// If the vector is zero-length or already normalized, no operation is performed, but the original magnitude will still be output.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d Normalize(out Fixed64 mag)
    {
        mag = GetMagnitude(this);

        // If magnitude is zero, return a zero vector to avoid divide-by-zero errors
        if (mag == Fixed64.Zero)
        {
            x = Fixed64.Zero;
            y = Fixed64.Zero;
            return this;
        }

        // If already normalized, return as-is
        if (mag == Fixed64.One)
            return this;

        x /= mag;
        y /= mag;

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
        if (amount >= Fixed64.One)
        {
            x = targetx;
            y = targety;
        }
        else if (amount > Fixed64.Zero)
        {
            x = targetx * amount + x * (Fixed64.One - amount);
            y = targety * amount + y * (Fixed64.One - amount);
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
    /// Rotates this vector by the specified cosine and sine values (counter-clockwise).
    /// </summary>
    public Vector2d RotateInPlace(Fixed64 cos, Fixed64 sin)
    {
        Fixed64 temp1 = x * cos - y * sin;
        y = x * sin + y * cos;
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
    /// Reflects this vector across the specified axis vector.
    /// </summary>
    public Vector2d ReflectInPlace(Vector2d axis)
    {
        return ReflectInPlace(axis.x, axis.y);
    }

    /// <summary>
    /// Reflects this vector across the specified x and y axis.
    /// </summary>
    public Vector2d ReflectInPlace(Fixed64 axisX, Fixed64 axisY)
    {
        Fixed64 projection = Dot(axisX, axisY);
        return ReflectInPlace(axisX, axisY, projection);
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
    /// Reflects this vector across the specified x and y axis.
    /// </summary>
    /// <returns>A new vector representing the result of the reflection.</returns>
    public Vector2d Reflected(Fixed64 axisX, Fixed64 axisY)
    {
        Vector2d vec = this;
        vec.ReflectInPlace(axisX, axisY);
        return vec;
    }

    /// <summary>
    /// Reflects this vector across the specified axis vector.
    /// </summary>
    /// <returns>A new vector representing the result of the reflection.</returns>
    public Vector2d Reflected(Vector2d axis)
    {
        return Reflected(axis.x, axis.y);
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
    /// Computes the cross product magnitude of this vector with another vector.
    /// </summary>
    /// <param name="otherX">The X component of the other vector.</param>
    /// <param name="otherY">The Y component of the other vector.</param>
    /// <returns>The cross product magnitude.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 CrossProduct(Fixed64 otherX, Fixed64 otherY)
    {
        return x * otherY - y * otherX;
    }

    /// <inheritdoc cref="CrossProduct(Fixed64, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 CrossProduct(Vector2d other)
    {
        return CrossProduct(other.x, other.y);
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
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <returns>The squared distance between the two vectors.</returns>
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
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <returns>The squared distance between the two vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 SqrDistance(Vector2d other)
    {
        return SqrDistance(other.x, other.y);
    }

    #endregion

    #region Vector2d Operations

    /// <summary>
    /// Normalizes the given vector, returning a unit vector with the same direction.
    /// </summary>
    /// <param name="value">The vector to normalize.</param>
    /// <returns>A normalized (unit) vector with the same direction.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d GetNormalized(Vector2d value)
    {
        Fixed64 mag = GetMagnitude(value);

        if (mag == Fixed64.Zero)
            return new Vector2d(Fixed64.Zero, Fixed64.Zero);

        // If already normalized, return as-is
        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return value;

        // Normalize it exactly
        return new Vector2d(
            value.x / mag,
            value.y / mag
        );
    }

    /// <summary>
    /// Returns the magnitude (length) of the given vector.
    /// </summary>
    /// <param name="vector">The vector to compute the magnitude of.</param>
    /// <returns>The magnitude (length) of the vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetMagnitude(Vector2d vector)
    {
        Fixed64 mag = (vector.x * vector.x) + (vector.y * vector.y);

        // If rounding error pushed magnitude slightly above 1, clamp it
        if (mag > Fixed64.One && mag <= Fixed64.One + Fixed64.Epsilon)
            return Fixed64.One;

        return mag.Abs() > Fixed64.Zero ? FixedMath.Sqrt(mag) : Fixed64.Zero;
    }

    /// <summary>
    /// Returns a new <see cref="Vector2d"/> where each component is the absolute value of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector with absolute values for each component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Abs(Vector2d value)
    {
        return new Vector2d(value.x.Abs(), value.y.Abs());
    }

    /// <summary>
    /// Returns a new <see cref="Vector2d"/> where each component is the sign of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector where each component is -1, 0, or 1 based on the sign of the input.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Sign(Vector2d value)
    {
        return new Vector2d(value.x.Sign(), value.y.Sign());
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
        return start.Distance(end);
    }

    /// <summary>
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <returns>The squared distance between the two vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 SqrDistance(Vector2d start, Vector2d end)
    {
        return start.SqrDistance(end);
    }

    /// <summary>
    /// Calculates the forward direction vector in 2D based on a yaw (angle).
    /// </summary>
    /// <param name="angle">The angle in radians representing the rotation in 2D space.</param>
    /// <returns>A unit vector representing the forward direction.</returns>
    public static Vector2d ForwardDirection(Fixed64 angle)
    {
        Fixed64 x = FixedMath.Cos(angle); // Forward in the x-direction
        Fixed64 y = FixedMath.Sin(angle); // Forward in the y-direction
        return new Vector2d(x, y);
    }

    /// <summary>
    /// Dot Product of two vectors.
    /// </summary>
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(Vector2d lhs, Vector2d rhs)
    {
        return lhs.Dot(rhs.x, rhs.y);
    }

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <returns></returns>
    public static Vector2d Scale(Vector2d a, Vector2d b)
    {
        return new Vector2d(a.x * b.x, a.y * b.y);
    }

    /// <summary>
    /// Cross Product of two vectors.
    /// </summary>
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CrossProduct(Vector2d lhs, Vector2d rhs)
    {
        return lhs.CrossProduct(rhs);
    }

    /// <summary>
    /// Rotates this vector by the specified angle (in radians).
    /// </summary>
    /// <param name="vec">The vector to rotate.</param>
    /// <param name="angleInRadians">The angle in radians.</param>
    /// <returns>The rotated vector.</returns>
    public static Vector2d Rotate(Vector2d vec, Fixed64 angleInRadians)
    {
        Fixed64 cos = FixedMath.Cos(angleInRadians);
        Fixed64 sin = FixedMath.Sin(angleInRadians);
        return new Vector2d(
            vec.x * cos - vec.y * sin,
            vec.x * sin + vec.y * cos
        );
    }

    #endregion

    #region Conversion

    /// <summary>
    /// Returns a string that represents the current object, displaying the x and y values rounded to two decimal places.
    /// </summary>
    /// <returns>A string in the format "(x, y)", where x and y are the values of the object rounded to two decimal places.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString()
    {
        return $"({Math.Round((double)x, 2)}, {Math.Round((double)y, 2)})";
    }

    /// <summary>
    /// Converts this <see cref="Vector2d"/> to a <see cref="Vector3d"/>, 
    /// mapping the Y component of this vector to the Z axis in the resulting vector.
    /// </summary>
    /// <param name="z">The value to assign to the Y axis of the resulting <see cref="Vector3d"/>.</param>
    /// <returns>
    /// A new <see cref="Vector3d"/> where (X, Y) from this <see cref="Vector2d"/> 
    /// become (X, Z) in the resulting vector, with the provided Z parameter assigned to Y.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3d ToVector3d(Fixed64 z)
    {
        return new Vector3d(x, z, y);
    }

    /// <summary>
    /// Deconstructs the current instance into its X and Y coordinate values as single-precision floating-point numbers.
    /// </summary>
    /// <remarks>
    /// This method enables deconstruction syntax, allowing the instance to be unpacked into separate
    /// X and Y values using tuple deconstruction in client code.
    /// </remarks>
    /// <param name="x">When this method returns, contains the X coordinate value as a single-precision floating-point number.</param>
    /// <param name="y">When this method returns, contains the Y coordinate value as a single-precision floating-point number.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void Deconstruct(out float x, out float y)
    {
        x = this.x.ToPreciseFloat();
        y = this.y.ToPreciseFloat();
    }

    /// <summary>
    /// Deconstructs the current instance into its X and Y components, rounding each value to the nearest integer.
    /// </summary>
    /// <remarks>
    /// This method enables deconstruction syntax, allowing the instance to be unpacked into two
    /// integer variables representing the rounded X and Y values.
    /// </remarks>
    /// <param name="x">When this method returns, contains the X component of the instance, rounded to the nearest integer.</param>
    /// <param name="y">When this method returns, contains the Y component of the instance, rounded to the nearest integer.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void Deconstruct(out int x, out int y)
    {
        x = this.x.RoundToInt();
        y = this.y.RoundToInt();
    }

    /// <summary>
    /// Converts each component of the vector from radians to degrees.
    /// </summary>
    /// <param name="radians">The vector with components in radians.</param>
    /// <returns>A new vector with components converted to degrees.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ToDegrees(Vector2d radians)
    {
        return new Vector2d(
            FixedMath.RadToDeg(radians.x),
            FixedMath.RadToDeg(radians.y)
        );
    }

    /// <summary>
    /// Converts each component of the vector from degrees to radians.
    /// </summary>
    /// <param name="degrees">The vector with components in degrees.</param>
    /// <returns>A new vector with components converted to radians.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ToRadians(Vector2d degrees)
    {
        return new Vector2d(
            FixedMath.DegToRad(degrees.x),
            FixedMath.DegToRad(degrees.y)
        );
    }

    #endregion

    #region Operators

    /// <summary>
    /// Adds two Vector2d instances component-wise.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">The second vector to add.</param>
    /// <returns>A new Vector2d whose components are the sums of the corresponding components of v1 and v2.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, Vector2d v2)
    {
        return new Vector2d(v1.x + v2.x, v1.y + v2.y);
    }

    /// <summary>
    /// Adds a scalar value to each component of the specified vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector to which the scalar value will be added.</param>
    /// <param name="mag">The scalar value to add to each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the sum of the corresponding components of the input vector and the scalar value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, Fixed64 mag)
    {
        return new Vector2d(v1.x + mag, v1.y + mag);
    }

    /// <inheritdoc cref="operator +(Vector2d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Fixed64 mag, Vector2d v1)
    {
        return v1 + mag;
    }

    /// <summary>
    /// Adds a Vector2d instance and a tuple representing X and Y components, returning a new Vector2d with the summed
    /// values.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">A tuple containing the X and Y values to add to the vector.</param>
    /// <returns>A new Vector2d whose X and Y components are the sums of the corresponding components of the input vector and tuple.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, (int x, int y) v2)
    {
        return new Vector2d(v1.x + v2.x, v1.y + v2.y);
    }

    /// <summary>
    /// Adds a tuple representing X and Y components and a Vector2d instance, returning a new Vector2d with the summed
    /// values.
    /// </summary>
    /// <param name="v2">A tuple containing the X and Y values to add to the vector.</param>
    /// <param name="v1">The vector to add.</param>
    /// <returns>A new Vector2d whose X and Y components are the sums of the corresponding components of the tuple and input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +((int x, int y) v2, Vector2d v1)
    {
        return v1 + v2;
    }

    /// <summary>
    /// Subtracts the components of one Vector2d from another and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector to subtract from.</param>
    /// <param name="v2">The vector to subtract.</param>
    /// <returns>A Vector2d whose components are the result of subtracting the corresponding components of v2 from v1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, Vector2d v2)
    {
        return new Vector2d(v1.x - v2.x, v1.y - v2.y);
    }

    /// <summary>
    /// Subtracts the specified scalar value from both components of the given vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector from which to subtract the scalar value.</param>
    /// <param name="mag">The scalar value to subtract from each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the result of subtracting the scalar value from the corresponding components
    /// of the input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, Fixed64 mag)
    {
        return new Vector2d(v1.x - mag, v1.y - mag);
    }

    /// <inheritdoc cref="operator -(Vector2d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Fixed64 mag, Vector2d v1)
    {
        return new Vector2d(mag - v1.x, mag - v1.y);
    }

    /// <summary>
    /// Subtracts the specified tuple from the given vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector from which to subtract the tuple values.</param>
    /// <param name="v2">A tuple containing the x and y values to subtract from the vector.</param>
    /// <returns>A new Vector2d representing the result of subtracting the tuple values from the original vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, (int x, int y) v2)
    {
        return new Vector2d(v1.x - v2.x, v1.y - v2.y);
    }

    /// <summary>
    /// Subtracts the specified Vector2d from the given integer tuple and returns the resulting vector.
    /// </summary>
    /// <param name="v1">A tuple containing the x and y components to subtract from.</param>
    /// <param name="v2">The vector whose components are subtracted from the tuple.</param>
    /// <returns>A Vector2d representing the result of subtracting the components of v2 from v1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -((int x, int y) v1, Vector2d v2)
    {
        return new Vector2d(v1.x - v2.x, v1.y - v2.y);
    }

    /// <summary>
    /// Negates the specified vector by reversing the sign of each of its components.
    /// </summary>
    /// <param name="v1">The vector to negate.</param>
    /// <returns>A new Vector2d whose components are the negated values of the input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1)
    {
        return new Vector2d(v1.x * -Fixed64.One, v1.y * -Fixed64.One);
    }

    /// <summary>
    /// Scales the specified vector by the given scalar value.
    /// </summary>
    /// <param name="v1">The vector to be scaled.</param>
    /// <param name="mag">The scalar value by which to multiply each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the components of v1 multiplied by mag.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator *(Vector2d v1, Fixed64 mag)
    {
        return new Vector2d(v1.x * mag, v1.y * mag);
    }

    /// <summary>
    /// Multiplies two vectors element-wise and returns the resulting vector.
    /// </summary>
    /// <remarks>Element-wise multiplication multiplies each component of the first vector by the
    /// corresponding component of the second vector. This operation is not a dot product or cross product.</remarks>
    /// <param name="v1">The first vector to multiply.</param>
    /// <param name="v2">The second vector to multiply.</param>
    /// <returns>A new Vector2d whose components are the products of the corresponding components of the input vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator *(Vector2d v1, Vector2d v2)
    {
        return new Vector2d(v1.x * v2.x, v1.y * v2.y);
    }

    /// <summary>
    /// Divides each component of a specified vector by a scalar value.
    /// </summary>
    /// <param name="v1">The vector whose components are to be divided.</param>
    /// <param name="div">The scalar value by which to divide each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the result of dividing the corresponding components of v1 by div.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator /(Vector2d v1, Fixed64 div)
    {
        return new Vector2d(v1.x / div, v1.y / div);
    }

    /// <summary>
    /// Determines whether two Vector2d instances are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Vector2d left, Vector2d right) => left.Equals(right);

    /// <summary>
    /// Determines whether two Vector2d instances are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Vector2d left, Vector2d right) => !left.Equals(right);

    #endregion

    #region Equality, HashCode, and Comparable Overrides

    /// <summary>
    /// Are all components of this vector equal to zero?
    /// </summary>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool EqualsZero() => this.Equals(Zero);

    /// <summary>
    /// Determines whether the current value is not equal to zero.
    /// </summary>
    /// <returns>true if the value is not zero; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool NotZero() => !EqualsZero();

    /// <summary>
    /// Checks whether all components are strictly greater than <see cref="Fixed64.Epsilon"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllComponentsGreaterThanEpsilon()
    {
        return x.Abs() > Fixed64.Epsilon && y.Abs() > Fixed64.Epsilon;
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Vector2d other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Vector2d other) => other.x == x && other.y == y;

    /// <inheritdoc/>
    public bool Equals(Vector2d x, Vector2d y) => x.Equals(y);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => StateHash;

    /// <inheritdoc/>
    public int GetHashCode(Vector2d obj)
    {
        return obj.GetHashCode();
    }

    /// <summary>
    /// Compares the current Vector2d instance with another Vector2d based on their squared magnitudes.
    /// </summary>
    /// <remarks>
    /// This comparison uses the squared magnitude of each vector, which avoids the computational
    /// cost of calculating the actual magnitude. 
    /// Use this method when only relative vector lengths are
    /// important.
    /// </remarks>
    /// <param name="other">The Vector2d instance to compare with the current instance.</param>
    /// <returns>A value less than zero if this instance is less than <paramref name="other"/>; zero if this instance is equal to
    /// <paramref name="other"/>; or a value greater than zero if this instance is greater than <paramref
    /// name="other"/>, as determined by their squared magnitudes.</returns>
    public int CompareTo(Vector2d other) => SqrMagnitude.CompareTo(other.SqrMagnitude);

    #endregion
}
