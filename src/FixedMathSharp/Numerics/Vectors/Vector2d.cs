//=======================================================================
// Vector2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

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
    public static Vector2d DefaultRotation => new(1, 0);

    /// <summary>
    /// (0, 1)
    /// </summary>
    public static Vector2d Forward => new(0, 1);

    /// <summary>
    /// (1, 0)
    /// </summary>
    public static Vector2d Right => new(1, 0);

    /// <summary>
    /// (0, -1)
    /// </summary>
    public static Vector2d Down => new(0, -1);

    /// <summary>
    /// (-1, 0)
    /// </summary>
    public static Vector2d Left => new(-1, 0);

    /// <summary>
    /// (1, 1)
    /// </summary>
    public static Vector2d One => new(1, 1);

    /// <summary>
    /// (-1, -1)
    /// </summary>
    public static Vector2d Negative => new(-1, -1);

    /// <summary>
    /// (0, 0)
    /// </summary>
    public static Vector2d Zero => new(0, 0);

    #endregion

    #region Fields

    /// <summary>
    /// The X component of the vector.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Fixed64 X;

    /// <summary>
    /// The Y component of the vector.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Fixed64 Y;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance of the Vector2d structure using integer values for the X and Y components.
    /// </summary>
    /// <param name="xInt">The X component of the vector, specified as an integer.</param>
    /// <param name="yInt">The Y component of the vector, specified as an integer.</param>
    public Vector2d(int xInt, int yInt) : this((Fixed64)xInt, (Fixed64)yInt) { }

    /// <summary>
    /// Initializes a new instance of the Vector2d structure with the specified X and Y components.
    /// </summary>
    /// <param name="x">The value to assign to the X component of the vector.</param>
    /// <param name="y">The value to assign to the Y component of the vector.</param>
    [JsonConstructor]
    public Vector2d(Fixed64 x, Fixed64 y)
    {
        this.X = x;
        this.Y = y;
    }

    /// <summary>
    /// Initializes a new instance of the Vector2d structure using double-precision floating-point values for the X and Y components.
    /// floating-point values.
    /// </summary>
    public static Vector2d FromFloatPoint(double xDoub, double yDoub) =>
        new(Fixed64.FromFloatPoint(xDoub), Fixed64.FromFloatPoint(yDoub));

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
        get => new(Y, -X);
    }

    /// <summary>
    /// Rotates the vector to the left (90 degrees counterclockwise).
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d RotatedLeft
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(-Y, X);
    }

    /// <summary>
    /// Gets the right-hand (counter-clockwise) normal vector.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d RightHandNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(-Y, X);
    }

    /// <summary>
    /// Gets the left-hand (clockwise) normal vector.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d LeftHandNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(Y, -X);
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
        get => X * X + Y * Y;
    }

    /// <summary>
    /// Returns a long hash of the vector based on its x and y values.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public long LongStateHash
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => X.m_rawValue * 31 + Y.m_rawValue * 7;
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
                0 => X,
                1 => Y,
                _ => throw new IndexOutOfRangeException("Invalid Vector2d index!"),
            };
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            switch (index)
            {
                case 0:
                    X = value;
                    break;
                case 1:
                    Y = value;
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
        X = newX;
        Y = newY;
    }

    /// <summary>
    /// Adds the specified values to the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="xAmount">The amount to add to the x component.</param>
    /// <param name="yAmount">The amount to add to the y component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d AddInPlace(Fixed64 xAmount, Fixed64 yAmount)
    {
        X += xAmount;
        Y += yAmount;
        return this;
    }

    /// <summary>
    /// Adds the specified values to the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="amount">The amount to add to the components.</param>
    /// <returns>The modified vector after addition.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d AddInPlace(Fixed64 amount) => AddInPlace(amount, amount);

    /// <inheritdoc cref="AddInPlace(Fixed64, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d AddInPlace(Vector2d other) => AddInPlace(other.X, other.Y);

    /// <summary>
    /// Subtracts the specified values from the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="xAmount">The amount to subtract from the x component.</param>
    /// <param name="yAmount">The amount to subtract from the y component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d SubtractInPlace(Fixed64 xAmount, Fixed64 yAmount)
    {
        X -= xAmount;
        Y -= yAmount;
        return this;
    }

    /// <summary>
    /// Subtracts the specified value from all components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="amount">The amount to subtract from each component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d SubtractInPlace(Fixed64 amount) => SubtractInPlace(amount, amount);

    /// <summary>
    /// Subtracts the specified vector from the components of the vector in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d SubtractInPlace(Vector2d other) => SubtractInPlace(other.X, other.Y);

    /// <summary>
    /// Multiplies the components of the vector by the specified x and y factors in place and returns the modified vector.
    /// </summary>
    /// <param name="factorX">The factor to multiply the x component by.</param>
    /// <param name="factorY">The factor to multiply the y component by.</param>
    /// <returns>The modified vector after multiplication.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d MultiplyInPlace(Fixed64 factorX, Fixed64 factorY)
    {
        X *= factorX;
        Y *= factorY;
        return this;
    }

    /// <summary>
    /// Multiplies the components of the vector by the specified scalar factor in place and returns the modified vector.
    /// </summary>
    /// <param name="factor">The scalar factor to multiply each component by.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d MultiplyInPlace(Fixed64 factor) => MultiplyInPlace(factor, factor);

    /// <summary>
    /// Multiplies each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
    /// </summary>
    /// <param name="factor">The vector containing the multiplication factors for each component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d MultiplyInPlace(Vector2d factor) => MultiplyInPlace(factor.X, factor.Y);

    /// <summary>
    /// Divides the components of the vector by the specified x and y divisors in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d DivideInPlace(Fixed64 divisorX, Fixed64 divisorY)
    {
        X /= divisorX;
        Y /= divisorY;
        return this;
    }

    /// <summary>
    /// Divides each component of the vector by the specified scalar divisor in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d DivideInPlace(Fixed64 divisor) => DivideInPlace(divisor, divisor);

    /// <summary>
    /// Divides each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d DivideInPlace(Vector2d divisor) => DivideInPlace(divisor.X, divisor.Y);

    /// <summary>
    /// Normalizes this vector in place, making its magnitude (length) equal to 1, and returns the modified vector.
    /// </summary>
    /// <remarks>
    /// If the vector is zero-length or already normalized, no operation is performed. 
    /// This method modifies the current vector in place and supports method chaining.
    /// </remarks>
    /// <returns>The normalized vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d Normalize() => this = GetNormalized(this);

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
            X = Fixed64.Zero;
            Y = Fixed64.Zero;
            return this;
        }

        // If already normalized, return as-is
        if (mag == Fixed64.One)
            return this;

        X = Fixed64.DivideByPositive(X, mag);
        Y = Fixed64.DivideByPositive(Y, mag);

        return this;
    }

    /// <summary>
    /// Linearly interpolates this vector toward the target vector by the specified amount.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d LerpInPlace(Vector2d target, Fixed64 amount)
    {
        LerpInPlace(target.X, target.Y, amount);
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
            X = targetx;
            Y = targety;
        }
        else if (amount > Fixed64.Zero)
        {
            X = targetx * amount + X * (Fixed64.One - amount);
            Y = targety * amount + Y * (Fixed64.One - amount);
        }
        return this;
    }

    /// <summary>
    /// Returns a new vector that is the result of linear interpolation toward the target by the specified amount.
    /// </summary>
    public Vector2d Lerp(Vector2d target, Fixed64 amount)
    {
        Vector2d vec = this;
        vec.LerpInPlace(target.X, target.Y, amount);
        return vec;
    }

    /// <summary>
    /// Rotates this vector by the specified cosine and sine values (counter-clockwise).
    /// </summary>
    public Vector2d RotateInPlace(Fixed64 cos, Fixed64 sin)
    {
        Fixed64 temp1 = X * cos - Y * sin;
        Y = X * sin + Y * cos;
        X = temp1;
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
    public Vector2d Rotated(Vector2d rotation) => Rotated(rotation.X, rotation.Y);

    /// <summary>
    /// Rotates this vector in the inverse direction using cosine and sine values.
    /// </summary>
    /// <param name="cos">The cosine of the rotation angle.</param>
    /// <param name="sin">The sine of the rotation angle.</param>
    public void RotateInverse(Fixed64 cos, Fixed64 sin) => RotateInPlace(cos, -sin);

    /// <summary>
    /// Rotates this vector 90 degrees to the right (clockwise).
    /// </summary>
    public Vector2d RotateRightInPlace()
    {
        Fixed64 temp1 = X;
        X = Y;
        Y = -temp1;
        return this;
    }

    /// <summary>
    /// Rotates this vector 90 degrees to the left (counterclockwise).
    /// </summary>
    public Vector2d RotateLeftInPlace()
    {
        Fixed64 temp1 = X;
        X = -Y;
        Y = temp1;
        return this;
    }

    /// <summary>
    /// Reflects this vector across the specified axis vector.
    /// </summary>
    public Vector2d ReflectInPlace(Vector2d axis) => ReflectInPlace(axis.X, axis.Y);

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
        X = temp1 + temp1 - X;
        Y = temp2 + temp2 - Y;
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
    public Vector2d Reflected(Vector2d axis) => Reflected(axis.X, axis.Y);

    /// <summary>
    /// Returns the dot product of this vector with another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Dot(Fixed64 otherX, Fixed64 otherY) => X * otherX + Y * otherY;

    /// <summary>
    /// Returns the dot product of this vector with another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Dot(Vector2d other) => Dot(other.X, other.Y);

    /// <summary>
    /// Computes the cross product magnitude of this vector with another vector.
    /// </summary>
    /// <param name="otherX">The X component of the other vector.</param>
    /// <param name="otherY">The Y component of the other vector.</param>
    /// <returns>The cross product magnitude.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 CrossProduct(Fixed64 otherX, Fixed64 otherY) => X * otherY - Y * otherX;

    /// <inheritdoc cref="CrossProduct(Fixed64, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 CrossProduct(Vector2d other) => CrossProduct(other.X, other.Y);

    /// <summary>
    /// Returns the distance between this vector and another vector specified by its components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Distance(Fixed64 otherX, Fixed64 otherY)
    {
        Fixed64 temp1 = X - otherX;
        temp1 *= temp1;
        Fixed64 temp2 = Y - otherY;
        temp2 *= temp2;
        return FixedMath.Sqrt(temp1 + temp2);
    }

    /// <summary>
    /// Returns the distance between this vector and another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Distance(Vector2d other) => Distance(other.X, other.Y);

    /// <summary>
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <returns>The squared distance between the two vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 SqrDistance(Fixed64 otherX, Fixed64 otherY)
    {
        Fixed64 temp1 = X - otherX;
        temp1 *= temp1;
        Fixed64 temp2 = Y - otherY;
        temp2 *= temp2;
        return temp1 + temp2;
    }

    /// <summary>
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <returns>The squared distance between the two vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 SqrDistance(Vector2d other) => SqrDistance(other.X, other.Y);

    #endregion

    #region Static Operations


    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Add(Vector2d v1, Vector2d v2) => v1 + v2;

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Subtract(Vector2d v1, Vector2d v2) => v1 - v2;

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Multiply(Vector2d v1, Vector2d v2) => v1 * v2;

    /// <summary>
    /// Multiplies each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Multiply(Vector2d value, Fixed64 factor) => value * factor;

    /// <summary>
    /// Divides each component of the first vector by the corresponding component of the second vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Divide(Vector2d v1, Vector2d v2) => new(v1.X / v2.X, v1.Y / v2.Y);

    /// <summary>
    /// Divides each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Divide(Vector2d value, Fixed64 divisor) => value / divisor;


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
            Fixed64.DivideByPositive(value.X, mag),
            Fixed64.DivideByPositive(value.Y, mag)
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
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y);

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
    public static Vector2d Abs(Vector2d value) => new(value.X.Abs(), value.Y.Abs());

    /// <summary>
    /// Returns a new <see cref="Vector2d"/> where each component is the sign of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector where each component is -1, 0, or 1 based on the sign of the input.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Sign(Vector2d value) => new(value.X.Sign(), value.Y.Sign());

    /// <summary>
    /// Creates a vector from a given angle in radians.
    /// </summary>
    public static Vector2d CreateRotation(Fixed64 angle) => new(FixedMath.Cos(angle), FixedMath.Sin(angle));

    /// <summary>
    /// Linearly interpolates between two vectors.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Lerp(Vector2d a, Vector2d b, Fixed64 amount)
    {
        amount = FixedMath.Clamp01(amount);
        return new Vector2d(a.X + (b.X - a.X) * amount, a.Y + (b.Y - a.Y) * amount);
    }

    /// <summary>
    /// Computes the distance between two vectors using the Euclidean distance formula.
    /// </summary>
    /// <param name="start">The starting vector.</param>
    /// <param name="end">The ending vector.</param>
    /// <returns>The Euclidean distance between the two vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector2d start, Vector2d end) => start.Distance(end);

    /// <summary>
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <returns>The squared distance between the two vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 SqrDistance(Vector2d start, Vector2d end) => start.SqrDistance(end);

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
    public static Fixed64 Dot(Vector2d lhs, Vector2d rhs) => lhs.Dot(rhs.X, rhs.Y);

    /// <summary>
    /// Cross Product of two vectors.
    /// </summary>
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CrossProduct(Vector2d lhs, Vector2d rhs) => lhs.CrossProduct(rhs);

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
            vec.X * cos - vec.Y * sin,
            vec.X * sin + vec.Y * cos
        );
    }

    #endregion

    #region Conversion

    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly string ToString() =>
        string.Format("({0}, {1})",
            (double)X,
            (double)Y);

    /// <summary>
    /// Returns a string representation of this vector with components formatted to a fixed number of decimal places.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly string ToFormattedString() =>
        string.Format("({0}, {1})",
            X.ToFormattedDouble(),
            Y.ToFormattedDouble());

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
    public Vector3d ToVector3d(Fixed64 z) => new(X, z, Y);

    /// <summary>
    /// Converts this <see cref="Vector2d"/> to a <see cref="Vector4d"/> with explicit Z and W components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d ToVector4d(Fixed64 z, Fixed64 w) => new(X, Y, z, w);

    /// <summary>
    /// Deconstructs the Vector2d into its two Fixed64 components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Fixed64 x, out Fixed64 y)
    {
        x = X;
        y = Y;
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two int components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out int x, out int y)
    {
        x = X.RoundToInt();
        y = Y.RoundToInt();
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two long components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out long x, out long y)
    {
        x = X.m_rawValue;
        y = Y.m_rawValue;
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two double components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out double x, out double y)
    {
        x = (double)X;
        y = (double)Y;
    }

    /// <summary>
    /// Converts each component of the vector from radians to degrees.
    /// </summary>
    /// <param name="radians">The vector with components in radians.</param>
    /// <returns>A new vector with components converted to degrees.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ToDegrees(Vector2d radians) =>
        new(FixedMath.RadToDeg(radians.X),
            FixedMath.RadToDeg(radians.Y));

    /// <summary>
    /// Converts each component of the vector from degrees to radians.
    /// </summary>
    /// <param name="degrees">The vector with components in degrees.</param>
    /// <returns>A new vector with components converted to radians.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ToRadians(Vector2d degrees) =>
        new(FixedMath.DegToRad(degrees.X),
            FixedMath.DegToRad(degrees.Y));

    #endregion

    #region Operators

    /// <summary>
    /// Adds two Vector2d instances component-wise.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">The second vector to add.</param>
    /// <returns>A new Vector2d whose components are the sums of the corresponding components of v1 and v2.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, Vector2d v2) => new(v1.X + v2.X, v1.Y + v2.Y);

    /// <summary>
    /// Adds a scalar value to each component of the specified vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector to which the scalar value will be added.</param>
    /// <param name="mag">The scalar value to add to each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the sum of the corresponding components of the input vector and the scalar value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, Fixed64 mag) => new(v1.X + mag, v1.Y + mag);

    /// <inheritdoc cref="operator +(Vector2d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Fixed64 mag, Vector2d v1) => v1 + mag;

    /// <summary>
    /// Adds a Vector2d instance and a tuple representing X and Y components, returning a new Vector2d with the summed
    /// values.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">A tuple containing the X and Y values to add to the vector.</param>
    /// <returns>A new Vector2d whose X and Y components are the sums of the corresponding components of the input vector and tuple.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, (int x, int y) v2) => new(v1.X + v2.x, v1.Y + v2.y);

    /// <summary>
    /// Adds a tuple representing X and Y components and a Vector2d instance, returning a new Vector2d with the summed
    /// values.
    /// </summary>
    /// <param name="v2">A tuple containing the X and Y values to add to the vector.</param>
    /// <param name="v1">The vector to add.</param>
    /// <returns>A new Vector2d whose X and Y components are the sums of the corresponding components of the tuple and input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +((int x, int y) v2, Vector2d v1) => v1 + v2;

    /// <summary>
    /// Subtracts the components of one Vector2d from another and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector to subtract from.</param>
    /// <param name="v2">The vector to subtract.</param>
    /// <returns>A Vector2d whose components are the result of subtracting the corresponding components of v2 from v1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, Vector2d v2) => new(v1.X - v2.X, v1.Y - v2.Y);

    /// <summary>
    /// Subtracts the specified scalar value from both components of the given vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector from which to subtract the scalar value.</param>
    /// <param name="mag">The scalar value to subtract from each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the result of subtracting the scalar value from the corresponding components
    /// of the input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, Fixed64 mag) => new(v1.X - mag, v1.Y - mag);

    /// <inheritdoc cref="operator -(Vector2d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Fixed64 mag, Vector2d v1) => new(mag - v1.X, mag - v1.Y);

    /// <summary>
    /// Subtracts the specified tuple from the given vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector from which to subtract the tuple values.</param>
    /// <param name="v2">A tuple containing the x and y values to subtract from the vector.</param>
    /// <returns>A new Vector2d representing the result of subtracting the tuple values from the original vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, (int x, int y) v2) => new(v1.X - v2.x, v1.Y - v2.y);

    /// <summary>
    /// Subtracts the specified Vector2d from the given integer tuple and returns the resulting vector.
    /// </summary>
    /// <param name="v1">A tuple containing the x and y components to subtract from.</param>
    /// <param name="v2">The vector whose components are subtracted from the tuple.</param>
    /// <returns>A Vector2d representing the result of subtracting the components of v2 from v1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -((int x, int y) v1, Vector2d v2) => new(v1.x - v2.X, v1.y - v2.Y);

    /// <summary>
    /// Negates the specified vector by reversing the sign of each of its components.
    /// </summary>
    /// <param name="v1">The vector to negate.</param>
    /// <returns>A new Vector2d whose components are the negated values of the input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1) => new(v1.X * -Fixed64.One, v1.Y * -Fixed64.One);

    /// <summary>
    /// Scales the specified vector by the given scalar value.
    /// </summary>
    /// <param name="v1">The vector to be scaled.</param>
    /// <param name="mag">The scalar value by which to multiply each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the components of v1 multiplied by mag.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator *(Vector2d v1, Fixed64 mag) => new(v1.X * mag, v1.Y * mag);

    /// <inheritdoc cref="operator *(Vector2d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator *(Fixed64 mag, Vector2d v1) => v1 * mag;

    /// <summary>
    /// Multiplies two vectors element-wise and returns the resulting vector.
    /// </summary>
    /// <remarks>Element-wise multiplication multiplies each component of the first vector by the
    /// corresponding component of the second vector. This operation is not a dot product or cross product.</remarks>
    /// <param name="v1">The first vector to multiply.</param>
    /// <param name="v2">The second vector to multiply.</param>
    /// <returns>A new Vector2d whose components are the products of the corresponding components of the input vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator *(Vector2d v1, Vector2d v2) => new(v1.X * v2.X, v1.Y * v2.Y);

    /// <summary>
    /// Divides each component of a specified vector by a scalar value.
    /// </summary>
    /// <param name="v1">The vector whose components are to be divided.</param>
    /// <param name="div">The scalar value by which to divide each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the result of dividing the corresponding components of v1 by div.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator /(Vector2d v1, Fixed64 div) => new(v1.X / div, v1.Y / div);

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
        return X.Abs() > Fixed64.Epsilon && Y.Abs() > Fixed64.Epsilon;
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Vector2d other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Vector2d other) => other.X == X && other.Y == Y;

    /// <inheritdoc/>
    public bool Equals(Vector2d x, Vector2d y) => x.Equals(y);

    /// <summary>
    /// Returns a hash code for this instance, which is based on the combined hash codes of the X and Y components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => StateHash;

    /// <inheritdoc/>
    public int GetHashCode(Vector2d obj) => obj.GetHashCode();

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
