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
        X = x;
        Y = y;
    }

    /// <summary>
    /// Initializes a new instance of the Vector2d structure using double-precision floating-point values for the X and Y components.
    /// floating-point values.
    /// </summary>
    public static Vector2d FromDouble(double xDoub, double yDoub) =>
        new(Fixed64.FromDouble(xDoub), Fixed64.FromDouble(yDoub));

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
    public Vector2d Normalized
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
    public Fixed64 MagnitudeSquared
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
    public Vector2d NormalizeInPlace() => this = GetNormalized(this);

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
    public Fixed64 DistanceSquared(Fixed64 otherX, Fixed64 otherY)
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
    public Fixed64 DistanceSquared(Vector2d other) => DistanceSquared(other.X, other.Y);

    #endregion
}
