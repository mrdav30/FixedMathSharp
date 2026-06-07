//=======================================================================
// Vector3d.cs
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
[MemoryPackable]
public partial struct Vector3d : IEquatable<Vector3d>, IComparable<Vector3d>, IEqualityComparer<Vector3d>
{
    #region Static Readonly Fields

    /// <summary>
    /// The upward direction vector (0, 1, 0).
    /// </summary>
    public static Vector3d Up => new(0, 1, 0);

    /// <summary>
    /// (1, 0, 0)
    /// </summary>
    public static Vector3d Right => new(1, 0, 0);

    /// <summary>
    /// (0, -1, 0)
    /// </summary>
    public static Vector3d Down => new(0, -1, 0);

    /// <summary>
    /// (-1, 0, 0)
    /// </summary>
    public static Vector3d Left => new(-1, 0, 0);

    /// <summary>
    /// The forward direction vector (0, 0, 1).
    /// </summary>
    public static Vector3d Forward => new(0, 0, 1);

    /// <summary>
    /// (0, 0, -1)
    /// </summary>
    public static Vector3d Backward => new(0, 0, -1);

    /// <summary>
    /// (1, 1, 1)
    /// </summary>
    public static Vector3d One => new(1, 1, 1);

    /// <summary>
    /// (-1, -1, -1)
    /// </summary>
    public static Vector3d Negative => new(-1, -1, -1);

    /// <summary>
    /// (0, 0, 0)
    /// </summary>
    public static Vector3d Zero => new(0, 0, 0);

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

    /// <summary>
    /// The Z component of the vector.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(2)]
    public Fixed64 Z;

    #endregion
    #region Constructors

    /// <summary>
    /// Initializes a new instance of the Vector3d structure using integer values for the X, Y, and Z components.
    /// </summary>
    /// <param name="xInt">The value of the X component as an integer.</param>
    /// <param name="yInt">The value of the Y component as an integer.</param>
    /// <param name="zInt">The value of the Z component as an integer.</param>
    public Vector3d(int xInt, int yInt, int zInt) : this((Fixed64)xInt, (Fixed64)yInt, (Fixed64)zInt) { }

    /// <summary>
    /// Initializes a new instance of the Vector3d structure with the specified X, Y, and Z components.
    /// </summary>
    /// <param name="x">The value of the X component of the vector.</param>
    /// <param name="y">The value of the Y component of the vector.</param>
    /// <param name="z">The value of the Z component of the vector.</param>
    [JsonConstructor]
    public Vector3d(Fixed64 x, Fixed64 y, Fixed64 z)
    {
        this.X = x;
        this.Y = y;
        this.Z = z;
    }

    /// <summary>
    /// Initializes a new instance of the Vector3d structure using the specified X, Y, and Z coordinates as
    /// double-precision floating-point values.
    /// </summary>
    /// <remarks>This constructor allows for convenient creation of a Vector3d from double values, which are
    /// internally converted to the Fixed64 representation used by the structure.</remarks>
    /// <param name="xDoub">The X coordinate of the vector, specified as a double-precision floating-point value.</param>
    /// <param name="yDoub">The Y coordinate of the vector, specified as a double-precision floating-point value.</param>
    /// <param name="zDoub">The Z coordinate of the vector, specified as a double-precision floating-point value.</param>
    public static Vector3d FromFloatPoint(double xDoub, double yDoub, double zDoub) =>
        new(Fixed64.FromFloatPoint(xDoub),
            Fixed64.FromFloatPoint(yDoub),
            Fixed64.FromFloatPoint(zDoub));

    #endregion
    #region Properties

    /// <summary>
    ///  Provides a rotated version of the current vector, where rotation is a 90 degrees rotation around the Y axis in the counter-clockwise direction.
    /// </summary>
    /// <remarks>
    /// These operations rotate the vector 90 degrees around the Y-axis.
    /// Note that the positive direction of rotation is defined by the right-hand rule:
    /// If your right hand's thumb points in the positive Y direction, then your fingers curl in the positive direction of rotation.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d RightHandNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(Z, Y, -X);
    }

    /// <summary>
    /// Provides a rotated version of the current vector, where rotation is a 90 degrees rotation around the Y axis in the clockwise direction.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d LeftHandNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(-Z, Y, X);
    }

    /// <inheritdoc cref="GetNormalized(Vector3d)"/>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Normal
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
    /// This vector's square magnitude (aka length squared).
    /// If you're doing distance checks, use SqrMagnitude and square the distance you're checking against
    /// If you need to know the actual distance, use MyMagnitude
    /// </summary>
    /// <returns>The magnitude.</returns>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 SqrMagnitude
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (X * X) + (Y * Y) + (Z * Z);
    }

    /// <summary>
    /// Calculates the forward direction vector from pitch (x) and yaw (y) angles.
    /// </summary>
    /// <remarks>
    /// This is commonly used to determine the direction an object is facing in 3D space,
    /// where 'x' represents pitch around the X axis and 'y' represents yaw around the Y axis.
    /// Positive pitch rotates the forward direction toward <see cref="Down"/>.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Direction
    {
        get
        {
            Fixed64 temp1 = FixedMath.Cos(X) * FixedMath.Sin(Y);
            Fixed64 temp2 = FixedMath.Sin(-X);
            Fixed64 temp3 = FixedMath.Cos(X) * FixedMath.Cos(Y);
            return new Vector3d(temp1, temp2, temp3);
        }
    }

    /// <summary>
    /// Are all components of this vector equal to zero?
    /// </summary>
    /// <returns></returns>
    [JsonIgnore]
    [MemoryPackIgnore]
    public bool IsZero
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.Equals(Zero);
    }

    /// <summary>
    /// Returns a long hash of the vector based on its x, y, and z values.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public long LongStateHash
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (X.m_rawValue * 31) + (Y.m_rawValue * 7) + (Z.m_rawValue * 11);
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
    /// Gets or sets the component value at the specified index.
    /// </summary>
    /// <remarks>
    /// Use this indexer to access or modify the x, y, or z components of the vector by index. 
    /// Index 0 corresponds to x, 1 to y, and 2 to z.
    /// </remarks>
    /// <param name="index">The zero-based index of the component to access. Valid values are 0 (x), 1 (y), or 2 (z).</param>
    /// <returns>The value of the component at the specified index.</returns>
    /// <exception cref="IndexOutOfRangeException">Thrown if index is less than 0 or greater than 2.</exception>
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
                2 => Z,
                _ => throw new IndexOutOfRangeException("Invalid Vector3d index!"),
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
                case 2:
                    Z = value;
                    break;
                default:
                    throw new IndexOutOfRangeException("Invalid Vector3d index!");
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
    /// <param name="newZ"></param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d Set(Fixed64 newX, Fixed64 newY, Fixed64 newZ)
    {
        X = newX;
        Y = newY;
        Z = newZ;
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
        X += xAmount;
        Y += yAmount;
        Z += zAmount;
        return this;
    }

    /// <summary>
    /// Adds the specified values to the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="amount">The amount to add to the components.</param>
    /// <returns>The modified vector after addition.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d AddInPlace(Fixed64 amount) => AddInPlace(amount, amount, amount);

    /// <summary>
    /// Adds the specified vector components to the corresponding components of the in place vector and returns the modified vector.
    /// </summary>
    /// <param name="other">The other vector to add the components.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d AddInPlace(Vector3d other) => AddInPlace(other.X, other.Y, other.Z);

    /// <summary>
    /// Subtracts the specified values from the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="xAmount">The amount to subtract from the x component.</param>
    /// <param name="yAmount">The amount to subtract from the y component.</param>
    /// <param name="zAmount">The amount to subtract from the z component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d SubtractInPlace(Fixed64 xAmount, Fixed64 yAmount, Fixed64 zAmount)
    {
        X -= xAmount;
        Y -= yAmount;
        Z -= zAmount;
        return this;
    }

    /// <summary>
    /// Subtracts the specified value from all components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="amount">The amount to subtract from each component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d SubtractInPlace(Fixed64 amount) => SubtractInPlace(amount, amount, amount);

    /// <summary>
    /// Subtracts the specified vector from the components of the vector in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d SubtractInPlace(Vector3d other) => SubtractInPlace(other.X, other.Y, other.Z);

    /// <summary>
    /// Multiplies each component of the vector by the corresponding factor in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d MultiplyInPlace(Fixed64 factorX, Fixed64 factorY, Fixed64 factorZ)
    {
        X *= factorX;
        Y *= factorY;
        Z *= factorZ;
        return this;
    }

    /// <summary>
    /// Multiplies the components of the vector by the specified scalar factor in place and returns the modified vector.
    /// </summary>
    /// <param name="factor">The scalar factor to multiply each component by.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d MultiplyInPlace(Fixed64 factor) => MultiplyInPlace(factor, factor, factor);

    /// <summary>
    /// Multiplies each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
    /// </summary>
    /// <param name="factor">The vector containing the multiplication factors for each component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d MultiplyInPlace(Vector3d factor) => MultiplyInPlace(factor.X, factor.Y, factor.Z);

    /// <summary>
    /// Divides each component of the vector by the corresponding divisor in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d DivideInPlace(Fixed64 divisorX, Fixed64 divisorY, Fixed64 divisorZ)
    {
        X = divisorX == Fixed64.Zero ? Fixed64.Zero : X / divisorX;
        Y = divisorY == Fixed64.Zero ? Fixed64.Zero : Y / divisorY;
        Z = divisorZ == Fixed64.Zero ? Fixed64.Zero : Z / divisorZ;
        return this;
    }

    /// <summary>
    /// Divides each component of the vector by the specified scalar divisor in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d DivideInPlace(Fixed64 divisor)
    {
        if (divisor == Fixed64.Zero)
            return this = Zero;

        X /= divisor;
        Y /= divisor;
        Z /= divisor;
        return this;
    }

    /// <summary>
    /// Divides each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d DivideInPlace(Vector3d divisor) => DivideInPlace(divisor.X, divisor.Y, divisor.Z);

    /// <summary>
    /// Normalizes this vector in place, making its magnitude (length) equal to 1, and returns the modified vector.
    /// </summary>
    /// <remarks>
    /// If the vector is zero-length or already normalized, no operation is performed. 
    /// This method modifies the current vector in place and supports method chaining.
    /// </remarks>
    /// <returns>The normalized vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d Normalize() => this = GetNormalized(this);

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
            X = Fixed64.Zero;
            Y = Fixed64.Zero;
            Z = Fixed64.Zero;
            return this;
        }

        // If already normalized, return as-is
        if (mag == Fixed64.One)
            return this;

        X = Fixed64.DivideByPositive(X, mag);
        Y = Fixed64.DivideByPositive(Y, mag);
        Z = Fixed64.DivideByPositive(Z, mag);

        return this;
    }

    /// <summary>
    /// Checks if this vector has been normalized by checking if the magnitude is close to 1.
    /// </summary>
    public bool IsNormalized()
    {
        Fixed64 sqrMagnitude = SqrMagnitude;
        return sqrMagnitude != Fixed64.Zero && FixedMath.Abs(sqrMagnitude - Fixed64.One) <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Checks whether all components are strictly greater than <see cref="Fixed64.Epsilon"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllComponentsGreaterThanEpsilon() =>
        X.Abs() > Fixed64.Epsilon && Y.Abs() > Fixed64.Epsilon && Z.Abs() > Fixed64.Epsilon;

    /// <summary>
    /// Returns a new vector with components whose absolute values are less than the specified threshold set to zero.
    /// </summary>
    /// <remarks>
    /// This method is useful for eliminating insignificant floating-point errors by zeroing out very small vector components. 
    /// The default threshold is suitable for most cases where near-zero values are considered noise.
    /// </remarks>
    /// <param name="threshold">
    /// The minimum absolute value a component must have to be retained. 
    /// If null, a default epsilon value is used.
    /// </param>
    /// <returns>A new Vector3d instance with small components snapped to zero based on the specified threshold.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d SnapSmallComponentsToZero(Fixed64? threshold = null)
    {
        Fixed64 effectiveThreshold = threshold ?? Fixed64.Epsilon;
        return new Vector3d(
            X.Abs() < effectiveThreshold ? Fixed64.Zero : X,
            Y.Abs() < effectiveThreshold ? Fixed64.Zero : Y,
            Z.Abs() < effectiveThreshold ? Fixed64.Zero : Z
        );
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
        Fixed64 temp1 = X - otherX;
        temp1 *= temp1;
        Fixed64 temp2 = Y - otherY;
        temp2 *= temp2;
        Fixed64 temp3 = Z - otherZ;
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
        Fixed64 temp1 = X - otherX;
        temp1 *= temp1;
        Fixed64 temp2 = Y - otherY;
        temp2 *= temp2;
        Fixed64 temp3 = Z - otherZ;
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
    public Fixed64 Dot(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ) =>
        X * otherX + Y * otherY + Z * otherZ;

    /// <summary>
    /// Computes the cross product magnitude of this vector with another vector.
    /// </summary>
    /// <param name="otherX">The X component of the other vector.</param>
    /// <param name="otherY">The Y component of the other vector.</param>
    /// <param name="otherZ">The Z component of the other vector.</param>
    /// <returns>The cross product magnitude.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 CrossProduct(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ) =>
        (Y * otherZ - Z * otherY) + (Z * otherX - X * otherZ) + (X * otherY - Y * otherX);

    /// <summary>
    /// Returns the cross vector of this vector with another vector.
    /// </summary>
    /// <returns>A new vector representing the cross product.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d Cross(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ) =>
        new(Y * otherZ - Z * otherY,
            Z * otherX - X * otherZ,
            X * otherY - Y * otherX);

    #endregion
}
