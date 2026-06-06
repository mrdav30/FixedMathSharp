//=======================================================================
// Vector3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
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

    #region Static Operations

    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Add(Vector3d v1, Vector3d v2) => v1 + v2;

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Subtract(Vector3d v1, Vector3d v2) => v1 - v2;

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Multiply(Vector3d v1, Vector3d v2) => v1 * v2;

    /// <summary>
    /// Multiplies each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Multiply(Vector3d value, Fixed64 factor) => value * factor;

    /// <summary>
    /// Divides each component of the first vector by the corresponding component of the second vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Divide(Vector3d v1, Vector3d v2) => v1 / v2;

    /// <summary>
    /// Divides each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Divide(Vector3d value, Fixed64 divisor) => value / divisor;

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
        return new Vector3d(
            FixedMath.Lerp(a.X, b.X, mag),
            FixedMath.Lerp(a.Y, b.Y, mag),
            FixedMath.Lerp(a.Z, b.Z, mag));
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
    public static Vector3d UnclampedLerp(Vector3d a, Vector3d b, Fixed64 t) => (b - a) * t + a;

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
        return dv > v.Magnitude
            ? b
            : a + v.Normal * dv;
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
        dot = FixedMath.Clamp(dot, -Fixed64.One, Fixed64.One);
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
    /// Calculates a position between four points using Catmull-Rom interpolation.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="value2">The second point.</param>
    /// <param name="value3">The third point.</param>
    /// <param name="value4">The fourth point.</param>
    /// <param name="amount">The interpolation factor.</param>
    /// <returns>The interpolated position.</returns>
    public static Vector3d CatmullRom(
        Vector3d value1,
        Vector3d value2,
        Vector3d value3,
        Vector3d value4,
        Fixed64 amount)
    {
        return new Vector3d(
            FixedMath.CatmullRom(value1.X, value2.X, value3.X, value4.X, amount),
            FixedMath.CatmullRom(value1.Y, value2.Y, value3.Y, value4.Y, amount),
            FixedMath.CatmullRom(value1.Z, value2.Z, value3.Z, value4.Z, amount)
        );
    }

    /// <summary>
    /// Calculates a position between two points using Hermite spline interpolation, 
    /// which takes into account the tangents at the endpoints for smoother transitions.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="tangent1">The tangent at the first point.</param>
    /// <param name="value2">The second point.</param>
    /// <param name="tangent2">The tangent at the second point.</param>
    /// <param name="amount">The interpolation factor.</param>
    /// <returns>The interpolated position.</returns>
    public static Vector3d HermiteSpline(
        Vector3d value1,
        Vector3d tangent1,
        Vector3d value2,
        Vector3d tangent2,
        Fixed64 amount)
    {
        return new Vector3d(
            FixedMath.HermiteSpline(value1.X, tangent1.X, value2.X, tangent2.X, amount),
            FixedMath.HermiteSpline(value1.Y, tangent1.Y, value2.Y, tangent2.Y, amount),
            FixedMath.HermiteSpline(value1.Z, tangent1.Z, value2.Z, tangent2.Z, amount));
    }

    /// <summary>
    /// Calculates a position between two points using a cubic Hermite interpolation, 
    /// which is similar to HermiteSpline but assumes zero tangents at the endpoints for a smoother curve.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="value2">The second point.</param>
    /// <param name="amount">The interpolation factor.</param>
    /// <returns>The interpolated position.</returns>
    public static Vector3d SmoothStep(Vector3d value1, Vector3d value2, Fixed64 amount)
    {
        return new Vector3d(
            FixedMath.SmoothStep(value1.X, value2.X, amount),
            FixedMath.SmoothStep(value1.Y, value2.Y, amount),
            FixedMath.SmoothStep(value1.Z, value2.Z, amount)
        );
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
        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return value;

        // Normalize it exactly
        return new Vector3d(
            Fixed64.DivideByPositive(value.X, mag),
            Fixed64.DivideByPositive(value.Y, mag),
            Fixed64.DivideByPositive(value.Z, mag)
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
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y) + (vector.Z * vector.Z);

        // Clamp tiny drift around 1 in either direction.
        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return Fixed64.One;

        return mag != Fixed64.Zero ? FixedMath.Sqrt(mag) : Fixed64.Zero;
    }

    /// <summary>
    /// Returns a new <see cref="Vector3d"/> where each component is the absolute value of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector with absolute values for each component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Abs(Vector3d value) => new(value.X.Abs(), value.Y.Abs(), value.Z.Abs());

    /// <summary>
    /// Returns a new <see cref="Vector3d"/> where each component is the sign of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector where each component is -1, 0, or 1 based on the sign of the input.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Sign(Vector3d value) => new(value.X.Sign(), value.Y.Sign(), value.Z.Sign());

    /// <summary>
    /// Clamps each component of the given <see cref="Vector3d"/> within the specified min and max bounds.
    /// </summary>
    /// <param name="value">The vector to clamp.</param>
    /// <param name="min">The minimum bounds.</param>
    /// <param name="max">The maximum bounds.</param>
    /// <returns>A vector with each component clamped between min and max.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Clamp(Vector3d value, Vector3d min, Vector3d max) =>
        new(FixedMath.Clamp(value.X, min.X, max.X),
            FixedMath.Clamp(value.Y, min.Y, max.Y),
            FixedMath.Clamp(value.Z, min.Z, max.Z));

    /// <summary>
    /// Clamps the given Vector3d within the specified magnitude.
    /// </summary>
    /// <param name="value"></param>
    /// <param name="maxMagnitude"></param>
    /// <returns></returns>
    public static Vector3d ClampMagnitude(Vector3d value, Fixed64 maxMagnitude)
    {
        if (value.SqrMagnitude > maxMagnitude * maxMagnitude)
            return value.Normal * maxMagnitude; // Clamp magnitude without changing direction

        return value;
    }

    /// <summary>
    /// Determines if two vectors are exactly parallel by checking if their cross product is zero.
    /// </summary>
    /// <param name="v1">The first vector.</param>
    /// <param name="v2">The second vector.</param>
    /// <returns>True if the vectors are exactly parallel, false otherwise.</returns>
    public static bool AreParallel(Vector3d v1, Vector3d v2) => Cross(v1, v2).SqrMagnitude == Fixed64.Zero;

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
    public static Vector3d Midpoint(Vector3d v1, Vector3d v2) =>
        new((v1.X + v2.X) * Fixed64.Half, (v1.Y + v2.Y) * Fixed64.Half, (v1.Z + v2.Z) * Fixed64.Half);

    /// <inheritdoc cref="Distance(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector3d start, Vector3d end) => start.Distance(end.X, end.Y, end.Z);

    /// <inheritdoc cref="SqrDistance(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 SqrDistance(Vector3d start, Vector3d end) => start.SqrDistance(end.X, end.Y, end.Z);

    /// <summary>
    /// Calculates the closest points on two line segments.
    /// </summary>
    /// <param name="line1Start">The starting point of the first line segment.</param>
    /// <param name="line1End">The ending point of the first line segment.</param>
    /// <param name="line2Start">The starting point of the second line segment.</param>
    /// <param name="line2End">The ending point of the second line segment.</param>
    /// <returns>
    /// A tuple containing two points representing the closest points on each line segment. 
    /// The first item is the closest point on the first line,
    /// and the second item is the closest point on the second line.
    /// </returns>
    /// <remarks>
    /// This method considers the line segments, not the infinite lines they represent, 
    /// ensuring that the returned points always lie within the provided segments.
    /// </remarks>
    public static (Vector3d, Vector3d) ClosestPointsOnTwoLines(
        Vector3d line1Start,
        Vector3d line1End,
        Vector3d line2Start,
        Vector3d line2End)
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

        (Fixed64 sc, Fixed64 tc) = SolveClosestLineParameters(a, b, c, d, e, D);

        // recompute sc if it is outside [0,1]
        if (sc < Fixed64.Zero)
        {
            sc = Fixed64.Zero;
            tc = ClampSegmentParameter(e, c);
        }
        else if (sc > Fixed64.One)
        {
            sc = Fixed64.One;
            tc = ClampSegmentParameter(e + b, c);
        }

        // recompute tc if it is outside [0,1]
        if (tc < Fixed64.Zero)
        {
            tc = Fixed64.Zero;
            sc = ClampSegmentParameter(-d, a);
        }
        else if (tc > Fixed64.One)
        {
            tc = Fixed64.One;
            sc = ClampSegmentParameter(-d + b, a);
        }

        // get the difference of the two closest points
        Vector3d pointOnLine1 = line1Start + sc * u;
        Vector3d pointOnLine2 = line2Start + tc * v;

        return (pointOnLine1, pointOnLine2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (Fixed64 sc, Fixed64 tc) SolveClosestLineParameters(
        Fixed64 a,
        Fixed64 b,
        Fixed64 c,
        Fixed64 d,
        Fixed64 e,
        Fixed64 determinant)
    {
        if (determinant.Abs() < Fixed64.Epsilon)
            return (Fixed64.Zero, b > c ? d / b : e / c);

        return ((b * e - c * d) / determinant, (a * e - b * d) / determinant);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ClampSegmentParameter(Fixed64 numerator, Fixed64 denominator)
    {
        if (numerator < Fixed64.Zero)
            return Fixed64.Zero;

        if (numerator > denominator)
            return Fixed64.One;

        return numerator / denominator;
    }

    /// <summary>
    /// Calculates the closest point on a line segment defined by start and end points to a given point in space.
    /// </summary>
    /// <param name="point">The point to project onto the segment.</param>
    /// <param name="start">The start of the line segment.</param>
    /// <param name="end">The end of the line segment.</param>
    /// <returns>The closest point on the line segment to the given point.</returns>
    public static Vector3d ClosestPointOnLineSegment(Vector3d point, Vector3d start, Vector3d end)
    {
        Vector3d segment = end - start;
        Fixed64 lengthSquared = segment.SqrMagnitude;

        if (lengthSquared == Fixed64.Zero)
            return start;

        Fixed64 t = Dot(point - start, segment) / lengthSquared;
        t = FixedMath.Clamp(t, Fixed64.Zero, Fixed64.One);

        return start + segment * t;
    }

    /// <summary>
    /// Dot Product of two vectors.
    /// </summary>
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(Vector3d lhs, Vector3d rhs) => lhs.Dot(rhs.X, rhs.Y, rhs.Z);

    /// <summary>
    /// Cross Product of two vectors.
    /// </summary>
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Cross(Vector3d lhs, Vector3d rhs) => lhs.Cross(rhs.X, rhs.Y, rhs.Z);

    /// <inheritdoc cref="CrossProduct(Fixed64, Fixed64, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CrossProduct(Vector3d lhs, Vector3d rhs) => lhs.CrossProduct(rhs.X, rhs.Y, rhs.Z);

    /// <summary>
    /// Projects a vector onto another vector.
    /// </summary>
    /// <param name="vector"></param>
    /// <param name="onNormal"></param>
    /// <returns></returns>
    public static Vector3d Project(Vector3d vector, Vector3d onNormal)
    {
        Fixed64 sqrMag = Dot(onNormal, onNormal);
        if (sqrMag.Abs() < Fixed64.Epsilon)
            return Zero;
        else
        {
            Fixed64 dot = Dot(vector, onNormal);
            return new Vector3d(onNormal.X * dot / sqrMag,
                onNormal.Y * dot / sqrMag,
                onNormal.Z * dot / sqrMag);
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
        if (sqrMag.Abs() < Fixed64.Epsilon)
            return vector;
        else
        {
            Fixed64 dot = Dot(vector, planeNormal);
            return new Vector3d(vector.X - planeNormal.X * dot / sqrMag,
                vector.Y - planeNormal.Y * dot / sqrMag,
                vector.Z - planeNormal.Z * dot / sqrMag);
        }
    }

    /// <summary>
    /// Projects a point onto a plane defined by a normal and a distance from the origin.
    /// </summary>
    /// <param name="point">The point to project.</param>
    /// <param name="plane">The plane onto which the point is projected.</param>
    /// <returns>The projected point.</returns>
    public static Vector3d ProjectOnPlane(Vector3d point, FixedPlane plane)
    {
        Fixed64 normalLengthSquared = plane.Normal.SqrMagnitude;
        if (normalLengthSquared == Fixed64.Zero)
            return point;

        Fixed64 distance = plane.DotCoordinate(point);
        return point - plane.Normal * Fixed64.DivideByPositive(distance, normalLengthSquared);
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
    /// Calculates the barycentric coordinates of a point with respect to a triangle defined by three vertices.
    /// </summary>
    /// <param name="value1">The first vertex of the triangle.</param>
    /// <param name="value2">The second vertex of the triangle.</param>
    /// <param name="value3">The third vertex of the triangle.</param>
    /// <param name="amount1">The first barycentric scalar which represents the weighting factor for the second vertex.</param>
    /// <param name="amount2">The second barycentric scalar which represents the weighting factor for the third vertex.</param>
    /// <returns>The cartesian translation represented by the barycentric coordinates within the triangle.</returns>
    public static Vector3d BarycentricCoordinates(
        Vector3d value1,
        Vector3d value2,
        Vector3d value3,
        Fixed64 amount1,
        Fixed64 amount2)
    {
        return new(
            FixedMath.BarycentricCoordinate(value1.X, value2.X, value3.X, amount1, amount2),
            FixedMath.BarycentricCoordinate(value1.Y, value2.Y, value3.Y, amount1, amount2),
            FixedMath.BarycentricCoordinate(value1.Z, value2.Z, value3.Z, amount1, amount2));
    }

    /// <summary>
    ///  Returns a vector whose elements are the maximum of each of the pairs of elements in two specified vectors.
    /// </summary>
    /// <param name="value1">The first vector.</param>
    /// <param name="value2">The second vector.</param>
    /// <returns>The maximized vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Max(Vector3d value1, Vector3d value2) =>
         new(FixedMath.Max(value1.X, value2.X),
             FixedMath.Max(value1.Y, value2.Y),
             FixedMath.Max(value1.Z, value2.Z));

    /// <summary>
    /// Returns a vector whose elements are the minimum of each of the pairs of elements in two specified vectors.
    /// </summary>
    /// <param name="value1">The first vector.</param>
    /// <param name="value2">The second vector.</param>
    /// <returns>The minimized vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Min(Vector3d value1, Vector3d value2) =>
        new(FixedMath.Min(value1.X, value2.X),
            FixedMath.Min(value1.Y, value2.Y),
            FixedMath.Min(value1.Z, value2.Z));

    /// <summary>
    /// Returns a vector that is the negation of the specified vector, effectively reversing its direction.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Negate(Vector3d value) => -value;

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

    /// <summary>
    /// Reflects a vector off the plane defined by a normal. 
    /// The result is a vector that points in the direction a perfectly reflected ray would go, 
    /// based on the incoming vector and the normal of the plane it reflects off.
    /// </summary>
    /// <param name="vector">The vector to reflect.</param>
    /// <param name="normal">The normal of the plane to reflect off.</param>
    /// <returns>The reflected vector.</returns>
    public static Vector3d Reflect(Vector3d vector, Vector3d normal)
    {
        Fixed64 dot = Dot(vector, normal);
        return vector - 2 * dot * normal;
    }

    /// <summary>
    /// Transforms a vector by the given 4x4 matrix, applying rotation, scaling, and translation as defined by the matrix.
    /// </summary>
    /// <param name="vector">The vector to transform.</param>
    /// <param name="matrix">The transformation matrix.</param>
    /// <returns>The transformed vector.</returns>
    /// <remarks>
    /// Same as <see cref="operator *(Vector3d, Fixed4x4)"/>.
    /// </remarks>
    public static Vector3d Transform(Vector3d vector, Fixed4x4 matrix) => matrix * vector;

    #endregion

    #region Operators

    /// <summary>
    /// Adds two Vector3d instances component-wise.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">The second vector to add.</param>
    /// <returns>A new Vector3d whose components are the sum of the corresponding components of v1 and v2.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator +(Vector3d v1, Vector3d v2) => new(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);

    /// <summary>
    /// Adds a scalar value to each component of the specified vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector to which the scalar value will be added.</param>
    /// <param name="mag">The scalar value to add to each component of the vector.</param>
    /// <returns>A new Vector3d whose components are the sum of the corresponding components of the input vector and the scalar
    /// value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator +(Vector3d v1, Fixed64 mag) => new(v1.X + mag, v1.Y + mag, v1.Z + mag);

    /// <inheritdoc cref="operator +(Vector3d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator +(Fixed64 mag, Vector3d v1) => v1 + mag;

    /// <summary>
    /// Adds a Vector3d instance to a tuple representing x, y, and z components and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">A tuple containing the x, y, and z values to add to the vector.</param>
    /// <returns>A new Vector3d that is the sum of the original vector and the specified tuple components.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator +(Vector3d v1, (int x, int y, int z) v2) => new(v1.X + v2.x, v1.Y + v2.y, v1.Z + v2.z);

    /// <summary>
    /// Adds a 3-tuple of integers to a Vector3d instance, returning the resulting vector.
    /// </summary>
    /// <param name="v2">A tuple containing the X, Y, and Z components to add to the vector.</param>
    /// <param name="v1">The Vector3d instance to which the tuple components are added.</param>
    /// <returns>A new Vector3d representing the sum of the original vector and the specified tuple components.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator +((int x, int y, int z) v2, Vector3d v1) => v1 + v2;

    /// <summary>
    /// Subtracts the components of one Vector3d from another and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector to subtract from.</param>
    /// <param name="v2">The vector to subtract.</param>
    /// <returns>A Vector3d whose components are the result of subtracting the corresponding components of v2 from v1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator -(Vector3d v1, Vector3d v2) => new(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);

    /// <summary>
    /// Subtracts the specified scalar value from each component of the given vector.
    /// </summary>
    /// <param name="v1">The vector from which to subtract the scalar value from each component.</param>
    /// <param name="mag">The scalar value to subtract from each component of the vector.</param>
    /// <returns>A new Vector3d whose components are the result of subtracting the scalar value from the corresponding components
    /// of the input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator -(Vector3d v1, Fixed64 mag) => new(v1.X - mag, v1.Y - mag, v1.Z - mag);

    /// <summary>
    /// Subtracts the specified tuple from the given vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector from which to subtract the tuple values.</param>
    /// <param name="v2">A tuple containing the x, y, and z values to subtract from the vector.</param>
    /// <returns>A new Vector3d representing the result of subtracting the tuple values from the original vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator -(Vector3d v1, (int x, int y, int z) v2) => new(v1.X - v2.x, v1.Y - v2.y, v1.Z - v2.z);

    /// <summary>
    /// Subtracts the components of a specified Vector3d from the corresponding components of a 3-tuple of integers and
    /// returns the resulting Vector3d.
    /// </summary>
    /// <remarks>This operator enables direct subtraction between a tuple of three integers and a Vector3d,
    /// returning a new Vector3d instance.</remarks>
    /// <param name="v1">A tuple containing the x, y, and z components to subtract from.</param>
    /// <param name="v2">The Vector3d whose components are subtracted from the corresponding components of the tuple.</param>
    /// <returns>A Vector3d whose components are the result of subtracting the components of v2 from the corresponding components
    /// of v1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator -((int x, int y, int z) v1, Vector3d v2) => new(v1.x - v2.X, v1.y - v2.Y, v1.z - v2.Z);

    /// <summary>
    /// Negates each component of the specified vector.
    /// </summary>
    /// <param name="v1">The vector whose components are to be negated.</param>
    /// <returns>A new vector whose components are the negated values of the input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator -(Vector3d v1) => new(v1.X * -Fixed64.One, v1.Y * -Fixed64.One, v1.Z * -Fixed64.One);

    /// <summary>
    /// Multiplies the specified vector by a scalar value.
    /// </summary>
    /// <param name="v1">The vector to be scaled.</param>
    /// <param name="mag">The scalar value by which to multiply each component of the vector.</param>
    /// <returns>A new vector whose components are the products of the corresponding components of the input vector and the
    /// scalar value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Vector3d v1, Fixed64 mag) => new(v1.X * mag, v1.Y * mag, v1.Z * mag);

    /// <inheritdoc cref="operator *(Vector3d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Fixed64 mag, Vector3d v1) => new(v1.X * mag, v1.Y * mag, v1.Z * mag);

    /// <summary>
    /// Multiplies each component of the specified vector by the given scalar value.
    /// </summary>
    /// <param name="v1">The vector whose components are to be multiplied.</param>
    /// <param name="mag">The scalar value by which to multiply each component of the vector.</param>
    /// <returns>A new Vector3d whose components are the products of the corresponding components of the input vector and the
    /// scalar value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Vector3d v1, int mag) => new(v1.X * mag, v1.Y * mag, v1.Z * mag);

    /// <inheritdoc cref="operator *(Vector3d, int)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(int mag, Vector3d v1) => new(v1.X * mag, v1.Y * mag, v1.Z * mag);

    /// <summary>
    /// Multiplies a 3x3 matrix by a 3-dimensional vector and returns the resulting vector.
    /// </summary>
    /// <remarks>
    /// This operation applies the linear transformation represented by the matrix to the vector. 
    /// The multiplication is performed using standard matrix-vector multiplication rules.
    /// </remarks>
    /// <param name="matrix">The 3x3 matrix to multiply.</param>
    /// <param name="vector">The 3-dimensional vector to be transformed by the matrix.</param>
    /// <returns>A new Vector3d that is the result of multiplying the specified matrix by the specified vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Fixed3x3 matrix, Vector3d vector) =>
         new(vector.X * matrix.M11 + vector.Y * matrix.M21 + vector.Z * matrix.M31,
             vector.X * matrix.M12 + vector.Y * matrix.M22 + vector.Z * matrix.M32,
             vector.X * matrix.M13 + vector.Y * matrix.M23 + vector.Z * matrix.M33);

    /// <inheritdoc cref="operator *(Fixed3x3, Vector3d)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Vector3d vector, Fixed3x3 matrix) => matrix * vector;

    /// <summary>
    /// Transforms the specified 3D vector by the given 4x4 matrix using homogeneous coordinates.
    /// </summary>
    /// <remarks>
    /// If the matrix is affine, the transformation is performed without perspective division. 
    /// For non-affine matrices, the result is divided by the computed w component to account for perspective
    /// transformations. 
    /// If the computed w component is zero, it is treated as one to avoid division by zero.
    /// </remarks>
    /// <param name="matrix">The 4x4 matrix to apply to the vector. Must represent a valid transformation.</param>
    /// <param name="point">The 3D vector to be transformed.</param>
    /// <returns>A new Vector3d representing the transformed vector after applying the matrix.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Fixed4x4 matrix, Vector3d point)
    {
        if (matrix.IsAffine)
        {
            return new Vector3d(
                point.X * matrix.M11 + point.Y * matrix.M21 + point.Z * matrix.M31 + matrix.M41,
                point.X * matrix.M12 + point.Y * matrix.M22 + point.Z * matrix.M32 + matrix.M42,
                point.X * matrix.M13 + point.Y * matrix.M23 + point.Z * matrix.M33 + matrix.M43
            );
        }

        // Full 4×4 transformation
        Fixed64 w = matrix.M14 * point.X + matrix.M24 * point.Y + matrix.M34 * point.Z + matrix.M44;
        if (w == Fixed64.Zero) w = Fixed64.One;  // Prevent divide-by-zero

        return new Vector3d(
            (point.X * matrix.M11 + point.Y * matrix.M21 + point.Z * matrix.M31 + matrix.M41) / w,
            (point.X * matrix.M12 + point.Y * matrix.M22 + point.Z * matrix.M32 + matrix.M42) / w,
            (point.X * matrix.M13 + point.Y * matrix.M23 + point.Z * matrix.M33 + matrix.M43) / w
        );
    }

    /// <inheritdoc cref="operator *(Fixed4x4, Vector3d)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Vector3d vector, Fixed4x4 matrix) => matrix * vector;

    /// <summary>
    /// Multiplies the corresponding components of two vectors and returns the resulting vector.
    /// </summary>
    /// <remarks>
    /// This operation performs component-wise multiplication, not a dot or cross product. 
    /// Each component of the result is calculated as the product of the corresponding components of the input
    /// vectors.
    /// </remarks>
    /// <param name="v1">The first vector to multiply.</param>
    /// <param name="v2">The second vector to multiply.</param>
    /// <returns>A new Vector3d whose components are the products of the corresponding components of the input vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Vector3d v1, Vector3d v2) => new(v1.X * v2.X, v1.Y * v2.Y, v1.Z * v2.Z);

    /// <summary>
    /// Divides each component of a vector by a specified scalar value.
    /// </summary>
    /// <remarks>If the scalar value is zero, the result is a zero vector to avoid division by zero.</remarks>
    /// <param name="v1">The vector whose components are to be divided.</param>
    /// <param name="div">The scalar value by which to divide each component of the vector.</param>
    /// <returns>
    /// A new vector whose components are the result of dividing the corresponding components of the input vector by the
    /// specified scalar. 
    /// Returns a zero vector if the scalar is zero.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator /(Vector3d v1, Fixed64 div) =>
        div == Fixed64.Zero
        ? Zero
        : new Vector3d(v1.X / div, v1.Y / div, v1.Z / div);

    /// <summary>
    /// Divides each component of one vector by the corresponding component of another vector.
    /// </summary>
    /// <remarks>Division by zero for any component in v2 results in a zero value for the corresponding
    /// component in the result vector.</remarks>
    /// <param name="v1">The vector whose components are to be divided (the dividend).</param>
    /// <param name="v2">The vector whose components are used as divisors.</param>
    /// <returns>
    /// A new Vector3d whose components are the result of dividing the corresponding components of v1 by v2. 
    /// If a component of v2 is zero, the corresponding result component is set to zero.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator /(Vector3d v1, Vector3d v2) =>
        new(v2.X == Fixed64.Zero ? Fixed64.Zero : v1.X / v2.X,
            v2.Y == Fixed64.Zero ? Fixed64.Zero : v1.Y / v2.Y,
            v2.Z == Fixed64.Zero ? Fixed64.Zero : v1.Z / v2.Z);

    /// <summary>
    /// Divides each component of a vector by the specified integer value.
    /// </summary>
    /// <param name="v1">The vector whose components are to be divided.</param>
    /// <param name="div">The integer divisor. If zero, the result is a zero vector.</param>
    /// <returns>
    /// A new vector whose components are the result of dividing each component of the input vector by the specified
    /// divisor. 
    /// Returns a zero vector if the divisor is zero.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator /(Vector3d v1, int div) =>
        div == 0
        ? Zero
        : new Vector3d(v1.X / div, v1.Y / div, v1.Z / div);

    /// <summary>
    /// Rotates the specified 3D point by the given quaternion.
    /// </summary>
    /// <remarks>
    /// This operator applies the rotation to the point as if performing a geometric transformation in 3D space.</remarks>
    /// <param name="point">The 3D point to be rotated.</param>
    /// <param name="rotation">The quaternion representing the rotation to apply.</param>
    /// <returns>A new Vector3d representing the rotated point.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(Vector3d point, FixedQuaternion rotation) => rotation * point;

    /// <inheritdoc cref="operator *(Vector3d, FixedQuaternion)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d operator *(FixedQuaternion rotation, Vector3d point)
    {
        Fixed64 num1 = rotation.X * 2;
        Fixed64 num2 = rotation.Y * 2;
        Fixed64 num3 = rotation.Z * 2;
        Fixed64 num4 = rotation.X * num1;
        Fixed64 num5 = rotation.Y * num2;
        Fixed64 num6 = rotation.Z * num3;
        Fixed64 num7 = rotation.X * num2;
        Fixed64 num8 = rotation.X * num3;
        Fixed64 num9 = rotation.Y * num3;
        Fixed64 num10 = rotation.W * num1;
        Fixed64 num11 = rotation.W * num2;
        Fixed64 num12 = rotation.W * num3;

        Vector3d vector3 = new(
            (Fixed64.One - (num5 + num6)) * point.X + (num7 - num12) * point.Y + (num8 + num11) * point.Z,
            (num7 + num12) * point.X + (Fixed64.One - (num4 + num6)) * point.Y + (num9 - num10) * point.Z,
            (num8 - num11) * point.X + (num9 + num10) * point.Y + (Fixed64.One - (num4 + num5)) * point.Z
        );

        return vector3;
    }

    /// <summary>
    /// Determines whether two Vector3d instances are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Vector3d left, Vector3d right) => left.Equals(right);

    /// <summary>
    /// Determines whether two Vector3d instances are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Vector3d left, Vector3d right) => !left.Equals(right);

    /// <summary>
    /// Determines whether each component of the left vector is greater than the corresponding component of the right
    /// vector.
    /// </summary>
    /// <param name="left">The first vector to compare.</param>
    /// <param name="right">The second vector to compare.</param>
    /// <returns>true if the x, y, and z components of left are all greater than those of right; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Vector3d left, Vector3d right) =>
        left.X > right.X
        && left.Y > right.Y
        && left.Z > right.Z;

    /// <summary>
    /// Determines whether each component of the first Vector3d is less than the corresponding component of the second
    /// Vector3d.
    /// </summary>
    /// <remarks>
    /// This operator performs a component-wise comparison. 
    /// All components of left must be less than the corresponding components of right for the result to be true.</remarks>
    /// <param name="left">The first Vector3d to compare.</param>
    /// <param name="right">The second Vector3d to compare.</param>
    /// <returns>true if the x, y, and z components of left are all less than those of right; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Vector3d left, Vector3d right) =>
        left.X < right.X
        && left.Y < right.Y
        && left.Z < right.Z;

    /// <summary>
    /// Determines whether each component of the left Vector3d is greater than or equal to the corresponding component
    /// of the right Vector3d.
    /// </summary>
    /// <param name="left">The first Vector3d to compare.</param>
    /// <param name="right">The second Vector3d to compare.</param>
    /// <returns>true if the x, y, and z components of left are each greater than or equal to those of right; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Vector3d left, Vector3d right) =>
        left.X >= right.X
        && left.Y >= right.Y
        && left.Z >= right.Z;

    /// <summary>
    /// Determines whether each component of the first Vector3d is less than or equal to the corresponding component of
    /// the second Vector3d.
    /// </summary>
    /// <param name="left">The first Vector3d to compare.</param>
    /// <param name="right">The second Vector3d to compare.</param>
    /// <returns>true if the x, y, and z components of left are each less than or equal to the corresponding components of right;
    /// otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Vector3d left, Vector3d right) =>
        left.X <= right.X
        && left.Y <= right.Y
        && left.Z <= right.Z;

    #endregion

    #region Conversion

    /// <summary>
    /// Returns a string that represents the current object in the format "(x, y, z)".
    /// </summary>
    /// <returns>A string representation of the object, displaying the x, y, and z values in a formatted tuple.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() =>
        string.Format("({0}, {1}, {2})", (double)X, (double)Y, (double)Z);

    /// <summary>
    /// Returns a string that represents the current object in the format "(x, y, z)", 
    /// with each component formatted to a fixed number of decimal places for improved readability.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToFormattedString() =>
        string.Format("({0}, {1}, {2})", X.ToFormattedDouble(), Y.ToFormattedDouble(), Z.ToFormattedDouble());

    /// <summary>
    /// Converts this <see cref="Vector3d"/> to a <see cref="Vector2d"/>, 
    /// dropping the Y component (height) of this vector in the resulting vector.
    /// </summary>
    /// <returns>
    /// A new <see cref="Vector2d"/> where (X, Z) from this <see cref="Vector3d"/> 
    /// become (X, Y) in the resulting vector.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ToVector2d() => new(X, Z);

    /// <summary>
    /// Converts this <see cref="Vector3d"/> to a <see cref="Vector4d"/> with an explicit W component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d ToVector4d(Fixed64 w) => new(X, Y, Z, w);

    /// <summary>
    /// Deconstructs the Vector3d into its three Fixed64 components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Fixed64 x, out Fixed64 y, out Fixed64 z)
    {
        x = X;
        y = Y;
        z = Z;
    }

    /// <summary>
    /// Deconstructs the Vector3d into its three int components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out int x, out int y, out int z)
    {
        x = X.RoundToInt();
        y = Y.RoundToInt();
        z = Z.RoundToInt();
    }

    /// <summary>
    /// Deconstructs the Vector3d into its three long components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out long x, out long y, out long z)
    {
        x = X.m_rawValue;
        y = Y.m_rawValue;
        z = Z.m_rawValue;
    }

    /// <summary>
    /// Deconstructs the Vector3d into its three double components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out double x, out double y, out double z)
    {
        x = (double)X;
        y = (double)Y;
        z = (double)Z;
    }

    /// <summary>
    /// Converts each component of the vector from radians to degrees.
    /// </summary>
    /// <param name="radians">The vector with components in radians.</param>
    /// <returns>A new vector with components converted to degrees.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ToDegrees(Vector3d radians) =>
         new(FixedMath.RadToDeg(radians.X),
             FixedMath.RadToDeg(radians.Y),
             FixedMath.RadToDeg(radians.Z));

    /// <summary>
    /// Converts each component of the vector from degrees to radians.
    /// </summary>
    /// <param name="degrees">The vector with components in degrees.</param>
    /// <returns>A new vector with components converted to radians.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ToRadians(Vector3d degrees) =>
        new(FixedMath.DegToRad(degrees.X),
            FixedMath.DegToRad(degrees.Y),
            FixedMath.DegToRad(degrees.Z));

    #endregion

    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Vector3d other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Vector3d other) => other.X == X && other.Y == Y && other.Z == Z;

    /// <inheritdoc/>
    public bool Equals(Vector3d x, Vector3d y) => x.Equals(y);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => StateHash;

    /// <inheritdoc/>
    public int GetHashCode(Vector3d obj) => obj.GetHashCode();

    /// <summary>
    /// Compares the current Vector3d instance with another Vector3d based on their squared magnitudes.
    /// </summary>
    /// <remarks>
    /// This comparison uses the squared magnitude of each vector, which avoids the computational
    /// cost of calculating the actual magnitude. 
    /// Use this method when only relative vector lengths are
    /// important.
    /// </remarks>
    /// <param name="other">The Vector3d instance to compare with the current instance.</param>
    /// <returns>A value less than zero if this instance is less than <paramref name="other"/>; zero if this instance is equal to
    /// <paramref name="other"/>; or a value greater than zero if this instance is greater than <paramref
    /// name="other"/>, as determined by their squared magnitudes.</returns>
    public int CompareTo(Vector3d other) => SqrMagnitude.CompareTo(other.SqrMagnitude);

    #endregion
}
