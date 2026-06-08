//=======================================================================
// Vector4d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a 4D vector with fixed-point precision.
/// </summary>
/// <remarks>
/// This type is useful for generic 4D component math and homogeneous coordinates used by 4x4 transforms.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Vector4d : IEquatable<Vector4d>, IComparable<Vector4d>, IEqualityComparer<Vector4d>, IFormattable
#if NET8_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly Fields

    /// <summary>
    /// (1, 0, 0, 0)
    /// </summary>
    public static Vector4d UnitX => new(1, 0, 0, 0);

    /// <summary>
    /// (0, 1, 0, 0)
    /// </summary>
    public static Vector4d UnitY => new(0, 1, 0, 0);

    /// <summary>
    /// (0, 0, 1, 0)
    /// </summary>
    public static Vector4d UnitZ => new(0, 0, 1, 0);

    /// <summary>
    /// (0, 0, 0, 1)
    /// </summary>
    public static Vector4d UnitW => new(0, 0, 0, 1);

    /// <summary>
    /// (1, 1, 1, 1)
    /// </summary>
    public static Vector4d One => new(1, 1, 1, 1);

    /// <summary>
    /// (-1, -1, -1, -1)
    /// </summary>
    public static Vector4d Negative => new(-1, -1, -1, -1);

    /// <summary>
    /// (0, 0, 0, 0)
    /// </summary>
    public static Vector4d Zero => new(0, 0, 0, 0);

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

    /// <summary>
    /// The W component of the vector.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(3)]
    public Fixed64 W;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new Vector4d using integer component values.
    /// </summary>
    public Vector4d(int xInt, int yInt, int zInt, int wInt)
        : this((Fixed64)xInt, (Fixed64)yInt, (Fixed64)zInt, (Fixed64)wInt) { }

    /// <summary>
    /// Initializes a new Vector4d using the specified components.
    /// </summary>
    [JsonConstructor]
    public Vector4d(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 w)
    {
        X = x;
        Y = y;
        Z = z;
        W = w;
    }

    /// <summary>
    /// Initializes a new Vector4d with all components set to the same value.
    /// </summary>
    public Vector4d(Fixed64 value)
        : this(value, value, value, value) { }

    /// <summary>
    /// Initializes a new Vector4d from a Vector2d plus Z and W components.
    /// </summary>
    public Vector4d(Vector2d value, Fixed64 z, Fixed64 w)
        : this(value.X, value.Y, z, w) { }

    /// <summary>
    /// Initializes a new Vector4d from a Vector3d plus a W component.
    /// </summary>
    public Vector4d(Vector3d value, Fixed64 w)
        : this(value.X, value.Y, value.Z, w) { }

    /// <summary>
    /// Initializes a new Vector4d using double component values.
    /// </summary>
    /// <remarks>
    /// Components are converted through <see cref="Fixed64.FromDouble(double)"/>, so non-finite
    /// values throw <see cref="ArgumentOutOfRangeException"/> and finite values outside the Q32.32
    /// range throw <see cref="OverflowException"/>.
    /// </remarks>
    public static Vector4d FromDouble(double xDoub, double yDoub, double zDoub, double wDoub) =>
        new(Fixed64.FromDouble(xDoub), Fixed64.FromDouble(yDoub), Fixed64.FromDouble(zDoub), Fixed64.FromDouble(wDoub));

    #endregion

    #region Properties

    /// <inheritdoc cref="GetNormalized(Vector4d)"/>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector4d Normalized
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetNormalized(this);
    }

    /// <summary>
    /// Returns the actual length of this vector.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Fixed64 Magnitude
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetMagnitude(this);
    }

    /// <summary>
    /// Returns the square magnitude of the vector.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Fixed64 MagnitudeSquared
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (X * X) + (Y * Y) + (Z * Z) + (W * W);
    }

    /// <summary>
    /// Returns a long hash of the vector based on its component values.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly long LongStateHash
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            unchecked
            {
                return (X.m_rawValue * 31) + (Y.m_rawValue * 7) + (Z.m_rawValue * 11) + (W.m_rawValue * 17);
            }
        }
    }

    /// <summary>
    /// Returns a hash of the vector based on its state.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly int StateHash
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => unchecked((int)(LongStateHash % int.MaxValue));
    }

    /// <summary>
    /// Are all components of this vector equal to zero?
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly bool IsZero
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Equals(Zero);
    }

    /// <summary>
    /// Gets or sets the vector component at the specified index.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get
        {
            return index switch
            {
                0 => X,
                1 => Y,
                2 => Z,
                3 => W,
                _ => throw new IndexOutOfRangeException("Invalid Vector4d index!"),
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
                case 3:
                    W = value;
                    break;
                default:
                    throw new IndexOutOfRangeException("Invalid Vector4d index!");
            }
        }
    }

    #endregion

    #region Instance Methods

    /// <summary>
    /// Sets the vector components in place and returns the modified vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d Set(Fixed64 newX, Fixed64 newY, Fixed64 newZ, Fixed64 newW)
    {
        X = newX;
        Y = newY;
        Z = newZ;
        W = newW;
        return this;
    }

    /// <summary>
    /// Adds the specified component values in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d AddInPlace(Fixed64 xAmount, Fixed64 yAmount, Fixed64 zAmount, Fixed64 wAmount)
    {
        X += xAmount;
        Y += yAmount;
        Z += zAmount;
        W += wAmount;
        return this;
    }

    /// <summary>
    /// Adds the specified scalar to all components in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d AddInPlace(Fixed64 amount) => AddInPlace(amount, amount, amount, amount);

    /// <summary>
    /// Adds another vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d AddInPlace(Vector4d other) => AddInPlace(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Subtracts the specified component values in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d SubtractInPlace(Fixed64 xAmount, Fixed64 yAmount, Fixed64 zAmount, Fixed64 wAmount)
    {
        X -= xAmount;
        Y -= yAmount;
        Z -= zAmount;
        W -= wAmount;
        return this;
    }

    /// <summary>
    /// Subtracts the specified scalar from all components in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d SubtractInPlace(Fixed64 amount) => SubtractInPlace(amount, amount, amount, amount);


    /// <summary>
    /// Subtracts another vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d SubtractInPlace(Vector4d other) => SubtractInPlace(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Multiplies the vector components by the specified factors in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d MultiplyInPlace(Fixed64 factorX, Fixed64 factorY, Fixed64 factorZ, Fixed64 factorW)
    {
        X *= factorX;
        Y *= factorY;
        Z *= factorZ;
        W *= factorW;
        return this;
    }

    /// <summary>
    /// Multiplies all components in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d MultiplyInPlace(Fixed64 factor) => MultiplyInPlace(factor, factor, factor, factor);

    /// <summary>
    /// Multiplies all components by another vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d MultiplyInPlace(Vector4d factor) => MultiplyInPlace(factor.X, factor.Y, factor.Z, factor.W);

    /// <summary>
    /// Divides the vector components by the specified divisors in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d DivideInPlace(Fixed64 divisorX, Fixed64 divisorY, Fixed64 divisorZ, Fixed64 divisorW)
    {
        X /= divisorX;
        Y /= divisorY;
        Z /= divisorZ;
        W /= divisorW;
        return this;
    }

    /// <summary>
    /// Divides all components in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d DivideInPlace(Fixed64 divisor) => DivideInPlace(divisor, divisor, divisor, divisor);

    /// <summary>
    /// Divides all components by another vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d DivideInPlace(Vector4d divisor) => DivideInPlace(divisor.X, divisor.Y, divisor.Z, divisor.W);

    /// <summary>
    /// Normalizes this vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d NormalizeInPlace() => this = GetNormalized(this);

    /// <summary>
    /// Normalizes this vector in place and returns its original magnitude.
    /// </summary>
    public Vector4d NormalizeInPlace(out Fixed64 mag)
    {
        mag = GetMagnitude(this);

        if (mag == Fixed64.Zero)
            return this = Zero;

        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return this;

        return this = new Vector4d(
            FixedMath.FastDiv(X, mag),
            FixedMath.FastDiv(Y, mag),
            FixedMath.FastDiv(Z, mag),
            FixedMath.FastDiv(W, mag));
    }

    /// <summary>
    /// Determines whether this vector is normalized.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool IsNormalized() =>
        !IsZero && FixedMath.Abs(MagnitudeSquared - Fixed64.One) <= Fixed64.Epsilon;

    /// <summary>
    /// Determines whether all components are greater than Fixed64.Epsilon.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool AllComponentsGreaterThanEpsilon() =>
        X > Fixed64.Epsilon && Y > Fixed64.Epsilon && Z > Fixed64.Epsilon && W > Fixed64.Epsilon;

    /// <summary>
    /// Snaps components with absolute values below the threshold to zero.
    /// </summary>
    public readonly Vector4d SnapSmallComponentsToZero(Fixed64? threshold = null)
    {
        Fixed64 t = threshold ?? Fixed64.Epsilon;
        return new Vector4d(
            FixedMath.Abs(X) < t ? Fixed64.Zero : X,
            FixedMath.Abs(Y) < t ? Fixed64.Zero : Y,
            FixedMath.Abs(Z) < t ? Fixed64.Zero : Z,
            FixedMath.Abs(W) < t ? Fixed64.Zero : W);
    }

    /// <summary>
    /// Calculates the distance to another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Distance(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ, Fixed64 otherW) =>
        FixedMath.Sqrt(DistanceSquared(otherX, otherY, otherZ, otherW));

    /// <summary>
    /// Calculates the distance to another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Distance(Vector4d other) => Distance(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Calculates the squared distance to another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 DistanceSquared(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ, Fixed64 otherW)
    {
        Fixed64 dx = otherX - X;
        Fixed64 dy = otherY - Y;
        Fixed64 dz = otherZ - Z;
        Fixed64 dw = otherW - W;
        return (dx * dx) + (dy * dy) + (dz * dz) + (dw * dw);
    }

    /// <summary>
    /// Calculates the squared distance to another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 DistanceSquared(Vector4d other) => DistanceSquared(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Calculates the dot product with another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Dot(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ, Fixed64 otherW) =>
        (X * otherX) + (Y * otherY) + (Z * otherZ) + (W * otherW);

    /// <summary>
    /// Calculates the dot product with another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Dot(Vector4d other) => Dot(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Converts this vector to Vector3d by dropping the W component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3d ToVector3d() => new Vector3d(X, Y, Z);

    #endregion

    #region Static Methods

    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Add(Vector4d v1, Vector4d v2) => v1 + v2;

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Subtract(Vector4d v1, Vector4d v2) => v1 - v2;

    /// <summary>
    /// Linearly interpolates between two vectors, clamping the interpolation amount to [0, 1].
    /// </summary>
    public static Vector4d Lerp(Vector4d a, Vector4d b, Fixed64 amount)
    {
        amount = FixedMath.Clamp01(amount);
        return UnclampedLerp(a, b, amount);
    }

    /// <summary>
    /// Linearly interpolates between two vectors without clamping the interpolation amount.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d UnclampedLerp(Vector4d a, Vector4d b, Fixed64 t) =>
        new(a.X + ((b.X - a.X) * t),
            a.Y + ((b.Y - a.Y) * t),
            a.Z + ((b.Z - a.Z) * t),
            a.W + ((b.W - a.W) * t));

    /// <summary>
    /// Normalizes the given vector, returning a unit vector with the same direction.
    /// </summary>
    public static Vector4d GetNormalized(Vector4d value)
    {
        Fixed64 mag = GetMagnitude(value);

        if (mag == Fixed64.Zero)
            return Zero;

        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return value;

        return new Vector4d(
            FixedMath.FastDiv(value.X, mag),
            FixedMath.FastDiv(value.Y, mag),
            FixedMath.FastDiv(value.Z, mag),
            FixedMath.FastDiv(value.W, mag));
    }

    /// <summary>
    /// Returns the magnitude of the given vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetMagnitude(Vector4d vector)
    {
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y) + (vector.Z * vector.Z) + (vector.W * vector.W);

        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return Fixed64.One;

        return mag != Fixed64.Zero ? FixedMath.Sqrt(mag) : Fixed64.Zero;
    }

    /// <summary>
    /// Returns a new vector containing the absolute value of each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Abs(Vector4d value) =>
        new(FixedMath.Abs(value.X),
            FixedMath.Abs(value.Y),
            FixedMath.Abs(value.Z),
            FixedMath.Abs(value.W));

    /// <summary>
    /// Returns a new vector containing the sign of each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Sign(Vector4d value) =>
        new(value.X.Sign(),
            value.Y.Sign(),
            value.Z.Sign(),
            value.W.Sign());

    /// <summary>
    /// Clamps each component of the given vector within the specified min and max bounds.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Clamp(Vector4d value, Vector4d min, Vector4d max) =>
        new(FixedMath.Clamp(value.X, min.X, max.X),
            FixedMath.Clamp(value.Y, min.Y, max.Y),
            FixedMath.Clamp(value.Z, min.Z, max.Z),
            FixedMath.Clamp(value.W, min.W, max.W));

    /// <summary>
    /// Computes the midpoint between two vectors.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Midpoint(Vector4d v1, Vector4d v2) =>
        new((v1.X + v2.X) * Fixed64.Half,
            (v1.Y + v2.Y) * Fixed64.Half,
            (v1.Z + v2.Z) * Fixed64.Half,
            (v1.W + v2.W) * Fixed64.Half);

    /// <inheritdoc cref="Distance(Fixed64, Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector4d start, Vector4d end) =>
        start.Distance(end.X, end.Y, end.Z, end.W);

    /// <inheritdoc cref="DistanceSquared(Fixed64, Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 DistanceSquared(Vector4d start, Vector4d end) =>
        start.DistanceSquared(end.X, end.Y, end.Z, end.W);

    /// <summary>
    /// Calculates the dot product of two vectors.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(Vector4d lhs, Vector4d rhs) =>
        lhs.Dot(rhs.X, rhs.Y, rhs.Z, rhs.W);

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Multiply(Vector4d a, Vector4d b) =>
        new(a.X * b.X, a.Y * b.Y, a.Z * b.Z, a.W * b.W);

    /// <summary>
    /// Multiplies each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Multiply(Vector4d value, Fixed64 factor) => value * factor;

    /// <summary>
    /// Divides each component of the first vector by the corresponding component of the second vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Divide(Vector4d v1, Vector4d v2) => v1 / v2;

    /// <summary>
    /// Divides each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Divide(Vector4d value, Fixed64 divisor) => value / divisor;

    /// <summary>
    /// Returns a vector whose elements are the maximum of each pair of elements.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Max(Vector4d value1, Vector4d value2) =>
        new(FixedMath.Max(value1.X, value2.X),
            FixedMath.Max(value1.Y, value2.Y),
            FixedMath.Max(value1.Z, value2.Z),
            FixedMath.Max(value1.W, value2.W));

    /// <summary>
    /// Returns a vector whose elements are the minimum of each pair of elements.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Min(Vector4d value1, Vector4d value2) =>
        new(FixedMath.Min(value1.X, value2.X),
            FixedMath.Min(value1.Y, value2.Y),
            FixedMath.Min(value1.Z, value2.Z),
            FixedMath.Min(value1.W, value2.W));

    /// <summary>
    /// Transforms a 4D vector by a 4x4 matrix.
    /// </summary>
    /// <remarks>
    /// This follows the same row-vector storage convention as <see cref="Fixed4x4.TransformPoint(Fixed4x4, Vector3d)"/>,
    /// preserving the computed W component instead of performing perspective division.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Transform(Fixed4x4 matrix, Vector4d vector) =>
        new(vector.X * matrix.M11 + vector.Y * matrix.M21 + vector.Z * matrix.M31 + vector.W * matrix.M41,
            vector.X * matrix.M12 + vector.Y * matrix.M22 + vector.Z * matrix.M32 + vector.W * matrix.M42,
            vector.X * matrix.M13 + vector.Y * matrix.M23 + vector.Z * matrix.M33 + vector.W * matrix.M43,
            matrix.M14 * vector.X + matrix.M24 * vector.Y + matrix.M34 * vector.Z + matrix.M44 * vector.W);

    #endregion

    #region Operators

    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, Vector4d v2) => new(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z, v1.W + v2.W);

    /// <summary>
    /// Adds a scalar to each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, Fixed64 mag) => new(v1.X + mag, v1.Y + mag, v1.Z + mag, v1.W + mag);

    /// <inheritdoc cref="operator +(Vector4d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Fixed64 mag, Vector4d v1) => v1 + mag;

    /// <summary>
    /// Adds a tuple to each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, (int x, int y, int z, int w) v2) =>
        new(v1.X + v2.x, v1.Y + v2.y, v1.Z + v2.z, v1.W + v2.w);

    /// <summary>
    /// Adds a tuple to each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +((int x, int y, int z, int w) v2, Vector4d v1) => v1 + v2;

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, Vector4d v2) =>
        new(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z, v1.W - v2.W);

    /// <summary>
    /// Subtracts a scalar from each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, Fixed64 mag) =>
        new(v1.X - mag, v1.Y - mag, v1.Z - mag, v1.W - mag);

    /// <inheritdoc cref="operator -(Vector4d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Fixed64 mag, Vector4d v1) =>
        new(mag - v1.X, mag - v1.Y, mag - v1.Z, mag - v1.W);

    /// <summary>
    /// Subtracts a tuple from each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, (int x, int y, int z, int w) v2) =>
        new(v1.X - v2.x, v1.Y - v2.y, v1.Z - v2.z, v1.W - v2.w);

    /// <summary>
    /// Subtracts each vector component from a tuple component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -((int x, int y, int z, int w) v1, Vector4d v2) =>
        new(v1.x - v2.X, v1.y - v2.Y, v1.z - v2.Z, v1.w - v2.W);

    /// <summary>
    /// Negates the vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1) =>
        new(-v1.X, -v1.Y, -v1.Z, -v1.W);

    /// <summary>
    /// Multiplies each component by a scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, Fixed64 mag) =>
        new(v1.X * mag, v1.Y * mag, v1.Z * mag, v1.W * mag);

    /// <inheritdoc cref="operator *(Vector4d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Fixed64 mag, Vector4d v1) => v1 * mag;

    /// <summary>
    /// Multiplies each component by an integer scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, int mag) => v1 * (Fixed64)mag;

    /// <inheritdoc cref="operator *(Vector4d, int)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(int mag, Vector4d v1) => v1 * mag;

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, Vector4d v2) => Multiply(v1, v2);

    /// <summary>
    /// Transforms a vector by a 4x4 matrix.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Fixed4x4 matrix, Vector4d vector) => Transform(matrix, vector);

    /// <inheritdoc cref="operator *(Fixed4x4, Vector4d)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d vector, Fixed4x4 matrix) => matrix * vector;

    /// <summary>
    /// Divides each component by a scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, Fixed64 div) => new(v1.X / div, v1.Y / div, v1.Z / div, v1.W / div);

    /// <summary>
    /// Divides each component by another vector's corresponding component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, Vector4d v2) => new(v1.X / v2.X, v1.Y / v2.Y, v1.Z / v2.Z, v1.W / v2.W);

    /// <summary>
    /// Divides each component by an integer scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, int div) => v1 / (Fixed64)div;

    /// <summary>
    /// Compares two vectors for equality.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Vector4d left, Vector4d right) => left.Equals(right);

    /// <summary>
    /// Compares two vectors for inequality.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Vector4d left, Vector4d right) => !left.Equals(right);

    /// <summary>
    /// Returns true when every component in the left vector is greater than the corresponding component in the right vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Vector4d left, Vector4d right) =>
        left.X > right.X && left.Y > right.Y && left.Z > right.Z && left.W > right.W;

    /// <summary>
    /// Returns true when every component in the left vector is less than the corresponding component in the right vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Vector4d left, Vector4d right) =>
        left.X < right.X && left.Y < right.Y && left.Z < right.Z && left.W < right.W;

    /// <summary>
    /// Returns true when every component in the left vector is greater than or equal to the corresponding component in the right vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Vector4d left, Vector4d right) =>
        left.X >= right.X && left.Y >= right.Y && left.Z >= right.Z && left.W >= right.W;

    /// <summary>
    /// Returns true when every component in the left vector is less than or equal to the corresponding component in the right vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Vector4d left, Vector4d right) =>
        left.X <= right.X && left.Y <= right.Y && left.Z <= right.Z && left.W <= right.W;

    #endregion

    #region Conversion and Formatting

    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly string ToString(string? format, IFormatProvider? formatProvider)
    {
        Vector4d value = this;
        return FixedDiagnosticsFormatter.ToString((Span<char> destination, out int charsWritten) =>
            value.TryFormat(destination, out charsWritten, format.AsSpan(), formatProvider));
    }

    /// <summary>
    /// Formats this vector into the provided destination buffer.
    /// </summary>
    public readonly bool TryFormat(
        Span<char> destination,
        out int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        int written = 0;
        if (!FixedDiagnosticsFormatter.Append('(', destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(X, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(", ", destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(Y, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(", ", destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(Z, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(", ", destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(W, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(')', destination, ref written))
        {
            charsWritten = 0;
            return false;
        }

        charsWritten = written;
        return true;
    }

    /// <summary>
    /// Deconstructs the Vector4d into its four Fixed64 components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Fixed64 x, out Fixed64 y, out Fixed64 z, out Fixed64 w)
    {
        x = X;
        y = Y;
        z = Z;
        w = W;
    }

    /// <summary>
    /// Deconstructs the Vector4d into its four int components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out int x, out int y, out int z, out int w)
    {
        x = X.RoundToInt();
        y = Y.RoundToInt();
        z = Z.RoundToInt();
        w = W.RoundToInt();
    }

    /// <summary>
    /// Deconstructs the Vector4d into its four long components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out long x, out long y, out long z, out long w)
    {
        x = X.m_rawValue;
        y = Y.m_rawValue;
        z = Z.m_rawValue;
        w = W.m_rawValue;
    }

    /// <summary>
    /// Deconstructs the Vector4d into its four double components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out double x, out double y, out double z, out double w)
    {
        x = (double)X;
        y = (double)Y;
        z = (double)Z;
        w = (double)W;
    }

    #endregion

    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly bool Equals(object? obj) => obj is Vector4d other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Equals(Vector4d other) => other.X == X && other.Y == Y && other.Z == Z && other.W == W;

    /// <inheritdoc/>
    public readonly bool Equals(Vector4d x, Vector4d y) => x.Equals(y);

    /// <summary>
    /// Returns a hash code for this vector based on its state.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly int GetHashCode() => StateHash;

    /// <inheritdoc/>
    public readonly int GetHashCode(Vector4d obj) => obj.GetHashCode();

    /// <summary>
    /// Compares the current Vector4d instance with another Vector4d based on squared magnitude.
    /// </summary>
    public readonly int CompareTo(Vector4d other) => MagnitudeSquared.CompareTo(other.MagnitudeSquared);

    #endregion
}
