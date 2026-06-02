//=======================================================================
// Vector4d.cs
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
/// Represents a 4D vector with fixed-point precision.
/// </summary>
/// <remarks>
/// This type is useful for generic 4D component math and homogeneous coordinates used by 4x4 transforms.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Vector4d : IEquatable<Vector4d>, IComparable<Vector4d>, IEqualityComparer<Vector4d>
{
    #region Static Readonly Fields

    /// <summary>
    /// (1, 0, 0, 0)
    /// </summary>
    public static readonly Vector4d UnitX = new(1, 0, 0, 0);

    /// <summary>
    /// (0, 1, 0, 0)
    /// </summary>
    public static readonly Vector4d UnitY = new(0, 1, 0, 0);

    /// <summary>
    /// (0, 0, 1, 0)
    /// </summary>
    public static readonly Vector4d UnitZ = new(0, 0, 1, 0);

    /// <summary>
    /// (0, 0, 0, 1)
    /// </summary>
    public static readonly Vector4d UnitW = new(0, 0, 0, 1);

    /// <summary>
    /// (1, 1, 1, 1)
    /// </summary>
    public static readonly Vector4d One = new(1, 1, 1, 1);

    /// <summary>
    /// (-1, -1, -1, -1)
    /// </summary>
    public static readonly Vector4d Negative = new(-1, -1, -1, -1);

    /// <summary>
    /// (0, 0, 0, 0)
    /// </summary>
    public static readonly Vector4d Zero = new(0, 0, 0, 0);

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

    /// <summary>
    /// The Z component of the vector.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(2)]
    public Fixed64 z;

    /// <summary>
    /// The W component of the vector.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(3)]
    public Fixed64 w;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new Vector4d using integer component values.
    /// </summary>
    public Vector4d(int xInt, int yInt, int zInt, int wInt)
        : this((Fixed64)xInt, (Fixed64)yInt, (Fixed64)zInt, (Fixed64)wInt) { }

    /// <summary>
    /// Initializes a new Vector4d using double component values.
    /// </summary>
    public Vector4d(double xDoub, double yDoub, double zDoub, double wDoub)
        : this((Fixed64)xDoub, (Fixed64)yDoub, (Fixed64)zDoub, (Fixed64)wDoub) { }

    /// <summary>
    /// Initializes a new Vector4d using the specified components.
    /// </summary>
    [JsonConstructor]
    public Vector4d(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 w)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
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
        : this(value.x, value.y, z, w) { }

    /// <summary>
    /// Initializes a new Vector4d from a Vector3d plus a W component.
    /// </summary>
    public Vector4d(Vector3d value, Fixed64 w)
        : this(value.x, value.y, value.z, w) { }

    #endregion

    #region Properties

    /// <inheritdoc cref="GetNormalized(Vector4d)"/>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector4d Normal
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
    public readonly Fixed64 SqrMagnitude
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (x * x) + (y * y) + (z * z) + (w * w);
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
                return (x.m_rawValue * 31) + (y.m_rawValue * 7) + (z.m_rawValue * 11) + (w.m_rawValue * 17);
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
                0 => x,
                1 => y,
                2 => z,
                3 => w,
                _ => throw new IndexOutOfRangeException("Invalid Vector4d index!"),
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
        x = newX;
        y = newY;
        z = newZ;
        w = newW;
        return this;
    }

    /// <summary>
    /// Adds the specified scalar to all components in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d AddInPlace(Fixed64 amount)
    {
        x += amount;
        y += amount;
        z += amount;
        w += amount;
        return this;
    }

    /// <summary>
    /// Adds the specified component values in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d AddInPlace(Fixed64 xAmount, Fixed64 yAmount, Fixed64 zAmount, Fixed64 wAmount)
    {
        x += xAmount;
        y += yAmount;
        z += zAmount;
        w += wAmount;
        return this;
    }

    /// <summary>
    /// Adds another vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d AddInPlace(Vector4d other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return this;
    }

    /// <summary>
    /// Subtracts the specified scalar from all components in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d SubtractInPlace(Fixed64 amount)
    {
        x -= amount;
        y -= amount;
        z -= amount;
        w -= amount;
        return this;
    }

    /// <summary>
    /// Subtracts the specified component values in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d SubtractInPlace(Fixed64 xAmount, Fixed64 yAmount, Fixed64 zAmount, Fixed64 wAmount)
    {
        x -= xAmount;
        y -= yAmount;
        z -= zAmount;
        w -= wAmount;
        return this;
    }

    /// <summary>
    /// Subtracts another vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d SubtractInPlace(Vector4d other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        w -= other.w;
        return this;
    }

    /// <summary>
    /// Scales all components in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d ScaleInPlace(Fixed64 scaleFactor)
    {
        x *= scaleFactor;
        y *= scaleFactor;
        z *= scaleFactor;
        w *= scaleFactor;
        return this;
    }

    /// <summary>
    /// Scales all components by another vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d ScaleInPlace(Vector4d scale)
    {
        x *= scale.x;
        y *= scale.y;
        z *= scale.z;
        w *= scale.w;
        return this;
    }

    /// <summary>
    /// Normalizes this vector in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d Normalize()
    {
        return this = GetNormalized(this);
    }

    /// <summary>
    /// Normalizes this vector in place and returns its original magnitude.
    /// </summary>
    public Vector4d Normalize(out Fixed64 mag)
    {
        mag = GetMagnitude(this);

        if (mag == Fixed64.Zero)
            return this = Zero;

        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return this;

        return this = new Vector4d(x / mag, y / mag, z / mag, w / mag);
    }

    /// <summary>
    /// Determines whether this vector is normalized.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool IsNormalized()
    {
        return !IsZero && FixedMath.Abs(SqrMagnitude - Fixed64.One) <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Determines whether all components are greater than Fixed64.Epsilon.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool AllComponentsGreaterThanEpsilon()
    {
        return x > Fixed64.Epsilon && y > Fixed64.Epsilon && z > Fixed64.Epsilon && w > Fixed64.Epsilon;
    }

    /// <summary>
    /// Snaps components with absolute values below the threshold to zero.
    /// </summary>
    public readonly Vector4d SnapSmallComponentsToZero(Fixed64? threshold = null)
    {
        Fixed64 t = threshold ?? Fixed64.Epsilon;
        return new Vector4d(
            FixedMath.Abs(x) < t ? Fixed64.Zero : x,
            FixedMath.Abs(y) < t ? Fixed64.Zero : y,
            FixedMath.Abs(z) < t ? Fixed64.Zero : z,
            FixedMath.Abs(w) < t ? Fixed64.Zero : w);
    }

    /// <summary>
    /// Calculates the distance to another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Distance(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ, Fixed64 otherW)
    {
        return FixedMath.Sqrt(SqrDistance(otherX, otherY, otherZ, otherW));
    }

    /// <summary>
    /// Calculates the distance to another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Distance(Vector4d other)
    {
        return Distance(other.x, other.y, other.z, other.w);
    }

    /// <summary>
    /// Calculates the squared distance to another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 SqrDistance(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ, Fixed64 otherW)
    {
        Fixed64 dx = otherX - x;
        Fixed64 dy = otherY - y;
        Fixed64 dz = otherZ - z;
        Fixed64 dw = otherW - w;
        return (dx * dx) + (dy * dy) + (dz * dz) + (dw * dw);
    }

    /// <summary>
    /// Calculates the squared distance to another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 SqrDistance(Vector4d other)
    {
        return SqrDistance(other.x, other.y, other.z, other.w);
    }

    /// <summary>
    /// Calculates the dot product with another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Dot(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ, Fixed64 otherW)
    {
        return (x * otherX) + (y * otherY) + (z * otherZ) + (w * otherW);
    }

    /// <summary>
    /// Calculates the dot product with another vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Dot(Vector4d other)
    {
        return Dot(other.x, other.y, other.z, other.w);
    }

    /// <summary>
    /// Converts this vector to Vector3d by dropping the W component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3d ToVector3d()
    {
        return new Vector3d(x, y, z);
    }

    #endregion

    #region Static Methods

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
    public static Vector4d UnclampedLerp(Vector4d a, Vector4d b, Fixed64 t)
    {
        return new Vector4d(
            a.x + ((b.x - a.x) * t),
            a.y + ((b.y - a.y) * t),
            a.z + ((b.z - a.z) * t),
            a.w + ((b.w - a.w) * t));
    }

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

        return new Vector4d(value.x / mag, value.y / mag, value.z / mag, value.w / mag);
    }

    /// <summary>
    /// Returns the magnitude of the given vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetMagnitude(Vector4d vector)
    {
        Fixed64 mag = (vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z) + (vector.w * vector.w);

        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return Fixed64.One;

        return mag != Fixed64.Zero ? FixedMath.Sqrt(mag) : Fixed64.Zero;
    }

    /// <summary>
    /// Returns a new vector containing the absolute value of each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Abs(Vector4d value)
    {
        return new Vector4d(FixedMath.Abs(value.x), FixedMath.Abs(value.y), FixedMath.Abs(value.z), FixedMath.Abs(value.w));
    }

    /// <summary>
    /// Returns a new vector containing the sign of each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Sign(Vector4d value)
    {
        return new Vector4d(value.x.Sign(), value.y.Sign(), value.z.Sign(), value.w.Sign());
    }

    /// <summary>
    /// Clamps each component of the given vector within the specified min and max bounds.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Clamp(Vector4d value, Vector4d min, Vector4d max)
    {
        return new Vector4d(
            FixedMath.Clamp(value.x, min.x, max.x),
            FixedMath.Clamp(value.y, min.y, max.y),
            FixedMath.Clamp(value.z, min.z, max.z),
            FixedMath.Clamp(value.w, min.w, max.w));
    }

    /// <summary>
    /// Computes the midpoint between two vectors.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Midpoint(Vector4d v1, Vector4d v2)
    {
        return new Vector4d(
            (v1.x + v2.x) * Fixed64.Half,
            (v1.y + v2.y) * Fixed64.Half,
            (v1.z + v2.z) * Fixed64.Half,
            (v1.w + v2.w) * Fixed64.Half);
    }

    /// <inheritdoc cref="Distance(Fixed64, Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector4d start, Vector4d end)
    {
        return start.Distance(end.x, end.y, end.z, end.w);
    }

    /// <inheritdoc cref="SqrDistance(Fixed64, Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 SqrDistance(Vector4d start, Vector4d end)
    {
        return start.SqrDistance(end.x, end.y, end.z, end.w);
    }

    /// <summary>
    /// Calculates the dot product of two vectors.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(Vector4d lhs, Vector4d rhs)
    {
        return lhs.Dot(rhs.x, rhs.y, rhs.z, rhs.w);
    }

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Scale(Vector4d a, Vector4d b)
    {
        return new Vector4d(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
    }

    /// <summary>
    /// Returns a vector whose elements are the maximum of each pair of elements.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Max(Vector4d value1, Vector4d value2)
    {
        return new Vector4d(
            FixedMath.Max(value1.x, value2.x),
            FixedMath.Max(value1.y, value2.y),
            FixedMath.Max(value1.z, value2.z),
            FixedMath.Max(value1.w, value2.w));
    }

    /// <summary>
    /// Returns a vector whose elements are the minimum of each pair of elements.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Min(Vector4d value1, Vector4d value2)
    {
        return new Vector4d(
            FixedMath.Min(value1.x, value2.x),
            FixedMath.Min(value1.y, value2.y),
            FixedMath.Min(value1.z, value2.z),
            FixedMath.Min(value1.w, value2.w));
    }

    /// <summary>
    /// Transforms a 4D vector by a 4x4 matrix.
    /// </summary>
    /// <remarks>
    /// This follows the same storage convention as <see cref="Fixed4x4.TransformPoint(Fixed4x4, Vector3d)"/>,
    /// preserving the computed W component instead of performing perspective division.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Transform(Fixed4x4 matrix, Vector4d vector) =>
        new(matrix.M11 * vector.x + matrix.M12 * vector.y + matrix.M13 * vector.z + matrix.M41 * vector.w,
            matrix.M21 * vector.x + matrix.M22 * vector.y + matrix.M23 * vector.z + matrix.M42 * vector.w,
            matrix.M31 * vector.x + matrix.M32 * vector.y + matrix.M33 * vector.z + matrix.M43 * vector.w,
            matrix.M14 * vector.x + matrix.M24 * vector.y + matrix.M34 * vector.z + matrix.M44 * vector.w);

    #endregion

    #region Operators

    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, Vector4d v2)
    {
        return new Vector4d(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w);
    }

    /// <summary>
    /// Adds a scalar to each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, Fixed64 mag)
    {
        return new Vector4d(v1.x + mag, v1.y + mag, v1.z + mag, v1.w + mag);
    }

    /// <inheritdoc cref="operator +(Vector4d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Fixed64 mag, Vector4d v1)
    {
        return v1 + mag;
    }

    /// <summary>
    /// Adds a tuple to each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, (int x, int y, int z, int w) v2)
    {
        return new Vector4d(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w);
    }

    /// <summary>
    /// Adds a tuple to each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +((int x, int y, int z, int w) v2, Vector4d v1)
    {
        return v1 + v2;
    }

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, Vector4d v2)
    {
        return new Vector4d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, v1.w - v2.w);
    }

    /// <summary>
    /// Subtracts a scalar from each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, Fixed64 mag)
    {
        return new Vector4d(v1.x - mag, v1.y - mag, v1.z - mag, v1.w - mag);
    }

    /// <inheritdoc cref="operator -(Vector4d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Fixed64 mag, Vector4d v1)
    {
        return new Vector4d(mag - v1.x, mag - v1.y, mag - v1.z, mag - v1.w);
    }

    /// <summary>
    /// Subtracts a tuple from each component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, (int x, int y, int z, int w) v2)
    {
        return new Vector4d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, v1.w - v2.w);
    }

    /// <summary>
    /// Subtracts each vector component from a tuple component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -((int x, int y, int z, int w) v1, Vector4d v2)
    {
        return new Vector4d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, v1.w - v2.w);
    }

    /// <summary>
    /// Negates the vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1)
    {
        return new Vector4d(-v1.x, -v1.y, -v1.z, -v1.w);
    }

    /// <summary>
    /// Multiplies each component by a scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, Fixed64 mag)
    {
        return new Vector4d(v1.x * mag, v1.y * mag, v1.z * mag, v1.w * mag);
    }

    /// <inheritdoc cref="operator *(Vector4d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Fixed64 mag, Vector4d v1)
    {
        return v1 * mag;
    }

    /// <summary>
    /// Multiplies each component by an integer scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, int mag)
    {
        return v1 * (Fixed64)mag;
    }

    /// <inheritdoc cref="operator *(Vector4d, int)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(int mag, Vector4d v1)
    {
        return v1 * mag;
    }

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, Vector4d v2)
    {
        return Scale(v1, v2);
    }

    /// <summary>
    /// Transforms a vector by a 4x4 matrix.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Fixed4x4 matrix, Vector4d vector)
    {
        return Transform(matrix, vector);
    }

    /// <inheritdoc cref="operator *(Fixed4x4, Vector4d)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d vector, Fixed4x4 matrix)
    {
        return matrix * vector;
    }

    /// <summary>
    /// Divides each component by a scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, Fixed64 div)
    {
        return new Vector4d(v1.x / div, v1.y / div, v1.z / div, v1.w / div);
    }

    /// <summary>
    /// Divides each component by another vector's corresponding component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, Vector4d v2)
    {
        return new Vector4d(v1.x / v2.x, v1.y / v2.y, v1.z / v2.z, v1.w / v2.w);
    }

    /// <summary>
    /// Divides each component by an integer scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, int div)
    {
        return v1 / (Fixed64)div;
    }

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
    public static bool operator >(Vector4d left, Vector4d right)
    {
        return left.x > right.x && left.y > right.y && left.z > right.z && left.w > right.w;
    }

    /// <summary>
    /// Returns true when every component in the left vector is less than the corresponding component in the right vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Vector4d left, Vector4d right)
    {
        return left.x < right.x && left.y < right.y && left.z < right.z && left.w < right.w;
    }

    /// <summary>
    /// Returns true when every component in the left vector is greater than or equal to the corresponding component in the right vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Vector4d left, Vector4d right)
    {
        return left.x >= right.x && left.y >= right.y && left.z >= right.z && left.w >= right.w;
    }

    /// <summary>
    /// Returns true when every component in the left vector is less than or equal to the corresponding component in the right vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Vector4d left, Vector4d right)
    {
        return left.x <= right.x && left.y <= right.y && left.z <= right.z && left.w <= right.w;
    }

    #endregion

    #region Conversion and Formatting

    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly string ToString()
    {
        return string.Format(
            "({0}, {1}, {2}, {3})",
            x.ToFormattedDouble(),
            y.ToFormattedDouble(),
            z.ToFormattedDouble(),
            w.ToFormattedDouble());
    }

    /// <summary>
    /// Deconstructs the vector into single-precision floating-point values.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void Deconstruct(out double x, out double y, out double z, out double w)
    {
        x = (double)this.x;
        y = (double)this.y;
        z = (double)this.z;
        w = (double)this.w;
    }

    /// <summary>
    /// Deconstructs the vector into rounded integer values.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void Deconstruct(out int x, out int y, out int z, out int w)
    {
        x = this.x.RoundToInt();
        y = this.y.RoundToInt();
        z = this.z.RoundToInt();
        w = this.w.RoundToInt();
    }

    /// <summary>
    /// Deconstructs the vector into its raw long component values.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void Deconstruct(out long x, out long y, out long z, out long w)
    {
        x = this.x.m_rawValue;
        y = this.y.m_rawValue;
        z = this.z.m_rawValue;
        w = this.w.m_rawValue;
    }

    #endregion

    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly bool Equals(object? obj) => obj is Vector4d other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Equals(Vector4d other)
    {
        return other.x == x && other.y == y && other.z == z && other.w == w;
    }

    /// <inheritdoc/>
    public readonly bool Equals(Vector4d x, Vector4d y) => x.Equals(y);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly int GetHashCode() => StateHash;

    /// <inheritdoc/>
    public readonly int GetHashCode(Vector4d obj) => obj.GetHashCode();

    /// <summary>
    /// Compares the current Vector4d instance with another Vector4d based on squared magnitude.
    /// </summary>
    public readonly int CompareTo(Vector4d other) => SqrMagnitude.CompareTo(other.SqrMagnitude);

    #endregion
}
