//=======================================================================
// FixedQuaternion.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a quaternion (x, y, z, w) with fixed-point numbers.
/// Quaternions are useful for representing rotations and can be used to perform smooth rotations and avoid gimbal lock.
/// </summary>
/// <remarks>
/// Direction-oriented quaternion APIs use FixedMathSharp's canonical 3D convention: <c>+X</c>
/// right, <c>+Y</c> up, and <c>+Z</c> forward. Convert external engine or tool directions
/// before calling these APIs when their semantic basis differs.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct FixedQuaternion : IEquatable<FixedQuaternion>, IFormattable
#if NET8_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly Fields

    /// <summary>
    /// Identity quaternion (0, 0, 0, 1).
    /// </summary>
    public static FixedQuaternion Identity => new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

    /// <summary>
    /// Empty quaternion (0, 0, 0, 0).
    /// </summary>
    public static FixedQuaternion Zero => new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

    #endregion
    #region Fields and Constants

    private const long NearOppositeDirectionDotRaw = -4290672328L;

    /// <summary>
    /// Represents the X component of the vector as a fixed-point value.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Fixed64 X;

    /// <summary>
    /// Represents the Y component of the vector as a fixed-point value.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Fixed64 Y;

    /// <summary>
    /// Represents the Z component of the vector as a fixed-point value.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(2)]
    public Fixed64 Z;

    /// <summary>
    /// Represents the W component of the vector as a fixed-point value.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(3)]
    public Fixed64 W;

    #endregion
    #region Constructors

    /// <summary>
    /// Creates a new FixedQuaternion with the specified components.
    /// </summary>
    public FixedQuaternion(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 w)
    {
        X = x;
        Y = y;
        Z = z;
        W = w;
    }

    #endregion
    #region Properties

    /// <summary>
    /// Normalized version of this quaternion.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedQuaternion Normalized
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetNormalized(this);
    }

    /// <summary>
    /// Gets the magnitude of this quaternion.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Magnitude
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetMagnitude(this);
    }

    /// <summary>
    /// Gets the squared magnitude of this quaternion.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MagnitudeSquared
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => X * X + Y * Y + Z * Z + W * W;
    }

    /// <summary>
    /// Returns the Euler angles (in degrees) of this quaternion.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d EulerAngles
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ToEulerAngles();
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => this = FromEulerAnglesInDegrees(value.X, value.Y, value.Z);
    }

    /// <summary>
    /// Gets or sets the component value at the specified index.
    /// </summary>
    /// <remarks>Index 0 corresponds to the x component, 1 to y, 2 to z, and 3 to w.</remarks>
    /// <param name="index">The zero-based index of the component to access. Valid values are 0 (x), 1 (y), 2 (z), and 3 (w).</param>
    /// <returns>The value of the component at the specified index.</returns>
    /// <exception cref="IndexOutOfRangeException">Thrown when the specified index is less than 0 or greater than 3.</exception>
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
                3 => W,
                _ => throw new IndexOutOfRangeException("Invalid FixedQuaternion index!"),
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
                    throw new IndexOutOfRangeException("Invalid FixedQuaternion index!");
            }
        }
    }

    #endregion
    #region Methods (Instance)

    /// <summary>
    /// Set x, y, z and w components of an existing Quaternion.
    /// </summary>
    /// <param name="newX"></param>
    /// <param name="newY"></param>
    /// <param name="newZ"></param>
    /// <param name="newW"></param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Set(Fixed64 newX, Fixed64 newY, Fixed64 newZ, Fixed64 newW)
    {
        X = newX;
        Y = newY;
        Z = newZ;
        W = newW;
    }

    /// <summary>
    /// Normalizes this quaternion in place.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedQuaternion NormalizeInPlace() => this = GetNormalized(this);

    /// <summary>
    /// Returns the conjugate of this quaternion (inverses the rotational effect).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedQuaternion Conjugate() => new(-X, -Y, -Z, W);

    /// <summary>
    /// Returns the inverse of this quaternion.
    /// </summary>
    public FixedQuaternion Inverse()
    {
        if (this == Identity) return Identity;
        Fixed64 norm = MagnitudeSquared;
        if (norm == Fixed64.Zero) return this; // Handle division by zero by returning the same quaternion

        Fixed64 invNorm = Fixed64.One / norm;
        return new FixedQuaternion(X * -invNorm, Y * -invNorm, Z * -invNorm, W * invNorm);
    }

    /// <summary>
    /// Rotates a vector by this quaternion.
    /// </summary>
    public Vector3d Rotate(Vector3d v)
    {
        FixedQuaternion normalizedQuat = Normalized;
        FixedQuaternion vQuat = new(v.X, v.Y, v.Z, Fixed64.Zero);
        FixedQuaternion invQuat = normalizedQuat.Conjugate();
        FixedQuaternion rotatedVQuat = (normalizedQuat * vQuat) * invQuat;
        return new Vector3d(rotatedVQuat.X, rotatedVQuat.Y, rotatedVQuat.Z);
    }

    /// <summary>
    /// Rotates this quaternion by a given angle around a specified axis (default: Y-axis).
    /// </summary>
    /// <param name="sin">Sine of the rotation angle.</param>
    /// <param name="cos">Cosine of the rotation angle.</param>
    /// <param name="axis">The axis to rotate around (default: Vector3d.Up).</param>
    /// <returns>A new quaternion representing the rotated result.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedQuaternion Rotated(Fixed64 sin, Fixed64 cos, Vector3d? axis = null)
    {
        Vector3d rotateAxis = axis ?? Vector3d.Up;

        // The rotation angle is the arc tangent of sin and cos
        Fixed64 angle = FixedMath.Atan2(sin, cos);

        // Construct a quaternion representing a rotation around the axis (default is y aka Vector3d.up)
        FixedQuaternion rotationQuat = FromAxisAngle(rotateAxis, angle);

        // Apply the rotation and return the result
        return rotationQuat * this;
    }

    #endregion
}
