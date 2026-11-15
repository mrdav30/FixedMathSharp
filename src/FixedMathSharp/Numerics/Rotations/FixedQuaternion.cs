//=======================================================================
// FixedQuaternion.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using FixedMathSharp.Geometry;
using MemoryPack;

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
#if NET10_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly Fields

    /// <summary>
    /// Identity quaternion (0, 0, 0, 1).
    /// </summary>
    /// <value>The identity rotation <c>(0, 0, 0, 1)</c>.</value>
    public static FixedQuaternion Identity => new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

    /// <summary>
    /// Empty quaternion (0, 0, 0, 0).
    /// </summary>
    /// <value>The zero payload <c>(0, 0, 0, 0)</c>. Individual rotation operations define how they handle this degenerate value.</value>
    public static FixedQuaternion Zero => new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

    #endregion
    #region Fields and Constants

    private const long NearOppositeDirectionDotRaw = -4290672328L;

    // Approximately 9.536743e-7 in Q32.32.
    private const long QuaternionLogVectorThresholdRaw = 4_096L;

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
    /// <param name="x">The X component.</param>
    /// <param name="y">The Y component.</param>
    /// <param name="z">The Z component.</param>
    /// <param name="w">The W component. The W component is the scalar part.</param>
    /// <remarks>Components are stored as supplied. The constructor does not normalize them.</remarks>
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
    /// <value>An approximately unit quaternion, or <see cref="Identity"/> for zero. A value already within the normalization tolerance is returned as supplied.</value>
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
    /// <value>The 4D Euclidean length, rounded once and saturated to <see cref="Fixed64.MaxValue"/> if needed.</value>
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
    /// <value>The sum of the individually rounded squares, saturated to <see cref="Fixed64.MaxValue"/> if needed.</value>
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
    /// <value>Pitch, yaw, and roll in degrees in X, Y, and Z order. Setting it replaces this quaternion with the corresponding normalized YXZ composition.</value>
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
    /// <exception cref="IndexOutOfRangeException">Thrown when the specified index is less than 0 or greater than 3.</exception>
    /// <value>The selected component. Setting it replaces only that component.</value>
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
    /// <param name="newX">The replacement X component.</param>
    /// <param name="newY">The replacement Y component.</param>
    /// <param name="newZ">The replacement Z component.</param>
    /// <param name="newW">The replacement W component.</param>
    /// <remarks>Components are assigned as supplied. The method does not normalize them.</remarks>
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
    /// <returns>A copy of the normalized quaternion. A zero input becomes <see cref="Identity"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedQuaternion NormalizeInPlace() => this = GetNormalized(this);

    /// <summary>Computes the quaternion conjugate.</summary>
    /// <returns>A copy with the vector part negated and W preserved.</returns>
    /// <remarks>For a unit quaternion, the conjugate represents the inverse rotation.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedQuaternion Conjugate() => new(-X, -Y, -Z, W);

    /// <summary>
    /// Returns the inverse of this quaternion.
    /// </summary>
    /// <returns>The conjugate scaled by the reciprocal of the computed squared magnitude. When that magnitude rounds to zero, returns the input.</returns>
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
    /// <param name="v">The vector expressed in the input frame.</param>
    /// <returns>The rotated vector using the normalized quaternion and fixed-point quaternion products. A zero quaternion returns the input vector.</returns>
    public Vector3d Rotate(Vector3d v)
    {
        FixedQuaternion normalizedQuat = Normalized;
        FixedQuaternion vQuat = new(v.X, v.Y, v.Z, Fixed64.Zero);
        FixedQuaternion invQuat = normalizedQuat.Conjugate();
        FixedQuaternion rotatedVQuat = (normalizedQuat * vQuat) * invQuat;
        return new Vector3d(rotatedVQuat.X, rotatedVQuat.Y, rotatedVQuat.Z);
    }

    /// <summary>
    /// Attempts to rotate a vector using the quaternion's exact
    /// scale-invariant rational basis.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> when every final component is representable;
    /// otherwise, <see langword="false"/> and <paramref name="result"/> is
    /// <see langword="default"/>.
    /// </returns>
    /// <param name="vector">The vector expressed in the input frame.</param>
    /// <param name="result">The rounded rotated vector on success, or the zero vector on failure.</param>
    /// <remarks>A zero quaternion succeeds with the zero vector. Each component is rounded once after the complete linear combination.</remarks>
    public bool TryRotate(Vector3d vector, out Vector3d result)
    {
        if (this == Zero)
        {
            result = Vector3d.Zero;
            return true;
        }

        return WideOrientedBox.TryTransformLocalOffset(
            this,
            vector,
            out result);
    }

    /// <summary>
    /// Attempts to transform a local point by this rotation and a world origin
    /// with one final round-half-to-even conversion per component.
    /// </summary>
    /// <remarks>
    /// The zero quaternion preserves the legacy rotation contract and returns
    /// <paramref name="origin"/>.
    /// </remarks>
    /// <param name="origin">The world-space origin of the local frame.</param>
    /// <param name="localPoint">The point expressed in the local frame.</param>
    /// <param name="result">The world-space point on success, or the zero vector on failure.</param>
    /// <returns><see langword="true"/> if every final world coordinate is representable, otherwise <see langword="false"/>.</returns>
    public bool TryTransformPoint(
        Vector3d origin,
        Vector3d localPoint,
        out Vector3d result)
    {
        if (this == Zero)
        {
            result = origin;
            return true;
        }
        if (this == Identity)
            return Vector3d.TryAdd(origin, localPoint, out result);

        return WideOrientedBox.TryMaterializeLocalPoint(
            origin,
            this,
            localPoint,
            out result);
    }

    /// <summary>
    /// Attempts to transform a local point by this rotation and add two world
    /// origins with one final round-half-to-even conversion per component.
    /// </summary>
    /// <remarks>
    /// The zero quaternion preserves the legacy rotation contract and returns
    /// the exact sum of the two origins.
    /// </remarks>
    /// <param name="firstOrigin">The first translation in world coordinates.</param>
    /// <param name="secondOrigin">The second translation in world coordinates.</param>
    /// <param name="localPoint">The point expressed in the local frame.</param>
    /// <param name="result">The world-space point on success, or the zero vector on failure.</param>
    /// <returns><see langword="true"/> if every final world coordinate is representable, otherwise <see langword="false"/>.</returns>
    public bool TryTransformPoint(
        Vector3d firstOrigin,
        Vector3d secondOrigin,
        Vector3d localPoint,
        out Vector3d result)
    {
        if (this == Zero)
        {
            return Vector3d.TrySubtractSums(
                firstOrigin,
                secondOrigin,
                Vector3d.Zero,
                Vector3d.Zero,
                out result);
        }

        return WideOrientedBox.TryMaterializeLocalPoint(
            firstOrigin,
            secondOrigin,
            this,
            localPoint,
            out result);
    }

    /// <summary>
    /// Attempts to obtain the exact relative offset
    /// <c>firstOrigin + firstOffset - secondOrigin - Rotate(secondLocalPoint)</c>.
    /// </summary>
    /// <param name="firstOrigin">The first frame’s world-space origin.</param>
    /// <param name="firstOffset">The first point offset in world-space axes.</param>
    /// <param name="secondOrigin">The second frame’s world-space origin.</param>
    /// <param name="secondLocalPoint">The second point offset before applying this rotation.</param>
    /// <param name="result">The rounded difference between the two world-space points on success, or the zero vector on failure.</param>
    /// <returns><see langword="true"/> if every final component is representable, otherwise <see langword="false"/>.</returns>
    /// <remarks>A zero quaternion treats the rotated second point as zero. Intermediates remain wide until the final difference is rounded.</remarks>
    public bool TryGetRelativeOffset(
        Vector3d firstOrigin,
        Vector3d firstOffset,
        Vector3d secondOrigin,
        Vector3d secondLocalPoint,
        out Vector3d result)
    {
        if (this == Zero)
        {
            return Vector3d.TrySubtractSums(
                firstOrigin,
                firstOffset,
                secondOrigin,
                Vector3d.Zero,
                out result);
        }
        if (this == Identity)
        {
            return Vector3d.TrySubtractSums(
                firstOrigin,
                firstOffset,
                secondOrigin,
                secondLocalPoint,
                out result);
        }

        return WideOrientedBox.TryGetRelativeOffset(
            this,
            firstOrigin,
            firstOffset,
            secondOrigin,
            secondLocalPoint,
            out result);
    }

    /// <summary>
    /// Rotates this quaternion by a given angle around a specified axis (default: Y-axis).
    /// </summary>
    /// <param name="sin">The sine of the angle.</param>
    /// <param name="cos">The cosine of the same angle.</param>
    /// <param name="axis">The rotation axis. Null selects <see cref="Vector3d.Up"/>.</param>
    /// <returns>The additional rotation multiplied on the left of this quaternion.</returns>
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
