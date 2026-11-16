//=======================================================================
// Vector4d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp;

/// <summary>
/// Represents a 4D vector with fixed-point precision.
/// </summary>
/// <remarks>
/// This type is useful for generic 4D component math and homogeneous coordinates used by 4x4
/// transforms. It does not define independent forward or backward semantics; direction naming
/// belongs to <see cref="Vector2d"/>, <see cref="Vector3d"/>, and adapter-level convention code.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Vector4d : IEquatable<Vector4d>, IComparable<Vector4d>, IEqualityComparer<Vector4d>, IFormattable
#if NET10_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly Fields

    /// <summary>
    /// (1, 0, 0, 0)
    /// </summary>
    /// <value>The positive X unit vector.</value>
    public static Vector4d UnitX => new(1, 0, 0, 0);

    /// <summary>
    /// (0, 1, 0, 0)
    /// </summary>
    /// <value>The positive Y unit vector.</value>
    public static Vector4d UnitY => new(0, 1, 0, 0);

    /// <summary>
    /// The unit Z component vector (0, 0, 1, 0).
    /// </summary>
    /// <value>The positive Z unit vector.</value>
    public static Vector4d UnitZ => new(0, 0, 1, 0);

    /// <summary>
    /// (0, 0, 0, 1)
    /// </summary>
    /// <value>The positive W unit vector.</value>
    public static Vector4d UnitW => new(0, 0, 0, 1);

    /// <summary>
    /// (1, 1, 1, 1)
    /// </summary>
    /// <value>All components equal to one.</value>
    public static Vector4d One => new(1, 1, 1, 1);

    /// <summary>
    /// (-1, -1, -1, -1)
    /// </summary>
    /// <value>All components equal to −1.</value>
    public static Vector4d Negative => new(-1, -1, -1, -1);

    /// <summary>
    /// (0, 0, 0, 0)
    /// </summary>
    /// <value>All components equal to zero.</value>
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
    /// <param name="xInt">The signed integer X component.</param>
    /// <param name="yInt">The signed integer Y component.</param>
    /// <param name="zInt">The signed integer Z component.</param>
    /// <param name="wInt">The signed integer W component.</param>
    public Vector4d(int xInt, int yInt, int zInt, int wInt)
        : this((Fixed64)xInt, (Fixed64)yInt, (Fixed64)zInt, (Fixed64)wInt) { }

    /// <summary>
    /// Initializes a new Vector4d using the specified components.
    /// </summary>
    /// <param name="x">The X component.</param>
    /// <param name="y">The Y component.</param>
    /// <param name="z">The Z component.</param>
    /// <param name="w">The W component.</param>
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
    /// <param name="value">The value assigned to all four components.</param>
    public Vector4d(Fixed64 value)
        : this(value, value, value, value) { }

    /// <summary>
    /// Initializes a new Vector4d from a Vector2d plus Z and W components.
    /// </summary>
    /// <param name="value">The existing X and Y components.</param>
    /// <param name="z">The Z component.</param>
    /// <param name="w">The W component.</param>
    public Vector4d(Vector2d value, Fixed64 z, Fixed64 w)
        : this(value.X, value.Y, z, w) { }

    /// <summary>
    /// Initializes a new Vector4d from a Vector3d plus a W component.
    /// </summary>
    /// <param name="value">The existing X, Y, and Z components.</param>
    /// <param name="w">The W component.</param>
    public Vector4d(Vector3d value, Fixed64 w)
        : this(value.X, value.Y, value.Z, w) { }

    /// <summary>Creates a vector from double-precision components.</summary>
    /// <param name="xDoub">The X component.</param>
    /// <param name="yDoub">The Y component.</param>
    /// <param name="zDoub">The Z component.</param>
    /// <param name="wDoub">The W component.</param>
    /// <returns>A vector whose components are rounded to the nearest Q32.32 value with ties to even.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A component is NaN or infinity.</exception>
    /// <exception cref="OverflowException">A rounded component lies outside the Q32.32 range.</exception>
    public static Vector4d FromDouble(double xDoub, double yDoub, double zDoub, double wDoub) =>
        new(Fixed64.FromDouble(xDoub), Fixed64.FromDouble(yDoub), Fixed64.FromDouble(zDoub), Fixed64.FromDouble(wDoub));

    #endregion

    #region Properties

    /// <summary>Gets a normalized copy of the vector.</summary>
    /// <value>The approximate unit vector in the same direction, or zero for a zero vector.</value>
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
    /// <value>The Euclidean length in input units, saturated to <see cref="Fixed64.MaxValue"/> when too large. Values within the normalization tolerance report one.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Fixed64 Magnitude
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetMagnitude(this);
    }

    /// <summary>Gets the squared length of the vector.</summary>
    /// <value>The rounded sum of the squared components, in squared input units and saturated at <see cref="Fixed64.MaxValue"/>.</value>
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
    /// <value>A deterministic weighted sum of the raw components using wrapping 64-bit arithmetic.</value>
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
    /// <value>The signed remainder of <see cref="LongStateHash"/> modulo <see cref="int.MaxValue"/>.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly int StateHash
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => unchecked((int)(LongStateHash % int.MaxValue));
    }

    /// <summary>Tests whether all components equal zero.</summary>
    /// <value><see langword="true"/> if all components are exactly zero, otherwise <see langword="false"/>.</value>
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
    /// <param name="index">The zero-based component index in X, Y, Z, W order.</param>
    /// <value>The selected component. Setting it replaces only that component.</value>
    /// <exception cref="IndexOutOfRangeException"><paramref name="index"/> is outside the interval [0, 3].</exception>
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
    /// <param name="newX">The new X component.</param>
    /// <param name="newY">The new Y component.</param>
    /// <param name="newZ">The new Z component.</param>
    /// <param name="newW">The new W component.</param>
    /// <returns>A copy of this vector after all components have been assigned.</returns>
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
    /// <param name="xAmount">The amount to add for the X component.</param>
    /// <param name="yAmount">The amount to add for the Y component.</param>
    /// <param name="zAmount">The amount to add for the Z component.</param>
    /// <param name="wAmount">The amount to add for the W component.</param>
    /// <returns>A copy of this vector after computing the sum of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="amount">The scalar to add for all components.</param>
    /// <returns>A copy of this vector after computing the sum of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d AddInPlace(Fixed64 amount) => AddInPlace(amount, amount, amount, amount);

    /// <summary>
    /// Adds another vector in place.
    /// </summary>
    /// <param name="other">The per-component values to add.</param>
    /// <returns>A copy of this vector after computing the sum of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d AddInPlace(Vector4d other) => AddInPlace(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Subtracts the specified component values in place.
    /// </summary>
    /// <param name="xAmount">The amount to subtract for the X component.</param>
    /// <param name="yAmount">The amount to subtract for the Y component.</param>
    /// <param name="zAmount">The amount to subtract for the Z component.</param>
    /// <param name="wAmount">The amount to subtract for the W component.</param>
    /// <returns>A copy of this vector after computing the difference of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="amount">The scalar to subtract for all components.</param>
    /// <returns>A copy of this vector after computing the difference of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d SubtractInPlace(Fixed64 amount) => SubtractInPlace(amount, amount, amount, amount);


    /// <summary>
    /// Subtracts another vector in place.
    /// </summary>
    /// <param name="other">The per-component values to subtract.</param>
    /// <returns>A copy of this vector after computing the difference of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d SubtractInPlace(Vector4d other) => SubtractInPlace(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Multiplies the vector components by the specified factors in place.
    /// </summary>
    /// <param name="factorX">The amount to multiply by for the X component.</param>
    /// <param name="factorY">The amount to multiply by for the Y component.</param>
    /// <param name="factorZ">The amount to multiply by for the Z component.</param>
    /// <param name="factorW">The amount to multiply by for the W component.</param>
    /// <returns>A copy of this vector after computing the product of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="factor">The scalar to multiply by for all components.</param>
    /// <returns>A copy of this vector after computing the product of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d MultiplyInPlace(Fixed64 factor) => MultiplyInPlace(factor, factor, factor, factor);

    /// <summary>
    /// Multiplies all components by another vector in place.
    /// </summary>
    /// <param name="factor">The per-component values to multiply by.</param>
    /// <returns>A copy of this vector after computing the product of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d MultiplyInPlace(Vector4d factor) => MultiplyInPlace(factor.X, factor.Y, factor.Z, factor.W);

    /// <summary>
    /// Divides the vector components by the specified divisors in place.
    /// </summary>
    /// <param name="divisorX">The amount to divide by for the X component.</param>
    /// <param name="divisorY">The amount to divide by for the Y component.</param>
    /// <param name="divisorZ">The amount to divide by for the Z component.</param>
    /// <param name="divisorW">The amount to divide by for the W component.</param>
    /// <returns>A copy of this vector after computing the quotient of its corresponding components. Each result saturates at the representable endpoints.</returns>
    /// <remarks>The method updates components in X, Y, Z, W order. On failure, the earlier components can already be updated.</remarks>
    /// <exception cref="DivideByZeroException">A divisor is zero.</exception>
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
    /// <param name="divisor">The scalar to divide by for all components.</param>
    /// <returns>A copy of this vector after computing the quotient of its components. Each result saturates at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">A divisor is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d DivideInPlace(Fixed64 divisor) => DivideInPlace(divisor, divisor, divisor, divisor);

    /// <summary>
    /// Divides all components by another vector in place.
    /// </summary>
    /// <param name="divisor">The per-component values to divide by.</param>
    /// <returns>A copy of this vector after computing the quotient of its corresponding components. Each result saturates at the representable endpoints.</returns>
    /// <remarks>The method updates components in X, Y, Z, W order. On failure, the earlier components can already be updated.</remarks>
    /// <exception cref="DivideByZeroException">A divisor is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d DivideInPlace(Vector4d divisor) => DivideInPlace(divisor.X, divisor.Y, divisor.Z, divisor.W);

    /// <summary>Normalizes this vector in place.</summary>
    /// <returns>A copy of this vector after normalization. The zero vector stays zero.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d NormalizeInPlace() => this = GetNormalized(this);

    /// <summary>Normalizes this vector in place.</summary>
    /// <param name="mag">The computed original length. It reports one inside the normalization tolerance and saturates to <see cref="Fixed64.MaxValue"/> if necessary.</param>
    /// <returns>A copy of this vector after normalization. The zero vector stays zero.</returns>
    public Vector4d NormalizeInPlace(out Fixed64 mag)
    {
        Vector4d source = this;
        bool magnitudeIsRepresentable = TryGetMagnitude(
            source,
            out mag,
            out bool isNormalized);

        if (mag == Fixed64.Zero)
            return this = Zero;

        if (isNormalized)
            return this;

        if (!magnitudeIsRepresentable || mag == Fixed64.One)
            return this = WideNormalization.GetNormalized(source);

        if (mag <= FixedMath.ScaleSafeMagnitudeThreshold)
            return this = GetScaleNormalized(source);

        this = new Vector4d(
            FixedMath.FastDiv(X, mag),
            FixedMath.FastDiv(Y, mag),
            FixedMath.FastDiv(Z, mag),
            FixedMath.FastDiv(W, mag));
        return IsNormalized()
            ? this
            : this = WideNormalization.GetNormalized(source);
    }

    /// <summary>
    /// Determines whether this vector is normalized.
    /// </summary>
    /// <returns><see langword="true"/> if the vector is nonzero and its squared length differs from one by at most <see cref="Fixed64.Epsilon"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool IsNormalized() =>
        !IsZero && FixedMath.Abs(MagnitudeSquared - Fixed64.One) <= Fixed64.Epsilon;

    /// <summary>Checks all signed components against epsilon.</summary>
    /// <returns><see langword="true"/> if every signed component strictly exceeds <see cref="Fixed64.Epsilon"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool AllComponentsGreaterThanEpsilon() =>
        X > Fixed64.Epsilon && Y > Fixed64.Epsilon && Z > Fixed64.Epsilon && W > Fixed64.Epsilon;

    /// <summary>
    /// Snaps components with absolute values below the threshold to zero.
    /// </summary>
    /// <param name="threshold">The exclusive magnitude threshold. Null selects <see cref="Fixed64.Epsilon"/>.</param>
    /// <returns>A copy in which each component with a magnitude below the threshold is zero.</returns>
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
    /// <param name="otherX">The X component in the same coordinate space as this vector.</param>
    /// <param name="otherY">The Y component in the same coordinate space as this vector.</param>
    /// <param name="otherZ">The Z component in the same coordinate space as this vector.</param>
    /// <param name="otherW">The W component in the same coordinate space as this vector.</param>
    /// <returns>The Euclidean distance in the same units as the coordinates, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Distance(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ, Fixed64 otherW)
    {
        Fixed64 deltaX = otherX - X;
        Fixed64 deltaY = otherY - Y;
        Fixed64 deltaZ = otherZ - Z;
        Fixed64 deltaW = otherW - W;
        Fixed64 squareSum = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ + deltaW * deltaW;
        return squareSum == Fixed64.MaxValue
            ? FixedMath.GetScaledMagnitude(deltaX, deltaY, deltaZ, deltaW)
            : FixedMath.Sqrt(squareSum);
    }

    /// <summary>
    /// Calculates the distance to another vector.
    /// </summary>
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>The Euclidean distance in the same units as the coordinates, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Distance(Vector4d other) => Distance(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Calculates the squared distance to another vector.
    /// </summary>
    /// <param name="otherX">The X component in the same coordinate space as this vector.</param>
    /// <param name="otherY">The Y component in the same coordinate space as this vector.</param>
    /// <param name="otherZ">The Z component in the same coordinate space as this vector.</param>
    /// <param name="otherW">The W component in the same coordinate space as this vector.</param>
    /// <returns>The squared Euclidean distance in squared coordinate units, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
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
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>The squared Euclidean distance in squared coordinate units, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 DistanceSquared(Vector4d other) => DistanceSquared(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Calculates the dot product with another vector.
    /// </summary>
    /// <param name="otherX">The X component in the same coordinate space as this vector.</param>
    /// <param name="otherY">The Y component in the same coordinate space as this vector.</param>
    /// <param name="otherZ">The Z component in the same coordinate space as this vector.</param>
    /// <param name="otherW">The W component in the same coordinate space as this vector.</param>
    /// <returns>The sum of the component products using rounded, saturating fixed-point operations.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Dot(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ, Fixed64 otherW) =>
        (X * otherX) + (Y * otherY) + (Z * otherZ) + (W * otherW);

    /// <summary>
    /// Calculates the dot product with another vector.
    /// </summary>
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>The sum of the component products using rounded, saturating fixed-point operations.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Fixed64 Dot(Vector4d other) => Dot(other.X, other.Y, other.Z, other.W);

    /// <summary>
    /// Converts this vector to Vector3d by dropping the W component.
    /// </summary>
    /// <remarks>
    /// This is a component-preserving conversion, not a coordinate-convention conversion.
    /// </remarks>
    /// <returns>The unchanged X, Y, and Z components.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3d ToVector3d() => new Vector3d(X, Y, Z);

    #endregion

    #region Static Methods

    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">The second vector to add.</param>
    /// <returns>The component-wise sum, with each component saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Add(Vector4d v1, Vector4d v2) => v1 + v2;

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    /// <param name="v1">The vector to subtract from.</param>
    /// <param name="v2">The vector to subtract.</param>
    /// <returns>The component-wise difference, with each component saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Subtract(Vector4d v1, Vector4d v2) => v1 - v2;

    /// <summary>
    /// Linearly interpolates between two vectors, clamping the interpolation amount to [0, 1].
    /// </summary>
    /// <param name="a">The starting vector.</param>
    /// <param name="b">The ending vector in the same coordinate space.</param>
    /// <param name="amount">The interpolation fraction, clamped to [0, 1].</param>
    /// <returns>
    /// <paramref name="a"/> for factors at or below zero, <paramref name="b"/> for factors at or above one,
    /// or the interpolated vector rounded to nearest with ties to even per component.
    /// </returns>
    public static Vector4d Lerp(Vector4d a, Vector4d b, Fixed64 amount) =>
        new(FixedMath.Lerp(a.X, b.X, amount),
            FixedMath.Lerp(a.Y, b.Y, amount),
            FixedMath.Lerp(a.Z, b.Z, amount),
            FixedMath.Lerp(a.W, b.W, amount));

    /// <summary>
    /// Linearly interpolates between two vectors without clamping the interpolation amount.
    /// </summary>
    /// <param name="a">The starting vector.</param>
    /// <param name="b">The ending vector in the same coordinate space.</param>
    /// <param name="t">The interpolation fraction. Values outside [0, 1] extrapolate.</param>
    /// <returns>
    /// <paramref name="a"/> at zero, <paramref name="b"/> at one,
    /// or the result of <c>a + (b − a) * t</c> using saturating component arithmetic.
    /// </returns>
    /// <remarks>A saturated component difference can cause results near a factor of one to differ sharply from the ending vector.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d UnclampedLerp(Vector4d a, Vector4d b, Fixed64 t) =>
        t == Fixed64.One ? b : new(a.X + ((b.X - a.X) * t),
            a.Y + ((b.Y - a.Y) * t),
            a.Z + ((b.Z - a.Z) * t),
            a.W + ((b.W - a.W) * t));

    /// <summary>
    /// Normalizes the given vector, returning a unit vector with the same direction.
    /// </summary>
    /// <param name="value">The vector to normalize.</param>
    /// <returns>An approximate unit vector in the same direction, or zero if the input is zero.</returns>
    public static Vector4d GetNormalized(Vector4d value)
    {
        bool magnitudeIsRepresentable = TryGetMagnitude(
            value,
            out Fixed64 mag,
            out bool isNormalized);

        if (mag == Fixed64.Zero)
            return Zero;

        if (isNormalized)
            return value;

        if (!magnitudeIsRepresentable || mag == Fixed64.One)
            return WideNormalization.GetNormalized(value);

        if (mag <= FixedMath.ScaleSafeMagnitudeThreshold)
            return GetScaleNormalized(value);

        var normalized = new Vector4d(
            FixedMath.FastDiv(value.X, mag),
            FixedMath.FastDiv(value.Y, mag),
            FixedMath.FastDiv(value.Z, mag),
            FixedMath.FastDiv(value.W, mag));
        return normalized.IsNormalized()
            ? normalized
            : WideNormalization.GetNormalized(value);
    }

    private static Vector4d GetScaleNormalized(Vector4d value)
    {
        Fixed64 scale = FixedMath.Max(
            FixedMath.Max(value.X.Abs(), value.Y.Abs()),
            FixedMath.Max(value.Z.Abs(), value.W.Abs()));
        Vector4d scaled = value / scale;
        Fixed64 scaledMagnitude = FixedMath.GetScaledMagnitude(scaled.X, scaled.Y, scaled.Z, scaled.W);
        return scaled / scaledMagnitude;
    }

    /// <summary>
    /// Returns the magnitude of the given vector.
    /// </summary>
    /// <param name="vector">The vector to measure.</param>
    /// <returns>The Euclidean length in input units. Lengths outside the representable range saturate to <see cref="Fixed64.MaxValue"/>. A vector within the normalization tolerance reports one.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetMagnitude(Vector4d vector)
    {
        _ = TryGetMagnitude(vector, out Fixed64 magnitude);
        return magnitude;
    }

    /// <summary>
    /// Attempts to return the magnitude of the given vector without saturating the result.
    /// </summary>
    /// <param name="vector">The vector to measure.</param>
    /// <param name="magnitude">The magnitude, or <see cref="Fixed64.MaxValue"/> when it is not representable.</param>
    /// <returns><see langword="true"/> when the magnitude fits in <see cref="Fixed64"/>; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryGetMagnitude(Vector4d vector, out Fixed64 magnitude) =>
        TryGetMagnitude(vector, out magnitude, out _);

    private static bool TryGetMagnitude(
        Vector4d vector,
        out Fixed64 magnitude,
        out bool isNormalized)
    {
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y) + (vector.Z * vector.Z) + (vector.W * vector.W);
        isNormalized = mag != Fixed64.Zero
            && FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon;

        if (mag == Fixed64.MaxValue)
            return FixedMath.TryGetScaledMagnitude(vector.X, vector.Y, vector.Z, vector.W, out magnitude);

        if (mag <= FixedMath.ScaleSafeMagnitudeSquaredThreshold)
        {
            magnitude = FixedMath.GetScaledMagnitude(vector.X, vector.Y, vector.Z, vector.W);
            return true;
        }

        if (isNormalized)
        {
            magnitude = Fixed64.One;
            return true;
        }

        magnitude = FixedMath.Sqrt(mag);
        return true;
    }

    /// <summary>
    /// Compares the exact squared magnitudes of two vectors without fixed-point saturation.
    /// </summary>
    /// <returns>A negative value when <paramref name="left"/> is shorter, zero when the
    /// magnitudes are equal, or a positive value when <paramref name="left"/> is longer.</returns>
    /// <param name="left">The first vector.</param>
    /// <param name="right">The second vector.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CompareMagnitudeSquared(Vector4d left, Vector4d right) =>
        Fixed64.CompareMagnitudeSquared(
            left.X,
            left.Y,
            left.Z,
            left.W,
            right.X,
            right.Y,
            right.Z,
            right.W);

    /// <summary>
    /// Returns a new vector containing the absolute value of each component.
    /// </summary>
    /// <param name="value">The signed vector.</param>
    /// <returns>A vector of component magnitudes. The magnitude of <see cref="Fixed64.MinValue"/> saturates to <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Abs(Vector4d value) =>
        new(FixedMath.Abs(value.X),
            FixedMath.Abs(value.Y),
            FixedMath.Abs(value.Z),
            FixedMath.Abs(value.W));

    /// <summary>
    /// Returns a new vector containing the sign of each component.
    /// </summary>
    /// <param name="value">The signed vector.</param>
    /// <returns>The signed component classifications, with −1 for negative values, zero for zero, and 1 for positive values.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Sign(Vector4d value) =>
        new(value.X.Sign(),
            value.Y.Sign(),
            value.Z.Sign(),
            value.W.Sign());

    /// <summary>
    /// Clamps each component of the given vector within the specified min and max bounds.
    /// </summary>
    /// <param name="value">The vector to clamp.</param>
    /// <param name="min">The inclusive lower limits. Each must be at most its matching upper limit.</param>
    /// <param name="max">The inclusive upper limits.</param>
    /// <returns>A copy with each component limited to its corresponding interval.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Clamp(Vector4d value, Vector4d min, Vector4d max) =>
        new(FixedMath.Clamp(value.X, min.X, max.X),
            FixedMath.Clamp(value.Y, min.Y, max.Y),
            FixedMath.Clamp(value.Z, min.Z, max.Z),
            FixedMath.Clamp(value.W, min.W, max.W));

    /// <summary>
    /// Computes the midpoint between two vectors.
    /// </summary>
    /// <param name="v1">The first point.</param>
    /// <param name="v2">The second point in the same coordinate space.</param>
    /// <returns>The midpoint, rounded to the nearest representable value with ties to even per component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Midpoint(Vector4d v1, Vector4d v2) =>
        new(FixedMath.Midpoint(v1.X, v2.X),
            FixedMath.Midpoint(v1.Y, v2.Y),
            FixedMath.Midpoint(v1.Z, v2.Z),
            FixedMath.Midpoint(v1.W, v2.W));

    /// <summary>Computes the distance between two points.</summary>
    /// <param name="start">The first point.</param>
    /// <param name="end">The second point in the same coordinate space.</param>
    /// <returns>The Euclidean distance in the same units as the coordinates, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector4d start, Vector4d end) =>
        start.Distance(end.X, end.Y, end.Z, end.W);

    /// <summary>Computes the squared distance between two points.</summary>
    /// <param name="start">The first point.</param>
    /// <param name="end">The second point in the same coordinate space.</param>
    /// <returns>The squared Euclidean distance in squared coordinate units, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 DistanceSquared(Vector4d start, Vector4d end) =>
        start.DistanceSquared(end.X, end.Y, end.Z, end.W);

    /// <summary>
    /// Calculates the dot product of two vectors.
    /// </summary>
    /// <param name="lhs">The first vector.</param>
    /// <param name="rhs">The second vector in the same coordinate space.</param>
    /// <returns>The sum of the component products using rounded, saturating fixed-point operations.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(Vector4d lhs, Vector4d rhs) =>
        lhs.Dot(rhs.X, rhs.Y, rhs.Z, rhs.W);

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    /// <param name="a">The first factor.</param>
    /// <param name="b">The second factor.</param>
    /// <returns>The component-wise product, with each component saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Multiply(Vector4d a, Vector4d b) =>
        new(a.X * b.X, a.Y * b.Y, a.Z * b.Z, a.W * b.W);

    /// <summary>
    /// Multiplies each vector component by the specified scalar.
    /// </summary>
    /// <param name="value">The vector to scale.</param>
    /// <param name="factor">The scalar factor.</param>
    /// <returns>The component-wise product, with each component saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Multiply(Vector4d value, Fixed64 factor) => value * factor;

    /// <summary>
    /// Divides each component of the first vector by the corresponding component of the second vector.
    /// </summary>
    /// <param name="v1">The numerator.</param>
    /// <param name="v2">The divisor.</param>
    /// <returns>The component-wise quotient, with each component saturated at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">A divisor component is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Divide(Vector4d v1, Vector4d v2) => v1 / v2;

    /// <summary>
    /// Divides each vector component by the specified scalar.
    /// </summary>
    /// <param name="value">The vector to scale.</param>
    /// <param name="divisor">The scalar divisor.</param>
    /// <returns>The component-wise quotient, with each component saturated at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">The divisor is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Divide(Vector4d value, Fixed64 divisor) => value / divisor;

    /// <summary>
    /// Returns a vector whose elements are the maximum of each pair of elements.
    /// </summary>
    /// <param name="value1">The first vector.</param>
    /// <param name="value2">The second vector in the same coordinate space.</param>
    /// <returns>A vector containing the larger value from each pair of corresponding components.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Max(Vector4d value1, Vector4d value2) =>
        new(FixedMath.Max(value1.X, value2.X),
            FixedMath.Max(value1.Y, value2.Y),
            FixedMath.Max(value1.Z, value2.Z),
            FixedMath.Max(value1.W, value2.W));

    /// <summary>
    /// Returns a vector whose elements are the minimum of each pair of elements.
    /// </summary>
    /// <param name="value1">The first vector.</param>
    /// <param name="value2">The second vector in the same coordinate space.</param>
    /// <returns>A vector containing the smaller value from each pair of corresponding components.</returns>
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
    /// <param name="matrix">The transform in the row-vector convention.</param>
    /// <param name="vector">The input homogeneous row vector.</param>
    /// <returns>The transformed vector with its computed W component, before any perspective division.</returns>
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
    /// <param name="v1">The left operand.</param>
    /// <param name="v2">The right operand.</param>
    /// <returns>The component-wise sum. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, Vector4d v2) => new(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z, v1.W + v2.W);

    /// <summary>
    /// Adds a scalar to each component.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="mag">The right operand, applied to every component.</param>
    /// <returns>The component-wise sum. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, Fixed64 mag) => new(v1.X + mag, v1.Y + mag, v1.Z + mag, v1.W + mag);

    /// <summary>Adds a scalar to each component.</summary>
    /// <param name="mag">The left operand, applied to every component.</param>
    /// <param name="v1">The right operand.</param>
    /// <returns>The component-wise sum. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Fixed64 mag, Vector4d v1) => v1 + mag;

    /// <summary>
    /// Adds a tuple to each component.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="v2">The right operand, in X, Y, Z, W order.</param>
    /// <returns>The component-wise sum. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +(Vector4d v1, (int x, int y, int z, int w) v2) =>
        new(v1.X + v2.x, v1.Y + v2.y, v1.Z + v2.z, v1.W + v2.w);

    /// <summary>
    /// Adds a tuple to each component.
    /// </summary>
    /// <param name="v2">The left operand, in X, Y, Z, W order.</param>
    /// <param name="v1">The right operand.</param>
    /// <returns>The component-wise sum. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator +((int x, int y, int z, int w) v2, Vector4d v1) => v1 + v2;

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="v2">The right operand.</param>
    /// <returns>The component-wise difference. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, Vector4d v2) =>
        new(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z, v1.W - v2.W);

    /// <summary>
    /// Subtracts a scalar from each component.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="mag">The right operand, applied to every component.</param>
    /// <returns>The component-wise difference. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, Fixed64 mag) =>
        new(v1.X - mag, v1.Y - mag, v1.Z - mag, v1.W - mag);

    /// <summary>Subtracts each component from a scalar.</summary>
    /// <param name="mag">The left operand, applied to every component.</param>
    /// <param name="v1">The right operand.</param>
    /// <returns>The component-wise difference. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Fixed64 mag, Vector4d v1) =>
        new(mag - v1.X, mag - v1.Y, mag - v1.Z, mag - v1.W);

    /// <summary>
    /// Subtracts a tuple from each component.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="v2">The right operand, in X, Y, Z, W order.</param>
    /// <returns>The component-wise difference. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1, (int x, int y, int z, int w) v2) =>
        new(v1.X - v2.x, v1.Y - v2.y, v1.Z - v2.z, v1.W - v2.w);

    /// <summary>
    /// Subtracts each vector component from a tuple component.
    /// </summary>
    /// <param name="v1">The left operand, in X, Y, Z, W order.</param>
    /// <param name="v2">The right operand.</param>
    /// <returns>The component-wise difference. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -((int x, int y, int z, int w) v1, Vector4d v2) =>
        new(v1.x - v2.X, v1.y - v2.Y, v1.z - v2.Z, v1.w - v2.W);

    /// <summary>
    /// Negates the vector.
    /// </summary>
    /// <param name="v1">The vector to negate.</param>
    /// <returns>The negated components. A component equal to <see cref="Fixed64.MinValue"/> saturates to <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator -(Vector4d v1) =>
        new(-v1.X, -v1.Y, -v1.Z, -v1.W);

    /// <summary>
    /// Multiplies each component by a scalar.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="mag">The right operand, applied to every component.</param>
    /// <returns>The component-wise product. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, Fixed64 mag) =>
        new(v1.X * mag, v1.Y * mag, v1.Z * mag, v1.W * mag);

    /// <summary>Multiplies each component by a scalar.</summary>
    /// <param name="mag">The left operand, applied to every component.</param>
    /// <param name="v1">The right operand.</param>
    /// <returns>The component-wise product. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Fixed64 mag, Vector4d v1) => v1 * mag;

    /// <summary>
    /// Multiplies each component by an integer scalar.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="mag">The right operand, applied to every component.</param>
    /// <returns>The component-wise product. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, int mag) => v1 * (Fixed64)mag;

    /// <summary>Multiplies each component by a scalar.</summary>
    /// <param name="mag">The left operand, applied to every component.</param>
    /// <param name="v1">The right operand.</param>
    /// <returns>The component-wise product. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(int mag, Vector4d v1) => v1 * mag;

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="v2">The right operand.</param>
    /// <returns>The component-wise product. Components saturate at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d v1, Vector4d v2) => Multiply(v1, v2);

    /// <summary>Transforms a homogeneous row vector by a matrix.</summary>
    /// <param name="matrix">The transform matrix.</param>
    /// <param name="vector">The input homogeneous row vector.</param>
    /// <returns>The transformed vector with its computed W component. Both operand orders apply the same transform.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Fixed4x4 matrix, Vector4d vector) => Transform(matrix, vector);

    /// <inheritdoc cref="operator *(Fixed4x4, Vector4d)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator *(Vector4d vector, Fixed4x4 matrix) => matrix * vector;

    /// <summary>
    /// Divides each component by a scalar.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="div">The right operand, applied to every component.</param>
    /// <returns>The component-wise quotient. Components saturate at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">The divisor or a divisor component is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, Fixed64 div) => new(v1.X / div, v1.Y / div, v1.Z / div, v1.W / div);

    /// <summary>
    /// Divides each component by another vector's corresponding component.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="v2">The right operand.</param>
    /// <returns>The component-wise quotient. Components saturate at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">The divisor or a divisor component is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, Vector4d v2) => new(v1.X / v2.X, v1.Y / v2.Y, v1.Z / v2.Z, v1.W / v2.W);

    /// <summary>
    /// Divides each component by an integer scalar.
    /// </summary>
    /// <param name="v1">The left operand.</param>
    /// <param name="div">The right operand, applied to every component.</param>
    /// <returns>The component-wise quotient. Components saturate at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">The divisor or a divisor component is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d operator /(Vector4d v1, int div) => v1 / (Fixed64)div;

    /// <summary>
    /// Compares two vectors for equality.
    /// </summary>
    /// <param name="left">The left vector.</param>
    /// <param name="right">The right vector.</param>
    /// <returns><see langword="true"/> if all corresponding components are exactly equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Vector4d left, Vector4d right) => left.Equals(right);

    /// <summary>
    /// Compares two vectors for inequality.
    /// </summary>
    /// <param name="left">The left vector.</param>
    /// <param name="right">The right vector.</param>
    /// <returns><see langword="true"/> if at least one corresponding component differs, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Vector4d left, Vector4d right) => !left.Equals(right);

    /// <summary>
    /// Returns true when every component in the left vector is greater than the corresponding component in the right vector.
    /// </summary>
    /// <param name="left">The left vector.</param>
    /// <param name="right">The right vector.</param>
    /// <returns><see langword="true"/> if every left component is greater than its matching right component, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Vector4d left, Vector4d right) =>
        left.X > right.X && left.Y > right.Y && left.Z > right.Z && left.W > right.W;

    /// <summary>
    /// Returns true when every component in the left vector is less than the corresponding component in the right vector.
    /// </summary>
    /// <param name="left">The left vector.</param>
    /// <param name="right">The right vector.</param>
    /// <returns><see langword="true"/> if every left component is less than its matching right component, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Vector4d left, Vector4d right) =>
        left.X < right.X && left.Y < right.Y && left.Z < right.Z && left.W < right.W;

    /// <summary>
    /// Returns true when every component in the left vector is greater than or equal to the corresponding component in the right vector.
    /// </summary>
    /// <param name="left">The left vector.</param>
    /// <param name="right">The right vector.</param>
    /// <returns><see langword="true"/> if every left component is greater than or equal to its matching right component, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Vector4d left, Vector4d right) =>
        left.X >= right.X && left.Y >= right.Y && left.Z >= right.Z && left.W >= right.W;

    /// <summary>
    /// Returns true when every component in the left vector is less than or equal to the corresponding component in the right vector.
    /// </summary>
    /// <param name="left">The left vector.</param>
    /// <param name="right">The right vector.</param>
    /// <returns><see langword="true"/> if every left component is less than or equal to its matching right component, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Vector4d left, Vector4d right) =>
        left.X <= right.X && left.Y <= right.Y && left.Z <= right.Z && left.W <= right.W;

    #endregion

    #region Conversion and Formatting

    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    /// <returns>A parenthesized, comma-separated list of components, using the default invariant numeric format.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    /// <param name="format">The numeric format for each component. A null or empty string uses the default format.</param>
    /// <param name="formatProvider">The provider used for all components. Null selects the invariant culture.</param>
    /// <returns>A parenthesized, comma-separated list of formatted components.</returns>
    /// <exception cref="FormatException">The component format is invalid.</exception>
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
    /// <param name="destination">The caller-owned output span. This method writes from its start and may leave a partial prefix on failure.</param>
    /// <param name="charsWritten">The number of characters written on success, or zero when the destination is too small.</param>
    /// <param name="format">The numeric format for each component. An empty span selects the default format.</param>
    /// <param name="provider">The provider used for all components. Null selects the invariant culture.</param>
    /// <returns><see langword="true"/> if all components and punctuation fit, otherwise <see langword="false"/>.</returns>
    /// <exception cref="FormatException">The component format is invalid.</exception>
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
    /// <param name="x">The fixed-point X component.</param>
    /// <param name="y">The fixed-point Y component.</param>
    /// <param name="z">The fixed-point Z component.</param>
    /// <param name="w">The fixed-point W component.</param>
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
    /// <param name="x">The X component rounded to the nearest integer with ties to even.</param>
    /// <param name="y">The Y component rounded to the nearest integer with ties to even.</param>
    /// <param name="z">The Z component rounded to the nearest integer with ties to even.</param>
    /// <param name="w">The W component rounded to the nearest integer with ties to even.</param>
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
    /// <param name="x">The signed 64-bit raw Q32.32 payload of the X component.</param>
    /// <param name="y">The signed 64-bit raw Q32.32 payload of the Y component.</param>
    /// <param name="z">The signed 64-bit raw Q32.32 payload of the Z component.</param>
    /// <param name="w">The signed 64-bit raw Q32.32 payload of the W component.</param>
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
    /// <param name="x">The double-precision approximation of the X component.</param>
    /// <param name="y">The double-precision approximation of the Y component.</param>
    /// <param name="z">The double-precision approximation of the Z component.</param>
    /// <param name="w">The double-precision approximation of the W component.</param>
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

    /// <summary>Compares vectors by corresponding components.</summary>
    /// <param name="obj">The object to compare.</param>
    /// <returns><see langword="true"/> if the object is a <see cref="Vector4d"/> with equal corresponding components, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly bool Equals(object? obj) => obj is Vector4d other && Equals(other);

    /// <summary>Compares vectors by corresponding components.</summary>
    /// <param name="other">The vector to compare.</param>
    /// <returns><see langword="true"/> if all corresponding components are exactly equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Equals(Vector4d other) => other.X == X && other.Y == Y && other.Z == Z && other.W == W;

    /// <summary>Compares vectors by corresponding components.</summary>
    /// <param name="x">The first vector.</param>
    /// <param name="y">The second vector.</param>
    /// <returns><see langword="true"/> if all corresponding components are exactly equal, otherwise <see langword="false"/>.</returns>
    public readonly bool Equals(Vector4d x, Vector4d y) => x.Equals(y);

    /// <summary>Gets a hash code based on the raw components.</summary>
    /// <returns>The value of <see cref="StateHash"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly int GetHashCode() => StateHash;

    /// <summary>Gets a hash code based on the raw components.</summary>
    /// <param name="obj">The vector to hash.</param>
    /// <returns>The supplied vector’s <see cref="StateHash"/>.</returns>
    public readonly int GetHashCode(Vector4d obj) => obj.GetHashCode();

    /// <summary>
    /// Compares the current Vector4d instance with another Vector4d based on squared magnitude.
    /// </summary>
    /// <param name="other">The vector whose squared length is compared.</param>
    /// <returns>A negative value, zero, or a positive value when this rounded and saturated squared length compares less than, equal to, or greater than the other squared length.</returns>
    /// <remarks>Use <see cref="CompareMagnitudeSquared(Vector4d, Vector4d)"/> for an exact length comparison.</remarks>
    public readonly int CompareTo(Vector4d other) => MagnitudeSquared.CompareTo(other.MagnitudeSquared);

    #endregion
}
