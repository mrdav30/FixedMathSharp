//=======================================================================
// Vector3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using FixedMathSharp.Geometry;
using MemoryPack;

namespace FixedMathSharp;

/// <summary>
/// Represents a 3D vector with fixed-point precision, supporting a wide range of vector operations such as rotation, scaling, interpolation, and projection.
/// </summary>
/// <remarks>
/// The Vector3d struct is designed for high-precision applications in 3D space, including games, simulations, and physics engines. 
/// It offers essential operations like addition, subtraction, dot product, cross product, distance calculation, and normalization.
///
/// FixedMathSharp's canonical 3D basis is <c>+X</c> right, <c>+Y</c> up, and <c>+Z</c>
/// forward. Direction-named core APIs use that convention; engine-specific naming, basis,
/// handedness, and matrix semantics should be converted at adapter boundaries.
/// 
/// Use Cases:
/// - Modeling 3D positions, directions, and velocities with fixed-point precision.
/// - Performing vector transformations, including rotations using quaternions.
/// - Calculating distances, angles, projections, and interpolation between vectors.
/// - Essential for fixed-point math scenarios where floating-point precision isn't suitable.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Vector3d : IEquatable<Vector3d>, IComparable<Vector3d>, IEqualityComparer<Vector3d>, IFormattable
#if NET10_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly Fields

    /// <summary>
    /// The upward direction vector in the canonical 3D basis (0, 1, 0).
    /// </summary>
    /// <value>The positive Y unit vector.</value>
    public static Vector3d Up => new(0, 1, 0);

    /// <summary>
    /// The right direction vector in the canonical 3D basis (1, 0, 0).
    /// </summary>
    /// <value>The positive X unit vector.</value>
    public static Vector3d Right => new(1, 0, 0);

    /// <summary>
    /// The downward direction vector in the canonical 3D basis (0, -1, 0).
    /// </summary>
    /// <value>The negative Y unit vector.</value>
    public static Vector3d Down => new(0, -1, 0);

    /// <summary>
    /// The left direction vector in the canonical 3D basis (-1, 0, 0).
    /// </summary>
    /// <value>The negative X unit vector.</value>
    public static Vector3d Left => new(-1, 0, 0);

    /// <summary>
    /// The forward direction vector in the canonical 3D basis (0, 0, 1).
    /// </summary>
    /// <remarks>
    /// FixedMathSharp defines semantic forward as <c>+Z</c>. Use
    /// <see cref="CoordinateConvention3d"/> at adapter boundaries when an external API uses
    /// different semantic axes, such as <c>-Z</c> forward or <c>+X</c> forward with <c>+Z</c> up.
    /// </remarks>
    /// <value>The positive Z unit vector.</value>
    public static Vector3d Forward => new(0, 0, 1);

    /// <summary>
    /// The backward direction vector in the canonical 3D basis (0, 0, -1).
    /// </summary>
    /// <value>The negative Z unit vector.</value>
    public static Vector3d Backward => new(0, 0, -1);

    /// <summary>
    /// (1, 1, 1)
    /// </summary>
    /// <value>All components equal to one.</value>
    public static Vector3d One => new(1, 1, 1);

    /// <summary>
    /// (-1, -1, -1)
    /// </summary>
    /// <value>All components equal to −1.</value>
    public static Vector3d Negative => new(-1, -1, -1);

    /// <summary>
    /// (0, 0, 0)
    /// </summary>
    /// <value>All components equal to zero.</value>
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
        X = x;
        Y = y;
        Z = z;
    }

    /// <summary>Creates a vector from double-precision components.</summary>
    /// <param name="xDoub">The X component.</param>
    /// <param name="yDoub">The Y component.</param>
    /// <param name="zDoub">The Z component.</param>
    /// <returns>A vector whose components are rounded to the nearest Q32.32 value with ties to even.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A component is NaN or infinity.</exception>
    /// <exception cref="OverflowException">A rounded component lies outside the Q32.32 range.</exception>
    public static Vector3d FromDouble(double xDoub, double yDoub, double zDoub) =>
        new(Fixed64.FromDouble(xDoub),
            Fixed64.FromDouble(yDoub),
            Fixed64.FromDouble(zDoub));

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
    /// <value>The copy <c>(Z, Y, −X)</c>, rotated 90 degrees about positive Y.</value>
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
    /// <value>The copy <c>(−Z, Y, X)</c>, rotated 90 degrees about negative Y.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d LeftHandNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(-Z, Y, X);
    }

    /// <summary>Gets a normalized copy of the vector.</summary>
    /// <value>The approximate unit vector in the same direction, or zero for a zero vector.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Normalized
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetNormalized(this);
    }

    /// <summary>
    /// Returns the actual length of this vector (RO).
    /// </summary>
    /// <value>The Euclidean length in input units, saturated to <see cref="Fixed64.MaxValue"/> when too large. Values within the normalization tolerance report one.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Magnitude
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => GetMagnitude(this);
    }

    /// <summary>Gets the squared length of the vector.</summary>
    /// <value>The rounded sum of the squared components, in squared input units and saturated at <see cref="Fixed64.MaxValue"/>.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MagnitudeSquared
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
    /// At zero pitch and yaw this returns <see cref="Forward"/>.
    /// Positive pitch rotates the forward direction toward <see cref="Down"/>.
    /// </remarks>
    /// <value>The approximate forward unit direction for pitch X and yaw Y in radians. Z is ignored.</value>
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

    /// <summary>Tests whether all components equal zero.</summary>
    /// <value><see langword="true"/> if all components are exactly zero, otherwise <see langword="false"/>.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public bool IsZero
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Equals(Zero);
    }

    /// <summary>
    /// Returns a long hash of the vector based on its x, y, and z values.
    /// </summary>
    /// <value>A deterministic weighted sum of the raw components using wrapping 64-bit arithmetic.</value>
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
    /// <value>The signed remainder of <see cref="LongStateHash"/> modulo <see cref="int.MaxValue"/>.</value>
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
    /// <param name="index">The zero-based component index in X, Y, Z order.</param>
    /// <value>The selected component. Setting it replaces only that component.</value>
    /// <exception cref="IndexOutOfRangeException"><paramref name="index"/> is outside the interval [0, 2].</exception>
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
    /// <param name="newX">The new X component.</param>
    /// <param name="newY">The new Y component.</param>
    /// <param name="newZ">The new Z component.</param>
    /// <returns>A copy of this vector after all components have been assigned.</returns>
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
    /// <param name="xAmount">The amount to add for the X component.</param>
    /// <param name="yAmount">The amount to add for the Y component.</param>
    /// <param name="zAmount">The amount to add for the Z component.</param>
    /// <returns>A copy of this vector after computing the sum of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="amount">The scalar to add for all components.</param>
    /// <returns>A copy of this vector after computing the sum of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d AddInPlace(Fixed64 amount) => AddInPlace(amount, amount, amount);

    /// <summary>
    /// Adds the specified vector components to the corresponding components of the in place vector and returns the modified vector.
    /// </summary>
    /// <param name="other">The per-component values to add.</param>
    /// <returns>A copy of this vector after computing the sum of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d AddInPlace(Vector3d other) => AddInPlace(other.X, other.Y, other.Z);

    /// <summary>
    /// Subtracts the specified values from the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="xAmount">The amount to subtract for the X component.</param>
    /// <param name="yAmount">The amount to subtract for the Y component.</param>
    /// <param name="zAmount">The amount to subtract for the Z component.</param>
    /// <returns>A copy of this vector after computing the difference of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="amount">The scalar to subtract for all components.</param>
    /// <returns>A copy of this vector after computing the difference of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d SubtractInPlace(Fixed64 amount) => SubtractInPlace(amount, amount, amount);

    /// <summary>
    /// Subtracts the specified vector from the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="other">The per-component values to subtract.</param>
    /// <returns>A copy of this vector after computing the difference of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d SubtractInPlace(Vector3d other) => SubtractInPlace(other.X, other.Y, other.Z);

    /// <summary>
    /// Multiplies each component of the vector by the corresponding factor in place and returns the modified vector.
    /// </summary>
    /// <param name="factorX">The amount to multiply by for the X component.</param>
    /// <param name="factorY">The amount to multiply by for the Y component.</param>
    /// <param name="factorZ">The amount to multiply by for the Z component.</param>
    /// <returns>A copy of this vector after computing the product of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="factor">The scalar to multiply by for all components.</param>
    /// <returns>A copy of this vector after computing the product of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d MultiplyInPlace(Fixed64 factor) => MultiplyInPlace(factor, factor, factor);

    /// <summary>
    /// Multiplies each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
    /// </summary>
    /// <param name="factor">The per-component values to multiply by.</param>
    /// <returns>A copy of this vector after computing the product of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d MultiplyInPlace(Vector3d factor) => MultiplyInPlace(factor.X, factor.Y, factor.Z);

    /// <summary>
    /// Divides each component of the vector by the corresponding divisor in place and returns the modified vector.
    /// </summary>
    /// <param name="divisorX">The amount to divide by for the X component.</param>
    /// <param name="divisorY">The amount to divide by for the Y component.</param>
    /// <param name="divisorZ">The amount to divide by for the Z component.</param>
    /// <returns>A copy of this vector after computing the quotient of its corresponding components. Each result saturates at the representable endpoints. A zero divisor sets the corresponding component to zero.</returns>
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
    /// <param name="divisor">The scalar to divide by for all components.</param>
    /// <returns>A copy of this vector after computing the quotient of its components. Each result saturates at the representable endpoints. A zero divisor sets the corresponding component to zero.</returns>
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
    /// <param name="divisor">The per-component values to divide by.</param>
    /// <returns>A copy of this vector after computing the quotient of its corresponding components. Each result saturates at the representable endpoints. A zero divisor sets the corresponding component to zero.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d DivideInPlace(Vector3d divisor) => DivideInPlace(divisor.X, divisor.Y, divisor.Z);

    /// <summary>Normalizes this vector in place.</summary>
    /// <returns>A copy of this vector after normalization. The zero vector stays zero.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d NormalizeInPlace() => this = GetNormalized(this);

    /// <summary>Normalizes this vector in place.</summary>
    /// <param name="mag">The computed original length. It reports one inside the normalization tolerance and saturates to <see cref="Fixed64.MaxValue"/> if necessary.</param>
    /// <returns>A copy of this vector after normalization. The zero vector stays zero.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d NormalizeInPlace(out Fixed64 mag)
    {
        Vector3d source = this;
        bool magnitudeIsRepresentable = TryGetMagnitude(
            source,
            out mag,
            out bool isNormalized);

        // If magnitude is zero, return a zero vector to avoid divide-by-zero errors
        if (mag == Fixed64.Zero)
        {
            X = Fixed64.Zero;
            Y = Fixed64.Zero;
            Z = Fixed64.Zero;
            return this;
        }

        if (isNormalized)
            return this;

        if (!magnitudeIsRepresentable || mag == Fixed64.One)
            return this = WideNormalization.GetNormalized(source);

        if (mag <= FixedMath.ScaleSafeMagnitudeThreshold)
            return this = GetScaleNormalized(source);

        X = FixedMath.FastDiv(X, mag);
        Y = FixedMath.FastDiv(Y, mag);
        Z = FixedMath.FastDiv(Z, mag);

        return IsNormalized()
            ? this
            : this = WideNormalization.GetNormalized(source);
    }

    /// <summary>
    /// Checks whether the nonzero squared magnitude is within
    /// <see cref="Fixed64.Epsilon"/> of 1.
    /// </summary>
    /// <returns><see langword="true"/> if the vector is nonzero and its squared length differs from one by at most <see cref="Fixed64.Epsilon"/>, otherwise <see langword="false"/>.</returns>
    public bool IsNormalized()
    {
        Fixed64 sqrMagnitude = MagnitudeSquared;
        return sqrMagnitude != Fixed64.Zero && FixedMath.Abs(sqrMagnitude - Fixed64.One) <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Attempts to return this vector's magnitude rounded outward to the
    /// smallest containing Q32.32 value.
    /// </summary>
    /// <param name="magnitude">The smallest Q32.32 value at least as large as the exact Euclidean length on success, or zero on failure.</param>
    /// <returns><see langword="true"/> if the outward-rounded length is representable, otherwise <see langword="false"/>.</returns>
    public readonly bool TryGetMagnitudeCeiling(
        out Fixed64 magnitude) =>
        WideGeometry.TryGetMagnitudeCeiling(
            X,
            Y,
            Z,
            out magnitude);

    /// <summary>Checks the absolute values of all components against epsilon.</summary>
    /// <returns><see langword="true"/> if every component magnitude strictly exceeds <see cref="Fixed64.Epsilon"/>, otherwise <see langword="false"/>.</returns>
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
        Fixed64 deltaX = X - otherX;
        Fixed64 deltaY = Y - otherY;
        Fixed64 deltaZ = Z - otherZ;
        Fixed64 squareSum = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
        return squareSum == Fixed64.MaxValue
            ? FixedMath.GetScaledMagnitude(deltaX, deltaY, deltaZ, Fixed64.Zero)
            : FixedMath.Sqrt(squareSum);
    }

    /// <summary>
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <param name="otherX">The X component in the same coordinate space as this vector.</param>
    /// <param name="otherY">The Y component in the same coordinate space as this vector.</param>
    /// <param name="otherZ">The Z component in the same coordinate space as this vector.</param>
    /// <returns>The squared Euclidean distance in squared coordinate units, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ)
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

    /// <summary>Sums the three components of the cross product.</summary>
    /// <param name="otherX">The X component of the other vector.</param>
    /// <param name="otherY">The Y component of the other vector.</param>
    /// <param name="otherZ">The Z component of the other vector.</param>
    /// <returns>The signed sum of the right-handed cross product components, using rounded, saturating component arithmetic.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 CrossProduct(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ) =>
        (Y * otherZ - Z * otherY) + (Z * otherX - X * otherZ) + (X * otherY - Y * otherX);

    /// <summary>
    /// Returns the cross vector of this vector with another vector.
    /// </summary>
    /// <returns>A new vector representing the cross product.</returns>
    /// <param name="otherX">The X component of the second vector in the same coordinate space.</param>
    /// <param name="otherY">The Y component of the second vector in the same coordinate space.</param>
    /// <param name="otherZ">The Z component of the second vector in the same coordinate space.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d Cross(Fixed64 otherX, Fixed64 otherY, Fixed64 otherZ) =>
        new(Y * otherZ - Z * otherY,
            Z * otherX - X * otherZ,
            X * otherY - Y * otherX);

    #endregion
}
