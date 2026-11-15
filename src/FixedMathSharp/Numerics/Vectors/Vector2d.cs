//=======================================================================
// Vector2d.cs
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
/// Represents a 2D vector with fixed-point precision, offering a range of mathematical operations
/// and transformations such as rotation, scaling, reflection, and interpolation.
/// </summary>
/// <remarks>
/// The Vector2d struct is designed for applications that require precise numerical operations, 
/// such as games, simulations, or physics engines. It provides methods for common vector operations
/// like addition, subtraction, dot product, cross product, distance calculations, and rotation.
///
/// FixedMathSharp treats <see cref="Vector2d"/> as plane math: <c>+X</c> is right and
/// <c>+Y</c> is the named <see cref="Forward"/> direction. Polar angle helpers are separate:
/// <see cref="ForwardDirection(Fixed64)"/> returns <c>+X</c> at angle zero.
/// 
/// Use Cases:
/// - Modeling 2D positions, directions, and velocities in fixed-point math environments.
/// - Performing vector transformations, including rotations and reflections.
/// - Handling interpolation and distance calculations in physics or simulation systems.
/// - Useful for fixed-point math scenarios where floating-point precision is insufficient or not desired.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Vector2d : IEquatable<Vector2d>, IComparable<Vector2d>, IEqualityComparer<Vector2d>, IFormattable
#if NET10_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly Fields

    /// <summary>
    /// The named 2D plane forward direction (0, 1).
    /// </summary>
    /// <remarks>
    /// This is distinct from polar angle math: <see cref="ForwardDirection(Fixed64)"/> returns
    /// <see cref="Right"/> when its angle argument is zero.
    /// </remarks>
    /// <value>The positive Y unit vector.</value>
    public static Vector2d Forward => new(0, 1);

    /// <summary>
    /// (0, -1)
    /// </summary>
    /// <value>The negative Y unit vector.</value>
    public static Vector2d Backward => new(0, -1);

    /// <summary>
    /// The named 2D plane right direction (1, 0).
    /// </summary>
    /// <value>The positive X unit vector.</value>
    public static Vector2d Right => new(1, 0);

    /// <summary>
    /// (-1, 0)
    /// </summary>
    /// <value>The negative X unit vector.</value>
    public static Vector2d Left => new(-1, 0);

    /// <summary>
    /// (1, 1)
    /// </summary>
    /// <value>All components equal to one.</value>
    public static Vector2d One => new(1, 1);

    /// <summary>
    /// (-1, -1)
    /// </summary>
    /// <value>All components equal to −1.</value>
    public static Vector2d Negative => new(-1, -1);

    /// <summary>
    /// (0, 0)
    /// </summary>
    /// <value>All components equal to zero.</value>
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

    /// <summary>Creates a vector from double-precision components.</summary>
    /// <param name="xDoub">The X component.</param>
    /// <param name="yDoub">The Y component.</param>
    /// <returns>A vector whose components are rounded to the nearest Q32.32 value with ties to even.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A component is NaN or infinity.</exception>
    /// <exception cref="OverflowException">A rounded component lies outside the Q32.32 range.</exception>
    public static Vector2d FromDouble(double xDoub, double yDoub) =>
        new(Fixed64.FromDouble(xDoub), Fixed64.FromDouble(yDoub));

    #endregion
    #region Properties

    /// <summary>
    /// Rotates the vector to the right (90 degrees clockwise).
    /// </summary>
    /// <value>The perpendicular copy <c>(Y, −X)</c>.</value>
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
    /// <value>The perpendicular copy <c>(−Y, X)</c>.</value>
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
    /// <value>The perpendicular copy <c>(−Y, X)</c>.</value>
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
    /// <value>The perpendicular copy <c>(Y, −X)</c>.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d LeftHandNormal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(Y, -X);
    }

    /// <summary>Gets a normalized copy of the vector.</summary>
    /// <value>The approximate unit vector in the same direction, or zero for a zero vector.</value>
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
        get => X * X + Y * Y;
    }

    /// <summary>
    /// Returns a long hash of the vector based on its x and y values.
    /// </summary>
    /// <value>A deterministic weighted sum of the raw components using wrapping 64-bit arithmetic.</value>
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
    /// <value>The signed remainder of <see cref="LongStateHash"/> modulo <see cref="int.MaxValue"/>.</value>
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
    /// <param name="index">The zero-based component index in X, Y order.</param>
    /// <value>The selected component. Setting it replaces only that component.</value>
    /// <exception cref="IndexOutOfRangeException"><paramref name="index"/> is outside the interval [0, 1].</exception>
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
    /// <param name="newX">The new X component.</param>
    /// <param name="newY">The new Y component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Set(Fixed64 newX, Fixed64 newY)
    {
        X = newX;
        Y = newY;
    }

    /// <summary>
    /// Adds the specified values to the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="xAmount">The amount to add for the X component.</param>
    /// <param name="yAmount">The amount to add for the Y component.</param>
    /// <returns>A copy of this vector after computing the sum of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="amount">The scalar to add for all components.</param>
    /// <returns>A copy of this vector after computing the sum of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d AddInPlace(Fixed64 amount) => AddInPlace(amount, amount);

    /// <inheritdoc cref="AddInPlace(Fixed64, Fixed64)"/>
    /// <param name="other">The per-component values to add.</param>
    /// <returns>A copy of this vector after computing the sum of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d AddInPlace(Vector2d other) => AddInPlace(other.X, other.Y);

    /// <summary>
    /// Subtracts the specified values from the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="xAmount">The amount to subtract for the X component.</param>
    /// <param name="yAmount">The amount to subtract for the Y component.</param>
    /// <returns>A copy of this vector after computing the difference of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="amount">The scalar to subtract for all components.</param>
    /// <returns>A copy of this vector after computing the difference of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d SubtractInPlace(Fixed64 amount) => SubtractInPlace(amount, amount);

    /// <summary>
    /// Subtracts the specified vector from the components of the vector in place and returns the modified vector.
    /// </summary>
    /// <param name="other">The per-component values to subtract.</param>
    /// <returns>A copy of this vector after computing the difference of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d SubtractInPlace(Vector2d other) => SubtractInPlace(other.X, other.Y);

    /// <summary>
    /// Multiplies the components of the vector by the specified x and y factors in place and returns the modified vector.
    /// </summary>
    /// <param name="factorX">The amount to multiply by for the X component.</param>
    /// <param name="factorY">The amount to multiply by for the Y component.</param>
    /// <returns>A copy of this vector after computing the product of its corresponding components. Each result saturates at the representable endpoints.</returns>
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
    /// <param name="factor">The scalar to multiply by for all components.</param>
    /// <returns>A copy of this vector after computing the product of its components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d MultiplyInPlace(Fixed64 factor) => MultiplyInPlace(factor, factor);

    /// <summary>
    /// Multiplies each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
    /// </summary>
    /// <param name="factor">The per-component values to multiply by.</param>
    /// <returns>A copy of this vector after computing the product of its corresponding components. Each result saturates at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d MultiplyInPlace(Vector2d factor) => MultiplyInPlace(factor.X, factor.Y);

    /// <summary>
    /// Divides the components of the vector by the specified x and y divisors in place and returns the modified vector.
    /// </summary>
    /// <param name="divisorX">The amount to divide by for the X component.</param>
    /// <param name="divisorY">The amount to divide by for the Y component.</param>
    /// <returns>A copy of this vector after computing the quotient of its corresponding components. Each result saturates at the representable endpoints.</returns>
    /// <remarks>The method updates components in X, Y order. On failure, the earlier components can already be updated.</remarks>
    /// <exception cref="DivideByZeroException">A divisor is zero.</exception>
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
    /// <param name="divisor">The scalar to divide by for all components.</param>
    /// <returns>A copy of this vector after computing the quotient of its components. Each result saturates at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">A divisor is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d DivideInPlace(Fixed64 divisor) => DivideInPlace(divisor, divisor);

    /// <summary>
    /// Divides each component of the vector by the corresponding component of the given vector in place and returns the modified vector.
    /// </summary>
    /// <param name="divisor">The per-component values to divide by.</param>
    /// <returns>A copy of this vector after computing the quotient of its corresponding components. Each result saturates at the representable endpoints.</returns>
    /// <remarks>The method updates components in X, Y order. On failure, the earlier components can already be updated.</remarks>
    /// <exception cref="DivideByZeroException">A divisor is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d DivideInPlace(Vector2d divisor) => DivideInPlace(divisor.X, divisor.Y);

    /// <summary>Normalizes this vector in place.</summary>
    /// <returns>A copy of this vector after normalization. The zero vector stays zero.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d NormalizeInPlace() => this = GetNormalized(this);

    /// <summary>Normalizes this vector in place.</summary>
    /// <param name="mag">The computed original length. It reports one inside the normalization tolerance and saturates to <see cref="Fixed64.MaxValue"/> if necessary.</param>
    /// <returns>A copy of this vector after normalization. The zero vector stays zero.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d NormalizeInPlace(out Fixed64 mag)
    {
        Vector2d source = this;
        bool magnitudeIsRepresentable = TryGetMagnitude(
            source,
            out mag,
            out bool isNormalized);

        // If magnitude is zero, return a zero vector to avoid divide-by-zero errors
        if (mag == Fixed64.Zero)
        {
            X = Fixed64.Zero;
            Y = Fixed64.Zero;
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

        return IsNormalized()
            ? this
            : this = WideNormalization.GetNormalized(source);
    }

    /// <summary>
    /// Checks whether this nonzero vector's squared magnitude is within epsilon of one.
    /// </summary>
    /// <returns><see langword="true"/> if the vector is nonzero and its squared length differs from one by at most <see cref="Fixed64.Epsilon"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool IsNormalized()
    {
        Fixed64 squaredMagnitude = X * X + Y * Y;
        return squaredMagnitude != Fixed64.Zero
            && FixedMath.Abs(squaredMagnitude - Fixed64.One) <= Fixed64.Epsilon;
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
            Fixed64.Zero,
            out magnitude);

    /// <summary>
    /// Linearly interpolates this vector toward the target vector by the specified amount.
    /// </summary>
    /// <param name="target">The target point in the same coordinate space.</param>
    /// <param name="amount">The interpolation fraction, clamped to [0, 1].</param>
    /// <returns>A copy of this vector after interpolation.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d LerpInPlace(Vector2d target, Fixed64 amount)
    {
        LerpInPlace(target.X, target.Y, amount);
        return this;
    }

    /// <summary>
    /// Linearly interpolates this vector toward the target values by the specified amount.
    /// </summary>
    /// <param name="targetx">The target X coordinate.</param>
    /// <param name="targety">The target Y coordinate.</param>
    /// <param name="amount">The interpolation fraction, clamped to [0, 1].</param>
    /// <returns>A copy of this vector after interpolation.</returns>
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
            X = FixedMath.Lerp(X, targetx, amount);
            Y = FixedMath.Lerp(Y, targety, amount);
        }
        return this;
    }

    /// <summary>
    /// Returns a new vector that is the result of linear interpolation toward the target by the specified amount.
    /// </summary>
    /// <param name="target">The target point in the same coordinate space.</param>
    /// <param name="amount">The interpolation fraction, clamped to [0, 1].</param>
    /// <returns>An interpolated copy.</returns>
    public Vector2d Lerp(Vector2d target, Fixed64 amount)
    {
        Vector2d vec = this;
        vec.LerpInPlace(target.X, target.Y, amount);
        return vec;
    }

    /// <summary>
    /// Rotates this vector by the specified cosine and sine values (counter-clockwise).
    /// </summary>
    /// <param name="cos">The cosine of the angle.</param>
    /// <param name="sin">The sine of the same angle. Positive angles rotate from +X toward +Y.</param>
    /// <returns>A copy of this vector after rotation.</returns>
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
    /// <param name="cos">The cosine of the angle.</param>
    /// <param name="sin">The sine of the same angle. Positive angles rotate from +X toward +Y.</param>
    /// <returns>A rotated copy of this vector.</returns>
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
    /// <returns>A copy of the updated vector, <c>(Y, −X)</c> in terms of the original components.</returns>
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
    /// <returns>A copy of the updated vector, <c>(−Y, X)</c> in terms of the original components.</returns>
    public Vector2d RotateLeftInPlace()
    {
        Fixed64 temp1 = X;
        X = -Y;
        Y = temp1;
        return this;
    }

    /// <summary>Reflects this vector in place across a line through the origin.</summary>
    /// <param name="axis">The unit direction along the line of reflection.</param>
    /// <returns>A copy of this vector after reflection.</returns>
    public Vector2d ReflectInPlace(Vector2d axis) => ReflectInPlace(axis.X, axis.Y);

    /// <summary>Reflects this vector in place across a line through the origin.</summary>
    /// <param name="axisX">The X component of the unit reflection axis.</param>
    /// <param name="axisY">The Y component of the unit reflection axis.</param>
    /// <returns>A copy of this vector after reflection.</returns>
    public Vector2d ReflectInPlace(Fixed64 axisX, Fixed64 axisY)
    {
        Fixed64 projection = Dot(axisX, axisY);
        return ReflectInPlace(axisX, axisY, projection);
    }

    /// <summary>
    /// Reflects this vector across the specified axis using the provided projection of this vector onto the axis.
    /// </summary>
    /// <param name="axisX">The X component of the unit reflection axis.</param>
    /// <param name="axisY">The Y component of the unit reflection axis.</param>
    /// <param name="projection">The dot product of the original vector and the unit reflection axis.</param>
    /// <returns>A copy of this vector after reflection.</returns>
    public Vector2d ReflectInPlace(Fixed64 axisX, Fixed64 axisY, Fixed64 projection)
    {
        Fixed64 temp1 = axisX * projection;
        Fixed64 temp2 = axisY * projection;
        X = temp1 + temp1 - X;
        Y = temp2 + temp2 - Y;
        return this;
    }

    /// <summary>Returns a copy reflected across a line through the origin.</summary>
    /// <param name="axisX">The X component of the unit reflection axis.</param>
    /// <param name="axisY">The Y component of the unit reflection axis.</param>
    /// <returns>A reflected copy of this vector.</returns>
    public Vector2d Reflected(Fixed64 axisX, Fixed64 axisY)
    {
        Vector2d vec = this;
        vec.ReflectInPlace(axisX, axisY);
        return vec;
    }

    /// <summary>Returns a copy reflected across a line through the origin.</summary>
    /// <param name="axis">The unit direction along the line of reflection.</param>
    /// <returns>A reflected copy of this vector.</returns>
    public Vector2d Reflected(Vector2d axis) => Reflected(axis.X, axis.Y);

    /// <summary>
    /// Returns the dot product of this vector with another vector.
    /// </summary>
    /// <param name="otherX">The X component in the same coordinate space as this vector.</param>
    /// <param name="otherY">The Y component in the same coordinate space as this vector.</param>
    /// <returns>The sum of the component products using rounded, saturating fixed-point operations.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Dot(Fixed64 otherX, Fixed64 otherY) => X * otherX + Y * otherY;

    /// <summary>
    /// Returns the dot product of this vector with another vector.
    /// </summary>
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>The sum of the component products using rounded, saturating fixed-point operations.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Dot(Vector2d other) => Dot(other.X, other.Y);

    /// <summary>Computes the signed 2D cross product.</summary>
    /// <param name="otherX">The X component of the other vector.</param>
    /// <param name="otherY">The Y component of the other vector.</param>
    /// <returns>The signed determinant. A negative value denotes a clockwise turn and a positive value denotes a counterclockwise turn for representable calculations.</returns>
    /// <remarks>The determinant uses rounded, saturating fixed-point products and subtraction.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 CrossProduct(Fixed64 otherX, Fixed64 otherY) => X * otherY - Y * otherX;

    /// <summary>Computes the signed 2D cross product.</summary>
    /// <param name="other">The other vector.</param>
    /// <returns>The signed determinant. A negative value denotes a clockwise turn and a positive value denotes a counterclockwise turn for representable calculations.</returns>
    /// <remarks>The determinant uses rounded, saturating fixed-point products and subtraction.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 CrossProduct(Vector2d other) => CrossProduct(other.X, other.Y);

    /// <summary>
    /// Returns the distance between this vector and another vector specified by its components.
    /// </summary>
    /// <param name="otherX">The X component in the same coordinate space as this vector.</param>
    /// <param name="otherY">The Y component in the same coordinate space as this vector.</param>
    /// <returns>The Euclidean distance in the same units as the coordinates, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Distance(Fixed64 otherX, Fixed64 otherY)
    {
        Fixed64 deltaX = X - otherX;
        Fixed64 deltaY = Y - otherY;
        Fixed64 squareSum = deltaX * deltaX + deltaY * deltaY;
        return squareSum == Fixed64.MaxValue
            ? FixedMath.GetScaledMagnitude(deltaX, deltaY, Fixed64.Zero, Fixed64.Zero)
            : FixedMath.Sqrt(squareSum);
    }

    /// <summary>
    /// Returns the distance between this vector and another vector.
    /// </summary>
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>The Euclidean distance in the same units as the coordinates, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 Distance(Vector2d other) => Distance(other.X, other.Y);

    /// <summary>
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <param name="otherX">The X component in the same coordinate space as this vector.</param>
    /// <param name="otherY">The Y component in the same coordinate space as this vector.</param>
    /// <returns>The squared Euclidean distance in squared coordinate units, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
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
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>The squared Euclidean distance in squared coordinate units, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector2d other) => DistanceSquared(other.X, other.Y);

    #endregion
}
