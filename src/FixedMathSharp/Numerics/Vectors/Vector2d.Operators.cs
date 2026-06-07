//=======================================================================
// Vector2d.Operators.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Vector2d
{
    #region Operators

    /// <summary>
    /// Adds two Vector2d instances component-wise.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">The second vector to add.</param>
    /// <returns>A new Vector2d whose components are the sums of the corresponding components of v1 and v2.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, Vector2d v2) => new(v1.X + v2.X, v1.Y + v2.Y);

    /// <summary>
    /// Adds a scalar value to each component of the specified vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector to which the scalar value will be added.</param>
    /// <param name="mag">The scalar value to add to each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the sum of the corresponding components of the input vector and the scalar value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, Fixed64 mag) => new(v1.X + mag, v1.Y + mag);

    /// <inheritdoc cref="operator +(Vector2d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Fixed64 mag, Vector2d v1) => v1 + mag;

    /// <summary>
    /// Adds a Vector2d instance and a tuple representing X and Y components, returning a new Vector2d with the summed
    /// values.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">A tuple containing the X and Y values to add to the vector.</param>
    /// <returns>A new Vector2d whose X and Y components are the sums of the corresponding components of the input vector and tuple.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +(Vector2d v1, (int x, int y) v2) => new(v1.X + v2.x, v1.Y + v2.y);

    /// <summary>
    /// Adds a tuple representing X and Y components and a Vector2d instance, returning a new Vector2d with the summed
    /// values.
    /// </summary>
    /// <param name="v2">A tuple containing the X and Y values to add to the vector.</param>
    /// <param name="v1">The vector to add.</param>
    /// <returns>A new Vector2d whose X and Y components are the sums of the corresponding components of the tuple and input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator +((int x, int y) v2, Vector2d v1) => v1 + v2;

    /// <summary>
    /// Subtracts the components of one Vector2d from another and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector to subtract from.</param>
    /// <param name="v2">The vector to subtract.</param>
    /// <returns>A Vector2d whose components are the result of subtracting the corresponding components of v2 from v1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, Vector2d v2) => new(v1.X - v2.X, v1.Y - v2.Y);

    /// <summary>
    /// Subtracts the specified scalar value from both components of the given vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector from which to subtract the scalar value.</param>
    /// <param name="mag">The scalar value to subtract from each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the result of subtracting the scalar value from the corresponding components
    /// of the input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, Fixed64 mag) => new(v1.X - mag, v1.Y - mag);

    /// <inheritdoc cref="operator -(Vector2d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Fixed64 mag, Vector2d v1) => new(mag - v1.X, mag - v1.Y);

    /// <summary>
    /// Subtracts the specified tuple from the given vector and returns the resulting vector.
    /// </summary>
    /// <param name="v1">The vector from which to subtract the tuple values.</param>
    /// <param name="v2">A tuple containing the x and y values to subtract from the vector.</param>
    /// <returns>A new Vector2d representing the result of subtracting the tuple values from the original vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1, (int x, int y) v2) => new(v1.X - v2.x, v1.Y - v2.y);

    /// <summary>
    /// Subtracts the specified Vector2d from the given integer tuple and returns the resulting vector.
    /// </summary>
    /// <param name="v1">A tuple containing the x and y components to subtract from.</param>
    /// <param name="v2">The vector whose components are subtracted from the tuple.</param>
    /// <returns>A Vector2d representing the result of subtracting the components of v2 from v1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -((int x, int y) v1, Vector2d v2) => new(v1.x - v2.X, v1.y - v2.Y);

    /// <summary>
    /// Negates the specified vector by reversing the sign of each of its components.
    /// </summary>
    /// <param name="v1">The vector to negate.</param>
    /// <returns>A new Vector2d whose components are the negated values of the input vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator -(Vector2d v1) => new(v1.X * -Fixed64.One, v1.Y * -Fixed64.One);

    /// <summary>
    /// Scales the specified vector by the given scalar value.
    /// </summary>
    /// <param name="v1">The vector to be scaled.</param>
    /// <param name="mag">The scalar value by which to multiply each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the components of v1 multiplied by mag.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator *(Vector2d v1, Fixed64 mag) => new(v1.X * mag, v1.Y * mag);

    /// <inheritdoc cref="operator *(Vector2d, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator *(Fixed64 mag, Vector2d v1) => v1 * mag;

    /// <summary>
    /// Multiplies two vectors element-wise and returns the resulting vector.
    /// </summary>
    /// <remarks>Element-wise multiplication multiplies each component of the first vector by the
    /// corresponding component of the second vector. This operation is not a dot product or cross product.</remarks>
    /// <param name="v1">The first vector to multiply.</param>
    /// <param name="v2">The second vector to multiply.</param>
    /// <returns>A new Vector2d whose components are the products of the corresponding components of the input vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator *(Vector2d v1, Vector2d v2) => new(v1.X * v2.X, v1.Y * v2.Y);

    /// <summary>
    /// Divides each component of a specified vector by a scalar value.
    /// </summary>
    /// <param name="v1">The vector whose components are to be divided.</param>
    /// <param name="div">The scalar value by which to divide each component of the vector.</param>
    /// <returns>A new Vector2d whose components are the result of dividing the corresponding components of v1 by div.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d operator /(Vector2d v1, Fixed64 div) => new(v1.X / div, v1.Y / div);

    /// <summary>
    /// Determines whether two Vector2d instances are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Vector2d left, Vector2d right) => left.Equals(right);

    /// <summary>
    /// Determines whether two Vector2d instances are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Vector2d left, Vector2d right) => !left.Equals(right);

    #endregion
}
