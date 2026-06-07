//=======================================================================
// Vector3d.Operators.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Vector3d
{
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
}
