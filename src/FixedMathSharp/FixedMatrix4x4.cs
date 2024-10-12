using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a 4x4 matrix using fixed-point arithmetic.
    /// </summary>
    [Serializable]
    public struct FixedMatrix4x4 : IEquatable<FixedMatrix4x4>
    {
        public Fixed64 m00, m01, m02, m03;
        public Fixed64 m10, m11, m12, m13;
        public Fixed64 m20, m21, m22, m23;
        public Fixed64 m30, m31, m32, m33;

        /// <summary>
        /// Initializes a new FixedMatrix4x4 with individual elements.
        /// </summary>
        public FixedMatrix4x4(
            Fixed64 m00, Fixed64 m01, Fixed64 m02, Fixed64 m03,
            Fixed64 m10, Fixed64 m11, Fixed64 m12, Fixed64 m13,
            Fixed64 m20, Fixed64 m21, Fixed64 m22, Fixed64 m23,
            Fixed64 m30, Fixed64 m31, Fixed64 m32, Fixed64 m33
        )
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02; this.m03 = m03;
            this.m10 = m10; this.m11 = m11; this.m12 = m12; this.m13 = m13;
            this.m20 = m20; this.m21 = m21; this.m22 = m22; this.m23 = m23;
            this.m30 = m30; this.m31 = m31; this.m32 = m32; this.m33 = m33;
        }

        /// <summary>
        /// Returns the identity matrix (diagonal elements set to 1).
        /// </summary>
        private static readonly FixedMatrix4x4 identityMatrix = new FixedMatrix4x4(
            FixedMath.One, FixedMath.Zero, FixedMath.Zero, FixedMath.Zero,
            FixedMath.Zero, FixedMath.One, FixedMath.Zero, FixedMath.Zero,
            FixedMath.Zero, FixedMath.Zero, FixedMath.One, FixedMath.Zero,
            FixedMath.Zero, FixedMath.Zero, FixedMath.Zero, FixedMath.One);
        public static FixedMatrix4x4 Identity => identityMatrix;

        /// <summary>
        /// Creates a 4x4 transformation matrix from translation, rotation, and scale.
        /// </summary>
        /// <param name="translation">Translation vector.</param>
        /// <param name="rotation">Rotation as a quaternion.</param>
        /// <param name="scale">Scale vector.</param>
        /// <returns>A combined transformation matrix.</returns>
        public static FixedMatrix4x4 TRS(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
        {
            // Create translation matrix
            FixedMatrix4x4 translationMatrix = new FixedMatrix4x4(
                FixedMath.One, FixedMath.Zero, FixedMath.Zero, translation.x,
                FixedMath.Zero, FixedMath.One, FixedMath.Zero, translation.y,
                FixedMath.Zero, FixedMath.Zero, FixedMath.One, translation.z,
                FixedMath.Zero, FixedMath.Zero, FixedMath.Zero, FixedMath.One);

            // Create rotation matrix using the quaternion
            Fixed64 xx = rotation.x * rotation.x;
            Fixed64 xy = rotation.x * rotation.y;
            Fixed64 xz = rotation.x * rotation.z;
            Fixed64 xw = rotation.x * rotation.w;

            Fixed64 yy = rotation.y * rotation.y;
            Fixed64 yz = rotation.y * rotation.z;
            Fixed64 yw = rotation.y * rotation.w;

            Fixed64 zz = rotation.z * rotation.z;
            Fixed64 zw = rotation.z * rotation.w;

            FixedMatrix4x4 rotationMatrix = new FixedMatrix4x4(
                FixedMath.One - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw), FixedMath.Zero,
                2 * (xy + zw), FixedMath.One - 2 * (xx + zz), 2 * (yz - xw), FixedMath.Zero,
                2 * (xz - yw), 2 * (yz + xw), FixedMath.One - 2 * (xx + yy), FixedMath.Zero,
                FixedMath.Zero, FixedMath.Zero, FixedMath.Zero, FixedMath.One);

            // Create scaling matrix
            FixedMatrix4x4 scalingMatrix = new FixedMatrix4x4(
                scale.x, FixedMath.Zero, FixedMath.Zero, FixedMath.Zero,
                FixedMath.Zero, scale.y, FixedMath.Zero, FixedMath.Zero,
                FixedMath.Zero, FixedMath.Zero, scale.z, FixedMath.Zero,
                FixedMath.Zero, FixedMath.Zero, FixedMath.Zero, FixedMath.One);

            // Combine all transformations
            return translationMatrix * rotationMatrix * scalingMatrix;
        }

        /// <summary>
        /// Calculates the determinant of a 4x4 matrix.
        /// </summary>
        public Fixed64 GetDeterminant()
        {
            // Calculating the determinant of a 4x4 matrix
            return
                m00 * (m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32
                     - m13 * m22 * m31 - m11 * m23 * m32 - m12 * m21 * m33)
               - m01 * (m10 * m22 * m33 + m12 * m23 * m30 + m13 * m20 * m32
                     - m13 * m22 * m30 - m10 * m23 * m32 - m12 * m20 * m33)
               + m02 * (m10 * m21 * m33 + m11 * m23 * m30 + m13 * m20 * m31
                     - m13 * m21 * m30 - m10 * m23 * m31 - m11 * m20 * m33)
               - m03 * (m10 * m21 * m32 + m11 * m22 * m30 + m12 * m20 * m31
                     - m12 * m21 * m30 - m10 * m22 * m31 - m11 * m20 * m32);
        }

        /// <summary>
        /// Inverts the matrix if it is invertible (i.e., if the determinant is not zero).
        /// </summary>
        /// <remarks>
        /// To Invert a FixedMatrix4x4, we need to calculate the inverse for each element. 
        /// This involves computing the cofactor for each element, 
        /// which is the determinant of the submatrix when the row and column of that element are removed, 
        /// multiplied by a sign based on the element's position. 
        /// After computing all cofactors, the result is transposed to get the inverse matrix.
        /// </remarks>
        public static bool Invert(FixedMatrix4x4 matrix, out FixedMatrix4x4? result)
        {
            Fixed64 det = matrix.GetDeterminant();

            if (det == FixedMath.Zero)
            {
                result = null;
                return false;
            }

            Fixed64 invDet = FixedMath.One / det;

            // Inversion using cofactors and determinants of 3x3 submatrices
            result = new FixedMatrix4x4
            {
                // First row
                m00 = invDet * ((matrix.m11 * matrix.m22 * matrix.m33 + matrix.m12 * matrix.m23 * matrix.m31 + matrix.m13 * matrix.m21 * matrix.m32)
                              - (matrix.m13 * matrix.m22 * matrix.m31 + matrix.m11 * matrix.m23 * matrix.m32 + matrix.m12 * matrix.m21 * matrix.m33)),
                m01 = invDet * ((matrix.m01 * matrix.m23 * matrix.m32 + matrix.m02 * matrix.m21 * matrix.m33 + matrix.m03 * matrix.m22 * matrix.m31)
                              - (matrix.m03 * matrix.m21 * matrix.m32 + matrix.m01 * matrix.m22 * matrix.m33 + matrix.m02 * matrix.m23 * matrix.m31)),
                m02 = invDet * ((matrix.m01 * matrix.m12 * matrix.m33 + matrix.m02 * matrix.m13 * matrix.m31 + matrix.m03 * matrix.m11 * matrix.m32)
                              - (matrix.m03 * matrix.m12 * matrix.m31 + matrix.m01 * matrix.m13 * matrix.m32 + matrix.m02 * matrix.m11 * matrix.m33)),
                m03 = invDet * ((matrix.m01 * matrix.m13 * matrix.m22 + matrix.m02 * matrix.m11 * matrix.m23 + matrix.m03 * matrix.m12 * matrix.m21)
                              - (matrix.m03 * matrix.m11 * matrix.m22 + matrix.m01 * matrix.m12 * matrix.m23 + matrix.m02 * matrix.m13 * matrix.m21)),

                // Second row
                m10 = invDet * ((matrix.m10 * matrix.m23 * matrix.m32 + matrix.m12 * matrix.m20 * matrix.m33 + matrix.m13 * matrix.m22 * matrix.m30)
                              - (matrix.m13 * matrix.m20 * matrix.m32 + matrix.m10 * matrix.m22 * matrix.m33 + matrix.m12 * matrix.m23 * matrix.m30)),
                m11 = invDet * ((matrix.m00 * matrix.m22 * matrix.m33 + matrix.m02 * matrix.m23 * matrix.m30 + matrix.m03 * matrix.m20 * matrix.m32)
                              - (matrix.m03 * matrix.m20 * matrix.m32 + matrix.m00 * matrix.m23 * matrix.m32 + matrix.m02 * matrix.m20 * matrix.m33)),
                m12 = invDet * ((matrix.m00 * matrix.m13 * matrix.m32 + matrix.m02 * matrix.m10 * matrix.m33 + matrix.m03 * matrix.m12 * matrix.m30)
                              - (matrix.m03 * matrix.m10 * matrix.m32 + matrix.m00 * matrix.m12 * matrix.m33 + matrix.m02 * matrix.m13 * matrix.m30)),
                m13 = invDet * ((matrix.m00 * matrix.m12 * matrix.m23 + matrix.m02 * matrix.m13 * matrix.m20 + matrix.m03 * matrix.m10 * matrix.m22)
                              - (matrix.m03 * matrix.m10 * matrix.m22 + matrix.m00 * matrix.m13 * matrix.m22 + matrix.m02 * matrix.m12 * matrix.m20)),

                // Third row
                m20 = invDet * ((matrix.m10 * matrix.m21 * matrix.m33 + matrix.m11 * matrix.m23 * matrix.m30 + matrix.m13 * matrix.m20 * matrix.m31)
                              - (matrix.m13 * matrix.m20 * matrix.m31 + matrix.m10 * matrix.m23 * matrix.m31 + matrix.m11 * matrix.m20 * matrix.m33)),
                m21 = invDet * ((matrix.m00 * matrix.m23 * matrix.m31 + matrix.m01 * matrix.m20 * matrix.m33 + matrix.m03 * matrix.m21 * matrix.m30)
                              - (matrix.m03 * matrix.m20 * matrix.m31 + matrix.m00 * matrix.m21 * matrix.m33 + matrix.m01 * matrix.m23 * matrix.m30)),
                m22 = invDet * ((matrix.m00 * matrix.m11 * matrix.m33 + matrix.m01 * matrix.m13 * matrix.m30 + matrix.m03 * matrix.m10 * matrix.m31)
                              - (matrix.m03 * matrix.m10 * matrix.m31 + matrix.m00 * matrix.m13 * matrix.m31 + matrix.m01 * matrix.m10 * matrix.m33)),
                m23 = invDet * ((matrix.m00 * matrix.m13 * matrix.m21 + matrix.m01 * matrix.m10 * matrix.m23 + matrix.m03 * matrix.m11 * matrix.m20)
                              - (matrix.m03 * matrix.m10 * matrix.m21 + matrix.m00 * matrix.m11 * matrix.m23 + matrix.m01 * matrix.m13 * matrix.m20)),

                // Fourth row
                m30 = invDet * ((matrix.m10 * matrix.m22 * matrix.m31 + matrix.m11 * matrix.m20 * matrix.m32 + matrix.m12 * matrix.m21 * matrix.m30)
                              - (matrix.m12 * matrix.m20 * matrix.m31 + matrix.m10 * matrix.m21 * matrix.m32 + matrix.m11 * matrix.m22 * matrix.m30)),
                m31 = invDet * ((matrix.m00 * matrix.m21 * matrix.m32 + matrix.m01 * matrix.m22 * matrix.m30 + matrix.m02 * matrix.m20 * matrix.m31)
                              - (matrix.m02 * matrix.m20 * matrix.m31 + matrix.m00 * matrix.m22 * matrix.m31 + matrix.m01 * matrix.m20 * matrix.m32)),
                m32 = invDet * ((matrix.m00 * matrix.m12 * matrix.m31 + matrix.m01 * matrix.m10 * matrix.m32 + matrix.m02 * matrix.m11 * matrix.m30)
                              - (matrix.m02 * matrix.m10 * matrix.m31 + matrix.m00 * matrix.m11 * matrix.m32 + matrix.m01 * matrix.m12 * matrix.m30)),
                m33 = invDet * ((matrix.m00 * matrix.m11 * matrix.m22 + matrix.m01 * matrix.m12 * matrix.m20 + matrix.m02 * matrix.m10 * matrix.m21)
                              - (matrix.m02 * matrix.m10 * matrix.m21 + matrix.m00 * matrix.m12 * matrix.m21 + matrix.m01 * matrix.m11 * matrix.m20)),
            };

            return true;
        }

        #region Operators

        /// <summary>
        /// Adds two matrices element-wise.
        /// </summary>
        public static FixedMatrix4x4 operator +(FixedMatrix4x4 lhs, FixedMatrix4x4 rhs)
        {
            return new FixedMatrix4x4(
                lhs.m00 + rhs.m00, lhs.m01 + rhs.m01, lhs.m02 + rhs.m02, lhs.m03 + rhs.m03,
                lhs.m10 + rhs.m10, lhs.m11 + rhs.m11, lhs.m12 + rhs.m12, lhs.m13 + rhs.m13,
                lhs.m20 + rhs.m20, lhs.m21 + rhs.m21, lhs.m22 + rhs.m22, lhs.m23 + rhs.m23,
                lhs.m30 + rhs.m30, lhs.m31 + rhs.m31, lhs.m32 + rhs.m32, lhs.m33 + rhs.m33);
        }

        /// <summary>
        /// Subtracts two matrices element-wise.
        /// </summary>
        public static FixedMatrix4x4 operator -(FixedMatrix4x4 lhs, FixedMatrix4x4 rhs)
        {
            return new FixedMatrix4x4(
                lhs.m00 - rhs.m00, lhs.m01 - rhs.m01, lhs.m02 - rhs.m02, lhs.m03 - rhs.m03,
                lhs.m10 - rhs.m10, lhs.m11 - rhs.m11, lhs.m12 - rhs.m12, lhs.m13 - rhs.m13,
                lhs.m20 - rhs.m20, lhs.m21 - rhs.m21, lhs.m22 - rhs.m22, lhs.m23 - rhs.m23,
                lhs.m30 - rhs.m30, lhs.m31 - rhs.m31, lhs.m32 - rhs.m32, lhs.m33 - rhs.m33);
        }

        /// <summary>
        /// Multiplies two matrices using matrix multiplication rules.
        /// </summary>
        public static FixedMatrix4x4 operator *(FixedMatrix4x4 lhs, FixedMatrix4x4 rhs)
        {
            return new FixedMatrix4x4(
                lhs.m00 * rhs.m00 + lhs.m01 * rhs.m10 + lhs.m02 * rhs.m20 + lhs.m03 * rhs.m30,
                lhs.m00 * rhs.m01 + lhs.m01 * rhs.m11 + lhs.m02 * rhs.m21 + lhs.m03 * rhs.m31,
                lhs.m00 * rhs.m02 + lhs.m01 * rhs.m12 + lhs.m02 * rhs.m22 + lhs.m03 * rhs.m32,
                lhs.m00 * rhs.m03 + lhs.m01 * rhs.m13 + lhs.m02 * rhs.m23 + lhs.m03 * rhs.m33,
                lhs.m10 * rhs.m00 + lhs.m11 * rhs.m10 + lhs.m12 * rhs.m20 + lhs.m13 * rhs.m30,
                lhs.m10 * rhs.m01 + lhs.m11 * rhs.m11 + lhs.m12 * rhs.m21 + lhs.m13 * rhs.m31,
                lhs.m10 * rhs.m02 + lhs.m11 * rhs.m12 + lhs.m12 * rhs.m22 + lhs.m13 * rhs.m32,
                lhs.m10 * rhs.m03 + lhs.m11 * rhs.m13 + lhs.m12 * rhs.m23 + lhs.m13 * rhs.m33,
                lhs.m20 * rhs.m00 + lhs.m21 * rhs.m10 + lhs.m22 * rhs.m20 + lhs.m23 * rhs.m30,
                lhs.m20 * rhs.m01 + lhs.m21 * rhs.m11 + lhs.m22 * rhs.m21 + lhs.m23 * rhs.m31,
                lhs.m20 * rhs.m02 + lhs.m21 * rhs.m12 + lhs.m22 * rhs.m22 + lhs.m23 * rhs.m32,
                lhs.m20 * rhs.m03 + lhs.m21 * rhs.m13 + lhs.m22 * rhs.m23 + lhs.m23 * rhs.m33,
                lhs.m30 * rhs.m00 + lhs.m31 * rhs.m10 + lhs.m32 * rhs.m20 + lhs.m33 * rhs.m30,
                lhs.m30 * rhs.m01 + lhs.m31 * rhs.m11 + lhs.m32 * rhs.m21 + lhs.m33 * rhs.m31,
                lhs.m30 * rhs.m02 + lhs.m31 * rhs.m12 + lhs.m32 * rhs.m22 + lhs.m33 * rhs.m32,
                lhs.m30 * rhs.m03 + lhs.m31 * rhs.m13 + lhs.m32 * rhs.m23 + lhs.m33 * rhs.m33);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(FixedMatrix4x4 left, FixedMatrix4x4 right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(FixedMatrix4x4 left, FixedMatrix4x4 right)
        {
            return !(left == right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is FixedMatrix4x4 x && Equals(x);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(FixedMatrix4x4 other)
        {
            return m00 == other.m00 && m01 == other.m01 && m02 == other.m02 && m03 == other.m03 &&
                   m10 == other.m10 && m11 == other.m11 && m12 == other.m12 && m13 == other.m13 &&
                   m20 == other.m20 && m21 == other.m21 && m22 == other.m22 && m23 == other.m23 &&
                   m30 == other.m30 && m31 == other.m31 && m32 == other.m32 && m33 == other.m33;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + m00.GetHashCode();
                hash = hash * 23 + m01.GetHashCode();
                hash = hash * 23 + m02.GetHashCode();
                hash = hash * 23 + m10.GetHashCode();
                hash = hash * 23 + m11.GetHashCode();
                hash = hash * 23 + m12.GetHashCode();
                hash = hash * 23 + m20.GetHashCode();
                hash = hash * 23 + m21.GetHashCode();
                hash = hash * 23 + m22.GetHashCode();
                hash = hash * 23 + m23.GetHashCode();
                hash = hash * 23 + m30.GetHashCode();
                hash = hash * 23 + m31.GetHashCode();
                hash = hash * 23 + m32.GetHashCode();
                hash = hash * 23 + m33.GetHashCode();
                return hash;
            }
        }

        #endregion
    }
}