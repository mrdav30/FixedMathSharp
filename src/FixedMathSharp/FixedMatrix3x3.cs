using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a 3x3 matrix with fixed-point arithmetic.
    /// Commonly used for linear transformations such as rotation, scaling, and shearing in 3D space.
    /// </summary>
    public struct FixedMatrix3x3 : IEquatable<FixedMatrix3x3>
    {
        public Fixed64 m00, m01, m02;
        public Fixed64 m10, m11, m12;
        public Fixed64 m20, m21, m22;

        /// <summary>
        /// Initializes a new FixedMatrix3x3 with the specified elements.
        /// </summary>
        public FixedMatrix3x3(
            Fixed64 m00, Fixed64 m01, Fixed64 m02,
            Fixed64 m10, Fixed64 m11, Fixed64 m12,
            Fixed64 m20, Fixed64 m21, Fixed64 m22
        )
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }

        /// <summary>
        /// Initializes a new FixedMatrix3x3 using three Vector3d values representing the rows.
        /// </summary>
        public FixedMatrix3x3(
            Vector3d m00_m01_m02,
            Vector3d m10_m11_m12,
            Vector3d m20_m21_m22
        )
        {
            m00 = m00_m01_m02.x; m01 = m00_m01_m02.y; m02 = m00_m01_m02.z;
            m10 = m10_m11_m12.x; m11 = m10_m11_m12.y; m12 = m10_m11_m12.z;
            m20 = m20_m21_m22.x; m21 = m20_m21_m22.y; m22 = m20_m21_m22.z;
        }

        /// <summary>
        /// Returns the identity matrix (no scaling, rotation, or translation).
        /// </summary>
        private static readonly FixedMatrix3x3 identityMatrix = new FixedMatrix3x3(new Vector3d(1f, 0f, 0f), new Vector3d(0f, 1f, 0f), new Vector3d(0f, 0f, 1f));
        public static FixedMatrix3x3 Identity => identityMatrix;

        /// <summary>
        /// Returns a matrix with all elements set to zero.
        /// </summary>
        private static readonly FixedMatrix3x3 zeroMatrix = new FixedMatrix3x3(new Vector3d(0f, 0f, 0f), new Vector3d(0f, 0f, 0f), new Vector3d(0f, 0f, 0f));
        public static FixedMatrix3x3 Zero => zeroMatrix;


        public Fixed64 GetDeterminant()
        {
            return m00 * (m11 * m22 - m21 * m12) -
                   m01 * (m10 * m22 - m20 * m12) +
                   m02 * (m10 * m21 - m20 * m11);
        }

        /// <summary>
        /// Attempts to invert the matrix. If the determinant is zero, returns false and sets result to null.
        /// </summary>
        public static bool Invert(FixedMatrix3x3 matrix, out FixedMatrix3x3? result)
        {
            // Calculate the determinant
            Fixed64 det = matrix.GetDeterminant();

            if (det == FixedMath.Zero)
            {
                result = null;
                return false;
            }

            // Calculate the inverse
            Fixed64 invDet = FixedMath.One / det;
            result = new FixedMatrix3x3(
                (matrix.m11 * matrix.m22 - matrix.m12 * matrix.m21) * invDet,
                (matrix.m02 * matrix.m21 - matrix.m01 * matrix.m22) * invDet,
                (matrix.m01 * matrix.m12 - matrix.m02 * matrix.m11) * invDet,

                (matrix.m12 * matrix.m20 - matrix.m10 * matrix.m22) * invDet,
                (matrix.m00 * matrix.m22 - matrix.m02 * matrix.m20) * invDet,
                (matrix.m02 * matrix.m10 - matrix.m00 * matrix.m12) * invDet,

                (matrix.m10 * matrix.m21 - matrix.m11 * matrix.m20) * invDet,
                (matrix.m01 * matrix.m20 - matrix.m00 * matrix.m21) * invDet,
                (matrix.m00 * matrix.m11 - matrix.m01 * matrix.m10) * invDet
            );
            return true;
        }

        /// <summary>
        /// Inverts the diagonal elements of the matrix.
        /// </summary>
        /// <remarks>
        /// protects against the case where you would have an infinite value on the diagonal, which would cause problems in subsequent computations.
        /// If m00 or m22 are zero, handle that as a special case and manually set the inverse to zero,
        /// since for a theoretical object with no inertia along those axes, it would be impossible to impart a rotation in those directions
        ///
        ///  bear in mind that having a zero on the inertia tensor's diagonal isn't generally valid for real,
        ///  3-dimensional objects (unless they are "infinitely thin" along one axis),
        ///  so if you end up with such a tensor, it's a sign that something else might be wrong in your setup.        
        /// </returns>
        public FixedMatrix3x3 InvertDiagonal()
        {
            if (m11 == FixedMath.Zero)
            {
                Console.WriteLine("Cannot invert a diagonal matrix with zero elements on the diagonal.");
                return this;
            }

            return new FixedMatrix3x3(
                m00 != FixedMath.Zero ? FixedMath.One / m00 : FixedMath.Zero, FixedMath.Zero, FixedMath.Zero,
                FixedMath.Zero, FixedMath.One / m11, FixedMath.Zero,
                FixedMath.Zero, FixedMath.Zero, m22 != FixedMath.Zero ? FixedMath.One / m22 : FixedMath.Zero
            );
        }

        /// <summary>
        /// Linearly interpolates between two matrices.
        /// </summary>
        public static FixedMatrix3x3 Lerp(FixedMatrix3x3 a, FixedMatrix3x3 b, Fixed64 t)
        {
            // Perform a linear interpolation between two matrices
            return new FixedMatrix3x3(
                FixedMath.LinearInterpolate(a.m00, b.m00, t), FixedMath.LinearInterpolate(a.m01, b.m01, t), FixedMath.LinearInterpolate(a.m02, b.m02, t),
                FixedMath.LinearInterpolate(a.m10, b.m10, t), FixedMath.LinearInterpolate(a.m11, b.m11, t), FixedMath.LinearInterpolate(a.m12, b.m12, t),
                FixedMath.LinearInterpolate(a.m20, b.m20, t), FixedMath.LinearInterpolate(a.m21, b.m21, t), FixedMath.LinearInterpolate(a.m22, b.m22, t)
            );
        }

        /// <summary>
        /// Creates a scaling matrix (puts the 'scale' vector down the diagonal)
        /// </summary>
        public static FixedMatrix3x3 Scale(Vector3d scale)
        {
            return Scale(scale.x, scale.y, scale.z);
        }

        public static FixedMatrix3x3 Scale(Fixed64 x, Fixed64 y, Fixed64 z)
        {
            return new FixedMatrix3x3(
                x, FixedMath.Zero, FixedMath.Zero,
                FixedMath.Zero, y, FixedMath.Zero,
                FixedMath.Zero, FixedMath.Zero, z
            );
        }

        /// <summary>
        /// Transposes the matrix (swaps rows and columns).
        /// </summary>
        public static FixedMatrix3x3 Transpose(FixedMatrix3x3 matrix)
        {
            return new FixedMatrix3x3(
                matrix.m00, matrix.m10, matrix.m20,
                matrix.m01, matrix.m11, matrix.m21,
                matrix.m02, matrix.m12, matrix.m22
            );
        }

        #region Convert

        public override string ToString()
        {
            return $"[{m00}, {m01}, {m02}; {m10}, {m11}, {m12}; {m20}, {m21}, {m22}]";
        }

        #endregion

        #region Operators

        public static FixedMatrix3x3 operator -(FixedMatrix3x3 a, FixedMatrix3x3 b)
        {
            // Subtract each element
            return new FixedMatrix3x3(
                a.m00 - b.m00, a.m01 - b.m01, a.m02 - b.m02,
                a.m10 - b.m10, a.m11 - b.m11, a.m12 - b.m12,
                a.m20 - b.m20, a.m21 - b.m21, a.m22 - b.m22
            );
        }
        public static FixedMatrix3x3 operator +(FixedMatrix3x3 a, FixedMatrix3x3 b)
        {
            // Add each element
            return new FixedMatrix3x3(
                a.m00 + b.m00, a.m01 + b.m01, a.m02 + b.m02,
                a.m10 + b.m10, a.m11 + b.m11, a.m12 + b.m12,
                a.m20 + b.m20, a.m21 + b.m21, a.m22 + b.m22
            );
        }
        /// <summary>
        /// Negates all elements of the matrix.
        /// </summary>
        public static FixedMatrix3x3 operator -(FixedMatrix3x3 a)
        {
            // Negate each element
            return new FixedMatrix3x3(
                -a.m00, -a.m01, -a.m02,
                -a.m10, -a.m11, -a.m12,
                -a.m20, -a.m21, -a.m22
            );
        }
        public static FixedMatrix3x3 operator *(FixedMatrix3x3 a, FixedMatrix3x3 b)
        {
            // Perform matrix multiplication
            return new FixedMatrix3x3(
                a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20,
                a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21,
                a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22,

                a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20,
                a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21,
                a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22,

                a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20,
                a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21,
                a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22
            );
        }
        public static FixedMatrix3x3 operator *(FixedMatrix3x3 a, Fixed64 scalar)
        {
            // Perform matrix multiplication by scalar
            return new FixedMatrix3x3(
                a.m00 * scalar, a.m01 * scalar, a.m02 * scalar,
                a.m10 * scalar, a.m11 * scalar, a.m12 * scalar,
                a.m20 * scalar, a.m21 * scalar, a.m22 * scalar
            );
        }
        public static FixedMatrix3x3 operator *(Fixed64 scalar, FixedMatrix3x3 a)
        {
            // Perform matrix multiplication by scalar
            return a * scalar;
        }
        public static FixedMatrix3x3 operator /(FixedMatrix3x3 a, int divisor)
        {
            // Perform matrix multiplication by scalar
            return new FixedMatrix3x3(
                a.m00 / divisor, a.m01 / divisor, a.m02 / divisor,
                a.m10 / divisor, a.m11 / divisor, a.m12 / divisor,
                a.m20 / divisor, a.m21 / divisor, a.m22 / divisor
            );
        }
        public static FixedMatrix3x3 operator /(int divisor, FixedMatrix3x3 a)
        {
            return a / divisor;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(FixedMatrix3x3 left, FixedMatrix3x3 right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(FixedMatrix3x3 left, FixedMatrix3x3 right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(FixedMatrix3x3 other)
        {
            // Compare each element for equality
            return
                m00 == other.m00 && m01 == other.m01 && m02 == other.m02 &&
                m10 == other.m10 && m11 == other.m11 && m12 == other.m12 &&
                m20 == other.m20 && m21 == other.m21 && m22 == other.m22;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is FixedMatrix3x3 other && Equals(other);
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
                return hash;
            }
        }

        #endregion
    }
}