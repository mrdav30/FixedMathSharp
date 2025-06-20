using MessagePack;
using System;
using System.Drawing.Drawing2D;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a 3x3 matrix used for linear transformations in 2D and 3D space, such as rotation, scaling, and shearing.
    /// </summary>
    /// <remarks>
    /// A 3x3 matrix handles only linear transformations and is typically used when translation is not needed.
    /// It operates on directions, orientations, and vectors within a given space without affecting position.
    /// This matrix is more lightweight compared to a 4x4 matrix, making it ideal when translation and perspective are unnecessary.
    /// 
    /// Use Cases:
    /// - Rotating or scaling objects around the origin in 2D and 3D space.
    /// - Transforming vectors and normals (e.g., in lighting calculations).
    /// - Used in physics engines for inertia tensors or to represent local orientations.
    /// - Useful when optimizing transformations, as it omits the overhead of translation and perspective.
    /// </remarks>
    [Serializable]
    [MessagePackObject]
    public struct Fixed3x3 : IEquatable<Fixed3x3>
    {
        #region Fields and Constants

        [Key(0)]
        public Fixed64 m00;
        [Key(1)]
        public Fixed64 m01;
        [Key(2)]
        public Fixed64 m02;

        [Key(3)]
        public Fixed64 m10;
        [Key(4)]
        public Fixed64 m11;
        [Key(5)]
        public Fixed64 m12;

        [Key(6)]
        public Fixed64 m20;
        [Key(7)]
        public Fixed64 m21;
        [Key(8)]
        public Fixed64 m22;

        /// <summary>
        /// Returns the identity matrix (no scaling, rotation, or translation).
        /// </summary>
        public static readonly Fixed3x3 Identity = new(new Vector3d(1f, 0f, 0f), new Vector3d(0f, 1f, 0f), new Vector3d(0f, 0f, 1f));      

        /// <summary>
        /// Returns a matrix with all elements set to zero.
        /// </summary>
        public static readonly Fixed3x3 Zero = new(new Vector3d(0f, 0f, 0f), new Vector3d(0f, 0f, 0f), new Vector3d(0f, 0f, 0f));

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new FixedMatrix3x3 with the specified elements.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed3x3(
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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed3x3(
            Vector3d m00_m01_m02,
            Vector3d m10_m11_m12,
            Vector3d m20_m21_m22
        ) : this(m00_m01_m02.x, m00_m01_m02.y, m00_m01_m02.z, m10_m11_m12.x, m10_m11_m12.y, m10_m11_m12.z, m20_m21_m22.x, m20_m21_m22.y, m20_m21_m22.z) { }

        #endregion

        #region Properties and Methods (Instance)

        [IgnoreMember]
        public Fixed64 this[int index]
        {
            get
            {
                return index switch
                {
                    0 => m00,
                    1 => m10,
                    2 => m20,
                    4 => m01,
                    5 => m11,
                    6 => m21,
                    8 => m02,
                    9 => m12,
                    10 => m22,
                    _ => throw new IndexOutOfRangeException("Invalid matrix index!"),
                };
            }
            set
            {
                switch (index)
                {
                    case 0:
                        m00 = value;
                        break;
                    case 1:
                        m10 = value;
                        break;
                    case 2:
                        m20 = value;
                        break;
                    case 4:
                        m01 = value;
                        break;
                    case 5:
                        m11 = value;
                        break;
                    case 6:
                        m21 = value;
                        break;
                    case 8:
                        m02 = value;
                        break;
                    case 9:
                        m12 = value;
                        break;
                    case 10:
                        m22 = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException("Invalid matrix index!");
                }
            }
        }

        /// <inheritdoc cref="Normalize(Fixed3x3)" />
        public Fixed3x3 Normalize()
        {
            return this = Normalize(this);
        }

        /// <inheritdoc cref="ResetScaleToIdentity(Fixed3x3)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed3x3 ResetScaleToIdentity()
        {
            return this = ResetScaleToIdentity(this);
        }

        /// <summary>
        /// Calculates the determinant of a 3x3 matrix.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 GetDeterminant()
        {
            return m00 * (m11 * m22 - m12 * m21) -
                   m01 * (m10 * m22 - m12 * m20) +
                   m02 * (m10 * m21 - m11 * m20);
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
        /// </remarks>
        public Fixed3x3 InvertDiagonal()
        {
            if (m11 == Fixed64.Zero)
            {
                Console.WriteLine("Cannot invert a diagonal matrix with zero elements on the diagonal.");
                return this;
            }

            return new Fixed3x3(
                m00 != Fixed64.Zero ? Fixed64.One / m00 : Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One / m11, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, m22 != Fixed64.Zero ? Fixed64.One / m22 : Fixed64.Zero
            );
        }

        #endregion

        #region Static Matrix Generators and Transformations

        /// <summary>
        /// Creates a 3x3 matrix representing a rotation around the X-axis.
        /// </summary>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>A 3x3 rotation matrix.</returns>
        public static Fixed3x3 CreateRotationX(Fixed64 angle)
        {
            Fixed64 cos = FixedMath.Cos(angle);
            Fixed64 sin = FixedMath.Sin(angle);

            return new Fixed3x3(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, cos, -sin,
                Fixed64.Zero, sin, cos
            );
        }

        /// <summary>
        /// Creates a 3x3 matrix representing a rotation around the Y-axis.
        /// </summary>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>A 3x3 rotation matrix.</returns>
        public static Fixed3x3 CreateRotationY(Fixed64 angle)
        {
            Fixed64 cos = FixedMath.Cos(angle);
            Fixed64 sin = FixedMath.Sin(angle);

            return new Fixed3x3(
                cos, Fixed64.Zero, sin,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                -sin, Fixed64.Zero, cos
            );
        }

        /// <summary>
        /// Creates a 3x3 matrix representing a rotation around the Z-axis.
        /// </summary>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>A 3x3 rotation matrix.</returns>
        public static Fixed3x3 CreateRotationZ(Fixed64 angle)
        {
            Fixed64 cos = FixedMath.Cos(angle);
            Fixed64 sin = FixedMath.Sin(angle);

            return new Fixed3x3(
                cos, -sin, Fixed64.Zero,
                sin, cos, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );
        }

        /// <summary>
        /// Creates a 3x3 shear matrix.
        /// </summary>
        /// <param name="shX">Shear factor along the X-axis.</param>
        /// <param name="shY">Shear factor along the Y-axis.</param>
        /// <param name="shZ">Shear factor along the Z-axis.</param>
        /// <returns>A 3x3 shear matrix.</returns>
        public static Fixed3x3 CreateShear(Fixed64 shX, Fixed64 shY, Fixed64 shZ)
        {
            return new Fixed3x3(
                Fixed64.One, shX, shY,
                shX, Fixed64.One, shZ,
                shY, shZ, Fixed64.One
            );
        }

        /// <summary>
        /// Creates a scaling matrix that applies a uniform or non-uniform scale transformation.
        /// </summary>
        /// <param name="scale">The scale factors along the X, Y, and Z axes.</param>
        /// <returns>A 3x3 scaling matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 CreateScale(Vector3d scale)
        {
            return new Fixed3x3(
                scale.x, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, scale.y, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, scale.z
            );
        }

        /// <summary>
        /// Creates a uniform scaling matrix with the same scale factor on all axes.
        /// </summary>
        /// <param name="scaleFactor">The uniform scale factor.</param>
        /// <returns>A 3x3 scaling matrix with uniform scaling.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 CreateScale(Fixed64 scaleFactor)
        {
            return CreateScale(new Vector3d(scaleFactor, scaleFactor, scaleFactor));
        }

        /// <summary>
        /// Normalizes the basis vectors of a 3x3 matrix to ensure they are orthogonal and unit length.
        /// </summary>
        /// <remarks>
        /// This method recalculates and normalizes the X, Y, and Z basis vectors of the matrix to avoid numerical drift 
        /// that can occur after multiple transformations. It also ensures that the Z-axis is recomputed to maintain 
        /// orthogonality by taking the cross-product of the normalized X and Y axes.
        /// 
        /// Use Cases:
        /// - Ensuring stability and correctness after repeated transformations involving rotation and scaling.
        /// - Useful in physics calculations where orthogonal matrices are required (e.g., inertia tensors or rotations).
        /// </remarks>
        public static Fixed3x3 Normalize(Fixed3x3 matrix)
        {
            var x = new Vector3d(matrix.m00, matrix.m01, matrix.m02).Normalize();
            var y = new Vector3d(matrix.m10, matrix.m11, matrix.m12).Normalize();
            var z = Vector3d.Cross(x, y).Normalize();

            matrix.m00 = x.x; matrix.m01 = x.y; matrix.m02 = x.z;
            matrix.m10 = y.x; matrix.m11 = y.y; matrix.m12 = y.z;
            matrix.m20 = z.x; matrix.m21 = z.y; matrix.m22 = z.z;

            return matrix;
        }

        /// <summary>
        /// Resets the scaling part of the matrix to identity (1,1,1).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 ResetScaleToIdentity(Fixed3x3 matrix)
        {
            matrix.m00 = Fixed64.One;  // Reset scale on X-axis
            matrix.m11 = Fixed64.One;  // Reset scale on Y-axis
            matrix.m22 = Fixed64.One;  // Reset scale on Z-axis
            return matrix;
        }

        /// <inheritdoc cref="SetLossyScale(Fixed64, Fixed64, Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 SetLossyScale(Vector3d scale)
        {
            return SetLossyScale(scale.x, scale.y, scale.z);
        }

        /// <summary>
        /// Creates a scaling matrix (puts the 'scale' vector down the diagonal)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 SetLossyScale(Fixed64 x, Fixed64 y, Fixed64 z)
        {
            return new Fixed3x3(
                x, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, y, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, z
            );
        }

        /// <summary>
        /// Applies the provided local scale to the matrix by modifying the diagonal elements.
        /// </summary>
        /// <param name="matrix">The matrix to set the scale against.</param>
        /// <param name="localScale">A Vector3d representing the local scale to apply.</param>
        public static Fixed3x3 SetScale(Fixed3x3 matrix, Vector3d localScale)
        {
            matrix.m00 = localScale.x; // Apply scale on X-axis
            matrix.m11 = localScale.y; // Apply scale on Y-axis
            matrix.m22 = localScale.z; // Apply scale on Z-axis

            return matrix;
        }

        /// <summary>
        /// Sets the global scale of an object using FixedMatrix3x3.
        /// Similar to SetGlobalScale for FixedMatrix4x4, but for a 3x3 matrix.
        /// </summary>
        /// <param name="matrix">The transformation matrix (3x3) representing the object's global state.</param>
        /// <param name="globalScale">The desired global scale represented as a Vector3d.</param>
        /// <remarks>
        /// The method extracts the current global scale from the matrix and computes the new local scale 
        /// by dividing the desired global scale by the current global scale. 
        /// The new local scale is then applied to the matrix.
        /// </remarks>
        public static Fixed3x3 SetGlobalScale(Fixed3x3 matrix, Vector3d globalScale)
        {
            // normalize the matrix to avoid drift in the rotation component
            matrix.Normalize();

            // Reset the local scaling portion of the matrix
            matrix.ResetScaleToIdentity();

            // Compute the new local scale by dividing the desired global scale by the current global scale
            Vector3d newLocalScale = new Vector3d(
                globalScale.x / Fixed64.One,
                globalScale.y / Fixed64.One,
                globalScale.z / Fixed64.One
            );

            // Apply the new local scale to the matrix
            return matrix.SetScale(newLocalScale);
        }

        /// <summary>
        /// Extracts the scaling factors from the matrix by returning the diagonal elements.
        /// </summary>
        /// <returns>A Vector3d representing the scale along X, Y, and Z axes.</returns>
        public static Vector3d ExtractScale(Fixed3x3 matrix)
        {
            return new Vector3d(
                new Vector3d(matrix.m00, matrix.m01, matrix.m02).Magnitude,
                new Vector3d(matrix.m10, matrix.m11, matrix.m12).Magnitude,
                new Vector3d(matrix.m20, matrix.m21, matrix.m22).Magnitude
            );
        }

        /// <summary>
        /// Extracts the scaling factors from the matrix by returning the diagonal elements (lossy).
        /// </summary>
        /// <returns>A Vector3d representing the scale along X, Y, and Z axes (lossy).</returns>
        public static Vector3d ExtractLossyScale(Fixed3x3 matrix)
        {
            return new Vector3d(matrix.m00, matrix.m11, matrix.m22);
        }

        #endregion

        #region Static Matrix Operations

        /// <summary>
        /// Linearly interpolates between two matrices.
        /// </summary>
        public static Fixed3x3 Lerp(Fixed3x3 a, Fixed3x3 b, Fixed64 t)
        {
            // Perform a linear interpolation between two matrices
            return new Fixed3x3(
                FixedMath.LinearInterpolate(a.m00, b.m00, t), FixedMath.LinearInterpolate(a.m01, b.m01, t), FixedMath.LinearInterpolate(a.m02, b.m02, t),
                FixedMath.LinearInterpolate(a.m10, b.m10, t), FixedMath.LinearInterpolate(a.m11, b.m11, t), FixedMath.LinearInterpolate(a.m12, b.m12, t),
                FixedMath.LinearInterpolate(a.m20, b.m20, t), FixedMath.LinearInterpolate(a.m21, b.m21, t), FixedMath.LinearInterpolate(a.m22, b.m22, t)
            );
        }

        /// <summary>
        /// Transposes the matrix (swaps rows and columns).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 Transpose(Fixed3x3 matrix)
        {
            return new Fixed3x3(
                matrix.m00, matrix.m10, matrix.m20,
                matrix.m01, matrix.m11, matrix.m21,
                matrix.m02, matrix.m12, matrix.m22
            );
        }

        /// <summary>
        /// Attempts to invert the matrix. If the determinant is zero, returns false and sets result to null.
        /// </summary>
        public static bool Invert(Fixed3x3 matrix, out Fixed3x3? result)
        {
            // Calculate the determinant
            Fixed64 det = matrix.GetDeterminant();

            if (det == Fixed64.Zero)
            {
                result = null;
                return false;
            }

            // Calculate the inverse
            Fixed64 invDet = Fixed64.One / det;

            // Compute the inverse matrix
            result = new Fixed3x3(
                invDet * (matrix.m11 * matrix.m22 - matrix.m21 * matrix.m12),
                invDet * (matrix.m02 * matrix.m21 - matrix.m01 * matrix.m22),
                invDet * (matrix.m01 * matrix.m12 - matrix.m02 * matrix.m11),

                invDet * (matrix.m12 * matrix.m20 - matrix.m10 * matrix.m22),
                invDet * (matrix.m00 * matrix.m22 - matrix.m02 * matrix.m20),
                invDet * (matrix.m02 * matrix.m10 - matrix.m00 * matrix.m12),

                invDet * (matrix.m10 * matrix.m21 - matrix.m11 * matrix.m20),
                invDet * (matrix.m01 * matrix.m20 - matrix.m00 * matrix.m21),
                invDet * (matrix.m00 * matrix.m11 - matrix.m01 * matrix.m10)
            );

            return true;
        }

        /// <summary>
        /// Transforms a direction vector from local space to world space using this transformation matrix.
        /// Ignores translation.
        /// </summary>
        /// <param name="matrix">The transformation matrix.</param>
        /// <param name="direction">The local-space direction vector.</param>
        /// <returns>The transformed direction in world space.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d TransformDirection(Fixed3x3 matrix, Vector3d direction)
        {
            return new Vector3d(
                matrix.m00 * direction.x + matrix.m01 * direction.y + matrix.m02 * direction.z,
                matrix.m10 * direction.x + matrix.m11 * direction.y + matrix.m12 * direction.z,
                matrix.m20 * direction.x + matrix.m21 * direction.y + matrix.m22 * direction.z
            );
        }

        /// <summary>
        /// Transforms a direction from world space into the local space of the matrix.
        /// Ignores translation.
        /// </summary>
        /// <param name="matrix">The transformation matrix.</param>
        /// <param name="direction">The world-space direction.</param>
        /// <returns>The transformed local-space direction.</returns>
        public static Vector3d InverseTransformDirection(Fixed3x3 matrix, Vector3d direction)
        {
            if (!Invert(matrix, out Fixed3x3? inverseMatrix) || !inverseMatrix.HasValue)
                throw new InvalidOperationException("Matrix is not invertible.");

            return new Vector3d(
                inverseMatrix.Value.m00 * direction.x + inverseMatrix.Value.m01 * direction.y + inverseMatrix.Value.m02 * direction.z,
                inverseMatrix.Value.m10 * direction.x + inverseMatrix.Value.m11 * direction.y + inverseMatrix.Value.m12 * direction.z,
                inverseMatrix.Value.m20 * direction.x + inverseMatrix.Value.m21 * direction.y + inverseMatrix.Value.m22 * direction.z
            );
        }

        #endregion

        #region Operators

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 operator -(Fixed3x3 a, Fixed3x3 b)
        {
            // Subtract each element
            return new Fixed3x3(
                a.m00 - b.m00, a.m01 - b.m01, a.m02 - b.m02,
                a.m10 - b.m10, a.m11 - b.m11, a.m12 - b.m12,
                a.m20 - b.m20, a.m21 - b.m21, a.m22 - b.m22
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 operator +(Fixed3x3 a, Fixed3x3 b)
        {
            // Add each element
            return new Fixed3x3(
                a.m00 + b.m00, a.m01 + b.m01, a.m02 + b.m02,
                a.m10 + b.m10, a.m11 + b.m11, a.m12 + b.m12,
                a.m20 + b.m20, a.m21 + b.m21, a.m22 + b.m22
            );
        }
        /// <summary>
        /// Negates all elements of the matrix.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 operator -(Fixed3x3 a)
        {
            // Negate each element
            return new Fixed3x3(
                -a.m00, -a.m01, -a.m02,
                -a.m10, -a.m11, -a.m12,
                -a.m20, -a.m21, -a.m22
            );
        }
        public static Fixed3x3 operator *(Fixed3x3 a, Fixed3x3 b)
        {
            // Perform matrix multiplication
            return new Fixed3x3(
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

        public static Fixed3x3 operator *(Fixed3x3 a, Fixed64 scalar)
        {
            // Perform matrix multiplication by scalar
            return new Fixed3x3(
                a.m00 * scalar, a.m01 * scalar, a.m02 * scalar,
                a.m10 * scalar, a.m11 * scalar, a.m12 * scalar,
                a.m20 * scalar, a.m21 * scalar, a.m22 * scalar
            );
        }

        public static Fixed3x3 operator *(Fixed64 scalar, Fixed3x3 a)
        {
            // Perform matrix multiplication by scalar
            return a * scalar;
        }

        public static Fixed3x3 operator /(Fixed3x3 a, int divisor)
        {
            // Perform matrix multiplication by scalar
            return new Fixed3x3(
                a.m00 / divisor, a.m01 / divisor, a.m02 / divisor,
                a.m10 / divisor, a.m11 / divisor, a.m12 / divisor,
                a.m20 / divisor, a.m21 / divisor, a.m22 / divisor
            );
        }

        public static Fixed3x3 operator /(int divisor, Fixed3x3 a)
        {
            return a / divisor;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Fixed3x3 left, Fixed3x3 right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Fixed3x3 left, Fixed3x3 right)
        {
            return !left.Equals(right);
        }

        #endregion

        #region Equality and HashCode Overrides

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Fixed3x3 other)
        {
            // Compare each element for equality
            return
                m00 == other.m00 && m01 == other.m01 && m02 == other.m02 &&
                m10 == other.m10 && m11 == other.m11 && m12 == other.m12 &&
                m20 == other.m20 && m21 == other.m21 && m22 == other.m22;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object? obj)
        {
            return obj is Fixed3x3 other && Equals(other);
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

        #region Conversion

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"[{m00}, {m01}, {m02}; {m10}, {m11}, {m12}; {m20}, {m21}, {m22}]";
        }

        #endregion
    }
}