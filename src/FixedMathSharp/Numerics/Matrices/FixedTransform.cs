//=======================================================================
// FixedTransform.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp;

/// <summary>
/// Provides a mutable deterministic transform snapshot with optional hierarchy composition.
/// </summary>
/// <remarks>
/// Stores local components by value and retains the parent by reference. World-space views
/// read the current hierarchy each time, so changing an ancestor affects subsequent reads.
/// Coordinate access and mutation externally when sharing a hierarchy between threads.
/// </remarks>
public class FixedTransform
{
    private Vector3d _localPosition;
    private FixedQuaternion _localRotation;
    private Vector3d _localScale;
    private FixedTransform? _parent;

    /// <summary>
    /// Initializes a transform from authoritative 3D local components.
    /// </summary>
    /// <param name="localPosition">Translation in the parent's coordinate space, or world space for a root.</param>
    /// <param name="localRotation">The local orientation. It is normalized before storage.</param>
    /// <param name="localScale">The signed scale factors along the local X, Y, and Z axes.</param>
    /// <param name="parent">The retained parent reference, or <see langword="null"/> for a root.</param>
    public FixedTransform(
        Vector3d localPosition,
        FixedQuaternion localRotation,
        Vector3d localScale,
        FixedTransform? parent = null)
    {
        _localPosition = localPosition;
        _localRotation = localRotation.Normalized;
        _localScale = localScale;
        _parent = parent;
    }

    /// <summary>
    /// Initializes an X/Z local transform at zero Y position with unit Y scale.
    /// </summary>
    /// <param name="localPositionXZ">Local X and Z translation, in that order.</param>
    /// <param name="localRotationXZRadians">The angle in radians from local +X toward +Z.</param>
    /// <param name="localScaleXZ">Signed scale factors along the local X and Z axes, in that order.</param>
    /// <param name="parent">The retained parent reference, or <see langword="null"/> for a root.</param>
    public FixedTransform(
        Vector2d localPositionXZ,
        Fixed64 localRotationXZRadians,
        Vector2d localScaleXZ,
        FixedTransform? parent = null)
        : this(
            localPositionXZ.ToVector3d(Fixed64.Zero),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -localRotationXZRadians),
            localScaleXZ.ToVector3d(Fixed64.One),
            parent)
    { }

    /// <summary>
    /// Gets or sets the authoritative local translation.
    /// </summary>
    /// <value>A position in the parent's coordinate space, or in world space for a root.</value>
    public Vector3d LocalPosition
    {
        get => _localPosition;
        set => _localPosition = value;
    }

    /// <summary>
    /// Gets or sets the authoritative normalized local rotation.
    /// </summary>
    /// <value>The local orientation. Assigned values are normalized before storage.</value>
    public FixedQuaternion LocalRotation
    {
        get => _localRotation;
        set => _localRotation = value.Normalized;
    }

    /// <summary>
    /// Gets or sets the authoritative exact signed local scale.
    /// </summary>
    /// <value>The signed scale factors along the local X, Y, and Z axes, including zero.</value>
    public Vector3d LocalScale
    {
        get => _localScale;
        set => _localScale = value;
    }

    /// <summary>
    /// Gets or sets the local rotation as Euler angles in degrees.
    /// </summary>
    /// <value>Pitch, yaw, and roll in degrees in X, Y, and Z, respectively.</value>
    public Vector3d LocalEulerAngles
    {
        get => _localRotation.EulerAngles;
        set => LocalRotation = FixedQuaternion.FromEulerAnglesInDegrees(value.X, value.Y, value.Z);
    }

    /// <summary>
    /// Gets or sets local X/Z position while preserving local Y.
    /// </summary>
    /// <value>The X and Z components of <see cref="LocalPosition"/>, in that order.</value>
    public Vector2d LocalPositionXZ
    {
        get => _localPosition.ToVector2d();
        set => _localPosition = value.ToVector3d(_localPosition.Y);
    }

    /// <summary>
    /// Gets or sets local X/Z rotation in radians from planar right toward planar forward.
    /// </summary>
    /// <value>The signed angle in radians from +X to the projection of the rotated +X axis onto X/Z.</value>
    /// <remarks>The setter replaces pitch and roll with a pure Y-axis rotation.</remarks>
    public Fixed64 LocalRotationXZRadians
    {
        get
        {
            Vector3d localRight = _localRotation.Rotate(Vector3d.Right);
            return FixedMath.Atan2(localRight.Z, localRight.X);
        }
        set => LocalRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -value);
    }

    /// <summary>
    /// Gets or sets local X/Z scale while preserving local Y.
    /// </summary>
    /// <value>The signed X and Z components of <see cref="LocalScale"/>, in that order.</value>
    public Vector2d LocalScaleXZ
    {
        get => _localScale.ToVector2d();
        set => _localScale = value.ToVector3d(_localScale.Y);
    }

    /// <summary>
    /// Gets the optional parent transform.
    /// </summary>
    /// <value>The live parent reference, or <see langword="null"/> for a root.</value>
    public FixedTransform? Parent => _parent;

    /// <summary>
    /// Gets the matrix rebuilt from the authoritative local components.
    /// </summary>
    /// <value>The row-vector matrix that applies local scale, rotation, then translation.</value>
    public Fixed4x4 LocalMatrix => Fixed4x4.CreateTransform(_localPosition, _localRotation, _localScale);

    /// <summary>
    /// Gets the iteratively composed local-to-world matrix.
    /// </summary>
    /// <value>A snapshot of the current hierarchy's row-vector transform. Intermediate entries can saturate.</value>
    /// <remarks>
    /// Row-vector order multiplies this local matrix by its parent local matrix and then each
    /// ancestor local matrix. Reading is linear in hierarchy depth and allocation-free.
    /// </remarks>
    public Fixed4x4 LocalToWorldMatrix
    {
        get
        {
            Fixed4x4 matrix = LocalMatrix;
            FixedTransform? ancestor = _parent;
            while (ancestor != null)
            {
                matrix *= ancestor.LocalMatrix;
                ancestor = ancestor._parent;
            }

            return matrix;
        }
    }

    /// <summary>
    /// Attempts to get the iteratively composed local-to-world matrix without intermediate saturation.
    /// </summary>
    /// <param name="matrix">The composed row-vector matrix on success, or <see cref="Fixed4x4.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if each local matrix and intermediate product is representable.</returns>
    /// <remarks>Failure returns a zero matrix and does not mutate this transform or any ancestor.</remarks>
    public bool TryGetLocalToWorldMatrix(out Fixed4x4 matrix)
    {
        if (!TryGetStrictLocalMatrix(out matrix))
            return false;

        FixedTransform? ancestor = _parent;
        while (ancestor != null)
        {
            if (!ancestor.TryGetStrictLocalMatrix(out Fixed4x4 localMatrix)
                || !Fixed4x4.TryMultiply(matrix, localMatrix, out Fixed4x4 combined))
            {
                matrix = Fixed4x4.Zero;
                return false;
            }

            matrix = combined;
            ancestor = ancestor._parent;
        }

        return true;
    }

    /// <summary>
    /// Gets world translation from the composed matrix.
    /// </summary>
    /// <value>The position of the local origin in world space.</value>
    public Vector3d WorldPosition => _parent == null
        ? _localPosition
        : LocalToWorldMatrix.Translation;

    /// <summary>
    /// Gets the normalized rotational parent-to-child quaternion chain.
    /// </summary>
    /// <value>The composed orientation in world space, derived from the local rotation quaternions.</value>
    /// <remarks>
    /// This is an orientation chain derived only from stored local quaternions. It is independent
    /// of scale, reflections, and any shear in <see cref="LocalToWorldMatrix"/>, so it is not a
    /// decomposition of that matrix.
    /// </remarks>
    public FixedQuaternion WorldRotation
    {
        get
        {
            if (_parent == null)
                return _localRotation;

            FixedQuaternion rotation = _localRotation;
            FixedTransform? ancestor = _parent;
            while (ancestor != null)
            {
                rotation = ancestor._localRotation * rotation;
                ancestor = ancestor._parent;
            }

            return rotation.Normalized;
        }
    }

    /// <summary>
    /// Gets canonical signed lossy scale from the composed matrix.
    /// </summary>
    /// <value>The world basis row lengths, with negative X for reflected handedness.</value>
    /// <remarks>Zero magnitudes are preserved and reflected handedness is canonicalized to negative X.</remarks>
    public Vector3d LossyScale => _parent == null
        && _localScale.X >= Fixed64.Zero
        && _localScale.Y >= Fixed64.Zero
        && _localScale.Z >= Fixed64.Zero
            ? _localScale
            : LocalToWorldMatrix.LossyScale;

    /// <summary>
    /// Attempts to get canonical signed lossy scale from strict hierarchy composition.
    /// </summary>
    /// <param name="scale">The signed world basis row lengths on success, or <see cref="Vector3d.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if composition and the signed lengths are representable.</returns>
    /// <remarks>Failure returns zero and does not mutate this transform or any ancestor.</remarks>
    public bool TryGetLossyScale(out Vector3d scale)
    {
        if (_parent == null
            && _localScale.X >= Fixed64.Zero
            && _localScale.Y >= Fixed64.Zero
            && _localScale.Z >= Fixed64.Zero)
        {
            scale = _localScale;
            return true;
        }

        if (!TryGetLocalToWorldMatrix(out Fixed4x4 matrix))
        {
            scale = Vector3d.Zero;
            return false;
        }

        return Fixed4x4.TryExtractLossyScale(matrix, out scale);
    }

    /// <summary>
    /// Gets world position projected onto X/Z.
    /// </summary>
    /// <value>The X and Z components of <see cref="WorldPosition"/>, in that order.</value>
    public Vector2d WorldPositionXZ => WorldPosition.ToVector2d();

    /// <summary>
    /// Gets world X/Z rotation in radians from planar right toward planar forward.
    /// </summary>
    /// <value>The signed angle in radians from world +X to the rotated +X axis projected onto world X/Z.</value>
    public Fixed64 WorldRotationXZRadians
    {
        get
        {
            Vector3d worldRight = WorldRotation.Rotate(Vector3d.Right);
            return FixedMath.Atan2(worldRight.Z, worldRight.X);
        }
    }

    /// <summary>
    /// Attempts to get a verified inverse of the composed local-to-world matrix.
    /// </summary>
    /// <param name="matrix">The world-to-local matrix on success, or <see cref="Fixed4x4.Identity"/> on failure.</param>
    /// <returns><see langword="true"/> if the hierarchy composes without saturation and its inverse passes verification.</returns>
    /// <remarks>
    /// The inverse candidate must multiply to identity in both orders within
    /// <see cref="FixedMath.CanonicalSinCosErrorBound"/>. Singular, saturated,
    /// and unrepresentable candidates return identity and false.
    /// </remarks>
    public bool TryGetWorldToLocalMatrix(out Fixed4x4 matrix)
    {
        if (!TryGetLocalToWorldMatrix(out Fixed4x4 world))
        {
            matrix = Fixed4x4.Identity;
            return false;
        }

        return TryGetVerifiedInverse(world, out matrix);
    }

    /// <summary>
    /// Transforms a point from local space to world space through the complete composed affine hierarchy.
    /// </summary>
    /// <param name="point">The point in local 3D coordinates.</param>
    /// <returns>The point in world 3D coordinates.</returns>
    /// <exception cref="InvalidOperationException">
    /// The composed transform or final world-space point is not representable.
    /// </exception>
    public Vector3d TransformPoint(Vector3d point)
    {
        if (!TryTransformPoint(point, out Vector3d result))
            throw new InvalidOperationException(
                "The composed transform or final world-space point is not representable.");

        return result;
    }

    /// <summary>
    /// Attempts to transform a point from local space to world space through the complete composed affine hierarchy.
    /// </summary>
    /// <param name="point">The point in local 3D coordinates.</param>
    /// <param name="result">The transformed world point on success, or <see cref="Vector3d.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if the composed matrix and world point are representable.</returns>
    public bool TryTransformPoint(Vector3d point, out Vector3d result)
    {
        if (!TryGetLocalToWorldMatrix(out Fixed4x4 matrix))
        {
            result = default;
            return false;
        }

        return Fixed4x4.TryTransformAffinePoint(matrix, point, out result);
    }

    /// <summary>
    /// Transforms a point from world space to local space through the complete composed affine hierarchy.
    /// </summary>
    /// <param name="point">The point in world 3D coordinates.</param>
    /// <returns>The point in local 3D coordinates.</returns>
    /// <exception cref="InvalidOperationException">
    /// The composed transform is singular or unrepresentable, or the final local-space point is not representable.
    /// </exception>
    public Vector3d InverseTransformPoint(Vector3d point)
    {
        if (!TryInverseTransformPoint(point, out Vector3d result))
            throw new InvalidOperationException(
                "The composed transform is singular or unrepresentable, or the final local-space point is not representable.");

        return result;
    }

    /// <summary>
    /// Attempts to transform a point from world space to local space through the complete composed affine hierarchy.
    /// </summary>
    /// <param name="point">The point in world 3D coordinates.</param>
    /// <param name="result">The transformed local point on success, or <see cref="Vector3d.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if the world-to-local matrix passes verification and the local point is representable.</returns>
    public bool TryInverseTransformPoint(Vector3d point, out Vector3d result)
    {
        if (!TryGetWorldToLocalMatrix(out Fixed4x4 matrix))
        {
            result = default;
            return false;
        }

        return Fixed4x4.TryTransformAffinePoint(matrix, point, out result);
    }

    /// <summary>
    /// Transforms a point from local X/Z space to world X/Z space through a plane-preserving affine hierarchy.
    /// </summary>
    /// <param name="point">The point in local X and Z coordinates, in that order.</param>
    /// <returns>The point in world X and Z coordinates.</returns>
    /// <exception cref="InvalidOperationException">
    /// The hierarchy couples X/Z with Y, or the composed transform or final world-space point is not representable.
    /// </exception>
    public Vector2d TransformPointXZ(Vector2d point)
    {
        if (!TryTransformPointXZ(point, out Vector2d result))
            throw new InvalidOperationException(
                "The hierarchy must preserve the X/Z plane and produce a representable world-space point.");

        return result;
    }

    /// <summary>
    /// Attempts to transform a point from local X/Z space to world X/Z space through a plane-preserving affine hierarchy.
    /// </summary>
    /// <param name="point">The point in local X and Z coordinates, in that order.</param>
    /// <param name="result">The world X/Z point on success, or <see cref="Vector2d.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if the hierarchy composes without saturation, X/Z is uncoupled from Y, and the point is representable.</returns>
    /// <remarks>The planar mapping uses only the X/Z basis and translation. It ignores Y translation and scale.</remarks>
    public bool TryTransformPointXZ(Vector2d point, out Vector2d result)
    {
        if (!TryGetPlanarXZMatrix(out Fixed4x4 matrix)
            || !Fixed4x4.TryTransformAffinePoint(
                matrix,
                point.ToVector3d(Fixed64.Zero),
                out Vector3d transformed))
        {
            result = default;
            return false;
        }

        result = transformed.ToVector2d();
        return true;
    }

    /// <summary>
    /// Transforms a point from world X/Z space to local X/Z space through a plane-preserving affine hierarchy.
    /// </summary>
    /// <param name="point">The point in world X and Z coordinates, in that order.</param>
    /// <returns>The point in local X and Z coordinates.</returns>
    /// <exception cref="InvalidOperationException">
    /// The hierarchy couples X/Z with Y or cannot be composed without saturation, the planar
    /// mapping has no verified inverse, or the final local X/Z coordinates are unrepresentable.
    /// </exception>
    public Vector2d InverseTransformPointXZ(Vector2d point)
    {
        if (!TryInverseTransformPointXZ(point, out Vector2d result))
            throw new InvalidOperationException(
                "The hierarchy must preserve an invertible X/Z plane and produce a representable local-space point.");

        return result;
    }

    /// <summary>
    /// Attempts to transform a point from world X/Z space to local X/Z space through a plane-preserving affine hierarchy.
    /// </summary>
    /// <param name="point">The point in world X and Z coordinates, in that order.</param>
    /// <param name="result">The local X/Z point on success, or <see cref="Vector2d.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if the hierarchy composes without saturation, its uncoupled X/Z mapping has a verified inverse, and the point is representable.</returns>
    /// <remarks>The inverse is verified for the planar mapping, independent of Y translation and scale.</remarks>
    public bool TryInverseTransformPointXZ(Vector2d point, out Vector2d result)
    {
        if (!TryGetPlanarXZMatrix(out Fixed4x4 matrix)
            || !TryGetVerifiedInverse(matrix, out Fixed4x4 inverse)
            || !Fixed4x4.TryTransformAffinePoint(
                inverse,
                point.ToVector3d(Fixed64.Zero),
                out Vector3d transformed))
        {
            result = default;
            return false;
        }

        result = transformed.ToVector2d();
        return true;
    }

    /// <summary>
    /// Changes the parent without changing any local component.
    /// </summary>
    /// <param name="parent">The new parent reference to retain, or <see langword="null"/> to make this a root.</param>
    /// <exception cref="ArgumentException">The proposed parent would create a cycle.</exception>
    public void SetParentKeepingLocal(FixedTransform? parent)
    {
        if (ReferenceEquals(_parent, parent))
            return;

        ValidateParent(parent);
        _parent = parent;
    }

    /// <summary>
    /// Attempts to change the parent while preserving the complete world matrix.
    /// </summary>
    /// <param name="parent">The new parent reference to retain on success, or <see langword="null"/> to make this a root.</param>
    /// <returns>
    /// <see langword="true"/> if the parent was already assigned or the world matrix was
    /// reproduced within <see cref="Fixed64.Epsilon"/> per entry with the new parent.
    /// </returns>
    /// <remarks>Failure leaves the parent and every local component unchanged.</remarks>
    /// <exception cref="ArgumentException">The proposed parent would create a cycle.</exception>
    public bool TrySetParentKeepingWorld(FixedTransform? parent)
    {
        if (ReferenceEquals(_parent, parent))
            return true;

        ValidateParent(parent);
        Fixed4x4 world = LocalToWorldMatrix;
        Fixed4x4 parentWorld = parent?.LocalToWorldMatrix ?? Fixed4x4.Identity;
        Fixed4x4 localMatrix;
        if (parent == null)
        {
            localMatrix = world;
        }
        else
        {
            if (!TryGetVerifiedInverse(parentWorld, out Fixed4x4 parentInverse))
                return false;

            localMatrix = world * parentInverse;
        }

        if (!Fixed4x4.Decompose(
            localMatrix,
            out Vector3d localPosition,
            out FixedQuaternion localRotation,
            out Vector3d localScale))
        {
            return false;
        }

        Fixed4x4 recomposedWorld = Fixed4x4.CreateTransform(localPosition, localRotation, localScale) * parentWorld;
        if (!recomposedWorld.FuzzyEqualAbsolute(world, Fixed64.Epsilon))
            return false;

        _parent = parent;
        _localPosition = localPosition;
        _localRotation = localRotation;
        _localScale = localScale;
        return true;
    }

    /// <summary>
    /// Attempts to set world position through the verified parent inverse.
    /// </summary>
    /// <param name="position">The requested position in world space.</param>
    /// <returns><see langword="true"/> for a root, or if the parent matrix has a verified inverse and recomposition matches within <see cref="Fixed64.Epsilon"/> per component.</returns>
    /// <remarks>Failure leaves local position unchanged.</remarks>
    public bool TrySetWorldPosition(Vector3d position)
    {
        if (_parent == null)
        {
            _localPosition = position;
            return true;
        }

        Fixed4x4 parentWorld = _parent.LocalToWorldMatrix;
        if (!TryGetVerifiedInverse(parentWorld, out Fixed4x4 parentInverse))
            return false;

        Vector3d localPosition = Fixed4x4.TransformPoint(parentInverse, position);
        if (!Fixed4x4.TransformPoint(parentWorld, localPosition).FuzzyEqualAbsolute(position, Fixed64.Epsilon))
            return false;

        _localPosition = localPosition;
        return true;
    }

    /// <summary>
    /// Attempts to apply world position and normalized world rotation together.
    /// </summary>
    /// <param name="position">The requested position in world space.</param>
    /// <param name="rotation">The requested world orientation, normalized before computing local rotation.</param>
    /// <returns><see langword="true"/> if both components were applied. Position uses the same acceptance test as <see cref="TrySetWorldPosition"/>.</returns>
    /// <remarks>
    /// Relative rotation is derived from the parent's stored rotational hierarchy, independent
    /// of scale and shear. Failure leaves both local position and local rotation unchanged.
    /// </remarks>
    public bool TrySetWorldPose(Vector3d position, FixedQuaternion rotation)
    {
        FixedQuaternion normalizedRotation = rotation.Normalized;
        if (_parent == null)
        {
            _localPosition = position;
            _localRotation = normalizedRotation;
            return true;
        }

        Fixed4x4 parentWorld = _parent.LocalToWorldMatrix;
        if (!TryGetVerifiedInverse(parentWorld, out Fixed4x4 parentInverse))
            return false;

        Vector3d localPosition = Fixed4x4.TransformPoint(parentInverse, position);
        if (!Fixed4x4.TransformPoint(parentWorld, localPosition).FuzzyEqualAbsolute(position, Fixed64.Epsilon))
            return false;

        FixedQuaternion parentWorldRotation = _parent.WorldRotation;
        FixedQuaternion localRotation = (parentWorldRotation.Inverse() * normalizedRotation).Normalized;

        _localPosition = localPosition;
        _localRotation = localRotation;
        return true;
    }

    /// <summary>
    /// Attempts to create a transform from a strict, nonsingular local TRS matrix.
    /// </summary>
    /// <param name="localMatrix">A local row-vector matrix for scale, then rotation, then translation.</param>
    /// <param name="transform">The newly constructed transform on success, or <see langword="null"/> on failure.</param>
    /// <param name="parent">The parent reference to retain on success, or <see langword="null"/> for a root.</param>
    /// <returns><see langword="true"/> if the supplied local matrix can be decomposed into representable components within tolerance.</returns>
    /// <remarks>
    /// Uses <see cref="Fixed4x4.Decompose"/>'s normalized-basis tolerance and scale-dependent
    /// reconstruction bound. Perspective, shear, singular and unrepresentable inputs return null and false.
    /// </remarks>
    public static bool TryCreateFromLocalMatrix(
        Fixed4x4 localMatrix,
        out FixedTransform? transform,
        FixedTransform? parent = null)
    {
        if (!Fixed4x4.Decompose(
            localMatrix,
            out Vector3d localPosition,
            out FixedQuaternion localRotation,
            out Vector3d localScale))
        {
            transform = null;
            return false;
        }

        transform = new FixedTransform(localPosition, localRotation, localScale, parent);
        return true;
    }

    private static bool TryGetVerifiedInverse(Fixed4x4 world, out Fixed4x4 inverse)
    {
        if (Fixed4x4.Invert(world, out Fixed4x4 candidate)
            && Fixed4x4.TryMultiply(world, candidate, out Fixed4x4 worldRoundTrip)
            && Fixed4x4.TryMultiply(candidate, world, out Fixed4x4 localRoundTrip)
            && worldRoundTrip.FuzzyEqualAbsolute(
                Fixed4x4.Identity,
                FixedMath.CanonicalSinCosErrorBound)
            && localRoundTrip.FuzzyEqualAbsolute(
                Fixed4x4.Identity,
                FixedMath.CanonicalSinCosErrorBound))
        {
            inverse = candidate;
            return true;
        }

        inverse = Fixed4x4.Identity;
        return false;
    }

    private bool TryGetPlanarXZMatrix(out Fixed4x4 matrix)
    {
        if (!TryGetLocalToWorldMatrix(out Fixed4x4 world)
            || ((world.M12 != Fixed64.Zero)
                | (world.M21 != Fixed64.Zero)
                | (world.M23 != Fixed64.Zero)
                | (world.M32 != Fixed64.Zero)))
        {
            matrix = Fixed4x4.Zero;
            return false;
        }

        matrix = new Fixed4x4(
            world.M11, Fixed64.Zero, world.M13, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            world.M31, Fixed64.Zero, world.M33, Fixed64.Zero,
            world.M41, Fixed64.Zero, world.M43, Fixed64.One);
        return true;
    }

    private bool TryGetStrictLocalMatrix(out Fixed4x4 matrix) =>
        Fixed4x4.TryCreateTransform(_localPosition, _localRotation.ToMatrix3x3(), _localScale, out matrix);

    private void ValidateParent(FixedTransform? parent)
    {
        FixedTransform? ancestor = parent;
        while (ancestor != null)
        {
            if (ReferenceEquals(ancestor, this))
                throw new ArgumentException("Parent assignment would create a transform cycle.", nameof(parent));

            ancestor = ancestor._parent;
        }
    }
}
