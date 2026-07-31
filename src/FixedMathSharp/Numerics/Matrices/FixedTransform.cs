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
/// This engine-neutral shell stores local components and a parent reference only. It does not
/// own children, scene state, synchronization, caches, or dirty propagation.
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
    public Vector3d LocalPosition
    {
        get => _localPosition;
        set => _localPosition = value;
    }

    /// <summary>
    /// Gets or sets the authoritative normalized local rotation.
    /// </summary>
    public FixedQuaternion LocalRotation
    {
        get => _localRotation;
        set => _localRotation = value.Normalized;
    }

    /// <summary>
    /// Gets or sets the authoritative exact signed local scale.
    /// </summary>
    public Vector3d LocalScale
    {
        get => _localScale;
        set => _localScale = value;
    }

    /// <summary>
    /// Gets or sets the local rotation as Euler angles in degrees.
    /// </summary>
    public Vector3d LocalEulerAngles
    {
        get => _localRotation.EulerAngles;
        set => LocalRotation = FixedQuaternion.FromEulerAnglesInDegrees(value.X, value.Y, value.Z);
    }

    /// <summary>
    /// Gets or sets local X/Z position while preserving local Y.
    /// </summary>
    public Vector2d LocalPositionXZ
    {
        get => _localPosition.ToVector2d();
        set => _localPosition = value.ToVector3d(_localPosition.Y);
    }

    /// <summary>
    /// Gets or sets local X/Z rotation in radians from planar right toward planar forward.
    /// </summary>
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
    public Vector2d LocalScaleXZ
    {
        get => _localScale.ToVector2d();
        set => _localScale = value.ToVector3d(_localScale.Y);
    }

    /// <summary>
    /// Gets the optional parent transform.
    /// </summary>
    public FixedTransform? Parent => _parent;

    /// <summary>
    /// Gets the matrix rebuilt from the authoritative local components.
    /// </summary>
    public Fixed4x4 LocalMatrix => Fixed4x4.CreateTransform(_localPosition, _localRotation, _localScale);

    /// <summary>
    /// Gets the iteratively composed local-to-world matrix.
    /// </summary>
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
    public Vector3d WorldPosition => _parent == null
        ? _localPosition
        : LocalToWorldMatrix.Translation;

    /// <summary>
    /// Gets the normalized rotational parent-to-child quaternion chain.
    /// </summary>
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
    public Vector2d WorldPositionXZ => WorldPosition.ToVector2d();

    /// <summary>
    /// Gets world X/Z rotation in radians from planar right toward planar forward.
    /// </summary>
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
    /// <remarks>
    /// The inverse candidate must multiply to identity in both orders within
    /// <see cref="Fixed64.Epsilon"/>. Singular and saturated candidates return identity and false.
    /// </remarks>
    public bool TryGetWorldToLocalMatrix(out Fixed4x4 matrix)
    {
        return TryGetVerifiedInverse(LocalToWorldMatrix, out matrix);
    }

    /// <summary>
    /// Changes the parent without changing any local component.
    /// </summary>
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
    /// Attempts to set world position and normalized world rotation atomically.
    /// </summary>
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
    /// <remarks>Perspective, shear, singular, and non-round-trippable inputs return null and false.</remarks>
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
            && (world * candidate).FuzzyEqualAbsolute(Fixed4x4.Identity, Fixed64.Epsilon)
            && (candidate * world).FuzzyEqualAbsolute(Fixed4x4.Identity, Fixed64.Epsilon))
        {
            inverse = candidate;
            return true;
        }

        inverse = Fixed4x4.Identity;
        return false;
    }

    private bool TryGetStrictLocalMatrix(out Fixed4x4 matrix)
    {
        Fixed3x3 rotation = _localRotation.ToMatrix3x3();
        bool representable = Fixed64.TryMultiplyAdd(rotation.M11, _localScale.X, Fixed64.Zero, out Fixed64 m11)
            & Fixed64.TryMultiplyAdd(rotation.M12, _localScale.X, Fixed64.Zero, out Fixed64 m12)
            & Fixed64.TryMultiplyAdd(rotation.M13, _localScale.X, Fixed64.Zero, out Fixed64 m13)
            & Fixed64.TryMultiplyAdd(rotation.M21, _localScale.Y, Fixed64.Zero, out Fixed64 m21)
            & Fixed64.TryMultiplyAdd(rotation.M22, _localScale.Y, Fixed64.Zero, out Fixed64 m22)
            & Fixed64.TryMultiplyAdd(rotation.M23, _localScale.Y, Fixed64.Zero, out Fixed64 m23)
            & Fixed64.TryMultiplyAdd(rotation.M31, _localScale.Z, Fixed64.Zero, out Fixed64 m31)
            & Fixed64.TryMultiplyAdd(rotation.M32, _localScale.Z, Fixed64.Zero, out Fixed64 m32)
            & Fixed64.TryMultiplyAdd(rotation.M33, _localScale.Z, Fixed64.Zero, out Fixed64 m33);
        if (!representable)
        {
            matrix = Fixed4x4.Zero;
            return false;
        }

        matrix = new Fixed4x4(
            m11, m12, m13, Fixed64.Zero,
            m21, m22, m23, Fixed64.Zero,
            m31, m32, m33, Fixed64.Zero,
            _localPosition.X, _localPosition.Y, _localPosition.Z, Fixed64.One);
        return true;
    }

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
