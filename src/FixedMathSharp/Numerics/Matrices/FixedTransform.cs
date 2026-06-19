//=======================================================================
// FixedTransform.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// Provides a mutable deterministic transform shell backed by a <see cref="Fixed4x4"/> matrix.
/// </summary>
/// <remarks>
/// This type intentionally keeps reference semantics so host-owned objects can publish one
/// transform instance while simulation systems mutate its fixed-point components explicitly.
/// </remarks>
public class FixedTransform
{
    private Fixed4x4 _matrix;
    private FixedTransform? _parent;

    /// <summary>
    /// Initializes a new transform from fixed-point position, rotation, and scale components.
    /// </summary>
    public FixedTransform(
        Vector3d position,
        FixedQuaternion rotation,
        Vector3d scale,
        FixedTransform? parent = null)
    {
        _matrix = Fixed4x4.CreateTransform(position, rotation, scale);
        _parent = parent;
    }

    /// <summary>
    /// Initializes a new transform from an existing fixed-point transformation matrix.
    /// </summary>
    public FixedTransform(Fixed4x4 matrix, FixedTransform? parent = null)
    {
        _matrix = matrix;
        _parent = parent;
    }

    /// <summary>
    /// Gets or sets the transform translation component.
    /// </summary>
    public Vector3d Position
    {
        get => _matrix.Translation;
        set => _matrix.SetTranslation(value);
    }

    /// <summary>
    /// Gets or sets the transform rotation component.
    /// </summary>
    public FixedQuaternion Rotation
    {
        get => _matrix.Rotation;
        set => _matrix.SetRotation(value);
    }

    /// <summary>
    /// Gets or sets the transform global scale component.
    /// </summary>
    public Vector3d Scale
    {
        get => _matrix.Scale;
        set => _matrix.SetGlobalScale(value);
    }

    /// <summary>
    /// Gets the current matrix scale used by this transform.
    /// </summary>
    public Vector3d LossyScale => _matrix.Scale;

    /// <summary>
    /// Gets or sets the transform rotation as Euler angles in degrees.
    /// </summary>
    public Vector3d EulerAngles
    {
        get => Rotation.EulerAngles;
        set => Rotation = FixedQuaternion.FromEulerAnglesInDegrees(value.X, value.Y, value.Z);
    }

    /// <summary>
    /// Gets or sets an optional parent transform reference.
    /// </summary>
    public FixedTransform? Parent
    {
        get => _parent;
        set => _parent = value;
    }
}
