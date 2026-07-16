//=======================================================================
// FixedTransform.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// Provides a mutable deterministic transform shell with explicit position, rotation, and scale components.
/// </summary>
/// <remarks>
/// This type intentionally keeps reference semantics so host-owned objects can publish one
/// transform instance while simulation systems mutate its fixed-point components explicitly.
/// </remarks>
public class FixedTransform
{
    private Vector3d _position;
    private FixedQuaternion _rotation;
    private Vector3d _scale;
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
        _position = position;
        _rotation = rotation.Normalized;
        _scale = scale;
        _parent = parent;
    }

    /// <summary>
    /// Initializes a new X/Z planar transform at zero elevation with unit Y scale.
    /// </summary>
    public FixedTransform(
        Vector2d positionXZ,
        Fixed64 rotationXZRadians,
        Vector2d scaleXZ,
        FixedTransform? parent = null)
        : this(
            positionXZ.ToVector3d(Fixed64.Zero),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -rotationXZRadians),
            scaleXZ.ToVector3d(Fixed64.One),
            parent)
    { }

    /// <summary>
    /// Initializes a new transform from an existing fixed-point transformation matrix.
    /// </summary>
    public FixedTransform(Fixed4x4 matrix, FixedTransform? parent = null)
    {
        Fixed4x4.Decompose(matrix, out _position, out _rotation, out _scale);
        _rotation = _rotation.Normalized;
        _parent = parent;
    }

    /// <summary>
    /// Gets or sets the transform translation component.
    /// </summary>
    public Vector3d Position
    {
        get => _position;
        set => _position = value;
    }

    /// <summary>
    /// Gets or sets the transform rotation component.
    /// </summary>
    public FixedQuaternion Rotation
    {
        get => _rotation;
        set => _rotation = value.Normalized;
    }

    /// <summary>
    /// Gets or sets the transform scale component.
    /// </summary>
    public Vector3d Scale
    {
        get => _scale;
        set => _scale = value;
    }

    /// <summary>
    /// Gets or sets the transform position projected onto the X/Z plane.
    /// </summary>
    public Vector2d PositionXZ
    {
        get => _position.ToVector2d();
        set => _position = value.ToVector3d(_position.Y);
    }

    /// <summary>
    /// Gets or sets the signed X/Z rotation in radians from planar right toward planar forward.
    /// </summary>
    /// <remarks>
    /// Setting this property replaces the complete 3D rotation with a pure Y-axis rotation.
    /// Getting it projects the rotated local-right direction onto X/Z and returns zero when that
    /// projection has zero length.
    /// </remarks>
    public Fixed64 RotationXZRadians
    {
        get
        {
            Vector3d localRight = _rotation.Rotate(Vector3d.Right);
            return FixedMath.Atan2(localRight.Z, localRight.X);
        }
        set => Rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -value);
    }

    /// <summary>
    /// Gets or sets the transform scale projected onto the X/Z plane.
    /// </summary>
    public Vector2d ScaleXZ
    {
        get => _scale.ToVector2d();
        set => _scale = value.ToVector3d(_scale.Y);
    }

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
