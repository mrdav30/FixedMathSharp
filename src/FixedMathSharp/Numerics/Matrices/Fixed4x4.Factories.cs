//=======================================================================
// Fixed4x4.Factories.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed4x4
{
    #region Static Matrix Generators and Transformations

    /// <summary>
    /// Creates a translation matrix from the specified 3-dimensional vector.
    /// </summary>
    /// <param name="position"></param>
    /// <returns>The translation matrix.</returns>
    public static Fixed4x4 CreateTranslation(Vector3d position)
    {
        Fixed4x4 result = default;
        result.M11 = Fixed64.One;
        result.M12 = Fixed64.Zero;
        result.M13 = Fixed64.Zero;
        result.M14 = Fixed64.Zero;
        result.M21 = Fixed64.Zero;
        result.M22 = Fixed64.One;
        result.M23 = Fixed64.Zero;
        result.M24 = Fixed64.Zero;
        result.M31 = Fixed64.Zero;
        result.M32 = Fixed64.Zero;
        result.M33 = Fixed64.One;
        result.M34 = Fixed64.Zero;
        result.M41 = position.X;
        result.M42 = position.Y;
        result.M43 = position.Z;
        result.M44 = Fixed64.One;
        return result;
    }

    /// <summary>
    /// Creates a translation matrix from the specified coordinates.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateTranslation(Fixed64 x, Fixed64 y, Fixed64 z) =>
        CreateTranslation(new Vector3d(x, y, z));

    /// <summary>
    /// Creates a rotation matrix from a quaternion.
    /// </summary>
    /// <param name="rotation">The quaternion representing the rotation.</param>
    /// <returns>A 4x4 matrix representing the rotation.</returns>
    public static Fixed4x4 CreateRotation(FixedQuaternion rotation)
    {
        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

        return new Fixed4x4(
            rotationMatrix.M11, rotationMatrix.M12, rotationMatrix.M13, Fixed64.Zero,
            rotationMatrix.M21, rotationMatrix.M22, rotationMatrix.M23, Fixed64.Zero,
            rotationMatrix.M31, rotationMatrix.M32, rotationMatrix.M33, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
    }

    /// <summary>
    /// Creates a rotation matrix around the X axis.
    /// </summary>
    public static Fixed4x4 CreateRotationX(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationX(angle));

    /// <summary>
    /// Creates a rotation matrix around the Y axis.
    /// </summary>
    public static Fixed4x4 CreateRotationY(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationY(angle));

    /// <summary>
    /// Creates a rotation matrix around the Z axis.
    /// </summary>
    public static Fixed4x4 CreateRotationZ(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationZ(angle));

    /// <summary>
    /// Creates a rotation matrix from an axis and angle.
    /// </summary>
    public static Fixed4x4 CreateFromAxisAngle(Vector3d axis, Fixed64 angle) =>
        CreateRotation(FixedQuaternion.FromAxisAngle(axis, angle));

    /// <summary>
    /// Creates a rotation matrix from pitch, yaw, and roll angles in radians.
    /// </summary>
    public static Fixed4x4 CreateFromEulerAngles(Fixed64 pitch, Fixed64 yaw, Fixed64 roll) =>
        CreateRotation(FixedQuaternion.FromEulerAngles(pitch, yaw, roll));

    /// <summary>
    /// Creates a scale matrix from a 3-dimensional vector.
    /// </summary>
    /// <param name="scale">The vector representing the scale along each axis.</param>
    /// <returns>A 4x4 matrix representing the scale transformation.</returns>
    public static Fixed4x4 CreateScale(Vector3d scale) =>
        new(
            scale.X, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, scale.Y, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, scale.Z, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

    /// <summary>
    /// Creates a uniform scale matrix.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateScale(Fixed64 scale) => CreateScale(new Vector3d(scale, scale, scale));

    /// <summary>
    /// Creates a non-uniform scale matrix from individual scale components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateScale(Fixed64 x, Fixed64 y, Fixed64 z) => CreateScale(new Vector3d(x, y, z));

    /// <summary>
    /// Creates a view matrix looking from a camera position toward a target.
    /// </summary>
    /// <remarks>
    /// The view direction is expressed in FixedMathSharp's canonical <c>+Z</c>-forward basis and
    /// the resulting matrix follows the row-vector convention used by <see cref="TransformPoint(Fixed4x4, Vector3d)"/>.
    /// Convert engine-specific camera conventions at adapter boundaries.
    /// </remarks>
    public static Fixed4x4 CreateLookAt(Vector3d cameraPosition, Vector3d cameraTarget, Vector3d cameraUpVector)
    {
        Vector3d forward = cameraTarget - cameraPosition;

        if (forward.MagnitudeSquared == Fixed64.Zero)
            throw new ArgumentException("Camera position and target must be different.");

        forward = forward.NormalizeInPlace();
        Vector3d right = Vector3d.Cross(cameraUpVector, forward);

        if (right.MagnitudeSquared == Fixed64.Zero)
            throw new ArgumentException("Camera up vector must not be parallel to the view direction.");

        right = right.NormalizeInPlace();
        Vector3d up = Vector3d.Cross(forward, right).NormalizeInPlace();

        return new Fixed4x4(
            right.X, right.Y, right.Z, Fixed64.Zero,
            up.X, up.Y, up.Z, Fixed64.Zero,
            forward.X, forward.Y, forward.Z, Fixed64.Zero,
            -Vector3d.Dot(right, cameraPosition),
            -Vector3d.Dot(up, cameraPosition),
            -Vector3d.Dot(forward, cameraPosition),
            Fixed64.One);
    }

    /// <summary>
    /// Creates an orthographic projection matrix centered on the origin.
    /// </summary>
    public static Fixed4x4 CreateOrthographic(Fixed64 width, Fixed64 height, Fixed64 zNearPlane, Fixed64 zFarPlane)
    {
        if (width <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(width), "Width must be greater than zero.");

        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height), "Height must be greater than zero.");

        Fixed64 halfWidth = width * Fixed64.Half;
        Fixed64 halfHeight = height * Fixed64.Half;
        return CreateOrthographicOffCenter(-halfWidth, halfWidth, -halfHeight, halfHeight, zNearPlane, zFarPlane);
    }

    /// <summary>
    /// Creates an off-center orthographic projection matrix.
    /// </summary>
    public static Fixed4x4 CreateOrthographicOffCenter(
        Fixed64 left,
        Fixed64 right,
        Fixed64 bottom,
        Fixed64 top,
        Fixed64 zNearPlane,
        Fixed64 zFarPlane)
    {
        if (left == right)
            throw new ArgumentOutOfRangeException(nameof(right), "Right must be different from left.");

        if (bottom == top)
            throw new ArgumentOutOfRangeException(nameof(top), "Top must be different from bottom.");

        ValidateDepthRange(zNearPlane, zFarPlane);

        Fixed64 width = right - left;
        Fixed64 height = top - bottom;
        Fixed64 depth = zFarPlane - zNearPlane;

        return new Fixed4x4(
            Fixed64.Two / width, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Two / height, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One / depth, Fixed64.Zero,
            (left + right) / (left - right),
            (top + bottom) / (bottom - top),
            -zNearPlane / depth,
            Fixed64.One);
    }

    /// <summary>
    /// Creates a perspective projection matrix centered on the near plane.
    /// </summary>
    public static Fixed4x4 CreatePerspective(
        Fixed64 width,
        Fixed64 height,
        Fixed64 nearPlaneDistance,
        Fixed64 farPlaneDistance)
    {
        if (width <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(width), "Width must be greater than zero.");

        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height), "Height must be greater than zero.");

        ValidatePerspectiveDepthRange(nearPlaneDistance, farPlaneDistance);

        Fixed64 depth = farPlaneDistance - nearPlaneDistance;
        Fixed64 twoNear = Fixed64.Two * nearPlaneDistance;

        return new Fixed4x4(
            twoNear / width, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, twoNear / height, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, farPlaneDistance / depth, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, -(nearPlaneDistance * farPlaneDistance) / depth, Fixed64.Zero);
    }

    /// <summary>
    /// Creates a perspective projection matrix from a vertical field of view.
    /// </summary>
    public static Fixed4x4 CreatePerspectiveFieldOfView(
        Fixed64 fieldOfView,
        Fixed64 aspectRatio,
        Fixed64 nearPlaneDistance,
        Fixed64 farPlaneDistance)
    {
        if (fieldOfView <= Fixed64.Zero || fieldOfView >= Fixed64.Pi)
            throw new ArgumentOutOfRangeException(nameof(fieldOfView), "Field of view must be greater than zero and less than PI.");

        if (aspectRatio <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(aspectRatio), "Aspect ratio must be greater than zero.");

        ValidatePerspectiveDepthRange(nearPlaneDistance, farPlaneDistance);

        Fixed64 yScale = Fixed64.One / FixedMath.Tan(fieldOfView * Fixed64.Half);
        Fixed64 xScale = yScale / aspectRatio;
        Fixed64 depth = farPlaneDistance - nearPlaneDistance;

        return new Fixed4x4(
            xScale, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, yScale, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, farPlaneDistance / depth, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, -(nearPlaneDistance * farPlaneDistance) / depth, Fixed64.Zero);
    }

    /// <summary>
    /// Creates an off-center perspective projection matrix.
    /// </summary>
    public static Fixed4x4 CreatePerspectiveOffCenter(
        Fixed64 left,
        Fixed64 right,
        Fixed64 bottom,
        Fixed64 top,
        Fixed64 nearPlaneDistance,
        Fixed64 farPlaneDistance)
    {
        if (left == right)
            throw new ArgumentOutOfRangeException(nameof(right), "Right must be different from left.");

        if (bottom == top)
            throw new ArgumentOutOfRangeException(nameof(top), "Top must be different from bottom.");

        ValidatePerspectiveDepthRange(nearPlaneDistance, farPlaneDistance);

        Fixed64 width = right - left;
        Fixed64 height = top - bottom;
        Fixed64 depth = farPlaneDistance - nearPlaneDistance;
        Fixed64 twoNear = Fixed64.Two * nearPlaneDistance;

        return new Fixed4x4(
            twoNear / width, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, twoNear / height, Fixed64.Zero, Fixed64.Zero,
            (left + right) / (left - right),
            (top + bottom) / (bottom - top),
            farPlaneDistance / depth,
            Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, -(nearPlaneDistance * farPlaneDistance) / depth, Fixed64.Zero);
    }

    /// <summary>
    /// Creates a world matrix from a position and orientation basis.
    /// </summary>
    /// <remarks>
    /// The <paramref name="forward"/> and <paramref name="up"/> vectors are semantic basis vectors
    /// in FixedMathSharp's canonical coordinate space. The returned matrix stores right, up, and
    /// forward basis rows for row-vector transforms.
    /// </remarks>
    public static Fixed4x4 CreateWorld(Vector3d position, Vector3d forward, Vector3d up)
    {
        if (forward.MagnitudeSquared == Fixed64.Zero)
            throw new ArgumentException("Forward vector must be non-zero.");

        forward = forward.NormalizeInPlace();
        Vector3d right = Vector3d.Cross(up, forward);

        if (right.MagnitudeSquared == Fixed64.Zero)
            throw new ArgumentException("Up vector must not be parallel to forward.");

        right = right.NormalizeInPlace();
        up = Vector3d.Cross(forward, right).NormalizeInPlace();

        return new Fixed4x4(
            right.X, right.Y, right.Z, Fixed64.Zero,
            up.X, up.Y, up.Z, Fixed64.Zero,
            forward.X, forward.Y, forward.Z, Fixed64.Zero,
            position.X, position.Y, position.Z, Fixed64.One);
    }

    /// <summary>
    /// Constructs a transformation matrix from translation, scale, and rotation.
    /// This method ensures that the rotation is properly normalized, applies the scale to the
    /// rotational basis, and sets the translation component separately.
    /// </summary>
    /// <remarks>
    /// - Uses a normalized rotation matrix to maintain numerical stability.
    /// - Applies non-uniform scaling to the rotation before setting translation.
    /// - Preferred when ensuring transformations remain mathematically correct.
    /// - If the rotation is already normalized and combined transformations are needed, consider using <see cref="ScaleRotateTranslate"/>.
    /// </remarks>
    /// <param name="translation">The translation vector.</param>
    /// <param name="scale">The scale vector.</param>
    /// <param name="rotation">The rotation quaternion.</param>
    /// <returns>A transformation matrix incorporating translation, rotation, and scale.</returns>
    public static Fixed4x4 CreateTransform(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
    {
        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

        return new Fixed4x4(
            rotationMatrix.M11 * scale.X, rotationMatrix.M12 * scale.X, rotationMatrix.M13 * scale.X, Fixed64.Zero,
            rotationMatrix.M21 * scale.Y, rotationMatrix.M22 * scale.Y, rotationMatrix.M23 * scale.Y, Fixed64.Zero,
            rotationMatrix.M31 * scale.Z, rotationMatrix.M32 * scale.Z, rotationMatrix.M33 * scale.Z, Fixed64.Zero,
            translation.X, translation.Y, translation.Z, Fixed64.One);
    }

    /// <summary>
    /// Constructs a transformation matrix from translation, rotation, and scale by multiplying
    /// separate matrices in the order: Scale * Rotation * Translation.
    /// </summary>
    /// <remarks>
    /// - This method directly multiplies the scale, rotation, and translation matrices.
    /// - Ensures that scale is applied first to preserve correct axis scaling.
    /// - Then rotation is applied so that rotation is not affected by non-uniform scaling.
    /// - Finally, translation moves the object to its correct world position.
    /// </remarks>
    public static Fixed4x4 ScaleRotateTranslate(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
    {
        return CreateTransform(translation, rotation, scale);
    }

    /// <summary>
    /// Constructs a transformation matrix from translation, rotation, and scale by multiplying
    /// matrices in the order: Translation * Rotation * Scale (T * R * S).
    /// </summary>
    /// <remarks>
    /// - Use this method when transformations need to be applied **relative to an object's local origin**.
    /// - Example use cases include **animation systems**, **hierarchical transformations**, and **UI transformations**.
    /// - If you need to apply world-space transformations, use <see cref="CreateTransform"/> instead.
    /// </remarks>
    public static Fixed4x4 TranslateRotateScale(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
    {
        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

        return new Fixed4x4(
            rotationMatrix.M11 * scale.X, rotationMatrix.M12 * scale.Y, rotationMatrix.M13 * scale.Z, Fixed64.Zero,
            rotationMatrix.M21 * scale.X, rotationMatrix.M22 * scale.Y, rotationMatrix.M23 * scale.Z, Fixed64.Zero,
            rotationMatrix.M31 * scale.X, rotationMatrix.M32 * scale.Y, rotationMatrix.M33 * scale.Z, Fixed64.Zero,
            (translation.X * rotationMatrix.M11 + translation.Y * rotationMatrix.M21 + translation.Z * rotationMatrix.M31) * scale.X,
            (translation.X * rotationMatrix.M12 + translation.Y * rotationMatrix.M22 + translation.Z * rotationMatrix.M32) * scale.Y,
            (translation.X * rotationMatrix.M13 + translation.Y * rotationMatrix.M23 + translation.Z * rotationMatrix.M33) * scale.Z,
            Fixed64.One);
    }

    #endregion
}
