//=======================================================================
// Fixed4x4.Factories.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Factory methods for constructing <see cref="Fixed4x4"/> matrices, including
/// translation, rotation, scale, and other common transformation matrices.
/// </content>
public partial struct Fixed4x4
{
    #region Static Matrix Generators and Transformations

    /// <summary>
    /// Creates a translation matrix from the specified 3-dimensional vector.
    /// </summary>
    /// <returns>The translation matrix.</returns>
    /// <param name="position">The translation offsets along X, Y, and Z.</param>
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
    /// <param name="x">The X translation offset.</param>
    /// <param name="y">The Y translation offset.</param>
    /// <param name="z">The Z translation offset.</param>
    /// <returns>A row-vector matrix that applies the specified translation.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateTranslation(Fixed64 x, Fixed64 y, Fixed64 z) =>
        CreateTranslation(new Vector3d(x, y, z));

    /// <summary>
    /// Creates a rotation matrix from a quaternion.
    /// </summary>
    /// <remarks>Quaternion magnitude does not affect the result; zero represents identity.</remarks>
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
    /// <param name="angle">The rotation angle in radians.</param>
    /// <returns>A row-vector rotation matrix.</returns>
    public static Fixed4x4 CreateRotationX(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationX(angle));

    /// <summary>
    /// Creates a rotation matrix around the Y axis.
    /// </summary>
    /// <param name="angle">The rotation angle in radians.</param>
    /// <returns>A row-vector rotation matrix.</returns>
    public static Fixed4x4 CreateRotationY(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationY(angle));

    /// <summary>
    /// Creates a rotation matrix around the Z axis.
    /// </summary>
    /// <param name="angle">The rotation angle in radians.</param>
    /// <returns>A row-vector rotation matrix.</returns>
    public static Fixed4x4 CreateRotationZ(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationZ(angle));

    /// <summary>
    /// Creates a rotation matrix from an axis and angle.
    /// </summary>
    /// <param name="axis">The rotation axis. A nonzero axis is normalized as needed.</param>
    /// <param name="angle">The rotation angle in radians.</param>
    /// <returns>A row-vector rotation matrix, or the identity matrix for a zero axis.</returns>
    public static Fixed4x4 CreateFromAxisAngle(Vector3d axis, Fixed64 angle) =>
        CreateRotation(FixedQuaternion.FromAxisAngle(axis, angle));

    /// <summary>
    /// Creates a rotation matrix from pitch, yaw, and roll angles in radians.
    /// </summary>
    /// <param name="pitch">The X rotation in radians.</param>
    /// <param name="yaw">The Y rotation in radians.</param>
    /// <param name="roll">The Z rotation in radians.</param>
    /// <returns>A row-vector rotation matrix.</returns>
    /// <remarks>The orientation applies roll, pitch, then yaw.</remarks>
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
    /// <param name="scale">The signed scale factor for all three axes.</param>
    /// <returns>A diagonal matrix with the supplied scale on X, Y, and Z and a homogeneous entry of one.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateScale(Fixed64 scale) => CreateScale(new Vector3d(scale, scale, scale));

    /// <summary>
    /// Creates a non-uniform scale matrix from individual scale components.
    /// </summary>
    /// <param name="x">The signed X scale factor.</param>
    /// <param name="y">The signed Y scale factor.</param>
    /// <param name="z">The signed Z scale factor.</param>
    /// <returns>A diagonal matrix with the supplied scale factors and a homogeneous entry of one.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateScale(Fixed64 x, Fixed64 y, Fixed64 z) => CreateScale(new Vector3d(x, y, z));

    /// <summary>
    /// Creates a world-to-view matrix from a camera position, target, and preferred up direction.
    /// </summary>
    /// <remarks>
    /// For row-vector transforms the camera basis is stored in columns, so each output
    /// coordinate projects the input onto the corresponding camera axis. Translation
    /// subtracts the camera position in view space. Basis construction and translation
    /// use rounded, saturating fixed-point arithmetic.
    /// </remarks>
    /// <param name="cameraPosition">The camera position in world coordinates.</param>
    /// <param name="cameraTarget">The look target in world coordinates.</param>
    /// <param name="cameraUpVector">The preferred up direction in world coordinates.</param>
    /// <returns>A row-vector affine view matrix with +Z forward.</returns>
    /// <exception cref="ArgumentException">The view direction or its cross product with the up vector has zero squared magnitude.</exception>
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
            right.X, up.X, forward.X, Fixed64.Zero,
            right.Y, up.Y, forward.Y, Fixed64.Zero,
            right.Z, up.Z, forward.Z, Fixed64.Zero,
            -Vector3d.Dot(right, cameraPosition),
            -Vector3d.Dot(up, cameraPosition),
            -Vector3d.Dot(forward, cameraPosition),
            Fixed64.One);
    }

    /// <summary>
    /// Creates an orthographic projection matrix centered on the origin.
    /// </summary>
    /// <param name="width">The positive horizontal span in view-space units. Its fixed-point half must be nonzero.</param>
    /// <param name="height">The positive vertical span in view-space units. Its fixed-point half must be nonzero.</param>
    /// <param name="zNearPlane">The nonnegative near plane distance along +Z.</param>
    /// <param name="zFarPlane">The far plane distance along +Z, strictly greater than the near plane distance.</param>
    /// <returns>A +Z-forward row-vector orthographic matrix using the zero-to-one depth convention.</returns>
    /// <remarks>Fixed-point rounding and saturation of the coefficients can affect the projected depth.</remarks>
    /// <exception cref="ArgumentOutOfRangeException">A span is nonpositive or rounds to zero when halved, the near distance is negative, or the far distance is at or below the near distance.</exception>
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
    /// <param name="left">The X coordinate of the left edge.</param>
    /// <param name="right">The X coordinate of the right edge. It must differ from the left edge.</param>
    /// <param name="bottom">The Y coordinate of the bottom edge.</param>
    /// <param name="top">The Y coordinate of the top edge. It must differ from the bottom edge.</param>
    /// <param name="zNearPlane">The nonnegative near plane distance along +Z.</param>
    /// <param name="zFarPlane">The far plane distance along +Z, strictly greater than the near plane distance.</param>
    /// <returns>A +Z-forward row-vector orthographic matrix using the zero-to-one depth convention.</returns>
    /// <remarks>Fixed-point rounding and saturation of the coefficients can affect the projected depth.</remarks>
    /// <exception cref="ArgumentOutOfRangeException">The horizontal or vertical endpoints coincide, the near distance is negative, or the far distance is at or below the near distance.</exception>
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
    /// <param name="width">The positive width of the near plane in view-space units.</param>
    /// <param name="height">The positive height of the near plane in view-space units.</param>
    /// <param name="nearPlaneDistance">The positive near plane distance along +Z.</param>
    /// <param name="farPlaneDistance">The far plane distance along +Z, strictly greater than the near plane distance.</param>
    /// <returns>A +Z-forward row-vector perspective matrix using the zero-to-one depth convention.</returns>
    /// <remarks>Each coefficient is rounded once, with saturation at the representable endpoints.
    /// Applying the matrix is also subject to the range of the point-transform arithmetic.</remarks>
    /// <exception cref="ArgumentOutOfRangeException">The width, height, or near distance is nonpositive, or the far distance is at or below the near distance.</exception>
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

        return BuildPerspective(
            Fixed64.MultiplyDivide(Fixed64.Two, nearPlaneDistance, width, out _),
            Fixed64.MultiplyDivide(Fixed64.Two, nearPlaneDistance, height, out _),
            Fixed64.Zero, Fixed64.Zero, nearPlaneDistance, farPlaneDistance);
    }

    /// <summary>
    /// Creates a perspective projection matrix from a vertical field of view.
    /// </summary>
    /// <param name="fieldOfView">The vertical field of view in radians, strictly between zero and pi.
    /// Its rounded half angle must be strictly between zero and pi/2.</param>
    /// <param name="aspectRatio">The positive ratio of width to height.</param>
    /// <param name="nearPlaneDistance">The positive near plane distance along +Z.</param>
    /// <param name="farPlaneDistance">The far plane distance along +Z, strictly greater than the near plane distance.</param>
    /// <returns>A +Z-forward row-vector perspective matrix using the zero-to-one depth convention.</returns>
    /// <remarks>The horizontal and vertical coefficients are rounded and saturated independently
    /// from the fixed-point sine and cosine of the half angle. Applying the matrix is also
    /// subject to the range of the point-transform arithmetic.</remarks>
    /// <exception cref="ArgumentOutOfRangeException">The field of view or its rounded half angle is outside
    /// its open interval, the aspect ratio or near distance is nonpositive, or the far distance is at or below the near distance.</exception>
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

        Fixed64 halfAngle = fieldOfView * Fixed64.Half;
        if (halfAngle == Fixed64.Zero || halfAngle == Fixed64.HalfPi)
            throw new ArgumentOutOfRangeException(nameof(fieldOfView), "The rounded half angle must be greater than zero and less than PI/2.");

        Fixed64 sine = FixedMath.Sin(halfAngle);
        Fixed64 cosine = FixedMath.Sin(Fixed64.HalfPi - halfAngle);
        Fixed64 yScale = cosine / sine;
        // sin(halfAngle) * aspect has at most 95 raw bits at Q64.64 scale.
        Signed192 horizontalDenominator = Signed192.NarrowValue(
            WideArithmetic.MultiplySigned192(Signed192.Raw(sine), Signed192.Raw(aspectRatio)));
        Fixed64 xScale = Fixed64.GetSignedRatio(
            WideArithmetic.ShiftLeftSigned192By32(Signed192.Raw(cosine)), horizontalDenominator);
        return BuildPerspective(xScale, yScale, Fixed64.Zero, Fixed64.Zero, nearPlaneDistance, farPlaneDistance);
    }

    /// <summary>
    /// Creates an off-center perspective projection matrix.
    /// </summary>
    /// <param name="left">The X coordinate of the left near-plane edge.</param>
    /// <param name="right">The X coordinate of the right near-plane edge. It must differ from the left edge.</param>
    /// <param name="bottom">The Y coordinate of the bottom near-plane edge.</param>
    /// <param name="top">The Y coordinate of the top near-plane edge. It must differ from the bottom edge.</param>
    /// <param name="nearPlaneDistance">The positive near plane distance along +Z.</param>
    /// <param name="farPlaneDistance">The far plane distance along +Z, strictly greater than the near plane distance.</param>
    /// <returns>A +Z-forward row-vector perspective matrix using the zero-to-one depth convention.</returns>
    /// <remarks>Each coefficient is rounded once, with saturation at the representable endpoints.
    /// Applying the matrix is also subject to the range of the point-transform arithmetic.</remarks>
    /// <exception cref="ArgumentOutOfRangeException">The horizontal or vertical endpoints coincide, the near distance is nonpositive, or the far distance is at or below the near distance.</exception>
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

        Signed192 twoNear = WideArithmetic.Double(Signed192.Raw(nearPlaneDistance));
        GetPerspectiveAxis(left, right, twoNear, out Fixed64 xScale, out Fixed64 xOffset);
        GetPerspectiveAxis(bottom, top, twoNear, out Fixed64 yScale, out Fixed64 yOffset);
        return BuildPerspective(xScale, yScale, xOffset, yOffset, nearPlaneDistance, farPlaneDistance);
    }

    /// <summary>
    /// Creates a world matrix from a position and orientation basis.
    /// </summary>
    /// <remarks>
    /// The <paramref name="forward"/> and <paramref name="up"/> vectors are semantic basis vectors
    /// in FixedMathSharp's canonical coordinate space. The returned matrix stores right, up, and
    /// forward basis rows for row-vector transforms.
    /// </remarks>
    /// <param name="position">The local origin in world coordinates.</param>
    /// <param name="forward">The desired +Z direction in world coordinates.</param>
    /// <param name="up">The preferred +Y direction in world coordinates.</param>
    /// <returns>The row-vector local-to-world matrix with an orthonormal basis, to fixed-point precision.</returns>
    /// <exception cref="ArgumentException">The forward direction or its cross product with the up vector has zero squared magnitude.</exception>
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
    /// Builds an affine matrix that applies local scale, then rotation, then translation.
    /// </summary>
    /// <remarks>
    /// Quaternion magnitude does not affect the rotation matrix. A zero quaternion represents identity.
    /// Use <see cref="TranslateRotateScale"/> to apply the offset before rotation and scaling.
    /// </remarks>
    /// <param name="translation">The offset in the output space.</param>
    /// <param name="rotation">The rotation to apply after scaling.</param>
    /// <param name="scale">The signed scale factors along the input X, Y, and Z axes.</param>
    /// <returns>The composed row-vector affine transform.</returns>
    public static Fixed4x4 CreateTransform(Vector3d translation, FixedQuaternion rotation, Vector3d scale) =>
        CreateTransform(translation, rotation.ToMatrix3x3(), scale);

    private static Fixed4x4 CreateTransform(Vector3d translation, Fixed3x3 rotation, Vector3d scale)
    {
        return new Fixed4x4(
            rotation.M11 * scale.X, rotation.M12 * scale.X, rotation.M13 * scale.X, Fixed64.Zero,
            rotation.M21 * scale.Y, rotation.M22 * scale.Y, rotation.M23 * scale.Y, Fixed64.Zero,
            rotation.M31 * scale.Z, rotation.M32 * scale.Z, rotation.M33 * scale.Z, Fixed64.Zero,
            translation.X, translation.Y, translation.Z, Fixed64.One);
    }

    // Shared checked TRS construction keeps decomposition and hierarchy materialization nonsaturating.
    internal static bool TryCreateTransform(
        Vector3d translation, Fixed3x3 rotation, Vector3d scale, out Fixed4x4 matrix)
    {
        bool representable = Fixed64.TryMultiplyAdd(rotation.M11, scale.X, Fixed64.Zero, out Fixed64 m11)
            & Fixed64.TryMultiplyAdd(rotation.M12, scale.X, Fixed64.Zero, out Fixed64 m12)
            & Fixed64.TryMultiplyAdd(rotation.M13, scale.X, Fixed64.Zero, out Fixed64 m13)
            & Fixed64.TryMultiplyAdd(rotation.M21, scale.Y, Fixed64.Zero, out Fixed64 m21)
            & Fixed64.TryMultiplyAdd(rotation.M22, scale.Y, Fixed64.Zero, out Fixed64 m22)
            & Fixed64.TryMultiplyAdd(rotation.M23, scale.Y, Fixed64.Zero, out Fixed64 m23)
            & Fixed64.TryMultiplyAdd(rotation.M31, scale.Z, Fixed64.Zero, out Fixed64 m31)
            & Fixed64.TryMultiplyAdd(rotation.M32, scale.Z, Fixed64.Zero, out Fixed64 m32)
            & Fixed64.TryMultiplyAdd(rotation.M33, scale.Z, Fixed64.Zero, out Fixed64 m33);
        if (!representable)
        {
            matrix = Fixed4x4.Zero;
            return false;
        }

        matrix = new Fixed4x4(
            m11, m12, m13, Fixed64.Zero,
            m21, m22, m23, Fixed64.Zero,
            m31, m32, m33, Fixed64.Zero,
            translation.X, translation.Y, translation.Z, Fixed64.One);
        return true;
    }

    /// <inheritdoc cref="CreateTransform(Vector3d, FixedQuaternion, Vector3d)"/>
    public static Fixed4x4 ScaleRotateTranslate(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
    {
        return CreateTransform(translation, rotation, scale);
    }

    /// <summary>
    /// Builds an affine matrix that applies translation, then rotation, then scale.
    /// </summary>
    /// <remarks>
    /// The composed translation row includes the effects of rotation and scale on the input offset.
    /// Use <see cref="CreateTransform(Vector3d, FixedQuaternion, Vector3d)"/> to apply the offset last.
    /// </remarks>
    /// <param name="translation">The offset applied before rotation and scale.</param>
    /// <param name="rotation">The rotation to apply after translation. A zero quaternion represents identity.</param>
    /// <param name="scale">The signed scale factors along the final X, Y, and Z axes.</param>
    /// <returns>A row-vector matrix that applies translation, rotation, then scale.</returns>
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
