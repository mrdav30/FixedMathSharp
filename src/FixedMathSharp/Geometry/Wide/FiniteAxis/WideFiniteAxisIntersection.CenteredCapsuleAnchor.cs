//=======================================================================
// WideFiniteAxisIntersection.CenteredCapsuleAnchor.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides methods for computing surface anchors on capsules centered at a given point,
/// supporting both 2D and 3D representations with rotation and axis-aligned variants.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>Returns an exact world direction from an anchored point toward the finite core in a rigid frame.</summary>
    internal static WideDirection3d GetPointToCenteredCapsuleCoreDirection(in FixedPointAnchor point,
        Vector3d center, FixedQuaternion rotation, Vector3d localAxis, Fixed64 length)
    {
        WideTriangleRelations.GetPointLocalNumerators(point, center, rotation,
            out Signed576 x, out Signed576 y, out Signed576 z, out Signed320 denominator);
        Signed192 squared = GetDot(localAxis, Vector3d.Zero, localAxis, Vector3d.Zero);
        Signed576 projection = WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(x, Signed192.Raw(localAxis.X)),
            WideArithmetic.MultiplySigned576(y, Signed192.Raw(localAxis.Y))),
            WideArithmetic.MultiplySigned576(z, Signed192.Raw(localAxis.Z)));
        Signed576 scaled = WideArithmetic.MultiplySigned576(projection, DoubleParameterScale),
            extent = WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(denominator, squared), Signed192.Raw(length));
        int cap = WideArithmetic.SubtractSigned576(scaled, extent).Sign > 0 ? 1
            : WideArithmetic.AddSigned576(scaled, extent).Sign < 0 ? -1 : 0;
        return new WideDirection3d(Component(x, localAxis.X), Component(y, localAxis.Y), Component(z, localAxis.Z))
            .Transform(new WideRationalBasis3d(rotation));

        Signed576 Component(Signed576 coordinate, Fixed64 axis)
        {
            // Complete anchor/local-frame numerators need <230 bits. Even a
            // full scalar axis and final rigid transform keep products <430.
            if (squared.IsZero) return WideArithmetic.SubtractSigned576(default, coordinate);
            if (cap == 0) return WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned576(projection, Signed192.Raw(axis)),
                WideArithmetic.MultiplySigned576(coordinate, squared));
            Signed192 axial = cap > 0 ? Signed192.Raw(length) : WideArithmetic.Negate(Signed192.Raw(length));
            return WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned576(
                    WideArithmetic.MultiplySigned320(denominator, axial), Signed192.Raw(axis)),
                WideArithmetic.MultiplySigned576(coordinate, DoubleParameterScale));
        }
    }

    internal static FixedPointAnchor2d GetSurfaceAnchorOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Vector2d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d localNormal)
    {
        _ = Vector2d.TryRotate(
            localAxisDirection,
            frameRotation,
            out Vector2d axisDirection);
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisLength,
            out Signed192 numerator,
            out Signed192 denominator);
        _ = TryGetCenteredCapsuleSurfaceCoordinate(
            Fixed64.Zero,
            Fixed64.One,
            numerator,
            denominator,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 axialDistance);
        Vector2d roundedRadialOffset = localNormal * radius;
        return new FixedPointAnchor2d(
            center,
            frameRotation,
            localAxisDirection * axialDistance,
            roundedRadialOffset,
            FixedPointAnchorTerm2d.CreateRadialSupport(
                localNormal,
                radius,
                roundedRadialOffset));
    }

    internal static FixedPointAnchor GetSurfaceAnchorOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localNormal)
    {
        GetRigidLocalPointRelation(
            point,
            center,
            frameRotation,
            localAxisDirection,
            out RigidLocalPointRelation relation);
        int cap = GetCenteredCap(relation, axisLength);
        Fixed64 axialDistance = GetClosestCenteredRigidAxisDistance(
            relation,
            axisLength,
            cap);
        Vector3d roundedRadialOffset = localNormal * radius;
        return new FixedPointAnchor(
            center,
            frameRotation,
            localAxisDirection * axialDistance,
            roundedRadialOffset,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                localNormal,
                radius,
                roundedRadialOffset));
    }

    internal static FixedPointAnchor GetSurfaceAnchorOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d surfaceDirection)
    {
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisLength,
            out Signed192 numerator,
            out Signed192 denominator);
        _ = TryGetCenteredCapsuleSurfaceCoordinate(
            Fixed64.Zero,
            axisDirection.X,
            numerator,
            denominator,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 axialX);
        _ = TryGetCenteredCapsuleSurfaceCoordinate(
            Fixed64.Zero,
            axisDirection.Y,
            numerator,
            denominator,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 axialY);
        _ = TryGetCenteredCapsuleSurfaceCoordinate(
            Fixed64.Zero,
            axisDirection.Z,
            numerator,
            denominator,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 axialZ);
        Vector3d roundedRadialOffset = surfaceDirection * radius;
        return new FixedPointAnchor(
            center,
            FixedQuaternion.Identity,
            new Vector3d(axialX, axialY, axialZ),
            roundedRadialOffset,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                surfaceDirection,
                radius,
                roundedRadialOffset));
    }
}
