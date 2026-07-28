//=======================================================================
// WideConvexPrismRelations.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Support types and helpers for convex prism relations, including finite shape kinds,
/// radial classifications, and high-precision axis representations used in narrow-phase math.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static readonly Signed192 Scale = Signed192.One;
    private const int CylinderPairRadicandWords = 64;
    private const int WideCandidateWords = 160;

    #region Nested Types

    private enum FiniteShapeKind
    {
        Capsule,
        Cylinder,
        Cone,
    }

    internal enum RadialKind
    {
        None,
        Capsule,
        Disk,
    }

    internal enum ShapeSupportFeature
    {
        Axis,
        ConeApex,
        ConeBase,
    }

    internal readonly struct Axis3
    {
        internal readonly Signed320 X;
        internal readonly Signed320 Y;
        internal readonly Signed320 Z;

        internal Axis3(Signed320 x, Signed320 y, Signed320 z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        internal bool IsZero => X.IsZero && Y.IsZero && Z.IsZero;
    }

    private readonly struct RigidAxis3
    {
        internal readonly Signed192 X;
        internal readonly Signed192 Y;
        internal readonly Signed192 Z;
        internal readonly Signed192 RotationDenominator;

        internal RigidAxis3(
            Signed192 x,
            Signed192 y,
            Signed192 z,
            Signed192 rotationDenominator)
        {
            X = x;
            Y = y;
            Z = z;
            RotationDenominator = rotationDenominator;
        }

        internal Axis3 ToWide() =>
            new(
                Signed320.ExtendValue(X),
                Signed320.ExtendValue(Y),
                Signed320.ExtendValue(Z));
    }

    private readonly struct Penetration
    {
        internal readonly Vector3d Normal;
        internal readonly FixedPointAnchor ShapeAnchor;
        internal readonly FixedQuaternion PrismOrientation;
        internal readonly Vector3d PrismOffset;
        internal readonly Fixed64 Depth;
        internal readonly bool DepthIsClamped;

        internal Penetration(
            Vector3d normal,
            FixedPointAnchor shapeAnchor,
            FixedQuaternion prismOrientation,
            Vector3d prismOffset,
            Fixed64 depth,
            bool depthIsClamped)
        {
            Normal = normal;
            ShapeAnchor = shapeAnchor;
            PrismOrientation = prismOrientation;
            PrismOffset = prismOffset;
            Depth = depth;
            DepthIsClamped = depthIsClamped;
        }
    }

    private readonly struct PenetrationCandidate
    {
        internal readonly Axis3 Axis;
        internal readonly bool Negate;
        internal readonly ShapeSupportFeature SupportFeature;
        internal readonly Vector3d PrismOffset;
        internal readonly bool PrismPlanarTie;
        internal readonly ProjectionDepth ExactDepth;

        internal PenetrationCandidate(
            Axis3 axis,
            bool negate,
            ShapeSupportFeature supportFeature,
            Vector3d prismOffset,
            bool prismPlanarTie,
            ProjectionDepth exactDepth)
        {
            Axis = axis;
            Negate = negate;
            SupportFeature = supportFeature;
            PrismOffset = prismOffset;
            PrismPlanarTie = prismPlanarTie;
            ExactDepth = exactDepth;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    internal readonly struct ProjectionDepth
    {
        internal readonly Signed704 Rational;
        internal readonly Signed192 Common;
        internal readonly Fixed64 Radius;
        internal readonly RadialKind RadialKind;
        internal readonly Signed576 AxisSquared;
        internal readonly Signed320 ShapeAxisSquared;
        internal readonly Signed832 PlaneSquared;
        internal readonly ShapeSupportFeature SupportFeature;

        internal ProjectionDepth(
            Signed704 rational,
            Signed192 common,
            Fixed64 radius,
            RadialKind radialKind,
            Signed576 axisSquared,
            Signed320 shapeAxisSquared,
            Signed832 planeSquared,
            ShapeSupportFeature supportFeature = ShapeSupportFeature.Axis)
        {
            Rational = rational;
            Common = common;
            Radius = radius;
            RadialKind = radialKind;
            AxisSquared = axisSquared;
            ShapeAxisSquared = shapeAxisSquared;
            PlaneSquared = planeSquared;
            SupportFeature = supportFeature;
        }
    }

    internal readonly struct CylinderCylinderPenetration
    {
        internal readonly Axis3 Axis;
        internal readonly bool Negate;
        internal readonly CylinderPairDepth Depth;
        internal readonly Vector3d WideNormal;
        internal readonly Fixed64 WideDepth;
        internal readonly bool WideDepthIsClamped;
        internal readonly bool IsWide;

        internal CylinderCylinderPenetration(
            Axis3 axis,
            bool negate,
            CylinderPairDepth depth)
        {
            Axis = axis;
            Negate = negate;
            Depth = depth;
            WideNormal = default;
            WideDepth = default;
            WideDepthIsClamped = false;
            IsWide = false;
            HasValue = true;
        }

        internal CylinderCylinderPenetration(
            Vector3d normal,
            Fixed64 depth,
            bool depthIsClamped)
        {
            Axis = default;
            Negate = false;
            Depth = default;
            WideNormal = normal;
            WideDepth = depth;
            WideDepthIsClamped = depthIsClamped;
            IsWide = true;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    internal readonly struct CylinderPairDepth
    {
        internal readonly Signed704 Rational;
        internal readonly Signed192 Common;
        internal readonly Fixed64 FirstRadius;
        internal readonly Fixed64 SecondRadius;
        internal readonly Signed576 AxisSquared;
        internal readonly Signed320 FirstAxisSquared;
        internal readonly Signed320 SecondAxisSquared;
        internal readonly Signed832 FirstPlaneSquared;
        internal readonly Signed832 SecondPlaneSquared;

        internal CylinderPairDepth(
            Signed704 rational,
            Signed192 common,
            Fixed64 firstRadius,
            Fixed64 secondRadius,
            Signed576 axisSquared,
            Signed320 firstAxisSquared,
            Signed320 secondAxisSquared,
            Signed832 firstPlaneSquared,
            Signed832 secondPlaneSquared)
        {
            Rational = rational;
            Common = common;
            FirstRadius = firstRadius;
            SecondRadius = secondRadius;
            AxisSquared = axisSquared;
            FirstAxisSquared = firstAxisSquared;
            SecondAxisSquared = secondAxisSquared;
            FirstPlaneSquared = firstPlaneSquared;
            SecondPlaneSquared = secondPlaneSquared;
        }

        internal bool TryGetFastDepth(
            out ProjectionDepth depth)
        {
            bool firstZero =
                FirstRadius == Fixed64.Zero
                || FirstPlaneSquared.IsZero;
            bool secondZero =
                SecondRadius == Fixed64.Zero
                || SecondPlaneSquared.IsZero;
            bool firstFull =
                !firstZero
                && IsFullDiskProjection(
                    AxisSquared,
                    FirstAxisSquared,
                    FirstPlaneSquared);
            bool secondFull =
                !secondZero
                && IsFullDiskProjection(
                    AxisSquared,
                    SecondAxisSquared,
                    SecondPlaneSquared);
            if (firstZero && secondZero)
            {
                depth = new ProjectionDepth(
                    Rational,
                    Common,
                    Fixed64.Zero,
                    RadialKind.None,
                    AxisSquared,
                    FirstAxisSquared,
                    default);
                return true;
            }
            if (firstZero || secondZero)
            {
                bool useFirst = secondZero;
                depth = new ProjectionDepth(
                    Rational,
                    Common,
                    useFirst
                        ? FirstRadius
                        : SecondRadius,
                    useFirst
                        ? (firstFull
                            ? RadialKind.Capsule
                            : RadialKind.Disk)
                        : (secondFull
                            ? RadialKind.Capsule
                            : RadialKind.Disk),
                    AxisSquared,
                    useFirst
                        ? FirstAxisSquared
                        : SecondAxisSquared,
                    useFirst
                        ? FirstPlaneSquared
                        : SecondPlaneSquared);
                return true;
            }
            if (firstFull
                && secondFull
                && Fixed64.TryAdd(
                    FirstRadius,
                    SecondRadius,
                    out Fixed64 combinedRadius))
            {
                depth = new ProjectionDepth(
                    Rational,
                    Common,
                    combinedRadius,
                    RadialKind.Capsule,
                    AxisSquared,
                    FirstAxisSquared,
                    default);
                return true;
            }

            depth = default;
            return false;
        }

        internal ProjectionDepth CreateFirstDiskDepth() =>
            new(
                default,
                Common,
                FirstRadius,
                RadialKind.Disk,
                AxisSquared,
                FirstAxisSquared,
                FirstPlaneSquared);

        internal ProjectionDepth CreateSecondDiskDepth() =>
            new(
                default,
                Common,
                SecondRadius,
                RadialKind.Disk,
                AxisSquared,
                SecondAxisSquared,
                SecondPlaneSquared);

        private static bool IsFullDiskProjection(
            Signed576 axisSquared,
            Signed320 shapeAxisSquared,
            Signed832 planeSquared)
        {
            Signed832 fullPlane =
                WideArithmetic.MultiplyNonNegativeToSigned832(
                    Signed832.ExtendValue(
                        axisSquared),
                    shapeAxisSquared);
            return fullPlane.Equals(planeSquared);
        }
    }

    internal readonly ref struct WideCandidateAxis3
    {
        internal readonly ReadOnlySpan<ulong> X;
        internal readonly ReadOnlySpan<ulong> Y;
        internal readonly ReadOnlySpan<ulong> Z;
        internal readonly int XSign;
        internal readonly int YSign;
        internal readonly int ZSign;

        internal WideCandidateAxis3(
            ReadOnlySpan<ulong> x,
            int xSign,
            ReadOnlySpan<ulong> y,
            int ySign,
            ReadOnlySpan<ulong> z,
            int zSign)
        {
            X = x;
            Y = y;
            Z = z;
            XSign = xSign;
            YSign = ySign;
            ZSign = zSign;
        }

        internal bool TryNarrow(out Axis3 axis)
        {
            bool representable =
                TryNarrowComponent(X, XSign, out Signed320 x)
                & TryNarrowComponent(Y, YSign, out Signed320 y)
                & TryNarrowComponent(Z, ZSign, out Signed320 z);
            axis = representable
                ? new Axis3(x, y, z)
                : default;
            return representable;
        }

        private static bool TryNarrowComponent(
            ReadOnlySpan<ulong> magnitude,
            int sign,
            out Signed320 result)
        {
            int length = GetActiveLength(magnitude);
            if (length > 5
                || (length == 5
                    && (magnitude[4] & (1UL << 63)) != 0UL))
            {
                result = default;
                return false;
            }

            Signed320 positive = new(
                length > 4 ? magnitude[4] : 0UL,
                length > 3 ? magnitude[3] : 0UL,
                length > 2 ? magnitude[2] : 0UL,
                length > 1 ? magnitude[1] : 0UL,
                length > 0 ? magnitude[0] : 0UL);
            result = sign < 0
                ? WideArithmetic.SubtractSigned320(default, positive)
                : positive;
            return true;
        }
    }

    internal readonly struct CylinderCapsulePenetration
    {
        internal readonly Axis3 Axis;
        internal readonly bool Negate;
        internal readonly ProjectionDepth ExactDepth;
        internal readonly Vector3d WideNormal;
        internal readonly Fixed64 WideDepth;
        internal readonly bool WideDepthIsClamped;
        internal readonly bool IsWide;

        internal CylinderCapsulePenetration(
            Axis3 axis,
            bool negate,
            ProjectionDepth exactDepth)
        {
            Axis = axis;
            Negate = negate;
            ExactDepth = exactDepth;
            WideNormal = default;
            WideDepth = default;
            WideDepthIsClamped = false;
            IsWide = false;
            HasValue = true;
        }

        internal CylinderCapsulePenetration(
            Vector3d normal,
            Fixed64 depth,
            bool depthIsClamped)
        {
            Axis = default;
            Negate = false;
            ExactDepth = default;
            WideNormal = normal;
            WideDepth = depth;
            WideDepthIsClamped = depthIsClamped;
            IsWide = true;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    private readonly ref struct WideCylinderCapsuleDepth
    {
        internal readonly ReadOnlySpan<ulong> Rational;
        internal readonly int RationalSign;
        internal readonly Signed192 Common;
        internal readonly Fixed64 Radius;
        internal readonly ReadOnlySpan<ulong> AxisSquared;
        internal readonly Signed320 ShapeAxisSquared;
        internal readonly ReadOnlySpan<ulong> PlaneSquared;

        internal WideCylinderCapsuleDepth(
            ReadOnlySpan<ulong> rational,
            int rationalSign,
            Signed192 common,
            Fixed64 radius,
            ReadOnlySpan<ulong> axisSquared,
            Signed320 shapeAxisSquared,
            ReadOnlySpan<ulong> planeSquared)
        {
            Rational = rational;
            RationalSign = rationalSign;
            Common = common;
            Radius = radius;
            AxisSquared = axisSquared;
            ShapeAxisSquared = shapeAxisSquared;
            PlaneSquared = planeSquared;
        }
    }

    private readonly ref struct WideCylinderPairDepth
    {
        internal readonly ReadOnlySpan<ulong> Rational;
        internal readonly int RationalSign;
        internal readonly Signed192 Common;
        internal readonly Fixed64 FirstRadius;
        internal readonly Fixed64 SecondRadius;
        internal readonly ReadOnlySpan<ulong> AxisSquared;
        internal readonly Signed320 FirstAxisSquared;
        internal readonly Signed320 SecondAxisSquared;
        internal readonly ReadOnlySpan<ulong> FirstPlaneSquared;
        internal readonly ReadOnlySpan<ulong> SecondPlaneSquared;

        internal WideCylinderPairDepth(
            ReadOnlySpan<ulong> rational,
            int rationalSign,
            Signed192 common,
            Fixed64 firstRadius,
            Fixed64 secondRadius,
            ReadOnlySpan<ulong> axisSquared,
            Signed320 firstAxisSquared,
            Signed320 secondAxisSquared,
            ReadOnlySpan<ulong> firstPlaneSquared,
            ReadOnlySpan<ulong> secondPlaneSquared)
        {
            Rational = rational;
            RationalSign = rationalSign;
            Common = common;
            FirstRadius = firstRadius;
            SecondRadius = secondRadius;
            AxisSquared = axisSquared;
            FirstAxisSquared = firstAxisSquared;
            SecondAxisSquared = secondAxisSquared;
            FirstPlaneSquared = firstPlaneSquared;
            SecondPlaneSquared = secondPlaneSquared;
        }
    }

    #endregion

    private static Penetration MaterializeContact(
        in PenetrationCandidate candidate,
        FiniteShapeKind shapeKind,
        RigidAxis3 shapeAxis,
        Vector3d center,
        FixedQuaternion shapeRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d prismOrigin,
        FixedQuaternion prismOrientation,
        ReadOnlySpan<Vector2d> prismOffsets,
        Fixed64 prismHalfThickness)
    {
        Axis3 orientedAxis = candidate.Negate
            ? Negate(candidate.Axis)
            : candidate.Axis;
        Vector3d normal = WideGeometry.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        int axialSign = GetAxisProjection(
            orientedAxis,
            shapeAxis).Sign;
        FixedPointAnchor shapeAnchor = GetShapeSupportAnchor(
            shapeKind,
            center,
            axisLength,
            radius,
            shapeRotation,
            localAxisDirection,
            normal,
            axialSign,
            candidate.SupportFeature);
        GetMatchedPrismOffset(
            shapeAnchor,
            prismOrigin,
            prismOrientation,
            prismOffsets,
            prismHalfThickness,
            candidate.PrismOffset,
            orientedAxis,
            candidate.PrismPlanarTie,
            out Vector3d prismOffset);
        Fixed64 depth = GetRoundedDepth(
            candidate.ExactDepth,
            out bool depthIsClamped);
        return new Penetration(
            normal,
            shapeAnchor,
            prismOrientation,
            prismOffset,
            depth,
            depthIsClamped);
    }

    private static void GetMatchedPrismOffset(
        FixedPointAnchor shapeAnchor,
        Vector3d prismOrigin,
        FixedQuaternion prismOrientation,
        ReadOnlySpan<Vector2d> prismOffsets,
        Fixed64 prismHalfThickness,
        Vector3d fallbackPrismOffset,
        Axis3 normalAxis,
        bool planarSupportTie,
        out Vector3d prismOffset)
    {
        if (!shapeAnchor.TryGetLocalPointIn(
                prismOrigin,
                prismOrientation,
                out Vector3d localPoint))
        {
            prismOffset = fallbackPrismOffset;
            return;
        }

        var planarPoint = new Vector2d(localPoint.X, localPoint.Z);
        Vector2d planarOffset;
        bool verticalNormal = normalAxis.X.IsZero
            && normalAxis.Z.IsZero;
        if (verticalNormal
            && FixedConvex2dRelations.ContainsPoint(
                planarPoint,
                Vector2d.Zero,
                prismOffsets))
        {
            planarOffset = planarPoint;
        }
        else if (verticalNormal | planarSupportTie)
        {
            planarOffset = FixedConvex2dRelations.GetClosestPointOffset(
                planarPoint,
                Vector2d.Zero,
                prismOffsets);
        }
        else
        {
            planarOffset = new Vector2d(
                fallbackPrismOffset.X,
                fallbackPrismOffset.Z);
        }

        Fixed64 y = normalAxis.Y.IsZero
            ? FixedMath.Clamp(
                localPoint.Y,
                -prismHalfThickness,
                prismHalfThickness)
            : fallbackPrismOffset.Y;
        prismOffset = new Vector3d(planarOffset.X, y, planarOffset.Y);
    }

    private static FixedPointAnchor GetShapeSupportAnchor(
        FiniteShapeKind shapeKind,
        Vector3d center,
        Fixed64 axisLength,
        Fixed64 radius,
        FixedQuaternion shapeRotation,
        Vector3d localAxisDirection,
        Vector3d direction,
        int axialSign,
        ShapeSupportFeature supportFeature) =>
        shapeKind switch
        {
            FiniteShapeKind.Capsule =>
                WideGeometry.GetCenteredCapsuleSupportAnchor(
                    center,
                    shapeRotation,
                    localAxisDirection,
                    axisLength,
                    radius,
                    direction,
                    axialSign),
            FiniteShapeKind.Cylinder =>
                WideGeometry.GetCenteredCylinderSupportAnchor(
                    center,
                    shapeRotation,
                    localAxisDirection,
                    axisLength,
                    radius,
                    direction,
                    axialSign),
            _ => WideGeometry.GetCenteredConeSupportAnchor(
                    center,
                    shapeRotation,
                    localAxisDirection,
                axisLength,
                radius,
                direction,
                supportFeature == ShapeSupportFeature.ConeApex),
        };

    private static Axis3 FromDirection(Vector3d direction) =>
        new(
            Signed320.ExtendValue(Signed192.Raw(direction.X)),
            Signed320.ExtendValue(Signed192.Raw(direction.Y)),
            Signed320.ExtendValue(Signed192.Raw(direction.Z)));

    private static int Compare(Signed576 left, Signed576 right) =>
        WideArithmetic.SubtractSigned576(left, right).Sign;

    private static Axis3 Negate(Axis3 axis) =>
        new(
            WideArithmetic.Negate(axis.X),
            WideArithmetic.Negate(axis.Y),
            WideArithmetic.Negate(axis.Z));
}
