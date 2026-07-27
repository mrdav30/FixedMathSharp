//=======================================================================
// WideFiniteAxisProjection.CapsuleSlab.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides SAT axis penetration calculations for capsule/cylinder shapes
/// tested against capsule-shaped slabs (e.g. rounded rectangular prisms).
/// </content>
internal static partial class WideFiniteAxisProjection
{
    internal static bool TryGetCapsuleCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d capsuleCenter,
        Vector3d capsuleAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxis,
        Fixed64 slabCapsuleLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        CreateSymmetricCapsuleSlabDepth(
            projectionAxis,
            capsuleCenter,
            capsuleAxis,
            capsuleLength,
            capsuleRadius,
            shapeUsesSphericalRadius: true,
            slabCenter,
            slabCapsuleAxis,
            slabCapsuleLength,
            slabCapsuleRadius,
            slabHalfThickness,
            out ProjectionDepth projectionDepth);
        return TryMaterializeDepth(
            projectionDepth,
            projectionAxis,
            GetDot(slabCenter, capsuleCenter, projectionAxis),
            out orientedAxis,
            out depth,
            out depthIsClamped);
    }

    internal static bool TryGetCylinderCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxis,
        Fixed64 slabCapsuleLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        CreateSymmetricCapsuleSlabDepth(
            projectionAxis,
            cylinderCenter,
            cylinderAxis,
            cylinderLength,
            cylinderRadius,
            shapeUsesSphericalRadius: false,
            slabCenter,
            slabCapsuleAxis,
            slabCapsuleLength,
            slabCapsuleRadius,
            slabHalfThickness,
            out ProjectionDepth projectionDepth);
        return TryMaterializeDepth(
            projectionDepth,
            projectionAxis,
            GetDot(slabCenter, cylinderCenter, projectionAxis),
            out orientedAxis,
            out depth,
            out depthIsClamped);
    }

    internal static bool TryGetConeCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d coneCenter,
        Vector3d coneAxis,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxis,
        Fixed64 slabCapsuleLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Signed192 centerProjection =
            GetDot(slabCenter, coneCenter, projectionAxis);
        Signed192 coneAlignment = GetDot(coneAxis, projectionAxis);
        Signed320 coneAxial = WideArithmetic.MultiplySigned192(
            coneAlignment,
            Signed192.Signed(coneHeight.m_rawValue));
        Radical coneDisk =
            CreateCylinderRadical(projectionAxis, coneAxis, coneRadius);
        Radical slabRadial =
            CreateCapsuleSlabRadical(projectionAxis, slabCapsuleRadius);
        Signed320 slabRational = CreateCapsuleSlabRational(
            projectionAxis,
            slabCapsuleAxis,
            slabCapsuleLength,
            slabHalfThickness);
        Signed320 centerRational = WideArithmetic.MultiplySigned192(
            centerProjection,
            DoubleScale);
        Signed192 denominator = GetRationalDenominator();

        bool maximumUsesBase = coneAxial.Sign <= 0
            || IsRadicalAtLeastRatio(coneDisk, coneAxial);
        Signed320 negativeConeAxial =
            WideArithmetic.SubtractSigned320(default, coneAxial);
        bool minimumUsesBase = negativeConeAxial.Sign <= 0
            || IsRadicalAtLeastRatio(coneDisk, negativeConeAxial);
        Signed320 maximumRational = maximumUsesBase
            ? negativeConeAxial
            : coneAxial;
        Signed320 minimumRational = minimumUsesBase
            ? negativeConeAxial
            : coneAxial;

        var positive = new ProjectionDepth(
            WideArithmetic.AddSigned320(
                WideArithmetic.SubtractSigned320(
                    maximumRational,
                    centerRational),
                slabRational),
            denominator,
            maximumUsesBase ? coneDisk : CreateZeroRadical(),
            slabRadial);
        var negative = new ProjectionDepth(
            WideArithmetic.SubtractSigned320(
                WideArithmetic.AddSigned320(
                    centerRational,
                    slabRational),
                minimumRational),
            denominator,
            minimumUsesBase ? coneDisk : CreateZeroRadical(),
            slabRadial);
        if (CompareToTwiceRaw(positive, default) < 0
            || CompareToTwiceRaw(negative, default) < 0)
        {
            orientedAxis = default;
            depth = default;
            depthIsClamped = default;
            return false;
        }

        bool positiveSelected =
            CompareProjectionDepthsWithoutSharedSecond(
                positive,
                negative) <= 0;
        ProjectionDepth selected =
            positiveSelected ? positive : negative;
        return TryMaterializeDepth(
            selected,
            positiveSelected ? projectionAxis : -projectionAxis,
            out orientedAxis,
            out depth,
            out depthIsClamped);
    }

    private static void CreateSymmetricCapsuleSlabDepth(
        Vector3d projectionAxis,
        Vector3d shapeCenter,
        Vector3d shapeAxis,
        Fixed64 shapeLength,
        Fixed64 shapeRadius,
        bool shapeUsesSphericalRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxis,
        Fixed64 slabCapsuleLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out ProjectionDepth depth)
    {
        Signed320 shapeAxial = WideArithmetic.MultiplySigned192(
            WideArithmetic.Absolute(GetDot(shapeAxis, projectionAxis)),
            Signed192.Signed(shapeLength.m_rawValue));
        Signed320 slabRational = CreateCapsuleSlabRational(
            projectionAxis,
            slabCapsuleAxis,
            slabCapsuleLength,
            slabHalfThickness);
        Signed320 centerRational = WideArithmetic.MultiplySigned192(
            WideArithmetic.Absolute(GetDot(slabCenter, shapeCenter, projectionAxis)),
            DoubleScale);
        depth = new ProjectionDepth(
            WideArithmetic.SubtractSigned320(
                WideArithmetic.AddSigned320(shapeAxial, slabRational),
                centerRational),
            GetRationalDenominator(),
            shapeUsesSphericalRadius
                ? CreateCapsuleRadical(projectionAxis, shapeRadius)
                : CreateCylinderRadical(
                    projectionAxis,
                    shapeAxis,
                    shapeRadius),
            CreateCapsuleSlabRadical(
                projectionAxis,
                slabCapsuleRadius));
    }

    private static Signed320 CreateCapsuleSlabRational(
        Vector3d projectionAxis,
        Vector2d slabCapsuleAxis,
        Fixed64 slabCapsuleLength,
        Fixed64 slabHalfThickness)
    {
        var worldSlabAxis = new Vector3d(
            slabCapsuleAxis.X,
            Fixed64.Zero,
            slabCapsuleAxis.Y);
        Signed320 planarAxial = WideArithmetic.MultiplySigned192(
            WideArithmetic.Absolute(GetDot(worldSlabAxis, projectionAxis)),
            Signed192.Signed(
                slabCapsuleLength.m_rawValue));
        Signed320 vertical = WideArithmetic.MultiplySigned192(
            WideArithmetic.Absolute(GetDot(Vector3d.Up, projectionAxis)),
            Signed192.Signed(
                slabHalfThickness.m_rawValue));
        return WideArithmetic.AddSigned320(
            planarAxial,
            WideArithmetic.AddSigned320(vertical, vertical));
    }

    private static Radical CreateCapsuleSlabRadical(
        Vector3d projectionAxis,
        Fixed64 radius) =>
        CreateCylinderRadical(
            projectionAxis,
            Vector3d.Up,
            radius);

    private static Radical CreateZeroRadical() =>
        new(default, ScaleSquared);

    private static Signed192 GetRationalDenominator()
    {
        Signed320 wide = WideArithmetic.MultiplySigned192(
            DoubleScale,
            Scale);
        return new Signed192(
            wide.Word2,
            wide.Word1,
            wide.Word0);
    }

    private static bool IsRadicalAtLeastRatio(
        Radical radical,
        Signed320 ratioNumerator) =>
        WideArithmetic.CompareNonNegativeRadicalSumToRatio(
            radical.Numerator,
            radical.Denominator,
            default,
            ScaleSquared,
            ratioNumerator,
            ScaleSquared) >= 0;

    private static int CompareProjectionDepthsWithoutSharedSecond(
        ProjectionDepth left,
        ProjectionDepth right)
    {
        Signed576 unit = Signed576.ExtendValue(
            Signed320.ExtendValue(
                Signed192.Signed(1L)));
        return WideArithmetic.CompareRadialProjectionDepths(
            Signed704.ExtendValue(
                Signed576.ExtendValue(
                    left.RationalNumerator)),
            Signed320.ExtendValue(
                left.RationalDenominator),
            Signed832.ExtendValue(
                left.First.Numerator),
            Signed576.ExtendValue(
                Signed320.ExtendValue(
                    left.First.Denominator)),
            unit,
            Signed704.ExtendValue(
                Signed576.ExtendValue(
                    right.RationalNumerator)),
            Signed320.ExtendValue(
                right.RationalDenominator),
            Signed832.ExtendValue(
                right.First.Numerator),
            Signed576.ExtendValue(
                Signed320.ExtendValue(
                    right.First.Denominator)),
            unit);
    }
}
