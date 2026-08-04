//=======================================================================
// WideFiniteAxisProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Provides deterministic, high-precision (wide/Signed192) axis-projection utilities for evaluating
/// separating-axis overlap and penetration depth between finite-length capsule and cylinder shapes.
/// Projects shape extents and centers onto a given axis using extended-precision arithmetic to avoid
/// overflow/rounding issues inherent in fixed-point math, enabling robust SAT-based collision detection
/// and accurate minimum translation vector (MTV) computation for narrow-phase physics queries.
/// </summary>
internal static class WideFiniteAxisProjection
{
    private static readonly Signed192 Scale = Signed192.Signed(Fixed64.One.m_rawValue);
    private static readonly Signed192 DoubleScale = Signed192.Signed(Fixed64.Two.m_rawValue);
    private static readonly Signed192 ScaleSquared = GetScaleSquared();

    #region Nested Types

    private readonly struct ProjectionDepth
    {
        internal readonly Signed320 RationalNumerator;
        internal readonly Signed192 RationalDenominator;
        internal readonly Radical First;
        internal readonly Radical Second;

        internal ProjectionDepth(
            Signed320 rationalNumerator,
            Signed192 rationalDenominator,
            Radical first,
            Radical second)
        {
            RationalNumerator = rationalNumerator;
            RationalDenominator = rationalDenominator;
            First = first;
            Second = second;
        }
    }

    private readonly struct Radical
    {
        internal readonly Signed576 Numerator;
        internal readonly Signed192 Denominator;

        internal Radical(Signed576 numerator, Signed192 denominator)
        {
            Numerator = numerator;
            Denominator = denominator;
        }
    }

    #endregion

    internal static bool DoCylinderCapsuleOverlapOnAxis(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        Vector3d capsuleAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius)
    {
        CreateCylinderCapsuleDepth(
            projectionAxis,
            cylinderCenter,
            cylinderAxis,
            cylinderLength,
            cylinderRadius,
            capsuleCenter,
            capsuleAxis,
            capsuleLength,
            capsuleRadius,
            out ProjectionDepth depth);
        return CompareToTwiceRaw(depth, default) >= 0;
    }

    internal static bool TryGetCylinderCapsuleAxisPenetration(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        Vector3d capsuleAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        out Vector3d orientedAxis,
        out Fixed64 penetrationDepth)
    {
        bool result = TryGetCylinderCapsuleAxisPenetration(
            projectionAxis,
            cylinderCenter,
            cylinderAxis,
            cylinderLength,
            cylinderRadius,
            capsuleCenter,
            capsuleAxis,
            capsuleLength,
            capsuleRadius,
            out orientedAxis,
            out penetrationDepth,
            out bool depthIsClamped);
        return result & !depthIsClamped;
    }

    internal static bool TryGetCylinderCapsuleAxisPenetration(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        Vector3d capsuleAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        out Vector3d orientedAxis,
        out Fixed64 penetrationDepth,
        out bool depthIsClamped)
    {
        CreateCylinderCapsuleDepth(
            projectionAxis,
            cylinderCenter,
            cylinderAxis,
            cylinderLength,
            cylinderRadius,
            capsuleCenter,
            capsuleAxis,
            capsuleLength,
            capsuleRadius,
            out ProjectionDepth depth);
        return TryMaterializeDepth(
            depth,
            projectionAxis,
            GetDot(capsuleCenter, cylinderCenter, projectionAxis),
            out orientedAxis,
            out penetrationDepth,
            out depthIsClamped);
    }

    internal static bool DoCylindersOverlapOnAxis(
        Vector3d projectionAxis,
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius)
    {
        CreateCylinderDepth(
            projectionAxis,
            firstCenter,
            firstAxis,
            firstLength,
            firstRadius,
            secondCenter,
            secondAxis,
            secondLength,
            secondRadius,
            out ProjectionDepth depth);
        return CompareToTwiceRaw(depth, default) >= 0;
    }

    internal static bool TryGetCylindersAxisPenetration(
        Vector3d projectionAxis,
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        out Vector3d orientedAxis,
        out Fixed64 penetrationDepth)
    {
        bool result = TryGetCylindersAxisPenetration(
            projectionAxis,
            firstCenter,
            firstAxis,
            firstLength,
            firstRadius,
            secondCenter,
            secondAxis,
            secondLength,
            secondRadius,
            out orientedAxis,
            out penetrationDepth,
            out bool depthIsClamped);
        return result && !depthIsClamped;
    }

    internal static bool TryGetCylindersAxisPenetration(
        Vector3d projectionAxis,
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        out Vector3d orientedAxis,
        out Fixed64 penetrationDepth,
        out bool depthIsClamped)
    {
        CreateCylinderDepth(
            projectionAxis,
            firstCenter,
            firstAxis,
            firstLength,
            firstRadius,
            secondCenter,
            secondAxis,
            secondLength,
            secondRadius,
            out ProjectionDepth depth);
        return TryMaterializeDepth(
            depth,
            projectionAxis,
            GetDot(secondCenter, firstCenter, projectionAxis),
            out orientedAxis,
            out penetrationDepth,
            out depthIsClamped);
    }

    private static void CreateCylinderCapsuleDepth(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        Vector3d capsuleAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        out ProjectionDepth depth)
    {
        CreateRationalDepth(
            projectionAxis,
            cylinderCenter,
            cylinderAxis,
            cylinderLength,
            capsuleCenter,
            capsuleAxis,
            capsuleLength,
            out Signed320 rationalNumerator,
            out Signed192 rationalDenominator);
        depth = new ProjectionDepth(
            rationalNumerator,
            rationalDenominator,
            CreateCylinderRadical(projectionAxis, cylinderAxis, cylinderRadius),
            CreateCapsuleRadical(projectionAxis, capsuleRadius));
    }

    private static void CreateCylinderDepth(
        Vector3d projectionAxis,
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        out ProjectionDepth depth)
    {
        CreateRationalDepth(
            projectionAxis,
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out Signed320 rationalNumerator,
            out Signed192 rationalDenominator);
        depth = new ProjectionDepth(
            rationalNumerator,
            rationalDenominator,
            CreateCylinderRadical(projectionAxis, firstAxis, firstRadius),
            CreateCylinderRadical(projectionAxis, secondAxis, secondRadius));
    }

    private static void CreateRationalDepth(
        Vector3d projectionAxis,
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        out Signed320 numerator,
        out Signed192 denominator)
    {
        Signed192 firstAlignment = WideArithmetic.Absolute(GetDot(firstAxis, projectionAxis));
        Signed192 secondAlignment = WideArithmetic.Absolute(GetDot(secondAxis, projectionAxis));
        Signed192 centerSeparation = WideArithmetic.Absolute(
            GetDot(secondCenter, firstCenter, projectionAxis));
        numerator = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    firstAlignment,
                    Signed192.Signed(firstLength.m_rawValue)),
                WideArithmetic.MultiplySigned192(
                    secondAlignment,
                    Signed192.Signed(secondLength.m_rawValue))),
            WideArithmetic.MultiplySigned192(centerSeparation, DoubleScale));
        Signed320 wideDenominator = WideArithmetic.MultiplySigned192(
            DoubleScale,
            Scale);
        denominator = new Signed192(
            wideDenominator.Word2,
            wideDenominator.Word1,
            wideDenominator.Word0);
    }

    private static Radical CreateCylinderRadical(
        Vector3d projectionAxis,
        Vector3d cylinderAxis,
        Fixed64 radius)
    {
        Signed192 axisSquared = GetDot(cylinderAxis, cylinderAxis);
        Signed192 projectionSquared = GetDot(projectionAxis, projectionAxis);
        Signed192 alignment = GetDot(cylinderAxis, projectionAxis);
        Signed320 planeSquared = WideArithmetic.MultiplySubtract(
            axisSquared,
            projectionSquared,
            alignment,
            alignment);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Signed(radius.m_rawValue),
            Signed192.Signed(radius.m_rawValue));
        Signed576 numerator = WideArithmetic.MultiplySigned320(
            radiusSquared,
            planeSquared);
        Signed576 wideDenominator = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(axisSquared, ScaleSquared)),
            Signed192.Signed(1L));
        // A normalized direction squared is Q64-scaled; multiplying by the
        // Q64 scale denominator needs at most 129 signed bits.
        Signed192 denominator = new(
            wideDenominator.Word2,
            wideDenominator.Word1,
            wideDenominator.Word0);
        return new Radical(numerator, denominator);
    }

    private static Radical CreateCapsuleRadical(
        Vector3d projectionAxis,
        Fixed64 radius)
    {
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Signed(radius.m_rawValue),
            Signed192.Signed(radius.m_rawValue));
        Signed192 projectionSquared = GetDot(projectionAxis, projectionAxis);
        Signed576 numerator = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(radiusSquared),
            projectionSquared);
        return new Radical(numerator, ScaleSquared);
    }

    private static bool TryMaterializeDepth(
        ProjectionDepth depth,
        Vector3d projectionAxis,
        Signed192 centerProjection,
        out Vector3d orientedAxis,
        out Fixed64 result,
        out bool depthIsClamped)
    {
        return TryMaterializeDepth(
            depth,
            centerProjection.Sign < 0
                ? -projectionAxis
                : projectionAxis,
            out orientedAxis,
            out result,
            out depthIsClamped);
    }

    private static bool TryMaterializeDepth(
        ProjectionDepth depth,
        Vector3d selectedAxis,
        out Vector3d orientedAxis,
        out Fixed64 result,
        out bool depthIsClamped)
    {
        if (CompareToTwiceRaw(depth, default) < 0)
        {
            orientedAxis = default;
            result = default;
            depthIsClamped = default;
            return false;
        }

        orientedAxis = selectedAxis;
        if (CompareToTwiceRaw(
                depth,
                new Signed192(0UL, 0UL, ulong.MaxValue - 1UL)) > 0)
        {
            result = Fixed64.MaxValue;
            depthIsClamped = true;
            return true;
        }

        bool approximationsAreRepresentable =
            TryGetRadicalApproximation(depth.First, out Fixed64 firstApproximation)
            & TryGetRadicalApproximation(depth.Second, out Fixed64 secondApproximation);
        if (!approximationsAreRepresentable)
        {
            // Individual supports may exceed Fixed64 even when their sum
            // cancels against the rational separation into range.
            result = GetRoundedDepthByExactSearch(depth);
            depthIsClamped = false;
            return true;
        }

        Signed192 approximatedRadicals = WideArithmetic.AddSigned192(
            Signed192.Signed(firstApproximation.m_rawValue),
            Signed192.Signed(secondApproximation.m_rawValue));
        Signed320 approximationNumerator = WideArithmetic.AddSigned320(
            depth.RationalNumerator,
            WideArithmetic.MultiplySigned192(
                depth.RationalDenominator,
                approximatedRadicals));
        // Cancel the rational and radical terms before narrowing. The
        // nonnegative floor gives correction a bounded endpoint even when an
        // individual term is outside the Fixed64 domain.
        result = Fixed64.GetNonNegativeRawRatioFloor(
            Signed704.ExtendValue(
                Signed576.ExtendValue(approximationNumerator)),
            Signed704.ExtendValue(
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        depth.RationalDenominator))));

        CorrectRoundedDepth(depth, ref result);
        depthIsClamped = false;
        return true;
    }

    private static bool TryGetRadicalApproximation(
        Radical radical,
        out Fixed64 result)
    {
        if (radical.Numerator.IsZero)
        {
            result = Fixed64.Zero;
            return true;
        }

        Signed576 root = WideArithmetic.GetFloorSquareRootOfProduct(
            Signed832.ExtendValue(radical.Numerator),
            radical.Denominator);
        return Fixed64.TryGetSignedRawRatio(
            root,
            Signed576.ExtendValue(
                Signed320.ExtendValue(radical.Denominator)),
            out result);
    }

    private static Fixed64 GetRoundedDepthByExactSearch(ProjectionDepth depth)
    {
        ulong low = 0UL;
        ulong high = (ulong)long.MaxValue;
        ulong floor = 0UL;
        while (low <= high)
        {
            ulong midpoint = low + ((high - low) >> 1);
            if (CompareToTwiceRaw(
                    depth,
                    new Signed192(0UL, 0UL, midpoint << 1)) >= 0)
            {
                floor = midpoint;
                low = midpoint + 1UL;
            }
            else
            {
                high = midpoint - 1UL;
            }
        }

        Fixed64 result = Fixed64.FromRaw((long)floor);
        CorrectRoundedDepth(depth, ref result);
        return result;
    }

    private static void CorrectRoundedDepth(
        ProjectionDepth depth,
        ref Fixed64 result)
    {
        for (int iteration = 0; (iteration < 3) & (result > Fixed64.Zero); iteration++)
        {
            Signed192 lowerMidpoint = new(
                0UL,
                unchecked((ulong)result.m_rawValue) >> 63,
                unchecked((ulong)(result.m_rawValue + result.m_rawValue - 1L)));
            int comparison = CompareToTwiceRaw(depth, lowerMidpoint);
            if (comparison >= (result.m_rawValue & 1L))
            {
                break;
            }

            result = Fixed64.FromRaw(result.m_rawValue - 1L);
        }

        for (int iteration = 0; (iteration < 3) & (result < Fixed64.MaxValue); iteration++)
        {
            Signed192 upperMidpoint = new(
                0UL,
                0UL,
                unchecked((ulong)result.m_rawValue << 1) | 1UL);
            int comparison = CompareToTwiceRaw(depth, upperMidpoint);
            if (comparison + (result.m_rawValue & 1L) <= 0)
            {
                break;
            }

            result = Fixed64.FromRaw(result.m_rawValue + 1L);
        }
    }

    private static int CompareToTwiceRaw(
        ProjectionDepth depth,
        Signed192 twiceRaw)
    {
        Signed320 thresholdNumerator = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                twiceRaw,
                depth.RationalDenominator),
            WideArithmetic.AddSigned320(
                depth.RationalNumerator,
                depth.RationalNumerator));
        if (thresholdNumerator.Sign <= 0)
        {
            if (thresholdNumerator.Sign < 0)
                return 1;
            return depth.First.Numerator.IsZero && depth.Second.Numerator.IsZero
                ? 0
                : 1;
        }

        Signed192 thresholdDenominator = WideArithmetic.AddSigned192(
            depth.RationalDenominator,
            depth.RationalDenominator);
        return WideArithmetic.CompareNonNegativeRadicalSumToRatio(
            depth.First.Numerator,
            depth.First.Denominator,
            depth.Second.Numerator,
            depth.Second.Denominator,
            thresholdNumerator,
            thresholdDenominator);
    }

    private static Signed192 GetDot(Vector3d left, Vector3d right) =>
        WideGeometry.GetDifferenceDotProduct3D(
            left.X,
            Fixed64.Zero,
            left.Y,
            Fixed64.Zero,
            left.Z,
            Fixed64.Zero,
            right.X,
            Fixed64.Zero,
            right.Y,
            Fixed64.Zero,
            right.Z,
            Fixed64.Zero);

    private static Signed192 GetDot(
        Vector3d end,
        Vector3d start,
        Vector3d direction) =>
        WideGeometry.GetDifferenceDotProduct3D(
            end.X,
            start.X,
            end.Y,
            start.Y,
            end.Z,
            start.Z,
            direction.X,
            Fixed64.Zero,
            direction.Y,
            Fixed64.Zero,
            direction.Z,
            Fixed64.Zero);

    private static Signed192 GetScaleSquared()
    {
        Signed320 value = WideArithmetic.MultiplySigned192(Scale, Scale);
        return new Signed192(value.Word2, value.Word1, value.Word0);
    }

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
