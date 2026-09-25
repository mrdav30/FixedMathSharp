//=======================================================================
// WideFiniteAxisIntersection.CenteredCylinderPairsStrictOverlap.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact strict centered finite-cylinder pair classification, without contact
/// materialization or a finite sampled separating-axis acceptance rule.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>
    /// Tests positive penetration of two positive-radius, positive-height
    /// local-+Y cylinders in authoritative normalized rigid frames.
    /// </summary>
    internal static bool DoesCenteredFiniteCylindersPenetrate(
        Vector3d firstCenter, FixedQuaternion firstRotation, Fixed64 firstHeight, Fixed64 firstRadius,
        Vector3d secondCenter, FixedQuaternion secondRotation, Fixed64 secondHeight, Fixed64 secondRadius)
    {
        // A minimum radial distance either lies on a cap of either solid or
        // on two smooth lateral surfaces. Their common normal is perpendicular
        // to both axes; in that case the infinite-core feet must lie inside
        // both finite axial intervals. The parallel case is an interval family.
        if (DoStrictCylinderLateralInteriorsMeet(firstCenter, firstRotation, firstHeight, firstRadius,
                secondCenter, secondRotation, secondHeight, secondRadius))
        {
            return true;
        }
        return DoesFiniteCapDiskEnterSolid(firstCenter, firstRotation, firstHeight, firstRadius, -1,
                secondCenter, secondRotation, secondHeight, secondRadius, false)
            || DoesFiniteCapDiskEnterSolid(firstCenter, firstRotation, firstHeight, firstRadius, 1,
                secondCenter, secondRotation, secondHeight, secondRadius, false)
            || DoesFiniteCapDiskEnterSolid(secondCenter, secondRotation, secondHeight, secondRadius, -1,
                firstCenter, firstRotation, firstHeight, firstRadius, false)
            || DoesFiniteCapDiskEnterSolid(secondCenter, secondRotation, secondHeight, secondRadius, 1,
                firstCenter, firstRotation, firstHeight, firstRadius, false);
    }

    private static bool DoStrictCylinderLateralInteriorsMeet(
        Vector3d firstCenter, FixedQuaternion firstRotation, Fixed64 firstHeight, Fixed64 firstRadius,
        Vector3d secondCenter, FixedQuaternion secondRotation, Fixed64 secondHeight, Fixed64 secondRadius)
    {
        WideRationalBasis3d firstBasis = new(firstRotation);
        WideRationalBasis3d secondBasis = new(secondRotation);
        WideRationalBasis3d relative = WideRationalBasis3d.CreateRelative(firstBasis, secondBasis);
        WideOrientedBox.GetRelativeLocalPointNumerators(secondCenter, firstCenter, firstRotation,
            out Signed192 dx, out Signed192 dy, out Signed192 dz, out Signed192 pointDenominator);
        Signed192 bx = relative.Yx;
        Signed192 by = relative.Yy;
        Signed192 bz = relative.Yz;
        Signed192 axisDenominator = relative.Denominator;
        Signed320 sineSquared = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(bx, bx), WideArithmetic.MultiplySigned192(bz, bz));
        Signed192 radiusSum = WideArithmetic.AddSigned192(Signed192.Raw(firstRadius), Signed192.Raw(secondRadius));
        Signed320 scaledRadius = WideArithmetic.MultiplySigned192(radiusSum, pointDenominator);
        if (sineSquared.IsZero)
        {
            Signed192 heightSum = WideArithmetic.AddSigned192(Signed192.Raw(firstHeight), Signed192.Raw(secondHeight));
            Signed320 axialExtent = WideArithmetic.MultiplySigned192(heightSum, pointDenominator);
            Signed320 axialDistance = Signed320.ExtendValue(WideArithmetic.Double(WideArithmetic.Absolute(dy)));
            if (WideArithmetic.SubtractSigned320(axialExtent, axialDistance).Sign <= 0)
                return false;
            Signed320 radialDistance = WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(dx, dx), WideArithmetic.MultiplySigned192(dz, dz));
            return WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(scaledRadius, scaledRadius),
                Signed576.ExtendValue(radialDistance)).Sign > 0;
        }
        Signed320 projection = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(dx, bx),
                WideArithmetic.MultiplySigned192(dy, by)), WideArithmetic.MultiplySigned192(dz, bz));
        Signed320 axisSquared = WideArithmetic.MultiplySigned192(axisDenominator, axisDenominator);
        Signed576 firstParameter = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(axisSquared, dy), WideArithmetic.MultiplySigned320(projection, by));
        Signed576 secondParameter = WideArithmetic.MultiplySigned320(
            WideArithmetic.SubtractSigned320(WideArithmetic.MultiplySigned192(by, dy), projection), axisDenominator);
        Signed576 parameterDenominator = WideArithmetic.MultiplySigned320(sineSquared, pointDenominator);
        if (!IsStrictCylinderAxialParameterInterior(firstParameter, parameterDenominator, firstHeight)
            || !IsStrictCylinderAxialParameterInterior(secondParameter, parameterDenominator, secondHeight))
        {
            return false;
        }
        Signed320 crossProjection = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(dx, bz), WideArithmetic.MultiplySigned192(dz, bx));
        Signed576 distanceNumerator = WideArithmetic.MultiplySigned320(crossProjection, crossProjection);
        Signed832 bound = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(scaledRadius, scaledRadius), sineSquared);
        return WideArithmetic.SubtractSigned832(bound, Signed832.ExtendValue(distanceNumerator)).Sign > 0;
    }

    private static bool IsStrictCylinderAxialParameterInterior(Signed576 numerator,
        Signed576 denominator, Fixed64 height)
    {
        if (numerator.Sign < 0)
            numerator = WideArithmetic.SubtractSigned576(default, numerator);
        return WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(denominator, Signed192.Raw(height)),
            WideArithmetic.AddSigned576(numerator, numerator)).Sign > 0;
    }

    /// <summary>
    /// Tests positive penetration between a positive local-+Y cylinder and a
    /// positive local-+Y cone, whose apex is at local +height/2.
    /// </summary>
    internal static bool DoesCenteredFiniteCylinderPenetrateCone(
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Fixed64 cylinderHeight, Fixed64 cylinderRadius,
        Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 coneHeight, Fixed64 coneRadius)
    {
        // Core entry covers zero radial minima. An apex inside the cylinder
        // covers the cone's nonsmooth vertex; the three disks cover cap-active
        // minima. Every remaining minimum lies on a smooth cone generator with
        // normal perpendicular to the cylinder axis, enumerated exactly below.
        if (DoesCenteredFiniteConePenetrateCapsule(coneCenter, coneRotation, coneHeight, coneRadius,
                cylinderCenter, cylinderRotation, Vector3d.Up, cylinderHeight, Fixed64.Zero)
            || DoesFiniteCapDiskEnterSolid(coneCenter, coneRotation, coneHeight, Fixed64.Zero, 1,
                cylinderCenter, cylinderRotation, cylinderHeight, cylinderRadius, false)
            || DoStrictCylinderConeLateralInteriorsMeet(cylinderCenter, cylinderRotation,
                cylinderHeight, cylinderRadius, coneCenter, coneRotation, coneHeight, coneRadius))
        {
            return true;
        }
        return DoesFiniteCapDiskEnterSolid(cylinderCenter, cylinderRotation, cylinderHeight, cylinderRadius, -1,
                coneCenter, coneRotation, coneHeight, coneRadius, true)
            || DoesFiniteCapDiskEnterSolid(cylinderCenter, cylinderRotation, cylinderHeight, cylinderRadius, 1,
                coneCenter, coneRotation, coneHeight, coneRadius, true)
            || DoesFiniteCapDiskEnterSolid(coneCenter, coneRotation, coneHeight, coneRadius, -1,
                cylinderCenter, cylinderRotation, cylinderHeight, cylinderRadius, false);
    }

    private static bool DoStrictCylinderConeLateralInteriorsMeet(
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Fixed64 cylinderHeight, Fixed64 cylinderRadius,
        Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 coneHeight, Fixed64 coneRadius)
    {
        WideRationalBasis3d coneBasis = new(coneRotation);
        WideRationalBasis3d cylinderBasis = new(cylinderRotation);
        WideRationalBasis3d relative = WideRationalBasis3d.CreateRelative(coneBasis, cylinderBasis);
        WideOrientedBox.GetRelativeLocalPointNumerators(cylinderCenter, coneCenter, coneRotation,
            out Signed192 cx, out Signed192 cy, out Signed192 cz, out Signed192 centerDenominator);
        Signed192 ax = relative.Yx, ay = relative.Yy, az = relative.Yz;
        Signed192 axisDenominator = relative.Denominator;
        // d points from the cylinder center to the cone apex in raw coordinates.
        Signed192 dx = WideArithmetic.Negate(WideArithmetic.Double(cx));
        Signed192 dy = WideArithmetic.SubtractSigned192(
            Signed192.NarrowProven(WideArithmetic.MultiplySigned192(Signed192.Raw(coneHeight), centerDenominator)),
            WideArithmetic.Double(cy));
        Signed192 dz = WideArithmetic.Negate(WideArithmetic.Double(cz));
        Signed192 pointDenominator = WideArithmetic.Double(centerDenominator);
        Signed320 s = WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(ax, ax),
            WideArithmetic.MultiplySigned192(az, az));
        Signed192 hSquared = StrictDiskRawSquare(coneHeight);
        Signed192 rSquared = StrictDiskRawSquare(coneRadius);
        Signed192 lSquared = WideArithmetic.AddSigned192(hSquared, rSquared);
        Signed576 discriminant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(s, hSquared),
            WideArithmetic.MultiplySigned320(WideArithmetic.MultiplySigned192(ay, ay), rSquared));
        if (discriminant.Sign < 0)
            return false;
        Signed320 radialDot = WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(dx, ax),
            WideArithmetic.MultiplySigned192(dz, az));
        Signed320 axisDot = WideArithmetic.AddSigned320(radialDot, WideArithmetic.MultiplySigned192(dy, ay));
        if (discriminant.IsZero)
        {
            return DoesStrictParallelConeGeneratorEnterCylinder(dx, dy, dz, pointDenominator,
                ax, ay, az, axisDenominator, axisDot, lSquared, cylinderHeight, cylinderRadius, coneHeight);
        }

        Signed320 cross = WideArithmetic.SubtractSigned320(WideArithmetic.MultiplySigned192(dx, az),
            WideArithmetic.MultiplySigned192(dz, ax));
        Signed576 v = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(s, dy),
            WideArithmetic.MultiplySigned320(radialDot, ay));
        Signed320 axisSquared = WideArithmetic.MultiplySigned192(axisDenominator, axisDenominator);
        Signed192 r = Signed192.Raw(coneRadius);
        Signed832 k = Signed832.ExtendValue(discriminant);
        Signed832 uRational = WideArithmetic.MultiplySigned576ToSigned832(discriminant, v);
        Signed832 uRadical = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(cross, r), axisSquared);
        Signed832 tRational = WideArithmetic.MultiplySigned832(
            WideArithmetic.MultiplySigned576ToSigned832(discriminant, radialDot), axisDenominator);
        Signed832 tRadical = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(WideArithmetic.MultiplySigned192(r, axisDenominator), ay), cross);
        Signed576 pointScale = WideArithmetic.MultiplySigned320(s, pointDenominator);
        Signed832 tDenominator = WideArithmetic.MultiplySigned576ToSigned832(pointScale, discriminant);
        Signed576 distanceRational = WideArithmetic.MultiplySigned576(v, r);

        // In cone coordinates b=-Y. With D the relative-axis denominator,
        // S=ax²+az² and K=H²S-R²ay², the two normals are
        // m=(-R*ay*ax +/- az*sqrt(K), R*S,
        //    -R*ay*az -/+ ax*sqrt(K))/S, with |m|²=H²+R².
        // The generator g=(H²+R²)b+Rm has |a x g|²=(H²+R²)K/D².
        // Both closest parameters therefore stay linear in sqrt(K).
        for (int sign = -1; sign <= 1; sign += 2)
        {
            Signed832 signedT = sign < 0 ? NegateStrict(tRadical) : tRadical;
            Signed832 signedU = sign > 0 ? NegateStrict(uRadical) : uRadical;
            if (!IsStrictCylinderConeFieldParameterInterior(tRational, signedT, tDenominator, cylinderHeight, k)
                || GetStrictCylinderConeFieldSign(uRational, signedU, k) <= 0
                || !IsStrictConeGeneratorBelowBase(uRational, signedU, pointDenominator,
                    s, lSquared, discriminant, coneHeight, k))
            {
                continue;
            }
            Signed832 distanceRadical = Signed832.ExtendValue(Signed576.ExtendValue(cross));
            if (sign < 0)
                distanceRadical = NegateStrict(distanceRadical);
            if (IsStrictConeGeneratorDistanceLess(distanceRational, distanceRadical,
                    pointScale, cylinderRadius, lSquared, k))
            {
                return true;
            }
        }
        return false;
    }

    private static bool IsStrictCylinderConeFieldParameterInterior(Signed832 rational,
        Signed832 radical, Signed832 denominator, Fixed64 height, Signed832 k)
    {
        Signed832 extent = WideArithmetic.MultiplySigned832(denominator, Signed192.Raw(height));
        rational = TwiceStrictDiskValue(rational);
        radical = TwiceStrictDiskValue(radical);
        return GetStrictCylinderConeFieldSign(WideArithmetic.SubtractSigned832(extent, rational), NegateStrict(radical), k) > 0
            && GetStrictCylinderConeFieldSign(WideArithmetic.AddSigned832(extent, rational), radical, k) > 0;
    }

    private static int GetStrictCylinderConeFieldSign(Signed832 rational, Signed832 radical, Signed832 k)
    {
        Span<ulong> a = stackalloc ulong[StrictFieldWords];
        Span<ulong> b = stackalloc ulong[StrictFieldWords];
        WideArithmetic.GetMagnitude(rational, a);
        WideArithmetic.GetMagnitude(radical, b);
        return GetStrictFieldSign(a, (sbyte)rational.Sign, b, (sbyte)radical.Sign, k);
    }

    private static bool IsStrictConeGeneratorBelowBase(Signed832 rational, Signed832 radical,
        Signed192 pointDenominator, Signed320 s, Signed192 lSquared, Signed576 discriminant,
        Fixed64 height, Signed832 k)
    {
        Span<ulong> a = stackalloc ulong[StrictFieldWords];
        Span<ulong> b = stackalloc ulong[StrictFieldWords];
        a.Clear();
        b.Clear();
        sbyte aSign = 0, bSign = 0;
        AccumulateStrictProduct(a, ref aSign, rational, Signed832.ExtendValue(Signed192.Raw(height)), StrictOne, StrictOne);
        AccumulateStrictProduct(a, ref aSign, NegateStrict(Signed832.ExtendValue(pointDenominator)),
            Signed832.ExtendValue(Signed576.ExtendValue(s)), Signed832.ExtendValue(lSquared), Signed832.ExtendValue(discriminant));
        AccumulateStrictProduct(b, ref bSign, radical, Signed832.ExtendValue(Signed192.Raw(height)), StrictOne, StrictOne);
        return GetStrictFieldSign(a, aSign, b, bSign, k) < 0;
    }

    private static bool IsStrictConeGeneratorDistanceLess(Signed576 rational, Signed832 radical,
        Signed576 pointScale, Fixed64 radius, Signed192 lSquared, Signed832 k)
    {
        Span<ulong> a = stackalloc ulong[StrictFieldWords];
        Span<ulong> b = stackalloc ulong[StrictFieldWords];
        a.Clear();
        b.Clear();
        sbyte aSign = 0, bSign = 0;
        Signed832 n = Signed832.ExtendValue(rational);
        Signed832 scale = Signed832.ExtendValue(pointScale);
        AccumulateStrictProduct(a, ref aSign, n, n, StrictOne, StrictOne);
        AccumulateStrictProduct(a, ref aSign, radical, radical, k, StrictOne);
        AccumulateStrictProduct(a, ref aSign, NegateStrict(scale), scale,
            Signed832.ExtendValue(StrictDiskRawSquare(radius)), Signed832.ExtendValue(lSquared));
        AccumulateStrictProduct(b, ref bSign, TwiceStrictDiskValue(n), radical, StrictOne, StrictOne);
        // K<2^394; the rational and radical terms here need fewer than 932
        // and 736 bits, so their squared sign comparison fits 4096 bits.
        return GetStrictFieldSign(a, aSign, b, bSign, k) < 0;
    }

    private static bool DoesStrictParallelConeGeneratorEnterCylinder(
        Signed192 dx, Signed192 dy, Signed192 dz, Signed192 pointDenominator,
        Signed192 ax, Signed192 ay, Signed192 az, Signed192 axisDenominator,
        Signed320 axisDot, Signed192 lSquared, Fixed64 cylinderHeight, Fixed64 cylinderRadius, Fixed64 coneHeight)
    {
        Signed576 start = WideArithmetic.MultiplySigned320(axisDot, Signed192.Raw(coneHeight));
        Signed576 end = WideArithmetic.SubtractSigned576(start,
            WideArithmetic.MultiplySigned320(WideArithmetic.MultiplySigned192(lSquared, ay), pointDenominator));
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(pointDenominator, axisDenominator), Signed192.Raw(coneHeight));
        Signed576 extent = WideArithmetic.MultiplySigned576(denominator, Signed192.Raw(cylinderHeight));
        Signed576 low = WideArithmetic.SubtractSigned576(start, end).Sign < 0 ? start : end;
        Signed576 high = WideArithmetic.SubtractSigned576(start, end).Sign < 0 ? end : start;
        if (WideArithmetic.SubtractSigned576(extent, WideArithmetic.AddSigned576(low, low)).Sign <= 0
            || WideArithmetic.AddSigned576(extent, WideArithmetic.AddSigned576(high, high)).Sign <= 0)
        {
            return false;
        }
        Signed320 x = WideArithmetic.SubtractSigned320(WideArithmetic.MultiplySigned192(dy, az),
            WideArithmetic.MultiplySigned192(dz, ay));
        Signed320 y = WideArithmetic.SubtractSigned320(WideArithmetic.MultiplySigned192(dz, ax),
            WideArithmetic.MultiplySigned192(dx, az));
        Signed320 z = WideArithmetic.SubtractSigned320(WideArithmetic.MultiplySigned192(dx, ay),
            WideArithmetic.MultiplySigned192(dy, ax));
        Signed576 distance = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(x, x), WideArithmetic.MultiplySigned320(y, y)),
            WideArithmetic.MultiplySigned320(z, z));
        Signed576 radius = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(pointDenominator, axisDenominator), Signed192.Raw(cylinderRadius));
        Signed832 bound = WideArithmetic.MultiplySigned576ToSigned832(radius, radius);
        return WideArithmetic.SubtractSigned832(bound, Signed832.ExtendValue(distance)).Sign > 0;
    }
}
