//=======================================================================
// WideFiniteAxisIntersection.FiniteDiskStrictOverlap.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact finite-solid cap-disk admission. Rim crossing and two containment
/// witnesses completely classify intersection with the other solid's interior.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool DoesFiniteCapDiskEnterSolid(
        Vector3d capOwnerCenter, FixedQuaternion capRotation, Fixed64 capOwnerHeight,
        Fixed64 capRadius, int capSign,
        Vector3d solidCenter, FixedQuaternion solidRotation, Fixed64 solidHeight,
        Fixed64 solidRadius, bool cone)
    {
        WideRationalBasis3d capBasis = new(capRotation);
        WideRationalBasis3d solidBasis = new(solidRotation);
        WideRationalBasis3d relative = WideRationalBasis3d.CreateRelative(solidBasis, capBasis);
        WideOrientedBox.GetRelativeLocalPointNumerators(capOwnerCenter, solidCenter, solidRotation,
            out Signed192 centerX, out Signed192 centerY, out Signed192 centerZ, out _);
        Signed192 denominator = WideArithmetic.Double(relative.Denominator);
        Signed192 doubledCapDenominator = WideArithmetic.Double(capBasis.Denominator);
        Signed192 signedHeight = Signed192.Raw(capOwnerHeight);
        if (capSign < 0)
            signedHeight = WideArithmetic.Negate(signedHeight);
        Span<Signed320> center = stackalloc Signed320[3]
        {
            CapCoordinate(centerX, doubledCapDenominator, relative.Yx, signedHeight),
            CapCoordinate(centerY, doubledCapDenominator, relative.Yy, signedHeight),
            CapCoordinate(centerZ, doubledCapDenominator, relative.Yz, signedHeight),
        };
        if (DoesStrictDiskPointEnterSolid(center, denominator, solidHeight, solidRadius, cone))
            return true;

        Signed192 diameter = WideArithmetic.Double(Signed192.Raw(capRadius));
        Span<Signed320> cosine = stackalloc Signed320[3]
        {
            WideArithmetic.MultiplySigned192(relative.Xx, diameter),
            WideArithmetic.MultiplySigned192(relative.Xy, diameter),
            WideArithmetic.MultiplySigned192(relative.Xz, diameter),
        };
        Span<Signed320> sine = stackalloc Signed320[3]
        {
            WideArithmetic.MultiplySigned192(relative.Zx, diameter),
            WideArithmetic.MultiplySigned192(relative.Zy, diameter),
            WideArithmetic.MultiplySigned192(relative.Zz, diameter),
        };
        Span<Signed320> coordinates = stackalloc Signed320[9];
        for (int index = 0; index < 3; index++)
        {
            coordinates[index * 3] = WideArithmetic.AddSigned320(center[index], cosine[index]);
            coordinates[index * 3 + 1] = WideArithmetic.AddSigned320(sine[index], sine[index]);
            coordinates[index * 3 + 2] = WideArithmetic.SubtractSigned320(center[index], cosine[index]);
        }
        Span<Signed832> radial = stackalloc Signed832[5];
        Span<Signed320> lower = stackalloc Signed320[3];
        Span<Signed320> upper = stackalloc Signed320[3];
        BuildStrictDiskCirclePolynomials(coordinates, denominator, solidHeight, solidRadius,
            cone, radial, lower, upper);
        if (radial[4].Sign < 0 && lower[2].Sign > 0 && upper[2].Sign > 0)
            return true; // The projective point omitted by finite t.
        if (HasStrictCirclePolynomialPoint(radial, lower, upper))
            return true;

        // If neither boundary crosses the other interior, one convex planar
        // region contains the other. The cap center covered one direction;
        // any strict point in the target plane section covers the other.
        return IsStrictSolidSectionWitnessInDisk(center, denominator, relative,
            capRadius, solidHeight, solidRadius, cone);
    }

    private static Signed320 CapCoordinate(Signed192 center, Signed192 denominator,
        Signed192 axis, Signed192 height) =>
        WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(center, denominator),
            WideArithmetic.MultiplySigned192(axis, height));

    private static bool DoesStrictDiskPointEnterSolid(ReadOnlySpan<Signed320> point,
        Signed192 denominator, Fixed64 height, Fixed64 radius, bool cone)
    {
        Signed320 h = WideArithmetic.MultiplySigned192(Signed192.Raw(height), denominator);
        Signed320 twiceY = WideArithmetic.AddSigned320(point[1], point[1]);
        Signed320 lower = WideArithmetic.AddSigned320(h, twiceY);
        Signed320 upper = WideArithmetic.SubtractSigned320(h, twiceY);
        if (lower.Sign <= 0 || upper.Sign <= 0)
            return false;
        Signed576 radial = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(point[0], point[0]),
            WideArithmetic.MultiplySigned320(point[2], point[2]));
        if (!cone)
        {
            Signed320 r = WideArithmetic.MultiplySigned192(Signed192.Raw(radius), denominator);
            return WideArithmetic.SubtractSigned576(radial, WideArithmetic.MultiplySigned320(r, r)).Sign < 0;
        }
        Signed192 hSquared = StrictDiskRawSquare(height);
        Signed192 rSquared = StrictDiskRawSquare(radius);
        Signed832 left = WideArithmetic.MultiplySigned832(Signed832.ExtendValue(radial), hSquared);
        left = TwiceStrictDiskValue(TwiceStrictDiskValue(left));
        Signed832 right = WideArithmetic.MultiplySigned832(
            Signed832.ExtendValue(WideArithmetic.MultiplySigned320(upper, upper)), rSquared);
        return WideArithmetic.SubtractSigned832(left, right).Sign < 0;
    }

    private static void BuildStrictDiskCirclePolynomials(ReadOnlySpan<Signed320> coordinates,
        Signed192 denominator, Fixed64 height, Fixed64 radius, bool cone,
        Span<Signed832> radial, Span<Signed320> lower, Span<Signed320> upper)
    {
        Signed320 h = WideArithmetic.MultiplySigned192(Signed192.Raw(height), denominator);
        for (int index = 0; index < 3; index++)
        {
            Signed320 extent = index == 1 ? default : h;
            Signed320 y = WideArithmetic.AddSigned320(coordinates[3 + index], coordinates[3 + index]);
            lower[index] = WideArithmetic.AddSigned320(extent, y);
            upper[index] = WideArithmetic.SubtractSigned320(extent, y);
        }
        Signed192 hSquared = StrictDiskRawSquare(height);
        Signed192 rSquared = StrictDiskRawSquare(radius);
        Signed320 scaledRadius = WideArithmetic.MultiplySigned192(Signed192.Raw(radius), denominator);
        Signed576 radiusBound = WideArithmetic.MultiplySigned320(scaledRadius, scaledRadius);
        for (int degree = 0; degree < 5; degree++)
        {
            Signed576 radialCoefficient = default;
            Signed576 apexCoefficient = default;
            for (int first = 0; first < 3; first++)
            {
                int second = degree - first;
                if ((uint)second >= 3U)
                    continue;
                radialCoefficient = WideArithmetic.AddSigned576(radialCoefficient,
                    WideArithmetic.AddSigned576(
                        WideArithmetic.MultiplySigned320(coordinates[first], coordinates[second]),
                        WideArithmetic.MultiplySigned320(coordinates[6 + first], coordinates[6 + second])));
                if (cone)
                    apexCoefficient = WideArithmetic.AddSigned576(apexCoefficient,
                        WideArithmetic.MultiplySigned320(upper[first], upper[second]));
            }
            if (cone)
            {
                Signed832 left = WideArithmetic.MultiplySigned832(Signed832.ExtendValue(radialCoefficient), hSquared);
                radial[degree] = WideArithmetic.SubtractSigned832(
                    TwiceStrictDiskValue(TwiceStrictDiskValue(left)),
                    WideArithmetic.MultiplySigned832(Signed832.ExtendValue(apexCoefficient), rSquared));
            }
            else
            {
                Signed576 bound = degree == 2 ? WideArithmetic.AddSigned576(radiusBound, radiusBound)
                    : degree == 0 || degree == 4 ? radiusBound : default;
                radial[degree] = Signed832.ExtendValue(WideArithmetic.SubtractSigned576(radialCoefficient, bound));
            }
        }
    }

    private static bool IsStrictSolidSectionWitnessInDisk(ReadOnlySpan<Signed320> center,
        Signed192 denominator, WideRationalBasis3d relative, Fixed64 capRadius,
        Fixed64 height, Fixed64 radius, bool cone)
    {
        Signed192 nx = relative.Yx;
        Signed192 ny = relative.Yy;
        Signed192 nz = relative.Yz;
        Signed320 radialNormalSquared = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(nx, nx), WideArithmetic.MultiplySigned192(nz, nz));
        if (radialNormalSquared.IsZero)
        {
            Span<Signed320> axisPoint = stackalloc Signed320[3] { default, center[1], default };
            if (!DoesStrictDiskPointEnterSolid(axisPoint, denominator, height, radius, cone))
                return false;
            Signed320 r = WideArithmetic.MultiplySigned192(Signed192.Raw(capRadius), denominator);
            Signed576 distance = WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(center[0], center[0]),
                WideArithmetic.MultiplySigned320(center[2], center[2]));
            return WideArithmetic.SubtractSigned576(distance, WideArithmetic.MultiplySigned320(r, r)).Sign <= 0;
        }
        Signed576 plane = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(center[0], nx),
                WideArithmetic.MultiplySigned320(center[1], ny)),
            WideArithmetic.MultiplySigned320(center[2], nz));
        Signed320 hD = WideArithmetic.MultiplySigned192(Signed192.Raw(height), denominator);
        Signed576 yTerm = WideArithmetic.MultiplySigned320(hD, ny);
        Signed576 m0 = WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(plane, plane), yTerm);
        Signed576 m1 = WideArithmetic.SubtractSigned576(default, WideArithmetic.AddSigned576(yTerm, yTerm));
        Signed192 hSquared = StrictDiskRawSquare(height);
        Signed192 rSquared = StrictDiskRawSquare(radius);
        Signed832 constant = WideArithmetic.MultiplySigned576ToSigned832(m0, m0);
        Signed832 linear = TwiceStrictDiskValue(WideArithmetic.MultiplySigned576ToSigned832(m0, m1));
        Signed832 quadratic = WideArithmetic.MultiplySigned576ToSigned832(m1, m1);
        Signed320 twiceRD = WideArithmetic.MultiplySigned192(
            WideArithmetic.Double(Signed192.Raw(cone ? height : radius)), denominator);
        Signed832 bound = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(twiceRD, twiceRD), radialNormalSquared);
        if (cone)
        {
            constant = WideArithmetic.MultiplySigned832(constant, hSquared);
            linear = WideArithmetic.MultiplySigned832(linear, hSquared);
            quadratic = WideArithmetic.MultiplySigned832(quadratic, hSquared);
            bound = WideArithmetic.MultiplySigned832(bound, rSquared);
            linear = WideArithmetic.AddSigned832(linear, TwiceStrictDiskValue(bound));
            quadratic = WideArithmetic.SubtractSigned832(quadratic, bound);
        }
        constant = WideArithmetic.SubtractSigned832(constant, bound);
        if (!TryGetStrictDiskQuadraticWitness(constant, linear, quadratic,
                out Signed832 parameter, out Signed832 parameterDenominator))
        {
            return false;
        }

        Signed576 pointDenominator = WideArithmetic.MultiplySigned320(radialNormalSquared,
            WideArithmetic.Double(denominator));
        Span<Signed576> initial = stackalloc Signed576[3]
        {
            WideArithmetic.MultiplySigned576(m0, nx),
            WideArithmetic.SubtractSigned576(default, WideArithmetic.MultiplySigned320(radialNormalSquared, hD)),
            WideArithmetic.MultiplySigned576(m0, nz),
        };
        Span<Signed576> slope = stackalloc Signed576[3]
        {
            WideArithmetic.MultiplySigned576(m1, nx),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(radialNormalSquared, hD), 2L),
            WideArithmetic.MultiplySigned576(m1, nz),
        };
        for (int index = 0; index < 3; index++)
            initial[index] = WideArithmetic.SubtractSigned576(initial[index],
                WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(center[index], radialNormalSquared), 2L));
        return IsStrictDiskRationalWitnessContained(initial, slope, pointDenominator,
            capRadius, parameter, parameterDenominator);
    }

    private static bool TryGetStrictDiskQuadraticWitness(Signed832 constant, Signed832 linear,
        Signed832 quadratic, out Signed832 numerator, out Signed832 denominator)
    {
        numerator = Signed832.ExtendValue(Signed192.Signed(1L));
        denominator = TwiceStrictDiskValue(numerator);
        Signed832 midpoint = WideArithmetic.AddSigned832(
            TwiceStrictDiskValue(TwiceStrictDiskValue(constant)),
            WideArithmetic.AddSigned832(TwiceStrictDiskValue(linear), quadratic));
        if (midpoint.Sign < 0)
            return true;
        Signed832 twiceQuadratic = TwiceStrictDiskValue(quadratic);
        Signed832 negativeLinear = WideArithmetic.SubtractSigned832(default, linear);
        if (quadratic.Sign > 0 && linear.Sign < 0
            && WideArithmetic.SubtractSigned832(twiceQuadratic, negativeLinear).Sign > 0
            && (constant.Sign < 0 || WideArithmetic.CompareNonNegativeProducts(
                quadratic, TwiceStrictDiskValue(TwiceStrictDiskValue(constant)), linear, linear) < 0))
        {
            numerator = negativeLinear;
            denominator = twiceQuadratic;
            return true;
        }
        bool reverse = constant.Sign >= 0;
        if (reverse)
        {
            constant = WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(constant, linear), quadratic);
            linear = WideArithmetic.SubtractSigned832(default,
                WideArithmetic.AddSigned832(linear, twiceQuadratic));
        }
        if (constant.Sign >= 0)
            return false;
        numerator = WideArithmetic.SubtractSigned832(default, constant);
        Signed832 coefficientBound = WideArithmetic.AddSigned832(AbsoluteStrictDiskValue(linear),
            AbsoluteStrictDiskValue(quadratic));
        // The nonnegative midpoint gives 2*l+q >= 4*|c|, hence
        // |l|+|q| >= 2*|c| > 0. Reversal preserves that midpoint, so
        // this denominator is positive and the witness lies strictly inside.
        denominator = TwiceStrictDiskValue(coefficientBound);
        if (reverse)
            numerator = WideArithmetic.SubtractSigned832(denominator, numerator);
        return true;
    }

    private static bool IsStrictDiskRationalWitnessContained(ReadOnlySpan<Signed576> initial,
        ReadOnlySpan<Signed576> slope, Signed576 pointDenominator, Fixed64 radius,
        Signed832 parameter, Signed832 parameterDenominator)
    {
        // Section coefficients are below 2^804; evaluating the rational
        // witness produces coordinates below 2^1280 and squares below 2^2563.
        const int words = 48;
        Span<ulong> numerator = stackalloc ulong[13];
        Span<ulong> denominator = stackalloc ulong[13];
        Span<ulong> firstCoefficient = stackalloc ulong[9];
        Span<ulong> secondCoefficient = stackalloc ulong[9];
        Span<ulong> first = stackalloc ulong[words];
        Span<ulong> second = stackalloc ulong[words];
        Span<ulong> coordinate = stackalloc ulong[words];
        Span<ulong> square = stackalloc ulong[words];
        Span<ulong> distance = stackalloc ulong[words];
        Span<ulong> sum = stackalloc ulong[words];
        WideArithmetic.GetMagnitude(parameter, numerator);
        WideArithmetic.GetMagnitude(parameterDenominator, denominator);
        distance.Clear();
        for (int index = 0; index < 3; index++)
        {
            WideArithmetic.GetMagnitude(initial[index], firstCoefficient);
            WideArithmetic.GetMagnitude(slope[index], secondCoefficient);
            MultiplyRoundedCylinderWide(firstCoefficient, denominator, first);
            MultiplyRoundedCylinderWide(secondCoefficient, numerator, second);
            AddRoundedCylinderSigned(first, (sbyte)initial[index].Sign,
                second, (sbyte)slope[index].Sign, coordinate, out _);
            MultiplyRoundedCylinderWide(coordinate, coordinate, square);
            WideArithmetic.AddEqualMagnitudes(distance, square, sum);
            sum.CopyTo(distance);
        }
        WideArithmetic.GetMagnitude(WideArithmetic.MultiplySigned576(pointDenominator, Signed192.Raw(radius)), firstCoefficient);
        MultiplyRoundedCylinderWide(firstCoefficient, denominator, coordinate);
        MultiplyRoundedCylinderWide(coordinate, coordinate, square);
        return WideArithmetic.CompareMagnitudeEqualLength(distance, square) <= 0;
    }

    private static Signed192 StrictDiskRawSquare(Fixed64 value) =>
        Signed192.NarrowProven(WideArithmetic.MultiplySigned192(Signed192.Raw(value), Signed192.Raw(value)));

    private static Signed832 TwiceStrictDiskValue(Signed832 value) => WideArithmetic.AddSigned832(value, value);

    private static Signed832 AbsoluteStrictDiskValue(Signed832 value) =>
        value.Sign < 0 ? WideArithmetic.SubtractSigned832(default, value) : value;
}
