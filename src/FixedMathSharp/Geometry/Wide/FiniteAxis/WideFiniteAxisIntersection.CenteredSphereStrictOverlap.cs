//=======================================================================
// WideFiniteAxisIntersection.CenteredSphereStrictOverlap.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact strict sphere overlap with local-+Y finite solids, without materializing
/// surface coordinates, distances, normals, or contact anchors.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>
    /// Tests positive penetration into a centered finite cylinder in its
    /// authoritative rigid frame. Height is positive, radii are nonnegative,
    /// and the quaternion is normalized by the owning collider.
    /// </summary>
    /// <remarks>
    /// A zero-radius sphere penetrates only a strict interior point. A zero-radius
    /// cylinder is a segment and therefore has no strict interior for that case.
    /// </remarks>
    internal static bool DoesCenteredFiniteCylinderPenetrateSphere(
        Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius,
        Vector3d sphereCenter, Fixed64 sphereRadius)
    {
        GetStrictSphereLocalPoint(sphereCenter, center, rotation,
            out Signed192 y, out Signed192 denominator, out Signed320 radialSquared);
        Signed192 extent = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            Signed192.Raw(height), denominator));
        Signed192 doubledY = WideArithmetic.AddSigned192(y, y);
        if (doubledY.Sign < 0)
            doubledY = WideArithmetic.SubtractSigned192(default, doubledY);
        Signed192 axialGap = WideArithmetic.SubtractSigned192(doubledY, extent);
        Signed192 scaledRadius = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            Signed192.Raw(radius), denominator));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius);
        int radialComparison = WideArithmetic.SubtractSigned320(radialSquared, radiusSquared).Sign;
        if (axialGap.Sign < 0 && radialComparison < 0)
            return true;
        if (sphereRadius == Fixed64.Zero)
            return false;

        if (axialGap.Sign <= 0)
        {
            // Within the cap interval the exact distance is max(rho-r,0).
            // Comparing rho < r+s avoids the rim radical altogether.
            Signed192 radiusSum = WideArithmetic.AddSigned192(
                Signed192.Raw(radius), Signed192.Raw(sphereRadius));
            Signed192 expandedRadius = Signed192.NarrowProven(
                WideArithmetic.MultiplySigned192(radiusSum, denominator));
            return WideArithmetic.SubtractSigned320(radialSquared,
                WideArithmetic.MultiplySigned192(expandedRadius, expandedRadius)).Sign < 0;
        }

        Signed192 scaledSphereRadius = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            Signed192.Raw(sphereRadius), denominator));
        return IsStrictSphereDiskDistanceLess(radialSquared, scaledRadius,
            radiusSquared, axialGap, scaledSphereRadius);
    }

    /// <summary>
    /// Tests positive penetration into a centered finite cone whose base is at
    /// local -height/2 and apex at +height/2. Height is positive, radii are
    /// nonnegative, and the quaternion is normalized by the owning collider.
    /// </summary>
    /// <remarks>
    /// A zero-radius sphere penetrates only a strict interior point. A zero-radius
    /// cone is a segment and therefore has no strict interior for that case.
    /// </remarks>
    internal static bool DoesCenteredFiniteConePenetrateSphere(
        Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius,
        Vector3d sphereCenter, Fixed64 sphereRadius)
    {
        GetStrictSphereLocalPoint(sphereCenter, center, rotation,
            out Signed192 y, out Signed192 denominator, out Signed320 radialSquared);
        Signed192 h = Signed192.Raw(height);
        Signed192 r = Signed192.Raw(radius);
        Signed192 extent = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(h, denominator));
        Signed192 doubledY = WideArithmetic.AddSigned192(y, y);
        Signed192 aboveBase = WideArithmetic.AddSigned192(doubledY, extent);
        Signed192 belowApex = WideArithmetic.SubtractSigned192(extent, doubledY);
        Signed192 heightSquared = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(h, h));
        Signed192 radiusSquared = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(r, r));
        Signed320 apexAxialSquared = WideArithmetic.MultiplySigned192(belowApex, belowApex);
        Signed576 radialHeightSquared = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(radialSquared, heightSquared), 4L);
        if (aboveBase.Sign > 0 && belowApex.Sign > 0
            && WideArithmetic.SubtractSigned576(radialHeightSquared,
                WideArithmetic.MultiplySigned320(apexAxialSquared, radiusSquared)).Sign < 0)
        {
            return true;
        }
        if (sphereRadius == Fixed64.Zero)
            return false;

        Signed192 scaledRadius = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(r, denominator));
        Signed192 scaledSphereRadius = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            Signed192.Raw(sphereRadius), denominator));
        if (IsStrictSphereDiskDistanceLess(radialSquared, scaledRadius,
                WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius),
                aboveBase, scaledSphereRadius))
        {
            return true;
        }
        // Below the base, its disk (including the rim) is the nearest feature.
        if (aboveBase.Sign <= 0)
            return false;

        Signed192 sideSquared = WideArithmetic.AddSigned192(heightSquared, radiusSquared);
        Signed320 parameter = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(h, aboveBase),
            WideArithmetic.MultiplySigned192(
                WideArithmetic.AddSigned192(radiusSquared, radiusSquared), denominator));
        Signed320 parameterRadical = Signed320.ExtendValue(WideArithmetic.SubtractSigned192(
            default, WideArithmetic.AddSigned192(r, r)));
        // Side projection from the base rim: h*(2*y+h*D)+2*r*r*D-2*r*sqrt(Q).
        if (CompareStrictSphereRadical(Signed576.ExtendValue(parameter),
                parameterRadical, radialSquared) <= 0)
        {
            return false; // The base-disk test already covered the shared rim.
        }
        Signed320 endpoint = WideArithmetic.MultiplySigned192(
            WideArithmetic.AddSigned192(sideSquared, sideSquared), denominator);
        if (CompareStrictSphereRadical(Signed576.ExtendValue(
                WideArithmetic.SubtractSigned320(parameter, endpoint)),
                parameterRadical, radialSquared) >= 0)
        {
            Signed320 apexDistance = WideArithmetic.AddSigned320(
                TwiceStrictSphereValue(TwiceStrictSphereValue(radialSquared)), apexAxialSquared);
            Signed320 sphereBound = TwiceStrictSphereValue(TwiceStrictSphereValue(
                WideArithmetic.MultiplySigned192(scaledSphereRadius, scaledSphereRadius)));
            return WideArithmetic.SubtractSigned320(apexDistance, sphereBound).Sign < 0;
        }

        // Interior side: [2*h*sqrt(Q) - r*(h*D-2*y)]^2
        //                  < 4*(s*D)^2*(h*h+r*r).
        Signed320 offset = WideArithmetic.MultiplySigned192(r,
            WideArithmetic.SubtractSigned192(default, belowApex));
        Signed576 rational = WideArithmetic.AddSigned576(radialHeightSquared,
            WideArithmetic.MultiplySigned320(offset, offset));
        Signed576 bound = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(scaledSphereRadius, scaledSphereRadius), sideSquared);
        rational = WideArithmetic.SubtractSigned576(rational,
            WideArithmetic.MultiplySigned576(bound, 4L));
        Signed576 coefficient = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(offset, h), 4L);
        // The coefficient is below 2^261; narrowing does not discard any value.
        return CompareStrictSphereRadical(rational, Signed320.NarrowValue(coefficient), radialSquared) < 0;
    }

    private static void GetStrictSphereLocalPoint(
        Vector3d point, Vector3d center, FixedQuaternion rotation,
        out Signed192 y, out Signed192 denominator, out Signed320 radialSquared)
    {
        WideOrientedBox.GetRelativeLocalPointNumerators(point, center, rotation,
            out Signed192 x, out y, out Signed192 z, out denominator);
        radialSquared = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(x, x), WideArithmetic.MultiplySigned192(z, z));
    }

    private static bool IsStrictSphereDiskDistanceLess(
        Signed320 radialSquared, Signed192 scaledRadius, Signed320 radiusSquared,
        Signed192 doubledAxialDistance, Signed192 scaledSphereRadius)
    {
        Signed320 axialSquared = WideArithmetic.MultiplySigned192(
            doubledAxialDistance, doubledAxialDistance);
        Signed320 sphereBound = TwiceStrictSphereValue(TwiceStrictSphereValue(
            WideArithmetic.MultiplySigned192(scaledSphereRadius, scaledSphereRadius)));
        if (WideArithmetic.SubtractSigned320(radialSquared, radiusSquared).Sign <= 0)
            return WideArithmetic.SubtractSigned320(axialSquared, sphereBound).Sign < 0;

        Signed320 rational = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(axialSquared,
                TwiceStrictSphereValue(TwiceStrictSphereValue(
                    WideArithmetic.AddSigned320(radialSquared, radiusSquared)))), sphereBound);
        Signed192 negativeRadius = WideArithmetic.SubtractSigned192(default, scaledRadius);
        Signed320 coefficient = TwiceStrictSphereValue(TwiceStrictSphereValue(
            TwiceStrictSphereValue(Signed320.ExtendValue(negativeRadius))));
        return CompareStrictSphereRadical(Signed576.ExtendValue(rational), coefficient, radialSquared) < 0;
    }

    private static Signed320 TwiceStrictSphereValue(Signed320 value) =>
        WideArithmetic.AddSigned320(value, value);

    private static int CompareStrictSphereRadical(
        Signed576 rational, Signed320 coefficient, Signed320 radicand)
    {
        int rationalSign = rational.Sign;
        int coefficientSign = radicand.IsZero ? 0 : coefficient.Sign;
        if (rationalSign == 0)
            return coefficientSign;
        if (coefficientSign == 0 || coefficientSign == rationalSign)
            return rationalSign;

        // A normalized quaternion has D < 2^66, including its normalization
        // tolerance. Orthogonality bounds full-domain
        // projected numerators below 2^131, hence Q below 2^263. The largest
        // cone rational is below 2^394 and its radical coefficient below 2^261.
        // Thus both squared comparison terms fit below 2^788, within Signed832.
        Signed832 rationalSquared = WideArithmetic.MultiplySigned576ToSigned832(rational, rational);
        Signed576 coefficientSquared = WideArithmetic.MultiplySigned320(coefficient, coefficient);
        Signed832 radicalSquared = WideArithmetic.MultiplySigned576ToSigned832(coefficientSquared, radicand);
        int comparison = WideArithmetic.SubtractSigned832(rationalSquared, radicalSquared).Sign;
        return comparison == 0 ? 0 : comparison > 0 ? rationalSign : coefficientSign;
    }
}
