//=======================================================================
// WideOrientedBox.Ray.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Wide-oriented box ray intersection operations.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct RayBound
    {
        internal readonly Signed320 Numerator;
        internal readonly Signed320 Denominator;

        internal RayBound(
            Signed320 numerator,
            Signed320 denominator)
        {
            Numerator = numerator;
            Denominator = denominator;
        }
    }

    #endregion

    internal static bool TryGetRayIntersectionInterval(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d rayOrigin,
        Vector3d rayDirection,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        WideRationalBasis3d basis = new(orientation);
        GetPointProjections(
            rayOrigin,
            center,
            basis,
            out Signed320 originX,
            out Signed320 originY,
            out Signed320 originZ);
        GetDirectionProjections(
            rayDirection,
            basis,
            out Signed320 directionX,
            out Signed320 directionY,
            out Signed320 directionZ);
        Signed192 narrowOriginX = Signed192.NarrowValue(originX);
        Signed192 narrowOriginY = Signed192.NarrowValue(originY);
        Signed192 narrowOriginZ = Signed192.NarrowValue(originZ);
        Signed192 narrowDirectionX = Signed192.NarrowValue(directionX);
        Signed192 narrowDirectionY = Signed192.NarrowValue(directionY);
        Signed192 narrowDirectionZ = Signed192.NarrowValue(directionZ);
        Signed192 extentX = Signed192.NarrowValue(
            GetExtentNumerator(halfExtents.X, basis.Denominator));
        Signed192 extentY = Signed192.NarrowValue(
            GetExtentNumerator(halfExtents.Y, basis.Denominator));
        Signed192 extentZ = Signed192.NarrowValue(
            GetExtentNumerator(halfExtents.Z, basis.Denominator));

        Signed320 one = Signed320.ExtendValue(
            Signed192.Raw(Fixed64.MinIncrement));
        var lower = new RayBound(default, one);
        var upper = new RayBound(
            Signed320.ExtendValue(Signed192.Raw(maxParameter)),
            one);
        if (!TryClipRayAxis(
                narrowOriginX,
                narrowDirectionX,
                extentX,
                ref lower,
                ref upper)
            || !TryClipRayAxis(
                narrowOriginY,
                narrowDirectionY,
                extentY,
                ref lower,
                ref upper)
            || !TryClipRayAxis(
                narrowOriginZ,
                narrowDirectionZ,
                extentZ,
                ref lower,
                ref upper))
        {
            entry = default;
            exit = default;
            return false;
        }

        entry = RoundRayBound(lower);
        exit = RoundRayBound(upper);
        return true;
    }

    private static bool TryClipRayAxis(
        Signed192 origin,
        Signed192 direction,
        Signed192 extent,
        ref RayBound lower,
        ref RayBound upper)
    {
        Signed192 minimum = WideArithmetic.SubtractSigned192(
            default,
            extent);
        if (direction.IsZero)
        {
            return WideArithmetic.SubtractSigned192(
                    origin,
                    minimum).Sign >= 0
                && WideArithmetic.SubtractSigned192(
                    origin,
                    extent).Sign <= 0;
        }

        RayBound first = NormalizeRayBound(
            WideArithmetic.SubtractSigned192(minimum, origin),
            direction);
        RayBound second = NormalizeRayBound(
            WideArithmetic.SubtractSigned192(extent, origin),
            direction);
        if (CompareRayBounds(first, second) > 0)
            (first, second) = (second, first);
        if (CompareRayBounds(second, lower) < 0
            || CompareRayBounds(first, upper) > 0)
        {
            return false;
        }

        if (CompareRayBounds(first, lower) > 0)
            lower = first;
        if (CompareRayBounds(second, upper) < 0)
            upper = second;
        return CompareRayBounds(lower, upper) <= 0;
    }

    private static RayBound NormalizeRayBound(
        Signed192 numerator,
        Signed192 denominator)
    {
        Signed320 scaledNumerator = WideArithmetic.MultiplySigned192(
            numerator,
            Signed192.One);
        Signed320 wideDenominator =
            Signed320.ExtendValue(denominator);
        if (denominator.Sign >= 0)
            return new RayBound(scaledNumerator, wideDenominator);

        return new RayBound(
            WideArithmetic.SubtractSigned320(default, scaledNumerator),
            WideArithmetic.SubtractSigned320(default, wideDenominator));
    }

    private static int CompareRayBounds(RayBound left, RayBound right) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                left.Numerator,
                right.Denominator),
            WideArithmetic.MultiplySigned320(
                right.Numerator,
                left.Denominator)).Sign;

    private static Fixed64 RoundRayBound(RayBound bound)
    {
        // Clipping keeps both bounds inside [0, maxParameter], so the quotient
        // is representable and each normalized denominator is positive.
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(bound.Numerator),
            Signed576.ExtendValue(bound.Denominator),
            out Fixed64 value);
        return value;
    }
}
