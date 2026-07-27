//=======================================================================
// WideFiniteAxisIntersection.Witness.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides witness/support point computation for finite axis and capsule
/// intersection queries, including centered axis endpoints and capsule supports.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private static readonly Signed192 DirectionRootScale = new(0UL, 1UL, 0UL);

    internal static bool TryGetCenteredAxisEndpoint(
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        bool positive,
        out Vector2d endpoint) =>
        TryGetCenteredSupport(
            center,
            axis,
            axisLength,
            Fixed64.Zero,
            default,
            default,
            default,
            positive,
            out endpoint);

    internal static bool TryGetCenteredAxisEndpoint(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        bool positive,
        out Vector3d endpoint) =>
        TryGetCenteredSupport(
            center,
            axis,
            axisLength,
            Fixed64.Zero,
            default,
            default,
            default,
            default,
            positive,
            out endpoint);

    internal static bool TryGetCenteredCapsuleSupport(
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        out Vector2d support)
    {
        GetDirection(direction, out Signed192 radialX, out Signed192 radialY, out Signed320 radialSquared);
        int axialSign = GetDot(axis, Vector2d.Zero, direction, Vector2d.Zero).Sign;
        return TryGetCenteredSupport(
            center,
            axis,
            axialSign == 0 ? Fixed64.Zero : axisLength,
            radius,
            radialX,
            radialY,
            radialSquared,
            axialSign > 0,
            out support);
    }

    internal static FixedPointAnchor2d GetCenteredCapsuleSupportAnchor(
        Vector2d center,
        Fixed64 frameRotation,
        Vector2d localAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d localDirection)
    {
        int axialSign =
            GetDot(
                localAxis,
                Vector2d.Zero,
                localDirection,
                Vector2d.Zero).Sign;
        Fixed64 halfLength = axialSign == 0
            ? Fixed64.Zero
            : axisLength / Fixed64.Two;
        Vector2d axialOffset =
            localAxis * (axialSign > 0 ? halfLength : -halfLength);
        Vector2d radialDirection =
            WideGeometry.GetNormalized(localDirection);
        Vector2d radialOffset = radialDirection * radius;
        FixedPointAnchorTerm2d exactLocalTerm =
            FixedPointAnchorTerm2d.CreateCenteredAxisSupport(
                localAxis,
                axialSign == 0
                    ? Fixed64.Zero
                    : axialSign > 0
                        ? axisLength
                        : -axisLength,
                radialDirection,
                radius,
                axialOffset,
                radialOffset);
        return new FixedPointAnchor2d(
            center,
            frameRotation,
            axialOffset,
            radialOffset,
            exactLocalTerm);
    }

    internal static bool TryGetCenteredCapsuleSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d support)
    {
        GetDirection(
            direction,
            out Signed192 radialX,
            out Signed192 radialY,
            out Signed192 radialZ,
            out Signed320 radialSquared);
        int axialSign = GetDot(axis, Vector3d.Zero, direction, Vector3d.Zero).Sign;
        return TryGetCenteredSupport(
            center,
            axis,
            axialSign == 0 ? Fixed64.Zero : axisLength,
            radius,
            radialX,
            radialY,
            radialZ,
            radialSquared,
            axialSign > 0,
            out support);
    }

    internal static bool TryGetCenteredFiniteCylinderSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d support)
    {
        GetPlaneDirection(
            direction,
            axis,
            out Signed192 radialX,
            out Signed192 radialY,
            out Signed192 radialZ,
            out Signed320 radialSquared);
        return TryGetCenteredSupport(
            center,
            axis,
            axisLength,
            radius,
            radialX,
            radialY,
            radialZ,
            radialSquared,
            GetDot(axis, Vector3d.Zero, direction, Vector3d.Zero).Sign > 0,
            out support);
    }

    internal static bool TryGetCenteredFiniteConeSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d support)
    {
        GetPlaneDirection(
            direction,
            axis,
            out Signed192 radialX,
            out Signed192 radialY,
            out Signed192 radialZ,
            out Signed320 radialSquared);
        bool apex = IsCenteredFiniteConeApexSupportCore(
            axis,
            height,
            radius,
            direction,
            radialX,
            radialY,
            radialZ,
            radialSquared);
        return TryGetCenteredSupport(
            center,
            axis,
            height,
            apex ? Fixed64.Zero : radius,
            apex ? default : radialX,
            apex ? default : radialY,
            apex ? default : radialZ,
            apex ? default : radialSquared,
            apex,
            out support);
    }

    private static bool TryGetCenteredSupport(
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Signed192 radialX,
        Signed192 radialY,
        Signed320 radialSquared,
        bool positive,
        out Vector2d support)
    {
        bool representable = TryGetSupportCoordinate(
                center.X, axis.X, axisLength, radius, radialX, radialSquared, positive, out Fixed64 x)
            & TryGetSupportCoordinate(
                center.Y, axis.Y, axisLength, radius, radialY, radialSquared, positive, out Fixed64 y);
        if (!representable)
        {
            support = default;
            return false;
        }

        support = new Vector2d(x, y);
        return true;
    }

    private static bool TryGetCenteredSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Signed192 radialX,
        Signed192 radialY,
        Signed192 radialZ,
        Signed320 radialSquared,
        bool positive,
        out Vector3d support)
    {
        bool representable = TryGetSupportCoordinate(
                center.X, axis.X, axisLength, radius, radialX, radialSquared, positive, out Fixed64 x)
            & TryGetSupportCoordinate(
                center.Y, axis.Y, axisLength, radius, radialY, radialSquared, positive, out Fixed64 y)
            & TryGetSupportCoordinate(
                center.Z, axis.Z, axisLength, radius, radialZ, radialSquared, positive, out Fixed64 z);
        if (!representable)
        {
            support = default;
            return false;
        }

        support = new Vector3d(x, y, z);
        return true;
    }

    private static bool TryGetSupportCoordinate(
        Fixed64 center,
        Fixed64 axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Signed192 radial,
        Signed320 radialSquared,
        bool positive,
        out Fixed64 coordinate)
    {
        Signed192 signedLength = Signed192.Signed(axisLength.m_rawValue);
        if (!positive)
            signedLength = WideArithmetic.SubtractSigned192(default, signedLength);
        Signed320 baseNumerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(center.m_rawValue),
                DoubleParameterScale),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(axis.m_rawValue),
                signedLength));
        if (radius == Fixed64.Zero || radialSquared.IsZero)
        {
            return Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(baseNumerator),
                Signed576.ExtendValue(
                    Signed320.ExtendValue(DoubleParameterScale)),
                out coordinate);
        }

        Signed320 radialFactor = WideArithmetic.MultiplySigned192(
            Signed192.Signed(radius.m_rawValue),
            radial);
        if (CompareSupportCoordinateToMidpoint(
                baseNumerator,
                radialFactor,
                radialSquared,
                GetMinimumMidpoint()) < 0
            || CompareSupportCoordinateToMidpoint(
                baseNumerator,
                radialFactor,
                radialSquared,
                GetMaximumMidpoint()) >= 0)
        {
            coordinate = default;
            return false;
        }

        // The square-root helper contributes one Q32.32 scale. Pre-scaling
        // the radicand by another scale yields floor(sqrt(q) * 2^64), keeping
        // the candidate within a sub-raw-unit interval before exact correction.
        Signed576 scaledSquared = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(radialSquared),
            DirectionRootScale);
        Signed320 scaledRoot = WideArithmetic.GetFloorSquareRootScaledByFixed64(scaledSquared);
        Signed576 numerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(baseNumerator, scaledRoot),
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(radialFactor),
                    DirectionRootScale),
                DoubleParameterScale));
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            scaledRoot,
            Signed320.ExtendValue(DoubleParameterScale));
        if (!Fixed64.TryGetSignedRawRatio(numerator, denominator, out coordinate))
        {
            coordinate = numerator.Sign < 0 ? Fixed64.MinValue : Fixed64.MaxValue;
        }

        long raw = coordinate.m_rawValue;
        // A nonzero nonsquare radical cannot equal a rational half-step; when
        // the radical is rational, scaledRoot is exact and already rounds ties
        // to even. Only a strict midpoint crossing can need correction.
        if (raw > long.MinValue)
        {
            Signed192 lowerMidpoint = WideArithmetic.SubtractSigned192(
                WideArithmetic.AddSigned192(
                    Signed192.Signed(raw),
                    Signed192.Signed(raw)),
                Scale);
            int lowerComparison = CompareSupportCoordinateToMidpoint(
                baseNumerator,
                radialFactor,
                radialSquared,
                lowerMidpoint);
            if (lowerComparison < 0)
            {
                coordinate = Fixed64.FromRaw(raw - 1L);
                return true;
            }
        }

        if (raw < long.MaxValue)
        {
            Signed192 upperMidpoint = WideArithmetic.AddSigned192(
                WideArithmetic.AddSigned192(
                    Signed192.Signed(raw),
                    Signed192.Signed(raw)),
                Scale);
            int upperComparison = CompareSupportCoordinateToMidpoint(
                baseNumerator,
                radialFactor,
                radialSquared,
                upperMidpoint);
            if (upperComparison > 0)
                coordinate = Fixed64.FromRaw(raw + 1L);
        }

        return true;
    }

    private static int CompareSupportCoordinateToMidpoint(
        Signed320 baseNumerator,
        Signed320 radialFactor,
        Signed320 radialSquared,
        Signed192 midpointTwice)
    {
        Signed320 rational = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(baseNumerator, baseNumerator),
            WideArithmetic.MultiplySigned192(midpointTwice, DoubleParameterScale));
        int rationalSign = rational.Sign;
        int radialSign = radialFactor.Sign;
        if (rationalSign == 0)
            return radialSign;
        if (radialSign == 0 || rationalSign == radialSign)
            return rationalSign;

        Signed832 rationalSquared = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(rational, rational),
            Signed576.ExtendValue(radialSquared));
        Signed192 doubleDenominator = WideArithmetic.AddSigned192(
            DoubleParameterScale,
            DoubleParameterScale);
        Signed832 radialSquaredValue = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(radialFactor, radialFactor),
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(doubleDenominator, doubleDenominator)));
        int magnitudeComparison = WideArithmetic.SubtractSigned832(
            rationalSquared,
            radialSquaredValue).Sign;
        return rationalSign * magnitudeComparison;
    }

    internal static bool IsCenteredFiniteConeApexSupport(
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        Vector3d direction)
    {
        GetPlaneDirection(
            direction,
            axis,
            out Signed192 radialX,
            out Signed192 radialY,
            out Signed192 radialZ,
            out Signed320 radialSquared);
        return IsCenteredFiniteConeApexSupportCore(
            axis,
            height,
            radius,
            direction,
            radialX,
            radialY,
            radialZ,
            radialSquared);
    }

    private static bool IsCenteredFiniteConeApexSupportCore(
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        Vector3d direction,
        Signed192 radialX,
        Signed192 radialY,
        Signed192 radialZ,
        Signed320 radialSquared)
    {
        Signed320 axisTerm = WideArithmetic.MultiplySigned192(
            GetDot(axis, Vector3d.Zero, direction, Vector3d.Zero),
            Signed192.Signed(height.m_rawValue));
        if (axisTerm.Sign <= 0)
            return false;
        if (radius == Fixed64.Zero || radialSquared.IsZero)
            return true;

        Signed320 radialProjection = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    radialX,
                    Signed192.Signed(direction.X.m_rawValue)),
                WideArithmetic.MultiplySigned192(
                    radialY,
                    Signed192.Signed(direction.Y.m_rawValue))),
            WideArithmetic.MultiplySigned192(
                radialZ,
                Signed192.Signed(direction.Z.m_rawValue)));
        Signed576 radialTerm = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(
                radialProjection,
                Signed320.ExtendValue(
                    Signed192.Signed(radius.m_rawValue))),
            ParameterScale);
        Signed832 axisSquared = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(axisTerm, axisTerm),
            Signed576.ExtendValue(radialSquared));
        Signed832 radialTermSquared = WideArithmetic.MultiplySigned576ToSigned832(
            radialTerm,
            radialTerm);
        return WideArithmetic.SubtractSigned832(axisSquared, radialTermSquared).Sign > 0;
    }

    private static void GetDirection(
        Vector2d direction,
        out Signed192 x,
        out Signed192 y,
        out Signed320 squared)
    {
        x = Signed192.Signed(direction.X.m_rawValue);
        y = Signed192.Signed(direction.Y.m_rawValue);
        squared = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(x, x),
            WideArithmetic.MultiplySigned192(y, y));
    }

    private static void GetDirection(
        Vector3d direction,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z,
        out Signed320 squared)
    {
        x = Signed192.Signed(direction.X.m_rawValue);
        y = Signed192.Signed(direction.Y.m_rawValue);
        z = Signed192.Signed(direction.Z.m_rawValue);
        squared = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(x, x),
                WideArithmetic.MultiplySigned192(y, y)),
            WideArithmetic.MultiplySigned192(z, z));
    }

    private static void GetPlaneDirection(
        Vector3d direction,
        Vector3d axis,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z,
        out Signed320 squared)
    {
        Signed192 axisSquared = GetDot(axis, Vector3d.Zero, axis, Vector3d.Zero);
        Signed192 projection = GetDot(axis, Vector3d.Zero, direction, Vector3d.Zero);
        x = Signed192.NarrowValue(WideArithmetic.MultiplySubtract(
            axisSquared,
            Signed192.Signed(direction.X.m_rawValue),
            Signed192.Signed(axis.X.m_rawValue),
            projection));
        y = Signed192.NarrowValue(WideArithmetic.MultiplySubtract(
            axisSquared,
            Signed192.Signed(direction.Y.m_rawValue),
            Signed192.Signed(axis.Y.m_rawValue),
            projection));
        z = Signed192.NarrowValue(WideArithmetic.MultiplySubtract(
            axisSquared,
            Signed192.Signed(direction.Z.m_rawValue),
            Signed192.Signed(axis.Z.m_rawValue),
            projection));
        squared = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(x, x),
                WideArithmetic.MultiplySigned192(y, y)),
            WideArithmetic.MultiplySigned192(z, z));
    }

    private static Signed192 GetMinimumMidpoint() =>
        WideArithmetic.SubtractSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Signed(long.MinValue),
                Signed192.Signed(long.MinValue)),
            Scale);

    private static Signed192 GetMaximumMidpoint() =>
        WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Signed(long.MaxValue),
                Signed192.Signed(long.MaxValue)),
            Scale);
}
