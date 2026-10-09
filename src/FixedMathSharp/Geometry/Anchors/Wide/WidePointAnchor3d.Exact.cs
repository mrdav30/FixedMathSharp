//=======================================================================
// WidePointAnchor3d.Exact.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact three-dimensional point-anchor term operations.
/// </content>
internal static partial class WidePointAnchor3d
{
    internal static bool TryGetPointInFrame(in FixedPointAnchor anchor, Vector3d samplingOrigin, out Vector3d point)
    {
        WideRationalBasis3d basis = new(anchor.Rotation);
        Signed320 denominator = GetAnchorDenominator(basis);
        Span<ulong> denominatorWords = stackalloc ulong[9], numeratorWords = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(Signed576.ExtendValue(denominator), denominatorWords);
        Span<Fixed64> coordinates = stackalloc Fixed64[3];
        for (int axis = 0; axis < 3; axis++)
        {
            Signed320 numerator = GetAnchorDeltaNumerator(anchor.Origin[axis], samplingOrigin[axis],
                axis == 0 ? basis.Xx : axis == 1 ? basis.Xy : basis.Xz,
                axis == 0 ? basis.Yx : axis == 1 ? basis.Yy : basis.Yz,
                axis == 0 ? basis.Zx : axis == 1 ? basis.Zy : basis.Zz,
                basis, anchor.LocalPoint, anchor.LocalDisplacement, anchor.LocalTranslation, anchor.ExactLocalTerm, denominator);
            WideArithmetic.GetMagnitude(Signed576.ExtendValue(numerator), numeratorWords);
            // Preserve global half-tie parity without requiring a representable
            // absolute coordinate. No intermediate origin difference is narrowed.
            if (!Fixed64.TryGetSignedRawRatio(numeratorWords, denominatorWords, numerator.Sign < 0,
                out coordinates[axis], samplingOrigin[axis].m_rawValue))
            {
                point = default;
                return false;
            }
        }
        point = new Vector3d(coordinates[0], coordinates[1], coordinates[2]);
        return true;
    }

    // Compare dot(first-second, direction) with dot(third-fourth, direction).
    // Keep both signed ratios exact: materializing either projection would
    // lose sub-raw distinctions or saturate unrelated large offsets equally.
    internal static int CompareProjectedOffsets(
        in FixedPointAnchor first,
        in FixedPointAnchor second,
        in FixedPointAnchor third,
        in FixedPointAnchor fourth,
        Vector3d direction) =>
        CompareProjectedOffsets(first, second, third, fourth, direction, direction);

    internal static int CompareProjectedOffsets(
        in FixedPointAnchor first,
        in FixedPointAnchor second,
        in FixedPointAnchor third,
        in FixedPointAnchor fourth,
        Vector3d firstDirection,
        Vector3d secondDirection)
    {
        GetExactProjectedOffsetRatio(
            first.Origin, first.Rotation, first.LocalPoint,
            first.LocalDisplacement, first.LocalTranslation, first.ExactLocalTerm,
            second.Origin, second.Rotation, second.LocalPoint,
            second.LocalDisplacement, second.LocalTranslation, second.ExactLocalTerm,
            firstDirection, out Signed704 left, out Signed704 leftDenominator);
        GetExactProjectedOffsetRatio(
            third.Origin, third.Rotation, third.LocalPoint,
            third.LocalDisplacement, third.LocalTranslation, third.ExactLocalTerm,
            fourth.Origin, fourth.Rotation, fourth.LocalPoint,
            fourth.LocalDisplacement, fourth.LocalTranslation, fourth.ExactLocalTerm,
            secondDirection, out Signed704 right, out Signed704 rightDenominator);
        int sign = left.Sign;
        int signComparison = sign.CompareTo(right.Sign);
        if (signComparison != 0)
            return signComparison;
        if (sign == 0)
            return 0;

        // The positive denominators and 704-bit signed numerators extend
        // losslessly; the existing full products retain all 1408 bits.
        return sign * WideArithmetic.CompareNonNegativeProducts(
            Signed832.ExtendValue(left), Signed832.ExtendValue(rightDenominator),
            Signed832.ExtendValue(right), Signed832.ExtendValue(leftDenominator));
    }

    internal static bool TryGetPoint(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d localDisplacement,
        Vector3d localTranslation,
        FixedPointAnchorTerm3d exactLocalTerm,
        out Vector3d point)
    {
        if (exactLocalTerm.IsZero
            && localTranslation == Vector3d.Zero)
        {
            return TryGetPoint(
                origin,
                rotation,
                localPoint,
                localDisplacement,
                out point);
        }

        WideRationalBasis3d basis = new(rotation);
        Signed320 denominator = GetAnchorDenominator(basis);
        Signed576 denominatorWide =
            Signed576.ExtendValue(denominator);
        bool representable = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(
                    GetAnchorCoordinateNumerator(
                        origin.X,
                        basis.Xx,
                        basis.Yx,
                        basis.Zx,
                        basis,
                        localPoint,
                        localDisplacement,
                        localTranslation,
                        exactLocalTerm)),
                denominatorWide,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(
                    GetAnchorCoordinateNumerator(
                        origin.Y,
                        basis.Xy,
                        basis.Yy,
                        basis.Zy,
                        basis,
                        localPoint,
                        localDisplacement,
                        localTranslation,
                        exactLocalTerm)),
                denominatorWide,
                out Fixed64 y)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(
                    GetAnchorCoordinateNumerator(
                        origin.Z,
                        basis.Xz,
                        basis.Yz,
                        basis.Zz,
                        basis,
                        localPoint,
                        localDisplacement,
                        localTranslation,
                        exactLocalTerm)),
                denominatorWide,
                out Fixed64 z);
        point = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    internal static bool TryGetRelativeOffset(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Fixed64 scale,
        out Vector3d offset)
    {
        if (firstExactLocalTerm.IsZero
            && secondExactLocalTerm.IsZero
            && firstLocalTranslation == Vector3d.Zero
            && secondLocalTranslation == Vector3d.Zero)
        {
            return TryGetRelativeOffset(
                firstOrigin,
                firstRotation,
                firstLocalPoint,
                firstLocalDisplacement,
                secondOrigin,
                secondRotation,
                secondLocalPoint,
                secondLocalDisplacement,
                scale,
                out offset);
        }

        GetExactRelativeOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            out Signed576 xNumerator,
            out Signed576 yNumerator,
            out Signed576 zNumerator,
            out Signed576 denominator);
        bool representable = TryGetScaledExactCoordinate(
                xNumerator,
                denominator,
                scale,
                out Fixed64 x)
            & TryGetScaledExactCoordinate(
                yNumerator,
                denominator,
                scale,
                out Fixed64 y)
            & TryGetScaledExactCoordinate(
                zNumerator,
                denominator,
                scale,
                out Fixed64 z);
        offset = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    internal static bool RepresentsSamePoint(
        in FixedPointAnchor first,
        in FixedPointAnchor second)
    {
        GetExactRelativeOffsetRatio(
            first.Origin,
            first.Rotation,
            first.LocalPoint,
            first.LocalDisplacement,
            first.LocalTranslation,
            first.ExactLocalTerm,
            second.Origin,
            second.Rotation,
            second.LocalPoint,
            second.LocalDisplacement,
            second.LocalTranslation,
            second.ExactLocalTerm,
            out Signed576 x,
            out Signed576 y,
            out Signed576 z,
            out _);
        return x.IsZero && y.IsZero && z.IsZero;
    }

    internal static bool TryGetLocalPointIn(
        Vector3d pointOrigin,
        FixedQuaternion pointRotation,
        Vector3d pointLocalPoint,
        Vector3d pointLocalDisplacement,
        Vector3d pointLocalTranslation,
        FixedPointAnchorTerm3d exactLocalTerm,
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out Vector3d localPoint)
    {
        if (exactLocalTerm.IsZero
            && pointLocalTranslation == Vector3d.Zero)
        {
            return TryGetLocalPointIn(
                pointOrigin,
                pointRotation,
                pointLocalPoint,
                pointLocalDisplacement,
                frameOrigin,
                frameRotation,
                out localPoint);
        }

        WideRationalBasis3d pointBasis = new(pointRotation);
        WideRationalBasis3d frameBasis = new(frameRotation);
        Signed320 pointDenominator = GetAnchorDenominator(pointBasis);
        Signed320 deltaX = GetAnchorDeltaNumerator(
            pointOrigin.X,
            frameOrigin.X,
            pointBasis.Xx,
            pointBasis.Yx,
            pointBasis.Zx,
            pointBasis,
            pointLocalPoint,
            pointLocalDisplacement,
            pointLocalTranslation,
            exactLocalTerm,
            pointDenominator);
        Signed320 deltaY = GetAnchorDeltaNumerator(
            pointOrigin.Y,
            frameOrigin.Y,
            pointBasis.Xy,
            pointBasis.Yy,
            pointBasis.Zy,
            pointBasis,
            pointLocalPoint,
            pointLocalDisplacement,
            pointLocalTranslation,
            exactLocalTerm,
            pointDenominator);
        Signed320 deltaZ = GetAnchorDeltaNumerator(
            pointOrigin.Z,
            frameOrigin.Z,
            pointBasis.Xz,
            pointBasis.Yz,
            pointBasis.Zz,
            pointBasis,
            pointLocalPoint,
            pointLocalDisplacement,
            pointLocalTranslation,
            exactLocalTerm,
            pointDenominator);
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            pointDenominator,
            Signed320.ExtendValue(frameBasis.Denominator));
        bool representable = Fixed64.TryGetSignedRawRatio(
                GetWideProjection(
                    deltaX,
                    deltaY,
                    deltaZ,
                    frameBasis.Xx,
                    frameBasis.Xy,
                    frameBasis.Xz),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                GetWideProjection(
                    deltaX,
                    deltaY,
                    deltaZ,
                    frameBasis.Yx,
                    frameBasis.Yy,
                    frameBasis.Yz),
                denominator,
                out Fixed64 y)
            & Fixed64.TryGetSignedRawRatio(
                GetWideProjection(
                    deltaX,
                    deltaY,
                    deltaZ,
                    frameBasis.Zx,
                    frameBasis.Zy,
                    frameBasis.Zz),
                denominator,
                out Fixed64 z);
        localPoint = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    internal static bool TryGetProjectedOffset(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Vector3d direction,
        out Fixed64 projection)
    {
        if (firstExactLocalTerm.IsZero
            && secondExactLocalTerm.IsZero
            && firstLocalTranslation == Vector3d.Zero
            && secondLocalTranslation == Vector3d.Zero)
        {
            return TryGetProjectedOffset(
                firstOrigin,
                firstRotation,
                firstLocalPoint,
                firstLocalDisplacement,
                secondOrigin,
                secondRotation,
                secondLocalPoint,
                secondLocalDisplacement,
                direction,
                out projection);
        }

        GetExactProjectedOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            direction,
            out Signed704 numerator,
            out Signed704 denominator);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out projection);
    }

    internal static Fixed64 ProjectNonNegativeOffset(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Vector3d direction)
    {
        if (firstExactLocalTerm.IsZero
            && secondExactLocalTerm.IsZero
            && firstLocalTranslation == Vector3d.Zero
            && secondLocalTranslation == Vector3d.Zero)
        {
            return ProjectNonNegativeOffset(
                firstOrigin,
                firstRotation,
                firstLocalPoint,
                firstLocalDisplacement,
                secondOrigin,
                secondRotation,
                secondLocalPoint,
                secondLocalDisplacement,
                direction);
        }

        GetExactProjectedOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            direction,
            out Signed704 numerator,
            out Signed704 denominator);
        return Fixed64.GetNonNegativeRawRatioFloor(
            numerator,
            denominator);
    }

    private static void GetExactProjectedOffsetRatio(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Vector3d direction,
        out Signed704 numerator,
        out Signed704 denominator)
    {
        GetExactRelativeOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            out Signed576 x,
            out Signed576 y,
            out Signed576 z,
            out Signed576 coordinateDenominator);
        numerator = WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    x,
                    Signed320.ExtendValue(Signed192.Raw(direction.X))),
                WideArithmetic.MultiplySigned576ToSigned704(
                    y,
                    Signed320.ExtendValue(Signed192.Raw(direction.Y)))),
            WideArithmetic.MultiplySigned576ToSigned704(
                z,
                Signed320.ExtendValue(Signed192.Raw(direction.Z))));
        denominator = WideArithmetic.MultiplySigned576ToSigned704(
            coordinateDenominator,
            Signed320.ExtendValue(Signed192.One));
    }

    internal static void GetExactRelativeOffsetRatio(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z,
        out Signed576 denominator)
    {
        WideRationalBasis3d firstBasis = new(firstRotation);
        WideRationalBasis3d secondBasis = new(secondRotation);
        Signed320 firstDenominator = GetAnchorDenominator(firstBasis);
        Signed320 secondDenominator = GetAnchorDenominator(secondBasis);
        denominator = WideArithmetic.MultiplySigned320(
            firstDenominator,
            secondDenominator);
        x = GetExactRelativeOffsetNumerator(
            firstOrigin.X,
            firstBasis.Xx,
            firstBasis.Yx,
            firstBasis.Zx,
            firstBasis,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            firstDenominator,
            secondOrigin.X,
            secondBasis.Xx,
            secondBasis.Yx,
            secondBasis.Zx,
            secondBasis,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            secondDenominator);
        y = GetExactRelativeOffsetNumerator(
            firstOrigin.Y,
            firstBasis.Xy,
            firstBasis.Yy,
            firstBasis.Zy,
            firstBasis,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            firstDenominator,
            secondOrigin.Y,
            secondBasis.Xy,
            secondBasis.Yy,
            secondBasis.Zy,
            secondBasis,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            secondDenominator);
        z = GetExactRelativeOffsetNumerator(
            firstOrigin.Z,
            firstBasis.Xz,
            firstBasis.Yz,
            firstBasis.Zz,
            firstBasis,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            firstDenominator,
            secondOrigin.Z,
            secondBasis.Xz,
            secondBasis.Yz,
            secondBasis.Zz,
            secondBasis,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            secondDenominator);
    }

    private static Signed576 GetExactRelativeOffsetNumerator(
        Fixed64 firstOrigin,
        Signed192 firstAxisX,
        Signed192 firstAxisY,
        Signed192 firstAxisZ,
        WideRationalBasis3d firstBasis,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Signed320 firstDenominator,
        Fixed64 secondOrigin,
        Signed192 secondAxisX,
        Signed192 secondAxisY,
        Signed192 secondAxisZ,
        WideRationalBasis3d secondBasis,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Signed320 secondDenominator) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                GetAnchorCoordinateNumerator(
                    firstOrigin,
                    firstAxisX,
                    firstAxisY,
                    firstAxisZ,
                    firstBasis,
                    firstLocalPoint,
                    firstLocalDisplacement,
                    firstLocalTranslation,
                    firstExactLocalTerm),
                secondDenominator),
            WideArithmetic.MultiplySigned320(
                GetAnchorCoordinateNumerator(
                    secondOrigin,
                    secondAxisX,
                    secondAxisY,
                    secondAxisZ,
                    secondBasis,
                    secondLocalPoint,
                    secondLocalDisplacement,
                    secondLocalTranslation,
                    secondExactLocalTerm),
                firstDenominator));

    private static Signed320 GetAnchorCoordinateNumerator(
        Fixed64 origin,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        WideRationalBasis3d basis,
        Vector3d localPoint,
        Vector3d localDisplacement,
        Vector3d localTranslation,
        FixedPointAnchorTerm3d exactLocalTerm)
    {
        Signed320 roundedNumerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(origin),
                basis.Denominator),
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    WideArithmetic.GetDotProduct3D(
                        Signed192.Raw(localPoint.X),
                        Signed192.Raw(localPoint.Y),
                        Signed192.Raw(localPoint.Z),
                        axisX,
                        axisY,
                        axisZ),
                    WideArithmetic.GetDotProduct3D(
                        Signed192.Raw(localDisplacement.X),
                        Signed192.Raw(localDisplacement.Y),
                        Signed192.Raw(localDisplacement.Z),
                        axisX,
                        axisY,
                        axisZ)),
                WideArithmetic.GetDotProduct3D(
                    Signed192.Raw(localTranslation.X),
                    Signed192.Raw(localTranslation.Y),
                    Signed192.Raw(localTranslation.Z),
                    axisX,
                    axisY,
                    axisZ)));
        Signed576 scaledRounded = WideArithmetic.MultiplySigned320(
            roundedNumerator,
            Signed320.ExtendValue(
                FixedPointAnchorTerm3d.Denominator));
        Signed320 residualProjection = WideArithmetic.GetDotProduct3D(
            Signed192.Signed(exactLocalTerm.X),
            Signed192.Signed(exactLocalTerm.Y),
            Signed192.Signed(exactLocalTerm.Z),
            axisX,
            axisY,
            axisZ);
        return WideArithmetic.AddSigned320(
            Signed320.NarrowValue(scaledRounded),
            residualProjection);
    }

    private static Signed320 GetAnchorDenominator(WideRationalBasis3d basis) =>
        WideArithmetic.MultiplySigned192(
            basis.Denominator,
            FixedPointAnchorTerm3d.Denominator);

    private static Signed320 GetAnchorDeltaNumerator(
        Fixed64 pointOrigin,
        Fixed64 frameOrigin,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        WideRationalBasis3d pointBasis,
        Vector3d pointLocalPoint,
        Vector3d pointLocalDisplacement,
        Vector3d pointLocalTranslation,
        FixedPointAnchorTerm3d exactLocalTerm,
        Signed320 pointDenominator) =>
        WideArithmetic.SubtractSigned320(
            GetAnchorCoordinateNumerator(
                pointOrigin,
                axisX,
                axisY,
                axisZ,
                pointBasis,
                pointLocalPoint,
                pointLocalDisplacement,
                pointLocalTranslation,
                exactLocalTerm),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(frameOrigin),
                Signed192.NarrowValue(pointDenominator)));

    private static Signed576 GetWideProjection(
        Signed320 x,
        Signed320 y,
        Signed320 z,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    x,
                    Signed320.ExtendValue(axisX)),
                WideArithmetic.MultiplySigned320(
                    y,
                    Signed320.ExtendValue(axisY))),
            WideArithmetic.MultiplySigned320(
                z,
                Signed320.ExtendValue(axisZ)));

    private static bool TryGetScaledExactCoordinate(
        Signed576 numerator,
        Signed576 denominator,
        Fixed64 scale,
        out Fixed64 coordinate)
    {
        if (scale == Fixed64.One)
        {
            return Fixed64.TryGetSignedRawRatio(
                numerator,
                denominator,
                out coordinate);
        }
        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(
                numerator,
                Signed320.ExtendValue(Signed192.Raw(scale))),
            WideArithmetic.MultiplySigned576ToSigned704(
                denominator,
                Signed320.ExtendValue(Signed192.One)),
            out coordinate);
    }
}
