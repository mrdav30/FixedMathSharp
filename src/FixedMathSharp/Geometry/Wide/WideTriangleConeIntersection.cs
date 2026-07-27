//=======================================================================
// WideTriangleConeIntersection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <summary>
/// Owns exact full-domain triangle-face reduction against finite cones.
/// </summary>
internal static class WideTriangleConeIntersection
{
    private static readonly Signed192 Scale = Signed192.Signed(Fixed64.One.m_rawValue);
    private static readonly Signed192 MaximumParameter = Signed192.Signed(Fixed64.MaxValue.m_rawValue);

    internal static bool TryGetFaceMinimumAxialPoint(
        FixedTriangle triangle,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Signed320 normalSquared,
        Vector3d apex,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        bool hasEdgeIntersection,
        out Vector3d point,
        out Fixed64 axialParameter)
    {
        Signed192 q = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed320 bWide = GetDot(normalX, normalY, normalZ, axisDirection, Vector3d.Zero);
        Signed192 b = Signed192.NarrowValue(bWide);
        Signed320 c = GetDot(normalX, normalY, normalZ, triangle.A, apex);
        Signed576 p = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(normalSquared), q),
            WideArithmetic.MultiplySigned320(bWide, bWide));

        if (p.IsZero)
            return TryGetPerpendicularPlanePoint(
                triangle,
                normalX,
                normalY,
                normalZ,
                apex,
                axisDirection,
                height,
                b,
                c,
                hasEdgeIntersection,
                out point,
                out axialParameter);

        Signed320 vX = GetRadialNormalComponent(q, normalX, b, axisDirection.X);
        Signed320 vY = GetRadialNormalComponent(q, normalY, b, axisDirection.Y);
        Signed320 vZ = GetRadialNormalComponent(q, normalZ, b, axisDirection.Z);
        Signed576 wX = GetStationaryDirectionComponent(p, axisDirection.X, bWide, vX);
        Signed576 wY = GetStationaryDirectionComponent(p, axisDirection.Y, bWide, vY);
        Signed576 wZ = GetStationaryDirectionComponent(p, axisDirection.Z, bWide, vZ);

        CreatePolynomial(
            q,
            bWide,
            c,
            p,
            height,
            baseRadius,
            out Signed576 coefficient,
            out Signed576 projection,
            out Signed576 constant);
        if (!TryGetMinimumScaledParameter(
                coefficient,
                projection,
                constant,
                b,
                c,
                height,
                out Fixed64 entry))
        {
            point = default;
            axialParameter = default;
            return false;
        }

        bool keepFace;
        if (!hasEdgeIntersection)
        {
            keepFace = ContainsNoEdgeProbe(
                triangle,
                normalX,
                normalY,
                normalZ,
                normalSquared,
                apex,
                axisDirection,
                height,
                q,
                b,
                bWide,
                c,
                p,
                coefficient,
                projection,
                constant);
        }
        else
        {
            GetLowerRootLatticeBracket(
                coefficient,
                projection,
                constant,
                entry,
                out Fixed64 lowerParameter,
                out Fixed64 upperParameter);
            keepFace = ContainsStationaryPoint(
                triangle,
                normalX,
                normalY,
                normalZ,
                normalSquared,
                apex,
                axisDirection,
                height,
                q,
                bWide,
                c,
                p,
                lowerParameter)
                && (upperParameter == lowerParameter
                    || ContainsStationaryPoint(
                        triangle,
                        normalX,
                        normalY,
                        normalZ,
                        normalSquared,
                        apex,
                        axisDirection,
                        height,
                        q,
                        bWide,
                        c,
                        p,
                        upperParameter));
        }
        if (!keepFace)
        {
            point = default;
            axialParameter = default;
            return false;
        }

        point = new Vector3d(
            GetWitnessCoordinate(apex.X, vX, wX, c, p, height, entry),
            GetWitnessCoordinate(apex.Y, vY, wY, c, p, height, entry),
            GetWitnessCoordinate(apex.Z, vZ, wZ, c, p, height, entry));
        axialParameter = Fixed64.GetSignedRawRatio(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(height.m_rawValue),
                Signed192.Signed(entry.m_rawValue)),
            MaximumParameter);
        return true;
    }

    private static bool TryGetMinimumScaledParameter(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Signed192 b,
        Signed320 c,
        Fixed64 height,
        out Fixed64 entry)
    {
        if (FitsSigned832Product(projection, projection)
            && FitsSigned832Product(coefficient, constant))
        {
            return WideFiniteConeIntersection.TrySolveUnitPolynomial(
                coefficient,
                projection,
                constant,
                Fixed64.MaxValue,
                out entry,
                out _);
        }

        if (!TryGetContainedUpperBound(
                coefficient,
                projection,
                constant,
                b,
                c,
                height,
                out Signed576 upperNumerator,
                out Signed576 upperDenominator))
        {
            entry = default;
            return false;
        }

        ulong low = 0UL;
        ulong high = (ulong)Fixed64.MaxValue.m_rawValue;
        Signed192 midpointDenominator = new(0UL, 0UL, high << 1);
        while (low < high)
        {
            ulong candidate = low + ((high - low) >> 1);
            Signed192 midpointNumerator = new(0UL, 0UL, (candidate << 1) | 1UL);
            int upperComparison = CompareRationals(
                midpointNumerator,
                midpointDenominator,
                upperNumerator,
                upperDenominator);
            int polynomialSign = upperComparison > 0
                ? -1
                : WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(
                    coefficient,
                    projection,
                    constant,
                    midpointNumerator,
                    midpointDenominator);
            bool midpointPrecedesRoot = polynomialSign > 0
                || (polynomialSign == 0 && (candidate & 1UL) != 0UL);
            if (midpointPrecedesRoot)
                low = candidate + 1UL;
            else
                high = candidate;
        }

        entry = Fixed64.FromRaw((long)low);
        return true;
    }

    private static bool TryGetContainedUpperBound(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Signed192 b,
        Signed320 c,
        Fixed64 height,
        out Signed576 numerator,
        out Signed576 denominator)
    {
        if (TryGetNonNegativeAxisPlaneRatio(
                b,
                c,
                height,
                out Signed192 axisDenominator,
                out Signed320 axisNumerator))
        {
            numerator = WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(axisNumerator),
                Scale);
            denominator = Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    axisDenominator,
                    Signed192.Signed(height.m_rawValue)));
            return true;
        }

        // Without an in-range axis crossing, the absolute plane distance is
        // linear on [0, 1]; any admitted interval therefore reaches the cap.
        if (WideFiniteConeIntersection.GetPolynomialSignAtScaledParameter(
                coefficient,
                projection,
                constant,
                Fixed64.One,
                Fixed64.One) > 0)
        {
            numerator = default;
            denominator = default;
            return false;
        }

        numerator = Signed576.ExtendValue(
            Signed320.ExtendValue(Signed192.Signed(1L)));
        denominator = numerator;
        return true;
    }

    private static int CompareRationals(
        Signed192 leftNumerator,
        Signed192 leftDenominator,
        Signed576 rightNumerator,
        Signed576 rightDenominator) =>
        WideArithmetic.CompareNonNegative(
            WideArithmetic.MultiplySigned576(rightDenominator, leftNumerator),
            WideArithmetic.MultiplySigned576(rightNumerator, leftDenominator));

    private static bool FitsSigned832Product(Signed576 left, Signed576 right)
    {
        int leftBits = GetMagnitudeBitLength(left);
        int rightBits = GetMagnitudeBitLength(right);
        return leftBits == 0 || rightBits == 0 || leftBits + rightBits <= 830;
    }

    private static int GetMagnitudeBitLength(Signed576 value)
    {
        System.Span<ulong> magnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(value, magnitude);
        for (int index = magnitude.Length - 1; index >= 0; index--)
        {
            if (magnitude[index] != 0UL)
                return (index << 6) + 64 - Fixed64.CountLeadingZeroes(magnitude[index]);
        }

        return 0;
    }

    private static void CreatePolynomial(
        Signed192 q,
        Signed320 b,
        Signed320 c,
        Signed576 p,
        Fixed64 height,
        Fixed64 baseRadius,
        out Signed576 coefficient,
        out Signed576 projection,
        out Signed576 constant)
    {
        Signed192 heightRaw = Signed192.Signed(height.m_rawValue);
        Signed192 radiusRaw = Signed192.Signed(baseRadius.m_rawValue);
        Signed576 bSquared = WideArithmetic.MultiplySigned320(b, b);
        Signed576 firstCoefficient = WideArithmetic.MultiplySigned576(bSquared, q, heightRaw, heightRaw);
        Signed576 secondCoefficient = WideArithmetic.MultiplySigned576(p, radiusRaw, radiusRaw, Scale, Scale);
        coefficient = WideArithmetic.SubtractSigned576(firstCoefficient, secondCoefficient);

        Signed576 bc = WideArithmetic.MultiplySigned320(b, c);
        projection = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned576(bc, q, Scale, heightRaw));
        constant = WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(c, c), q, Scale, Scale);
    }

    private static void GetLowerRootLatticeBracket(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Fixed64 entry,
        out Fixed64 lowerParameter,
        out Fixed64 upperParameter)
    {
        int entrySign = WideFiniteConeIntersection.GetPolynomialSignAtScaledParameter(
                coefficient,
                projection,
                constant,
                entry,
                Fixed64.MaxValue);
        if (entrySign == 0)
        {
            lowerParameter = entry;
            upperParameter = entry;
            return;
        }

        if (entrySign > 0)
        {
            lowerParameter = entry;
            upperParameter = Fixed64.FromRaw(entry.m_rawValue + 1L);
            return;
        }

        upperParameter = entry;
        lowerParameter = Fixed64.FromRaw(entry.m_rawValue - 1L);
    }

    private static bool ContainsNoEdgeProbe(
        FixedTriangle triangle,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Signed320 normalSquared,
        Vector3d apex,
        Vector3d axisDirection,
        Fixed64 height,
        Signed192 q,
        Signed192 b,
        Signed320 bWide,
        Signed320 c,
        Signed576 p,
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant)
    {
        if (TryGetNonNegativeAxisPlaneRatio(b, c, height, out Signed192 denominator, out Signed320 numerator))
        {
            return ContainsAxisPlanePoint(
                triangle,
                normalX,
                normalY,
                normalZ,
                apex,
                axisDirection,
                denominator,
                numerator);
        }

        return ContainsStationaryPoint(
            triangle,
            normalX,
            normalY,
            normalZ,
            normalSquared,
            apex,
            axisDirection,
            height,
            q,
            bWide,
            c,
            p,
            Fixed64.MaxValue);
    }

    private static bool TryGetPerpendicularPlanePoint(
        FixedTriangle triangle,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Vector3d apex,
        Vector3d axisDirection,
        Fixed64 height,
        Signed192 b,
        Signed320 c,
        bool hasEdgeIntersection,
        out Vector3d point,
        out Fixed64 axialParameter)
    {
        if (hasEdgeIntersection
            || !TryGetNonNegativeAxisPlaneRatio(b, c, height, out Signed192 denominator, out Signed320 numerator)
            || !ContainsAxisPlanePoint(
                triangle,
                normalX,
                normalY,
                normalZ,
                apex,
                axisDirection,
                denominator,
                numerator))
        {
            point = default;
            axialParameter = default;
            return false;
        }

        point = new Vector3d(
            GetAxisPlaneCoordinate(apex.X, axisDirection.X, denominator, numerator),
            GetAxisPlaneCoordinate(apex.Y, axisDirection.Y, denominator, numerator),
            GetAxisPlaneCoordinate(apex.Z, axisDirection.Z, denominator, numerator));
        Signed576 axialNumerator = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(numerator),
            Scale);
        _ = Fixed64.TryGetSignedRawRatio(
            axialNumerator,
            Signed576.ExtendValue(Signed320.ExtendValue(denominator)),
            out axialParameter);
        return true;
    }

    private static bool TryGetNonNegativeAxisPlaneRatio(
        Signed192 b,
        Signed320 c,
        Fixed64 height,
        out Signed192 denominator,
        out Signed320 numerator)
    {
        if (b.Sign == 0)
        {
            denominator = default;
            numerator = default;
            return false;
        }

        denominator = b.Sign > 0
            ? b
            : WideArithmetic.SubtractSigned192(default, b);
        numerator = b.Sign > 0
            ? c
            : WideArithmetic.SubtractSigned320(default, c);
        if (numerator.Sign < 0)
            return false;

        Signed576 scaledNumerator = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(numerator),
            Scale);
        Signed576 maximumNumerator = Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(
                denominator,
                Signed192.Signed(height.m_rawValue)));
        return WideArithmetic.CompareNonNegative(scaledNumerator, maximumNumerator) <= 0;
    }

    private static bool ContainsAxisPlanePoint(
        FixedTriangle triangle,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Vector3d apex,
        Vector3d axisDirection,
        Signed192 denominator,
        Signed320 numerator) =>
        GetAxisPlaneHalfspaceSign(triangle.A, triangle.B, normalX, normalY, normalZ, apex, axisDirection, denominator, numerator) >= 0
        && GetAxisPlaneHalfspaceSign(triangle.B, triangle.C, normalX, normalY, normalZ, apex, axisDirection, denominator, numerator) >= 0
        && GetAxisPlaneHalfspaceSign(triangle.C, triangle.A, normalX, normalY, normalZ, apex, axisDirection, denominator, numerator) >= 0;

    private static int GetAxisPlaneHalfspaceSign(
        Vector3d edgeStart,
        Vector3d edgeEnd,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Vector3d apex,
        Vector3d axisDirection,
        Signed192 denominator,
        Signed320 numerator)
    {
        GetEdgeProducts(
            edgeStart,
            edgeEnd,
            normalX,
            normalY,
            normalZ,
            apex,
            axisDirection,
            out Signed320 hO,
            out Signed320 l);
        Signed576 first = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(hO),
            denominator);
        Signed576 second = WideArithmetic.MultiplySigned320(numerator, l);
        return WideArithmetic.AddSigned576(first, second).Sign;
    }

    private static bool ContainsStationaryPoint(
        FixedTriangle triangle,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Signed320 normalSquared,
        Vector3d apex,
        Vector3d axisDirection,
        Fixed64 height,
        Signed192 q,
        Signed320 b,
        Signed320 c,
        Signed576 p,
        Fixed64 scaledParameter) =>
        GetStationaryHalfspaceSign(triangle.A, triangle.B, normalX, normalY, normalZ, normalSquared, apex, axisDirection, height, q, b, c, p, scaledParameter) >= 0
        && GetStationaryHalfspaceSign(triangle.B, triangle.C, normalX, normalY, normalZ, normalSquared, apex, axisDirection, height, q, b, c, p, scaledParameter) >= 0
        && GetStationaryHalfspaceSign(triangle.C, triangle.A, normalX, normalY, normalZ, normalSquared, apex, axisDirection, height, q, b, c, p, scaledParameter) >= 0;

    private static int GetStationaryHalfspaceSign(
        Vector3d edgeStart,
        Vector3d edgeEnd,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Signed320 normalSquared,
        Vector3d apex,
        Vector3d axisDirection,
        Fixed64 height,
        Signed192 q,
        Signed320 b,
        Signed320 c,
        Signed576 p,
        Fixed64 scaledParameter)
    {
        GetEdgeProducts(
            edgeStart,
            edgeEnd,
            normalX,
            normalY,
            normalZ,
            apex,
            axisDirection,
            out Signed320 hO,
            out Signed320 l);
        Signed192 scaleParameter = Signed192.Signed(Fixed64.MaxValue.m_rawValue);
        Signed192 parameter = Signed192.Signed(scaledParameter.m_rawValue);
        Signed192 heightRaw = Signed192.Signed(height.m_rawValue);

        Signed576 pScale = WideArithmetic.MultiplySigned576(p, scaleParameter, Scale);
        Signed704 first = WideArithmetic.MultiplySigned576ToSigned704(pScale, hO);

        Signed576 negativeBC = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned320(b, c));
        Signed704 second = WideArithmetic.MultiplySigned576ToSigned704(
            WideArithmetic.MultiplySigned576(negativeBC, scaleParameter, Scale),
            l);

        Signed576 normalEdge = WideArithmetic.MultiplySigned320(normalSquared, l);
        Signed192 qHeight = Signed192.NarrowValue(
            WideArithmetic.MultiplySigned192(q, heightRaw));
        Signed320 qHeightParameter = WideArithmetic.MultiplySigned192(qHeight, parameter);
        Signed704 third = WideArithmetic.MultiplySigned576ToSigned704(
            normalEdge,
            qHeightParameter);
        return WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(first, second),
            third).Sign;
    }

    private static void GetEdgeProducts(
        Vector3d edgeStart,
        Vector3d edgeEnd,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Vector3d apex,
        Vector3d axisDirection,
        out Signed320 hO,
        out Signed320 l)
    {
        Signed192 edgeX = WideArithmetic.Difference(edgeEnd.X, edgeStart.X);
        Signed192 edgeY = WideArithmetic.Difference(edgeEnd.Y, edgeStart.Y);
        Signed192 edgeZ = WideArithmetic.Difference(edgeEnd.Z, edgeStart.Z);
        GetCross(
            edgeX,
            edgeY,
            edgeZ,
            WideArithmetic.Difference(apex.X, edgeStart.X),
            WideArithmetic.Difference(apex.Y, edgeStart.Y),
            WideArithmetic.Difference(apex.Z, edgeStart.Z),
            out Signed192 originCrossX,
            out Signed192 originCrossY,
            out Signed192 originCrossZ);
        GetCross(
            edgeX,
            edgeY,
            edgeZ,
            Signed192.Signed(axisDirection.X.m_rawValue),
            Signed192.Signed(axisDirection.Y.m_rawValue),
            Signed192.Signed(axisDirection.Z.m_rawValue),
            out Signed192 axisCrossX,
            out Signed192 axisCrossY,
            out Signed192 axisCrossZ);
        hO = GetDot(normalX, normalY, normalZ, originCrossX, originCrossY, originCrossZ);
        l = GetDot(normalX, normalY, normalZ, axisCrossX, axisCrossY, axisCrossZ);
    }

    private static Fixed64 GetWitnessCoordinate(
        Fixed64 apexCoordinate,
        Signed320 v,
        Signed576 w,
        Signed320 c,
        Signed576 p,
        Fixed64 height,
        Fixed64 scaledParameter)
    {
        Signed576 baseValue = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(
                p,
                Signed192.Signed(apexCoordinate.m_rawValue)),
            WideArithmetic.MultiplySigned320(c, v));
        Signed576 first = WideArithmetic.MultiplySigned576(baseValue, MaximumParameter, Scale);
        Signed576 second = WideArithmetic.MultiplySigned576(
            w,
            Signed192.Signed(scaledParameter.m_rawValue),
            Signed192.Signed(height.m_rawValue));
        Signed576 denominator = WideArithmetic.MultiplySigned576(p, MaximumParameter, Scale);
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.AddSigned576(first, second),
            denominator,
            out Fixed64 coordinate);
        return coordinate;
    }

    private static Fixed64 GetAxisPlaneCoordinate(
        Fixed64 apexCoordinate,
        Fixed64 axisCoordinate,
        Signed192 denominator,
        Signed320 numerator)
    {
        Signed576 first = Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(
                denominator,
                Signed192.Signed(apexCoordinate.m_rawValue)));
        Signed576 second = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(numerator),
            Signed192.Signed(axisCoordinate.m_rawValue));
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.AddSigned576(first, second),
            Signed576.ExtendValue(Signed320.ExtendValue(denominator)),
            out Fixed64 coordinate);
        return coordinate;
    }

    private static Signed320 GetRadialNormalComponent(
        Signed192 q,
        Signed192 normal,
        Signed192 b,
        Fixed64 axis) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(q, normal),
            WideArithmetic.MultiplySigned192(
                b,
                Signed192.Signed(axis.m_rawValue)));

    private static Signed576 GetStationaryDirectionComponent(
        Signed576 p,
        Fixed64 axis,
        Signed320 b,
        Signed320 v) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                p,
                Signed192.Signed(axis.m_rawValue)),
            WideArithmetic.MultiplySigned320(b, v));

    private static Signed192 GetDot(
        Vector3d leftEnd,
        Vector3d leftStart,
        Vector3d rightEnd,
        Vector3d rightStart) =>
        WideGeometry.GetDifferenceDotProduct3D(
            leftEnd.X, leftStart.X, leftEnd.Y, leftStart.Y, leftEnd.Z, leftStart.Z,
            rightEnd.X, rightStart.X, rightEnd.Y, rightStart.Y, rightEnd.Z, rightStart.Z);

    private static Signed320 GetDot(
        Signed192 leftX,
        Signed192 leftY,
        Signed192 leftZ,
        Vector3d rightEnd,
        Vector3d rightStart) =>
        GetDot(
            leftX,
            leftY,
            leftZ,
            WideArithmetic.Difference(rightEnd.X, rightStart.X),
            WideArithmetic.Difference(rightEnd.Y, rightStart.Y),
            WideArithmetic.Difference(rightEnd.Z, rightStart.Z));

    private static Signed320 GetDot(
        Signed192 leftX,
        Signed192 leftY,
        Signed192 leftZ,
        Signed192 rightX,
        Signed192 rightY,
        Signed192 rightZ) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(leftX, rightX),
                WideArithmetic.MultiplySigned192(leftY, rightY)),
            WideArithmetic.MultiplySigned192(leftZ, rightZ));

    private static void GetCross(
        Signed192 leftX,
        Signed192 leftY,
        Signed192 leftZ,
        Signed192 rightX,
        Signed192 rightY,
        Signed192 rightZ,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z)
    {
        x = Signed192.NarrowValue(WideArithmetic.MultiplySubtract(leftY, rightZ, leftZ, rightY));
        y = Signed192.NarrowValue(WideArithmetic.MultiplySubtract(leftZ, rightX, leftX, rightZ));
        z = Signed192.NarrowValue(WideArithmetic.MultiplySubtract(leftX, rightY, leftY, rightX));
    }
}
