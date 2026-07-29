//=======================================================================
// WideOrientedBox.RelativeBounds.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Computes the relative axis-aligned bounds of one oriented box's local extents
/// within another oriented box's coordinate space, using wide (extended-precision)
/// arithmetic and clipping results to the valid <see cref="Fixed64"/> domain.
/// </content>
internal static partial class WideOrientedBox
{
    internal static FixedBoundBox GetRelativeRotatedBoundsClippedToDomain(
        Vector3d sourceOrigin,
        FixedQuaternion sourceRotation,
        Vector3d sourceLocalMin,
        Vector3d sourceLocalMax,
        Vector3d targetOrigin,
        FixedQuaternion targetRotation)
    {
        Vector3d normalizedMinimum =
            Vector3d.Min(sourceLocalMin, sourceLocalMax);
        sourceLocalMax = Vector3d.Max(sourceLocalMin, sourceLocalMax);
        sourceLocalMin = normalizedMinimum;
        WideRationalBasis3d sourceBasis = new(sourceRotation);
        WideRationalBasis3d targetBasis = new(targetRotation);
        Signed320 denominator = WideArithmetic.MultiplySigned192(
            sourceBasis.Denominator,
            targetBasis.Denominator);
        Signed192 deltaX = WideArithmetic.SubtractSigned192(
            Signed192.Raw(sourceOrigin.X),
            Signed192.Raw(targetOrigin.X));
        Signed192 deltaY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(sourceOrigin.Y),
            Signed192.Raw(targetOrigin.Y));
        Signed192 deltaZ = WideArithmetic.SubtractSigned192(
            Signed192.Raw(sourceOrigin.Z),
            Signed192.Raw(targetOrigin.Z));

        GetRelativeBoundsCoordinate(
            sourceBasis,
            targetBasis.Xx,
            targetBasis.Xy,
            targetBasis.Xz,
            sourceLocalMin,
            sourceLocalMax,
            deltaX,
            deltaY,
            deltaZ,
            denominator,
            out Fixed64 minX,
            out Fixed64 maxX);
        GetRelativeBoundsCoordinate(
            sourceBasis,
            targetBasis.Yx,
            targetBasis.Yy,
            targetBasis.Yz,
            sourceLocalMin,
            sourceLocalMax,
            deltaX,
            deltaY,
            deltaZ,
            denominator,
            out Fixed64 minY,
            out Fixed64 maxY);
        GetRelativeBoundsCoordinate(
            sourceBasis,
            targetBasis.Zx,
            targetBasis.Zy,
            targetBasis.Zz,
            sourceLocalMin,
            sourceLocalMax,
            deltaX,
            deltaY,
            deltaZ,
            denominator,
            out Fixed64 minZ,
            out Fixed64 maxZ);
        return FixedBoundBox.FromMinMax(
            new Vector3d(minX, minY, minZ),
            new Vector3d(maxX, maxY, maxZ));
    }

    private static void GetRelativeBoundsCoordinate(
        WideRationalBasis3d sourceBasis,
        Signed192 targetAxisX,
        Signed192 targetAxisY,
        Signed192 targetAxisZ,
        Vector3d sourceLocalMin,
        Vector3d sourceLocalMax,
        Signed192 deltaX,
        Signed192 deltaY,
        Signed192 deltaZ,
        Signed320 denominator,
        out Fixed64 minimum,
        out Fixed64 maximum)
    {
        Signed320 sourceXCoefficient = GetBasisDot(
            targetAxisX,
            targetAxisY,
            targetAxisZ,
            sourceBasis.Xx,
            sourceBasis.Xy,
            sourceBasis.Xz);
        Signed320 sourceYCoefficient = GetBasisDot(
            targetAxisX,
            targetAxisY,
            targetAxisZ,
            sourceBasis.Yx,
            sourceBasis.Yy,
            sourceBasis.Yz);
        Signed320 sourceZCoefficient = GetBasisDot(
            targetAxisX,
            targetAxisY,
            targetAxisZ,
            sourceBasis.Zx,
            sourceBasis.Zy,
            sourceBasis.Zz);
        Signed320 translatedProjection = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(deltaX, targetAxisX),
                WideArithmetic.MultiplySigned192(deltaY, targetAxisY)),
            WideArithmetic.MultiplySigned192(deltaZ, targetAxisZ));
        Signed576 translatedNumerator = WideArithmetic.MultiplySigned320(
            translatedProjection,
            Signed320.ExtendValue(sourceBasis.Denominator));

        Signed576 minimumNumerator = GetRelativeEndpointNumerator(
            translatedNumerator,
            sourceXCoefficient,
            sourceYCoefficient,
            sourceZCoefficient,
            sourceLocalMin,
            sourceLocalMax,
            true);
        Signed576 maximumNumerator = GetRelativeEndpointNumerator(
            translatedNumerator,
            sourceXCoefficient,
            sourceYCoefficient,
            sourceZCoefficient,
            sourceLocalMin,
            sourceLocalMax,
            false);
        minimum = GetRelativeEndpointClippedToDomain(
            minimumNumerator,
            denominator,
            true);
        maximum = GetRelativeEndpointClippedToDomain(
            maximumNumerator,
            denominator,
            false);
    }

    private static Signed576 GetRelativeEndpointNumerator(
        Signed576 translatedNumerator,
        Signed320 sourceXCoefficient,
        Signed320 sourceYCoefficient,
        Signed320 sourceZCoefficient,
        Vector3d sourceLocalMin,
        Vector3d sourceLocalMax,
        bool minimum)
    {
        Fixed64 x = SelectEndpoint(
            sourceXCoefficient,
            sourceLocalMin.X,
            sourceLocalMax.X,
            minimum);
        Fixed64 y = SelectEndpoint(
            sourceYCoefficient,
            sourceLocalMin.Y,
            sourceLocalMax.Y,
            minimum);
        Fixed64 z = SelectEndpoint(
            sourceZCoefficient,
            sourceLocalMin.Z,
            sourceLocalMax.Z,
            minimum);
        return WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.AddSigned576(
                    translatedNumerator,
                    WideArithmetic.MultiplySigned320(
                        sourceXCoefficient,
                        Signed320.ExtendValue(Signed192.Raw(x)))),
                WideArithmetic.MultiplySigned320(
                    sourceYCoefficient,
                    Signed320.ExtendValue(Signed192.Raw(y)))),
            WideArithmetic.MultiplySigned320(
                sourceZCoefficient,
                Signed320.ExtendValue(Signed192.Raw(z))));
    }

    private static Fixed64 SelectEndpoint(
        Signed320 coefficient,
        Fixed64 minimum,
        Fixed64 maximum,
        bool lower) =>
        (coefficient.Sign < 0) == lower ? maximum : minimum;

    private static Fixed64 GetRelativeEndpointClippedToDomain(
        Signed576 numerator,
        Signed320 denominator,
        bool minimum)
    {
        Signed576 minimumNumerator = WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(Signed192.Raw(Fixed64.MinValue)),
            denominator);
        Signed576 maximumNumerator = WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(Signed192.Raw(Fixed64.MaxValue)),
            denominator);
        if (CompareSigned(numerator, minimumNumerator) <= 0)
            return Fixed64.MinValue;
        if (CompareSigned(numerator, maximumNumerator) >= 0)
            return Fixed64.MaxValue;

        _ = Fixed64.TryGetSignedRawRatio(
            numerator,
            Signed576.ExtendValue(denominator),
            out Fixed64 endpoint);
        Signed576 roundedNumerator = WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(Signed192.Raw(endpoint)),
            denominator);
        int comparison = CompareSigned(roundedNumerator, numerator);
        if (minimum && comparison > 0)
            return Fixed64.FromRaw(endpoint.m_rawValue - 1L);
        if (!minimum && comparison < 0)
            return Fixed64.FromRaw(endpoint.m_rawValue + 1L);
        return endpoint;
    }

    private static Signed320 GetBasisDot(
        Signed192 firstX,
        Signed192 firstY,
        Signed192 firstZ,
        Signed192 secondX,
        Signed192 secondY,
        Signed192 secondZ) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(firstX, secondX),
                WideArithmetic.MultiplySigned192(firstY, secondY)),
            WideArithmetic.MultiplySigned192(firstZ, secondZ));
}
