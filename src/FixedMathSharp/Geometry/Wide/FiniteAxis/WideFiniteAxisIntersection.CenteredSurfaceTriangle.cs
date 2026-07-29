//=======================================================================
// WideFiniteAxisIntersection.CenteredSurfaceTriangle.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Computes wide-precision contact points and penetration depth between a
/// support point along an axis normal and a triangle surface, using centered
/// (relative) coordinates to preserve precision.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetTriangleSupportPointContact(
        FixedTriangle triangle,
        Vector3d supportPoint,
        Vector3d normal,
        out Vector3d pointOnTriangle,
        out Fixed64 depth,
        out bool depthIsClamped) =>
        TryGetTriangleSupportContactOffsets(
            triangle,
            Vector3d.Zero,
            supportPoint,
            Vector3d.Zero,
            normal,
            out pointOnTriangle,
            out depth,
            out depthIsClamped);

    private static bool TryGetTriangleSupportContactOffsets(
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        Vector3d center,
        Vector3d centerOffset,
        Vector3d normal,
        out Vector3d triangleOriginOffset,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Signed192 relativeX = GetSupportDifference(
            center.X,
            triangle.A.X,
            centerOffset.X);
        Signed192 relativeY = GetSupportDifference(
            center.Y,
            triangle.A.Y,
            centerOffset.Y);
        Signed192 relativeZ = GetSupportDifference(
            center.Z,
            triangle.A.Z,
            centerOffset.Z);
        Signed192 normalX = Signed192.Signed(normal.X.m_rawValue);
        Signed192 normalY = Signed192.Signed(normal.Y.m_rawValue);
        Signed192 normalZ = Signed192.Signed(normal.Z.m_rawValue);
        Signed320 normalSquared = WideGeometry.GetSquaredMagnitude(
            normalX,
            normalY,
            normalZ,
            out _,
            out _,
            out _);
        Signed320 signedDistanceNumerator = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(relativeX, normalX),
                WideArithmetic.MultiplySigned192(relativeY, normalY)),
            WideArithmetic.MultiplySigned192(relativeZ, normalZ));
        if (signedDistanceNumerator.Sign > 0
            && WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                Signed576.ExtendValue(signedDistanceNumerator),
                Signed576.ExtendValue(normalSquared),
                Scale320,
                out _) > Fixed64.Epsilon)
        {
            triangleOriginOffset = default;
            depth = default;
            depthIsClamped = default;
            return false;
        }

        Signed320 projectionDenominator = normalSquared;
        Signed576 projectedX = GetProjectedCoordinateNumerator(
            relativeX,
            normalX,
            signedDistanceNumerator,
            projectionDenominator);
        Signed576 projectedY = GetProjectedCoordinateNumerator(
            relativeY,
            normalY,
            signedDistanceNumerator,
            projectionDenominator);
        Signed576 projectedZ = GetProjectedCoordinateNumerator(
            relativeZ,
            normalZ,
            signedDistanceNumerator,
            projectionDenominator);
        Signed576 wideProjectionDenominator =
            Signed576.ExtendValue(projectionDenominator);
        GetTriangleNormal(
            triangle,
            out Signed192 triangleNormalX,
            out Signed192 triangleNormalY,
            out Signed192 triangleNormalZ);
        if (WideGeometry.GetSquaredMagnitude(
                triangleNormalX,
                triangleNormalY,
                triangleNormalZ,
                out _,
                out _,
                out _).IsZero)
        {
            triangleOriginOffset = default;
            depth = default;
            depthIsClamped = default;
            return false;
        }
        if (!ContainsTriangleProjection(
                triangle,
                triangleNormalX,
                triangleNormalY,
                triangleNormalZ,
                GetAbsoluteCoordinateNumerator(
                    projectedX,
                    triangle.A.X,
                    wideProjectionDenominator),
                GetAbsoluteCoordinateNumerator(
                    projectedY,
                    triangle.A.Y,
                    wideProjectionDenominator),
                GetAbsoluteCoordinateNumerator(
                    projectedZ,
                    triangle.A.Z,
                    wideProjectionDenominator),
                wideProjectionDenominator))
        {
            triangleOriginOffset = default;
            depth = default;
            depthIsClamped = default;
            return false;
        }

        // A point contained by a triangle of representable local vertices is
        // representable in the same local chart.
        _ = TryGetTriangleOriginOffsetCoordinate(
            projectedX,
            triangle.A.X,
            triangleOrigin.X,
            wideProjectionDenominator,
            out Fixed64 triangleX);
        _ = TryGetTriangleOriginOffsetCoordinate(
            projectedY,
            triangle.A.Y,
            triangleOrigin.Y,
            wideProjectionDenominator,
            out Fixed64 triangleY);
        _ = TryGetTriangleOriginOffsetCoordinate(
            projectedZ,
            triangle.A.Z,
            triangleOrigin.Z,
            wideProjectionDenominator,
            out Fixed64 triangleZ);

        triangleOriginOffset = new Vector3d(triangleX, triangleY, triangleZ);
        if (signedDistanceNumerator.Sign >= 0)
        {
            depth = Fixed64.Zero;
            depthIsClamped = false;
            return true;
        }

        depth = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            Signed576.ExtendValue(
                WideArithmetic.SubtractSigned320(
                    default,
                    signedDistanceNumerator)),
            Signed576.ExtendValue(normalSquared),
            Scale320,
            out depthIsClamped);
        return true;
    }

    private static Signed192 GetSupportDifference(
        Fixed64 center,
        Fixed64 trianglePoint,
        Fixed64 centerOffset) =>
        WideArithmetic.AddSigned192(
            GetComponentDifference(center, trianglePoint),
            Signed192.Signed(centerOffset.m_rawValue));

    private static Signed576 GetProjectedCoordinateNumerator(
        Signed192 relative,
        Signed192 normal,
        Signed320 signedDistanceNumerator,
        Signed320 projectionDenominator) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(relative),
                projectionDenominator),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(normal),
                signedDistanceNumerator));

    private static Signed576 GetAbsoluteCoordinateNumerator(
        Signed576 relativeToTriangleA,
        Fixed64 triangleA,
        Signed576 denominator) =>
        WideArithmetic.AddSigned576(
            relativeToTriangleA,
            WideArithmetic.MultiplySigned576(
                denominator,
                Signed192.Signed(triangleA.m_rawValue)));

    private static bool TryGetTriangleOriginOffsetCoordinate(
        Signed576 relativeToTriangleA,
        Fixed64 triangleA,
        Fixed64 triangleOrigin,
        Signed576 denominator,
        out Fixed64 coordinate)
    {
        Signed192 originDifference =
            GetComponentDifference(triangleA, triangleOrigin);
        Signed576 numerator = WideArithmetic.AddSigned576(
            relativeToTriangleA,
            WideArithmetic.MultiplySigned576(denominator, originDifference));
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out coordinate);
    }
}
