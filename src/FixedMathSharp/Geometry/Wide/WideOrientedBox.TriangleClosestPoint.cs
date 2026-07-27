//=======================================================================
// WideOrientedBox.TriangleClosestPoint.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides high-precision, wide-arithmetic computation of the closest point
/// on a triangle to a given point, transformed into the triangle's local space.
/// </content>
internal static partial class WideOrientedBox
{
    internal static FixedPointAnchor GetClosestPointOnTriangle(
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        in FixedPointAnchor point)
    {
        GetPointLocalNumerators(
            point,
            triangleOrigin,
            triangleRotation,
            out Signed576 pointX,
            out Signed576 pointY,
            out Signed576 pointZ,
            out Signed320 pointDenominator);
        Vector3d closest = GetClosestTriangleLocalPoint(
            triangle,
            pointX,
            pointY,
            pointZ,
            pointDenominator);
        return new FixedPointAnchor(
            triangleOrigin,
            triangleRotation,
            closest);
    }

    private static Vector3d GetClosestTriangleLocalPoint(
        FixedTriangle triangle,
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed320 pointDenominator)
    {
        Signed192 abAb = GetTriangleDifferenceDot(
            triangle.B,
            triangle.A,
            triangle.B,
            triangle.A);
        Signed192 abAc = GetTriangleDifferenceDot(
            triangle.B,
            triangle.A,
            triangle.C,
            triangle.A);
        Signed192 acAc = GetTriangleDifferenceDot(
            triangle.C,
            triangle.A,
            triangle.C,
            triangle.A);
        Signed320 gram = WideArithmetic.MultiplySubtract(
            abAb,
            acAc,
            abAc,
            abAc);
        if (WideGeometry.IsQ128MagnitudeAtMostEpsilon(gram))
        {
            return GetClosestTriangleEdgePoint(
                triangle,
                pointX,
                pointY,
                pointZ,
                pointDenominator);
        }

        Signed576 d1 = GetPointEdgeDotNumerator(
            pointX,
            pointY,
            pointZ,
            pointDenominator,
            triangle.A,
            triangle.B);
        Signed576 d2 = GetPointEdgeDotNumerator(
            pointX,
            pointY,
            pointZ,
            pointDenominator,
            triangle.A,
            triangle.C);
        if (d1.Sign <= 0 && d2.Sign <= 0)
            return triangle.A;

        Signed576 abAbScaled = ScaleTriangleDot(abAb, pointDenominator);
        Signed576 abAcScaled = ScaleTriangleDot(abAc, pointDenominator);
        Signed576 acAcScaled = ScaleTriangleDot(acAc, pointDenominator);
        Signed576 d3 = WideArithmetic.SubtractSigned576(d1, abAbScaled);
        Signed576 d4 = WideArithmetic.SubtractSigned576(d2, abAcScaled);
        if (d3.Sign >= 0
            && WideArithmetic.SubtractSigned576(d4, d3).Sign <= 0)
        {
            return triangle.B;
        }

        Signed576 vc = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(d2, abAb),
            WideArithmetic.MultiplySigned576(d1, abAc));
        if (vc.Sign <= 0 && d1.Sign >= 0 && d3.Sign <= 0)
        {
            Fixed64 parameter = GetTriangleUnitRatio(d1, abAbScaled);
            return Vector3d.Lerp(triangle.A, triangle.B, parameter);
        }

        Signed576 d5 = WideArithmetic.SubtractSigned576(d1, abAcScaled);
        Signed576 d6 = WideArithmetic.SubtractSigned576(d2, acAcScaled);
        if (d6.Sign >= 0
            && WideArithmetic.SubtractSigned576(d5, d6).Sign <= 0)
        {
            return triangle.C;
        }

        Signed576 vb = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(d1, acAc),
            WideArithmetic.MultiplySigned576(d2, abAc));
        if (vb.Sign <= 0 && d2.Sign >= 0 && d6.Sign <= 0)
        {
            Fixed64 parameter = GetTriangleUnitRatio(d2, acAcScaled);
            return Vector3d.Lerp(triangle.A, triangle.C, parameter);
        }

        Signed576 d4MinusD3 = WideArithmetic.SubtractSigned576(d4, d3);
        Signed576 d5MinusD6 = WideArithmetic.SubtractSigned576(d5, d6);
        Signed576 gramScaled = WideArithmetic.MultiplySigned320(
            gram,
            pointDenominator);
        Signed576 va = WideArithmetic.SubtractSigned576(
            WideArithmetic.SubtractSigned576(gramScaled, vb),
            vc);
        if (va.Sign <= 0
            && d4MinusD3.Sign >= 0
            && d5MinusD6.Sign >= 0)
        {
            Fixed64 parameter = GetTriangleUnitRatio(
                d4MinusD3,
                WideArithmetic.AddSigned576(d4MinusD3, d5MinusD6));
            return Vector3d.Lerp(triangle.B, triangle.C, parameter);
        }

        Fixed64 weightB = GetTriangleUnitRatio(vb, gramScaled);
        Fixed64 weightC = GetTriangleUnitRatio(vc, gramScaled);
        return triangle.GetPoint(weightB, weightC);
    }

    private static Vector3d GetClosestTriangleEdgePoint(
        FixedTriangle triangle,
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed320 pointDenominator)
    {
        Vector3d best = GetClosestTriangleSegmentPoint(
            triangle.A,
            triangle.B,
            pointX,
            pointY,
            pointZ,
            pointDenominator);
        KeepCloserTriangleSegmentPoint(
            triangle.B,
            triangle.C,
            pointX,
            pointY,
            pointZ,
            pointDenominator,
            ref best);
        KeepCloserTriangleSegmentPoint(
            triangle.C,
            triangle.A,
            pointX,
            pointY,
            pointZ,
            pointDenominator,
            ref best);
        return best;
    }

    private static void KeepCloserTriangleSegmentPoint(
        Vector3d start,
        Vector3d end,
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed320 pointDenominator,
        ref Vector3d best)
    {
        Vector3d candidate = GetClosestTriangleSegmentPoint(
            start,
            end,
            pointX,
            pointY,
            pointZ,
            pointDenominator);
        Signed832 candidateDistance = GetTriangleSquaredDistanceNumerator(
            pointX,
            pointY,
            pointZ,
            pointDenominator,
            candidate);
        Signed832 bestDistance = GetTriangleSquaredDistanceNumerator(
            pointX,
            pointY,
            pointZ,
            pointDenominator,
            best);
        if (WideArithmetic.SubtractSigned832(
                candidateDistance,
                bestDistance).Sign < 0)
        {
            best = candidate;
        }
    }

    private static Vector3d GetClosestTriangleSegmentPoint(
        Vector3d start,
        Vector3d end,
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed320 pointDenominator)
    {
        Signed192 lengthSquared = GetTriangleDifferenceDot(
            end,
            start,
            end,
            start);
        if (lengthSquared.Sign <= 0)
            return start;

        Signed576 projection = GetPointEdgeDotNumerator(
            pointX,
            pointY,
            pointZ,
            pointDenominator,
            start,
            end);
        if (projection.Sign <= 0)
            return start;

        Signed576 scaledLength =
            ScaleTriangleDot(lengthSquared, pointDenominator);
        if (WideArithmetic.SubtractSigned576(
                projection,
                scaledLength).Sign >= 0)
        {
            return end;
        }

        return Vector3d.Lerp(
            start,
            end,
            GetTriangleUnitRatio(projection, scaledLength));
    }

    private static Signed832 GetTriangleSquaredDistanceNumerator(
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed320 pointDenominator,
        Vector3d candidate)
    {
        Signed576 x = GetTrianglePointDeltaNumerator(
            pointX,
            candidate.X,
            pointDenominator);
        Signed576 y = GetTrianglePointDeltaNumerator(
            pointY,
            candidate.Y,
            pointDenominator);
        Signed576 z = GetTrianglePointDeltaNumerator(
            pointZ,
            candidate.Z,
            pointDenominator);
        return WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, x),
                WideArithmetic.MultiplySigned576ToSigned832(y, y)),
            WideArithmetic.MultiplySigned576ToSigned832(z, z));
    }

    private static Signed576 GetPointEdgeDotNumerator(
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed320 pointDenominator,
        Vector3d edgeStart,
        Vector3d edgeEnd)
    {
        Signed576 deltaX = GetTrianglePointDeltaNumerator(
            pointX,
            edgeStart.X,
            pointDenominator);
        Signed576 deltaY = GetTrianglePointDeltaNumerator(
            pointY,
            edgeStart.Y,
            pointDenominator);
        Signed576 deltaZ = GetTrianglePointDeltaNumerator(
            pointZ,
            edgeStart.Z,
            pointDenominator);
        Signed192 edgeX = WideArithmetic.SubtractSigned192(
            Signed192.Raw(edgeEnd.X),
            Signed192.Raw(edgeStart.X));
        Signed192 edgeY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(edgeEnd.Y),
            Signed192.Raw(edgeStart.Y));
        Signed192 edgeZ = WideArithmetic.SubtractSigned192(
            Signed192.Raw(edgeEnd.Z),
            Signed192.Raw(edgeStart.Z));
        return WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(deltaX, edgeX),
                WideArithmetic.MultiplySigned576(deltaY, edgeY)),
            WideArithmetic.MultiplySigned576(deltaZ, edgeZ));
    }

    private static Signed576 GetTrianglePointDeltaNumerator(
        Signed576 pointCoordinate,
        Fixed64 triangleCoordinate,
        Signed320 pointDenominator) =>
        WideArithmetic.SubtractSigned576(
            pointCoordinate,
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(Signed192.Raw(triangleCoordinate)),
                pointDenominator));

    private static Signed576 ScaleTriangleDot(
        Signed192 value,
        Signed320 scale) =>
        WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(value),
            scale);

    private static Fixed64 GetTriangleUnitRatio(
        Signed576 numerator,
        Signed576 denominator)
    {
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(
                numerator,
                Signed320.ExtendValue(Signed192.One)),
            Signed704.ExtendValue(denominator),
            out Fixed64 ratio);
        return ratio;
    }

    private static Signed192 GetTriangleDifferenceDot(
        Vector3d leftEnd,
        Vector3d leftStart,
        Vector3d rightEnd,
        Vector3d rightStart) =>
        WideGeometry.GetDifferenceDotProduct3D(
            leftEnd.X,
            leftStart.X,
            leftEnd.Y,
            leftStart.Y,
            leftEnd.Z,
            leftStart.Z,
            rightEnd.X,
            rightStart.X,
            rightEnd.Y,
            rightStart.Y,
            rightEnd.Z,
            rightStart.Z);

    private static void GetPointLocalNumerators(
        in FixedPointAnchor point,
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z,
        out Signed320 denominator)
    {
        RationalBasis pointBasis = new(point.Rotation);
        RationalBasis frameBasis = new(frameRotation);
        denominator = WideArithmetic.MultiplySigned192(
            pointBasis.Denominator,
            frameBasis.Denominator);
        Signed192 originX = WideArithmetic.SubtractSigned192(
            Signed192.Raw(point.Origin.X),
            Signed192.Raw(frameOrigin.X));
        Signed192 originY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(point.Origin.Y),
            Signed192.Raw(frameOrigin.Y));
        Signed192 originZ = WideArithmetic.SubtractSigned192(
            Signed192.Raw(point.Origin.Z),
            Signed192.Raw(frameOrigin.Z));
        Signed320 rotatedX = GetTriangleRotatedPointCoordinate(
            point,
            pointBasis.Xx,
            pointBasis.Yx,
            pointBasis.Zx);
        Signed320 rotatedY = GetTriangleRotatedPointCoordinate(
            point,
            pointBasis.Xy,
            pointBasis.Yy,
            pointBasis.Zy);
        Signed320 rotatedZ = GetTriangleRotatedPointCoordinate(
            point,
            pointBasis.Xz,
            pointBasis.Yz,
            pointBasis.Zz);
        x = GetTriangleFrameCoordinateNumerator(
            originX,
            originY,
            originZ,
            rotatedX,
            rotatedY,
            rotatedZ,
            pointBasis.Denominator,
            frameBasis.Xx,
            frameBasis.Xy,
            frameBasis.Xz);
        y = GetTriangleFrameCoordinateNumerator(
            originX,
            originY,
            originZ,
            rotatedX,
            rotatedY,
            rotatedZ,
            pointBasis.Denominator,
            frameBasis.Yx,
            frameBasis.Yy,
            frameBasis.Yz);
        z = GetTriangleFrameCoordinateNumerator(
            originX,
            originY,
            originZ,
            rotatedX,
            rotatedY,
            rotatedZ,
            pointBasis.Denominator,
            frameBasis.Zx,
            frameBasis.Zy,
            frameBasis.Zz);
    }

    private static Signed320 GetTriangleRotatedPointCoordinate(
        in FixedPointAnchor point,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ) =>
        WideArithmetic.AddSigned320(
            GetProjection(
                Signed192.Raw(point.LocalPoint.X),
                Signed192.Raw(point.LocalPoint.Y),
                Signed192.Raw(point.LocalPoint.Z),
                axisX,
                axisY,
                axisZ),
            GetProjection(
                Signed192.Raw(point.LocalDisplacement.X),
                Signed192.Raw(point.LocalDisplacement.Y),
                Signed192.Raw(point.LocalDisplacement.Z),
                axisX,
                axisY,
                axisZ));

    private static Signed576 GetTriangleFrameCoordinateNumerator(
        Signed192 originX,
        Signed192 originY,
        Signed192 originZ,
        Signed320 rotatedX,
        Signed320 rotatedY,
        Signed320 rotatedZ,
        Signed192 pointDenominator,
        Signed192 frameAxisX,
        Signed192 frameAxisY,
        Signed192 frameAxisZ)
    {
        Signed320 originProjection = GetProjection(
            originX,
            originY,
            originZ,
            frameAxisX,
            frameAxisY,
            frameAxisZ);
        Signed576 rotatedProjection = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    rotatedX,
                    Signed320.ExtendValue(frameAxisX)),
                WideArithmetic.MultiplySigned320(
                    rotatedY,
                    Signed320.ExtendValue(frameAxisY))),
            WideArithmetic.MultiplySigned320(
                rotatedZ,
                Signed320.ExtendValue(frameAxisZ)));
        return WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                originProjection,
                Signed320.ExtendValue(pointDenominator)),
            rotatedProjection);
    }
}
