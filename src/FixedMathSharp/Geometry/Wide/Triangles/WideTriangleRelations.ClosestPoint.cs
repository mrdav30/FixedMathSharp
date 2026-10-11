//=======================================================================
// WideTriangleRelations.ClosestPoint.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides the exact closest-point reducer for rigid triangle frames.
/// </content>
internal static partial class WideTriangleRelations
{
    // Tests the orthogonal projection inclusively, without materializing the
    // point or its barycentric coordinates. A degenerate face has no interior.
    internal static bool ContainsProjection(
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        in FixedPointAnchor point)
    {
        return TryGetProjectionBarycentrics(triangle, triangleOrigin, triangleRotation, point,
                out Signed576 va, out Signed576 vb, out Signed576 vc)
            && va.Sign >= 0 && vb.Sign >= 0 && vc.Sign >= 0;
    }

    private static bool TryGetProjectionBarycentrics(
        FixedTriangle triangle, Vector3d triangleOrigin, FixedQuaternion triangleRotation,
        in FixedPointAnchor point, out Signed576 va, out Signed576 vb, out Signed576 vc)
    {
        Signed192 abAb = GetTriangleDifferenceDot(triangle.B, triangle.A, triangle.B, triangle.A);
        Signed192 abAc = GetTriangleDifferenceDot(triangle.B, triangle.A, triangle.C, triangle.A);
        Signed192 acAc = GetTriangleDifferenceDot(triangle.C, triangle.A, triangle.C, triangle.A);
        Signed320 gram = WideArithmetic.MultiplySubtract(abAb, acAc, abAc, abAc);
        if (gram.Sign <= 0)
        {
            va = vb = vc = default;
            return false;
        }

        GetPointLocalNumerators(point, triangleOrigin, triangleRotation,
            out Signed576 pointX, out Signed576 pointY, out Signed576 pointZ,
            out Signed320 pointDenominator);
        Signed576 d1 = GetPointEdgeDotNumerator(pointX, pointY, pointZ,
            pointDenominator, triangle.A, triangle.B);
        Signed576 d2 = GetPointEdgeDotNumerator(pointX, pointY, pointZ,
            pointDenominator, triangle.A, triangle.C);
        vb = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(d1, acAc),
            WideArithmetic.MultiplySigned576(d2, abAc));
        vc = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(d2, abAb),
            WideArithmetic.MultiplySigned576(d1, abAc));
        va = WideArithmetic.SubtractSigned576(
            WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(gram, pointDenominator), vb), vc);
        return true;
    }

    // Clips the exact finite core projection before either witness is rounded.
    // Endpoint terms share a denominator, including odd raw half-length residues.
    internal static bool TryGetProjectedCoreParameter(
        FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
        in FixedPointAnchor firstCoreEnd, in FixedPointAnchor secondCoreEnd,
        Vector3d coreCenter, FixedQuaternion coreRotation, Vector3d coreAxis,
        Fixed64 coreLength, out Signed832 numerator, out Signed832 denominator)
    {
        if (TryGetProjectionBarycentrics(triangle, origin, rotation, firstCoreEnd,
                out Signed576 firstA, out Signed576 firstB, out Signed576 firstC))
        {
            _ = TryGetProjectionBarycentrics(triangle, origin, rotation, secondCoreEnd,
                out Signed576 secondA, out Signed576 secondB, out Signed576 secondC);
            Span<Signed576> first = stackalloc Signed576[3] { firstA, firstB, firstC };
            Span<Signed576> second = stackalloc Signed576[3] { secondA, secondB, secondC };
            Signed832 lowerN = default, lowerD = Signed832.ExtendValue(Signed192.Signed(1));
            Signed832 upperN = lowerD, upperD = lowerD;
            for (int index = 0; index < 3; index++)
            {
                if (first[index].Sign < 0 && second[index].Sign < 0)
                {
                    numerator = denominator = default;
                    return false;
                }
                if (first[index].Sign >= 0 && second[index].Sign >= 0)
                    continue;
                Signed576 difference = WideArithmetic.SubtractSigned576(second[index], first[index]);
                Signed832 n = Signed832.ExtendValue(WideArithmetic.SubtractSigned576(default, first[index]));
                Signed832 d = Signed832.ExtendValue(difference);
                if (d.Sign < 0)
                {
                    n = WideArithmetic.SubtractSigned832(default, n);
                    d = WideArithmetic.SubtractSigned832(default, d);
                    if (WideArithmetic.CompareNonNegativeProducts(n, upperD, upperN, d) < 0)
                    { upperN = n; upperD = d; }
                }
                else if (WideArithmetic.CompareNonNegativeProducts(n, lowerD, lowerN, d) > 0)
                { lowerN = n; lowerD = d; }
            }
            if (WideArithmetic.CompareNonNegativeProducts(lowerN, upperD, upperN, lowerD) > 0)
            {
                numerator = denominator = default;
                return false;
            }
            Signed832 twiceLower = WideArithmetic.AddSigned832(lowerN, lowerN);
            Signed832 twiceUpper = WideArithmetic.AddSigned832(upperN, upperN);
            Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
            Signed832 chosenN = default, chosenD = one;
            if (WideArithmetic.CompareNonNegativeProducts(twiceLower, one, lowerD, one) > 0)
            { chosenN = WideArithmetic.SubtractSigned832(twiceLower, lowerD); chosenD = lowerD; }
            else if (WideArithmetic.CompareNonNegativeProducts(twiceUpper, one, upperD, one) < 0)
            { chosenN = WideArithmetic.SubtractSigned832(twiceUpper, upperD); chosenD = upperD; }
            // Raw axial s = length * (2t - 1) / 2; fits 512 significant bits.
            numerator = WideArithmetic.MultiplySigned832(chosenN, Signed192.Raw(coreLength));
            denominator = WideArithmetic.AddSigned832(chosenD, chosenD);
            return true;
        }

        // A degenerate triangle owns its complete finite edge. Ranking its
        // duplicated vertices separately could replace a valid interior foot
        // with a closer-to-midpoint parameter that has tangential separation.
        Vector3d start = triangle.A, end = triangle.B;
        Signed192 longest = GetTriangleDifferenceDot(triangle.B, triangle.A, triangle.B, triangle.A);
        Signed192 bc = GetTriangleDifferenceDot(triangle.C, triangle.B, triangle.C, triangle.B);
        if (WideArithmetic.SubtractSigned192(bc, longest).Sign > 0)
        { start = triangle.B; end = triangle.C; longest = bc; }
        Signed192 ca = GetTriangleDifferenceDot(triangle.A, triangle.C, triangle.A, triangle.C);
        if (WideArithmetic.SubtractSigned192(ca, longest).Sign > 0)
        { start = triangle.C; end = triangle.A; }
        return TryGetProjectedSegmentCoreParameter(
            new FixedPointAnchor(origin, rotation, start),
            new FixedPointAnchor(origin, rotation, end), coreCenter, coreRotation,
            coreAxis, coreLength, out numerator, out denominator);
    }

    private static bool TryGetProjectedSegmentCoreParameter(
        in FixedPointAnchor start, in FixedPointAnchor end, Vector3d center,
        FixedQuaternion rotation, Vector3d axis, Fixed64 length,
        out Signed832 numerator, out Signed832 denominator)
    {
        GetPointLocalNumerators(start, center, rotation, out Signed576 px, out Signed576 py,
            out Signed576 pz, out Signed320 pointD);
        GetPointLocalNumerators(end, center, rotation, out Signed576 ex, out Signed576 ey,
            out Signed576 ez, out _);
        ex = WideArithmetic.SubtractSigned576(ex, px);
        ey = WideArithmetic.SubtractSigned576(ey, py);
        ez = WideArithmetic.SubtractSigned576(ez, pz);
        Signed576 ux = Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(axis.X)));
        Signed576 uy = Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(axis.Y)));
        Signed576 uz = Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(axis.Z)));
        Signed192 aa = Signed192.NarrowValue(WideArithmetic.GetDotProduct3D(
            Signed192.Raw(axis.X), Signed192.Raw(axis.Y), Signed192.Raw(axis.Z),
            Signed192.Raw(axis.X), Signed192.Raw(axis.Y), Signed192.Raw(axis.Z)));
        Signed576 bb = GetProjectedCoreDot(ex, ey, ez, ex, ey, ez);
        Signed320 ab = Signed320.NarrowValue(GetProjectedCoreDot(ex, ey, ez, ux, uy, uz));
        Signed320 ao = Signed320.NarrowValue(GetProjectedCoreDot(px, py, pz, ux, uy, uz));
        Signed576 bo = GetProjectedCoreDot(ex, ey, ez, px, py, pz);
        Signed576 determinant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(bb, aa), WideArithmetic.MultiplySigned320(ab, ab));
        if (determinant.IsZero)
        {
            Signed320 second = WideArithmetic.AddSigned320(ao, ab);
            Signed320 chosen = default;
            if (ao.Sign > 0 && second.Sign > 0)
                chosen = WideArithmetic.SubtractSigned320(ao, second).Sign <= 0 ? ao : second;
            else if (ao.Sign < 0 && second.Sign < 0)
                chosen = WideArithmetic.SubtractSigned320(ao, second).Sign >= 0 ? ao : second;
            numerator = WideArithmetic.MultiplySigned832(Signed832.ExtendValue(Signed576.ExtendValue(chosen)), Signed192.One);
            denominator = Signed832.ExtendValue(WideArithmetic.MultiplySigned320(pointD, Signed320.ExtendValue(aa)));
        }
        else
        {
            Signed576 edgeN = WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(ab, ao), WideArithmetic.MultiplySigned576(bo, aa));
            if (edgeN.Sign < 0 || WideArithmetic.SubtractSigned576(determinant, edgeN).Sign < 0)
            { numerator = denominator = default; return false; }
            // Normalized bases have <=65-bit coefficients and denominators.
            // Raw-origin differences and transformed points fit 201 bits;
            // Gram products fit 476, axial ratios 680, and final coordinates
            // 744 signed bits. All products therefore fit their wide owners.
            Signed704 coreN = WideArithmetic.SubtractSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(bb, ao),
                WideArithmetic.MultiplySigned576ToSigned704(bo, ab));
            numerator = WideArithmetic.MultiplySigned704ToSigned832(coreN, Signed192.One);
            denominator = Signed832.ExtendValue(WideArithmetic.MultiplySigned576ToSigned704(determinant, pointD));
        }
        Signed832 magnitude = numerator.Sign < 0 ? WideArithmetic.SubtractSigned832(default, numerator) : numerator;
        return WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned832(denominator, Signed192.Raw(length)),
            WideArithmetic.AddSigned832(magnitude, magnitude)).Sign >= 0;
    }

    private static Signed576 GetProjectedCoreDot(
        Signed576 ax, Signed576 ay, Signed576 az, Signed576 bx, Signed576 by, Signed576 bz) =>
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(Signed320.NarrowValue(ax), Signed320.NarrowValue(bx)),
            WideArithmetic.MultiplySigned320(Signed320.NarrowValue(ay), Signed320.NarrowValue(by))),
            WideArithmetic.MultiplySigned320(Signed320.NarrowValue(az), Signed320.NarrowValue(bz)));

    internal static int CompareCoreParameters(Signed832 firstN, Signed832 firstD, Signed832 secondN, Signed832 secondD) =>
        WideArithmetic.CompareNonNegativeProducts(
            firstN.Sign < 0 ? WideArithmetic.SubtractSigned832(default, firstN) : firstN, secondD,
            secondN.Sign < 0 ? WideArithmetic.SubtractSigned832(default, secondN) : secondN, firstD);

    internal static Vector3d MaterializeCorePoint(Vector3d axis, Signed832 numerator, Signed832 denominator)
    {
        Signed832 scaledD = WideArithmetic.MultiplySigned832(denominator, Signed192.One);
        _ = Fixed64.TryGetSignedRawRatio(WideArithmetic.MultiplySigned832(numerator, Signed192.Raw(axis.X)), scaledD, 0, out Fixed64 x);
        _ = Fixed64.TryGetSignedRawRatio(WideArithmetic.MultiplySigned832(numerator, Signed192.Raw(axis.Y)), scaledD, 0, out Fixed64 y);
        _ = Fixed64.TryGetSignedRawRatio(WideArithmetic.MultiplySigned832(numerator, Signed192.Raw(axis.Z)), scaledD, 0, out Fixed64 z);
        return new Vector3d(x, y, z);
    }

    internal static FixedPointAnchor GetClosestPointAnchor(
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        in FixedPointAnchor point,
        bool preserveExactFace = false)
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
            pointDenominator,
            preserveExactFace);
        return FixedPointAnchor.FromValidatedFrame(
            triangleOrigin,
            triangleRotation,
            closest);
    }

    private static Vector3d GetClosestTriangleLocalPoint(
        FixedTriangle triangle,
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed320 pointDenominator,
        bool preserveExactFace)
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
        // A certified support footprint must retain even a tiny exact face.
        if (preserveExactFace ? gram.Sign <= 0 : WideGeometry.IsQ128MagnitudeAtMostEpsilon(gram))
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

    internal static void GetPointLocalNumerators(
        in FixedPointAnchor point,
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z,
        out Signed320 denominator)
    {
        WideRationalBasis3d pointBasis = new(point.Rotation);
        Signed192 pointDenominator = point.ExactLocalTerm.IsZero
            ? pointBasis.Denominator
            : Signed192.NarrowValue(WideArithmetic.MultiplySigned192(
                pointBasis.Denominator, FixedPointAnchorTerm3d.Denominator));
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
        if (frameRotation == FixedQuaternion.Identity)
        {
            // The identity frame needs no projection or additional basis scale.
            denominator = Signed320.ExtendValue(pointDenominator);
            x = Signed576.ExtendValue(WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(originX, pointDenominator), rotatedX));
            y = Signed576.ExtendValue(WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(originY, pointDenominator), rotatedY));
            z = Signed576.ExtendValue(WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(originZ, pointDenominator), rotatedZ));
            return;
        }
        WideRationalBasis3d frameBasis = new(frameRotation);
        denominator = WideArithmetic.MultiplySigned192(pointDenominator, frameBasis.Denominator);
        x = GetTriangleFrameCoordinateNumerator(
            originX,
            originY,
            originZ,
            rotatedX,
            rotatedY,
            rotatedZ,
            pointDenominator,
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
            pointDenominator,
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
            pointDenominator,
            frameBasis.Zx,
            frameBasis.Zy,
            frameBasis.Zz);
    }

    private static Signed320 GetTriangleRotatedPointCoordinate(
        in FixedPointAnchor point,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ)
    {
        Signed320 rounded = WideArithmetic.AddSigned320(
            WideArithmetic.GetDotProduct3D(
                Signed192.Raw(point.LocalPoint.X),
                Signed192.Raw(point.LocalPoint.Y),
                Signed192.Raw(point.LocalPoint.Z),
                axisX,
                axisY,
                axisZ),
            WideArithmetic.GetDotProduct3D(
                Signed192.Raw(point.LocalDisplacement.X),
                Signed192.Raw(point.LocalDisplacement.Y),
                Signed192.Raw(point.LocalDisplacement.Z),
                axisX,
                axisY,
                axisZ));
        if (point.LocalTranslation != Vector3d.Zero)
            rounded = WideArithmetic.AddSigned320(rounded,
                WideArithmetic.GetDotProduct3D(
                    Signed192.Raw(point.LocalTranslation.X),
                    Signed192.Raw(point.LocalTranslation.Y),
                    Signed192.Raw(point.LocalTranslation.Z), axisX, axisY, axisZ));
        if (point.ExactLocalTerm.IsZero)
            return rounded;

        // Normalized rational bases, three scalar feature components and the
        // shared 33-bit residual denominator fit well within Signed320. The
        // frame transform and barycentric products remain within Signed576.
        Signed576 scaled = WideArithmetic.MultiplySigned320(rounded,
            Signed320.ExtendValue(FixedPointAnchorTerm3d.Denominator));
        Signed320 residual = WideArithmetic.GetDotProduct3D(
            Signed192.Signed(point.ExactLocalTerm.X),
            Signed192.Signed(point.ExactLocalTerm.Y),
            Signed192.Signed(point.ExactLocalTerm.Z), axisX, axisY, axisZ);
        return WideArithmetic.AddSigned320(Signed320.NarrowValue(scaled), residual);
    }

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
        Signed320 originProjection = WideArithmetic.GetDotProduct3D(
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
