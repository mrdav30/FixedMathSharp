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

    internal static bool TryGetProjectedSegmentInterval(
        FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
        in FixedPointAnchor first, in FixedPointAnchor second,
        out Signed832 lowerN, out Signed832 lowerD, out Signed832 upperN, out Signed832 upperD)
    {
        if (TryGetProjectionBarycentrics(triangle, origin, rotation, first,
                out Signed576 firstA, out Signed576 firstB, out Signed576 firstC))
        {
            _ = TryGetProjectionBarycentrics(triangle, origin, rotation, second,
                out Signed576 secondA, out Signed576 secondB, out Signed576 secondC);
            Span<Signed576> firstValues = stackalloc Signed576[3] { firstA, firstB, firstC };
            Span<Signed576> secondValues = stackalloc Signed576[3] { secondA, secondB, secondC };
            lowerN = default; lowerD = Signed832.ExtendValue(Signed192.Signed(1));
            upperN = lowerD; upperD = lowerD;
            for (int index = 0; index < 3; index++)
            {
                if (firstValues[index].Sign < 0 && secondValues[index].Sign < 0)
                {
                    lowerN = lowerD = upperN = upperD = default;
                    return false;
                }
                if (firstValues[index].Sign >= 0 && secondValues[index].Sign >= 0)
                    continue;
                Signed576 difference = WideArithmetic.SubtractSigned576(secondValues[index], firstValues[index]);
                Signed832 n = Signed832.ExtendValue(WideArithmetic.SubtractSigned576(default, firstValues[index]));
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
                lowerN = lowerD = upperN = upperD = default;
                return false;
            }
            return true;
        }
        lowerN = lowerD = upperN = upperD = default;
        return false;
    }

    internal static bool TryGetProjectedSegmentFaceAnchors(
        FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
        in FixedPointAnchor first, in FixedPointAnchor second, Signed832 n, Signed832 d,
        int physicalBoundaryMask, out FixedPointAnchor source, out FixedPointAnchor target)
    {
        source = target = default;
        System.Diagnostics.Debug.Assert(first.Origin == second.Origin && first.Rotation == second.Rotation
            && first.ExactLocalTerm.IsZero && second.ExactLocalTerm.IsZero);
        bool nondegenerate = TryGetProjectionBarycentrics(triangle, origin, rotation, first,
            out Signed576 a0, out Signed576 b0, out Signed576 c0);
        System.Diagnostics.Debug.Assert(nondegenerate);
        _ = TryGetProjectionBarycentrics(triangle, origin, rotation, second,
            out Signed576 a1, out Signed576 b1, out Signed576 c1);
        Span<Signed576> start = stackalloc Signed576[3] { a0, b0, c0 };
        Span<Signed576> end = stackalloc Signed576[3] { a1, b1, c1 };
        Span<ulong> magnitude = stackalloc ulong[13];
        int coefficientWords = 1, parameterWords = 1;
        for (int index = 0; index < 3; index++)
        {
            magnitude.Clear(); WideArithmetic.GetMagnitude(start[index], magnitude);
            coefficientWords = Math.Max(coefficientWords, WideArithmetic.GetActiveMagnitudeLength(magnitude));
            magnitude.Clear(); WideArithmetic.GetMagnitude(end[index], magnitude);
            coefficientWords = Math.Max(coefficientWords, WideArithmetic.GetActiveMagnitudeLength(magnitude));
        }
        WideArithmetic.GetMagnitude(n, magnitude);
        parameterWords = Math.Max(parameterWords, WideArithmetic.GetActiveMagnitudeLength(magnitude));
        WideArithmetic.GetMagnitude(d, magnitude);
        parameterWords = Math.Max(parameterWords, WideArithmetic.GetActiveMagnitudeLength(magnitude));
        // Products need both original coefficient widths, one raw-coordinate limb and sum carries.
        int words = Math.Max(9, coefficientWords + parameterWords + 3);
        Span<ulong> values = stackalloc ulong[20 * words]; Span<int> signs = stackalloc int[20];
        ContactQuadratic parameter = ContactQuadratic.At(values, signs, 0, words),
            divisor = ContactQuadratic.At(values, signs, 1, words), remaining = ContactQuadratic.At(values, signs, 2, words),
            denominator = ContactQuadratic.At(values, signs, 6, words), numerator = ContactQuadratic.At(values, signs, 7, words),
            term = ContactQuadratic.At(values, signs, 8, words), coefficient = ContactQuadratic.At(values, signs, 9, words);
        Import(n, parameter); Import(d, divisor); divisor.CopyTo(remaining); remaining.Add(parameter, -1);
        System.Diagnostics.Debug.Assert(n.Sign > 0 && remaining.Signs[0] > 0);
        bool physical = false;
        denominator.Clear();
        for (int index = 0; index < 3; index++)
        {
            ContactQuadratic barycentric = ContactQuadratic.At(values, signs, 3 + index, words);
            coefficient.Set(start[index]); ContactQuadratic.Multiply(coefficient, remaining, ReadOnlySpan<ulong>.Empty, barycentric);
            coefficient.Set(end[index]); ContactQuadratic.Multiply(coefficient, parameter, ReadOnlySpan<ulong>.Empty, term);
            barycentric.Add(term);
            System.Diagnostics.Debug.Assert(barycentric.Signs[0] >= 0);
            physical |= barycentric.Signs[0] == 0 && (physicalBoundaryMask & (1 << index)) != 0;
            denominator.Add(barycentric);
        }
        if (!physical) return false;
        Vector3d sourcePoint = default, targetPoint = default;
        Span<Vector3d> corners = stackalloc Vector3d[3] { triangle.A, triangle.B, triangle.C };
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic.Scale(remaining, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(first.LocalPoint[axis]))), numerator);
            ContactQuadratic.Scale(parameter, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(second.LocalPoint[axis]))), term);
            numerator.Add(term); sourcePoint[axis] = ContactQuadratic.RoundRatio(numerator, divisor, ReadOnlySpan<ulong>.Empty);
            numerator.Clear();
            for (int index = 0; index < 3; index++)
            {
                ContactQuadratic.Scale(ContactQuadratic.At(values, signs, 3 + index, words),
                    Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(corners[index][axis]))), term);
                numerator.Add(term);
            }
            targetPoint[axis] = ContactQuadratic.RoundRatio(numerator, denominator, ReadOnlySpan<ulong>.Empty);
        }
        source = new FixedPointAnchor(first.Origin, first.Rotation, sourcePoint);
        target = new FixedPointAnchor(origin, rotation, targetPoint);
        return true;

        static void Import(Signed832 value, ContactQuadratic destination)
        {
            Span<ulong> magnitude = stackalloc ulong[13]; WideArithmetic.GetMagnitude(value, magnitude);
            destination.Clear(); magnitude[..WideArithmetic.GetActiveMagnitudeLength(magnitude)].CopyTo(destination.Rational);
            destination.Signs[0] = value.Sign;
        }
    }

    // Clips the exact finite core projection before either witness is rounded.
    // Endpoint terms share a denominator, including odd raw half-length residues.
    internal static bool TryGetProjectedCoreParameter(
        FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
        in FixedPointAnchor firstCoreEnd, in FixedPointAnchor secondCoreEnd,
        Vector3d coreCenter, FixedQuaternion coreRotation, Vector3d coreAxis,
        Fixed64 coreLength, out Signed832 numerator, out Signed832 denominator)
    {
        if (TryGetProjectedSegmentInterval(triangle, origin, rotation, firstCoreEnd, secondCoreEnd,
                out Signed832 lowerN, out Signed832 lowerD, out Signed832 upperN, out Signed832 upperD))
        {
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

        if (TryGetProjectionBarycentrics(triangle, origin, rotation, firstCoreEnd, out _, out _, out _))
        {
            numerator = denominator = default;
            return false;
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
            GetClosestNonparallelLineNumerators(aa, bb, ab, ao, bo, out Signed576 edgeN, out Signed704 coreN);
            if (edgeN.Sign < 0 || WideArithmetic.SubtractSigned576(determinant, edgeN).Sign < 0)
            { numerator = denominator = default; return false; }
            // Normalized bases have <=65-bit coefficients and denominators.
            // Raw-origin differences and transformed points fit 201 bits;
            // Gram products fit 476, axial ratios 680, and final coordinates
            // 744 signed bits. All products therefore fit their wide owners.
            numerator = WideArithmetic.MultiplySigned704ToSigned832(coreN, Signed192.One);
            denominator = Signed832.ExtendValue(WideArithmetic.MultiplySigned576ToSigned704(determinant, pointD));
        }
        Signed832 magnitude = numerator.Sign < 0 ? WideArithmetic.SubtractSigned832(default, numerator) : numerator;
        return WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned832(denominator, Signed192.Raw(length)),
            WideArithmetic.AddSigned832(magnitude, magnitude)).Sign >= 0;
    }

    private static void GetClosestNonparallelLineNumerators(Signed192 aa, Signed576 bb,
        Signed320 ab, Signed320 ao, Signed576 bo, out Signed576 edgeN, out Signed704 coreN)
    {
        edgeN = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(ab, ao), WideArithmetic.MultiplySigned576(bo, aa));
        coreN = WideArithmetic.SubtractSigned704(WideArithmetic.MultiplySigned576ToSigned704(bb, ao),
            WideArithmetic.MultiplySigned576ToSigned704(bo, ab));
    }

    internal static bool TryGetClosestNonparallelSegmentAnchors(
        Vector3d firstOrigin, FixedQuaternion firstRotation, FixedSegment first,
        Vector3d secondOrigin, FixedQuaternion secondRotation, FixedSegment second,
        out FixedPointAnchor firstAnchor, out FixedPointAnchor secondAnchor)
    {
        firstAnchor = secondAnchor = default;
        GetPointLocalNumerators(new FixedPointAnchor(firstOrigin, firstRotation, first.Start), secondOrigin, secondRotation,
            out Signed576 px, out Signed576 py, out Signed576 pz, out Signed320 pointD);
        GetPointLocalNumerators(new FixedPointAnchor(firstOrigin, firstRotation, first.End), secondOrigin, secondRotation,
            out Signed576 ex, out Signed576 ey, out Signed576 ez, out _);
        ex = WideArithmetic.SubtractSigned576(ex, px); ey = WideArithmetic.SubtractSigned576(ey, py);
        ez = WideArithmetic.SubtractSigned576(ez, pz);
        px = WideArithmetic.SubtractSigned576(px, WideArithmetic.MultiplySigned320(pointD, Signed192.Raw(second.Start.X)));
        py = WideArithmetic.SubtractSigned576(py, WideArithmetic.MultiplySigned320(pointD, Signed192.Raw(second.Start.Y)));
        pz = WideArithmetic.SubtractSigned576(pz, WideArithmetic.MultiplySigned320(pointD, Signed192.Raw(second.Start.Z)));
        Signed576 ux = Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.Difference(second.End.X, second.Start.X))),
            uy = Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.Difference(second.End.Y, second.Start.Y))),
            uz = Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.Difference(second.End.Z, second.Start.Z)));
        Signed192 aa = Signed192.NarrowValue(GetProjectedCoreDot(ux, uy, uz, ux, uy, uz));
        Signed576 bb = GetProjectedCoreDot(ex, ey, ez, ex, ey, ez), bo = GetProjectedCoreDot(ex, ey, ez, px, py, pz);
        Signed320 ab = Signed320.NarrowValue(GetProjectedCoreDot(ex, ey, ez, ux, uy, uz)),
            ao = Signed320.NarrowValue(GetProjectedCoreDot(px, py, pz, ux, uy, uz));
        Signed576 determinant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(bb, aa), WideArithmetic.MultiplySigned320(ab, ab));
        if (determinant.Sign <= 0) return false;
        GetClosestNonparallelLineNumerators(aa, bb, ab, ao, bo, out Signed576 firstN, out Signed704 secondN);
        Signed704 secondD = WideArithmetic.MultiplySigned576ToSigned704(determinant, pointD);
        if (firstN.Sign < 0 || WideArithmetic.SubtractSigned576(determinant, firstN).Sign < 0
            || secondN.Sign < 0 || WideArithmetic.SubtractSigned704(secondD, secondN).Sign < 0) return false;
        // Rigid numerators fit 202 bits; line minors fit 540/680, and final local coordinates fit 744/808.
        firstAnchor = new FixedPointAnchor(firstOrigin, firstRotation,
            Materialize(first, Signed832.ExtendValue(firstN), Signed832.ExtendValue(determinant)));
        secondAnchor = new FixedPointAnchor(secondOrigin, secondRotation,
            Materialize(second, Signed832.ExtendValue(secondN), Signed832.ExtendValue(secondD)));
        return true;

        static Vector3d Materialize(FixedSegment segment, Signed832 n, Signed832 d)
        {
            Vector3d point = default;
            Signed832 remaining = WideArithmetic.SubtractSigned832(d, n);
            for (int axis = 0; axis < 3; axis++)
            {
                Signed832 coordinate = WideArithmetic.AddSigned832(
                    WideArithmetic.MultiplySigned832(remaining, Signed192.Raw(segment.Start[axis])),
                    WideArithmetic.MultiplySigned832(n, Signed192.Raw(segment.End[axis])));
                bool represented = Fixed64.TryGetSignedRawRatio(coordinate, d, 0, out Fixed64 component);
                System.Diagnostics.Debug.Assert(represented); point[axis] = component;
            }
            return point;
        }
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
