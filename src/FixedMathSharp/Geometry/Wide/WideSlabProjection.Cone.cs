//=======================================================================
// WideSlabProjection.Cone.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Cone-specific support computations for wide slab projection, including
/// vertical and rotated cone lateral/cap support queries using exact
/// wide-precision arithmetic.
/// </content>
internal static partial class WideSlabProjection
{
    private static bool TryGetVerticalConeSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        out Vector2d support)
    {
        Signed320 apexY = GetConeEndpointNumerator(center.Y, axis.Y, height, 1);
        Signed320 baseY = GetConeEndpointNumerator(center.Y, axis.Y, height, -1);
        Signed320 lower = WideArithmetic.MultiplySigned192(Signed192.Raw(slab.Min), WideArithmetic.Double(Scale));
        Signed320 upper = WideArithmetic.MultiplySigned192(Signed192.Raw(slab.Max), WideArithmetic.Double(Scale));
        bool pointsUp = axis.Y.m_rawValue >= 0L;
        Signed320 selectedY = pointsUp ? Maximum(baseY, lower) : Minimum(baseY, upper);
        bool outsideCone = pointsUp
            ? Compare(selectedY, apexY) > 0
            : Compare(selectedY, apexY) < 0;
        if (Compare(selectedY, lower) < 0 || Compare(selectedY, upper) > 0 || outsideCone)
        {
            support = default;
            return false;
        }

        Signed320 axial = axis.Y.m_rawValue >= 0L
            ? WideArithmetic.SubtractSigned320(apexY, selectedY)
            : WideArithmetic.SubtractSigned320(selectedY, apexY);
        Signed576 radiusNumerator = WideArithmetic.MultiplySigned320(axial, Signed192.Raw(radius));
        Signed320 heightDenominator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(Signed192.Raw(height), Signed192.Raw(axis.Y.Abs())),
            WideArithmetic.MultiplySigned192(Signed192.Raw(height), Signed192.Raw(axis.Y.Abs())));
        Signed576 planarRadiusNumerator = radiusNumerator;
        Signed576 planarRadiusDenominator = Signed576.ExtendValue(heightDenominator);
        Signed192 directionLength = GetPlanarDirectionLength(direction);
        Signed576 denominator = WideArithmetic.MultiplySigned576(planarRadiusDenominator, directionLength);
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(denominator, Signed192.Raw(center.X)),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(planarRadiusNumerator, Signed192.Raw(direction.X)), Scale));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(denominator, Signed192.Raw(center.Z)),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(planarRadiusNumerator, Signed192.Raw(direction.Y)), Scale));
        return TryCreateResult(true, new WidePlanarCandidate(x, z, denominator), out support);
    }

    private static bool TryGetRotatedConeSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        out Vector2d support)
    {
        // Endpoint/cap candidates are exact. Lateral slab-plane candidates use
        // the cone's exact centered containment polynomial to select the
        // representable planar boundary witness without constructing a
        // saturated cone endpoint.
        bool found = false;
        WidePlanarCandidate best = default;
        AddConeApex(center, axis, height, slab, direction, ref found, ref best);
        AddConeBaseDisk(center, axis, height, radius, slab, direction, axisLengthSquared, ref found, ref best);
        AddConeBasePlaneCandidate(center, axis, height, radius, slab.Min, direction, ref found, ref best);
        if (slab.Max != slab.Min)
            AddConeBasePlaneCandidate(center, axis, height, radius, slab.Max, direction, ref found, ref best);
        AddConeLateralCandidates(center, axis, height, radius, slab, direction, ref found, ref best);
        return TryCreateResult(found, best, out support);
    }

    private static void AddConeApex(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        FixedRange slab,
        Vector2d direction,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed320 y = GetConeEndpointNumerator(center.Y, axis.Y, height, 1);
        Signed192 denominator = WideArithmetic.Double(Scale);
        if (!IsInRange(y, denominator, slab))
            return;

        KeepBest(
            new WidePlanarCandidate(
                Signed576.ExtendValue(GetConeEndpointNumerator(center.X, axis.X, height, 1)),
                Signed576.ExtendValue(GetConeEndpointNumerator(center.Z, axis.Z, height, 1)),
                Signed576.ExtendValue(Signed320.ExtendValue(denominator))),
            direction,
            ref found,
            ref best);
    }

    private static void AddConeBaseDisk(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        // Double every centered quantity so an odd raw-unit height is retained.
        Signed320 baseX = GetConeEndpointNumerator(center.X, axis.X, height, -1);
        Signed320 baseY = GetConeEndpointNumerator(center.Y, axis.Y, height, -1);
        Signed320 baseZ = GetConeEndpointNumerator(center.Z, axis.Z, height, -1);
        AddDiskAtRationalCenter(baseX, baseY, baseZ, WideArithmetic.Double(Scale), axis, radius, slab, direction, axisLengthSquared, ref found, ref best);
    }

    private static void AddConeBasePlaneCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        Fixed64 plane,
        Vector2d direction,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        const ulong baseParameter = (ulong)long.MaxValue;
        if (TryCreateConeDiskPlaneCandidate(
            center, axis, height, radius, plane, direction, baseParameter, out WidePlanarCandidate baseCandidate))
        {
            KeepBest(baseCandidate, direction, ref found, ref best);
        }

    }

    private static void AddConeLateralCandidates(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        GetConeLateralStationaryPolynomial(axis, height, radius, direction,
            out Signed576 coefficient, out Signed576 projection, out Signed576 constant);
        if (coefficient.IsZero)
        {
            if (!projection.IsZero)
            {
                Signed576 doubleProjection = WideArithmetic.AddSigned576(projection, projection);
                Signed576 absoluteProjection = WideArithmetic.Absolute(doubleProjection);
                Signed576 y = WideArithmetic.SubtractSigned576(default, constant);
                if (doubleProjection.Sign < 0)
                    y = WideArithmetic.SubtractSigned576(default, y);
                AddConeLateralNormal(center, axis, height, radius, slab, direction,
                    WideArithmetic.MultiplySigned576(absoluteProjection, Signed192.Raw(direction.X)), y,
                    WideArithmetic.MultiplySigned576(absoluteProjection, Signed192.Raw(direction.Y)), ref found, ref best);
            }
            return;
        }

        Signed832 discriminant = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(projection, projection),
            WideArithmetic.MultiplySigned576ToSigned832(coefficient, constant));
        if (discriminant.Sign < 0)
            return;

        Signed576 squareRoot = WideArithmetic.GetFloorSquareRootOfProduct(discriminant, ScaleSquared);
        Signed576 scaledProjection = WideArithmetic.MultiplySigned576(projection, Scale);
        Signed576 scaledCoefficient = WideArithmetic.MultiplySigned576(coefficient, Scale);
        Signed576 absoluteCoefficient = WideArithmetic.Absolute(scaledCoefficient);
        Signed576 negativeProjection = WideArithmetic.SubtractSigned576(default, scaledProjection);
        Signed576 firstY = WideArithmetic.SubtractSigned576(negativeProjection, squareRoot);
        if (coefficient.Sign < 0)
            firstY = WideArithmetic.SubtractSigned576(default, firstY);
        AddConeLateralNormal(center, axis, height, radius, slab, direction,
            WideArithmetic.MultiplySigned576(absoluteCoefficient, Signed192.Raw(direction.X)), firstY,
            WideArithmetic.MultiplySigned576(absoluteCoefficient, Signed192.Raw(direction.Y)), ref found, ref best);
        if (!squareRoot.IsZero)
        {
            Signed576 secondY = WideArithmetic.AddSigned576(negativeProjection, squareRoot);
            if (coefficient.Sign < 0)
                secondY = WideArithmetic.SubtractSigned576(default, secondY);
            AddConeLateralNormal(center, axis, height, radius, slab, direction,
                WideArithmetic.MultiplySigned576(absoluteCoefficient, Signed192.Raw(direction.X)), secondY,
                WideArithmetic.MultiplySigned576(absoluteCoefficient, Signed192.Raw(direction.Y)), ref found, ref best);
        }
    }

    private static void GetConeLateralStationaryPolynomial(
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        Vector2d direction,
        out Signed576 coefficient,
        out Signed576 projection,
        out Signed576 constant)
    {
        Signed192 q = GetAxisLengthSquared(axis);
        Signed192 s = GetPlanarDot(axis, direction);
        Signed192 d = GetPlanarDirectionLengthSquared(direction);
        Signed320 heightSquared = WideArithmetic.MultiplySigned192(Signed192.Raw(height), Signed192.Raw(height));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(Signed192.Raw(radius), Signed192.Raw(radius));
        Signed320 k = Signed320.NarrowValue(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(heightSquared), q),
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(radiusSquared), ScaleSquared)));
        Signed320 axisYSquared = WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Y), Signed192.Raw(axis.Y));
        Signed576 radiusQScaleSquared = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(radiusSquared), q), ScaleSquared);

        coefficient = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(k, axisYSquared),
            radiusQScaleSquared);
        projection = WideArithmetic.MultiplySigned320(
            k,
            WideArithmetic.MultiplySigned192(s, Signed192.Raw(axis.Y)));
        constant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(k, WideArithmetic.MultiplySigned192(s, s)),
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(
                    WideArithmetic.MultiplySigned576(Signed576.ExtendValue(radiusSquared), q), d),
                ScaleSquared));
    }

    private static void AddConeLateralNormal(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed576 nx,
        Signed576 ny,
        Signed576 nz,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed576 axial = SumProducts(axis, nx, ny, nz);
        if (axial.Sign < 0 || (radius != Fixed64.Zero && axial.IsZero))
            return;

        Signed192 q = GetAxisLengthSquared(axis);
        Signed576 gx = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned576(nx, q), WideArithmetic.MultiplySigned576(axial, Signed192.Raw(axis.X)));
        Signed576 gy = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned576(ny, q), WideArithmetic.MultiplySigned576(axial, Signed192.Raw(axis.Y)));
        Signed576 gz = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned576(nz, q), WideArithmetic.MultiplySigned576(axial, Signed192.Raw(axis.Z)));
        Signed832 radialSquared = Add(
            Add(
                WideArithmetic.MultiplySigned576ToSigned832(gx, gx),
                WideArithmetic.MultiplySigned576ToSigned832(gy, gy)),
            WideArithmetic.MultiplySigned576ToSigned832(gz, gz));
        Signed576 radialLength = WideArithmetic.GetFloorSquareRootOfProduct(radialSquared, ScaleSquared);
        ReduceDirection(gx, gy, gz, radialLength,
            out Signed192 reducedX, out Signed192 reducedY, out Signed192 reducedZ, out Signed192 reducedLength);
        Signed192 doubleScale = WideArithmetic.Double(Scale);
        Signed320 apexX = GetConeEndpointNumerator(center.X, axis.X, height, 1);
        Signed320 apexY = GetConeEndpointNumerator(center.Y, axis.Y, height, 1);
        Signed320 apexZ = GetConeEndpointNumerator(center.Z, axis.Z, height, 1);
        Signed320 baseX = GetConeEndpointNumerator(center.X, axis.X, height, -1);
        Signed320 baseY = GetConeEndpointNumerator(center.Y, axis.Y, height, -1);
        Signed320 baseZ = GetConeEndpointNumerator(center.Z, axis.Z, height, -1);
        Signed192 radialScale = WideArithmetic.Double(Signed192.NarrowValue(WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(radius))), ScaleSquared)));
        Signed576 rimX = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(baseX, reducedLength), Signed576.ExtendValue(WideArithmetic.MultiplySigned192(reducedX, radialScale)));
        Signed576 rimY = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(baseY, reducedLength), Signed576.ExtendValue(WideArithmetic.MultiplySigned192(reducedY, radialScale)));
        Signed576 rimZ = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(baseZ, reducedLength), Signed576.ExtendValue(WideArithmetic.MultiplySigned192(reducedZ, radialScale)));
        Signed576 apexScaleX = WideArithmetic.MultiplySigned320(apexX, reducedLength);
        Signed576 apexScaleY = WideArithmetic.MultiplySigned320(apexY, reducedLength);
        Signed576 apexScaleZ = WideArithmetic.MultiplySigned320(apexZ, reducedLength);
        Signed576 ux = WideArithmetic.SubtractSigned576(rimX, apexScaleX);
        Signed576 uy = WideArithmetic.SubtractSigned576(rimY, apexScaleY);
        Signed576 uz = WideArithmetic.SubtractSigned576(rimZ, apexScaleZ);
        AddConeGeneratorPlane(apexX, apexY, apexZ, ux, uy, uz, reducedLength,
            slab.Min, direction, doubleScale, ref found, ref best);
        if (slab.Max != slab.Min)
        {
            AddConeGeneratorPlane(apexX, apexY, apexZ, ux, uy, uz, reducedLength,
                slab.Max, direction, doubleScale, ref found, ref best);
        }
    }

    private static void AddConeGeneratorPlane(
        Signed320 apexX,
        Signed320 apexY,
        Signed320 apexZ,
        Signed576 ux,
        Signed576 uy,
        Signed576 uz,
        Signed192 radialLength,
        Fixed64 plane,
        Vector2d direction,
        Signed192 doubleScale,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed320 planeOffset = WideArithmetic.SubtractSigned320(WideArithmetic.MultiplySigned192(Signed192.Raw(plane), doubleScale), apexY);
        Signed576 scaledPlaneOffset = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(planeOffset), radialLength);
        if (!IsUnitInterval(scaledPlaneOffset, uy))
            return;

        Signed192 planeOffsetNarrow = Signed192.NarrowValue(planeOffset);
        Signed576 candidateX = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned576(uy, Signed192.NarrowValue(apexX)), WideArithmetic.MultiplySigned576(ux, planeOffsetNarrow));
        Signed576 candidateZ = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned576(uy, Signed192.NarrowValue(apexZ)), WideArithmetic.MultiplySigned576(uz, planeOffsetNarrow));
        Signed576 candidateDenominator = WideArithmetic.MultiplySigned576(uy, doubleScale);
        Normalize(ref candidateX, ref candidateZ, ref candidateDenominator);
        KeepBest(new WidePlanarCandidate(candidateX, candidateZ, candidateDenominator), direction, ref found, ref best);
    }

    private static bool TryCreateConeDiskPlaneCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        Fixed64 plane,
        Vector2d direction,
        ulong parameter,
        out WidePlanarCandidate candidate)
    {
        Signed192 maximum = Signed192.Signed(long.MaxValue);
        Signed192 parameterValue = Signed192.Signed((long)parameter);
        Signed192 doubleScale = WideArithmetic.Double(Scale);
        Signed320 centerDenominator = WideArithmetic.MultiplySigned192(doubleScale, maximum);
        Signed192 axialWeight = WideArithmetic.SubtractSigned192(
            maximum,
            WideArithmetic.AddSigned192(parameterValue, parameterValue));
        Signed320 centerX = GetConeDiskCenterNumerator(center.X, axis.X, height, maximum, axialWeight);
        Signed320 centerY = GetConeDiskCenterNumerator(center.Y, axis.Y, height, maximum, axialWeight);
        Signed320 centerZ = GetConeDiskCenterNumerator(center.Z, axis.Z, height, maximum, axialWeight);
        Signed320 radiusNumerator = WideArithmetic.MultiplySigned192(Signed192.Raw(radius), parameterValue);
        Signed192 q = GetAxisLengthSquared(axis);
        Signed192 planarAxisSquared = WideArithmetic.SubtractSigned192(
            q,
            Signed192.NarrowValue(WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Y), Signed192.Raw(axis.Y))));
        Signed320 k = Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(centerDenominator), Signed192.Raw(plane)),
            Signed576.ExtendValue(centerY)));

        Signed192 radiusNarrow = Signed192.NarrowValue(radiusNumerator);
        Signed192 centerDenominatorNarrow = Signed192.NarrowValue(centerDenominator);
        Signed192 kNarrow = Signed192.NarrowValue(k);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(radiusNarrow, radiusNarrow);
        Signed320 centerDenominatorSquared = WideArithmetic.MultiplySigned192(
            centerDenominatorNarrow,
            centerDenominatorNarrow);
        Signed576 first = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(radiusSquared, centerDenominatorSquared),
            planarAxisSquared);
        Signed320 kSquared = WideArithmetic.MultiplySigned192(kNarrow, kNarrow);
        Signed320 qParameterSquared = Signed320.NarrowValue(WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(WideArithmetic.MultiplySigned192(maximum, maximum)),
            q));
        Signed576 second = WideArithmetic.MultiplySigned320(kSquared, qParameterSquared);
        Signed576 radicand = WideArithmetic.SubtractSigned576(first, second);
        if (radicand.Sign < 0)
        {
            candidate = default;
            return false;
        }

        Signed320 root = WideArithmetic.GetFloorSquareRootScaledByFixed64(radicand);
        Signed320 denominator320 = Signed320.NarrowValue(WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(Signed576.ExtendValue(centerDenominator), maximum),
                planarAxisSquared),
            Scale));
        Signed576 denominatorWide = Signed576.ExtendValue(denominator320);
        Signed576 common = WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(Signed320.ExtendValue(maximum)),
            planarAxisSquared), Scale);
        Signed576 x = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(centerX), Signed192.NarrowValue(common));
        Signed576 z = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(centerZ), Signed192.NarrowValue(common));
        Signed576 particularScale = WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(Signed320.ExtendValue(maximum)),
            Scale), Signed192.Raw(axis.Y));
        x = WideArithmetic.SubtractSigned576(x, WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(particularScale, Signed192.Raw(axis.X)), kNarrow));
        z = WideArithmetic.SubtractSigned576(z, WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(particularScale, Signed192.Raw(axis.Z)), kNarrow));
        Signed576 tangentX = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(root), Signed192.Raw(-axis.Z));
        Signed576 tangentZ = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(root), Signed192.Raw(axis.X));
        WidePlanarCandidate firstCandidate = new(
            WideArithmetic.AddSigned576(x, tangentX),
            WideArithmetic.AddSigned576(z, tangentZ),
            denominatorWide);
        WidePlanarCandidate secondCandidate = new(
            WideArithmetic.SubtractSigned576(x, tangentX),
            WideArithmetic.SubtractSigned576(z, tangentZ),
            denominatorWide);
        candidate = CompareProjection(firstCandidate, secondCandidate, direction) >= 0
            ? firstCandidate
            : secondCandidate;
        return true;
    }

    private static Signed320 GetConeDiskCenterNumerator(
        Fixed64 center,
        Fixed64 axis,
        Fixed64 height,
        Signed192 maximum,
        Signed192 axialWeight) =>
        WideArithmetic.AddSigned320(
            Signed320.NarrowValue(WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(center))),
                WideArithmetic.Double(Scale)), maximum)),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(WideArithmetic.MultiplySigned192(Signed192.Raw(axis), Signed192.Raw(height)), axialWeight)));

}
