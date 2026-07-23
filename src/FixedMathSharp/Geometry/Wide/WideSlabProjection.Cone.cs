//=======================================================================
// WideSlabProjection.Cone.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

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
        Signed320 lower = Multiply(Raw(slab.Min), Double(Scale));
        Signed320 upper = Multiply(Raw(slab.Max), Double(Scale));
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
        Signed576 radiusNumerator = Multiply(axial, Raw(radius));
        Signed320 heightDenominator = WideArithmetic.AddSigned320(
            Multiply(Raw(height), Raw(axis.Y.Abs())),
            Multiply(Raw(height), Raw(axis.Y.Abs())));
        Signed576 planarRadiusNumerator = radiusNumerator;
        Signed576 planarRadiusDenominator = WideArithmetic.ExtendToSigned576(heightDenominator);
        Signed192 directionLength = GetPlanarDirectionLength(direction);
        Signed576 denominator = Multiply(planarRadiusDenominator, directionLength);
        Signed576 x = WideArithmetic.AddSigned576(
            Multiply(denominator, Raw(center.X)),
            Multiply(Multiply(planarRadiusNumerator, Raw(direction.X)), Scale));
        Signed576 z = WideArithmetic.AddSigned576(
            Multiply(denominator, Raw(center.Z)),
            Multiply(Multiply(planarRadiusNumerator, Raw(direction.Y)), Scale));
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
        Signed192 denominator = Double(Scale);
        if (!IsInRange(y, denominator, slab))
            return;

        KeepBest(
            new WidePlanarCandidate(
                WideArithmetic.ExtendToSigned576(GetConeEndpointNumerator(center.X, axis.X, height, 1)),
                WideArithmetic.ExtendToSigned576(GetConeEndpointNumerator(center.Z, axis.Z, height, 1)),
                WideArithmetic.ExtendToSigned576(WideArithmetic.ExtendToSigned320(denominator))),
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
        AddDiskAtRationalCenter(baseX, baseY, baseZ, Double(Scale), axis, radius, slab, direction, axisLengthSquared, ref found, ref best);
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
                Signed576 absoluteProjection = Absolute(doubleProjection);
                Signed576 y = WideArithmetic.SubtractSigned576(default, constant);
                if (doubleProjection.Sign < 0)
                    y = WideArithmetic.SubtractSigned576(default, y);
                AddConeLateralNormal(center, axis, height, radius, slab, direction,
                    Multiply(absoluteProjection, Raw(direction.X)), y,
                    Multiply(absoluteProjection, Raw(direction.Y)), ref found, ref best);
            }
            return;
        }

        Signed832 discriminant = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(projection, projection),
            WideArithmetic.MultiplySigned576ToSigned832(coefficient, constant));
        if (discriminant.Sign < 0)
            return;

        Signed576 squareRoot = WideArithmetic.GetFloorSquareRootOfProduct(discriminant, ScaleSquared);
        Signed576 scaledProjection = Multiply(projection, Scale);
        Signed576 scaledCoefficient = Multiply(coefficient, Scale);
        Signed576 absoluteCoefficient = Absolute(scaledCoefficient);
        Signed576 negativeProjection = WideArithmetic.SubtractSigned576(default, scaledProjection);
        Signed576 firstY = WideArithmetic.SubtractSigned576(negativeProjection, squareRoot);
        if (coefficient.Sign < 0)
            firstY = WideArithmetic.SubtractSigned576(default, firstY);
        AddConeLateralNormal(center, axis, height, radius, slab, direction,
            Multiply(absoluteCoefficient, Raw(direction.X)), firstY,
            Multiply(absoluteCoefficient, Raw(direction.Y)), ref found, ref best);
        if (!squareRoot.IsZero)
        {
            Signed576 secondY = WideArithmetic.AddSigned576(negativeProjection, squareRoot);
            if (coefficient.Sign < 0)
                secondY = WideArithmetic.SubtractSigned576(default, secondY);
            AddConeLateralNormal(center, axis, height, radius, slab, direction,
                Multiply(absoluteCoefficient, Raw(direction.X)), secondY,
                Multiply(absoluteCoefficient, Raw(direction.Y)), ref found, ref best);
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
        Signed320 heightSquared = WideArithmetic.MultiplySigned192(Raw(height), Raw(height));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(Raw(radius), Raw(radius));
        Signed320 k = Narrow(WideArithmetic.AddSigned576(
            Multiply(WideArithmetic.ExtendToSigned576(heightSquared), q),
            Multiply(WideArithmetic.ExtendToSigned576(radiusSquared), ScaleSquared)));
        Signed320 axisYSquared = WideArithmetic.MultiplySigned192(Raw(axis.Y), Raw(axis.Y));
        Signed576 radiusQScaleSquared = Multiply(
            Multiply(WideArithmetic.ExtendToSigned576(radiusSquared), q), ScaleSquared);

        coefficient = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(k, axisYSquared),
            radiusQScaleSquared);
        projection = WideArithmetic.MultiplySigned320(
            k,
            WideArithmetic.MultiplySigned192(s, Raw(axis.Y)));
        constant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(k, WideArithmetic.MultiplySigned192(s, s)),
            Multiply(Multiply(Multiply(WideArithmetic.ExtendToSigned576(radiusSquared), q), d), ScaleSquared));
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
        Signed576 gx = WideArithmetic.SubtractSigned576(Multiply(nx, q), Multiply(axial, Raw(axis.X)));
        Signed576 gy = WideArithmetic.SubtractSigned576(Multiply(ny, q), Multiply(axial, Raw(axis.Y)));
        Signed576 gz = WideArithmetic.SubtractSigned576(Multiply(nz, q), Multiply(axial, Raw(axis.Z)));
        Signed832 radialSquared = Add(
            Add(
                WideArithmetic.MultiplySigned576ToSigned832(gx, gx),
                WideArithmetic.MultiplySigned576ToSigned832(gy, gy)),
            WideArithmetic.MultiplySigned576ToSigned832(gz, gz));
        Signed576 radialLength = WideArithmetic.GetFloorSquareRootOfProduct(radialSquared, ScaleSquared);
        ReduceDirection(gx, gy, gz, radialLength,
            out Signed192 reducedX, out Signed192 reducedY, out Signed192 reducedZ, out Signed192 reducedLength);
        Signed192 doubleScale = Double(Scale);
        Signed320 apexX = GetConeEndpointNumerator(center.X, axis.X, height, 1);
        Signed320 apexY = GetConeEndpointNumerator(center.Y, axis.Y, height, 1);
        Signed320 apexZ = GetConeEndpointNumerator(center.Z, axis.Z, height, 1);
        Signed320 baseX = GetConeEndpointNumerator(center.X, axis.X, height, -1);
        Signed320 baseY = GetConeEndpointNumerator(center.Y, axis.Y, height, -1);
        Signed320 baseZ = GetConeEndpointNumerator(center.Z, axis.Z, height, -1);
        Signed192 radialScale = Double(Narrow192(Multiply(
            WideArithmetic.ExtendToSigned576(WideArithmetic.ExtendToSigned320(Raw(radius))), ScaleSquared)));
        Signed576 rimX = WideArithmetic.AddSigned576(Multiply(baseX, reducedLength), WideArithmetic.ExtendToSigned576(Multiply(reducedX, radialScale)));
        Signed576 rimY = WideArithmetic.AddSigned576(Multiply(baseY, reducedLength), WideArithmetic.ExtendToSigned576(Multiply(reducedY, radialScale)));
        Signed576 rimZ = WideArithmetic.AddSigned576(Multiply(baseZ, reducedLength), WideArithmetic.ExtendToSigned576(Multiply(reducedZ, radialScale)));
        Signed576 apexScaleX = Multiply(apexX, reducedLength);
        Signed576 apexScaleY = Multiply(apexY, reducedLength);
        Signed576 apexScaleZ = Multiply(apexZ, reducedLength);
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
        Signed320 planeOffset = WideArithmetic.SubtractSigned320(Multiply(Raw(plane), doubleScale), apexY);
        Signed576 scaledPlaneOffset = Multiply(WideArithmetic.ExtendToSigned576(planeOffset), radialLength);
        if (!IsUnitInterval(scaledPlaneOffset, uy))
            return;

        Signed192 planeOffsetNarrow = Narrow(planeOffset);
        Signed576 candidateX = WideArithmetic.AddSigned576(Multiply(uy, Narrow(apexX)), Multiply(ux, planeOffsetNarrow));
        Signed576 candidateZ = WideArithmetic.AddSigned576(Multiply(uy, Narrow(apexZ)), Multiply(uz, planeOffsetNarrow));
        Signed576 candidateDenominator = Multiply(uy, doubleScale);
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
        Signed192 maximum = Signed(long.MaxValue);
        Signed192 parameterValue = Signed((long)parameter);
        Signed192 doubleScale = Double(Scale);
        Signed320 centerDenominator = Multiply(doubleScale, maximum);
        Signed192 axialWeight = WideArithmetic.SubtractSigned192(
            maximum,
            WideArithmetic.AddSigned192(parameterValue, parameterValue));
        Signed320 centerX = GetConeDiskCenterNumerator(center.X, axis.X, height, maximum, axialWeight);
        Signed320 centerY = GetConeDiskCenterNumerator(center.Y, axis.Y, height, maximum, axialWeight);
        Signed320 centerZ = GetConeDiskCenterNumerator(center.Z, axis.Z, height, maximum, axialWeight);
        Signed320 radiusNumerator = Multiply(Raw(radius), parameterValue);
        Signed192 q = GetAxisLengthSquared(axis);
        Signed192 planarAxisSquared = WideArithmetic.SubtractSigned192(
            q,
            Narrow(WideArithmetic.MultiplySigned192(Raw(axis.Y), Raw(axis.Y))));
        Signed320 k = Narrow(WideArithmetic.SubtractSigned576(
            Multiply(WideArithmetic.ExtendToSigned576(centerDenominator), Raw(plane)),
            WideArithmetic.ExtendToSigned576(centerY)));

        Signed192 radiusNarrow = Narrow(radiusNumerator);
        Signed192 centerDenominatorNarrow = Narrow(centerDenominator);
        Signed192 kNarrow = Narrow(k);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(radiusNarrow, radiusNarrow);
        Signed320 centerDenominatorSquared = WideArithmetic.MultiplySigned192(
            centerDenominatorNarrow,
            centerDenominatorNarrow);
        Signed576 first = Multiply(
            WideArithmetic.MultiplySigned320(radiusSquared, centerDenominatorSquared),
            planarAxisSquared);
        Signed320 kSquared = WideArithmetic.MultiplySigned192(kNarrow, kNarrow);
        Signed320 qParameterSquared = Narrow(Multiply(
            WideArithmetic.ExtendToSigned576(WideArithmetic.MultiplySigned192(maximum, maximum)),
            q));
        Signed576 second = WideArithmetic.MultiplySigned320(kSquared, qParameterSquared);
        Signed576 radicand = WideArithmetic.SubtractSigned576(first, second);
        if (radicand.Sign < 0)
        {
            candidate = default;
            return false;
        }

        Signed320 root = WideArithmetic.GetFloorSquareRootScaledByFixed64(radicand);
        Signed320 denominator320 = Narrow(Multiply(
            Multiply(
                Multiply(WideArithmetic.ExtendToSigned576(centerDenominator), maximum),
                planarAxisSquared),
            Scale));
        Signed576 denominatorWide = WideArithmetic.ExtendToSigned576(denominator320);
        Signed576 common = Multiply(Multiply(
            WideArithmetic.ExtendToSigned576(WideArithmetic.ExtendToSigned320(maximum)),
            planarAxisSquared), Scale);
        Signed576 x = Multiply(WideArithmetic.ExtendToSigned576(centerX), Narrow192(common));
        Signed576 z = Multiply(WideArithmetic.ExtendToSigned576(centerZ), Narrow192(common));
        Signed576 particularScale = Multiply(Multiply(
            WideArithmetic.ExtendToSigned576(WideArithmetic.ExtendToSigned320(maximum)),
            Scale), Raw(axis.Y));
        x = WideArithmetic.SubtractSigned576(x, Multiply(Multiply(particularScale, Raw(axis.X)), kNarrow));
        z = WideArithmetic.SubtractSigned576(z, Multiply(Multiply(particularScale, Raw(axis.Z)), kNarrow));
        Signed576 tangentX = Multiply(WideArithmetic.ExtendToSigned576(root), Raw(-axis.Z));
        Signed576 tangentZ = Multiply(WideArithmetic.ExtendToSigned576(root), Raw(axis.X));
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
            Narrow(Multiply(Multiply(
                WideArithmetic.ExtendToSigned576(WideArithmetic.ExtendToSigned320(Raw(center))),
                Double(Scale)), maximum)),
            Narrow(Multiply(WideArithmetic.MultiplySigned192(Raw(axis), Raw(height)), axialWeight)));

}
