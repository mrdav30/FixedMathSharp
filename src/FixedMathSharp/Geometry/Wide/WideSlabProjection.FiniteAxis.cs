//=======================================================================
// WideSlabProjection.FiniteAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Finite-axis projection helpers: computes sphere endpoint and plane-intersection
/// candidates against a bounded axis segment for wide (high-precision) slab tests.
/// </content>
internal static partial class WideSlabProjection
{
    private static void AddSphereEndpoint(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        int sign,
        FixedRange slab,
        Vector2d direction,
        Signed192 directionLength,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed320 y = GetEndpointNumerator(center.Y, axis.Y, axisLength, sign);
        if (!IsInRange(y, CenteredAxisScale, slab))
            return;

        KeepBest(CreateEndpointRadialCandidate(
            center, axis, axisLength, radius, sign, direction, directionLength), direction, ref found, ref best);
    }

    private static void AddSpherePlaneCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        int sign,
        Fixed64 plane,
        Vector2d direction,
        Signed192 directionLength,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed320 k = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(WideArithmetic.SubtractSigned192(Signed192.Raw(plane), Signed192.Raw(center.Y)), CenteredAxisScale),
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Y), Signed192.Signed(sign * axisLength.m_rawValue)));
        Signed576 squaredRadius = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(radius), Signed192.Raw(radius))),
            CenteredAxisScaleSquared);
        Signed576 radicand = WideArithmetic.SubtractSigned576(
            squaredRadius,
            WideArithmetic.MultiplySigned320(k, k));
        if (radicand.Sign < 0)
            return;

        Signed320 root = WideArithmetic.GetFloorSquareRootScaledByFixed64(radicand);
        Signed576 denominator = Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(CenteredAxisScale, directionLength));
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(GetEndpointNumerator(center.X, axis.X, axisLength, sign), directionLength),
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(root), Signed192.Raw(direction.X)));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(GetEndpointNumerator(center.Z, axis.Z, axisLength, sign), directionLength),
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(root), Signed192.Raw(direction.Y)));
        KeepBest(new WidePlanarCandidate(x, z, denominator), direction, ref found, ref best);
    }

    private static WidePlanarCandidate CreateEndpointRadialCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        int sign,
        Vector2d direction,
        Signed192 directionLength)
    {
        Signed576 denominator = Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(CenteredAxisScale, directionLength));
        Signed576 radialScale = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(WideArithmetic.MultiplySigned192(
                Signed192.Raw(radius),
                CenteredAxisScaleTimesScale)),
            Signed192.Raw(direction.X));
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(GetEndpointNumerator(center.X, axis.X, axisLength, sign), directionLength),
            radialScale);
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(GetEndpointNumerator(center.Z, axis.Z, axisLength, sign), directionLength),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(WideArithmetic.MultiplySigned192(
                    Signed192.Raw(radius),
                    CenteredAxisScaleTimesScale)),
                Signed192.Raw(direction.Y)));
        return new WidePlanarCandidate(x, z, denominator);
    }

    private static void AddCapsuleSideCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 plane,
        Vector2d direction,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        if (axis.Y == Fixed64.Zero)
            return;

        Signed192 m = GetPlanarDot(axis, direction);
        int sign = axis.Y.m_rawValue < 0L ? -1 : 1;
        Signed192 absY = Signed192.Raw(axis.Y.Abs());
        Signed320 wx = WideArithmetic.MultiplySigned192(absY, Signed192.Raw(direction.X));
        Signed320 wy = Signed320.ExtendValue(sign < 0 ? m : WideArithmetic.SubtractSigned192(default, m));
        Signed320 wz = WideArithmetic.MultiplySigned192(absY, Signed192.Raw(direction.Y));
        Signed576 squared = SumSquares(wx, wy, wz);
        Signed320 length = WideArithmetic.GetFloorSquareRootScaledByFixed64(squared);
        Signed576 radialX = WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(Signed576.ExtendValue(wx), Signed192.Raw(radius)), Scale);
        Signed576 radialY = WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(Signed576.ExtendValue(wy), Signed192.Raw(radius)), Scale);
        Signed576 radialZ = WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned576(Signed576.ExtendValue(wz), Signed192.Raw(radius)), Scale);
        Signed576 delta = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(length), WideArithmetic.SubtractSigned192(Signed192.Raw(plane), Signed192.Raw(center.Y))),
            radialY);
        Signed576 axisDenominator = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(length), Signed192.Raw(axis.Y));
        Signed576 parameterNumerator = WideArithmetic.MultiplySigned576(delta, CenteredAxisScale);
        Signed576 parameterLimit = WideArithmetic.MultiplySigned576(WideArithmetic.Absolute(axisDenominator), Signed192.Raw(axisLength));
        if (CompareMagnitude(parameterNumerator, parameterLimit) > 0)
            return;

        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(axisDenominator, Signed192.Raw(center.X)),
                WideArithmetic.MultiplySigned576(delta, Signed192.Raw(axis.X))),
            WideArithmetic.MultiplySigned576(radialX, Signed192.Raw(axis.Y)));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(axisDenominator, Signed192.Raw(center.Z)),
                WideArithmetic.MultiplySigned576(delta, Signed192.Raw(axis.Z))),
            WideArithmetic.MultiplySigned576(radialZ, Signed192.Raw(axis.Y)));
        Normalize(ref x, ref z, ref axisDenominator);
        KeepBest(new WidePlanarCandidate(x, z, axisDenominator), direction, ref found, ref best);
    }

    private static void AddDiskEndpoint(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        int sign,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed320 baseX = GetEndpointNumerator(center.X, axis.X, axisLength, sign);
        Signed320 baseY = GetEndpointNumerator(center.Y, axis.Y, axisLength, sign);
        Signed320 baseZ = GetEndpointNumerator(center.Z, axis.Z, axisLength, sign);
        AddDiskAtRationalCenter(
            baseX,
            baseY,
            baseZ,
            CenteredAxisScale,
            axis,
            radius,
            slab,
            direction,
            axisLengthSquared,
            ref found,
            ref best);
    }

    private static void AddDiskAtRationalCenter(
        Signed320 baseX,
        Signed320 baseY,
        Signed320 baseZ,
        Signed192 baseDenominator,
        Vector3d axis,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed192 m = GetPlanarDot(axis, direction);
        Signed320 gx = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, Signed192.Raw(direction.X)),
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.X), m));
        Signed320 gy = WideArithmetic.SubtractSigned320(
            default,
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Y), m));
        Signed320 gz = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, Signed192.Raw(direction.Y)),
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Z), m));
        Signed576 squared = SumSquares(gx, gy, gz);
        if (squared.IsZero)
        {
            if (IsInRange(baseY, baseDenominator, slab))
            {
                Signed576 denominator = Signed576.ExtendValue(Signed320.ExtendValue(baseDenominator));
                KeepBest(new WidePlanarCandidate(
                    Signed576.ExtendValue(baseX),
                    Signed576.ExtendValue(baseZ),
                    denominator), direction, ref found, ref best);
            }
            return;
        }

        Signed320 length = WideArithmetic.GetFloorSquareRootScaledByFixed64(squared);
        Signed576 denominatorWide = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(length), baseDenominator);
        Signed320 radialFactor = WideArithmetic.MultiplySigned192(Signed192.Raw(radius), baseDenominator);
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(baseX, length),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(radialFactor, gx), Scale));
        Signed576 y = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(baseY, length),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(radialFactor, gy), Scale));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(baseZ, length),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(radialFactor, gz), Scale));
        if (!IsInRange(y, denominatorWide, slab))
            return;

        KeepBest(new WidePlanarCandidate(x, z, denominatorWide), direction, ref found, ref best);
    }

    private static void AddDiskPlaneCandidates(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        int sign,
        Fixed64 plane,
        Vector2d direction,
        Signed192 axisLengthSquared,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed192 planarAxisSquared = WideArithmetic.SubtractSigned192(
            axisLengthSquared,
            Signed192.NarrowValue(WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Y), Signed192.Raw(axis.Y))));
        Signed320 k = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(WideArithmetic.SubtractSigned192(Signed192.Raw(plane), Signed192.Raw(center.Y)), CenteredAxisScale),
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Y), Signed192.Signed(sign * axisLength.m_rawValue)));
        if (planarAxisSquared.IsZero)
        {
            if (!k.IsZero)
                return;
            KeepBest(CreateEndpointRadialCandidate(
                center, axis, axisLength, radius, sign, direction, GetPlanarDirectionLength(direction)),
                direction, ref found, ref best);
            return;
        }

        Signed576 first = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(radius), Signed192.Raw(radius))),
                CenteredAxisScaleSquared),
            planarAxisSquared);
        Signed576 second = WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(k, k), axisLengthSquared);
        Signed576 radicand = WideArithmetic.SubtractSigned576(first, second);
        if (radicand.Sign < 0)
            return;

        Signed320 root = WideArithmetic.GetFloorSquareRootScaledByFixed64(radicand);
        Signed320 denominator320 = WideArithmetic.MultiplySigned192(
            CenteredAxisScaleSquared,
            planarAxisSquared);
        Signed576 denominator = Signed576.ExtendValue(denominator320);
        AddDiskPlaneCandidate(center, axis, axisLength, sign, k, root, planarAxisSquared, -axis.Z, axis.X, -1, denominator, direction, ref found, ref best);
        AddDiskPlaneCandidate(center, axis, axisLength, sign, k, root, planarAxisSquared, -axis.Z, axis.X, 1, denominator, direction, ref found, ref best);
    }

    private static void AddDiskPlaneCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        int endpointSign,
        Signed320 k,
        Signed320 root,
        Signed192 planarAxisSquared,
        Fixed64 tangentX,
        Fixed64 tangentZ,
        int tangentSign,
        Signed576 denominator,
        Vector2d direction,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed576 x = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(
                GetEndpointNumerator(center.X, axis.X, axisLength, endpointSign),
                CenteredAxisScale),
            planarAxisSquared);
        Signed576 z = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(
                GetEndpointNumerator(center.Z, axis.Z, axisLength, endpointSign),
                CenteredAxisScale),
            planarAxisSquared);
        Signed576 xParticular = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(Signed576.ExtendValue(k), Signed192.Raw(axis.Y)),
                Signed192.Raw(axis.X)),
            CenteredAxisScale);
        Signed576 zParticular = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(Signed576.ExtendValue(k), Signed192.Raw(axis.Y)),
                Signed192.Raw(axis.Z)),
            CenteredAxisScale);
        x = WideArithmetic.SubtractSigned576(x, xParticular);
        z = WideArithmetic.SubtractSigned576(z, zParticular);
        Signed576 xTangent = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(root), Signed192.Raw(tangentX));
        Signed576 zTangent = WideArithmetic.MultiplySigned576(Signed576.ExtendValue(root), Signed192.Raw(tangentZ));
        // The disk denominator contains the squared centered-axis scale; the
        // square root contributes only one factor.
        xTangent = WideArithmetic.Double(xTangent);
        zTangent = WideArithmetic.Double(zTangent);
        x = tangentSign < 0 ? WideArithmetic.SubtractSigned576(x, xTangent) : WideArithmetic.AddSigned576(x, xTangent);
        z = tangentSign < 0 ? WideArithmetic.SubtractSigned576(z, zTangent) : WideArithmetic.AddSigned576(z, zTangent);
        KeepBest(new WidePlanarCandidate(x, z, denominator), direction, ref found, ref best);
    }
}
