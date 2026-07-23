//=======================================================================
// WideSlabProjection.FiniteAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

internal static partial class WideSlabProjection
{

    private static void AddSphereEndpoint(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
        Fixed64 radius,
        int sign,
        FixedRange slab,
        Vector2d direction,
        Signed192 directionLength,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed320 y = GetEndpointNumerator(center.Y, axis.Y, halfLength, sign);
        if (!IsInRange(y, Scale, slab))
            return;

        KeepBest(CreateEndpointRadialCandidate(
            center, axis, halfLength, radius, sign, direction, directionLength), direction, ref found, ref best);
    }

    private static void AddSpherePlaneCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
        Fixed64 radius,
        int sign,
        Fixed64 plane,
        Vector2d direction,
        Signed192 directionLength,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed320 k = WideArithmetic.SubtractSigned320(
            Multiply(WideArithmetic.SubtractSigned192(Raw(plane), Raw(center.Y)), Scale),
            Multiply(Raw(axis.Y), Signed(sign * halfLength.m_rawValue)));
        Signed576 squaredRadius = Multiply(
            WideArithmetic.ExtendToSigned576(WideArithmetic.MultiplySigned192(Raw(radius), Raw(radius))),
            ScaleSquared);
        Signed576 radicand = WideArithmetic.SubtractSigned576(
            squaredRadius,
            WideArithmetic.MultiplySigned320(k, k));
        if (radicand.Sign < 0)
            return;

        Signed320 root = WideArithmetic.GetFloorSquareRootScaledByFixed64(radicand);
        Signed576 denominator = WideArithmetic.ExtendToSigned576(Multiply(Scale, directionLength));
        Signed576 x = WideArithmetic.AddSigned576(
            Multiply(GetEndpointNumerator(center.X, axis.X, halfLength, sign), directionLength),
            Multiply(WideArithmetic.ExtendToSigned576(root), Raw(direction.X)));
        Signed576 z = WideArithmetic.AddSigned576(
            Multiply(GetEndpointNumerator(center.Z, axis.Z, halfLength, sign), directionLength),
            Multiply(WideArithmetic.ExtendToSigned576(root), Raw(direction.Y)));
        KeepBest(new WidePlanarCandidate(x, z, denominator), direction, ref found, ref best);
    }

    private static WidePlanarCandidate CreateEndpointRadialCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
        Fixed64 radius,
        int sign,
        Vector2d direction,
        Signed192 directionLength)
    {
        Signed576 denominator = WideArithmetic.ExtendToSigned576(Multiply(Scale, directionLength));
        Signed576 radialScale = Multiply(
            WideArithmetic.ExtendToSigned576(WideArithmetic.MultiplySigned192(Raw(radius), ScaleSquared)),
            Raw(direction.X));
        Signed576 x = WideArithmetic.AddSigned576(
            Multiply(GetEndpointNumerator(center.X, axis.X, halfLength, sign), directionLength),
            radialScale);
        Signed576 z = WideArithmetic.AddSigned576(
            Multiply(GetEndpointNumerator(center.Z, axis.Z, halfLength, sign), directionLength),
            Multiply(
                WideArithmetic.ExtendToSigned576(WideArithmetic.MultiplySigned192(Raw(radius), ScaleSquared)),
                Raw(direction.Y)));
        return new WidePlanarCandidate(x, z, denominator);
    }

    private static void AddCapsuleSideCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
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
        Signed192 absY = Raw(axis.Y.Abs());
        Signed320 wx = WideArithmetic.MultiplySigned192(absY, Raw(direction.X));
        Signed320 wy = WideArithmetic.ExtendToSigned320(sign < 0 ? m : WideArithmetic.SubtractSigned192(default, m));
        Signed320 wz = WideArithmetic.MultiplySigned192(absY, Raw(direction.Y));
        Signed576 squared = SumSquares(wx, wy, wz);
        Signed320 length = WideArithmetic.GetFloorSquareRootScaledByFixed64(squared);
        Signed576 radialX = Multiply(Multiply(WideArithmetic.ExtendToSigned576(wx), Raw(radius)), Scale);
        Signed576 radialY = Multiply(Multiply(WideArithmetic.ExtendToSigned576(wy), Raw(radius)), Scale);
        Signed576 radialZ = Multiply(Multiply(WideArithmetic.ExtendToSigned576(wz), Raw(radius)), Scale);
        Signed576 delta = WideArithmetic.SubtractSigned576(
            Multiply(WideArithmetic.ExtendToSigned576(length), WideArithmetic.SubtractSigned192(Raw(plane), Raw(center.Y))),
            radialY);
        Signed576 axisDenominator = Multiply(WideArithmetic.ExtendToSigned576(length), Raw(axis.Y));
        Signed576 parameterNumerator = Multiply(delta, Scale);
        Signed576 parameterLimit = Multiply(Absolute(axisDenominator), Raw(halfLength));
        if (CompareMagnitude(parameterNumerator, parameterLimit) > 0)
            return;

        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                Multiply(axisDenominator, Raw(center.X)),
                Multiply(delta, Raw(axis.X))),
            Multiply(radialX, Raw(axis.Y)));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                Multiply(axisDenominator, Raw(center.Z)),
                Multiply(delta, Raw(axis.Z))),
            Multiply(radialZ, Raw(axis.Y)));
        Normalize(ref x, ref z, ref axisDenominator);
        KeepBest(new WidePlanarCandidate(x, z, axisDenominator), direction, ref found, ref best);
    }

    private static void AddDiskEndpoint(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
        Fixed64 radius,
        int sign,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        Signed320 baseX = GetEndpointNumerator(center.X, axis.X, halfLength, sign);
        Signed320 baseY = GetEndpointNumerator(center.Y, axis.Y, halfLength, sign);
        Signed320 baseZ = GetEndpointNumerator(center.Z, axis.Z, halfLength, sign);
        AddDiskAtRationalCenter(baseX, baseY, baseZ, Scale, axis, radius, slab, direction, axisLengthSquared, ref found, ref best);
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
            WideArithmetic.MultiplySigned192(axisLengthSquared, Raw(direction.X)),
            WideArithmetic.MultiplySigned192(Raw(axis.X), m));
        Signed320 gy = WideArithmetic.SubtractSigned320(
            default,
            WideArithmetic.MultiplySigned192(Raw(axis.Y), m));
        Signed320 gz = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, Raw(direction.Y)),
            WideArithmetic.MultiplySigned192(Raw(axis.Z), m));
        Signed576 squared = SumSquares(gx, gy, gz);
        if (squared.IsZero)
        {
            if (IsInRange(baseY, baseDenominator, slab))
            {
                Signed576 denominator = WideArithmetic.ExtendToSigned576(WideArithmetic.ExtendToSigned320(baseDenominator));
                KeepBest(new WidePlanarCandidate(
                    WideArithmetic.ExtendToSigned576(baseX),
                    WideArithmetic.ExtendToSigned576(baseZ),
                    denominator), direction, ref found, ref best);
            }
            return;
        }

        Signed320 length = WideArithmetic.GetFloorSquareRootScaledByFixed64(squared);
        Signed576 denominatorWide = Multiply(WideArithmetic.ExtendToSigned576(length), baseDenominator);
        Signed320 radialFactor = Multiply(Raw(radius), baseDenominator);
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(baseX, length),
            Multiply(WideArithmetic.MultiplySigned320(radialFactor, gx), Scale));
        Signed576 y = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(baseY, length),
            Multiply(WideArithmetic.MultiplySigned320(radialFactor, gy), Scale));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(baseZ, length),
            Multiply(WideArithmetic.MultiplySigned320(radialFactor, gz), Scale));
        if (!IsInRange(y, denominatorWide, slab))
            return;

        KeepBest(new WidePlanarCandidate(x, z, denominatorWide), direction, ref found, ref best);
    }

    private static void AddDiskPlaneCandidates(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
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
            Narrow(WideArithmetic.MultiplySigned192(Raw(axis.Y), Raw(axis.Y))));
        Signed320 k = WideArithmetic.SubtractSigned320(
            Multiply(WideArithmetic.SubtractSigned192(Raw(plane), Raw(center.Y)), Scale),
            Multiply(Raw(axis.Y), Signed(sign * halfLength.m_rawValue)));
        if (planarAxisSquared.IsZero)
        {
            if (!k.IsZero)
                return;
            KeepBest(CreateEndpointRadialCandidate(
                center, axis, halfLength, radius, sign, direction, GetPlanarDirectionLength(direction)),
                direction, ref found, ref best);
            return;
        }

        Signed576 first = Multiply(
            Multiply(
                WideArithmetic.ExtendToSigned576(WideArithmetic.MultiplySigned192(Raw(radius), Raw(radius))),
                ScaleSquared),
            planarAxisSquared);
        Signed576 second = Multiply(WideArithmetic.MultiplySigned320(k, k), axisLengthSquared);
        Signed576 radicand = WideArithmetic.SubtractSigned576(first, second);
        if (radicand.Sign < 0)
            return;

        Signed320 root = WideArithmetic.GetFloorSquareRootScaledByFixed64(radicand);
        Signed320 denominator320 = WideArithmetic.MultiplySigned192(ScaleSquared, planarAxisSquared);
        Signed576 denominator = WideArithmetic.ExtendToSigned576(denominator320);
        AddDiskPlaneCandidate(center, axis, halfLength, sign, k, root, planarAxisSquared, -axis.Z, axis.X, -1, denominator, direction, ref found, ref best);
        AddDiskPlaneCandidate(center, axis, halfLength, sign, k, root, planarAxisSquared, -axis.Z, axis.X, 1, denominator, direction, ref found, ref best);
    }

    private static void AddDiskPlaneCandidate(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
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
        Signed576 x = Multiply(Multiply(GetEndpointNumerator(center.X, axis.X, halfLength, endpointSign), Scale), planarAxisSquared);
        Signed576 z = Multiply(Multiply(GetEndpointNumerator(center.Z, axis.Z, halfLength, endpointSign), Scale), planarAxisSquared);
        Signed576 xParticular = Multiply(Multiply(Multiply(WideArithmetic.ExtendToSigned576(k), Raw(axis.Y)), Raw(axis.X)), Scale);
        Signed576 zParticular = Multiply(Multiply(Multiply(WideArithmetic.ExtendToSigned576(k), Raw(axis.Y)), Raw(axis.Z)), Scale);
        x = WideArithmetic.SubtractSigned576(x, xParticular);
        z = WideArithmetic.SubtractSigned576(z, zParticular);
        Signed576 xTangent = Multiply(WideArithmetic.ExtendToSigned576(root), Raw(tangentX));
        Signed576 zTangent = Multiply(WideArithmetic.ExtendToSigned576(root), Raw(tangentZ));
        x = tangentSign < 0 ? WideArithmetic.SubtractSigned576(x, xTangent) : WideArithmetic.AddSigned576(x, xTangent);
        z = tangentSign < 0 ? WideArithmetic.SubtractSigned576(z, zTangent) : WideArithmetic.AddSigned576(z, zTangent);
        KeepBest(new WidePlanarCandidate(x, z, denominator), direction, ref found, ref best);
    }

}

