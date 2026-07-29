//=======================================================================
// WideSlabProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns full-domain candidate construction for finite-slab projections.
/// </summary>
internal static class WideSlabProjection
{
    private static readonly Signed192 Scale = Signed192.Signed(Fixed64.One.m_rawValue);
    private static readonly Signed192 CenteredAxisScale = WideArithmetic.Double(Scale);
    private static readonly Signed192 ScaleSquared = new(0UL, 1UL, 0UL);
    private static readonly Signed192 CenteredAxisScaleTimesScale = WideArithmetic.Double(ScaleSquared);
    private static readonly Signed192 CenteredAxisScaleSquared = WideArithmetic.Double(CenteredAxisScaleTimesScale);

    #region Nested Types

    private readonly struct WidePlanarCandidate
    {
        internal readonly Signed576 X;
        internal readonly Signed576 Z;
        internal readonly Signed576 Denominator;

        internal WidePlanarCandidate(Signed576 x, Signed576 z, Signed576 denominator)
        {
            X = x;
            Z = z;
            Denominator = denominator;
        }
    }

    #endregion

    private static bool TryCreateResult(bool found, WidePlanarCandidate candidate, out Vector2d support)
    {
        if (!found
            || !Fixed64.TryGetSignedRawRatio(candidate.X, candidate.Denominator, out Fixed64 x)
            || !Fixed64.TryGetSignedRawRatio(candidate.Z, candidate.Denominator, out Fixed64 z))
        {
            support = default;
            return false;
        }

        support = new Vector2d(x, z);
        return true;
    }

    private static void KeepBest(
        WidePlanarCandidate candidate,
        Vector2d direction,
        ref bool found,
        ref WidePlanarCandidate best)
    {
        if (found)
        {
            int projection = CompareProjection(candidate, best, direction);
            if (projection < 0 || (projection == 0 && ComesAfter(candidate, best)))
                return;
        }

        found = true;
        best = candidate;
    }

    private static int CompareProjection(WidePlanarCandidate left, WidePlanarCandidate right, Vector2d direction)
    {
        Signed576 leftProjection = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(left.X, Signed192.Raw(direction.X)),
            WideArithmetic.MultiplySigned576(left.Z, Signed192.Raw(direction.Y)));
        Signed576 rightProjection = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(right.X, Signed192.Raw(direction.X)),
            WideArithmetic.MultiplySigned576(right.Z, Signed192.Raw(direction.Y)));
        return WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(leftProjection, right.Denominator),
            WideArithmetic.MultiplySigned576ToSigned832(rightProjection, left.Denominator)).Sign;
    }

    private static bool ComesAfter(WidePlanarCandidate left, WidePlanarCandidate right)
    {
        int x = CompareRatio(left.X, left.Denominator, right.X, right.Denominator);
        return x > 0 || (x == 0 && CompareRatio(left.Z, left.Denominator, right.Z, right.Denominator) > 0);
    }

    private static int CompareRatio(Signed576 left, Signed576 leftDenominator, Signed576 right, Signed576 rightDenominator) =>
        WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(left, rightDenominator),
            WideArithmetic.MultiplySigned576ToSigned832(right, leftDenominator)).Sign;

    private static bool IsInRange(Signed320 numerator, Signed192 denominator, FixedRange range) =>
        Compare(numerator, WideArithmetic.MultiplySigned192(Signed192.Raw(range.Min), denominator)) >= 0
        && Compare(numerator, WideArithmetic.MultiplySigned192(Signed192.Raw(range.Max), denominator)) <= 0;

    private static bool IsInRange(Signed576 numerator, Signed576 denominator, FixedRange range) =>
        WideArithmetic.SubtractSigned576(numerator, WideArithmetic.MultiplySigned576(denominator, Signed192.Raw(range.Min))).Sign >= 0
        && WideArithmetic.SubtractSigned576(numerator, WideArithmetic.MultiplySigned576(denominator, Signed192.Raw(range.Max))).Sign <= 0;

    private static Signed192 GetPlanarDirectionLength(Vector2d direction)
    {
        Signed192 squared = WideGeometry.GetDifferenceDotProduct2D(
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero,
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero);
        return WideArithmetic.GetFloorSquareRoot(
            WideArithmetic.MultiplySigned192(squared, ScaleSquared), out _);
    }

    private static Signed192 GetPlanarDirectionLengthSquared(Vector2d direction) =>
        WideGeometry.GetDifferenceDotProduct2D(
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero,
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero);

    private static Signed576 SumProducts(Vector3d axis, Signed576 x, Signed576 y, Signed576 z) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(x, Signed192.Raw(axis.X)),
                WideArithmetic.MultiplySigned576(y, Signed192.Raw(axis.Y))),
            WideArithmetic.MultiplySigned576(z, Signed192.Raw(axis.Z)));

    private static bool IsUnitInterval(Signed576 numerator, Signed576 denominator)
    {
        int denominatorSign = denominator.Sign;
        int numeratorSign = numerator.Sign;
        int remainderSign = WideArithmetic.SubtractSigned576(numerator, denominator).Sign;
        return ((denominatorSign > 0) & (numeratorSign >= 0) & (remainderSign <= 0))
            | ((denominatorSign < 0) & (numeratorSign <= 0) & (remainderSign >= 0));
    }

    private static void ReduceDirection(
        Signed576 x,
        Signed576 y,
        Signed576 z,
        Signed576 length,
        out Signed192 reducedX,
        out Signed192 reducedY,
        out Signed192 reducedZ,
        out Signed192 reducedLength)
    {
        // The extra zero word makes cross-word shifts branch-free at the
        // upper edge of a 576-bit magnitude.
        Span<ulong> xMagnitude = stackalloc ulong[10];
        Span<ulong> yMagnitude = stackalloc ulong[10];
        Span<ulong> zMagnitude = stackalloc ulong[10];
        Span<ulong> lengthMagnitude = stackalloc ulong[10];
        xMagnitude.Clear();
        yMagnitude.Clear();
        zMagnitude.Clear();
        lengthMagnitude.Clear();
        WideArithmetic.GetMagnitude(x, xMagnitude[..9]);
        WideArithmetic.GetMagnitude(y, yMagnitude[..9]);
        WideArithmetic.GetMagnitude(z, zMagnitude[..9]);
        WideArithmetic.GetMagnitude(length, lengthMagnitude[..9]);
        int shift = Math.Max(0,
            Math.Max(
                Math.Max(WideArithmetic.GetMagnitudeBitLength(xMagnitude), WideArithmetic.GetMagnitudeBitLength(yMagnitude)),
                Math.Max(WideArithmetic.GetMagnitudeBitLength(zMagnitude), WideArithmetic.GetMagnitudeBitLength(lengthMagnitude))) - 190);
        reducedX = CreateReduced(xMagnitude, shift, x.Sign < 0);
        reducedY = CreateReduced(yMagnitude, shift, y.Sign < 0);
        reducedZ = CreateReduced(zMagnitude, shift, z.Sign < 0);
        reducedLength = CreateReduced(lengthMagnitude, shift, false);
    }

    private static Signed192 CreateReduced(ReadOnlySpan<ulong> magnitude, int shift, bool negative)
    {
        int wordShift = shift >> 6;
        int bitShift = shift & 63;
        ulong low = GetShiftedWord(magnitude, wordShift, bitShift);
        ulong middle = GetShiftedWord(magnitude, wordShift + 1, bitShift);
        ulong high = GetShiftedWord(magnitude, wordShift + 2, bitShift);
        Signed192 value = new(high, middle, low);
        return negative ? WideArithmetic.SubtractSigned192(default, value) : value;
    }

    private static ulong GetShiftedWord(ReadOnlySpan<ulong> magnitude, int wordIndex, int bitShift)
    {
        ulong value = magnitude[wordIndex] >> bitShift;
        int complementaryShift = (64 - bitShift) & 63;
        ulong nonZeroShiftMask = 0UL - ((ulong)(bitShift + 63) >> 6);
        return value | ((magnitude[wordIndex + 1] << complementaryShift) & nonZeroShiftMask);
    }

    private static Signed192 GetAxisLengthSquared(Vector3d axis) =>
        WideGeometry.GetDifferenceDotProduct3D(
            axis.X, Fixed64.Zero, axis.Y, Fixed64.Zero, axis.Z, Fixed64.Zero,
            axis.X, Fixed64.Zero, axis.Y, Fixed64.Zero, axis.Z, Fixed64.Zero);

    private static Signed192 GetPlanarDot(Vector3d axis, Vector2d direction) =>
        WideGeometry.GetDifferenceDotProduct2D(
            axis.X, Fixed64.Zero, axis.Z, Fixed64.Zero,
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero);

    private static Signed576 SumSquares(Signed320 x, Signed320 y, Signed320 z) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(x, x),
                WideArithmetic.MultiplySigned320(y, y)),
            WideArithmetic.MultiplySigned320(z, z));

    private static Signed320 GetEndpointNumerator(Fixed64 center, Fixed64 axis, Fixed64 axisLength, int sign) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(Signed192.Raw(center), CenteredAxisScale),
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis), Signed192.Signed(sign * axisLength.m_rawValue)));

    private static Signed320 GetConeEndpointNumerator(Fixed64 center, Fixed64 axis, Fixed64 height, int sign) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(Signed192.Raw(center), WideArithmetic.Double(Scale)),
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis), Signed192.Signed(sign * height.m_rawValue)));

    private static int Compare(Signed320 left, Signed320 right) =>
        WideArithmetic.SubtractSigned320(left, right).Sign;

    private static Signed320 Minimum(Signed320 left, Signed320 right) => Compare(left, right) <= 0 ? left : right;

    private static Signed320 Maximum(Signed320 left, Signed320 right) => Compare(left, right) >= 0 ? left : right;

    private static int CompareMagnitude(Signed576 left, Signed576 right) =>
        WideArithmetic.CompareNonNegative(WideArithmetic.Absolute(left), WideArithmetic.Absolute(right));

    private static void Normalize(ref Signed576 x, ref Signed576 z, ref Signed576 denominator)
    {
        if (denominator.Sign >= 0)
            return;
        x = WideArithmetic.SubtractSigned576(default, x);
        z = WideArithmetic.SubtractSigned576(default, z);
        denominator = WideArithmetic.SubtractSigned576(default, denominator);
    }

internal static bool TryGetCapsuleSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        out Vector2d support)
    {
        Signed192 directionLength = GetPlanarDirectionLength(direction);
        if (TryAdmitCapsuleSupport(center, axis, axisLength, radius, slab, direction, directionLength, out support))
            return true;

        bool found = false;
        WidePlanarCandidate best = default;
        AddSphereEndpoint(center, axis, axisLength, radius, -1, slab, direction, directionLength, ref found, ref best);
        AddSphereEndpoint(center, axis, axisLength, radius, 1, slab, direction, directionLength, ref found, ref best);
        AddSpherePlaneCandidate(center, axis, axisLength, radius, -1, slab.Min, direction, directionLength, ref found, ref best);
        AddSpherePlaneCandidate(center, axis, axisLength, radius, 1, slab.Min, direction, directionLength, ref found, ref best);
        AddCapsuleSideCandidate(center, axis, axisLength, radius, slab.Min, direction, ref found, ref best);
        if (slab.Max != slab.Min)
        {
            AddSpherePlaneCandidate(center, axis, axisLength, radius, -1, slab.Max, direction, directionLength, ref found, ref best);
            AddSpherePlaneCandidate(center, axis, axisLength, radius, 1, slab.Max, direction, directionLength, ref found, ref best);
            AddCapsuleSideCandidate(center, axis, axisLength, radius, slab.Max, direction, ref found, ref best);
        }

        return TryCreateResult(found, best, out support);
    }

    internal static bool TryGetCylinderSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        out Vector2d support)
    {
        Signed192 axisLengthSquared = GetAxisLengthSquared(axis);
        if (TryAdmitCylinderSupport(center, axis, axisLength, radius, slab, direction, axisLengthSquared, out support))
            return true;

        bool found = false;
        WidePlanarCandidate best = default;
        AddDiskEndpoint(center, axis, axisLength, radius, -1, slab, direction, axisLengthSquared, ref found, ref best);
        AddDiskEndpoint(center, axis, axisLength, radius, 1, slab, direction, axisLengthSquared, ref found, ref best);
        AddDiskPlaneCandidates(center, axis, axisLength, radius, -1, slab.Min, direction, axisLengthSquared, ref found, ref best);
        AddDiskPlaneCandidates(center, axis, axisLength, radius, 1, slab.Min, direction, axisLengthSquared, ref found, ref best);
        AddCapsuleSideCandidate(center, axis, axisLength, radius, slab.Min, direction, ref found, ref best);
        if (slab.Max != slab.Min)
        {
            AddDiskPlaneCandidates(center, axis, axisLength, radius, -1, slab.Max, direction, axisLengthSquared, ref found, ref best);
            AddDiskPlaneCandidates(center, axis, axisLength, radius, 1, slab.Max, direction, axisLengthSquared, ref found, ref best);
            AddCapsuleSideCandidate(center, axis, axisLength, radius, slab.Max, direction, ref found, ref best);
        }

        return TryCreateResult(found, best, out support);
    }

    internal static bool TryGetConeSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        out Vector2d support)
    {
        // A vertical cone slice is a disk whose radius changes linearly with Y.
        // The general rotated-cone branch is added below after the shared cap
        // candidates so the same exact endpoint ownership is retained.
        if (axis.X == Fixed64.Zero && axis.Z == Fixed64.Zero)
            return TryGetVerticalConeSupport(center, axis, height, radius, slab, direction, out support);

        Signed192 axisLengthSquared = GetAxisLengthSquared(axis);
        if (TryAdmitConeSupport(center, axis, height, radius, slab, direction, axisLengthSquared, out support))
            return true;

        return TryGetRotatedConeSupport(center, axis, height, radius, slab, direction, axisLengthSquared, out support);
    }

    private static bool TryAdmitCapsuleSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 directionLength,
        out Vector2d support)
    {
        Signed192 dot = GetPlanarDot(axis, direction);
        bool found = false;
        WidePlanarCandidate best = default;
        if (dot.Sign <= 0)
            AddSphereEndpoint(center, axis, axisLength, radius, -1, slab, direction, directionLength, ref found, ref best);
        if (dot.Sign >= 0)
            AddSphereEndpoint(center, axis, axisLength, radius, 1, slab, direction, directionLength, ref found, ref best);
        return TryCreateResult(found, best, out support);
    }

    private static bool TryAdmitCylinderSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        out Vector2d support)
    {
        if (!HasUniqueDiskSupport(axis, direction, axisLengthSquared))
        {
            support = default;
            return false;
        }

        Signed192 dot = GetPlanarDot(axis, direction);
        bool found = false;
        WidePlanarCandidate best = default;
        if (dot.Sign <= 0)
            AddDiskEndpoint(center, axis, axisLength, radius, -1, slab, direction, axisLengthSquared, ref found, ref best);
        if (dot.Sign >= 0)
            AddDiskEndpoint(center, axis, axisLength, radius, 1, slab, direction, axisLengthSquared, ref found, ref best);
        return TryCreateResult(found, best, out support);
    }

    private static bool TryAdmitConeSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        out Vector2d support)
    {
        if (!HasUniqueDiskSupport(axis, direction, axisLengthSquared))
        {
            support = default;
            return false;
        }

        bool apexFound = false;
        WidePlanarCandidate apex = default;
        AddConeApex(center, axis, height, slab, direction, ref apexFound, ref apex);
        bool baseFound = false;
        WidePlanarCandidate baseCandidate = default;
        AddConeBaseDisk(center, axis, height, radius, slab, direction, axisLengthSquared, ref baseFound, ref baseCandidate);
        if (!apexFound || !baseFound)
        {
            support = default;
            return false;
        }

        bool found = true;
        KeepBest(baseCandidate, direction, ref found, ref apex);
        return TryCreateResult(true, apex, out support);
    }

    private static bool HasUniqueDiskSupport(Vector3d axis, Vector2d direction, Signed192 axisLengthSquared)
    {
        Signed192 dot = GetPlanarDot(axis, direction);
        Signed320 gx = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, Signed192.Raw(direction.X)),
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.X), dot));
        Signed320 gy = WideArithmetic.SubtractSigned320(
            default,
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Y), dot));
        Signed320 gz = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, Signed192.Raw(direction.Y)),
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Z), dot));
        return !SumSquares(gx, gy, gz).IsZero;
    }

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
        Signed832 radialSquared = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
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
