//=======================================================================
// WideGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Owns exact coordinate products and fixed-point geometry policy.
/// </summary>
internal static partial class WideGeometry
{
    /// <summary>
    /// Returns the exact representable size of a normalized scalar interval.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 GetIntervalSize(Fixed64 min, Fixed64 max)
    {
        ulong span = unchecked((ulong)max.m_rawValue - (ulong)min.m_rawValue);
        if (span > long.MaxValue)
            throw new System.OverflowException("The interval size is outside the representable Fixed64 range.");

        return Fixed64.FromRaw((long)span);
    }

    /// <summary>
    /// Returns the smallest representable half-extent that contains both ends
    /// of a normalized scalar interval around its lattice midpoint.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 GetIntervalScope(Fixed64 min, Fixed64 max)
    {
        ulong span = unchecked((ulong)max.m_rawValue - (ulong)min.m_rawValue);
        ulong scope = (span >> 1) + (span & 1UL);
        if (scope > long.MaxValue)
            throw new System.OverflowException("The interval scope is outside the representable Fixed64 range.");

        return Fixed64.FromRaw((long)scope);
    }

    /// <summary>
    /// Returns the representable absolute magnitude of a scalar extent.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 GetExtentMagnitude(Fixed64 extent)
    {
        if (extent == Fixed64.MinValue)
            throw new System.OverflowException("The extent magnitude is outside the representable Fixed64 range.");

        return extent.Abs();
    }

    /// <summary>
    /// Returns the smallest scalar half-extent whose symmetric interval covers
    /// the requested size magnitude, including the minimum scalar input.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 GetHalfSizeMagnitude(Fixed64 size)
    {
        ulong magnitude = size.m_rawValue < 0L
            ? unchecked(0UL - (ulong)size.m_rawValue)
            : (ulong)size.m_rawValue;
        ulong half = (magnitude >> 1) + (magnitude & 1UL);
        return Fixed64.FromRaw((long)half);
    }

    /// <summary>
    /// Returns an exact full-domain AABB union-volume growth metric.
    /// </summary>
    internal static long GetVolumeExpansionCost(
        Vector3d min,
        Vector3d max,
        Vector3d otherMin,
        Vector3d otherMax)
    {
        Signed192 volume = GetIntervalVolume(min, max);
        Vector3d unionMin = Vector3d.Min(min, otherMin);
        Vector3d unionMax = Vector3d.Max(max, otherMax);
        Signed192 unionVolume = GetIntervalVolume(unionMin, unionMax);
        Signed192 growth = WideArithmetic.SubtractSigned192(unionVolume, volume);

        // Q32.32 axis spans produce a Q96.96 product. Shifting by 96 floors
        // the exact volume to integer world units. Any remaining bit above the
        // signed 63-bit result range maps to the public metric's upper bound.
        if (growth.High > 0x0000_0000_7FFF_FFFFUL) // 2,147,483,647: highest 31-bit word that fits after the 96-bit shift.
            return long.MaxValue;

        ulong floor = (growth.High << 32) | (growth.Middle >> 32);
        return (long)floor;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetIntervalVolume(Vector3d min, Vector3d max)
    {
        ulong x = unchecked((ulong)max.X.m_rawValue - (ulong)min.X.m_rawValue);
        ulong y = unchecked((ulong)max.Y.m_rawValue - (ulong)min.Y.m_rawValue);
        ulong z = unchecked((ulong)max.Z.m_rawValue - (ulong)min.Z.m_rawValue);
        Fixed64.Multiply64To128(x, y, out ulong productHigh, out ulong productLow);
        Fixed64.Multiply64To128(productLow, z, out ulong lowHigh, out ulong low);
        Fixed64.Multiply64To128(productHigh, z, out ulong high, out ulong highLow);
        return WideArithmetic.AddSigned192(
            new Signed192(0UL, lowHigh, low),
            new Signed192(high, highLow, 0UL));
    }

    /// <summary>
    /// Compares an exact 2D distance with the exact sum of two non-negative radii.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CompareDistanceToRadiusSum(
        Vector2d first,
        Vector2d second,
        Fixed64 firstRadius,
        Fixed64 secondRadius)
    {
        Signed192 squaredDistance = GetDifferenceDotProduct2D(
            first.X, second.X, first.Y, second.Y,
            first.X, second.X, first.Y, second.Y);
        return WideArithmetic.CompareMagnitude(
            squaredDistance,
            GetSquaredRadiusSum(firstRadius, secondRadius));
    }

    /// <summary>
    /// Compares an exact 3D distance with the exact sum of two non-negative radii.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CompareDistanceToRadiusSum(
        Vector3d first,
        Vector3d second,
        Fixed64 firstRadius,
        Fixed64 secondRadius)
    {
        Signed192 squaredDistance = GetDifferenceDotProduct3D(
            first.X, second.X, first.Y, second.Y, first.Z, second.Z,
            first.X, second.X, first.Y, second.Y, first.Z, second.Z);
        return WideArithmetic.CompareMagnitude(
            squaredDistance,
            GetSquaredRadiusSum(firstRadius, secondRadius));
    }

    /// <summary>
    /// Returns whether an axis interval contains the centered extent of a non-negative radius.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ContainsCenteredExtent(
        Fixed64 min,
        Fixed64 max,
        Fixed64 center,
        Fixed64 radius)
    {
        if (center < min || center > max)
            return false;

        ulong radiusRaw = (ulong)radius.m_rawValue;
        ulong minimumClearance = unchecked((ulong)center.m_rawValue - (ulong)min.m_rawValue);
        ulong maximumClearance = unchecked((ulong)max.m_rawValue - (ulong)center.m_rawValue);
        return minimumClearance >= radiusRaw && maximumClearance >= radiusRaw;
    }

    /// <summary>
    /// Attempts to return the rounded distance between two 2D endpoints without
    /// narrowing their component differences.
    /// </summary>
    internal static bool TryGetDistance(Vector2d start, Vector2d end, out Fixed64 distance)
    {
        Signed192 squaredDistance = GetDifferenceDotProduct2D(
            end.X, start.X, end.Y, start.Y,
            end.X, start.X, end.Y, start.Y);
        return TryRoundDistance(squaredDistance, out distance);
    }

    /// <summary>
    /// Attempts to return the rounded distance between two 3D endpoints without
    /// narrowing their component differences.
    /// </summary>
    internal static bool TryGetDistance(Vector3d start, Vector3d end, out Fixed64 distance)
    {
        Signed192 squaredDistance = GetDifferenceDotProduct3D(
            end.X, start.X, end.Y, start.Y, end.Z, start.Z,
            end.X, start.X, end.Y, start.Y, end.Z, start.Z);
        return TryRoundDistance(squaredDistance, out distance);
    }

    /// <summary>
    /// Interpolates one coordinate from an exact nonnegative numerator and
    /// denominator with one final round-half-to-even conversion.
    /// </summary>
    internal static Fixed64 InterpolateCoordinate(
        Fixed64 start,
        Fixed64 end,
        Signed192 numerator,
        Signed192 denominator)
    {
        if (start == end)
            return start;

        Signed192 remaining = WideArithmetic.SubtractSigned192(denominator, numerator);
        Signed320 weighted = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                WideArithmetic.FromSignedRaw(start.m_rawValue),
                remaining),
            WideArithmetic.MultiplySigned192(
                WideArithmetic.FromSignedRaw(end.m_rawValue),
                numerator));
        return Fixed64.GetSignedRawRatio(weighted, denominator);
    }

    /// <summary>
    /// Returns the normalized direction between two 2D endpoints without
    /// narrowing their component differences.
    /// </summary>
    internal static Vector2d GetDirection(Vector2d start, Vector2d end)
    {
        Signed192 x = GetDifference(end.X, start.X);
        Signed192 y = GetDifference(end.Y, start.Y);
        return GetNormalized(x, y);
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for a nonzero 2D
    /// vector using its exact raw components.
    /// </summary>
    internal static Vector2d GetNormalized(Vector2d value) =>
        GetNormalized(
            WideArithmetic.FromSignedRaw(value.X.m_rawValue),
            WideArithmetic.FromSignedRaw(value.Y.m_rawValue));

    internal static Vector2d GetNormalized(Signed192 x, Signed192 y)
    {
        Signed320 squaredMagnitude = GetSquaredMagnitude(
            x,
            y,
            default,
            out Signed320 xSquare,
            out Signed320 ySquare,
            out _);
        if (squaredMagnitude.IsZero)
            return Vector2d.Zero;

        if (Max(GetMagnitudeBitLength(x), GetMagnitudeBitLength(y))
            <= FixedMath.SHIFT_AMOUNT_I + 1)
        {
            return Vector2d.GetScaleNormalized(new Vector2d(
                Fixed64.FromRaw(unchecked((long)x.Low)),
                Fixed64.FromRaw(unchecked((long)y.Low))));
        }

        Signed192 magnitude = WideArithmetic.GetFloorSquareRoot(squaredMagnitude, out Signed192 remainder);
        Signed192 ceilingMagnitude = remainder.IsZero
            ? magnitude
            : WideArithmetic.AddSigned192(magnitude, WideArithmetic.FromSignedRaw(1L));
        return new Vector2d(
            Fixed64.NormalizeWideComponent(x, xSquare, ceilingMagnitude, squaredMagnitude),
            Fixed64.NormalizeWideComponent(y, ySquare, ceilingMagnitude, squaredMagnitude));
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for exact 2D
    /// components wider than the public scalar domain.
    /// </summary>
    internal static Vector2d GetNormalized(Signed320 x, Signed320 y)
    {
        Signed320 largest = WideArithmetic.CompareMagnitude(x, y) >= 0 ? x : y;
        if (largest.IsZero)
            return Vector2d.Zero;

        Signed320 scale = GetPositiveMagnitude(largest);
        return Vector2d.GetScaleNormalized(new Vector2d(
            Fixed64.GetSignedRatio(x, scale),
            Fixed64.GetSignedRatio(y, scale)));
    }

    /// <summary>
    /// Returns the normalized direction between two 3D endpoints without
    /// narrowing their component differences.
    /// </summary>
    internal static Vector3d GetDirection(Vector3d start, Vector3d end)
    {
        Signed192 x = GetDifference(end.X, start.X);
        Signed192 y = GetDifference(end.Y, start.Y);
        Signed192 z = GetDifference(end.Z, start.Z);
        return GetNormalized(x, y, z);
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for a nonzero 3D
    /// vector using its exact raw components.
    /// </summary>
    internal static Vector3d GetNormalized(Vector3d value) =>
        GetNormalized(
            WideArithmetic.FromSignedRaw(value.X.m_rawValue),
            WideArithmetic.FromSignedRaw(value.Y.m_rawValue),
            WideArithmetic.FromSignedRaw(value.Z.m_rawValue));

    internal static Vector3d GetNormalized(Signed192 x, Signed192 y, Signed192 z)
    {
        Signed320 squaredMagnitude = GetSquaredMagnitude(
            x,
            y,
            z,
            out Signed320 xSquare,
            out Signed320 ySquare,
            out Signed320 zSquare);
        if (squaredMagnitude.IsZero)
            return Vector3d.Zero;

        if (Max(
                Max(GetMagnitudeBitLength(x), GetMagnitudeBitLength(y)),
                GetMagnitudeBitLength(z))
            <= FixedMath.SHIFT_AMOUNT_I + 1)
        {
            return Vector3d.GetScaleNormalized(new Vector3d(
                Fixed64.FromRaw(unchecked((long)x.Low)),
                Fixed64.FromRaw(unchecked((long)y.Low)),
                Fixed64.FromRaw(unchecked((long)z.Low))));
        }

        Signed192 magnitude = WideArithmetic.GetFloorSquareRoot(squaredMagnitude, out Signed192 remainder);
        Signed192 ceilingMagnitude = remainder.IsZero
            ? magnitude
            : WideArithmetic.AddSigned192(magnitude, WideArithmetic.FromSignedRaw(1L));
        return new Vector3d(
            Fixed64.NormalizeWideComponent(x, xSquare, ceilingMagnitude, squaredMagnitude),
            Fixed64.NormalizeWideComponent(y, ySquare, ceilingMagnitude, squaredMagnitude),
            Fixed64.NormalizeWideComponent(z, zSquare, ceilingMagnitude, squaredMagnitude));
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for exact 3D
    /// components wider than the public scalar domain.
    /// </summary>
    internal static Vector3d GetNormalized(Signed320 x, Signed320 y, Signed320 z)
    {
        Signed320 largest = WideArithmetic.CompareMagnitude(x, y) >= 0 ? x : y;
        if (WideArithmetic.CompareMagnitude(z, largest) > 0)
            largest = z;
        if (largest.IsZero)
            return Vector3d.Zero;

        Signed320 scale = GetPositiveMagnitude(largest);
        return Vector3d.GetScaleNormalized(new Vector3d(
            Fixed64.GetSignedRatio(x, scale),
            Fixed64.GetSignedRatio(y, scale),
            Fixed64.GetSignedRatio(z, scale)));
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for a nonzero 4D
    /// vector using its exact raw components.
    /// </summary>
    internal static Vector4d GetNormalized(Vector4d value)
    {
        Signed192 x = WideArithmetic.FromSignedRaw(value.X.m_rawValue);
        Signed192 y = WideArithmetic.FromSignedRaw(value.Y.m_rawValue);
        Signed192 z = WideArithmetic.FromSignedRaw(value.Z.m_rawValue);
        Signed192 w = WideArithmetic.FromSignedRaw(value.W.m_rawValue);
        Signed320 squaredMagnitude = GetSquaredMagnitude(
            x,
            y,
            z,
            out Signed320 xSquare,
            out Signed320 ySquare,
            out Signed320 zSquare);
        Signed320 wSquare = WideArithmetic.MultiplySigned192(w, w);
        squaredMagnitude = WideArithmetic.AddSigned320(squaredMagnitude, wSquare);
        if (squaredMagnitude.IsZero)
            return Vector4d.Zero;

        Signed192 magnitude = WideArithmetic.GetFloorSquareRoot(squaredMagnitude, out Signed192 remainder);
        Signed192 ceilingMagnitude = remainder.IsZero
            ? magnitude
            : WideArithmetic.AddSigned192(magnitude, WideArithmetic.FromSignedRaw(1L));
        return new Vector4d(
            Fixed64.NormalizeWideComponent(x, xSquare, ceilingMagnitude, squaredMagnitude),
            Fixed64.NormalizeWideComponent(y, ySquare, ceilingMagnitude, squaredMagnitude),
            Fixed64.NormalizeWideComponent(z, zSquare, ceilingMagnitude, squaredMagnitude),
            Fixed64.NormalizeWideComponent(w, wSquare, ceilingMagnitude, squaredMagnitude));
    }

    /// <summary>
    /// Returns the nearest representable unit quaternion for nonzero raw
    /// components. The zero quaternion retains its public identity fallback.
    /// </summary>
    internal static FixedQuaternion GetNormalized(FixedQuaternion value)
    {
        Vector4d normalized = GetNormalized(new Vector4d(value.X, value.Y, value.Z, value.W));
        return normalized.IsZero
            ? FixedQuaternion.Identity
            : new FixedQuaternion(normalized.X, normalized.Y, normalized.Z, normalized.W);
    }

    private static int GetMagnitudeBitLength(Signed192 value)
    {
        WideArithmetic.GetMagnitude(value, out ulong high, out ulong middle, out ulong low);
        return WideArithmetic.GetBitLength(high, middle, low);
    }

    private static Signed320 GetPositiveMagnitude(Signed320 value)
    {
        WideArithmetic.GetMagnitude(
            value,
            out ulong word4,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        return new Signed320(word4, word3, word2, word1, word0);
    }

    private static int Max(int left, int right) => left >= right ? left : right;

    private static bool TryRoundDistance(Signed192 squaredDistance, out Fixed64 distance)
    {
        Signed192 root = WideArithmetic.GetFloorSquareRoot(
            WideArithmetic.ExtendToSigned320(squaredDistance),
            out Signed192 remainder);
        WideArithmetic.GetMagnitude(root, out ulong high, out ulong middle, out ulong low);
        // A 2D/3D Fixed64 endpoint difference cannot produce a distance root
        // beyond the middle word; only its representable low-word limit varies.
        if (middle != 0UL || low > (ulong)long.MaxValue)
        {
            distance = Fixed64.MaxValue;
            return false;
        }

        // For integer n = root^2 + remainder, sqrt(n) rounds upward exactly
        // when remainder is at least root + 1. A half-way tie is impossible.
        if (WideArithmetic.CompareMagnitude(remainder, root) > 0)
        {
            if (low == (ulong)long.MaxValue)
            {
                distance = Fixed64.MaxValue;
                return false;
            }

            low++;
        }

        distance = Fixed64.FromRaw((long)low);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetSquaredRadiusSum(Fixed64 firstRadius, Fixed64 secondRadius)
    {
        ulong radiusSum = unchecked((ulong)firstRadius.m_rawValue + (ulong)secondRadius.m_rawValue);
        Fixed64.Multiply64To128(radiusSum, radiusSum, out ulong middle, out ulong low);
        return new Signed192(0UL, middle, low);
    }

    /// <summary>
    /// Returns the exact dot product of two two-dimensional endpoint differences.
    /// </summary>
    internal static Signed192 GetDifferenceDotProduct2D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY)
    {
        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            leftEndX.m_rawValue,
            leftStartX.m_rawValue,
            rightEndX.m_rawValue,
            rightStartX.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndY.m_rawValue,
            leftStartY.m_rawValue,
            rightEndY.m_rawValue,
            rightStartY.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Returns the exact dot product of two three-dimensional endpoint differences.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 GetDifferenceDotProduct3D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 leftEndZ,
        Fixed64 leftStartZ,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY,
        Fixed64 rightEndZ,
        Fixed64 rightStartZ)
    {
        if (TrySubtractRaw(leftEndX.m_rawValue, leftStartX.m_rawValue, out long leftX)
            && TrySubtractRaw(leftEndY.m_rawValue, leftStartY.m_rawValue, out long leftY)
            && TrySubtractRaw(leftEndZ.m_rawValue, leftStartZ.m_rawValue, out long leftZ)
            && TrySubtractRaw(rightEndX.m_rawValue, rightStartX.m_rawValue, out long rightX)
            && TrySubtractRaw(rightEndY.m_rawValue, rightStartY.m_rawValue, out long rightY)
            && TrySubtractRaw(rightEndZ.m_rawValue, rightStartZ.m_rawValue, out long rightZ))
        {
            ulong narrowHigh = 0UL;
            ulong narrowMiddle = 0UL;
            ulong narrowLow = 0UL;
            AccumulateRawProduct(leftX, rightX, false, ref narrowHigh, ref narrowMiddle, ref narrowLow);
            AccumulateRawProduct(leftY, rightY, false, ref narrowHigh, ref narrowMiddle, ref narrowLow);
            AccumulateRawProduct(leftZ, rightZ, false, ref narrowHigh, ref narrowMiddle, ref narrowLow);
            return new Signed192(narrowHigh, narrowMiddle, narrowLow);
        }

        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            leftEndX.m_rawValue,
            leftStartX.m_rawValue,
            rightEndX.m_rawValue,
            rightStartX.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndY.m_rawValue,
            leftStartY.m_rawValue,
            rightEndY.m_rawValue,
            rightStartY.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndZ.m_rawValue,
            leftStartZ.m_rawValue,
            rightEndZ.m_rawValue,
            rightStartZ.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Returns whether an exact squared direction rounds to zero in Q32.32.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsSquaredLengthDegenerate(Signed192 value) =>
        value.High == 0UL
        && value.Middle == 0UL
        && value.Low <= (1UL << (FixedMath.SHIFT_AMOUNT_I - 1));

    /// <summary>
    /// Returns the exact 2D cross product of two endpoint differences.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 GetDifferenceCrossProduct2D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY)
    {
        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            leftEndX.m_rawValue,
            leftStartX.m_rawValue,
            rightEndY.m_rawValue,
            rightStartY.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndY.m_rawValue,
            leftStartY.m_rawValue,
            rightStartX.m_rawValue,
            rightEndX.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Returns all three exact components of a 3D endpoint-difference cross product.
    /// </summary>
    internal static void GetDifferenceCrossProduct3D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 leftEndZ,
        Fixed64 leftStartZ,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY,
        Fixed64 rightEndZ,
        Fixed64 rightStartZ,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z)
    {
        if (TrySubtractRaw(leftEndX.m_rawValue, leftStartX.m_rawValue, out long leftX)
            && TrySubtractRaw(leftEndY.m_rawValue, leftStartY.m_rawValue, out long leftY)
            && TrySubtractRaw(leftEndZ.m_rawValue, leftStartZ.m_rawValue, out long leftZ)
            && TrySubtractRaw(rightEndX.m_rawValue, rightStartX.m_rawValue, out long rightX)
            && TrySubtractRaw(rightEndY.m_rawValue, rightStartY.m_rawValue, out long rightY)
            && TrySubtractRaw(rightEndZ.m_rawValue, rightStartZ.m_rawValue, out long rightZ))
        {
            x = GetRawCrossComponent(leftY, leftZ, rightZ, rightY);
            y = GetRawCrossComponent(leftZ, leftX, rightX, rightZ);
            z = GetRawCrossComponent(leftX, leftY, rightY, rightX);
            return;
        }

        x = GetDifferenceCrossProduct2D(
            leftEndY, leftStartY, leftEndZ, leftStartZ,
            rightEndY, rightStartY, rightEndZ, rightStartZ);
        y = GetDifferenceCrossProduct2D(
            leftEndZ, leftStartZ, leftEndX, leftStartX,
            rightEndZ, rightStartZ, rightEndX, rightStartX);
        z = GetDifferenceCrossProduct2D(
            leftEndX, leftStartX, leftEndY, leftStartY,
            rightEndX, rightStartX, rightEndY, rightStartY);
    }

    /// <summary>
    /// Returns the exact nonnegative squared magnitude of three wide components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 GetSquaredMagnitude(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        out Signed320 xSquare,
        out Signed320 ySquare,
        out Signed320 zSquare)
    {
        xSquare = WideArithmetic.MultiplySigned192(x, x);
        ySquare = WideArithmetic.MultiplySigned192(y, y);
        zSquare = WideArithmetic.MultiplySigned192(z, z);
        return WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(xSquare, ySquare), zSquare);
    }

    /// <summary>
    /// Applies the inclusive public epsilon threshold to an exact Q128.128 magnitude.
    /// </summary>
    internal static bool IsQ128MagnitudeAtMostEpsilon(Signed320 value)
    {
        WideArithmetic.GetMagnitude(
            value,
            out ulong word4,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        ulong epsilonRaw = (ulong)Fixed64.Epsilon.m_rawValue;
        return WideArithmetic.CompareUnsigned(
            word4,
            word3,
            word2,
            word1,
            word0,
            0UL,
            0UL,
            epsilonRaw >> 32,
            epsilonRaw << 32,
            0UL) <= 0;
    }

    /// <summary>
    /// Compares exact squared distances between two pairs of 3D points.
    /// </summary>
    internal static int CompareSquaredDistance3D(
        Fixed64 leftStartX,
        Fixed64 leftEndX,
        Fixed64 leftStartY,
        Fixed64 leftEndY,
        Fixed64 leftStartZ,
        Fixed64 leftEndZ,
        Fixed64 rightStartX,
        Fixed64 rightEndX,
        Fixed64 rightStartY,
        Fixed64 rightEndY,
        Fixed64 rightStartZ,
        Fixed64 rightEndZ)
    {
        Signed192 left = GetDifferenceDotProduct3D(
            leftStartX, leftEndX, leftStartY, leftEndY, leftStartZ, leftEndZ,
            leftStartX, leftEndX, leftStartY, leftEndY, leftStartZ, leftEndZ);
        Signed192 right = GetDifferenceDotProduct3D(
            rightStartX, rightEndX, rightStartY, rightEndY, rightStartZ, rightEndZ,
            rightStartX, rightEndX, rightStartY, rightEndY, rightStartZ, rightEndZ);
        return WideArithmetic.CompareMagnitude(left, right);
    }

    /// <summary>
    /// Returns the exact sign of the scalar triple product of three raw
    /// three-component vectors.
    /// </summary>
    internal static int GetTripleProductSign(
        Fixed64 firstX,
        Fixed64 firstY,
        Fixed64 firstZ,
        Fixed64 secondX,
        Fixed64 secondY,
        Fixed64 secondZ,
        Fixed64 thirdX,
        Fixed64 thirdY,
        Fixed64 thirdZ)
    {
        Signed192 minorX = GetDifferenceCrossProduct2D(
            secondY, Fixed64.Zero, secondZ, Fixed64.Zero,
            thirdY, Fixed64.Zero, thirdZ, Fixed64.Zero);
        Signed192 minorY = GetDifferenceCrossProduct2D(
            secondZ, Fixed64.Zero, secondX, Fixed64.Zero,
            thirdZ, Fixed64.Zero, thirdX, Fixed64.Zero);
        Signed192 minorZ = GetDifferenceCrossProduct2D(
            secondX, Fixed64.Zero, secondY, Fixed64.Zero,
            thirdX, Fixed64.Zero, thirdY, Fixed64.Zero);
        Signed320 tripleProduct = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(WideArithmetic.FromSignedRaw(firstX.m_rawValue), minorX),
                WideArithmetic.MultiplySigned192(WideArithmetic.FromSignedRaw(firstY.m_rawValue), minorY)),
            WideArithmetic.MultiplySigned192(WideArithmetic.FromSignedRaw(firstZ.m_rawValue), minorZ));
        return tripleProduct.Sign;
    }

    /// <summary>
    /// Applies the public 3D segment near-parallel threshold to an exact Q128.128 determinant.
    /// </summary>
    internal static bool IsSegmentDeterminantNearParallel(Signed320 determinant)
    {
        WideArithmetic.GetMagnitude(
            determinant,
            out ulong word4,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        ulong epsilonRaw = (ulong)Fixed64.Epsilon.m_rawValue;
        return WideArithmetic.CompareUnsigned(
            word4,
            word3,
            word2,
            word1,
            word0,
            0UL,
            0UL,
            epsilonRaw >> 32,
            epsilonRaw << 32,
            0UL) < 0;
    }

    /// <summary>
    /// Compares the exact projection of three component differences.
    /// </summary>
    internal static int CompareDifferenceProjection(
        Fixed64 candidateX,
        Fixed64 currentX,
        Fixed64 directionX,
        Fixed64 candidateY,
        Fixed64 currentY,
        Fixed64 directionY,
        Fixed64 candidateZ,
        Fixed64 currentZ,
        Fixed64 directionZ)
    {
        GetDifferenceProjectionWords(
            candidateX,
            currentX,
            directionX,
            candidateY,
            currentY,
            directionY,
            candidateZ,
            currentZ,
            directionZ,
            out ulong sumHigh,
            out ulong sumMiddle,
            out ulong sumLow);

        if ((sumHigh & (1UL << 63)) != 0UL)
            return -1;

        return (sumHigh | sumMiddle | sumLow) == 0UL ? 0 : 1;
    }

    /// <summary>
    /// Returns the exact projection words of three component differences.
    /// </summary>
    internal static void GetDifferenceProjectionWords(
        Fixed64 candidateX,
        Fixed64 currentX,
        Fixed64 directionX,
        Fixed64 candidateY,
        Fixed64 currentY,
        Fixed64 directionY,
        Fixed64 candidateZ,
        Fixed64 currentZ,
        Fixed64 directionZ,
        out ulong sumHigh,
        out ulong sumMiddle,
        out ulong sumLow)
    {
        sumHigh = 0UL;
        sumMiddle = 0UL;
        sumLow = 0UL;
        AccumulateDifferenceProduct(
            candidateX.m_rawValue,
            currentX.m_rawValue,
            directionX.m_rawValue,
            0L,
            ref sumHigh,
            ref sumMiddle,
            ref sumLow);
        AccumulateDifferenceProduct(
            candidateY.m_rawValue,
            currentY.m_rawValue,
            directionY.m_rawValue,
            0L,
            ref sumHigh,
            ref sumMiddle,
            ref sumLow);
        AccumulateDifferenceProduct(
            candidateZ.m_rawValue,
            currentZ.m_rawValue,
            directionZ.m_rawValue,
            0L,
            ref sumHigh,
            ref sumMiddle,
            ref sumLow);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void AccumulateDifferenceProduct(
        long candidate,
        long current,
        long directionEnd,
        long directionStart,
        ref ulong sumHigh,
        ref ulong sumMiddle,
        ref ulong sumLow)
    {
        if (candidate == current || directionEnd == directionStart)
            return;

        bool negativeDifference = candidate < current;
        ulong differenceMagnitude = negativeDifference
            ? unchecked((ulong)current - (ulong)candidate)
            : unchecked((ulong)candidate - (ulong)current);
        bool negativeDirection = directionEnd < directionStart;
        ulong directionMagnitude = negativeDirection
            ? unchecked((ulong)directionStart - (ulong)directionEnd)
            : unchecked((ulong)directionEnd - (ulong)directionStart);
        bool negativeProduct = negativeDifference != negativeDirection;

        Fixed64.Multiply64To128(
            differenceMagnitude,
            directionMagnitude,
            out ulong productMiddle,
            out ulong productLow);

        ulong productHigh = 0UL;
        if (negativeProduct)
        {
            productLow = unchecked(~productLow + 1UL);
            productMiddle = unchecked(~productMiddle + (productLow == 0UL ? 1UL : 0UL));
            productHigh = ulong.MaxValue;
        }

        ulong previousLow = sumLow;
        sumLow = unchecked(sumLow + productLow);
        ulong carry = sumLow < previousLow ? 1UL : 0UL;

        ulong addMiddle = unchecked(productMiddle + carry);
        ulong carryHigh = addMiddle < productMiddle ? 1UL : 0UL;
        ulong previousMiddle = sumMiddle;
        sumMiddle = unchecked(sumMiddle + addMiddle);
        if (sumMiddle < previousMiddle)
            carryHigh = 1UL;

        sumHigh = unchecked(sumHigh + productHigh + carryHigh);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetDifference(Fixed64 end, Fixed64 start) =>
        WideArithmetic.SubtractSigned192(
            WideArithmetic.FromSignedRaw(end.m_rawValue),
            WideArithmetic.FromSignedRaw(start.m_rawValue));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetRawCrossComponent(
        long leftFirst,
        long leftSecond,
        long rightFirst,
        long rightSecond)
    {
        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateRawProduct(leftFirst, rightFirst, false, ref high, ref middle, ref low);
        AccumulateRawProduct(leftSecond, rightSecond, true, ref high, ref middle, ref low);
        return new Signed192(high, middle, low);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TrySubtractRaw(long end, long start, out long difference)
    {
        difference = unchecked(end - start);
        return ((end ^ start) & (end ^ difference)) >= 0L;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AccumulateRawProduct(
        long left,
        long right,
        bool subtract,
        ref ulong sumHigh,
        ref ulong sumMiddle,
        ref ulong sumLow)
    {
        if (left == 0L || right == 0L)
            return;

        ulong leftMagnitude = left < 0L ? unchecked(0UL - (ulong)left) : (ulong)left;
        ulong rightMagnitude = right < 0L ? unchecked(0UL - (ulong)right) : (ulong)right;
        bool negativeProduct = (left < 0L) != (right < 0L) != subtract;
        Fixed64.Multiply64To128(leftMagnitude, rightMagnitude, out ulong productMiddle, out ulong productLow);

        ulong productHigh = 0UL;
        if (negativeProduct)
        {
            productLow = unchecked(~productLow + 1UL);
            productMiddle = unchecked(~productMiddle + (productLow == 0UL ? 1UL : 0UL));
            productHigh = ulong.MaxValue;
        }

        ulong previousLow = sumLow;
        sumLow = unchecked(sumLow + productLow);
        ulong carry = sumLow < previousLow ? 1UL : 0UL;

        ulong addMiddle = unchecked(productMiddle + carry);
        ulong carryHigh = addMiddle < productMiddle ? 1UL : 0UL;
        ulong previousMiddle = sumMiddle;
        sumMiddle = unchecked(sumMiddle + addMiddle);
        if (sumMiddle < previousMiddle)
            carryHigh = 1UL;

        sumHigh = unchecked(sumHigh + productHigh + carryHigh);
    }
}
