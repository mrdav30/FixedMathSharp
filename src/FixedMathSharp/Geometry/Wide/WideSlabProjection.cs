//=======================================================================
// WideSlabProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns full-domain candidate construction for finite-slab projections.
/// </summary>
internal static partial class WideSlabProjection
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

    private static Signed832 Add(Signed832 left, Signed832 right) =>
        WideArithmetic.AddSigned832(left, right);

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
        System.Span<ulong> xMagnitude = stackalloc ulong[10];
        System.Span<ulong> yMagnitude = stackalloc ulong[10];
        System.Span<ulong> zMagnitude = stackalloc ulong[10];
        System.Span<ulong> lengthMagnitude = stackalloc ulong[10];
        xMagnitude.Clear();
        yMagnitude.Clear();
        zMagnitude.Clear();
        lengthMagnitude.Clear();
        WideArithmetic.GetMagnitude(x, xMagnitude[..9]);
        WideArithmetic.GetMagnitude(y, yMagnitude[..9]);
        WideArithmetic.GetMagnitude(z, zMagnitude[..9]);
        WideArithmetic.GetMagnitude(length, lengthMagnitude[..9]);
        int shift = System.Math.Max(0,
            System.Math.Max(
                System.Math.Max(GetBitLength(xMagnitude), GetBitLength(yMagnitude)),
                System.Math.Max(GetBitLength(zMagnitude), GetBitLength(lengthMagnitude))) - 190);
        reducedX = CreateReduced(xMagnitude, shift, x.Sign < 0);
        reducedY = CreateReduced(yMagnitude, shift, y.Sign < 0);
        reducedZ = CreateReduced(zMagnitude, shift, z.Sign < 0);
        reducedLength = CreateReduced(lengthMagnitude, shift, false);
    }

    private static int GetBitLength(System.ReadOnlySpan<ulong> magnitude)
    {
        for (int index = magnitude.Length - 1; index >= 0; index--)
        {
            ulong word = magnitude[index];
            if (word == 0UL)
                continue;

            int wordBits = 0;
            while (word != 0UL)
            {
                wordBits++;
                word >>= 1;
            }
            return (index * 64) + wordBits;
        }
        return 0;
    }

    private static Signed192 CreateReduced(System.ReadOnlySpan<ulong> magnitude, int shift, bool negative)
    {
        int wordShift = shift >> 6;
        int bitShift = shift & 63;
        ulong low = GetShiftedWord(magnitude, wordShift, bitShift);
        ulong middle = GetShiftedWord(magnitude, wordShift + 1, bitShift);
        ulong high = GetShiftedWord(magnitude, wordShift + 2, bitShift);
        Signed192 value = new(high, middle, low);
        return negative ? WideArithmetic.SubtractSigned192(default, value) : value;
    }

    private static ulong GetShiftedWord(System.ReadOnlySpan<ulong> magnitude, int wordIndex, int bitShift)
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
}
