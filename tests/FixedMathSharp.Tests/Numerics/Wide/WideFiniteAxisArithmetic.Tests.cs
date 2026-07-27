using FixedMathSharp.Geometry;
using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideFiniteAxisArithmeticTests
{
    [Fact]
    public void WideNormalization_PreservesDirectionsBeyondSigned320()
    {
        Signed576 large = ToSigned576(BigInteger.One << 400);
        Signed576 half = ToSigned576(BigInteger.One << 399);
        Signed576 negativeLarge = ToSigned576(-(BigInteger.One << 400));

        Assert.Equal(
            Vector2d.GetScaleNormalized(new Vector2d(Fixed64.One, Fixed64.Half)),
            WideGeometry.GetNormalized(large, half));
        Assert.Equal(
            Vector2d.GetScaleNormalized(new Vector2d(Fixed64.Half, Fixed64.One)),
            WideGeometry.GetNormalized(half, large));
        Assert.Equal(
            Vector2d.Left,
            WideGeometry.GetNormalized(negativeLarge, default));
        Assert.Equal(
            Vector2d.Zero,
            WideGeometry.GetNormalized(default(Signed576), default(Signed576)));

        Assert.Equal(
            Vector3d.GetScaleNormalized(
                new Vector3d(Fixed64.One, Fixed64.Half, Fixed64.Zero)),
            WideGeometry.GetNormalized(large, half, default));
        Assert.Equal(
            Vector3d.GetScaleNormalized(
                new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero)),
            WideGeometry.GetNormalized(half, large, default));
        Assert.Equal(
            Vector3d.GetScaleNormalized(
                new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.One)),
            WideGeometry.GetNormalized(default, half, large));
        Assert.Equal(
            Vector3d.Left,
            WideGeometry.GetNormalized(negativeLarge, default, default));
        Assert.Equal(
            Vector3d.Zero,
            WideGeometry.GetNormalized(
                default(Signed576),
                default(Signed576),
                default(Signed576)));
    }

    [Fact]
    public void Signed576_AddSubtractAndMultiply_MatchBigIntegerAcrossWordBoundaries()
    {
        BigInteger left = (BigInteger.One << 511) - 1;
        BigInteger right = (BigInteger.One << 256) + 1;
        AssertSigned576(left + right, WideArithmetic.AddSigned576(ToSigned576(left), ToSigned576(right)));
        AssertSigned576(-left + right, WideArithmetic.AddSigned576(ToSigned576(-left), ToSigned576(right)));
        AssertSigned576(left - right, WideArithmetic.SubtractSigned576(ToSigned576(left), ToSigned576(right)));
        AssertSigned576(-left - right, WideArithmetic.SubtractSigned576(ToSigned576(-left), ToSigned576(right)));

        BigInteger first = (BigInteger.One << 261) - 1;
        BigInteger second = (BigInteger.One << 260) + (BigInteger.One << 128) + 1;
        AssertSigned576(first * second, WideArithmetic.MultiplySigned320(ToSigned320(first), ToSigned320(second)));
        AssertSigned576(-(first * second), WideArithmetic.MultiplySigned320(ToSigned320(-first), ToSigned320(second)));
    }

    [Fact]
    public void Signed704_AddAndTripleMultiply_MatchBigIntegerAcrossWordBoundaries()
    {
        BigInteger left = (BigInteger.One << 650) - 1;
        BigInteger right = (BigInteger.One << 384) + 1;
        AssertSigned704(left + right, WideArithmetic.AddSigned704(ToSigned704(left), ToSigned704(right)));
        AssertSigned704(-left + right, WideArithmetic.AddSigned704(ToSigned704(-left), ToSigned704(right)));
        AssertSigned704(BigInteger.Zero, WideArithmetic.AddSigned704(ToSigned704(left), ToSigned704(-left)));

        BigInteger first = (BigInteger.One << 261) - 1;
        BigInteger second = (BigInteger.One << 192) - 1;
        BigInteger third = (BigInteger.One << 191) + (BigInteger.One << 64) + 1;
        AssertSigned704(
            first * second * third,
            WideArithmetic.MultiplySigned320(ToSigned320(first), ToSigned320(second), ToSigned320(third)));
        AssertSigned704(
            -(first * second * third),
            WideArithmetic.MultiplySigned320(ToSigned320(first), ToSigned320(-second), ToSigned320(third)));
    }

    [Fact]
    public void WideMagnitudeExtraction_ClearsOversizedDirtyDestinations()
    {
        BigInteger signed576 = (BigInteger.One << 511) + (BigInteger.One << 257) + 17;
        BigInteger signed704 = (BigInteger.One << 639) + (BigInteger.One << 321) + 19;
        BigInteger signed832 = (BigInteger.One << 767) + (BigInteger.One << 385) + 23;

        AssertMagnitude(signed576, ToSigned576(signed576));
        AssertMagnitude(-signed576, ToSigned576(-signed576));
        AssertMagnitude(BigInteger.Zero, default(Signed576));
        AssertMagnitude(signed704, ToSigned704(signed704));
        AssertMagnitude(-signed704, ToSigned704(-signed704));
        AssertMagnitude(BigInteger.Zero, default(Signed704));
        AssertMagnitude(signed832, ToSigned832(signed832));
        AssertMagnitude(-signed832, ToSigned832(-signed832));
        AssertMagnitude(BigInteger.Zero, default(Signed832));
    }

    [Fact]
    public void WideValueTypedEquality_UsesEveryWordWithoutAllocating()
    {
        Signed192 signed192 = ToSigned192(
            (BigInteger.One << 190)
            + (BigInteger.One << 129)
            + 17);
        Signed192 different192 = ToSigned192(
            (BigInteger.One << 190)
            + (BigInteger.One << 129)
            + 18);
        Signed320 signed320 = ToSigned320(
            (BigInteger.One << 300)
            + (BigInteger.One << 129)
            + 17);
        Signed320 different320 = ToSigned320(
            (BigInteger.One << 300)
            + (BigInteger.One << 129)
            + 18);
        Signed576 signed576 = ToSigned576(
            (BigInteger.One << 511)
            + (BigInteger.One << 257)
            + 17);
        Signed576 different576 = ToSigned576(
            (BigInteger.One << 511)
            + (BigInteger.One << 257)
            + 18);
        Signed832 signed832 = ToSigned832(
            (BigInteger.One << 800)
            + (BigInteger.One << 513)
            + (BigInteger.One << 129)
            + 17);
        Signed832 different832 = ToSigned832(
            (BigInteger.One << 800)
            + (BigInteger.One << 513)
            + (BigInteger.One << 129)
            + 18);

        Assert.True(signed192.Equals(signed192));
        Assert.False(signed192.Equals(different192));
        Assert.True(signed320.Equals(signed320));
        Assert.False(signed320.Equals(different320));
        Assert.True(signed576.Equals(signed576));
        Assert.False(signed576.Equals(different576));
        Assert.True(signed832.Equals(signed832));
        Assert.False(signed832.Equals(different832));

        long before = GC.GetAllocatedBytesForCurrentThread();
        int matches = 0;
        for (int iteration = 0; iteration < 32; iteration++)
        {
            if (signed320.Equals(signed320))
                matches++;
            if (signed832.Equals(signed832))
                matches++;
            if (signed320.Equals(different320)
                || signed832.Equals(different832))
            {
                matches = -1;
            }
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(64, matches);
        Assert.Equal(before, after);
    }

    [Fact]
    public void Signed704_NonnegativeProductAndFloorSquareRoot_MatchBigInteger()
    {
        BigInteger left = (BigInteger.One << 520) + (BigInteger.One << 257) + 1;
        BigInteger right = (BigInteger.One << 120) + 3;
        BigInteger product = left * right;

        Signed704 actualProduct = WideArithmetic.MultiplyNonNegative(
            ToSigned576(left),
            new Signed192(0UL, (ulong)(right >> 64), (ulong)(right & ulong.MaxValue)));
        AssertSigned704(product, actualProduct);
        AssertSigned576(IntegerSquareRoot(product), WideArithmetic.GetFloorSquareRoot(actualProduct));
        AssertSigned576(BigInteger.Zero, WideArithmetic.GetFloorSquareRoot(default));
    }

    [Fact]
    public void NonNegativeRadicalSumComparison_HandlesZeroSingleAndPairedRadicalsExactly()
    {
        static int Compare(
            long firstNumerator,
            long firstDenominator,
            long secondNumerator,
            long secondDenominator,
            long ratioNumerator,
            long ratioDenominator) =>
            WideArithmetic.CompareNonNegativeRadicalSumToRatio(
                ToSigned576(firstNumerator),
                ToSigned192(firstDenominator),
                ToSigned576(secondNumerator),
                ToSigned192(secondDenominator),
                ToSigned320(ratioNumerator),
                ToSigned192(ratioDenominator));

        Assert.Equal(0, Compare(0, 1, 0, 1, 0, 1));
        Assert.Equal(-1, Compare(0, 1, 0, 1, 1, 1));

        Assert.Equal(0, Compare(0, 1, 4, 1, 2, 1));
        Assert.Equal(1, Compare(0, 1, 4, 1, 1, 1));
        Assert.Equal(-1, Compare(0, 1, 4, 1, 3, 1));
        Assert.Equal(0, Compare(4, 1, 0, 1, 2, 1));

        Assert.Equal(1, Compare(1, 1, 4, 1, 2, 1));
        Assert.Equal(0, Compare(1, 1, 4, 1, 3, 1));
        Assert.Equal(-1, Compare(1, 1, 4, 1, 4, 1));
    }

    [Fact]
    public void NonNegativeRadicalComparison_PreservesWideDenominators()
    {
        BigInteger numerator =
            (BigInteger.One << 318)
            + (BigInteger.One << 193)
            + 17;
        BigInteger denominator =
            (BigInteger.One << 191)
            + (BigInteger.One << 127)
            + 3;
        BigInteger ratioDenominator =
            (BigInteger.One << 255)
            + (BigInteger.One << 129)
            + 5;
        BigInteger exactFloor = IntegerSquareRoot(
            (numerator * ratioDenominator * ratioDenominator)
            / denominator);

        Assert.Equal(
            1,
            WideArithmetic.CompareNonNegativeRadicalToRatio(
                ToSigned576(numerator),
                ToSigned320(denominator),
                ToSigned320(exactFloor),
                ToSigned320(ratioDenominator)));
        Assert.Equal(
            -1,
            WideArithmetic.CompareNonNegativeRadicalToRatio(
                ToSigned576(numerator),
                ToSigned320(denominator),
                ToSigned320(exactFloor + 1),
                ToSigned320(ratioDenominator)));
    }

    [Fact]
    public void NonNegativeRadicalComparison_ResolvesTheScalarLimitMidpoint()
    {
        BigInteger scale = BigInteger.One << 32;
        BigInteger maximum = long.MaxValue;
        BigInteger transverse = 3_037_000_499L;
        BigInteger basisScale = BigInteger.Pow(scale, 6);
        BigInteger numerator =
            ((maximum * maximum)
                + (transverse * transverse))
            * basisScale;

        Assert.True(
            (transverse * transverse) - maximum < BigInteger.Zero);
        Assert.Equal(
            -1,
            WideArithmetic.CompareNonNegativeRadicalToRatio(
                ToSigned576(numerator),
                ToSigned320(basisScale),
                ToSigned320((maximum << 1) + BigInteger.One),
                ToSigned320((BigInteger)2)));
    }

    [Fact]
    public void SignedLinearRadicalComparison_PreservesExactCancellation()
    {
        BigInteger radicandNumerator =
            (BigInteger.One << 318)
            + (BigInteger.One << 193)
            + 17;
        BigInteger radicandDenominator =
            (BigInteger.One << 191)
            + (BigInteger.One << 127)
            + 3;
        BigInteger coefficient =
            (BigInteger.One << 630)
            + (BigInteger.One << 257)
            + 5;
        BigInteger exactFloor = IntegerSquareRoot(
            (coefficient
                * coefficient
                * radicandNumerator)
            / radicandDenominator);

        Assert.Equal(
            1,
            WideArithmetic.CompareSignedLinearRadicalToZero(
                ToSigned832(-exactFloor),
                ToSigned704(coefficient),
                ToSigned576(radicandNumerator),
                ToSigned320(radicandDenominator)));
        Assert.Equal(
            -1,
            WideArithmetic.CompareSignedLinearRadicalToZero(
                ToSigned832(-(exactFloor + 1)),
                ToSigned704(coefficient),
                ToSigned576(radicandNumerator),
                ToSigned320(radicandDenominator)));
        Assert.Equal(
            -1,
            WideArithmetic.CompareSignedLinearRadicalToZero(
                ToSigned832(-exactFloor),
                ToSigned704(-coefficient),
                ToSigned576(radicandNumerator),
                ToSigned320(radicandDenominator)));
    }

    [Fact]
    public void SignedLinearRadicalComparison_HandlesDegenerateAndExactTerms()
    {
        Assert.Equal(
            1,
            WideArithmetic.CompareSignedLinearRadicalToZero(
                ToSigned832(BigInteger.Zero),
                ToSigned704(BigInteger.One),
                ToSigned576(BigInteger.One),
                ToSigned320(BigInteger.One)));
        Assert.Equal(
            0,
            WideArithmetic.CompareSignedLinearRadicalToZero(
                ToSigned832(BigInteger.Zero),
                ToSigned704(BigInteger.One),
                ToSigned576(BigInteger.Zero),
                ToSigned320(BigInteger.One)));
        Assert.Equal(
            1,
            WideArithmetic.CompareSignedLinearRadicalToZero(
                ToSigned832(BigInteger.One),
                ToSigned704(BigInteger.One),
                ToSigned576(BigInteger.Zero),
                ToSigned320(BigInteger.One)));
        Assert.Equal(
            0,
            WideArithmetic.CompareSignedLinearRadicalToZero(
                ToSigned832(-BigInteger.One),
                ToSigned704(BigInteger.One),
                ToSigned576(BigInteger.One),
                ToSigned320(BigInteger.One)));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(65)]
    [InlineData(129)]
    [InlineData(193)]
    [InlineData(257)]
    [InlineData(321)]
    [InlineData(385)]
    [InlineData(449)]
    [InlineData(513)]
    [InlineData(577)]
    [InlineData(641)]
    public void Signed704_FloorSquareRoot_MatchesOracleAcrossEveryInputWord(int highBit)
    {
        BigInteger value = (BigInteger.One << highBit) + 3;

        AssertSigned576(
            IntegerSquareRoot(value),
            WideArithmetic.GetFloorSquareRoot(ToSigned704(value)));
    }

    [Fact]
    public void Signed576_RawRatio_RoundsToEvenAndRejectsUnrepresentableResults()
    {
        Assert.True(Fixed64.TryGetSignedRawRatio(ToSigned576(0), ToSigned576(1), out Fixed64 zero));
        Assert.Equal(0L, zero.m_rawValue);
        Assert.False(Fixed64.TryGetSignedRawRatio(ToSigned576(1), default, out _));

        AssertRawRatio(2L, 5, 2);
        AssertRawRatio(4L, 7, 2);
        AssertRawRatio(2L, 5, 3);
        AssertRawRatio(-2L, -5, 2);
        AssertRawRatio(-2L, 5, -2);
        AssertRawRatio(long.MinValue, -(BigInteger.One << 63), 1);

        Assert.False(Fixed64.TryGetSignedRawRatio(
            ToSigned576(BigInteger.One << 64),
            ToSigned576(1),
            out _));
        Assert.False(Fixed64.TryGetSignedRawRatio(
            ToSigned576((BigInteger.One << 65) - 1),
            ToSigned576(2),
            out _));
        Assert.False(Fixed64.TryGetSignedRawRatio(
            ToSigned576(BigInteger.One << 63),
            ToSigned576(1),
            out _));
    }

    [Fact]
    public void Signed832_RawRatioRejectsZeroDenominator()
    {
        Assert.False(Fixed64.TryGetSignedRawRatio(
            ToSigned832(BigInteger.One),
            default,
            numeratorLeftShift: 0,
            out _));
    }

    [Fact]
    public void ProductCombinationRejectsUnrepresentableExactSums()
    {
        Assert.False(Fixed64.TryAddProducts(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out _));
        Assert.False(Fixed64.TrySubtractProducts(
            Fixed64.MaxValue,
            -Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out _));
    }

    [Fact]
    public void Signed704_RawRatio_HandlesWideWordsRoundingAndRange()
    {
        Assert.True(Fixed64.TryGetSignedRawRatio(
            ToSigned704(0),
            ToSigned704(1),
            out Fixed64 zero));
        Assert.Equal(Fixed64.Zero, zero);
        Assert.False(Fixed64.TryGetSignedRawRatio(
            ToSigned704(1),
            default,
            out _));

        BigInteger wideUnit = BigInteger.One << 640;
        Assert.True(Fixed64.TryGetSignedRawRatio(
            ToSigned704((wideUnit * 5) + (wideUnit / 2)),
            ToSigned704(wideUnit),
            out Fixed64 roundedEven));
        Assert.Equal(Fixed64.FromRaw(6L), roundedEven);
        Assert.True(Fixed64.TryGetSignedRawRatio(
            ToSigned704(-(wideUnit * 5)),
            ToSigned704(wideUnit * 2),
            out Fixed64 negative));
        Assert.Equal(Fixed64.FromRaw(-2L), negative);
        Assert.False(Fixed64.TryGetSignedRawRatio(
            ToSigned704(BigInteger.One << 64),
            ToSigned704(1),
            out _));
    }

    [Fact]
    public void Signed576_RawRatio_HandlesEveryMagnitudeWordWithoutNarrowing()
    {
        for (int bit = 0; bit <= 512; bit += 64)
        {
            BigInteger denominator = BigInteger.One << bit;
            AssertRawRatio(3L, denominator * 3, denominator);
        }
    }

    [Fact]
    public void Signed576_Fixed64ScaledFloorSquareRoot_PreservesFractionalRootBits()
    {
        BigInteger value = (BigInteger.One << 519) + (BigInteger.One << 257) + 3;
        BigInteger expected = IntegerSquareRoot(value << 64);

        AssertSigned320(BigInteger.Zero, WideArithmetic.GetFloorSquareRootScaledByFixed64(default));
        AssertSigned320(expected, WideArithmetic.GetFloorSquareRootScaledByFixed64(ToSigned576(value)));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(65)]
    [InlineData(129)]
    [InlineData(193)]
    [InlineData(257)]
    [InlineData(321)]
    [InlineData(385)]
    [InlineData(449)]
    [InlineData(519)]
    public void Signed576_Fixed64ScaledFloorSquareRoot_MatchesOracleAcrossEveryInputWord(int highBit)
    {
        BigInteger value = (BigInteger.One << highBit) + 1;

        AssertSigned320(
            IntegerSquareRoot(value << 64),
            WideArithmetic.GetFloorSquareRootScaledByFixed64(ToSigned576(value)));
    }

    [Fact]
    public void Signed576_NonnegativeComparisonHonorsEveryWordAndOrdering()
    {
        for (int bit = 0; bit <= 512; bit += 64)
        {
            Signed576 value = ToSigned576(BigInteger.One << bit);

            Assert.Equal(1, WideArithmetic.CompareNonNegative(value, default));
            Assert.Equal(-1, WideArithmetic.CompareNonNegative(default, value));
            Assert.Equal(0, WideArithmetic.CompareNonNegative(value, value));
        }
    }

    [Fact]
    public void ConicProvenFitProducts_MatchBigIntegerAcrossHighWordsAndSigns()
    {
        BigInteger left = (BigInteger.One << 430) + (BigInteger.One << 257) + 1;
        BigInteger narrow = (BigInteger.One << 120) + (BigInteger.One << 65) + 3;
        BigInteger medium = (BigInteger.One << 250) + (BigInteger.One << 129) + 5;

        AssertSigned576(
            left * narrow,
            WideArithmetic.MultiplySigned576(ToSigned576(left), ToSigned192(narrow)));
        AssertSigned576(
            -(left * narrow),
            WideArithmetic.MultiplySigned576(ToSigned576(-left), ToSigned192(narrow)));
        AssertSigned704(
            left * medium,
            WideArithmetic.MultiplySigned576ToSigned704(ToSigned576(left), ToSigned320(medium)));
        AssertSigned704(
            -(left * medium),
            WideArithmetic.MultiplySigned576ToSigned704(ToSigned576(left), ToSigned320(-medium)));
    }

    [Fact]
    public void Signed832_MultiplySubtractAndProductSquareRoot_MatchBigIntegerOracle()
    {
        BigInteger left = (BigInteger.One << 410) + (BigInteger.One << 321) + 1;
        BigInteger right = (BigInteger.One << 368) + (BigInteger.One << 129) + 3;
        BigInteger product = left * right;
        Signed832 actualProduct = WideArithmetic.MultiplySigned576ToSigned832(
            ToSigned576(left),
            ToSigned576(right));
        AssertSigned832(product, actualProduct);
        AssertSigned832(
            -product,
            WideArithmetic.MultiplySigned576ToSigned832(ToSigned576(-left), ToSigned576(right)));

        BigInteger minuend = (BigInteger.One << 810) + (BigInteger.One << 705);
        BigInteger subtrahend = (BigInteger.One << 769) + (BigInteger.One << 641) + 1;
        AssertSigned832(
            minuend - subtrahend,
            WideArithmetic.SubtractSigned832(ToSigned832(minuend), ToSigned832(subtrahend)));
        AssertSigned832(
            -minuend - subtrahend,
            WideArithmetic.SubtractSigned832(ToSigned832(-minuend), ToSigned832(subtrahend)));

        BigInteger factor = (BigInteger.One << 120) + 7;
        AssertSigned576(
            IntegerSquareRoot(product * factor),
            WideArithmetic.GetFloorSquareRootOfProduct(actualProduct, ToSigned192(factor)));
        AssertSigned576(
            BigInteger.Zero,
            WideArithmetic.GetFloorSquareRootOfProduct(default, ToSigned192(factor)));
        AssertSigned576(
            BigInteger.Zero,
            WideArithmetic.GetFloorSquareRootOfProduct(actualProduct, default));
    }

    [Fact]
    public void BoundedConePolynomialEvaluation_MatchesBigIntegerAcrossRigidFrameWidths()
    {
        ulong state = 0xD1B54A32D192ED03UL;
        for (int iteration = 0; iteration < 64; iteration++)
        {
            BigInteger coefficient =
                (BigInteger.One << (380 + iteration % 141))
                + (NextUInt64(ref state) & 0xFFFFUL);
            BigInteger projection =
                (BigInteger.One << (379 + (iteration * 3) % 141))
                + (NextUInt64(ref state) & 0xFFFFUL);
            BigInteger constant =
                (BigInteger.One << (378 + (iteration * 5) % 141))
                + (NextUInt64(ref state) & 0xFFFFUL);
            BigInteger numerator =
                (BigInteger.One << (120 + iteration % 11))
                + (NextUInt64(ref state) & 0xFFFFUL);
            BigInteger denominator =
                (BigInteger.One << (120 + (iteration * 7) % 11))
                + (NextUInt64(ref state) & 0xFFFFUL)
                + 1;
            if ((iteration & 1) != 0)
                coefficient = -coefficient;
            if ((iteration & 2) != 0)
                projection = -projection;
            if ((iteration & 4) != 0)
                constant = -constant;
            if ((iteration & 8) != 0)
                numerator = -numerator;

            BigInteger value =
                coefficient * numerator * numerator
                + 2 * projection * numerator * denominator
                + constant * denominator * denominator;
            BigInteger derivative =
                coefficient * numerator
                + projection * denominator;

            Assert.Equal(
                value.Sign,
                WideFiniteConeIntersection.EvaluateBoundedUnitPolynomialSign(
                    ToSigned576(coefficient),
                    ToSigned576(projection),
                    ToSigned576(constant),
                    ToSigned192(numerator),
                    ToSigned192(denominator)));
            Assert.Equal(
                derivative.Sign,
                WideFiniteConeIntersection.EvaluateBoundedUnitPolynomialDerivativeSign(
                    ToSigned576(coefficient),
                    ToSigned576(projection),
                    ToSigned192(numerator),
                    ToSigned192(denominator)));
        }
    }

    [Fact]
    public void AdaptiveWideOperations_MatchBigIntegerForDeterministicSparseOperands()
    {
        ulong state = 0x9E3779B97F4A7C15UL;
        for (int iteration = 0; iteration < 64; iteration++)
        {
            int firstShift = (iteration % 4) * 64;
            int secondShift = ((iteration * 3) % 4) * 64;
            BigInteger first = ((BigInteger)(NextUInt64(ref state) & 0x1FFFFFFFFFFFFFFFUL) << firstShift)
                + (NextUInt64(ref state) & 0xFFFFUL);
            BigInteger second = ((BigInteger)(NextUInt64(ref state) & 0x1FFFFFFFFFFFFFFFUL) << secondShift)
                + (NextUInt64(ref state) & 0xFFFFUL);
            if ((iteration & 1) != 0)
                first = -first;
            if ((iteration & 2) != 0)
                second = -second;

            AssertSigned576(
                first * second,
                WideArithmetic.MultiplySigned320(ToSigned320(first), ToSigned320(second)));

            BigInteger nonnegative = BigInteger.Abs(first * second) + iteration + 1;
            AssertSigned320(
                IntegerSquareRoot(nonnegative << 64),
                WideArithmetic.GetFloorSquareRootScaledByFixed64(ToSigned576(nonnegative)));

            int denominatorShift = (iteration % 8) * 64;
            BigInteger denominator = (((BigInteger)(NextUInt64(ref state) | 1UL)) << denominatorShift) + 1;
            long quotient = (long)(NextUInt64(ref state) & 0x3FFFFFFFUL);
            BigInteger remainder = denominator >> 1;
            BigInteger numerator = (denominator * quotient) + remainder;
            BigInteger expected = BigInteger.DivRem(numerator, denominator, out BigInteger actualRemainder);
            int midpoint = (actualRemainder << 1).CompareTo(denominator);
            if (midpoint > 0 || (midpoint == 0 && !expected.IsEven))
                expected++;
            AssertRawRatio((long)expected, numerator, denominator);
        }
    }

    [Fact]
    public void LowerWidthDispatchBoundaries_MatchBigIntegerOracle()
    {
        BigInteger narrow = (BigInteger.One << 158) + (BigInteger.One << 64) + 1;
        BigInteger narrowPeer = (BigInteger.One << 159) - 1;
        BigInteger widerProductPeer = (BigInteger.One << 160) + 3;
        AssertSigned576(
            narrow * narrowPeer,
            WideArithmetic.MultiplySigned320(ToSigned320(narrow), ToSigned320(narrowPeer)));
        AssertSigned576(
            -(narrow * widerProductPeer),
            WideArithmetic.MultiplySigned320(ToSigned320(-narrow), ToSigned320(widerProductPeer)));
        AssertSigned576(
            narrow * narrowPeer,
            WideArithmetic.MultiplySigned576(ToSigned576(narrow), ToSigned192(narrowPeer)));
        AssertSigned576(
            -(widerProductPeer * narrowPeer),
            WideArithmetic.MultiplySigned576(ToSigned576(-widerProductPeer), ToSigned192(narrowPeer)));

        BigInteger narrowRadicand = (BigInteger.One << 254) + (BigInteger.One << 127) + 1;
        BigInteger wideRadicand = (BigInteger.One << 255) + (BigInteger.One << 129) + 1;
        AssertSigned320(
            IntegerSquareRoot(narrowRadicand << 64),
            WideArithmetic.GetFloorSquareRootScaledByFixed64(ToSigned576(narrowRadicand)));
        AssertSigned320(
            IntegerSquareRoot(wideRadicand << 64),
            WideArithmetic.GetFloorSquareRootScaledByFixed64(ToSigned576(wideRadicand)));

        BigInteger denominator192 = (BigInteger.One << 190) + 3;
        BigInteger denominator320 = (BigInteger.One << 192) + 5;
        AssertRawRatio(7, (denominator192 * 7) + (denominator192 >> 2), denominator192);
        AssertRawRatio(-9, -((denominator320 * 9) + (denominator320 >> 2)), denominator320);
        AssertRawRatio(11, ((BigInteger.One << 321) + 7) * 11, (BigInteger.One << 321) + 7);
    }

    [Fact]
    public void WideNormalization_SelectsTheLargestMagnitudeAcrossEveryAxis()
    {
        Assert.Equal(
            new Vector2d(Fixed64.FromRaw(3), Fixed64.FromRaw(-4)).Normalized,
            WideGeometry.GetNormalized(ToSigned320(3), ToSigned320(-4)));
        Assert.Equal(
            new Vector2d(Fixed64.FromRaw(4), Fixed64.FromRaw(3)).Normalized,
            WideGeometry.GetNormalized(ToSigned320(4), ToSigned320(3)));
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(3), Fixed64.FromRaw(4), Fixed64.FromRaw(12)).Normalized,
            WideGeometry.GetNormalized(ToSigned320(3), ToSigned320(4), ToSigned320(12)));
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(4), Fixed64.FromRaw(3), Fixed64.FromRaw(1)).Normalized,
            WideGeometry.GetNormalized(ToSigned320(4), ToSigned320(3), ToSigned320(1)));
        Assert.Equal(Vector4d.Zero, WideGeometry.GetNormalized(Vector4d.Zero));
        Assert.Equal(FixedQuaternion.Identity, WideGeometry.GetNormalized(default(FixedQuaternion)));
    }

    private static BigInteger IntegerSquareRoot(BigInteger value)
    {
        BigInteger low = BigInteger.Zero;
        int highBit = checked((int)(((value.GetBitLength() + 1) >> 1) + 1));
        BigInteger high = BigInteger.One << highBit;
        while (high - low > BigInteger.One)
        {
            BigInteger middle = (low + high) >> 1;
            if (middle * middle <= value)
                low = middle;
            else
                high = middle;
        }

        return low;
    }

    private static ulong NextUInt64(ref ulong state)
    {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        return state * 2685821657736338717UL;
    }

    private static void AssertRawRatio(long expectedRaw, BigInteger numerator, BigInteger denominator)
    {
        Assert.True(Fixed64.TryGetSignedRawRatio(
            ToSigned576(numerator),
            ToSigned576(denominator),
            out Fixed64 actual));
        Assert.Equal(expectedRaw, actual.m_rawValue);
    }

    private static void AssertMagnitude(BigInteger expected, Signed576 value)
    {
        Span<ulong> magnitude = stackalloc ulong[36];
        magnitude.Fill(ulong.MaxValue);
        WideArithmetic.GetMagnitude(value, magnitude);
        Assert.Equal(BigInteger.Abs(expected), FromMagnitudeWords(magnitude));
        AssertZeroTail(magnitude, 9);
    }

    private static void AssertMagnitude(BigInteger expected, Signed704 value)
    {
        Span<ulong> magnitude = stackalloc ulong[36];
        magnitude.Fill(ulong.MaxValue);
        WideArithmetic.GetMagnitude(value, magnitude);
        Assert.Equal(BigInteger.Abs(expected), FromMagnitudeWords(magnitude));
        AssertZeroTail(magnitude, 11);
    }

    private static void AssertMagnitude(BigInteger expected, Signed832 value)
    {
        Span<ulong> magnitude = stackalloc ulong[36];
        magnitude.Fill(ulong.MaxValue);
        WideArithmetic.GetMagnitude(value, magnitude);
        Assert.Equal(BigInteger.Abs(expected), FromMagnitudeWords(magnitude));
        AssertZeroTail(magnitude, 13);
    }

    private static void AssertZeroTail(
        ReadOnlySpan<ulong> magnitude,
        int start)
    {
        for (int index = start; index < magnitude.Length; index++)
            Assert.Equal(0UL, magnitude[index]);
    }

    private static void AssertSigned576(BigInteger expected, Signed576 actual)
    {
        Assert.Equal(expected, ToBigInteger(actual));
        Assert.Equal(expected.Sign, actual.Sign);
        Assert.Equal(expected.IsZero, actual.IsZero);
    }

    private static void AssertSigned320(BigInteger expected, Signed320 actual)
    {
        Assert.Equal(expected, ToBigInteger(actual));
        Assert.Equal(expected.Sign, actual.Sign);
        Assert.Equal(expected.IsZero, actual.IsZero);
    }

    private static void AssertSigned704(BigInteger expected, Signed704 actual)
    {
        Assert.Equal(expected, ToBigInteger(actual));
        Assert.Equal(expected.Sign, actual.Sign);
        Assert.Equal(expected.IsZero, actual.IsZero);
    }

    private static Signed320 ToSigned320(BigInteger value)
    {
        ulong[] words = ToTwosComplementWords(value, 5);
        return new Signed320(words[4], words[3], words[2], words[1], words[0]);
    }

    private static Signed192 ToSigned192(BigInteger value)
    {
        ulong[] words = ToTwosComplementWords(value, 3);
        return new Signed192(words[2], words[1], words[0]);
    }

    private static Signed576 ToSigned576(BigInteger value)
    {
        ulong[] words = ToTwosComplementWords(value, 9);
        return new Signed576(words[8], words[7], words[6], words[5], words[4], words[3], words[2], words[1], words[0]);
    }

    private static Signed704 ToSigned704(BigInteger value)
    {
        ulong[] words = ToTwosComplementWords(value, 11);
        return new Signed704(words[10], words[9], words[8], words[7], words[6], words[5], words[4], words[3], words[2], words[1], words[0]);
    }

    private static Signed832 ToSigned832(BigInteger value)
    {
        ulong[] words = ToTwosComplementWords(value, 13);
        return new Signed832(
            words[12], words[11], words[10], words[9], words[8], words[7], words[6],
            words[5], words[4], words[3], words[2], words[1], words[0]);
    }

    private static ulong[] ToTwosComplementWords(BigInteger value, int wordCount)
    {
        BigInteger modulus = BigInteger.One << (wordCount * 64);
        BigInteger encoded = value.Sign < 0 ? modulus + value : value;
        var words = new ulong[wordCount];
        for (int index = 0; index < words.Length; index++)
        {
            words[index] = (ulong)(encoded & ulong.MaxValue);
            encoded >>= 64;
        }

        return words;
    }

    private static BigInteger ToBigInteger(Signed576 value)
    {
        ulong[] words =
        {
            value.Word0, value.Word1, value.Word2, value.Word3, value.Word4,
            value.Word5, value.Word6, value.Word7, value.Word8
        };
        return FromTwosComplementWords(words, value.Sign);
    }

    private static BigInteger ToBigInteger(Signed320 value)
    {
        ulong[] words = { value.Word0, value.Word1, value.Word2, value.Word3, value.Word4 };
        return FromTwosComplementWords(words, value.Sign);
    }

    private static BigInteger ToBigInteger(Signed704 value)
    {
        ulong[] words =
        {
            value.Word0, value.Word1, value.Word2, value.Word3, value.Word4, value.Word5,
            value.Word6, value.Word7, value.Word8, value.Word9, value.Word10
        };
        return FromTwosComplementWords(words, value.Sign);
    }

    private static void AssertSigned832(BigInteger expected, Signed832 actual)
    {
        Assert.Equal(expected, ToBigInteger(actual));
        Assert.Equal(expected.Sign, actual.Sign);
        Assert.Equal(expected.IsZero, actual.IsZero);
    }

    private static BigInteger ToBigInteger(Signed832 value)
    {
        ulong[] words =
        {
            value.Word0, value.Word1, value.Word2, value.Word3, value.Word4,
            value.Word5, value.Word6, value.Word7, value.Word8, value.Word9,
            value.Word10, value.Word11, value.Word12
        };
        return FromTwosComplementWords(words, value.Sign);
    }

    private static BigInteger FromTwosComplementWords(ulong[] words, int sign)
    {
        BigInteger encoded = BigInteger.Zero;
        for (int index = words.Length - 1; index >= 0; index--)
            encoded = (encoded << 64) | words[index];
        return sign < 0 ? encoded - (BigInteger.One << (words.Length * 64)) : encoded;
    }

    private static BigInteger FromMagnitudeWords(ReadOnlySpan<ulong> words)
    {
        BigInteger value = BigInteger.Zero;
        for (int index = words.Length - 1; index >= 0; index--)
            value = (value << 64) | words[index];
        return value;
    }
}
