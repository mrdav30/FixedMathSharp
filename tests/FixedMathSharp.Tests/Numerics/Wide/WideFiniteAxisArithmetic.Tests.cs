using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideFiniteAxisArithmeticTests
{
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
}
