using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64EnclosedProductTests
{
    [Fact]
    public void EnclosedQ96Product_ShouldCertifyOnlyUniqueRepresentableHalfEvenResults()
    {
        BigInteger q = BigInteger.One << 96;
        BigInteger[] coefficients = { 0, 1, -1, q / 6, -(q / 6), q, -q,
            q / 2 - 1, q / 2, q / 2 + 1, 3 * q / 2, -3 * q / 2,
            q + (BigInteger.One << 32) - 1, q + (BigInteger.One << 32), q + (BigInteger.One << 32) + 1,
            (BigInteger.One << 100) - 1, BigInteger.One << 100,
            -(BigInteger.One << 100) + 1, -(BigInteger.One << 100),
            (BigInteger.One << 95) + (BigInteger.One << 36),
            -(BigInteger.One << 95) - (BigInteger.One << 36),
            (BigInteger.One << 128) - 1, BigInteger.One << 128, -(BigInteger.One << 128),
            (BigInteger.One << 191) - 1, -(BigInteger.One << 191) };
        long[] masses = { 0, 1, -1, 2, -2, 3, -3, 5, 7, 1L << 32,
            (1L << 58) - 1, 1L << 58, -(1L << 58) + 1, -(1L << 58), long.MaxValue, long.MinValue };
        uint[] errors = { 0, 1, 9, uint.MaxValue };
        foreach (BigInteger coefficient in coefficients)
        foreach (long mass in masses)
        foreach (uint error in errors)
            CheckProduct(coefficient, mass, error);

        // Fill both numerator limbs and exercise their addition carry without
        // selecting samples through the implementation's arithmetic helpers.
        ulong state = 0x96A57F3D8B2041C7UL;
        for (int index = 0; index < 256; index++)
        {
            state = unchecked(state * 6364136223846793005UL + 1442695040888963407UL);
            BigInteger coefficient = ((BigInteger)(state & 0xFFFFFFFFUL) << 64) | (state ^ 0xD83C74BFA219E601UL);
            if ((index & 1) != 0) coefficient = -coefficient;
            state = unchecked(state * 6364136223846793005UL + 1442695040888963407UL);
            CheckProduct(coefficient, unchecked((long)state), (uint)(index % 10));
        }
    }

    [Fact]
    public void EnclosedCoefficient_ShouldRetainSignedValueErrorIdentityAndDefaultZero()
    {
        EnclosedQ96Coefficient zero = default;
        Assert.Equal(BigInteger.Zero, From192(zero.Value));
        Assert.True(zero.TryMultiplyRaw(Fixed64.MaxValue, out long product));
        Assert.Equal(0L, product);
        Assert.True(zero.Equals(new EnclosedQ96Coefficient(default, 0)));
        Assert.False(zero.Equals(new EnclosedQ96Coefficient(default, 9)));
        var value = new EnclosedQ96Coefficient(To192(17), 9);
        Assert.True(value.Equals(new EnclosedQ96Coefficient(To192(17), 9)));
        Assert.False(value.Equals(new EnclosedQ96Coefficient(To192(-17), 9)));
        Assert.False(value.Equals(new EnclosedQ96Coefficient(To192(18), 9)));
        Assert.False(value.Equals(new EnclosedQ96Coefficient(To192(17), 8)));
    }

    [Fact]
    public void SignedWideDivision_ShouldTruncateTowardZeroAndRejectOnlyUnrepresentableQuotients()
    {
        BigInteger[] numerators = { 0, 1, -1, 10, -10, long.MinValue,
            (BigInteger.One << 191) - 1, -(BigInteger.One << 191),
            BigInteger.One << 191, -(BigInteger.One << 191) - 1, 3 * (BigInteger.One << 190),
            BigInteger.One << 500, -(BigInteger.One << 500),
            (BigInteger.One << 575) - 1, -(BigInteger.One << 575) };
        BigInteger[] denominators = { 0, 1, -1, 3, -3, long.MinValue,
            BigInteger.One << 310, -(BigInteger.One << 310),
            (BigInteger.One << 319) - 1, -(BigInteger.One << 319) };
        foreach (BigInteger numerator in numerators)
        foreach (BigInteger denominator in denominators)
        {
            BigInteger expected = denominator == 0 ? 0 : numerator / denominator;
            bool valid = denominator != 0 && expected >= -(BigInteger.One << 191) && expected < BigInteger.One << 191;
            if (denominator == 0)
                Assert.Throws<DivideByZeroException>(() => WideArithmetic.DivideSigned576ToSigned192(To576(numerator), To320(denominator)));
            else if (!valid)
                Assert.Throws<OverflowException>(() => WideArithmetic.DivideSigned576ToSigned192(To576(numerator), To320(denominator)));
            else
                Assert.Equal(expected, From192(WideArithmetic.DivideSigned576ToSigned192(To576(numerator), To320(denominator))));
        }
    }

    [Fact]
    public void SignedWordTruncation_ShouldDiscardFractionTowardZero()
    {
        BigInteger[] values = { 0, 1, -1, ulong.MaxValue, -(BigInteger)ulong.MaxValue,
            BigInteger.One << 64, -(BigInteger.One << 64),
            (BigInteger.One << 64) + 1, -(BigInteger.One << 64) - 1,
            (BigInteger.One << 192) + 17, -(BigInteger.One << 192) - 17,
            (BigInteger.One << 255) - 1, -(BigInteger.One << 255) + 1 };
        foreach (BigInteger value in values)
            Assert.Equal(value / (BigInteger.One << 64), From192(WideArithmetic.TruncateSigned320By64(To320(value))));
    }

    [Fact]
    public void SignedShiftBy32_ShouldPreserveTheLow192Bits()
    {
        BigInteger[] values = { 0, 1, -1, long.MinValue,
            (BigInteger.One << 64) + 3, -(BigInteger.One << 64) - 3,
            BigInteger.One << 159, BigInteger.One << 160,
            (BigInteger.One << 191) - 1, -(BigInteger.One << 191) };
        foreach (BigInteger value in values)
            Assert.Equal(From192(To192(value << 32)), From192(WideArithmetic.ShiftLeftSigned192By32(To192(value))));
    }

    private static void CheckProduct(BigInteger coefficient, long mass, uint error)
    {
        BigInteger product = coefficient * mass;
        BigInteger scale = BigInteger.One << 96;
        BigInteger magnitude = BigInteger.Abs(product);
        BigInteger integral = BigInteger.DivRem(magnitude, scale, out BigInteger fraction);
        BigInteger distance = BigInteger.Abs(fraction - scale / 2);
        BigInteger errorRadius = BigInteger.Abs((BigInteger)mass) * error;
        BigInteger rounded = integral;
        int half = (2 * fraction).CompareTo(scale);
        if (half > 0 || half == 0 && !rounded.IsEven) rounded++;
        if (product.Sign < 0) rounded = -rounded;
        bool exactZero = coefficient == 0 || mass == 0;
        bool certified = exactZero || BigInteger.Abs(coefficient) < BigInteger.One << 128
            && integral <= ulong.MaxValue && (error == 0 || distance > errorRadius)
            && rounded >= long.MinValue && rounded <= long.MaxValue;
        var prepared = new EnclosedQ96Coefficient(To192(coefficient), error);
        Assert.Equal(coefficient, From192(prepared.Value));
        Assert.Equal(certified, prepared.TryMultiplyRaw(new Fixed64(mass), out long actual));
        Assert.Equal(certified ? (long)rounded : 0, actual);
    }

    private static ulong Word(BigInteger value, int index) => (ulong)((value >> (64 * index)) & ulong.MaxValue);
    private static Signed192 To192(BigInteger value) => new(Word(value, 2), Word(value, 1), Word(value, 0));
    private static Signed320 To320(BigInteger value) => new(Word(value, 4), Word(value, 3), Word(value, 2), Word(value, 1), Word(value, 0));
    private static Signed576 To576(BigInteger value) => new(Word(value, 8), Word(value, 7), Word(value, 6), Word(value, 5),
        Word(value, 4), Word(value, 3), Word(value, 2), Word(value, 1), Word(value, 0));
    private static BigInteger From192(Signed192 value)
    {
        BigInteger bits = ((BigInteger)value.High << 128) | ((BigInteger)value.Middle << 64) | value.Low;
        return value.Sign < 0 ? bits - (BigInteger.One << 192) : bits;
    }
}
