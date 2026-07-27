using FixedMathSharp.Bounds;
using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedBoundAreaFiniteAxisTests
{
    [Fact]
    public void RotatedCapsuleBounds_ShouldUseScalarFrameAndRemainPeriodicAtFullDomain()
    {
        Fixed64 halfDomain = Fixed64.FromRaw(1L << 62);
        FixedBoundArea vertical = FixedBoundArea
            .FromCenteredRotatedCapsuleClippedToDomain(
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.Zero);
        FixedBoundArea horizontal = FixedBoundArea
            .FromCenteredRotatedCapsuleClippedToDomain(
                Vector2d.Zero,
                Fixed64.HalfPi,
                Fixed64.MaxValue,
                Fixed64.Zero);
        Fixed64 rotation = FixedMath.DegToRad((Fixed64)30);
        FixedBoundArea first = FixedBoundArea
            .FromCenteredRotatedCapsuleClippedToDomain(
                Vector2d.Zero,
                rotation,
                Fixed64.MaxValue,
                Fixed64.Half);
        FixedBoundArea wrapped = FixedBoundArea
            .FromCenteredRotatedCapsuleClippedToDomain(
                Vector2d.Zero,
                rotation + Fixed64.TwoPi,
                Fixed64.MaxValue,
                Fixed64.Half);

        Assert.Equal(
            FixedBoundArea.FromMinMax(
                new Vector2d(Fixed64.Zero, -halfDomain),
                new Vector2d(Fixed64.Zero, halfDomain)),
            vertical);
        Assert.Equal(
            FixedBoundArea.FromMinMax(
                new Vector2d(-halfDomain, Fixed64.Zero),
                new Vector2d(halfDomain, Fixed64.Zero)),
            horizontal);
        Assert.Equal(first, wrapped);
        Assert.Equal(-first.Max, first.Min);
    }

    [Fact]
    public void CenteredCapsuleBounds_AreTightForCardinalAndDegenerateGeometry()
    {
        Vector2d center = new(10, 20);

        Assert.Equal(
            FixedBoundArea.FromMinMax(new Vector2d(8, 15), new Vector2d(12, 25)),
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                center,
                Vector2d.Forward,
                (Fixed64)6,
                (Fixed64)2));
        Assert.Equal(
            FixedBoundArea.FromMinMax(new Vector2d(8, 18), new Vector2d(12, 22)),
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                center,
                Vector2d.Forward,
                Fixed64.Zero,
                (Fixed64)2));
        Assert.Equal(
            FixedBoundArea.FromMinMax(new Vector2d(10, 17), new Vector2d(10, 23)),
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                center,
                Vector2d.Forward,
                (Fixed64)6,
                Fixed64.Zero));
        Assert.Equal(
            FixedBoundArea.FromMinMax(center, center),
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                center,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero));
    }

    [Fact]
    public void CenteredCapsuleBounds_RejectInvalidGeometryWithExactParameters()
    {
        ArgumentException zeroAxis = Assert.Throws<ArgumentException>(() =>
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.One,
                Fixed64.One));
        Assert.Equal("axisDirection", zeroAxis.ParamName);

        ArgumentException nonUnitAxis = Assert.Throws<ArgumentException>(() =>
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                Vector2d.Zero,
                Vector2d.One,
                Fixed64.One,
                Fixed64.One));
        Assert.Equal("axisDirection", nonUnitAxis.ParamName);

        ArgumentOutOfRangeException negativeLength = Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                Vector2d.Zero,
                Vector2d.Right,
                -Fixed64.One,
                Fixed64.One));
        Assert.Equal("fullAxisLength", negativeLength.ParamName);

        ArgumentOutOfRangeException negativeRadius = Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                -Fixed64.One));
        Assert.Equal("radius", negativeRadius.ParamName);

        ArgumentOutOfRangeException negativeRotatedLength =
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedBoundArea.FromCenteredRotatedCapsuleClippedToDomain(
                    Vector2d.Zero,
                    Fixed64.Zero,
                    -Fixed64.One,
                    Fixed64.One));
        Assert.Equal("fullAxisLength", negativeRotatedLength.ParamName);

        ArgumentOutOfRangeException negativeRotatedRadius =
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedBoundArea.FromCenteredRotatedCapsuleClippedToDomain(
                    Vector2d.Zero,
                    Fixed64.Zero,
                    Fixed64.One,
                    -Fixed64.One));
        Assert.Equal("radius", negativeRotatedRadius.ParamName);
    }

    [Fact]
    public void CenteredCapsuleBounds_RoundOddRawHalfLengthOutwardOnce()
    {
        Fixed64 rawOne = Fixed64.FromRaw(1);
        Fixed64 rawTwo = Fixed64.FromRaw(2);

        FixedBoundArea aroundZero = FixedBoundArea.FromCenteredCapsuleClippedToDomain(
            Vector2d.Zero,
            Vector2d.Right,
            rawOne,
            Fixed64.Zero);
        Assert.Equal(new Vector2d(-rawOne, Fixed64.Zero), aroundZero.Min);
        Assert.Equal(new Vector2d(rawOne, Fixed64.Zero), aroundZero.Max);

        FixedBoundArea alreadyOutward = FixedBoundArea.FromCenteredCapsuleClippedToDomain(
            new Vector2d(rawOne, Fixed64.Zero),
            Vector2d.Right,
            rawOne,
            Fixed64.Zero);
        Assert.Equal(Vector2d.Zero, alreadyOutward.Min);
        Assert.Equal(new Vector2d(rawTwo, Fixed64.Zero), alreadyOutward.Max);

        FixedBoundArea exactInteger = FixedBoundArea.FromCenteredCapsuleClippedToDomain(
            Vector2d.Zero,
            Vector2d.Right,
            rawTwo,
            Fixed64.Zero);
        Assert.Equal(new Vector2d(-rawOne, Fixed64.Zero), exactInteger.Min);
        Assert.Equal(new Vector2d(rawOne, Fixed64.Zero), exactInteger.Max);
    }

    [Fact]
    public void CenteredCapsuleBounds_MatchExactObliqueOracleAndAxisSign()
    {
        Vector2d axis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Vector2d center = new(
            Fixed64.FromRaw(7),
            Fixed64.FromRaw(-11));
        Fixed64 length = Fixed64.FromRaw(17);
        Fixed64 radius = Fixed64.FromRaw(13);

        FixedBoundArea expected = GetExpectedBounds(center, axis, length, radius);
        Assert.Equal(
            expected,
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(center, axis, length, radius));
        Assert.Equal(
            expected,
            FixedBoundArea.FromCenteredCapsuleClippedToDomain(center, -axis, length, radius));
    }

    [Fact]
    public void CenteredCapsuleBounds_ClipOnlyFinalMirroredScalarFaces()
    {
        Vector2d axis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Fixed64 length = Fixed64.FromRaw(17);
        Fixed64 radius = Fixed64.FromRaw(13);
        Vector2d maximumCenter = new(Fixed64.MaxValue, Fixed64.MaxValue);
        Vector2d minimumCenter = new(Fixed64.MinValue, Fixed64.MinValue);

        FixedBoundArea maximum = FixedBoundArea.FromCenteredCapsuleClippedToDomain(
            maximumCenter,
            axis,
            length,
            radius);
        FixedBoundArea minimum = FixedBoundArea.FromCenteredCapsuleClippedToDomain(
            minimumCenter,
            axis,
            length,
            radius);

        Assert.Equal(GetExpectedBounds(maximumCenter, axis, length, radius), maximum);
        Assert.Equal(GetExpectedBounds(minimumCenter, axis, length, radius), minimum);
        Assert.Equal(maximumCenter, maximum.Max);
        Assert.Equal(minimumCenter, minimum.Min);
    }

    [Fact]
    public void CenteredCapsuleBounds_FullDomainExtentClipsWithoutIntermediateSaturation()
    {
        FixedBoundArea bounds = FixedBoundArea.FromCenteredCapsuleClippedToDomain(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.MaxValue,
            Fixed64.MaxValue);

        Assert.Equal(Fixed64.MinValue, bounds.Min.X);
        Assert.Equal(Fixed64.MaxValue, bounds.Max.X);
        Assert.Equal(-Fixed64.MaxValue, bounds.Min.Y);
        Assert.Equal(Fixed64.MaxValue, bounds.Max.Y);
    }

    [Fact]
    public void CenteredCapsuleBounds_WarmedCallsAllocateNoManagedMemory()
    {
        Vector2d axis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        _ = FixedBoundArea.FromCenteredCapsuleClippedToDomain(
            Vector2d.One,
            axis,
            (Fixed64)7,
            (Fixed64)3);

        long before = GC.GetAllocatedBytesForCurrentThread();
        Fixed64 checksum = Fixed64.Zero;
        for (int index = 0; index < 255; index++)
        {
            FixedBoundArea bounds = FixedBoundArea.FromCenteredCapsuleClippedToDomain(
                Vector2d.One,
                axis,
                (Fixed64)7,
                (Fixed64)3);
            checksum += bounds.Min.X + bounds.Max.Y;
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.NotEqual(Fixed64.Zero, checksum);
        Assert.Equal(0L, allocated);
    }

    private static FixedBoundArea GetExpectedBounds(
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        BigInteger denominator = (BigInteger)2 * FixedMath.ONE_L;
        return FixedBoundArea.FromMinMax(
            new Vector2d(
                GetExpectedCoordinate(center.X, axis.X, axisLength, radius, denominator, true),
                GetExpectedCoordinate(center.Y, axis.Y, axisLength, radius, denominator, true)),
            new Vector2d(
                GetExpectedCoordinate(center.X, axis.X, axisLength, radius, denominator, false),
                GetExpectedCoordinate(center.Y, axis.Y, axisLength, radius, denominator, false)));
    }

    private static Fixed64 GetExpectedCoordinate(
        Fixed64 center,
        Fixed64 axis,
        Fixed64 axisLength,
        Fixed64 radius,
        BigInteger denominator,
        bool minimum)
    {
        BigInteger extent =
            BigInteger.Abs(axis.m_rawValue) * axisLength.m_rawValue
            + (BigInteger)radius.m_rawValue * denominator;
        BigInteger numerator = (BigInteger)center.m_rawValue * denominator;
        numerator = minimum ? numerator - extent : numerator + extent;
        BigInteger raw = minimum
            ? Floor(numerator, denominator)
            : Ceiling(numerator, denominator);
        raw = BigInteger.Max(long.MinValue, BigInteger.Min(long.MaxValue, raw));
        return Fixed64.FromRaw((long)raw);
    }

    private static BigInteger Floor(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        return remainder.Sign < 0 ? quotient - BigInteger.One : quotient;
    }

    private static BigInteger Ceiling(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        return remainder.Sign > 0 ? quotient + BigInteger.One : quotient;
    }
}
