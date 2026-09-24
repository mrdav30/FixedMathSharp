using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCapsuleStrictOverlapTests
{
    private static readonly Vector2d[] Corner =
    {
        new(-1, -1), new(0, -1), new(0, 0), new(-1, 0),
    };

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, false)]
    [InlineData(1, false)]
    public void CapsulePairs_SeparateStrictPenetrationFromClosedTangency(long rawStep, bool strict)
    {
        Vector2d second = new(Fixed64.FromRaw(Fixed64.Two.m_rawValue + rawStep), Fixed64.Zero);
        Assert.Equal(strict, FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            Vector2d.Zero, Vector2d.Forward, Fixed64.Two, Fixed64.One,
            second, Vector2d.Forward, Fixed64.Two, Fixed64.One));
        Assert.Equal(rawStep <= 0, FixedSegment2d.DoCenteredCapsulesOverlap(
            Vector2d.Zero, Vector2d.Forward, Fixed64.Two, Fixed64.One,
            second, Vector2d.Forward, Fixed64.Two, Fixed64.One));
        Assert.Equal(strict, FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One,
            new(second.X, second.Y, Fixed64.Zero), Vector3d.Up, Fixed64.Two, Fixed64.One));
        Assert.Equal(rawStep <= 0, FixedSegment.DoCenteredCapsulesOverlap(
            Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One,
            new(second.X, second.Y, Fixed64.Zero), Vector3d.Up, Fixed64.Two, Fixed64.One));
    }

    [Fact]
    public void CapsulePairs_PreserveHalfRawPositivePenetrationWhenContactDepthRoundsToZero()
    {
        // One endpoint lies at 1/2 raw; the other is 1 raw away. Radius is 1 raw.
        Assert.True(FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            Vector2d.Zero, Vector2d.Right, Fixed64.MinIncrement, Fixed64.MinIncrement,
            new(Fixed64.MinIncrement, Fixed64.Zero), Vector2d.Forward, Fixed64.Zero, Fixed64.Zero));
        Assert.True(FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, Vector3d.Right, Fixed64.MinIncrement, Fixed64.MinIncrement,
            new(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero), Vector3d.Up, Fixed64.Zero, Fixed64.Zero));
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.MinIncrement, Fixed64.MinIncrement,
            new(Fixed64.MinIncrement, Fixed64.Zero), Fixed64.Zero, Vector2d.Forward,
            Fixed64.Zero, Fixed64.Zero, Vector2d.Right, out FixedContactAnchors2d contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
    }

    [Fact]
    public void StrictQueries_PreserveIrrationalSubRawPenetration()
    {
        // 3 - sqrt(2^2 + 2^2) is positive but less than 1/2 raw.
        Vector2d center = new(Fixed64.FromRaw(2), Fixed64.FromRaw(2));
        Assert.True(FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            Vector2d.Zero, Vector2d.Right, Fixed64.Zero, Fixed64.MinIncrement,
            center, Vector2d.Forward, Fixed64.Zero, Fixed64.FromRaw(2)));
        Assert.True(FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, Vector3d.Right, Fixed64.Zero, Fixed64.MinIncrement,
            new(center.X, center.Y, Fixed64.Zero), Vector3d.Up, Fixed64.Zero, Fixed64.FromRaw(2)));
        Assert.True(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            center, Vector2d.Forward, Fixed64.Zero, Fixed64.FromRaw(3),
            Vector2d.Zero, Fixed64.Zero, Corner));
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            center, Vector2d.Forward, Fixed64.Zero, Fixed64.FromRaw(3),
            Vector2d.Zero, Fixed64.Zero, Corner, out _, out Fixed64 depth));
        Assert.Equal(Fixed64.Zero, depth);
    }

    [Fact]
    public void CapsulePairs_RotatedAxesRetainExactEndpointTangency()
    {
        Fixed64 rotation = Fixed64.Pi / (Fixed64)6;
        Vector2d axis = new(-FixedMath.Sin(rotation), FixedMath.Cos(rotation));
        Vector2d center = new Vector2d(-3, 4) + axis;
        Assert.False(FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            center, rotation, Fixed64.Two, (Fixed64)5,
            Vector2d.Zero, -rotation, Fixed64.Zero, Fixed64.Zero));
        Assert.True(FixedSegment2d.DoCenteredCapsulesOverlap(
            center, rotation, Fixed64.Two, (Fixed64)5,
            Vector2d.Zero, -rotation, Fixed64.Zero, Fixed64.Zero));
        Assert.True(FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            center, rotation, Fixed64.Two, (Fixed64)5 + Fixed64.MinIncrement,
            Vector2d.Zero, -rotation, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void CapsulePairs_ZeroRadiusCrossingsHaveNoStrictRadialOverlap()
    {
        Assert.False(FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            Vector2d.Zero, Vector2d.Right, Fixed64.Two, Fixed64.Zero,
            Vector2d.Zero, Vector2d.Forward, Fixed64.Two, Fixed64.Zero));
        Assert.False(FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, Vector3d.Right, Fixed64.Two, Fixed64.Zero,
            Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.Zero));
    }

    [Theory]
    [InlineData(-2, true)]
    [InlineData(-1, false)]
    [InlineData(0, false)]
    public void CapsulePairs_CompareUnsaturatedRadiusSumAcrossEntireDomain(long secondRawStep, bool strict)
    {
        Vector2d first = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d second = new(Fixed64.FromRaw(long.MaxValue + secondRawStep), Fixed64.Zero);
        Assert.Equal(strict, FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            first, Vector2d.Forward, Fixed64.Zero, Fixed64.MaxValue,
            second, Vector2d.Forward, Fixed64.Zero, Fixed64.MaxValue));
        Assert.Equal(strict, FixedSegment.DoCenteredCapsulesOverlapStrict(
            new(first.X, Fixed64.Zero, Fixed64.Zero), Vector3d.Up, Fixed64.Zero, Fixed64.MaxValue,
            new(second.X, Fixed64.Zero, Fixed64.Zero), Vector3d.Up, Fixed64.Zero, Fixed64.MaxValue));
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(0, false)]
    [InlineData(1, true)]
    public void ConvexCorner_ClassifiesExactRadialSignBeforeDepthRounding(long radiusStep, bool strict)
    {
        Fixed64 radius = Fixed64.FromRaw(((Fixed64)5).m_rawValue + radiusStep);
        Assert.Equal(strict, FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            new(3, 4), Vector2d.Forward, Fixed64.Zero, radius,
            Vector2d.Zero, Fixed64.Zero, Corner));
        Assert.Equal(radiusStep >= 0, FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            new(3, 4), Vector2d.Forward, Fixed64.Zero, radius,
            Vector2d.Zero, Fixed64.Zero, Corner, out _, out Fixed64 depth));
        Assert.Equal(Math.Max(radiusStep, 0), depth.m_rawValue);
    }

    [Fact]
    public void ConvexFace_PreservesHalfRawPenetrationWhenDepthRoundsToZero()
    {
        Vector2d center = new(Fixed64.MinIncrement, -Fixed64.Half);
        Assert.True(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            center, Vector2d.Right, Fixed64.MinIncrement, Fixed64.MinIncrement,
            Vector2d.Zero, Fixed64.Zero, Corner));
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            center, Vector2d.Right, Fixed64.MinIncrement, Fixed64.MinIncrement,
            Vector2d.Zero, Fixed64.Zero, Corner, out _, out Fixed64 depth));
        Assert.Equal(Fixed64.Zero, depth);
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, false)]
    [InlineData(1, false)]
    public void ZeroRadiusPoint_RequiresPolygonInterior(long xRaw, bool strict)
    {
        Assert.Equal(strict, FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            new(Fixed64.FromRaw(xRaw), -Fixed64.Half), Vector2d.Forward,
            Fixed64.Zero, Fixed64.Zero, Vector2d.Zero, Fixed64.Zero, Corner));
    }

    [Fact]
    public void ZeroRadiusAxis_CrossingInteriorDiffersFromRunningAlongBoundary()
    {
        Assert.True(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            new(-Fixed64.Half, -Fixed64.Half), Vector2d.Right,
            Fixed64.Two, Fixed64.Zero, Vector2d.Zero, Fixed64.Zero, Corner));
        Assert.False(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            new(-Fixed64.Half, Fixed64.Zero), Vector2d.Right,
            Fixed64.Two, Fixed64.Zero, Vector2d.Zero, Fixed64.Zero, Corner));
    }

    [Fact]
    public void CollinearBoundary_UsesPositiveSeparatingTranslation()
    {
        Vector2d[] boundary = { new(-1, 0), new(1, 0), new(0, 0) };
        Assert.True(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            Vector2d.Zero, Vector2d.Forward, Fixed64.Two, Fixed64.Zero,
            Vector2d.Zero, Fixed64.Zero, boundary));
        Assert.False(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            Vector2d.Zero, Vector2d.Right, Fixed64.Two, Fixed64.Zero,
            Vector2d.Zero, Fixed64.Zero, boundary));
        Assert.False(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            Vector2d.Zero, Vector2d.Forward, Fixed64.Zero, Fixed64.Zero,
            Vector2d.Zero, Fixed64.Zero, boundary));
    }

    [Fact]
    public void ConvexClassification_SupportsUnrepresentableVerticesAndDegenerateEdges()
    {
        Vector2d origin = new(Fixed64.MaxValue, Fixed64.MaxValue);
        Vector2d[] polygon = { new(0, 0), new(1, 0), new(1, 0), new(1, 1), new(0, 1) };
        Vector2d center = origin - new Vector2d(3, 4);
        Assert.False(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            center, Vector2d.Forward, Fixed64.Zero, (Fixed64)5,
            origin, Fixed64.Zero, polygon));
        Assert.True(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            center, Vector2d.Forward, Fixed64.Zero, (Fixed64)5 + Fixed64.MinIncrement,
            origin, Fixed64.Zero, polygon));
        Assert.False(FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            Vector2d.Zero, Vector2d.Forward, Fixed64.Zero, Fixed64.One,
            Vector2d.Zero, Fixed64.Zero, new Vector2d[3]));
    }

    [Fact]
    public void CapsulePairs_DegenerateAxesMatchIndependentBigIntegerRadialOracle()
    {
        System.Random random = new(24092026);
        for (int i = 0; i < 256; i++)
        {
            long x = random.NextInt64();
            long y = random.NextInt64();
            long z = random.NextInt64();
            long radiusA = random.NextInt64();
            long radiusB = random.NextInt64();
            BigInteger squaredRadius = BigInteger.Pow((BigInteger)radiusA + radiusB, 2);
            BigInteger distance2 = (BigInteger)x * x + (BigInteger)y * y;
            Assert.Equal(distance2 < squaredRadius, FixedSegment2d.DoCenteredCapsulesOverlapStrict(
                Vector2d.Zero, Vector2d.Right, Fixed64.Zero, Fixed64.FromRaw(radiusA),
                new(Fixed64.FromRaw(x), Fixed64.FromRaw(y)), Vector2d.Forward, Fixed64.Zero, Fixed64.FromRaw(radiusB)));
            Assert.Equal(distance2 + (BigInteger)z * z < squaredRadius, FixedSegment.DoCenteredCapsulesOverlapStrict(
                Vector3d.Zero, Vector3d.Right, Fixed64.Zero, Fixed64.FromRaw(radiusA),
                new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z)), Vector3d.Up, Fixed64.Zero, Fixed64.FromRaw(radiusB)));
        }
    }

    [Fact]
    public void StrictQueries_RejectInvalidAuthoredGeometry()
    {
        Assert.Throws<ArgumentOutOfRangeException>("firstRadius", () => FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.One, -Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>("secondRadius", () => FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.One, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.One, -Fixed64.One));
        Assert.Throws<ArgumentException>(() => FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            Vector2d.Zero, Vector2d.Zero, Fixed64.One, Fixed64.One,
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One,
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            Vector2d.Zero, Fixed64.Zero, -Fixed64.One, Fixed64.One,
            Vector2d.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentException>(() => FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            Vector2d.Zero, Vector2d.Forward, Fixed64.Zero, Fixed64.One,
            Vector2d.Zero, Fixed64.Zero, Array.Empty<Vector2d>()));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
            Vector2d.Zero, Vector2d.Forward, Fixed64.Zero, -Fixed64.One,
            Vector2d.Zero, Fixed64.Zero, Corner));
    }

    [Theory]
    [InlineData(-1, 0)]
    [InlineData(0, -1)]
    public void CapsulePairs_RejectEitherNegativeRadius(int first, int second)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.DoCenteredCapsulesOverlapStrict(
            Vector2d.Zero, Vector2d.Forward, Fixed64.Zero, (Fixed64)first,
            Vector2d.Zero, Vector2d.Forward, Fixed64.Zero, (Fixed64)second));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, (Fixed64)first,
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, (Fixed64)second));
    }

    [Fact]
    public void StrictQueries_DoNotAllocateAfterWarmup()
    {
        bool correct = true;
        long bytes = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int i = 0; i < 32; i++)
            {
                correct &= !FixedSegment2d.DoesCenteredCapsulePenetrateConvex(
                    new(3, 4), Vector2d.Forward, Fixed64.Zero, (Fixed64)5,
                    Vector2d.Zero, Fixed64.Zero, Corner);
                correct &= FixedSegment2d.DoCenteredCapsulesOverlapStrict(
                    Vector2d.Zero, Vector2d.Forward, Fixed64.Two, Fixed64.One,
                    Vector2d.Right, Vector2d.Forward, Fixed64.Two, Fixed64.One);
                correct &= FixedSegment.DoCenteredCapsulesOverlapStrict(
                    Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One,
                    Vector3d.Right, Vector3d.Up, Fixed64.Two, Fixed64.One);
            }
        });
        Assert.True(correct);
        Assert.Equal(0, bytes);
    }
}
