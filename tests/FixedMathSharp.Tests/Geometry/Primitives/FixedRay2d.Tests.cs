using FixedMathSharp.Geometry;
using MemoryPack;
using System;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedRay2dTests
{
    [Fact]
    public void Constructor_AssignsPositionAndDirectionWithoutNormalizing()
    {
        var ray = new FixedRay2d(new Vector2d(1, 2), new Vector2d(2, 0));

        Assert.Equal(new Vector2d(1, 2), ray.Position);
        Assert.Equal(new Vector2d(2, 0), ray.Direction);
        Assert.Equal(new Vector2d(7, 2), ray.GetPoint(new Fixed64(3)));
    }

    [Fact]
    public void Intersects_Area_ReturnsNearestForwardParameter()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));
        var ray = new FixedRay2d(new Vector2d(-5, 0), Vector2d.Right);

        Fixed64? hit = ray.Intersects(area);

        Assert.Equal(new Fixed64(4), hit);
    }

    [Fact]
    public void Intersects_Area_HandlesNegativeDirectionAndDiagonalNonUnitDirection()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));
        var negative = new FixedRay2d(new Vector2d(5, 0), Vector2d.Left);
        var diagonal = new FixedRay2d(new Vector2d(-5, -5), new Vector2d(2, 2));

        Assert.Equal(new Fixed64(4), negative.Intersects(area));
        Assert.Equal(new Fixed64(2), diagonal.Intersects(area));
    }

    [Theory]
    [InlineData(1L)]
    [InlineData(-1L)]
    public void Intersects_Area_TreatsOneRawDirectionAsMotion(long rawDirection)
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 positionX = rawDirection > 0 ? -oneRaw : Fixed64.One + oneRaw;
        var area = FixedBoundArea.FromMinMax(Vector2d.Zero, Vector2d.One);
        var ray = new FixedRay2d(
            new Vector2d(positionX, Fixed64.Zero),
            new Vector2d(Fixed64.FromRaw(rawDirection), Fixed64.One));

        Assert.Equal(Fixed64.One, ray.Intersects(area));
    }

    [Fact]
    public void Intersects_Area_ZeroDirectionOutsideSlabReturnsNull()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        var area = FixedBoundArea.FromMinMax(Vector2d.Zero, Vector2d.One);
        var ray = new FixedRay2d(
            new Vector2d(-oneRaw, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One));

        Assert.Null(ray.Intersects(area));
    }

    [Fact]
    public void Intersects_Area_ReturnsZeroWhenRayStartsInsideOrOnBoundary()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));

        Assert.Equal(Fixed64.Zero, new FixedRay2d(Vector2d.Zero, Vector2d.Right).Intersects(area));
        Assert.Equal(Fixed64.Zero, new FixedRay2d(new Vector2d(-1, 0), Vector2d.Left).Intersects(area));
    }

    [Fact]
    public void Intersects_Area_IsBoundaryInclusiveForGrazingRay()
    {
        var area = FixedBoundArea.FromMinMax(Vector2d.Zero, new Vector2d(4, 4));
        var alongTopEdge = new FixedRay2d(new Vector2d(-2, 4), Vector2d.Right);
        var alongRightEdge = new FixedRay2d(new Vector2d(4, -2), Vector2d.Forward);

        Assert.Equal(new Fixed64(2), alongTopEdge.Intersects(area));
        Assert.Equal(new Fixed64(2), alongRightEdge.Intersects(area));
    }

    [Fact]
    public void Intersects_Area_ReturnsNullWhenParallelOutsideBehindOrZeroOutside()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));

        Assert.Null(new FixedRay2d(new Vector2d(-5, 2), Vector2d.Right).Intersects(area));
        Assert.Null(new FixedRay2d(new Vector2d(5, 0), Vector2d.Right).Intersects(area));
        Assert.Null(new FixedRay2d(new Vector2d(2, 2), Vector2d.Zero).Intersects(area));
        Assert.Equal(Fixed64.Zero, new FixedRay2d(Vector2d.Zero, Vector2d.Zero).Intersects(area));
    }

    [Fact]
    public void Intersects_Circle_ReturnsNearestForwardParameter()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var ray = new FixedRay2d(new Vector2d(-5, 0), Vector2d.Right);

        Fixed64? hit = ray.Intersects(circle);

        Assert.Equal(new Fixed64(4), hit);
    }

    [Fact]
    public void Intersects_Circle_HandlesNonUnitDirectionAndTangency()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var nonUnit = new FixedRay2d(new Vector2d(-10, 0), new Vector2d(2, 0));
        var tangent = new FixedRay2d(new Vector2d(-5, 1), Vector2d.Right);

        Assert.Equal(Fixed64.FromDouble(4.5), nonUnit.Intersects(circle));
        Assert.Equal(new Fixed64(5), tangent.Intersects(circle));
    }

    [Fact]
    public void Intersects_Circle_OrdersExtremeRangeQuadraticWithoutSaturation()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var crossing = new FixedRay2d(
            new Vector2d((Fixed64)(-100_000), Fixed64.Zero),
            new Vector2d((Fixed64)200_000, Fixed64.Zero));
        var miss = new FixedRay2d(
            new Vector2d((Fixed64)(-100_000), (Fixed64)2),
            new Vector2d((Fixed64)200_000, Fixed64.Zero));

        Assert.Equal(Fixed64.FromFraction(99_999, 200_000), crossing.Intersects(circle));
        Assert.Null(miss.Intersects(circle));
    }

    [Fact]
    public void Intersects_Circle_TreatsOneRawDirectionAsMotion()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var ray = new FixedRay2d(
            new Vector2d(-Fixed64.One - oneRaw, Fixed64.Zero),
            new Vector2d(oneRaw, Fixed64.Zero));

        Assert.Equal(Fixed64.One, ray.Intersects(circle));
    }

    [Fact]
    public void Intersects_Circle_OrdersUnrepresentableOffsetWithinBound()
    {
        var circle = new FixedBoundCircle(new Vector2d(-1, 0), Fixed64.One);
        var ray = new FixedRay2d(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(-Fixed64.MaxValue, Fixed64.Zero));

        Assert.Equal(Fixed64.One, ray.Intersects(circle, Fixed64.One));
        Assert.Null(ray.Intersects(circle, Fixed64.One - Fixed64.FromRaw(1L)));
    }

    [Fact]
    public void Intersects_Circle_ExpandsRadiusWithoutSaturatingTheSum()
    {
        var circle = new FixedBoundCircle(new Vector2d(-1, 0), Fixed64.MaxValue);
        var ray = new FixedRay2d(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Vector2d.Zero);

        Assert.Null(ray.Intersects(circle, Fixed64.Zero));
        Assert.Equal(
            Fixed64.Zero,
            ray.Intersects(circle, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            ray.Intersects(circle, -Fixed64.One, Fixed64.One));
    }

    [Fact]
    public void Intersects_Circle_BoundedExpansionCoversFastAndInvalidBounds()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var ray = new FixedRay2d(new Vector2d(-5, 0), Vector2d.Right);

        Assert.Equal((Fixed64)3, ray.Intersects(circle, Fixed64.One, (Fixed64)3));
        Assert.Null(ray.Intersects(circle, Fixed64.One, (Fixed64)2));
        Assert.Equal((Fixed64)4, ray.Intersects(circle, Fixed64.Zero, (Fixed64)4));
        Assert.Null(ray.Intersects(circle, -Fixed64.One));
        Assert.Null(ray.Intersects(circle, Fixed64.One, -Fixed64.One));
    }

    [Fact]
    public void Intersects_Circle_RoundsWideRootsHalfToEven()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var circle = new FixedBoundCircle(Vector2d.Zero, (Fixed64)65);
        var evenLower = new FixedRay2d(
            new Vector2d(-(Fixed64)65 - oneRaw, Fixed64.Zero),
            new Vector2d(2, 0));
        var oddLower = new FixedRay2d(
            new Vector2d(-(Fixed64)65 - Fixed64.FromRaw(3L), Fixed64.Zero),
            new Vector2d(2, 0));

        Assert.Equal(Fixed64.Zero, evenLower.Intersects(circle));
        Assert.Equal(Fixed64.FromRaw(2L), oddLower.Intersects(circle));
    }

    [Fact]
    public void Intersects_Circle_RoundsWideRootsOnEitherSideOfMidpoint()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, (Fixed64)65);
        var belowMidpoint = new FixedRay2d(
            new Vector2d(-(Fixed64)65 - Fixed64.FromRaw(1L), Fixed64.Zero),
            new Vector2d(3, 0));
        var aboveMidpoint = new FixedRay2d(
            new Vector2d(-(Fixed64)65 - Fixed64.FromRaw(2L), Fixed64.Zero),
            new Vector2d(3, 0));

        Assert.Equal(Fixed64.Zero, belowMidpoint.Intersects(circle));
        Assert.Equal(Fixed64.FromRaw(1L), aboveMidpoint.Intersects(circle));
    }

    [Fact]
    public void Intersects_Circle_RoundsWideTangentAndSubRawParameters()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, (Fixed64)65);
        var tangent = new FixedRay2d(
            new Vector2d(-100, 65),
            new Vector2d(199, 0));
        var subRaw = new FixedRay2d(
            new Vector2d(-Fixed64.FromRaw(1L), Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero));

        Assert.Equal(Fixed64.FromFraction(100, 199), tangent.Intersects(circle));
        Assert.Equal(Fixed64.Zero, subRaw.Intersects(new FixedBoundCircle(Vector2d.Zero, Fixed64.Zero)));
    }

    [Fact]
    public void Intersects_Circle_PreservesExactRootAcrossEquivalentScale()
    {
        const long positionX = -21_474_923_131L;
        const long positionY = 4_294_881_327L;
        const long directionX = 4_295_022_249L;
        const long directionY = 2_228L;
        const long radius = 8_589_994_090L;
        var ray = new FixedRay2d(
            new Vector2d(Fixed64.FromRaw(positionX), Fixed64.FromRaw(positionY)),
            new Vector2d(Fixed64.FromRaw(directionX), Fixed64.FromRaw(directionY)));
        var scaledRay = new FixedRay2d(
            new Vector2d(Fixed64.FromRaw(positionX * 100), Fixed64.FromRaw(positionY * 100)),
            new Vector2d(Fixed64.FromRaw(directionX * 100), Fixed64.FromRaw(directionY * 100)));

        Fixed64? hit = ray.Intersects(new FixedBoundCircle(Vector2d.Zero, Fixed64.FromRaw(radius)));
        Fixed64? scaledHit = scaledRay.Intersects(
            new FixedBoundCircle(Vector2d.Zero, Fixed64.FromRaw(radius * 100)));

        Assert.Equal(Fixed64.FromRaw(14_035_527_845L), hit);
        Assert.Equal(hit, scaledHit);
    }

    [Fact]
    public void Intersects_Circle_RejectsExactRootBeyondRepresentableBound()
    {
        var ray = new FixedRay2d(
            new Vector2d(Fixed64.FromRaw(-17_179_869_185L), Fixed64.Zero),
            new Vector2d(Fixed64.FromRaw(17_179_869_184L), Fixed64.Zero));

        Assert.Null(ray.Intersects(
            new FixedBoundCircle(Vector2d.Zero, Fixed64.Zero),
            Fixed64.One));
    }

    [Fact]
    public void Intersects_Circle_RefinesFloorSquareRootSeedExactly()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var ray = new FixedRay2d(
            new Vector2d(Fixed64.FromRaw(-20L), Fixed64.FromRaw(-20L)),
            new Vector2d(oneRaw, oneRaw));

        Fixed64? hit = ray.Intersects(
            new FixedBoundCircle(Vector2d.Zero, oneRaw));

        Assert.Equal(Fixed64.FromRaw(82_862_345_420L), hit);
    }

    [Fact]
    public void Intersects_Circle_ReturnsZeroWhenRayStartsInsideOrOnBoundary()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, new Fixed64(2));

        Assert.Equal(Fixed64.Zero, new FixedRay2d(Vector2d.Zero, Vector2d.Right).Intersects(circle));
        Assert.Equal(Fixed64.Zero, new FixedRay2d(new Vector2d(2, 0), Vector2d.Right).Intersects(circle));
    }

    [Fact]
    public void Intersects_Circle_ReturnsNullWhenMissingPointingAwayOrZeroOutside()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);

        Assert.Null(new FixedRay2d(new Vector2d(-5, 2), Vector2d.Right).Intersects(circle));
        Assert.Null(new FixedRay2d(new Vector2d(-5, 0), Vector2d.Left).Intersects(circle));
        Assert.Null(new FixedRay2d(new Vector2d(2, 0), Vector2d.Zero).Intersects(circle));
        Assert.Equal(Fixed64.Zero, new FixedRay2d(Vector2d.Zero, Vector2d.Zero).Intersects(circle));
    }

    [Fact]
    public void TryGetIntersectionInterval_CircleReturnsOrderedClippedParameters()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var ray = new FixedRay2d(new Vector2d(-10, 0), new Vector2d(2, 0));

        Assert.True(ray.TryGetIntersectionInterval(circle, (Fixed64)10, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(9, 2), entry);
        Assert.Equal(Fixed64.FromFraction(11, 2), exit);

        Assert.True(ray.TryGetIntersectionInterval(circle, (Fixed64)5, out entry, out exit));
        Assert.Equal(Fixed64.FromFraction(9, 2), entry);
        Assert.Equal((Fixed64)5, exit);

        Assert.False(ray.TryGetIntersectionInterval(circle, (Fixed64)4, out entry, out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Zero, exit);
    }

    [Fact]
    public void TryGetIntersectionInterval_CirclePreservesClosedBoundaryAndZeroDirectionSemantics()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);

        Assert.True(new FixedRay2d(Vector2d.Zero, Vector2d.Zero)
            .TryGetIntersectionInterval(circle, (Fixed64)3, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal((Fixed64)3, exit);

        Assert.True(new FixedRay2d(Vector2d.Right, Vector2d.Right)
            .TryGetIntersectionInterval(circle, (Fixed64)3, out entry, out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Zero, exit);

        Assert.True(new FixedRay2d(Vector2d.Right, Vector2d.Left)
            .TryGetIntersectionInterval(circle, (Fixed64)3, out entry, out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal((Fixed64)2, exit);

        Assert.False(new FixedRay2d((Fixed64)2 * Vector2d.Right, Vector2d.Zero)
            .TryGetIntersectionInterval(circle, (Fixed64)3, out _, out _));
        Assert.False(new FixedRay2d(Vector2d.Zero, Vector2d.Right)
            .TryGetIntersectionInterval(circle, -Fixed64.One, out _, out _));
    }

    [Fact]
    public void TryGetIntersectionInterval_CircleHandlesTangencyAndExactBoundBeforeRounding()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var tangent = new FixedRay2d(new Vector2d(-5, 1), Vector2d.Right);

        Assert.True(tangent.TryGetIntersectionInterval(circle, (Fixed64)10, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal((Fixed64)5, entry);
        Assert.Equal(entry, exit);

        var justBeyond = new FixedRay2d(
            new Vector2d(Fixed64.FromRaw(-17_179_869_185L), Fixed64.Zero),
            new Vector2d(Fixed64.FromRaw(17_179_869_184L), Fixed64.Zero));
        var justInside = new FixedRay2d(
            new Vector2d(Fixed64.FromRaw(-17_179_869_183L), Fixed64.Zero),
            new Vector2d(Fixed64.FromRaw(17_179_869_184L), Fixed64.Zero));
        var point = new FixedBoundCircle(Vector2d.Zero, Fixed64.Zero);

        Assert.False(justBeyond.TryGetIntersectionInterval(point, Fixed64.One, out _, out _));
        Assert.True(justInside.TryGetIntersectionInterval(point, Fixed64.One, out entry, out exit));
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal(entry, exit);
    }

    [Fact]
    public void TryGetIntersectionInterval_CircleExpandsRadiusWithoutSaturating()
    {
        var circle = new FixedBoundCircle(new Vector2d(-1, 0), Fixed64.MaxValue);
        var ray = new FixedRay2d(new Vector2d(Fixed64.MaxValue, Fixed64.Zero), Vector2d.Zero);

        Assert.False(ray.TryGetIntersectionInterval(circle, Fixed64.One, out _, out _));
        Assert.True(ray.TryGetIntersectionInterval(
            circle,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.One, exit);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            ray.TryGetIntersectionInterval(circle, -Fixed64.One, Fixed64.One, out _, out _));
    }

    [Fact]
    public void TryGetIntersectionInterval_CircleRoundsBothRootsToEven()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var circle = new FixedBoundCircle(Vector2d.Zero, (Fixed64)65);
        var evenCandidates = new FixedRay2d(
            new Vector2d(-(Fixed64)65 - oneRaw, Fixed64.Zero),
            new Vector2d(2, 0));
        var oddCandidates = new FixedRay2d(
            new Vector2d(-(Fixed64)65 - Fixed64.FromRaw(3L), Fixed64.Zero),
            new Vector2d(2, 0));

        Assert.True(evenCandidates.TryGetIntersectionInterval(circle, (Fixed64)100, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal((Fixed64)65, exit);

        Assert.True(oddCandidates.TryGetIntersectionInterval(circle, (Fixed64)100, out entry, out exit));
        Assert.Equal(Fixed64.FromRaw(2L), entry);
        Assert.Equal((Fixed64)65 + Fixed64.FromRaw(2L), exit);
    }

    [Fact]
    public void TryGetIntersectionInterval_CirclePreservesScaleAndSubRawOverlap()
    {
        var scaledCircle = new FixedBoundCircle(Vector2d.Zero, (Fixed64)100_000);
        var scaledRay = new FixedRay2d(
            new Vector2d(-1_000_000, 0),
            new Vector2d(200_000, 0));

        Assert.True(scaledRay.TryGetIntersectionInterval(
            scaledCircle,
            (Fixed64)10,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(9, 2), entry);
        Assert.Equal(Fixed64.FromFraction(11, 2), exit);

        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var subRawCircle = new FixedBoundCircle(new Vector2d(oneRaw, Fixed64.Zero), oneRaw);
        var subRawRay = new FixedRay2d(Vector2d.Zero, new Vector2d(Fixed64.MaxValue, Fixed64.Zero));

        Assert.True(subRawRay.TryGetIntersectionInterval(
            subRawCircle,
            Fixed64.One,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Zero, exit);

        var fractionalSubRawCircle = new FixedBoundCircle(
            new Vector2d(Fixed64.Zero, Fixed64.FromRaw(1L)),
            Fixed64.FromRaw(2L));
        var fractionalSubRawRay = new FixedRay2d(
            new Vector2d(Fixed64.FromRaw(-6L), Fixed64.Zero),
            new Vector2d(8, 0));

        Assert.True(fractionalSubRawRay.TryGetIntersectionInterval(
            fractionalSubRawCircle,
            Fixed64.One,
            out entry,
            out exit));
        Assert.Equal(Fixed64.FromRaw(1L), entry);
        Assert.Equal(entry, exit);
    }

    [Fact]
    public void TryGetIntersectionInterval_CircleCorrectsIrrationalUpperRoot()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var circle = new FixedBoundCircle(new Vector2d(Fixed64.Zero, oneRaw), Fixed64.FromRaw(2L));
        var ray = new FixedRay2d(new Vector2d(Fixed64.FromRaw(-3L), Fixed64.Zero), new Vector2d(oneRaw, Fixed64.Zero));

        Assert.True(ray.TryGetIntersectionInterval(circle, (Fixed64)10, out Fixed64 entry, out Fixed64 exit));
        // The exact roots are 3 +/- sqrt(3); these are their independently
        // rounded nearest-even Q32.32 representations.
        Assert.Equal(Fixed64.FromRaw(5_445_800_314L), entry);
        Assert.Equal(Fixed64.FromRaw(20_324_003_462L), exit);
    }

    [Fact]
    public void TryGetIntersectionInterval_CircleRejectsUnavailableForwardIntervals()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);

        Assert.False(new FixedRay2d(new Vector2d(2, 0), Vector2d.Right)
            .TryGetIntersectionInterval(circle, (Fixed64)10, out _, out _));
        Assert.False(new FixedRay2d(new Vector2d(-2, 0), Vector2d.Right)
            .TryGetIntersectionInterval(circle, Fixed64.Zero, out _, out _));
        Assert.False(new FixedRay2d(new Vector2d(-2, 2), Vector2d.Right)
            .TryGetIntersectionInterval(circle, (Fixed64)10, out _, out _));
    }

    [Fact]
    public void EqualityDeconstructAndHashCode_UsePositionAndDirection()
    {
        var ray = new FixedRay2d(new Vector2d(1, 2), Vector2d.Forward);
        var same = new FixedRay2d(new Vector2d(1, 2), Vector2d.Forward);
        var different = new FixedRay2d(new Vector2d(1, 2), Vector2d.Right);
        var differentPosition = new FixedRay2d(new Vector2d(2, 2), Vector2d.Forward);

        ray.Deconstruct(out Vector2d position, out Vector2d direction);

        Assert.Equal(new Vector2d(1, 2), position);
        Assert.Equal(Vector2d.Forward, direction);
        Assert.True(ray == same);
        Assert.True(ray.Equals((object)same));
        Assert.False(ray != same);
        Assert.Equal(ray.GetHashCode(), same.GetHashCode());
        Assert.NotEqual(ray, different);
        Assert.False(ray == different);
        Assert.True(ray != different);
        Assert.False(ray.Equals(differentPosition));
        Assert.False(ray.Equals("not a ray"));
    }

    [Fact]
    public void JsonSerialization_RoundTripsState()
    {
        var ray = new FixedRay2d(new Vector2d(1, 2), Vector2d.Forward);

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(ray);
        var roundTrip = JsonSerializer.Deserialize<FixedRay2d>(json);

        Assert.Equal(ray, roundTrip);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_RoundTripsState()
    {
        var ray = new FixedRay2d(new Vector2d(1, 2), Vector2d.Forward);

        byte[] bytes = MemoryPackSerializer.Serialize(ray);
        var roundTrip = MemoryPackSerializer.Deserialize<FixedRay2d>(bytes);

        Assert.Equal(ray, roundTrip);
    }
#endif
}
