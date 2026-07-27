using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    private static readonly FixedSegment UnitCylinderAxis = new(
        new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero),
        new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

    [Fact]
    public void FiniteCylinder_OrdinaryCrossing_ReturnsClosedQueryInterval()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-1), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 3), entry);
        Assert.Equal(Fixed64.FromFraction(2, 3), exit);
    }

    [Fact]
    public void FiniteCylinder_AdvancedClassification_DoesNotContainOutsideStartWhoseEntryRoundsToZero()
    {
        Fixed64 radius = Fixed64.Half;
        var query = new FixedSegment(
            new Vector3d(radius + Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            radius,
            Fixed64.Zero,
            out Fixed64 entry,
            out _,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void FiniteCylinder_AdvancedClassification_ReportsBoundaryEndAsNotStrictlyContained()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal(entry, exit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void FiniteCylinder_AdvancedClassification_ReportsBoundaryStartAsContained()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(entry, exit);
        Assert.True(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void FiniteCylinder_AxialClipRejectsEntryRootAndKeepsExitRoot()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.FromFraction(-5, 2), Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.FromFraction(3, 2), Fixed64.Zero));
        var axis = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(5, 8), entry);
        Assert.Equal(Fixed64.FromFraction(3, 4), exit);
    }

    [Theory]
    [InlineData(1L, 3L, 5L)]
    [InlineData(3L, 17L, 1L)]
    [InlineData(101L, 7L, 33L)]
    public void FiniteCylinder_ArbitraryRawFullWidthRootsMatchBigIntegerOracle(
        long axisOffsetRaw,
        long queryOffsetRaw,
        long radiusOffsetRaw)
    {
        Fixed64 scale = (Fixed64)100_000_000;
        Fixed64 twiceScale = scale * 2;
        var query = new FixedSegment(
            new Vector3d(-twiceScale + Fixed64.FromRaw(queryOffsetRaw), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(twiceScale + Fixed64.FromRaw(queryOffsetRaw + 2), Fixed64.Zero, Fixed64.Zero));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, -scale, -scale),
            new Vector3d(Fixed64.Zero, scale, scale + Fixed64.FromRaw(axisOffsetRaw)));
        Fixed64 radius = (scale * Fixed64.Half) + Fixed64.FromRaw(radiusOffsetRaw);
        (Fixed64 expectedEntry, Fixed64 expectedExit) = GetRadialIntervalOracle(query, axis, radius);

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            radius,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);

    }

    [Fact]
    public void FiniteCylinder_SubRawFullWidthIntervalsMatchBigIntegerOracle()
    {
        Fixed64 scale = (Fixed64)100_000_000;
        foreach (long offsetRaw in new[] { 1L, 3L, 5L })
        {
            var query = new FixedSegment(
                new Vector3d(-scale + Fixed64.FromRaw(offsetRaw), Fixed64.Zero, Fixed64.Zero),
                new Vector3d(scale + Fixed64.FromRaw(offsetRaw + 2), Fixed64.Zero, Fixed64.Zero));
            var axis = new FixedSegment(
                new Vector3d(Fixed64.Zero, -scale, -scale - Fixed64.FromRaw(offsetRaw)),
                new Vector3d(Fixed64.Zero, scale, scale + Fixed64.FromRaw(offsetRaw)));
            Fixed64 radius = Fixed64.FromRaw(offsetRaw);
            (Fixed64 expectedEntry, Fixed64 expectedExit) = GetRadialIntervalOracle(query, axis, radius);

            Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
                axis,
                radius,
                Fixed64.Zero,
                out Fixed64 entry,
                out Fixed64 exit));
            Assert.Equal(expectedEntry, entry);
            Assert.Equal(expectedExit, exit);
        }
    }

    [Fact]
    public void FiniteCylinder_WideAdjacentRootCorrectionCrossesBothMidpointSides()
    {
        // This construction keeps both endpoints axially valid while A = 2^180
        // and an odd C place the exact radial roots on opposite midpoint sides.
        var axis = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(
                Fixed64.FromRaw(4_611_404_548_819_353_601L),
                Fixed64.FromRaw(65_535L),
                Fixed64.Zero));
        var forward = new FixedSegment(
            new Vector3d(
                Fixed64.FromRaw(1_729_201_941_163_999_233L),
                Fixed64.FromRaw(4_611_404_547_879_895_040L),
                Fixed64.Zero),
            new Vector3d(
                Fixed64.FromRaw(3_098_281_934_401_238_017L),
                Fixed64.FromRaw(4_611_404_547_611_462_656L),
                Fixed64.Zero));
        Fixed64 radius = Fixed64.FromRaw(4_611_404_547_745_644_545L);
        (Fixed64 expectedEntry, Fixed64 expectedExit) = GetRadialIntervalOracle(forward, axis, radius);
        expectedEntry = FixedMath.Clamp01(expectedEntry);
        expectedExit = FixedMath.Clamp01(expectedExit);

        Assert.True(forward.TryGetFiniteCylinderIntersectionInterval(
            axis, radius, Fixed64.Zero, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);
        Assert.Equal(Fixed64.FromRaw(2_147_483_649L), entry);

        var reverse = new FixedSegment(forward.End, forward.Start);
        (expectedEntry, expectedExit) = GetRadialIntervalOracle(reverse, axis, radius);
        expectedEntry = FixedMath.Clamp01(expectedEntry);
        expectedExit = FixedMath.Clamp01(expectedExit);
        Assert.True(reverse.TryGetFiniteCylinderIntersectionInterval(
            axis, radius, Fixed64.Zero, out entry, out exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);
        Assert.Equal(Fixed64.FromRaw(2_147_483_647L), exit);
    }

    [Fact]
    public void FiniteCylinder_FullWidthEndpointContainmentClipsWideRoots()
    {
        Fixed64 scale = (Fixed64)100_000_000;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, -scale, -scale),
            new Vector3d(Fixed64.Zero, scale, scale + oneRaw));
        Fixed64 radius = (scale * Fixed64.Half) + oneRaw;
        var outward = new FixedSegment(
            Vector3d.Zero,
            new Vector3d((scale * 2) + oneRaw, Fixed64.Zero, Fixed64.Zero));
        var inward = new FixedSegment(outward.End, outward.Start);

        foreach (FixedSegment query in new[] { outward, inward })
        {
            (Fixed64 expectedEntry, Fixed64 expectedExit) = GetRadialIntervalOracle(query, axis, radius);
            expectedEntry = FixedMath.Clamp01(expectedEntry);
            expectedExit = FixedMath.Clamp01(expectedExit);
            Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
                axis,
                radius,
                Fixed64.Zero,
                out Fixed64 entry,
                out Fixed64 exit));
            Assert.Equal(expectedEntry, entry);
            Assert.Equal(expectedExit, exit);

        }
    }

    [Fact]
    public void FiniteCylinder_FullWidthInteriorMinimumRejectsWithoutSaturatingDiscriminant()
    {
        Fixed64 scale = (Fixed64)100_000_000;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        var query = new FixedSegment(
            new Vector3d((-scale * 2) + oneRaw, scale, -scale),
            new Vector3d((scale * 2) + Fixed64.FromRaw(3), scale, -scale));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, -scale, -scale),
            new Vector3d(Fixed64.Zero, scale, scale + oneRaw));
        Fixed64 radius = scale * Fixed64.Half;

        Assert.False(query.TryGetFiniteCylinderIntersectionInterval(
            axis, radius, Fixed64.Zero, out _, out _));
    }

    [Theory]
    [InlineData(1L << 30, 2L)]
    [InlineData(1L << 31, 1L)]
    [InlineData(1L << 33, 0L)]
    public void FiniteCylinder_AdjacentRawRootCorrectionHandlesUnitRadialCoefficient(
        long n,
        long expectedForwardExitRaw)
    {
        var axis = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.FromRaw(n), Fixed64.FromRaw(1), Fixed64.Zero));
        Vector3d start = new(Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Zero);
        Vector3d end = new(Fixed64.FromRaw(n - 1), Fixed64.FromRaw(2), Fixed64.Zero);

        var forward = new FixedSegment(start, end);
        Assert.True(forward.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.FromRaw(1),
            Fixed64.Zero,
            out Fixed64 forwardEntry,
            out Fixed64 forwardExit));
        Assert.Equal(Fixed64.Zero, forwardEntry);
        Assert.Equal(Fixed64.FromRaw(expectedForwardExitRaw), forwardExit);

        var reverse = new FixedSegment(end, start);
        Assert.True(reverse.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.FromRaw(1),
            Fixed64.Zero,
            out Fixed64 reverseEntry,
            out Fixed64 reverseExit));
        Assert.Equal(Fixed64.One - Fixed64.FromRaw(expectedForwardExitRaw), reverseEntry);
        Assert.Equal(Fixed64.One, reverseExit);
    }

    [Fact]
    public void FiniteCylinder_WideArbitraryRawRootsRoundAtParameterBoundaries()
    {
        Fixed64 scale = (Fixed64)100_000;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 radius = scale * Fixed64.Half;
        Fixed64 direction = (scale * 4) + oneRaw;
        Fixed64 axialPosition = scale * Fixed64.Half;
        var axis = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, scale + oneRaw, Fixed64.Zero));

        Fixed64 earlyStart = -radius - oneRaw;
        var earlyRootQuery = new FixedSegment(
            new Vector3d(earlyStart, axialPosition, Fixed64.Zero),
            new Vector3d(earlyStart + direction, axialPosition, Fixed64.Zero));
        (Fixed64 expectedEarlyEntry, Fixed64 expectedEarlyExit) =
            GetRadialIntervalOracle(earlyRootQuery, axis, radius);
        Assert.True(earlyRootQuery.TryGetFiniteCylinderIntersectionInterval(
            axis,
            radius,
            Fixed64.Zero,
            out Fixed64 earlyEntry,
            out Fixed64 earlyExit));
        Assert.Equal(expectedEarlyEntry, earlyEntry);
        Assert.Equal(expectedEarlyExit, earlyExit);
        Assert.Equal(Fixed64.Zero, earlyEntry);

        Fixed64 lateStart = radius - direction + oneRaw;
        var lateRootQuery = new FixedSegment(
            new Vector3d(lateStart, axialPosition, Fixed64.Zero),
            new Vector3d(radius + oneRaw, axialPosition, Fixed64.Zero));
        (Fixed64 expectedLateEntry, Fixed64 expectedLateExit) =
            GetRadialIntervalOracle(lateRootQuery, axis, radius);
        Assert.True(lateRootQuery.TryGetFiniteCylinderIntersectionInterval(
            axis,
            radius,
            Fixed64.Zero,
            out Fixed64 lateEntry,
            out Fixed64 lateExit));
        Assert.Equal(expectedLateEntry, lateEntry);
        Assert.Equal(expectedLateExit, lateExit);
        Assert.Equal(Fixed64.One, lateExit);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    [InlineData(5L, 2L)]
    public void FiniteCylinder_WideHalfRawRootsRoundToEven(long oddMultiple, long expectedLowerRaw)
    {
        const long rootScale = 100_001L;
        Fixed64 scale = (Fixed64)100_000;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 radius = scale * Fixed64.Half;
        Fixed64 direction = Fixed64.FromRaw(rootScale << 33);
        Fixed64 delta = Fixed64.FromRaw(rootScale * oddMultiple);
        Fixed64 axialPosition = scale * Fixed64.Half;
        var axis = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, scale + oneRaw, Fixed64.Zero));

        Fixed64 lowerStart = -radius - delta;
        var lowerTieQuery = new FixedSegment(
            new Vector3d(lowerStart, axialPosition, Fixed64.Zero),
            new Vector3d(lowerStart + direction, axialPosition, Fixed64.Zero));
        (Fixed64 expectedEntry, Fixed64 expectedExit) =
            GetRadialIntervalOracle(lowerTieQuery, axis, radius);
        Assert.True(lowerTieQuery.TryGetFiniteCylinderIntersectionInterval(
            axis,
            radius,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);
        Assert.Equal(Fixed64.FromRaw(expectedLowerRaw), entry);

        Fixed64 upperEnd = radius + delta;
        var upperTieQuery = new FixedSegment(
            new Vector3d(upperEnd - direction, axialPosition, Fixed64.Zero),
            new Vector3d(upperEnd, axialPosition, Fixed64.Zero));
        (expectedEntry, expectedExit) = GetRadialIntervalOracle(upperTieQuery, axis, radius);
        Assert.True(upperTieQuery.TryGetFiniteCylinderIntersectionInterval(
            axis,
            radius,
            Fixed64.Zero,
            out entry,
            out exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);
        Assert.Equal(Fixed64.FromRaw(FixedMath.ONE_L - expectedLowerRaw), exit);
    }

    [Fact]
    public void FiniteCylinder_WideArbitraryRawTangentKeepsOneRoundedParameter()
    {
        Fixed64 scale = (Fixed64)100_000;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 radius = scale * Fixed64.Half;
        Fixed64 direction = (scale * 4) + oneRaw;
        Fixed64 offset = scale + oneRaw;
        Fixed64 axialPosition = scale * Fixed64.Half;
        var axis = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, scale + oneRaw, Fixed64.Zero));
        var query = new FixedSegment(
            new Vector3d(-offset, axialPosition, radius),
            new Vector3d(direction - offset, axialPosition, radius));
        (Fixed64 expectedEntry, Fixed64 expectedExit) = GetRadialIntervalOracle(query, axis, radius);

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            radius,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);
        Assert.Equal(entry, exit);
    }

    [Fact]
    public void FiniteCylinder_ExtremeRadiusDoesNotSaturateBeforeSolving()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)1_500_000_000, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)1_200_000_000, Fixed64.Zero, Fixed64.Zero));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-1), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            (Fixed64)1_400_000_000,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 3), entry);
        Assert.Equal(Fixed64.One, exit);

        var reverse = new FixedSegment(query.End, query.Start);
        Assert.True(reverse.TryGetFiniteCylinderIntersectionInterval(
            axis,
            (Fixed64)1_400_000_000,
            Fixed64.Zero,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.FromFraction(2, 3), exit);
    }

    [Fact]
    public void FiniteCylinder_FullDomainAxisDoesNotNarrowEndpointDifference()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-2)),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 4), entry);
        Assert.Equal(Fixed64.FromFraction(3, 4), exit);
    }

    [Fact]
    public void FiniteCylinder_NonparallelRadialMiss_ReturnsFalse()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2),
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));

        Assert.False(query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.One,
            Fixed64.Zero,
            out _,
            out _));
    }

    [Fact]
    public void FiniteCylinder_AffineExpansionRejectsEveryRadiallyDisjointMotionClass()
    {
        var parallelOutside = new FixedSegment(
            new Vector3d((Fixed64)2, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero));
        var movingAway = new FixedSegment(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        var movingToward = new FixedSegment(movingAway.End, movingAway.Start);
        var interiorMinimumOutside = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2),
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));

        Assert.False(parallelOutside.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));
        Assert.False(movingAway.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));
        Assert.False(movingToward.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));
        Assert.False(interiorMinimumOutside.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));

        Assert.False(movingAway.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis, Fixed64.One, Fixed64.Zero, out _, out _));
        Assert.False(movingToward.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis, Fixed64.One, Fixed64.Zero, out _, out _));
    }

    [Fact]
    public void FiniteCylinder_AffineExpansionPreservesWhollyContainedAndClippedIntervals()
    {
        var contained = new FixedSegment(
            new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        Assert.True(contained.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.One, exit);

        var leavingRadially = new FixedSegment(
            Vector3d.Zero,
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        Assert.True(leavingRadially.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Half, exit);

        var enteringRadially = new FixedSegment(leavingRadially.End, leavingRadially.Start);
        Assert.True(enteringRadially.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(Fixed64.One, exit);

        var leavingAxially = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        Assert.True(leavingAxially.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.FromFraction(1, 3), exit);
    }

    [Fact]
    public void FiniteCylinder_AxiallyDisjointSegmentsRejectBeforeRadialSolving()
    {
        var belowParallel = new FixedSegment(
            new Vector3d(-Fixed64.One, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(Fixed64.One, (Fixed64)(-2), Fixed64.Zero));
        var aboveParallel = new FixedSegment(
            new Vector3d(-Fixed64.One, (Fixed64)2, Fixed64.Zero),
            new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero));
        var before = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero));
        var after = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));

        Assert.False(belowParallel.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis, Fixed64.One, out _, out _));
        Assert.False(aboveParallel.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis, Fixed64.One, out _, out _));
        Assert.False(before.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis, Fixed64.One, out _, out _));
        Assert.False(after.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis, Fixed64.One, out _, out _));

        Assert.False(belowParallel.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));
        Assert.False(aboveParallel.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));
        Assert.False(before.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));
        Assert.False(after.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));
        Assert.False(new FixedSegment(after.End, after.Start).TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero, out _, out _));
    }

    [Fact]
    public void FiniteCylinder_TangentHasOneDiscriminantZeroParameter()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.One),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.One));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(entry, exit);
    }

    [Theory]
    [InlineData(0, true)]
    [InlineData(2, false)]
    public void FiniteCylinder_ParallelQueryUsesConstantRadialDistance(int x, bool expected)
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)x, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d((Fixed64)x, (Fixed64)2, Fixed64.Zero));

        bool found = query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit);

        Assert.Equal(expected, found);
        if (expected)
        {
            Assert.Equal(Fixed64.FromFraction(1, 4), entry);
            Assert.Equal(Fixed64.FromFraction(3, 4), exit);
        }
    }

    [Theory]
    [InlineData(0, true)]
    [InlineData(2, false)]
    public void FiniteCylinder_ZeroLengthQueryReturnsWholeParameterDomainOnlyWhenInside(int x, bool expected)
    {
        Vector3d point = new((Fixed64)x, Fixed64.Zero, Fixed64.Zero);
        var query = new FixedSegment(point, point);

        bool found = query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit);

        Assert.Equal(expected, found);
        if (expected)
        {
            Assert.Equal(Fixed64.Zero, entry);
            Assert.Equal(Fixed64.One, exit);
        }
    }

    [Fact]
    public void FiniteCylinder_ZeroRadiusFindsTheAxisCrossing()
    {
        var query = new FixedSegment(
            new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(entry, exit);
    }

    [Fact]
    public void FiniteCylinder_SubRawIntervalRoundsBothHalfEvenRootsToTheSameParameter()
    {
        var query = new FixedSegment(
            new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.FromRaw(1),
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(entry, exit);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(100_000)]
    public void FiniteCylinder_ScaledSubRawIntervalKeepsIdenticalHalfEvenRoots(int scale)
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-scale), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)scale, Fixed64.Zero, Fixed64.Zero));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-scale), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)scale, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.FromRaw(scale),
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(entry, exit);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    public void FiniteCylinder_AxialHalfRawTieRoundsToEven(long capStartRaw, long expectedEntryRaw)
    {
        var query = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, Fixed64.FromRaw(capStartRaw), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out _));
        Assert.Equal(Fixed64.FromRaw(expectedEntryRaw), entry);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    public void FiniteCylinder_ScaledAxialHalfRawTieKeepsExactClip(long capStartRaw, long expectedEntryRaw)
    {
        const int scale = 100_000;
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-scale), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)scale, (Fixed64)(scale * 2), Fixed64.Zero));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, Fixed64.FromRaw(capStartRaw * scale), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)scale, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            axis,
            (Fixed64)(scale * 2),
            Fixed64.Zero,
            out Fixed64 entry,
            out _));
        Assert.Equal(Fixed64.FromRaw(expectedEntryRaw), entry);
    }

    [Fact]
    public void FiniteCylinder_ExpandedAxialRationalBoundsRoundBetweenRawParameters()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero));
        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 2), Fixed64.Zero),
            Vector3d.Up,
            (Fixed64)3,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 6), entry);
        Assert.Equal(Fixed64.One, exit);
    }

    [Fact]
    public void FiniteAxisQueries_RejectNegativeRadiiAndDegenerateCylinderAxis()
    {
        var query = new FixedSegment(Vector3d.Zero, Vector3d.One);
        var axis = new FixedSegment(Vector3d.Zero, Vector3d.Up);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query.TryGetCapsuleIntersectionInterval(axis, -Fixed64.One, Fixed64.Zero, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query.TryGetCapsuleIntersectionInterval(axis, Fixed64.One, -Fixed64.One, out _, out _));
        var query2d = new FixedSegment2d(Vector2d.Zero, Vector2d.One);
        var axis2d = new FixedSegment2d(Vector2d.Zero, Vector2d.One);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query2d.TryGetCapsuleIntersectionInterval(axis2d, -Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query2d.TryGetCapsuleIntersectionInterval(axis2d, Fixed64.One, -Fixed64.One, out _, out _));
        Assert.Throws<ArgumentException>(() =>
            query.TryGetFiniteCylinderIntersectionInterval(
                new FixedSegment(Vector3d.Zero, Vector3d.Zero),
                Fixed64.One,
                Fixed64.Zero,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query.TryGetFiniteCylinderIntersectionInterval(
                axis,
                -Fixed64.One,
                Fixed64.Zero,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query.TryGetFiniteCylinderIntersectionInterval(
                axis,
                Fixed64.One,
                -Fixed64.One,
                out _,
                out _));
    }

}
