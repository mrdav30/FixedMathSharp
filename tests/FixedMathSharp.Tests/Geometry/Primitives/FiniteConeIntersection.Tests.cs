using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void FiniteConePointWitnesses_ClassifyCollapsedInsideAndOutsideSegments()
    {
        var inside = new FixedSegment(Fixed64.One * Vector3d.Right, Fixed64.One * Vector3d.Right);
        var outside = new FixedSegment((Fixed64)2 * Vector3d.Left, (Fixed64)2 * Vector3d.Left);

        Assert.True(inside.TryGetFiniteConeIntersectionPointInterval(
            Vector3d.Zero, Vector3d.Right, Fixed64.Two, Fixed64.One,
            out Vector3d apexEntry, out Vector3d apexExit));
        Assert.Equal(inside.Start, apexEntry);
        Assert.Equal(inside.End, apexExit);
        Assert.True(inside.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero, Vector3d.Right, Fixed64.Two, Fixed64.One,
            out Vector3d minimumAxial));
        Assert.Equal(inside.Start, minimumAxial);
        Assert.True(inside.TryGetCenteredFiniteConeIntersectionPointInterval(
            Vector3d.Right, Vector3d.Left, Fixed64.Two, Fixed64.One,
            out Vector3d centeredEntry, out Vector3d centeredExit));
        Assert.Equal(inside.Start, centeredEntry);
        Assert.Equal(inside.End, centeredExit);

        Assert.False(outside.TryGetFiniteConeIntersectionPointInterval(
            Vector3d.Zero, Vector3d.Right, Fixed64.Two, Fixed64.One, out _, out _));
        Assert.False(outside.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero, Vector3d.Right, Fixed64.Two, Fixed64.One, out _));
        Assert.False(outside.TryGetCenteredFiniteConeIntersectionPointInterval(
            Vector3d.Right, Vector3d.Left, Fixed64.Two, Fixed64.One, out _, out _));
    }
    [Fact]
    public void FiniteCone_ExtremeRadialCrossing_RemainsExact()
    {
        Fixed64 height = (Fixed64)1_000_000_000;
        Fixed64 halfHeight = (Fixed64)500_000_000;
        var query = new FixedSegment(
            new Vector3d(halfHeight, -height, Fixed64.Zero),
            new Vector3d(halfHeight, height, Fixed64.Zero));

        Assert.True(query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            height,
            height,
            out Fixed64 apexEntry,
            out Fixed64 apexExit));
        Assert.Equal(Fixed64.FromFraction(1, 4), apexEntry);
        Assert.Equal(Fixed64.FromFraction(3, 4), apexExit);

        Assert.True(query.TryGetCenteredFiniteConeIntersectionInterval(
            new Vector3d(halfHeight, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Left,
            height,
            height,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit));
        Assert.Equal(apexEntry, centeredEntry);
        Assert.Equal(apexExit, centeredExit);
    }

    [Fact]
    public void FiniteConeDistance_LongChordRetainsRootsThatParameterRoundingCollapses()
    {
        Fixed64 billion = (Fixed64)1_000_000_000;
        Fixed64 totalDistance = (Fixed64)2_000_000_000;
        Fixed64 tenth = Fixed64.FromFraction(1, 10);
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        var query = new FixedSegment(
            new Vector3d(-billion, Fixed64.One, Fixed64.Zero),
            new Vector3d(billion, Fixed64.One, Fixed64.Zero));

        Assert.True(query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            Fixed64.One,
            out Fixed64 parameterEntry,
            out Fixed64 parameterExit));
        Assert.Equal(parameterEntry, parameterExit);

        Assert.True(query.TryGetFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            Fixed64.One,
            totalDistance,
            out Fixed64 apexEntry,
            out Fixed64 apexExit));
        Assert.Equal(billion - tenth, apexEntry);
        Assert.Equal(billion + tenth, apexExit);
        Assert.NotEqual(apexEntry, apexExit);
        Assert.Equal(
            new Vector3d(-tenth, Fixed64.One, Fixed64.Zero),
            query.GetPointAtDistance(apexEntry, totalDistance));
        Assert.Equal(
            new Vector3d(tenth, Fixed64.One, Fixed64.Zero),
            query.GetPointAtDistance(apexExit, totalDistance));
        Assert.True(query.TryGetFiniteConeIntersectionPointInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            Fixed64.One,
            out Vector3d apexEntryPoint,
            out Vector3d apexExitPoint));
        Assert.Equal(new Vector3d(-tenth + oneRaw, Fixed64.One, Fixed64.Zero), apexEntryPoint);
        Assert.Equal(new Vector3d(tenth - oneRaw, Fixed64.One, Fixed64.Zero), apexExitPoint);
        Assert.True(query.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            Fixed64.One,
            out Vector3d minimumAxialPoint));
        Assert.Equal(apexEntryPoint, minimumAxialPoint);

        Assert.True(query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            new Vector3d(Fixed64.Zero, (Fixed64)5, Fixed64.Zero),
            Vector3d.Down,
            (Fixed64)10,
            Fixed64.One,
            totalDistance,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit));
        Assert.Equal(apexEntry, centeredEntry);
        Assert.Equal(apexExit, centeredExit);
        Assert.True(query.TryGetCenteredFiniteConeIntersectionPointInterval(
            new Vector3d(Fixed64.Zero, (Fixed64)5, Fixed64.Zero),
            Vector3d.Down,
            (Fixed64)10,
            Fixed64.One,
            out Vector3d centeredEntryPoint,
            out Vector3d centeredExitPoint));
        Assert.Equal(apexEntryPoint, centeredEntryPoint);
        Assert.Equal(apexExitPoint, centeredExitPoint);
    }

    [Fact]
    public void FiniteConeDistance_AuthoredSegmentEndpointRemainsExact()
    {
        var query = new FixedSegment(Vector3d.Zero, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal((Fixed64)4, exit);
        Assert.True(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void CenteredFiniteCone_OddRawHeightRetainsItsConceptualHalfHeight()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        var point = new FixedSegment(Vector3d.Zero, Vector3d.Zero);

        Assert.True(point.TryGetCenteredFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            oneRaw,
            oneRaw,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.One, exit);
        Assert.True(startContained);
        Assert.True(endContainedStrict);
        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCone(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            oneRaw,
            oneRaw,
            strict: true));
    }

    [Fact]
    public void FiniteCone_TangentAndOneRawOutside_AreDistinct()
    {
        var tangent = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.One, (Fixed64)(-2)),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.Two));
        var miss = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.One + Fixed64.FromRaw(1), (Fixed64)(-2)),
            new Vector3d(Fixed64.Two, Fixed64.One + Fixed64.FromRaw(1), Fixed64.Two));

        Assert.True(tangent.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(entry, exit);
        Assert.False(miss.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out _,
            out _));
    }

    [Fact]
    public void FiniteCone_AxialClipRejectsOppositeLobeBeforeAdmittingCone()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-4), Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)4, Fixed64.One, Fixed64.Zero));

        Assert.True(query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(3, 4), entry);
        Assert.Equal(Fixed64.One, exit);
    }

    [Fact]
    public void FiniteCone_GeneratorParallelChord_UsesLinearBoundary()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(Fixed64.One, exit);

        var reverse = new FixedSegment(query.End, query.Start);
        Assert.True(reverse.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Half, exit);
    }

    [Fact]
    public void FiniteCone_ConstantAndLinearSections_ClassifyTheWholeChord()
    {
        var containedPoint = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.Half, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.Half, Fixed64.Zero));
        var outsidePoint = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.Two, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.Two, Fixed64.Zero));
        var generator = new FixedSegment(
            Vector3d.Zero,
            new Vector3d((Fixed64)4, Fixed64.Two, Fixed64.Zero));
        var interiorParallel = new FixedSegment(
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, -Fixed64.One, Fixed64.Zero));
        var exteriorParallel = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.Two, Fixed64.Zero));
        var skewGenerator = new FixedSegment(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One));

        AssertInterval(containedPoint, Fixed64.Zero, Fixed64.One);
        Assert.False(outsidePoint.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
        AssertInterval(generator, Fixed64.Zero, Fixed64.One);
        AssertInterval(interiorParallel, Fixed64.Zero, Fixed64.One);
        Assert.False(exteriorParallel.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
        Assert.False(skewGenerator.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
    }

    [Fact]
    public void FiniteCone_AxisDominantChords_SelectTheCorrectConeLobeBoundary()
    {
        var entering = new FixedSegment(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)4, Fixed64.One, Fixed64.Zero));
        var leaving = new FixedSegment(entering.End, entering.Start);
        var outside = new FixedSegment(
            new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Two, Fixed64.Zero));

        AssertInterval(entering, Fixed64.Half, Fixed64.One);
        AssertInterval(leaving, Fixed64.Zero, Fixed64.Half);
        Assert.False(outside.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
    }

    [Fact]
    public void FiniteCone_RadialChordsMovingAwayCannotCreateAHit()
    {
        var leaving = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.Two, Fixed64.Zero),
            new Vector3d(Fixed64.Two, (Fixed64)3, Fixed64.Zero));
        var arriving = new FixedSegment(leaving.End, leaving.Start);
        var departingBoundary = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.Two, Fixed64.Zero));
        var arrivingBoundary = new FixedSegment(departingBoundary.End, departingBoundary.Start);

        Assert.False(leaving.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
        Assert.False(arriving.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
        AssertInterval(departingBoundary, Fixed64.Zero, Fixed64.Zero);
        AssertInterval(arrivingBoundary, Fixed64.One, Fixed64.One);
    }

    [Fact]
    public void FiniteCone_AxiallyDisjointChordsAreRejectedInEitherDirection()
    {
        var stationaryBefore = new FixedSegment(
            new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero),
            new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.Zero));
        var before = new FixedSegment(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        var after = new FixedSegment(
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)7, Fixed64.Zero, Fixed64.Zero));

        Assert.False(stationaryBefore.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
        Assert.False(before.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
        Assert.False(after.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
        Assert.False(new FixedSegment(after.End, after.Start).TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out _, out _));
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    [InlineData(5L, 2L)]
    public void FiniteCone_HalfRawSideRootsRoundToEven(long oddMultiple, long expectedLowerRaw)
    {
        const long rootScale = 100_001L;
        Fixed64 scale = (Fixed64)100_000;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 height = scale + oneRaw;
        Fixed64 radiusAtSection = scale * Fixed64.Half;
        Fixed64 direction = Fixed64.FromRaw(rootScale << 33);
        Fixed64 delta = Fixed64.FromRaw(rootScale * oddMultiple);

        Fixed64 lowerStart = -radiusAtSection - delta;
        var lowerTie = new FixedSegment(
            new Vector3d(Fixed64.Zero, radiusAtSection, lowerStart),
            new Vector3d(Fixed64.Zero, radiusAtSection, lowerStart + direction));
        Assert.True(lowerTie.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            height,
            height,
            out Fixed64 entry,
            out _));
        Assert.Equal(Fixed64.FromRaw(expectedLowerRaw), entry);

        Fixed64 upperEnd = radiusAtSection + delta;
        var upperTie = new FixedSegment(
            new Vector3d(Fixed64.Zero, radiusAtSection, upperEnd - direction),
            new Vector3d(Fixed64.Zero, radiusAtSection, upperEnd));
        Assert.True(upperTie.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            height,
            height,
            out _,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromRaw(FixedMath.ONE_L - expectedLowerRaw), exit);
    }

    [Fact]
    public void FiniteCone_ArbitraryRawSideRootsMatchExactRadialOracle()
    {
        Fixed64 scale = (Fixed64)100_000;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 height = scale + oneRaw;
        Fixed64 radiusAtSection = scale * Fixed64.Half;
        Fixed64 direction = (scale * 4) + oneRaw;
        var oracleAxis = new FixedSegment(Vector3d.Zero, new Vector3d(Fixed64.Zero, height, Fixed64.Zero));

        Fixed64 earlyStart = -radiusAtSection - oneRaw;
        var early = new FixedSegment(
            new Vector3d(Fixed64.Zero, radiusAtSection, earlyStart),
            new Vector3d(Fixed64.Zero, radiusAtSection, earlyStart + direction));
        (Fixed64 expectedEntry, Fixed64 expectedExit) =
            GetRadialIntervalOracle(early, oracleAxis, radiusAtSection);
        Assert.True(early.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, height, height, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);

        Fixed64 lateStart = radiusAtSection - direction + oneRaw;
        var late = new FixedSegment(
            new Vector3d(Fixed64.Zero, radiusAtSection, lateStart),
            new Vector3d(Fixed64.Zero, radiusAtSection, radiusAtSection + oneRaw));
        (expectedEntry, expectedExit) = GetRadialIntervalOracle(late, oracleAxis, radiusAtSection);
        Assert.True(late.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, height, height, out entry, out exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);
    }

    [Fact]
    public void FiniteCone_NonSquareDiscriminantCorrectsAdjacentRawRootSeeds()
    {
        Fixed64 height = Fixed64.FromRaw(10);
        Fixed64 sectionRadius = Fixed64.FromRaw(2);
        Fixed64 radialOffset = Fixed64.FromRaw(1);
        var oracleAxis = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, height, Fixed64.Zero));
        var entering = new FixedSegment(
            new Vector3d(radialOffset, sectionRadius, Fixed64.FromRaw(-2)),
            new Vector3d(radialOffset, sectionRadius, Fixed64.FromRaw(-1)));
        var leaving = new FixedSegment(
            new Vector3d(radialOffset, sectionRadius, Fixed64.Zero),
            new Vector3d(radialOffset, sectionRadius, Fixed64.FromRaw(2)));

        (Fixed64 expectedEntry, Fixed64 expectedExit) =
            GetRadialIntervalOracle(entering, oracleAxis, sectionRadius);
        Assert.True(entering.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, height, height, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(Fixed64.One, exit);

        (expectedEntry, expectedExit) = GetRadialIntervalOracle(leaving, oracleAxis, sectionRadius);
        Assert.True(leaving.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, height, height, out entry, out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(expectedExit, exit);
    }

    [Fact]
    public void FiniteCone_NearHalfRawRootsUseExactMidpointCorrection()
    {
        const long offsetRaw = 3_000_000_000L;
        const long squaredOffsetRaw = 9_000_000_000_000_000_000L;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 twoRaw = Fixed64.FromRaw(2);
        var entering = new FixedSegment(
            new Vector3d(
                Fixed64.FromRaw(-833_855_397_541_865_379L),
                Fixed64.FromRaw(-416_927_698_770_932_689L),
                Fixed64.Zero),
            new Vector3d(
                Fixed64.FromRaw(3_461_111_898_673_772_047L),
                Fixed64.FromRaw(1_730_555_949_336_886_024L),
                oneRaw));
        Assert.True(entering.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            oneRaw,
            twoRaw,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromRaw(833_855_397L), entry);
        Assert.Equal(Fixed64.FromRaw(833_855_398L), exit);

        Fixed64 sectionRadius = Fixed64.FromRaw(squaredOffsetRaw + 1L);
        Vector3d sectionStart = new(
            Fixed64.FromRaw(offsetRaw),
            oneRaw,
            Fixed64.FromRaw(squaredOffsetRaw));
        var leaving = new FixedSegment(
            sectionStart,
            sectionStart + (Vector3d.Forward * oneRaw));

        Assert.True(leaving.TryGetFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            oneRaw,
            sectionRadius,
            oneRaw,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(oneRaw, exit);
    }

    [Fact]
    public void FiniteCone_UpperRootBelowNegativeHalfRawRoundsTowardNearestRaw()
    {
        const long q = 3_000_000_000L;
        const long k = 9_000_000_000_000_000_000L;
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Vector3d inside = new(Fixed64.FromRaw(q), oneRaw, Fixed64.FromRaw(k));
        Vector3d outside = inside + (Vector3d.Forward * Fixed64.FromRaw(2));
        var entering = new FixedSegment(outside, inside);

        Assert.True(entering.TryGetFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            oneRaw,
            Fixed64.FromRaw(k + 1L),
            Fixed64.FromRaw(2),
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(oneRaw, entry);
        Assert.Equal(Fixed64.FromRaw(2), exit);
    }

    [Fact]
    public void FiniteCone_NonCardinalNormalizedAxisRetainsAuthoredEndpoints()
    {
        Vector3d axis = new(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5),
            Fixed64.Zero);
        Vector3d apex = new((Fixed64)7, (Fixed64)(-3), Fixed64.Two);
        Vector3d baseCenter = apex + axis;
        var apexAuthored = new FixedSegment(apex, baseCenter);

        Assert.True(axis.IsNormalized());
        Assert.True(FixedSegment.ContainsPointInFiniteCone(
            baseCenter,
            apex,
            axis,
            Fixed64.One,
            Fixed64.Half));
        Assert.True(apexAuthored.TryGetFiniteConeIntersectionInterval(
            apex,
            axis,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 apexEntry,
            out Fixed64 apexExit));
        Assert.Equal(Fixed64.Zero, apexEntry);
        Assert.Equal(Fixed64.One, apexExit);

        Vector3d center = new((Fixed64)(-5), (Fixed64)9, Fixed64.One);
        Vector3d centeredApex = center + axis;
        Vector3d centeredBase = center - axis;
        var centeredAxis = new FixedSegment(centeredApex, centeredBase);

        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCone(
            centeredApex,
            center,
            axis,
            Fixed64.Two,
            Fixed64.Half));
        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCone(
            centeredBase,
            center,
            axis,
            Fixed64.Two,
            Fixed64.Half));
        Assert.True(centeredAxis.TryGetCenteredFiniteConeIntersectionInterval(
            center,
            axis,
            Fixed64.Two,
            Fixed64.Half,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit));
        Assert.Equal(Fixed64.Zero, centeredEntry);
        Assert.Equal(Fixed64.One, centeredExit);
    }

    [Fact]
    public void FiniteCone_ZeroRadiusNonCardinalAxisRetainsItsGenerator()
    {
        Vector3d axis = new(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5),
            Fixed64.Zero);
        var generator = new FixedSegment(Vector3d.Zero, axis);

        Assert.True(generator.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            axis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.One, exit);
        Assert.True(FixedSegment.ContainsPointInFiniteCone(
            axis,
            Vector3d.Zero,
            axis,
            Fixed64.One,
            Fixed64.Zero));
    }

    [Fact]
    public void FiniteCone_BelowUnitNonCardinalAxisRetainsItsZeroRadiusGenerator()
    {
        Vector3d axis = new(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5) - Fixed64.FromRaw(1),
            Fixed64.Zero);
        var generator = new FixedSegment(Vector3d.Zero, axis);

        Assert.True(axis.IsNormalized());
        Assert.True(generator.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            axis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.One, exit);
    }

    [Fact]
    public void CenteredFiniteCone_NonCardinalMidplaneUsesHalfBaseRadius()
    {
        Vector3d axis = new(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5),
            Fixed64.Zero);
        Vector3d perpendicular = new(-axis.Y, axis.X, Fixed64.Zero);
        Vector3d outsideMidplane = perpendicular * Fixed64.FromFraction(3, 4);

        Assert.False(FixedSegment.ContainsPointInCenteredFiniteCone(
            outsideMidplane,
            Vector3d.Zero,
            axis,
            Fixed64.Two,
            Fixed64.One));
    }

    [Fact]
    public void FiniteCone_NonTieThirdRootsRoundToNearestRawParameter()
    {
        var entering = new FixedSegment(
            new Vector3d(Fixed64.Two, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.Zero));
        var leaving = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Two, (Fixed64)3, Fixed64.Zero));

        Assert.True(entering.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 3), entry);
        Assert.Equal(Fixed64.One, exit);

        Assert.True(leaving.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.FromFraction(1, 3), exit);
    }

    [Fact]
    public void FiniteCone_StartInsideAndFlatBase_AreClassifiedExactly()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.FromFraction(2, 3), exit);
        Assert.True(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void FiniteCone_ApexAndBaseEndpointContacts_AreRetained()
    {
        var apexArrival = new FixedSegment(
            new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero);
        var baseDeparture = new FixedSegment(
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));

        Assert.True(apexArrival.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out Fixed64 apexEntry, out Fixed64 apexExit));
        Assert.Equal(Fixed64.One, apexEntry);
        Assert.Equal(apexEntry, apexExit);
        Assert.True(apexArrival.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out Vector3d forwardPoint));
        Assert.Equal(Vector3d.Zero, forwardPoint);
        Assert.True(new FixedSegment(apexArrival.End, apexArrival.Start)
            .TryGetFiniteConeIntersectionMinimumAxialPoint(
                Vector3d.Zero,
                Vector3d.Right,
                (Fixed64)4,
                Fixed64.Two,
                out Vector3d reversePoint));
        Assert.Equal(Vector3d.Zero, reversePoint);

        Assert.True(baseDeparture.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, out Fixed64 baseEntry, out Fixed64 baseExit));
        Assert.Equal(Fixed64.Zero, baseEntry);
        Assert.Equal(baseEntry, baseExit);
    }

    [Fact]
    public void FiniteConeContainment_DistinguishesEveryBoundaryFromInterior()
    {
        Vector3d side = new(Fixed64.Two, Fixed64.One, Fixed64.Zero);
        Vector3d interior = new(Fixed64.Two, Fixed64.Half, Fixed64.Zero);

        Assert.True(FixedSegment.ContainsPointInFiniteCone(
            side, Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two));
        Assert.False(FixedSegment.ContainsPointInFiniteCone(
            side, Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, strict: true));
        Assert.True(FixedSegment.ContainsPointInFiniteCone(
            interior, Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, strict: true));
        Assert.True(FixedSegment.ContainsPointInFiniteCone(
            Vector3d.Zero, Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two));
        Assert.False(FixedSegment.ContainsPointInFiniteCone(
            Vector3d.Zero, Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, strict: true));

        Vector3d center = new(Fixed64.Two, Fixed64.Zero, Fixed64.Zero);
        Assert.Equal(
            FixedSegment.ContainsPointInFiniteCone(
                interior, Vector3d.Zero, Vector3d.Right, (Fixed64)4, Fixed64.Two, strict: true),
            FixedSegment.ContainsPointInCenteredFiniteCone(
                interior, center, Vector3d.Left, (Fixed64)4, Fixed64.Two, strict: true));
        Assert.False(FixedSegment.ContainsPointInCenteredFiniteCone(
            new Vector3d(-Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
            center,
            Vector3d.Left,
            (Fixed64)4,
            Fixed64.Two));
    }

    [Fact]
    public void FiniteCone_RejectsInvalidAuthoredParameters()
    {
        var query = new FixedSegment(Vector3d.Zero, Vector3d.One);

        Assert.Throws<ArgumentException>(() => query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentException>(() => query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right * Fixed64.Two, Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Right, Fixed64.One, -Fixed64.One, out _, out _));

        Assert.Throws<ArgumentException>(() => query.TryGetCenteredFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetCenteredFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Left, Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetCenteredFiniteConeIntersectionInterval(
            Vector3d.Zero, Vector3d.Left, Fixed64.One, -Fixed64.One, out _, out _));

        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Right, Fixed64.One, Fixed64.One, -Fixed64.One, out _, out _));
        Assert.Throws<ArgumentException>(() => query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Left, Fixed64.One, Fixed64.One, Fixed64.Zero, out _, out _));

        Assert.Throws<ArgumentException>(() => FixedSegment.ContainsPointInFiniteCone(
            Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.ContainsPointInCenteredFiniteCone(
            Vector3d.Zero, Vector3d.Zero, Vector3d.Left, Fixed64.Zero, Fixed64.One));
    }

    private static void AssertInterval(FixedSegment query, Fixed64 expectedEntry, Fixed64 expectedExit)
    {
        Assert.True(query.TryGetFiniteConeIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(expectedEntry, entry);
        Assert.Equal(expectedExit, exit);
    }
}
