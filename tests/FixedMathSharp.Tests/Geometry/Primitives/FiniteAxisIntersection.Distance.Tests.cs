using System;
using System.Numerics;
using FixedMathSharp.Bounds;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void CapsuleDistanceIntervals_PreserveOneRawPhysicalOrderingOnLongSegments()
    {
        var query2d = new FixedSegment2d(
            Vector2d.Zero,
            new Vector2d((Fixed64)1_000_000, Fixed64.Zero));
        var query3d = new FixedSegment(
            Vector3d.Zero,
            new Vector3d((Fixed64)1_000_000, Fixed64.Zero, Fixed64.Zero));
        Fixed64 length = (Fixed64)1_000_000;

        foreach (long centerOffsetRaw in new[] { 0L, 1L })
        {
            Fixed64 centerX = (Fixed64)101 + Fixed64.FromRaw(centerOffsetRaw);
            var center2d = new Vector2d(centerX, Fixed64.Zero);
            var center3d = new Vector3d(centerX, Fixed64.Zero, Fixed64.Zero);
            Fixed64 expectedEntry = (Fixed64)100 + Fixed64.FromRaw(centerOffsetRaw);

            Assert.True(query2d.TryGetCapsuleIntersectionDistanceInterval(
                center2d,
                Vector2d.Forward,
                Fixed64.Zero,
                Fixed64.One,
                length,
                out Fixed64 entry2d,
                out _));
            Assert.True(query3d.TryGetCapsuleIntersectionDistanceInterval(
                center3d,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                length,
                out Fixed64 entry3d,
                out _));

            Assert.Equal(expectedEntry, entry2d);
            Assert.Equal(expectedEntry, entry3d);
        }
    }

    [Fact]
    public void CapsuleDistanceInterval_RetainsTwoRawTransverseChordAtHalfway()
    {
        var query2d = new FixedSegment2d(
            new Vector2d((Fixed64)(-100_000), Fixed64.Zero),
            new Vector2d((Fixed64)100_000, Fixed64.FromRaw(2L)));
        var query3d = new FixedSegment(
            new Vector3d(query2d.Start.X, query2d.Start.Y, Fixed64.Zero),
            new Vector3d(query2d.End.X, query2d.End.Y, Fixed64.Zero));
        var center2d = new Vector2d(Fixed64.Zero, Fixed64.FromRaw(1L));
        var center3d = new Vector3d(center2d.X, center2d.Y, Fixed64.Zero);
        Fixed64 length = (Fixed64)200_000;

        Assert.True(query2d.TryGetCapsuleIntersectionDistanceInterval(
            center2d,
            Vector2d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            length,
            out Fixed64 entry2d,
            out Fixed64 exit2d));
        Assert.True(query3d.TryGetCapsuleIntersectionDistanceInterval(
            center3d,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.Zero,
            length,
            out Fixed64 entry3d,
            out Fixed64 exit3d));

        Assert.Equal((Fixed64)100_000, entry2d);
        Assert.Equal(entry2d, exit2d);
        Assert.Equal(entry2d, entry3d);
        Assert.Equal(exit2d, exit3d);
    }

    [Fact]
    public void FiniteCylinderDistanceIntervals_EndpointCenteredAndAffineFamiliesAgree()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis,
            Fixed64.One,
            Fixed64.Zero,
            (Fixed64)6,
            out Fixed64 endpointEntry,
            out Fixed64 endpointExit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.True(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)6,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit,
            out _,
            out _));
        Assert.True(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)6,
            out Fixed64 affineEntry,
            out Fixed64 affineExit));

        Assert.Equal((Fixed64)2, endpointEntry);
        Assert.Equal((Fixed64)4, endpointExit);
        Assert.Equal(endpointEntry, centeredEntry);
        Assert.Equal(endpointExit, centeredExit);
        Assert.Equal(endpointEntry, affineEntry);
        Assert.Equal(endpointExit, affineExit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void CapsuleDistanceIntervals_EndpointAndCenteredFamiliesAgreeWithContainment()
    {
        var query2d = new FixedSegment2d(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero));
        var axis2d = new FixedSegment2d(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One));
        var query3d = new FixedSegment(
            new Vector3d(query2d.Start.X, query2d.Start.Y, Fixed64.Zero),
            new Vector3d(query2d.End.X, query2d.End.Y, Fixed64.Zero));
        var axis3d = new FixedSegment(
            new Vector3d(axis2d.Start.X, axis2d.Start.Y, Fixed64.Zero),
            new Vector3d(axis2d.End.X, axis2d.End.Y, Fixed64.Zero));

        Assert.True(query2d.TryGetCapsuleIntersectionDistanceInterval(
            axis2d, Fixed64.One, Fixed64.Zero, (Fixed64)6,
            out Fixed64 endpointEntry2d, out Fixed64 endpointExit2d,
            out bool startContained2d, out bool endContainedStrict2d));
        Assert.True(query2d.TryGetCapsuleIntersectionDistanceInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One,
            Fixed64.Zero, (Fixed64)6,
            out Fixed64 centeredEntry2d, out Fixed64 centeredExit2d,
            out _, out _));
        Assert.True(query3d.TryGetCapsuleIntersectionDistanceInterval(
            axis3d, Fixed64.One, Fixed64.Zero, (Fixed64)6,
            out Fixed64 endpointEntry3d, out Fixed64 endpointExit3d,
            out bool startContained3d, out bool endContainedStrict3d));
        Assert.True(query3d.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, (Fixed64)6,
            out Fixed64 centeredEntry3d, out Fixed64 centeredExit3d,
            out _, out _));

        Assert.Equal((Fixed64)2, endpointEntry2d);
        Assert.Equal((Fixed64)4, endpointExit2d);
        Assert.Equal(endpointEntry2d, centeredEntry2d);
        Assert.Equal(endpointExit2d, centeredExit2d);
        Assert.Equal(endpointEntry2d, endpointEntry3d);
        Assert.Equal(endpointExit2d, endpointExit3d);
        Assert.Equal(endpointEntry3d, centeredEntry3d);
        Assert.Equal(endpointExit3d, centeredExit3d);
        Assert.False(startContained2d);
        Assert.False(endContainedStrict2d);
        Assert.False(startContained3d);
        Assert.False(endContainedStrict3d);
    }

    [Fact]
    public void CapsuleDistanceIntervals_CollapsedEndpointAxesUseCircleAndSphereLimits()
    {
        var query2d = new FixedSegment2d(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero));
        var query3d = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        var collapsed2d = new FixedSegment2d(Vector2d.Zero, Vector2d.Zero);
        var collapsed3d = new FixedSegment(Vector3d.Zero, Vector3d.Zero);

        Assert.True(query2d.TryGetCapsuleIntersectionDistanceInterval(
            collapsed2d, Fixed64.One, (Fixed64)4,
            out Fixed64 entry2d, out Fixed64 exit2d));
        Assert.True(query3d.TryGetCapsuleIntersectionDistanceInterval(
            collapsed3d, Fixed64.One, (Fixed64)4,
            out Fixed64 entry3d, out Fixed64 exit3d));
        Assert.True(query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            new FixedSegment(-Vector3d.Up, Vector3d.Up),
            Fixed64.One,
            (Fixed64)4,
            out Fixed64 cylinderEntry,
            out Fixed64 cylinderExit));

        Assert.Equal(Fixed64.One, entry2d);
        Assert.Equal((Fixed64)3, exit2d);
        Assert.Equal(entry2d, entry3d);
        Assert.Equal(exit2d, exit3d);
        Assert.Equal(entry2d, cylinderEntry);
        Assert.Equal(exit2d, cylinderExit);
    }

    [Fact]
    public void DistanceIntervals_RejectInvalidAuthoredShapeAndDistanceContracts()
    {
        var query2d = new FixedSegment2d(Vector2d.Zero, Vector2d.One);
        var query3d = new FixedSegment(Vector3d.Zero, Vector3d.One);
        var axis2d = new FixedSegment2d(Vector2d.Zero, Vector2d.Forward);
        var axis3d = new FixedSegment(Vector3d.Zero, Vector3d.Up);
        var collapsed3d = new FixedSegment(Vector3d.Zero, Vector3d.Zero);

        Assert.Throws<ArgumentOutOfRangeException>(() => query2d.TryGetCapsuleIntersectionDistanceInterval(
            axis2d, -Fixed64.One, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query2d.TryGetCapsuleIntersectionDistanceInterval(
            axis2d, Fixed64.One, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query2d.TryGetCapsuleIntersectionDistanceInterval(
            axis2d, Fixed64.One, Fixed64.Zero, -Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentException>(() => query3d.TryGetCapsuleIntersectionDistanceInterval(
            axis3d, Fixed64.One, Fixed64.Zero, Fixed64.Zero, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetCapsuleIntersectionDistanceInterval(
            axis3d, -Fixed64.One, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetCapsuleIntersectionDistanceInterval(
            axis3d, Fixed64.One, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));

        Assert.Throws<ArgumentException>(() => query2d.TryGetCapsuleIntersectionDistanceInterval(
            Vector2d.Zero, Vector2d.One, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query2d.TryGetCapsuleIntersectionDistanceInterval(
            Vector2d.Zero, Vector2d.Forward, -Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query2d.TryGetCapsuleIntersectionDistanceInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, -Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query2d.TryGetCapsuleIntersectionDistanceInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One,
            -Fixed64.One, Fixed64.One, out _, out _, out _, out _));

        Assert.Throws<ArgumentException>(() => query3d.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.One, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, -Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            -Fixed64.One, Fixed64.One, out _, out _, out _, out _));

        Assert.Throws<ArgumentException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            collapsed3d, Fixed64.One, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis3d, -Fixed64.One, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis3d, Fixed64.One, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));

        Assert.Throws<ArgumentException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.One, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            -Fixed64.One, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));

        Assert.Throws<ArgumentException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            collapsed3d, Fixed64.One, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis3d, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis3d, Fixed64.One, -Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis3d, Fixed64.One, Fixed64.One, -Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis3d, Fixed64.One, Fixed64.One, Fixed64.Zero,
            -Fixed64.One, Fixed64.One, out _, out _));
    }

    [Fact]
    public void FiniteCylinderDistanceIntervals_RejectParallelQueriesOutsideCapRange()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-3), (Fixed64)2, Fixed64.Zero),
            new Vector3d((Fixed64)3, (Fixed64)2, Fixed64.Zero));
        var axis = new FixedSegment(-Vector3d.Up, Vector3d.Up);

        Assert.False(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis, Fixed64.One, Fixed64.Zero, (Fixed64)6,
            out _, out _, out _, out _));
        Assert.False(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, (Fixed64)6,
            out _, out _, out _, out _));
        Assert.False(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            axis, Fixed64.One, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, (Fixed64)6, out _, out _));
    }

    [Fact]
    public void CapsuleDistanceIntervals_ClassifyDegenerateAndBoundedRadialMissesExactly()
    {
        var collapsed = new FixedSegment2d(Vector2d.Zero, Vector2d.Zero);
        var pointOutside = new FixedSegment2d(
            new Vector2d(Fixed64.Two, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.Zero));
        var movingAway = new FixedSegment2d(
            new Vector2d(Fixed64.Two, Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero));
        var missesRadially = new FixedSegment2d(
            new Vector2d((Fixed64)(-2), Fixed64.Two),
            new Vector2d(Fixed64.Two, Fixed64.Two));

        Assert.False(pointOutside.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, Fixed64.Zero,
            out _, out _));
        Assert.False(movingAway.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, Fixed64.One,
            out _, out _));
        Assert.False(missesRadially.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, (Fixed64)4,
            out _, out _));
    }

    [Fact]
    public void CapsuleDistanceIntervals_RoundBoundaryRootsOnceInPhysicalDistanceSpace()
    {
        var collapsed = new FixedSegment2d(Vector2d.Zero, Vector2d.Zero);
        Fixed64 oneRaw = Fixed64.FromRaw(1L);

        var nearEntry = new FixedSegment2d(
            new Vector2d(Fixed64.One + oneRaw, Fixed64.Zero),
            new Vector2d(-Fixed64.One, Fixed64.Zero));
        Assert.True(nearEntry.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, oneRaw,
            out Fixed64 nearEntryDistance, out _));
        Assert.Equal(Fixed64.Zero, nearEntryDistance);

        var nearExit = new FixedSegment2d(
            new Vector2d(-Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.One + oneRaw, Fixed64.Zero));
        Assert.True(nearExit.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, oneRaw,
            out _, out Fixed64 nearExitDistance));
        Assert.Equal(oneRaw, nearExitDistance);

        var exactLowerMidpoint = new FixedSegment2d(
            new Vector2d(Fixed64.Two, Fixed64.Zero),
            Vector2d.Zero);
        Assert.True(exactLowerMidpoint.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, Fixed64.FromRaw(3L),
            out Fixed64 lowerMidpoint, out _));
        Assert.Equal(Fixed64.FromRaw(2L), lowerMidpoint);

        var exactUpperMidpoint = new FixedSegment2d(
            Vector2d.Zero,
            new Vector2d(Fixed64.Two, Fixed64.Zero));
        Assert.True(exactUpperMidpoint.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, Fixed64.FromRaw(5L),
            out _, out Fixed64 upperMidpoint));
        Assert.Equal(Fixed64.FromRaw(2L), upperMidpoint);

        var lowerCorrection = new FixedSegment2d(
            new Vector2d(Fixed64.Two, Fixed64.Zero),
            new Vector2d(-oneRaw, Fixed64.Zero));
        Assert.True(lowerCorrection.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, Fixed64.FromRaw(3L),
            out Fixed64 correctedLower, out _));
        Assert.Equal(Fixed64.FromRaw(1L), correctedLower);

        var retainedLower = new FixedSegment2d(
            new Vector2d(Fixed64.Two, Fixed64.Zero),
            new Vector2d((Fixed64)(-2), Fixed64.Zero));
        Assert.True(retainedLower.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, Fixed64.FromRaw(7L),
            out Fixed64 retainedLowerDistance, out _));
        Assert.Equal(Fixed64.FromRaw(2L), retainedLowerDistance);

        var upperCorrection = new FixedSegment2d(
            new Vector2d(oneRaw, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.Zero));
        Assert.True(upperCorrection.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, Fixed64.FromRaw(5L),
            out _, out Fixed64 correctedUpper));
        Assert.Equal(Fixed64.FromRaw(2L), correctedUpper);

        var incrementedUpper = new FixedSegment2d(
            new Vector2d(-oneRaw, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.Zero));
        Assert.True(incrementedUpper.TryGetCapsuleIntersectionDistanceInterval(
            collapsed, Fixed64.One, Fixed64.FromRaw(5L),
            out _, out Fixed64 incrementedUpperDistance));
        Assert.Equal(Fixed64.FromRaw(3L), incrementedUpperDistance);
    }

    [Fact]
    public void CapsuleDistanceIntervals_MatchExactRawRootOracleNearEveryMidpointParity()
    {
        var collapsed = new FixedSegment2d(Vector2d.Zero, Vector2d.Zero);
        for (long radiusRaw = 1L; radiusRaw <= 4L; radiusRaw++)
        {
            Fixed64 radius = Fixed64.FromRaw(radiusRaw);
            for (long offsetRaw = 1L; offsetRaw <= 8L; offsetRaw++)
            {
                long outsideRaw = radiusRaw + offsetRaw;
                for (long interiorRaw = -radiusRaw + 1L; interiorRaw < radiusRaw; interiorRaw++)
                {
                    for (long totalRaw = 1L; totalRaw <= 24L; totalRaw++)
                    {
                        Fixed64 totalDistance = Fixed64.FromRaw(totalRaw);
                        var entering = new FixedSegment2d(
                            new Vector2d(Fixed64.FromRaw(outsideRaw), Fixed64.Zero),
                            new Vector2d(Fixed64.FromRaw(interiorRaw), Fixed64.Zero));
                        Assert.True(entering.TryGetCapsuleIntersectionDistanceInterval(
                            collapsed, radius, totalDistance,
                            out Fixed64 entry, out _));
                        Assert.Equal(
                            RoundRawRatioToEven(
                                (BigInteger)totalRaw * offsetRaw,
                                outsideRaw - interiorRaw),
                            entry.m_rawValue);

                        var exiting = new FixedSegment2d(
                            new Vector2d(Fixed64.FromRaw(interiorRaw), Fixed64.Zero),
                            new Vector2d(Fixed64.FromRaw(outsideRaw), Fixed64.Zero));
                        Assert.True(exiting.TryGetCapsuleIntersectionDistanceInterval(
                            collapsed, radius, totalDistance,
                            out _, out Fixed64 exit));
                        Assert.Equal(
                            RoundRawRatioToEven(
                                (BigInteger)totalRaw * (radiusRaw - interiorRaw),
                                outsideRaw - interiorRaw),
                            exit.m_rawValue);
                    }
                }
            }
        }
    }

    [Fact]
    public void CapsuleDistanceIntervals_MatchExactIrrationalCircleRootOracle()
    {
        const long radiusRaw = 5L;
        var collapsed = new FixedSegment2d(Vector2d.Zero, Vector2d.Zero);
        Fixed64 radius = Fixed64.FromRaw(radiusRaw);
        for (long startX = 6L; startX <= 9L; startX++)
        {
            for (long startY = -4L; startY <= 4L; startY++)
            {
                if ((startX * startX) + (startY * startY) <= radiusRaw * radiusRaw)
                    continue;

                for (long endX = -4L; endX <= 4L; endX += 2L)
                {
                    for (long endY = -4L; endY <= 4L; endY += 2L)
                    {
                        if ((endX * endX) + (endY * endY) >= radiusRaw * radiusRaw)
                            continue;

                        var entering = new FixedSegment2d(
                            new Vector2d(Fixed64.FromRaw(startX), Fixed64.FromRaw(startY)),
                            new Vector2d(Fixed64.FromRaw(endX), Fixed64.FromRaw(endY)));
                        var exiting = new FixedSegment2d(entering.End, entering.Start);
                        for (long totalRaw = 3L; totalRaw <= 19L; totalRaw += 2L)
                        {
                            Fixed64 total = Fixed64.FromRaw(totalRaw);
                            Assert.True(entering.TryGetCapsuleIntersectionDistanceInterval(
                                collapsed, radius, total, out Fixed64 entry, out _));
                            Assert.Equal(
                                GetExpectedCircleRootRaw(
                                    startX, startY, endX, endY,
                                    radiusRaw, totalRaw, entering: true),
                                entry.m_rawValue);

                            Assert.True(exiting.TryGetCapsuleIntersectionDistanceInterval(
                                collapsed, radius, total, out _, out Fixed64 exit));
                            Assert.Equal(
                                GetExpectedCircleRootRaw(
                                    endX, endY, startX, startY,
                                    radiusRaw, totalRaw, entering: false),
                                exit.m_rawValue);
                        }
                    }
                }
            }
        }
    }

    [Fact]
    public void CenteredCapsuleDistanceIntervals_RejectPathsBeyondBothConceptualCaps()
    {
        var query2d = new FixedSegment2d(
            new Vector2d(Fixed64.Zero, (Fixed64)3),
            new Vector2d(Fixed64.Zero, (Fixed64)4));
        var query3d = new FixedSegment(
            new Vector3d(query2d.Start.X, query2d.Start.Y, Fixed64.Zero),
            new Vector3d(query2d.End.X, query2d.End.Y, Fixed64.Zero));

        Assert.False(query2d.TryGetCapsuleIntersectionDistanceInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.Half,
            Fixed64.One, out _, out _));
        Assert.False(query3d.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.Half,
            Fixed64.One, out _, out _));
    }

    [Fact]
    public void GetPointAtDistance_FusesOppositeFacesAndPreservesTransverseRawMidpoint()
    {
        var segment2d = new FixedSegment2d(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.FromRaw(2L)));
        var segment3d = new FixedSegment(
            new Vector3d(segment2d.Start.X, segment2d.Start.Y, Fixed64.MinValue),
            new Vector3d(segment2d.End.X, segment2d.End.Y, Fixed64.MaxValue));

        Assert.Equal(segment2d.Start, segment2d.GetPointAtDistance(Fixed64.Zero, Fixed64.Two));
        Assert.Equal(segment2d.End, segment2d.GetPointAtDistance(Fixed64.Two, Fixed64.Two));
        Assert.Equal(
            new Vector2d(Fixed64.Zero, Fixed64.FromRaw(1L)),
            segment2d.GetPointAtDistance(Fixed64.One, Fixed64.Two));
        Assert.Equal(segment3d.Start, segment3d.GetPointAtDistance(Fixed64.Zero, Fixed64.Two));
        Assert.Equal(segment3d.End, segment3d.GetPointAtDistance(Fixed64.Two, Fixed64.Two));
        Assert.Equal(
            new Vector3d(Fixed64.Zero, Fixed64.FromRaw(1L), Fixed64.Zero),
            segment3d.GetPointAtDistance(Fixed64.One, Fixed64.Two));

        var axisAligned2d = new FixedSegment2d(
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue));
        var axisAligned3d = new FixedSegment(
            new Vector3d(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MaxValue),
            new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue));

        Assert.Equal(
            new Vector2d(Fixed64.Zero, Fixed64.MaxValue),
            axisAligned2d.GetPointAtDistance(Fixed64.One, Fixed64.Two));
        Assert.Equal(
            new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.MaxValue),
            axisAligned3d.GetPointAtDistance(Fixed64.One, Fixed64.Two));
    }

    [Fact]
    public void GetPointAtDistance_RoundsHalfwayCoordinatesUsingFinalRawParity()
    {
        Fixed64 odd = Fixed64.FromRaw(1L);
        Fixed64 even = Fixed64.FromRaw(2L);
        var ascending2d = new FixedSegment2d(new Vector2d(odd, odd), new Vector2d(even, even));
        var descending2d = new FixedSegment2d(new Vector2d(odd, odd), Vector2d.Zero);
        var ascending3d = new FixedSegment(new Vector3d(odd, odd, odd), new Vector3d(even, even, even));
        var descending3d = new FixedSegment(new Vector3d(odd, odd, odd), Vector3d.Zero);

        Assert.Equal(new Vector2d(even, even), ascending2d.GetPointAtDistance(Fixed64.Half, Fixed64.One));
        Assert.Equal(Vector2d.Zero, descending2d.GetPointAtDistance(Fixed64.Half, Fixed64.One));
        Assert.Equal(new Vector3d(even, even, even), ascending3d.GetPointAtDistance(Fixed64.Half, Fixed64.One));
        Assert.Equal(Vector3d.Zero, descending3d.GetPointAtDistance(Fixed64.Half, Fixed64.One));
    }

    [Fact]
    public void GetPointAtDistance_RejectsInvalidDistanceDomain()
    {
        var segment2d = new FixedSegment2d(Vector2d.Zero, Vector2d.One);
        var segment3d = new FixedSegment(Vector3d.Zero, Vector3d.One);

        Assert.Throws<ArgumentException>(() =>
            segment2d.GetPointAtDistance(Fixed64.Zero, Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            segment3d.GetPointAtDistance(Fixed64.Zero, -Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            segment2d.GetPointAtDistance(-Fixed64.Epsilon, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            segment3d.GetPointAtDistance(Fixed64.One + Fixed64.Epsilon, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            segment3d.GetPointAtDistance(-Fixed64.Epsilon, Fixed64.One));
    }

    private static long RoundRawRatioToEven(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        int midpoint = (remainder << 1).CompareTo(denominator);
        if (midpoint > 0 || (midpoint == 0 && !quotient.IsEven))
            quotient++;
        return (long)quotient;
    }

    private static long GetExpectedCircleRootRaw(
        long startX,
        long startY,
        long endX,
        long endY,
        long radiusRaw,
        long totalRaw,
        bool entering)
    {
        for (long lowerRaw = 0L; lowerRaw < totalRaw; lowerRaw++)
        {
            BigInteger denominator = totalRaw * 2L;
            BigInteger midpoint = (lowerRaw * 2L) + 1L;
            BigInteger x = ((BigInteger)startX * denominator)
                + ((BigInteger)(endX - startX) * midpoint);
            BigInteger y = ((BigInteger)startY * denominator)
                + ((BigInteger)(endY - startY) * midpoint);
            BigInteger value = (x * x) + (y * y)
                - ((BigInteger)radiusRaw * radiusRaw * denominator * denominator);
            bool crossedMidpoint = entering ? value.Sign <= 0 : value.Sign >= 0;
            if (!crossedMidpoint)
                continue;

            return value.IsZero && (lowerRaw & 1L) != 0L
                ? lowerRaw + 1L
                : lowerRaw;
        }

        return totalRaw;
    }

    [Fact]
    public void ZeroDistancePointQueries_ClassifyAndReconstructWithoutDownstreamSpecialCases()
    {
        var point2d = new Vector2d((Fixed64)2, (Fixed64)3);
        var point3d = new Vector3d(point2d.X, point2d.Y, (Fixed64)4);
        var query2d = new FixedSegment2d(point2d, point2d);
        var query3d = new FixedSegment(point3d, point3d);

        Assert.Equal(point2d, query2d.GetPointAtDistance(Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(point3d, query3d.GetPointAtDistance(Fixed64.Zero, Fixed64.Zero));
        Assert.True(query2d.TryGetCapsuleIntersectionDistanceInterval(
            point2d,
            Vector2d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry2d,
            out Fixed64 exit2d,
            out bool startContained2d,
            out bool endContainedStrict2d));
        Assert.True(query3d.TryGetFiniteCylinderIntersectionDistanceInterval(
            point3d,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry3d,
            out Fixed64 exit3d,
            out bool startContained3d,
            out bool endContainedStrict3d));

        Assert.Equal(Fixed64.Zero, entry2d);
        Assert.Equal(Fixed64.Zero, exit2d);
        Assert.True(startContained2d);
        Assert.True(endContainedStrict2d);
        Assert.Equal(Fixed64.Zero, entry3d);
        Assert.Equal(Fixed64.Zero, exit3d);
        Assert.True(startContained3d);
        Assert.True(endContainedStrict3d);
    }
}
