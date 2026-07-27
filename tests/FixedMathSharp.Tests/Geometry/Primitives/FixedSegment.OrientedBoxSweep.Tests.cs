using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedSegmentOrientedBoxSweepTests
{
    [Fact]
    public void SweptSphereOrientedBox_IdentityFaceEntry_MatchesAxisAlignedBox()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero);
        var orientedBox = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * Fixed64.Half);
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(
            Vector3d.Zero,
            Vector3d.One);

        bool orientedHit = query.TryGetSweptSphereOrientedBoxIntersectionDistance(
            orientedBox,
            Fixed64.Half,
            Fixed64.Two,
            out Fixed64 orientedDistance);
        bool alignedHit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.Half,
            Fixed64.Two,
            out Fixed64 alignedDistance);

        Assert.Equal(alignedHit, orientedHit);
        Assert.Equal(alignedDistance, orientedDistance);
    }

    [Fact]
    public void SweptSphereOrientedBox_ThinIdentityBox_PreservesFaceEntry()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)(-6), Fixed64.Zero, Fixed64.Zero));
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                (Fixed64)3,
                Fixed64.FromFraction(1, 10),
                Fixed64.FromFraction(1, 10)));

        Assert.True(query.TryGetSweptSphereOrientedBoxIntersectionDistance(
            box,
            Fixed64.Half,
            (Fixed64)10,
            out Fixed64 distance));
        Assert.Equal(Fixed64.Half, distance);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void SweptSphereOrientedBox_RotatedFaceEdgeAndCorner_MatchLocalBox(
        int feature)
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)23,
            (Fixed64)37,
            (Fixed64)11);
        Vector3d center = new((Fixed64)3, (Fixed64)(-2), (Fixed64)5);
        Vector3d localStart = feature switch
        {
            0 => new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            1 => new Vector3d(
                (Fixed64)2,
                Fixed64.FromFraction(9, 10),
                Fixed64.Zero),
            _ => new Vector3d(
                (Fixed64)2,
                Fixed64.FromFraction(4, 5),
                Fixed64.FromFraction(4, 5))
        };
        Vector3d localEnd = new(
            Fixed64.Zero,
            localStart.Y,
            localStart.Z);
        Assert.True(rotation.TryTransformPoint(
            center,
            localStart,
            out Vector3d worldStart));
        Assert.True(rotation.TryTransformPoint(
            center,
            localEnd,
            out Vector3d worldEnd));
        var worldQuery = new FixedSegment(worldStart, worldEnd);
        var localQuery = new FixedSegment(localStart, localEnd);
        var orientedBox = new FixedOrientedBox(
            center,
            rotation,
            Vector3d.One * Fixed64.Half);
        FixedBoundBox localBox = FixedBoundBox.FromCenterAndSize(
            Vector3d.Zero,
            Vector3d.One);
        Assert.True(Vector3d.TryGetDistance(
            worldStart,
            worldEnd,
            out Fixed64 totalDistance));

        bool worldHit = worldQuery.TryGetSweptSphereOrientedBoxIntersectionDistance(
            orientedBox,
            Fixed64.Half,
            totalDistance,
            out Fixed64 worldDistance);
        bool localHit = localQuery.TryGetSweptSphereBoxIntersectionDistance(
            localBox,
            Fixed64.Half,
            totalDistance,
            out Fixed64 localDistance);

        Assert.Equal(localHit, worldHit);
        Assert.True((worldDistance - localDistance).Abs() <= Fixed64.Epsilon);
    }

    [Fact]
    public void SweptSphereOrientedBox_StartContainedAndPointMiss_AreExplicit()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)19,
            (Fixed64)41,
            (Fixed64)(-7));
        var box = new FixedOrientedBox(
            Vector3d.One,
            rotation,
            Vector3d.One * Fixed64.Half);
        var contained = new FixedSegment(
            Vector3d.One,
            Vector3d.One * (Fixed64)3);
        var missedPoint = new FixedSegment(
            Vector3d.One * (Fixed64)3,
            Vector3d.One * (Fixed64)3);

        Assert.True(contained.TryGetSweptSphereOrientedBoxIntersectionDistance(
            box,
            Fixed64.Half,
            Fixed64.One,
            out Fixed64 containedDistance));
        Assert.Equal(Fixed64.Zero, containedDistance);
        Assert.False(missedPoint.TryGetSweptSphereOrientedBoxIntersectionDistance(
            box,
            Fixed64.Half,
            Fixed64.Zero,
            out Fixed64 missedDistance));
        Assert.Equal(Fixed64.Zero, missedDistance);
    }

    [Fact]
    public void SweptSphereOrientedBox_FullDomainChord_PreservesIntersection()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)23,
                (Fixed64)37,
                (Fixed64)11),
            Vector3d.One);

        Assert.True(query.TryGetSweptSphereOrientedBoxIntersectionDistance(
            box,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 distance));
        Assert.True(distance > Fixed64.Zero);
        Assert.True(distance < Fixed64.One);
    }

    [Fact]
    public void SweptSphereOrientedBox_TraversesCoincidentBreakpointsAndBoundaryStarts()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var diagonal = new FixedSegment(
            new Vector3d(3, 3, 3),
            Vector3d.Zero);
        Assert.True(Vector3d.TryGetDistance(
            diagonal.Start,
            diagonal.End,
            out Fixed64 diagonalLength));
        Assert.True(diagonal.TryGetSweptSphereOrientedBoxIntersectionDistance(
            box,
            Fixed64.Half,
            diagonalLength,
            out Fixed64 diagonalDistance));
        Assert.True(diagonalDistance > Fixed64.Zero);

        var parallelMiss = new FixedSegment(
            new Vector3d(-3, 2, 0),
            new Vector3d(3, 2, 0));
        Assert.False(parallelMiss.TryGetSweptSphereOrientedBoxIntersectionDistance(
            box,
            Fixed64.Half,
            (Fixed64)6,
            out _));

        FixedSegment[] boundaryStarts =
        {
            new(new Vector3d(-1, 3, 0), new Vector3d(-2, 4, 0)),
            new(new Vector3d(-1, 3, 0), new Vector3d(0, 4, 0)),
            new(new Vector3d(1, 3, 0), new Vector3d(2, 4, 0)),
            new(new Vector3d(1, 3, 0), new Vector3d(0, 4, 0)),
        };
        foreach (FixedSegment query in boundaryStarts)
        {
            Assert.True(Vector3d.TryGetDistance(
                query.Start,
                query.End,
                out Fixed64 length));
            Assert.False(query.TryGetSweptSphereOrientedBoxIntersectionDistance(
                box,
                Fixed64.Zero,
                length,
                out _));
        }
    }

    [Fact]
    public void SweptSphereOrientedBox_InvalidArguments_Throw()
    {
        var query = new FixedSegment(Vector3d.Zero, Vector3d.One);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query.TryGetSweptSphereOrientedBoxIntersectionDistance(
                box,
                -Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query.TryGetSweptSphereOrientedBoxIntersectionDistance(
                box,
                Fixed64.Zero,
                -Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            query.TryGetSweptSphereOrientedBoxIntersectionDistance(
                box,
                Fixed64.Zero,
                Fixed64.Zero,
                out _));
    }

    [Fact]
    public void SweptSphereOrientedBox_DoesNotAllocateAfterWarmup()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)23,
                (Fixed64)37,
                (Fixed64)11),
            Vector3d.One * Fixed64.Half);
        Assert.True(query.TryGetSweptSphereOrientedBoxIntersectionDistance(
            box,
            Fixed64.Half,
            Fixed64.Two,
            out _));

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 32; iteration++)
        {
            _ = query.TryGetSweptSphereOrientedBoxIntersectionDistance(
                box,
                Fixed64.Half,
                Fixed64.Two,
                out _);
        }

        Assert.Equal(
            0L,
            GC.GetAllocatedBytesForCurrentThread() - before);
    }
}
