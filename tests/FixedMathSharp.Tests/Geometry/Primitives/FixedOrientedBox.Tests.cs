using FixedMathSharp.Geometry;
using System;
using System.Numerics;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxTests
{
    [Fact]
    public void Constructor_StoresCanonicalStateAndRejectsInvalidGeometry()
    {
        Vector3d center = new(1, 2, 3);
        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)10,
            (Fixed64)20,
            (Fixed64)30);
        Vector3d halfExtents = new(4, 5, 6);

        var box = new FixedOrientedBox(center, orientation, halfExtents);

        Assert.Equal(center, box.Center);
        Assert.Equal(orientation, box.Orientation);
        Assert.Equal(halfExtents, box.HalfExtents);
        Assert.Throws<ArgumentException>(() =>
            new FixedOrientedBox(center, FixedQuaternion.Zero, halfExtents));
        Assert.Throws<ArgumentException>(() =>
            new FixedOrientedBox(
                center,
                new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Two),
                halfExtents));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new FixedOrientedBox(
                center,
                orientation,
                new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new FixedOrientedBox(
                center,
                orientation,
                new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.One)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new FixedOrientedBox(
                center,
                orientation,
                new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero)));
    }

    [Fact]
    public void DefaultValue_RemainsInvalidGeometry()
    {
        FixedOrientedBox box = default;

        Assert.Throws<InvalidOperationException>(() =>
            box.GetAxes(out _, out _, out _));
        Assert.Throws<InvalidOperationException>(() => box.GetLocalCorner(0));
        Assert.Throws<InvalidOperationException>(() => box.GetLocalSupportPoint(Vector3d.Zero));
        Assert.Throws<InvalidOperationException>(() => box.GetBoundsClippedToDomain());
        Assert.Throws<InvalidOperationException>(() => box.Contains(Vector3d.Zero));
        Assert.Throws<InvalidOperationException>(() =>
            box.TryGetClosestPointOnSurface(Vector3d.Zero, out _));
        Assert.Throws<InvalidOperationException>(() => box.GetNearestFaceNormal(Vector3d.Zero));
        Assert.Throws<InvalidOperationException>(() =>
            box.TryMaterializeLocalPoint(Vector3d.Zero, out _));
    }

    [Fact]
    public void Equality_IsStructuralWhileQuaternionSignsHaveGeometricParity()
    {
        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)(-31),
            (Fixed64)43);
        FixedQuaternion negated = -orientation;
        Vector3d center = new(3, -5, 7);
        Vector3d halfExtents = new(2, 4, 6);
        var first = new FixedOrientedBox(center, orientation, halfExtents);
        var same = new FixedOrientedBox(center, orientation, halfExtents);
        var oppositeSign = new FixedOrientedBox(center, negated, halfExtents);

        Assert.True(first == same);
        Assert.False(first != same);
        Assert.True(first.Equals(same));
        Assert.True(first.Equals((object)same));
        Assert.Equal(first.GetHashCode(), same.GetHashCode());
        Assert.False(first.Equals("not a box"));
        Assert.False(first == oppositeSign);
        Assert.True(first != oppositeSign);

        first.GetAxes(out Vector3d firstX, out Vector3d firstY, out Vector3d firstZ);
        oppositeSign.GetAxes(out Vector3d secondX, out Vector3d secondY, out Vector3d secondZ);
        Assert.Equal(firstX, secondX);
        Assert.Equal(firstY, secondY);
        Assert.Equal(firstZ, secondZ);
        Assert.Equal(first.GetBoundsClippedToDomain(), oppositeSign.GetBoundsClippedToDomain());

        Vector3d localCorner = first.GetLocalCorner(7);
        Assert.True(first.TryMaterializeLocalPoint(localCorner, out Vector3d firstCorner));
        Assert.True(oppositeSign.TryMaterializeLocalPoint(localCorner, out Vector3d secondCorner));
        Assert.Equal(firstCorner, secondCorner);
        Assert.Equal(
            first.GetLocalSupportPoint(new Vector3d(7, -11, 13)),
            oppositeSign.GetLocalSupportPoint(new Vector3d(7, -11, 13)));
    }

    [Fact]
    public void ExactQuaternionBasis_AgreesAcrossClassificationAndSignParity()
    {
        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)(-180),
            (Fixed64)(-180),
            (Fixed64)(-154));
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            new Vector3d(2, 3, 5));
        var negatedBox = new FixedOrientedBox(box.Center, -orientation, box.HalfExtents);
        Vector3d neighborhoodCenter = new(
            Fixed64.FromRaw(-2_072_212_855L),
            Fixed64.FromRaw(-15_346_452_586L),
            Fixed64.FromRaw(-21_474_836_480L));

        for (long xOffset = -16L; xOffset <= 16L; xOffset++)
        {
            for (long yOffset = -16L; yOffset <= 16L; yOffset++)
            {
                Vector3d point = new(
                    Fixed64.FromRaw(neighborhoodCenter.X.m_rawValue + xOffset),
                    Fixed64.FromRaw(neighborhoodCenter.Y.m_rawValue + yOffset),
                    neighborhoodCenter.Z);
                bool expected = ContainsExactly(box, point);
                Assert.Equal(expected, box.Contains(point));
                Assert.Equal(expected, negatedBox.Contains(point));
            }
        }

        Vector3d insetLocal = new(
            Fixed64.FromRaw(box.HalfExtents.X.m_rawValue - 128L),
            Fixed64.FromRaw(-box.HalfExtents.Y.m_rawValue + 128L),
            Fixed64.FromRaw(-box.HalfExtents.Z.m_rawValue + 128L));
        Assert.True(box.TryMaterializeLocalPoint(insetLocal, out Vector3d insetWorld));
        Assert.True(negatedBox.TryMaterializeLocalPoint(insetLocal, out Vector3d negatedInsetWorld));
        Assert.Equal(insetWorld, negatedInsetWorld);
        Assert.True(ContainsExactly(box, insetWorld));
        Assert.True(box.Contains(insetWorld));
        Assert.True(negatedBox.Contains(insetWorld));

        Vector3d outsideLocal = new(
            Fixed64.FromRaw(box.HalfExtents.X.m_rawValue + 128L),
            -box.HalfExtents.Y,
            -box.HalfExtents.Z);
        Assert.True(box.TryMaterializeLocalPoint(outsideLocal, out Vector3d outsideWorld));
        Assert.False(ContainsExactly(box, outsideWorld));
        Assert.False(box.Contains(outsideWorld));
        Assert.False(negatedBox.Contains(outsideWorld));
        Assert.True(box.TryGetClosestPointOnSurface(outsideWorld, out Vector3d closest));
        Assert.True(negatedBox.TryGetClosestPointOnSurface(outsideWorld, out Vector3d negatedClosest));
        Assert.Equal(closest, negatedClosest);
        Assert.Equal(
            box.GetNearestFaceNormal(outsideWorld),
            negatedBox.GetNearestFaceNormal(outsideWorld));
    }

    [Fact]
    public void LocalCornersAndSupport_UseStableBitOrderAndLowerTieIndex()
    {
        var box = new FixedOrientedBox(
            new Vector3d(10, 20, 30),
            FixedQuaternion.Identity,
            new Vector3d(1, 2, 3));

        Vector3d[] expected =
        {
            new(-1, -2, -3),
            new(1, -2, -3),
            new(-1, 2, -3),
            new(1, 2, -3),
            new(-1, -2, 3),
            new(1, -2, 3),
            new(-1, 2, 3),
            new(1, 2, 3),
        };

        for (int index = 0; index < expected.Length; index++)
            Assert.Equal(expected[index], box.GetLocalCorner(index));

        Assert.Throws<ArgumentOutOfRangeException>(() => box.GetLocalCorner(-1));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.GetLocalCorner(FixedOrientedBox.CornerCount));
        Assert.Equal(expected[0], box.GetLocalSupportPoint(Vector3d.Zero));
        Assert.Equal(expected[1], box.GetLocalSupportPoint(Vector3d.Right));
        Assert.Equal(expected[3], box.GetLocalSupportPoint(Vector3d.Right + Vector3d.Up));
    }

    [Fact]
    public void Bounds_MatchExactAnalyticalExtentsForOrdinaryAndRotatedBoxes()
    {
        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)19,
            (Fixed64)(-37),
            (Fixed64)53);
        var rotated = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromFraction(3, 7),
                Fixed64.FromFraction(-11, 13),
                Fixed64.FromFraction(17, 5)),
            orientation,
            new Vector3d(
                Fixed64.FromFraction(7, 3),
                Fixed64.FromFraction(11, 5),
                Fixed64.FromFraction(13, 7)));
        var aligned = new FixedOrientedBox(
            new Vector3d(3, -5, 7),
            FixedQuaternion.Identity,
            new Vector3d(2, 4, 6));

        Assert.Equal(
            FixedBoundBox.FromMinMax(new Vector3d(1, -9, 1), new Vector3d(5, -1, 13)),
            aligned.GetBoundsClippedToDomain());
        Assert.Equal(GetExpectedBounds(rotated), rotated.GetBoundsClippedToDomain());
    }

    [Fact]
    public void Bounds_ClipOnlyFinalMirroredScalarFacesAndRoundLeastOutward()
    {
        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)23,
            (Fixed64)41,
            (Fixed64)(-29));
        Vector3d halfExtents = new(
            Fixed64.FromRaw(9_876_543_211L),
            Fixed64.FromRaw(8_765_432_109L),
            Fixed64.FromRaw(7_654_321_097L));
        var maximumFace = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue),
            orientation,
            halfExtents);
        var minimumFace = new FixedOrientedBox(
            new Vector3d(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue),
            orientation,
            halfExtents);

        FixedBoundBox expectedMaximum = GetExpectedBounds(maximumFace);
        FixedBoundBox expectedMinimum = GetExpectedBounds(minimumFace);

        Assert.Equal(expectedMaximum, maximumFace.GetBoundsClippedToDomain());
        Assert.Equal(expectedMinimum, minimumFace.GetBoundsClippedToDomain());
        Assert.Equal(
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue),
            expectedMaximum.Max);
        Assert.Equal(
            new Vector3d(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue),
            expectedMinimum.Min);
    }

    [Fact]
    public void Contains_ClassifiesFaceEdgeCornerAndFullDomainOppositeFaces()
    {
        var ordinary = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, 3, 4));

        Assert.True(ordinary.Contains(new Vector3d(2, 0, 0)));
        Assert.True(ordinary.Contains(new Vector3d(-2, 3, 0)));
        Assert.True(ordinary.Contains(new Vector3d(2, -3, 4)));
        Assert.False(ordinary.Contains(new Vector3d(
            Fixed64.FromRaw((2L << 32) + 1L),
            Fixed64.Zero,
            Fixed64.Zero)));
        Assert.False(ordinary.Contains(new Vector3d(
            Fixed64.Zero,
            Fixed64.FromRaw((-3L << 32) - 1L),
            Fixed64.Zero)));

        var fullDomain = new FixedOrientedBox(
            new Vector3d(Fixed64.FromRaw(-1L), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.One));
        Assert.True(fullDomain.Contains(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero)));
        Assert.True(fullDomain.Contains(new Vector3d(
            Fixed64.FromRaw(long.MaxValue - 1L),
            Fixed64.Zero,
            Fixed64.Zero)));
        Assert.False(fullDomain.Contains(new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero)));
    }

    [Fact]
    public void ClosestPoint_ClampsOutsideAndUsesStableInsideFaceTies()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(4, 2, 2));

        Assert.True(box.TryGetClosestPointOnSurface(new Vector3d(7, -5, 1), out Vector3d outside));
        Assert.Equal(new Vector3d(4, -2, 1), outside);

        Assert.True(box.TryGetClosestPointOnSurface(Vector3d.Zero, out Vector3d yzTie));
        Assert.Equal(new Vector3d(0, 2, 0), yzTie);
        Assert.True(box.TryGetClosestPointOnSurface(
            new Vector3d(0, -1, 0),
            out Vector3d negativeYFace));
        Assert.Equal(new Vector3d(0, -2, 0), negativeYFace);

        var allTied = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, 2, 2));
        Assert.True(allTied.TryGetClosestPointOnSurface(Vector3d.Zero, out Vector3d xyzTie));
        Assert.Equal(new Vector3d(2, 0, 0), xyzTie);
        Assert.True(allTied.TryGetClosestPointOnSurface(
            new Vector3d(
                Fixed64.FromFraction(-7, 4),
                Fixed64.Zero,
                Fixed64.Zero),
            out Vector3d negativeFace));
        Assert.Equal(new Vector3d(-2, 0, 0), negativeFace);

        var zNearest = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(5, 4, 3));
        Assert.True(zNearest.TryGetClosestPointOnSurface(Vector3d.Zero, out Vector3d positiveZFace));
        Assert.Equal(new Vector3d(0, 0, 3), positiveZFace);
        Assert.True(zNearest.TryGetClosestPointOnSurface(
            new Vector3d(0, 0, -1),
            out Vector3d negativeZFace));
        Assert.Equal(new Vector3d(0, 0, -3), negativeZFace);

        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)13,
            (Fixed64)29,
            (Fixed64)(-47));
        var rotated = new FixedOrientedBox(
            new Vector3d(3, -5, 7),
            orientation,
            new Vector3d(4, 2, 2));
        Assert.True(rotated.TryMaterializeLocalPoint(
            new Vector3d(7, -5, 1),
            out Vector3d rotatedOutside));
        Assert.True(TryGetExpectedClosestPoint(rotated, rotatedOutside, out Vector3d expectedRotatedClosest));
        Assert.True(rotated.TryGetClosestPointOnSurface(
            rotatedOutside,
            out Vector3d actualRotatedClosest));
        Assert.Equal(expectedRotatedClosest, actualRotatedClosest);

        Assert.True(TryGetExpectedClosestPoint(rotated, rotated.Center, out Vector3d expectedRotatedInside));
        Assert.True(rotated.TryGetClosestPointOnSurface(
            rotated.Center,
            out Vector3d actualRotatedInside));
        Assert.Equal(expectedRotatedInside, actualRotatedInside);

        var domainEdge = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        FixedPointAnchor domainEdgeAnchor =
            domainEdge.GetClosestPointAnchor(domainEdge.Center);
        Assert.Equal(domainEdge.Center, domainEdgeAnchor.Origin);
        Assert.Equal(Vector3d.Right, domainEdgeAnchor.LocalPoint);
        Assert.False(domainEdgeAnchor.TryGetPoint(out _));
        Assert.False(domainEdge.TryGetClosestPointOnSurface(domainEdge.Center, out Vector3d unavailable));
        Assert.Equal(default, unavailable);
    }

    [Fact]
    public void NearestFaceNormal_UsesCoordinateSignAndStableAxisTies()
    {
        var allTied = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, 2, 2));
        Assert.Equal(Vector3d.Right, allTied.GetNearestFaceNormal(Vector3d.Zero));
        Assert.Equal(-Vector3d.Right, allTied.GetNearestFaceNormal(new Vector3d(-1, 0, 0)));

        var yzTie = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(4, 2, 2));
        Assert.Equal(Vector3d.Up, yzTie.GetNearestFaceNormal(Vector3d.Zero));
        Assert.Equal(-Vector3d.Up, yzTie.GetNearestFaceNormal(new Vector3d(0, -1, 0)));

        var zNearest = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(5, 4, 3));
        Assert.Equal(Vector3d.Forward, zNearest.GetNearestFaceNormal(Vector3d.Zero));
        Assert.Equal(
            -Vector3d.Forward,
            zNearest.GetNearestFaceNormal(new Vector3d(0, 0, -1)));

        var outside = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, 3, 4));
        Assert.Equal(Vector3d.Right, outside.GetNearestFaceNormal(new Vector3d(100, 0, 0)));
        Assert.Equal(-Vector3d.Right, outside.GetNearestFaceNormal(new Vector3d(-100, 0, 0)));
        Assert.Equal(
            Vector3d.Right,
            outside.GetNearestFaceNormal(new Vector3d(100, -100, 0)));
        Assert.Equal(Vector3d.Up, outside.GetNearestFaceNormal(new Vector3d(0, 100, 0)));
        Assert.Equal(-Vector3d.Up, outside.GetNearestFaceNormal(new Vector3d(0, -100, 0)));
        Assert.Equal(
            Vector3d.Forward,
            outside.GetNearestFaceNormal(new Vector3d(0, 0, 100)));
        Assert.Equal(
            -Vector3d.Forward,
            outside.GetNearestFaceNormal(new Vector3d(0, 0, -100)));

        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)13,
            (Fixed64)29,
            (Fixed64)(-47));
        var rotated = new FixedOrientedBox(Vector3d.Zero, orientation, new Vector3d(5, 4, 3));
        rotated.GetAxes(out _, out _, out Vector3d axisZ);
        Assert.Equal(axisZ, rotated.GetNearestFaceNormal(Vector3d.Zero));
    }

    [Fact]
    public void Materialization_PreservesCancellationAndFailsAtomicallyPerWorldCoordinate()
    {
        long quaternionUnit = 858_993_459L;
        FixedQuaternion orientation = new(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.FromRaw(3L * quaternionUnit),
            Fixed64.FromRaw(4L * quaternionUnit));
        long cancellationScale = (long.MaxValue - 100L) / 25L;
        var cancellation = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            orientation,
            Vector3d.One);
        Vector3d local = new(
            Fixed64.FromRaw(24L * cancellationScale),
            Fixed64.FromRaw(7L * cancellationScale),
            Fixed64.Zero);

        bool expectedFits = TryGetExpectedMaterialization(cancellation, local, out Vector3d expected);
        Assert.True(expectedFits);
        Assert.True(cancellation.TryMaterializeLocalPoint(local, out Vector3d actual));
        Assert.Equal(expected, actual);

        var ordinary = new FixedOrientedBox(
            new Vector3d(1, 2, 3),
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(ordinary.TryMaterializeLocalPoint(new Vector3d(2, -4, 6), out Vector3d ordinaryPoint));
        Assert.Equal(new Vector3d(3, -2, 9), ordinaryPoint);
        var origin = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(origin.TryMaterializeLocalPoint(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            out Vector3d minimumPoint));
        Assert.Equal(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            minimumPoint);

        var halfEven = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)13,
                (Fixed64)29,
                (Fixed64)(-47)),
            Vector3d.One);
        Assert.True(halfEven.TryMaterializeLocalPoint(
            new Vector3d(Fixed64.FromRaw(1L << 30), Fixed64.Zero, Fixed64.Zero),
            out Vector3d evenMidpoint));
        Assert.True(TryGetExpectedMaterialization(
            halfEven,
            new Vector3d(Fixed64.FromRaw(1L << 30), Fixed64.Zero, Fixed64.Zero),
            out Vector3d expectedEvenMidpoint));
        Assert.Equal(expectedEvenMidpoint, evenMidpoint);
        Assert.True(halfEven.TryMaterializeLocalPoint(
            new Vector3d(Fixed64.FromRaw(3L << 30), Fixed64.Zero, Fixed64.Zero),
            out Vector3d oddMidpoint));
        Assert.True(TryGetExpectedMaterialization(
            halfEven,
            new Vector3d(Fixed64.FromRaw(3L << 30), Fixed64.Zero, Fixed64.Zero),
            out Vector3d expectedOddMidpoint));
        Assert.Equal(expectedOddMidpoint, oddMidpoint);

        AssertMaterializationFailure(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right);
        AssertMaterializationFailure(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            Vector3d.Up);
        AssertMaterializationFailure(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue),
            Vector3d.Forward);
        Assert.False(new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            halfEven.Orientation,
            Vector3d.One).TryMaterializeLocalPoint(
                new Vector3d(Fixed64.FromRaw(1L), Fixed64.Zero, Fixed64.Zero),
                out Vector3d roundedOutside));
        Assert.Equal(default, roundedOutside);
        Assert.False(new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            halfEven.Orientation,
            Vector3d.One).TryMaterializeLocalPoint(
                new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue),
                out Vector3d farOutside));
        Assert.Equal(default, farOutside);
    }

    [Fact]
    public void TransformLocalOffset_UsesTheExactBasisWithoutAddingTheBoxCenter()
    {
        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)13,
            (Fixed64)29,
            (Fixed64)(-47));
        var translated = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, (Fixed64)17),
            orientation,
            Vector3d.One);
        var origin = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            Vector3d.One);
        Vector3d localOffset = new(
            Fixed64.FromRaw(1L << 30),
            Fixed64.FromRaw(-3L << 29),
            Fixed64.FromRaw(7L << 28));

        Assert.True(translated.TryTransformLocalOffset(localOffset, out Vector3d actual));
        Assert.True(origin.TryMaterializeLocalPoint(localOffset, out Vector3d expected));
        Assert.Equal(expected, actual);

        FixedQuaternion yaw = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero);
        var rotated = new FixedOrientedBox(Vector3d.Zero, yaw, Vector3d.One);
        Assert.False(rotated.TryTransformLocalOffset(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.MaxValue),
            out Vector3d outside));
        Assert.Equal(default, outside);
    }

    [Fact]
    public void SupportOffsets_StayRelativeAndSupportDifferencesNarrowOnlyOnce()
    {
        var scalarFace = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(scalarFace.TryGetSupportOffset(Vector3d.Right, out Vector3d offset));
        Assert.Equal(new Vector3d(1, -1, -1), offset);
        Assert.True(scalarFace.TryGetSupportDifference(
            scalarFace.Center,
            Vector3d.Zero,
            Vector3d.Right,
            out Vector3d difference));
        Assert.Equal(new Vector3d(1, -1, -1), difference);

        Assert.True(scalarFace.TryGetSupportDifference(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(1, -1, -1),
            Vector3d.Right,
            out Vector3d coincident));
        Assert.Equal(Vector3d.Zero, coincident);

        Assert.False(scalarFace.TryGetSupportDifference(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            -Vector3d.Right,
            Vector3d.Right,
            out Vector3d outside));
        Assert.Equal(default, outside);
    }

    [Fact]
    public void SupportOffsets_PreserveStableTiesAndRejectAnUnrepresentableRotatedOffset()
    {
        var identity = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, 3, 4));

        Assert.True(identity.TryGetSupportOffset(Vector3d.Up, out Vector3d tied));
        Assert.Equal(new Vector3d(-2, 3, -4), tied);

        FixedQuaternion yaw = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero);
        var rotated = new FixedOrientedBox(
            Vector3d.Zero,
            yaw,
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.MaxValue));
        Assert.False(rotated.TryGetSupportOffset(
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
            out Vector3d outside));
        Assert.Equal(default, outside);
    }

    [Fact]
    public void SupportDifference_BetweenBoxes_DefersBothSupportOffsetsUntilFinalNarrowing()
    {
        FixedQuaternion yaw = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero);
        Vector3d halfExtents = new(
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.MaxValue);
        var first = new FixedOrientedBox(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            yaw,
            halfExtents);
        var second = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            yaw,
            halfExtents);
        Assert.False(second.TryGetSupportOffset(
            -Vector3d.Right,
            out _));

        Assert.True(first.TryGetSupportDifference(
            second,
            Vector3d.Right,
            out Vector3d cancellation));
        Assert.True(cancellation.X > Fixed64.Zero);
        Assert.True(cancellation.X < Fixed64.MaxValue);

        var coincident = new FixedOrientedBox(
            Vector3d.Zero,
            yaw,
            halfExtents);
        Assert.False(coincident.TryGetSupportDifference(
            coincident,
            Vector3d.Right,
            out Vector3d overflow));
        Assert.Equal(default, overflow);
    }

    [Fact]
    public void Serialization_RoundTripsCanonicalState()
    {
        var box = new FixedOrientedBox(
            new Vector3d(1, -2, 3),
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)11,
                (Fixed64)23,
                (Fixed64)(-37)),
            new Vector3d(4, 5, 6));

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(box);
        Assert.Equal(box, JsonSerializer.Deserialize<FixedOrientedBox>(json));
        byte[] invalid = JsonSerializer.SerializeToUtf8Bytes(default(FixedOrientedBox));
        Assert.Throws<ArgumentException>(() =>
            JsonSerializer.Deserialize<FixedOrientedBox>(invalid));
    }

    [Fact]
    public void WarmedConstructionQueriesBoundsAndMaterialization_AllocateNoManagedMemory()
    {
        FixedQuaternion orientation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)(-31),
            (Fixed64)43);
        Vector3d center = new(3, -5, 7);
        Vector3d halfExtents = new(2, 4, 6);
        Vector3d point = new(11, -13, 17);

        int checksum = 0;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            checksum = 0;
            for (int i = 0; i < 255; i++)
                checksum ^= Exercise(center, orientation, halfExtents, point);
        });

        Assert.NotEqual(0, checksum);
        Assert.Equal(0L, allocated);
    }

    private static int Exercise(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d point)
    {
        var box = new FixedOrientedBox(center, orientation, halfExtents);
        box.GetAxes(out Vector3d axisX, out Vector3d axisY, out Vector3d axisZ);
        Vector3d corner = box.GetLocalCorner(7);
        Vector3d support = box.GetLocalSupportPoint(point);
        FixedBoundBox bounds = box.GetBoundsClippedToDomain();
        bool contains = box.Contains(point);
        bool hasClosest = box.TryGetClosestPointOnSurface(point, out Vector3d closest);
        Vector3d normal = box.GetNearestFaceNormal(point);
        bool materialized = box.TryMaterializeLocalPoint(corner, out Vector3d worldCorner);
        return axisX.GetHashCode()
            ^ axisY.GetHashCode()
            ^ axisZ.GetHashCode()
            ^ support.GetHashCode()
            ^ bounds.GetHashCode()
            ^ closest.GetHashCode()
            ^ normal.GetHashCode()
            ^ worldCorner.GetHashCode()
            ^ (contains ? 1 : 0)
            ^ (hasClosest ? 2 : 0)
            ^ (materialized ? 4 : 0);
    }

    private static void AssertMaterializationFailure(Vector3d center, Vector3d localPoint)
    {
        var box = new FixedOrientedBox(center, FixedQuaternion.Identity, Vector3d.One);

        Assert.False(box.TryMaterializeLocalPoint(localPoint, out Vector3d worldPoint));
        Assert.Equal(default, worldPoint);
    }

    private static bool ContainsExactly(FixedOrientedBox box, Vector3d point)
    {
        GetExactBasis(box.Orientation, out BigInteger denominator, out BigInteger[,] rows);
        BigInteger x = point.X.m_rawValue - (BigInteger)box.Center.X.m_rawValue;
        BigInteger y = point.Y.m_rawValue - (BigInteger)box.Center.Y.m_rawValue;
        BigInteger z = point.Z.m_rawValue - (BigInteger)box.Center.Z.m_rawValue;

        for (int row = 0; row < 3; row++)
        {
            BigInteger projection =
                (x * rows[row, 0])
                + (y * rows[row, 1])
                + (z * rows[row, 2]);
            BigInteger limit = box.HalfExtents[row].m_rawValue * denominator;
            if (BigInteger.Abs(projection) > limit)
                return false;
        }

        return true;
    }

    private static void GetExactBasis(
        FixedQuaternion orientation,
        out BigInteger denominator,
        out BigInteger[,] rows)
    {
        BigInteger x = orientation.X.m_rawValue;
        BigInteger y = orientation.Y.m_rawValue;
        BigInteger z = orientation.Z.m_rawValue;
        BigInteger w = orientation.W.m_rawValue;
        BigInteger xx = x * x;
        BigInteger yy = y * y;
        BigInteger zz = z * z;
        BigInteger ww = w * w;
        BigInteger xy = x * y;
        BigInteger xz = x * z;
        BigInteger xw = x * w;
        BigInteger yz = y * z;
        BigInteger yw = y * w;
        BigInteger zw = z * w;
        denominator = xx + yy + zz + ww;
        rows = new[,]
        {
            { xx - yy - zz + ww, 2 * (xy + zw), 2 * (xz - yw) },
            { 2 * (xy - zw), -xx + yy - zz + ww, 2 * (yz + xw) },
            { 2 * (xz + yw), 2 * (yz - xw), -xx - yy + zz + ww },
        };
    }

    private static FixedBoundBox GetExpectedBounds(FixedOrientedBox box)
    {
        GetExactBasis(box.Orientation, out BigInteger denominator, out BigInteger[,] rows);
        BigInteger xExtent = GetExtentNumerator(rows[0, 0], rows[1, 0], rows[2, 0], box.HalfExtents);
        BigInteger yExtent = GetExtentNumerator(rows[0, 1], rows[1, 1], rows[2, 1], box.HalfExtents);
        BigInteger zExtent = GetExtentNumerator(rows[0, 2], rows[1, 2], rows[2, 2], box.HalfExtents);

        return FixedBoundBox.FromMinMax(
            new Vector3d(
                FromClippedRaw(FloorRatio(
                    ((BigInteger)box.Center.X.m_rawValue * denominator) - xExtent,
                    denominator)),
                FromClippedRaw(FloorRatio(
                    ((BigInteger)box.Center.Y.m_rawValue * denominator) - yExtent,
                    denominator)),
                FromClippedRaw(FloorRatio(
                    ((BigInteger)box.Center.Z.m_rawValue * denominator) - zExtent,
                    denominator))),
            new Vector3d(
                FromClippedRaw(CeilingRatio(
                    ((BigInteger)box.Center.X.m_rawValue * denominator) + xExtent,
                    denominator)),
                FromClippedRaw(CeilingRatio(
                    ((BigInteger)box.Center.Y.m_rawValue * denominator) + yExtent,
                    denominator)),
                FromClippedRaw(CeilingRatio(
                    ((BigInteger)box.Center.Z.m_rawValue * denominator) + zExtent,
                    denominator))));
    }

    private static BigInteger GetExtentNumerator(
        BigInteger axisX,
        BigInteger axisY,
        BigInteger axisZ,
        Vector3d halfExtents)
        =>
            (BigInteger.Abs(axisX) * halfExtents.X.m_rawValue)
            + (BigInteger.Abs(axisY) * halfExtents.Y.m_rawValue)
            + (BigInteger.Abs(axisZ) * halfExtents.Z.m_rawValue);

    private static bool TryGetExpectedMaterialization(
        FixedOrientedBox box,
        Vector3d localPoint,
        out Vector3d worldPoint)
    {
        GetExactBasis(box.Orientation, out BigInteger denominator, out BigInteger[,] rows);
        bool xFits = TryRoundWorldCoordinate(
            box.Center.X,
            rows[0, 0],
            rows[1, 0],
            rows[2, 0],
            denominator,
            localPoint,
            out Fixed64 x);
        bool yFits = TryRoundWorldCoordinate(
            box.Center.Y,
            rows[0, 1],
            rows[1, 1],
            rows[2, 1],
            denominator,
            localPoint,
            out Fixed64 y);
        bool zFits = TryRoundWorldCoordinate(
            box.Center.Z,
            rows[0, 2],
            rows[1, 2],
            rows[2, 2],
            denominator,
            localPoint,
            out Fixed64 z);
        if (!(xFits & yFits & zFits))
        {
            worldPoint = default;
            return false;
        }

        worldPoint = new Vector3d(x, y, z);
        return true;
    }

    private static bool TryGetExpectedClosestPoint(
        FixedOrientedBox box,
        Vector3d point,
        out Vector3d closestPoint)
    {
        GetExactBasis(box.Orientation, out BigInteger denominator, out BigInteger[,] rows);
        BigInteger differenceX = point.X.m_rawValue - (BigInteger)box.Center.X.m_rawValue;
        BigInteger differenceY = point.Y.m_rawValue - (BigInteger)box.Center.Y.m_rawValue;
        BigInteger differenceZ = point.Z.m_rawValue - (BigInteger)box.Center.Z.m_rawValue;
        BigInteger[] projections = new BigInteger[3];
        BigInteger[] extents =
        {
            box.HalfExtents.X.m_rawValue * denominator,
            box.HalfExtents.Y.m_rawValue * denominator,
            box.HalfExtents.Z.m_rawValue * denominator,
        };
        bool outside = false;
        for (int row = 0; row < 3; row++)
        {
            projections[row] =
                (differenceX * rows[row, 0])
                + (differenceY * rows[row, 1])
                + (differenceZ * rows[row, 2]);
            if (BigInteger.Abs(projections[row]) > extents[row])
                outside = true;
        }

        BigInteger[] localNumerators = new BigInteger[3];
        for (int axis = 0; axis < 3; axis++)
        {
            localNumerators[axis] = BigInteger.Abs(projections[axis]) > extents[axis]
                ? projections[axis].Sign < 0 ? -extents[axis] : extents[axis]
                : projections[axis];
        }

        if (!outside)
        {
            int nearestAxis = 0;
            BigInteger minimumClearance = extents[0] - BigInteger.Abs(projections[0]);
            for (int axis = 1; axis < 3; axis++)
            {
                BigInteger clearance = extents[axis] - BigInteger.Abs(projections[axis]);
                if (clearance < minimumClearance)
                {
                    nearestAxis = axis;
                    minimumClearance = clearance;
                }
            }

            localNumerators[nearestAxis] = projections[nearestAxis].Sign < 0
                ? -extents[nearestAxis]
                : extents[nearestAxis];
        }

        BigInteger denominatorSquared = denominator * denominator;
        bool xFits = TryRoundClosestCoordinate(
            box.Center.X,
            rows[0, 0],
            rows[1, 0],
            rows[2, 0],
            denominatorSquared,
            localNumerators,
            out Fixed64 x);
        bool yFits = TryRoundClosestCoordinate(
            box.Center.Y,
            rows[0, 1],
            rows[1, 1],
            rows[2, 1],
            denominatorSquared,
            localNumerators,
            out Fixed64 y);
        bool zFits = TryRoundClosestCoordinate(
            box.Center.Z,
            rows[0, 2],
            rows[1, 2],
            rows[2, 2],
            denominatorSquared,
            localNumerators,
            out Fixed64 z);
        if (!(xFits & yFits & zFits))
        {
            closestPoint = default;
            return false;
        }

        closestPoint = new Vector3d(x, y, z);
        return true;
    }

    private static bool TryRoundClosestCoordinate(
        Fixed64 center,
        BigInteger axisX,
        BigInteger axisY,
        BigInteger axisZ,
        BigInteger denominatorSquared,
        BigInteger[] localNumerators,
        out Fixed64 coordinate)
    {
        BigInteger numerator = (center.m_rawValue * denominatorSquared)
            + (localNumerators[0] * axisX)
            + (localNumerators[1] * axisY)
            + (localNumerators[2] * axisZ);
        BigInteger rounded = RoundHalfEven(numerator, denominatorSquared);
        if (rounded < long.MinValue || rounded > long.MaxValue)
        {
            coordinate = default;
            return false;
        }

        coordinate = Fixed64.FromRaw((long)rounded);
        return true;
    }

    private static bool TryRoundWorldCoordinate(
        Fixed64 center,
        BigInteger axisX,
        BigInteger axisY,
        BigInteger axisZ,
        BigInteger denominator,
        Vector3d localPoint,
        out Fixed64 coordinate)
    {
        BigInteger numerator = ((BigInteger)center.m_rawValue * denominator)
            + (axisX * localPoint.X.m_rawValue)
            + (axisY * localPoint.Y.m_rawValue)
            + (axisZ * localPoint.Z.m_rawValue);
        BigInteger rounded = RoundHalfEven(numerator, denominator);
        if (rounded < long.MinValue || rounded > long.MaxValue)
        {
            coordinate = default;
            return false;
        }

        coordinate = Fixed64.FromRaw((long)rounded);
        return true;
    }

    private static BigInteger RoundHalfEven(BigInteger numerator, BigInteger denominator)
    {
        BigInteger magnitude = BigInteger.Abs(numerator);
        BigInteger quotient = BigInteger.DivRem(magnitude, denominator, out BigInteger remainder);
        int midpoint = (remainder << 1).CompareTo(denominator);
        if (midpoint > 0 || (midpoint == 0 && !quotient.IsEven))
            quotient++;
        return numerator.Sign < 0 ? -quotient : quotient;
    }

    private static BigInteger FloorRatio(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        return numerator.Sign < 0 && !remainder.IsZero ? quotient - BigInteger.One : quotient;
    }

    private static BigInteger CeilingRatio(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        return numerator.Sign > 0 && !remainder.IsZero ? quotient + BigInteger.One : quotient;
    }

    private static Fixed64 FromClippedRaw(BigInteger raw)
    {
        if (raw < long.MinValue)
            return Fixed64.MinValue;
        if (raw > long.MaxValue)
            return Fixed64.MaxValue;
        return Fixed64.FromRaw((long)raw);
    }

}
