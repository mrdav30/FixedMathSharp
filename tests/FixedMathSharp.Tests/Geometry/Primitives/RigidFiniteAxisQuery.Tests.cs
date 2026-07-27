//=======================================================================
// RigidFiniteAxisQuery.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class RigidFiniteAxisQueryTests
{
    [Fact]
    public void CapsuleInterval_UsesTheRigidFrameWithAnOddRawAxisLength()
    {
        Fixed64 length = Fixed64.FromRaw(((long)2 << FixedMath.SHIFT_AMOUNT_I) + 1L);
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        bool found = query.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            length,
            Fixed64.One,
            Fixed64.Zero,
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict);

        Assert.True(found);
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal((Fixed64)3, exit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void CylinderInterval_RetainsAQuaternionRigidFrameNearTheScalarFace()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)29,
            (Fixed64)11);
        Vector3d translation = new(
            Fixed64.MaxValue - (Fixed64)8,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d localStart = new((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero);
        Vector3d localEnd = new((Fixed64)2, Fixed64.Zero, Fixed64.Zero);
        var query = new FixedSegment(
            translation + rotation * localStart,
            translation + rotation * localEnd);

        bool found = query.TryGetFiniteCylinderIntersectionDistanceInterval(
            translation,
            rotation,
            Fixed64.FromRaw(((long)2 << FixedMath.SHIFT_AMOUNT_I) + 1L),
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict);

        Assert.True(found);
        Assert.True((entry - Fixed64.One).Abs() <= Fixed64.Epsilon);
        Assert.True((exit - (Fixed64)3).Abs() <= Fixed64.Epsilon);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void ConeInterval_PreservesTheBaseAndApexAsymmetry()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));

        bool found = query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            (Fixed64)2,
            Fixed64.One,
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict);

        Assert.True(found);
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal((Fixed64)3, exit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void ConeInterval_WithFullDomainNonCardinalCrossingRetainsTheMidpointHit()
    {
        Fixed64 halfDistance = (Fixed64)1_000_000_000;
        Fixed64 totalDistance = (Fixed64)2_000_000_000;
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)29,
            (Fixed64)11);
        var query = new FixedSegment(
            new Vector3d(-halfDistance, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(halfDistance, Fixed64.Zero, Fixed64.Zero));

        bool found = query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            totalDistance,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict);

        Assert.True(found);
        Assert.True(entry < halfDistance);
        Assert.True(exit > halfDistance);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void ConeInterval_WithOddRawDimensionsMirrorsAcrossAReversedFullDomainChord()
    {
        Fixed64 halfDistance = (Fixed64)1_000_000_000;
        Fixed64 totalDistance = (Fixed64)2_000_000_000;
        Fixed64 height = Fixed64.FromRaw(
            ((long)2 << FixedMath.SHIFT_AMOUNT_I) + 1L);
        Fixed64 radius = Fixed64.FromRaw(
            Fixed64.One.m_rawValue + 1L);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)29,
            (Fixed64)11);
        var forward = new FixedSegment(
            new Vector3d(-halfDistance, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(halfDistance, Fixed64.Zero, Fixed64.Zero));
        var reverse = new FixedSegment(forward.End, forward.Start);

        Assert.True(forward.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            height,
            radius,
            totalDistance,
            out Fixed64 forwardEntry,
            out Fixed64 forwardExit,
            out _,
            out _));
        Assert.True(reverse.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            height,
            radius,
            totalDistance,
            out Fixed64 reverseEntry,
            out Fixed64 reverseExit,
            out _,
            out _));

        Assert.Equal(totalDistance - forwardExit, reverseEntry);
        Assert.Equal(totalDistance - forwardEntry, reverseExit);
    }

    [Fact]
    public void RigidIntervals_CertifyEveryCardinalLocalYAxis()
    {
        Fixed64 halfRoot = FixedMath.Sqrt(Fixed64.Half);
        FixedQuaternion[] rotations =
        {
            new(Fixed64.Zero, Fixed64.Zero, -halfRoot, halfRoot),
            new(Fixed64.Zero, Fixed64.Zero, halfRoot, halfRoot),
            FixedQuaternion.Identity,
            new(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new(halfRoot, Fixed64.Zero, Fixed64.Zero, halfRoot),
            new(-halfRoot, Fixed64.Zero, Fixed64.Zero, halfRoot),
        };
        Vector3d[] axes =
        {
            Vector3d.Right,
            Vector3d.Left,
            Vector3d.Up,
            Vector3d.Down,
            Vector3d.Forward,
            Vector3d.Backward,
        };
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-3), (Fixed64)(-3), (Fixed64)(-3)),
            new Vector3d((Fixed64)3, (Fixed64)3, (Fixed64)3));

        for (int i = 0; i < rotations.Length; i++)
        {
            bool rigidFound = query.TryGetCapsuleIntersectionDistanceInterval(
                Vector3d.Zero,
                rotations[i],
                (Fixed64)2,
                Fixed64.One,
                Fixed64.Quarter,
                (Fixed64)9,
                out Fixed64 rigidEntry,
                out Fixed64 rigidExit,
                out bool rigidStartContained,
                out bool rigidEndContained);
            bool axisFound = query.TryGetCapsuleIntersectionDistanceInterval(
                Vector3d.Zero,
                axes[i],
                (Fixed64)2,
                Fixed64.One,
                Fixed64.Quarter,
                (Fixed64)9,
                out Fixed64 axisEntry,
                out Fixed64 axisExit,
                out bool axisStartContained,
                out bool axisEndContained);

            Assert.Equal(axisFound, rigidFound);
            Assert.Equal(axisEntry, rigidEntry);
            Assert.Equal(axisExit, rigidExit);
            Assert.Equal(axisStartContained, rigidStartContained);
            Assert.Equal(axisEndContained, rigidEndContained);

            rigidFound = query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
                Vector3d.Zero,
                rotations[i],
                (Fixed64)2,
                Fixed64.One,
                (Fixed64)9,
                out rigidEntry,
                out rigidExit,
                out rigidStartContained,
                out rigidEndContained);
            axisFound = query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
                Vector3d.Zero,
                axes[i],
                (Fixed64)2,
                Fixed64.One,
                (Fixed64)9,
                out axisEntry,
                out axisExit,
                out axisStartContained,
                out axisEndContained);

            Assert.Equal(axisFound, rigidFound);
            Assert.Equal(axisEntry, rigidEntry);
            Assert.Equal(axisExit, rigidExit);
            Assert.Equal(axisStartContained, rigidStartContained);
            Assert.Equal(axisEndContained, rigidEndContained);
        }

        Assert.True(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Quarter,
            Fixed64.Half,
            (Fixed64)9,
            out _,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void RigidIntervals_RejectAxiallySeparatedSegments()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)29,
            (Fixed64)11);
        var query = new FixedSegment(
            rotation * new Vector3d((Fixed64)(-2), (Fixed64)4, Fixed64.Zero),
            rotation * new Vector3d((Fixed64)2, (Fixed64)4, Fixed64.Zero));

        Assert.False(query.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)4,
            out _,
            out _,
            out bool capsuleStartContained,
            out bool capsuleEndContained));
        Assert.False(capsuleStartContained);
        Assert.False(capsuleEndContained);

        Assert.False(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)4,
            out _,
            out _,
            out bool cylinderStartContained,
            out bool cylinderEndContained));
        Assert.False(cylinderStartContained);
        Assert.False(cylinderEndContained);

        Assert.False(query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            (Fixed64)4,
            out _,
            out _,
            out bool coneStartContained,
            out bool coneEndContained));
        Assert.False(coneStartContained);
        Assert.False(coneEndContained);
    }

    [Fact]
    public void RigidIntervals_ReportClosedStartAndStrictEndContainment()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)29,
            (Fixed64)11);
        var query = new FixedSegment(
            rotation * Vector3d.Zero,
            rotation * new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero));

        Assert.True(query.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Quarter,
            out Fixed64 capsuleEntry,
            out Fixed64 capsuleExit,
            out bool capsuleStartContained,
            out bool capsuleEndContained));
        Assert.Equal(Fixed64.Zero, capsuleEntry);
        Assert.Equal(Fixed64.Quarter, capsuleExit);
        Assert.True(capsuleStartContained);
        Assert.True(capsuleEndContained);

        Assert.True(query.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Quarter,
            out Fixed64 cylinderEntry,
            out Fixed64 cylinderExit,
            out bool cylinderStartContained,
            out bool cylinderEndContained));
        Assert.Equal(Fixed64.Zero, cylinderEntry);
        Assert.Equal(Fixed64.Quarter, cylinderExit);
        Assert.True(cylinderStartContained);
        Assert.True(cylinderEndContained);

        Assert.True(query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Quarter,
            out Fixed64 coneEntry,
            out Fixed64 coneExit,
            out bool coneStartContained,
            out bool coneEndContained));
        Assert.Equal(Fixed64.Zero, coneEntry);
        Assert.Equal(Fixed64.Quarter, coneExit);
        Assert.True(coneStartContained);
        Assert.True(coneEndContained);
    }

    [Fact]
    public void CapsuleInterval_MergesBothCapsWithTheFiniteSide()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)29,
            (Fixed64)11);
        var query = new FixedSegment(
            rotation * new Vector3d(Fixed64.Half, (Fixed64)(-3), Fixed64.Zero),
            rotation * new Vector3d(Fixed64.Half, (Fixed64)3, Fixed64.Zero));

        Assert.True(query.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Zero,
            (Fixed64)6,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.True(entry > Fixed64.Zero);
        Assert.True(exit < (Fixed64)6);
        Assert.True(entry < (Fixed64)2);
        Assert.True(exit > (Fixed64)4);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void RigidPointIntervals_ClassifyAxialAndCapsuleCapRegions()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)29,
            (Fixed64)11);

        AssertCapsulePointContained(rotation, (Fixed64)(-1.5m));
        AssertCapsulePointContained(rotation, (Fixed64)1.5m);
        AssertCylinderPoint(rotation, Fixed64.Zero, expected: true);
        AssertCylinderPoint(rotation, (Fixed64)(-3), expected: false);
        AssertCylinderPoint(rotation, (Fixed64)3, expected: false);
    }

    [Fact]
    public void RigidIntervals_ValidateAuthoredContracts()
    {
        var point = new FixedSegment(Vector3d.Zero, Vector3d.Zero);
        Assert.Throws<ArgumentException>(() =>
            point.TryGetCapsuleIntersectionDistanceInterval(
                Vector3d.Zero,
                FixedQuaternion.Zero,
                Fixed64.One,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            point.TryGetCapsuleIntersectionDistanceInterval(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                -Fixed64.One,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            point.TryGetFiniteCylinderIntersectionDistanceInterval(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            point.TryGetFiniteCylinderIntersectionDistanceInterval(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                Fixed64.One,
                Fixed64.Zero,
                -Fixed64.MinIncrement,
                Fixed64.Zero,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            point.TryGetCenteredFiniteConeIntersectionDistanceInterval(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                -Fixed64.MinIncrement,
                Fixed64.Zero,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            point.TryGetCapsuleIntersectionDistanceInterval(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                Fixed64.One,
                -Fixed64.MinIncrement,
                Fixed64.Zero,
                out _,
                out _,
                out _,
                out _));
    }

    [Fact]
    public void RigidIntervals_DoNotAllocateAfterWarmup()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)29,
            (Fixed64)11);
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        QueryAll(query, rotation);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++)
            QueryAll(query, rotation);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0L, allocated);
    }

    private static void QueryAll(
        FixedSegment query,
        FixedQuaternion rotation)
    {
        _ = query.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Zero,
            (Fixed64)6,
            out _,
            out _,
            out _,
            out _);
        _ = query.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)6,
            out _,
            out _,
            out _,
            out _);
        _ = query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            (Fixed64)6,
            out _,
            out _,
            out _,
            out _);
    }

    private static void AssertCapsulePointContained(
        FixedQuaternion rotation,
        Fixed64 localY)
    {
        Vector3d point = rotation * new Vector3d(
            Fixed64.Zero,
            localY,
            Fixed64.Zero);
        var query = new FixedSegment(point, point);

        Assert.True(query.TryGetCapsuleIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Zero, exit);
        Assert.True(startContained);
        Assert.True(endContainedStrict);
    }

    private static void AssertCylinderPoint(
        FixedQuaternion rotation,
        Fixed64 localY,
        bool expected)
    {
        Vector3d point = rotation * new Vector3d(
            Fixed64.Zero,
            localY,
            Fixed64.Zero);
        var query = new FixedSegment(point, point);

        Assert.Equal(expected, query.TryGetFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            rotation,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(expected, startContained);
        Assert.Equal(expected, endContainedStrict);
    }
}
