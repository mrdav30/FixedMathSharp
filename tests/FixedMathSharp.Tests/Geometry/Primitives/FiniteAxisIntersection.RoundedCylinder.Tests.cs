using System;
using System.Numerics;
using FixedMathSharp.Bounds;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void SphericallyExpandedCylinder_DiagonalRimMissAndEndpointTangent_AreDistinct()
    {
        Fixed64 y = Fixed64.FromFraction(9, 10);
        var miss = new FixedSegment(
            new Vector3d((Fixed64)2, y, Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(9, 10), y, Fixed64.Zero));
        var tangent = new FixedSegment(
            new Vector3d((Fixed64)2, y, Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(4, 5), y, Fixed64.Zero));
        Vector3d.TryGetDistance(miss.Start, miss.End, out Fixed64 missLength);
        Vector3d.TryGetDistance(tangent.Start, tangent.End, out Fixed64 tangentLength);

        Assert.False(miss.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            missLength,
            out _,
            out _,
            out _,
            out _));
        Assert.True(tangent.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            tangentLength,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(tangentLength, entry);
        Assert.Equal(entry, exit);
        Assert.False(startContained);
    }

    [Fact]
    public void SphericallyExpandedCylinder_RimCrossing_ReturnsHalfEvenInterval()
    {
        Fixed64 y = Fixed64.FromFraction(9, 10);
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, y, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, y, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.FromFraction(6, 5), entry);
        Assert.Equal(Fixed64.Two, exit);
        Assert.False(startContained);
        Assert.True(endContainedStrict);
    }

    [Theory]
    [InlineData(3, 4)]
    [InlineData(4, 3)]
    public void SphericallyExpandedCylinder_RimCrossing_UsesSphericalCapProfile(
        long capOffset,
        long radialReach)
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)10, (Fixed64)(3 + capOffset), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(3 + capOffset), Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)3,
            (Fixed64)2,
            (Fixed64)5,
            (Fixed64)10,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal((Fixed64)(8 - radialReach), entry);
        Assert.Equal((Fixed64)10, exit);
        Assert.False(startContained);
        Assert.True(endContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_AxisParallelSweepOutsideCapCore_UsesRoundedRims()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)5, (Fixed64)(-10), Fixed64.Zero),
            new Vector3d((Fixed64)5, (Fixed64)10, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)3,
            (Fixed64)2,
            (Fixed64)5,
            (Fixed64)20,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal((Fixed64)3, entry);
        Assert.Equal((Fixed64)17, exit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_CapPlaneSweepPreservesSymmetricRoundedRimInterval()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-10), Fixed64.One, (Fixed64)5),
            new Vector3d((Fixed64)10, Fixed64.One, (Fixed64)5));
        Fixed64 radialReach = FixedMath.Sqrt((Fixed64)24);

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            (Fixed64)3,
            (Fixed64)4,
            (Fixed64)20,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal((Fixed64)10 - radialReach, entry);
        Assert.Equal((Fixed64)10 + radialReach, exit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_DegenerateQuarticPreservesRoundedCapExit()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)4, (Fixed64)3, Fixed64.Zero),
            new Vector3d((Fixed64)4, (Fixed64)10, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)3,
            (Fixed64)3,
            (Fixed64)5,
            (Fixed64)7,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(FixedMath.Sqrt((Fixed64)24), exit);
        Assert.True(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_RepeatedAxisRimRootPreservesAxialInterval()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-5), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)5, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)3,
            Fixed64.One,
            Fixed64.One,
            (Fixed64)10,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal((Fixed64)9, exit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_PerfectSquareRimPolynomialPreservesAxialInterval()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-10), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)10, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)3,
            (Fixed64)3,
            (Fixed64)5,
            (Fixed64)20,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal((Fixed64)2, entry);
        Assert.Equal((Fixed64)18, exit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_LinearRemainderRimExitIsDirectionSymmetric()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-12), (Fixed64)(-29), Fixed64.Zero),
            new Vector3d((Fixed64)(-24), (Fixed64)(-14), Fixed64.Zero));
        var reverse = new FixedSegment(query.End, query.Start);
        Assert.True(Vector3d.TryGetDistance(query.Start, query.End, out Fixed64 totalDistance));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)20,
            Fixed64.One,
            (Fixed64)15,
            totalDistance,
            out Fixed64 forwardEntry,
            out Fixed64 forwardExit,
            out bool forwardStartContained,
            out bool forwardEndContainedStrict));
        Assert.Equal(Fixed64.Zero, forwardEntry);
        Assert.True(forwardExit > (Fixed64)4);
        Assert.True(forwardExit < (Fixed64)5);
        Assert.True(forwardStartContained);
        Assert.False(forwardEndContainedStrict);

        Assert.True(reverse.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)20,
            Fixed64.One,
            (Fixed64)15,
            totalDistance,
            out Fixed64 reverseEntry,
            out Fixed64 reverseExit,
            out bool reverseStartContained,
            out bool reverseEndContainedStrict));
        Assert.Equal(totalDistance - forwardExit, reverseEntry);
        Assert.Equal(totalDistance, reverseExit);
        Assert.False(reverseStartContained);
        Assert.True(reverseEndContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_ConstantRemainderRimEntryIsDirectionSymmetric()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-7), (Fixed64)(-24), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, (Fixed64)(-20), (Fixed64)(-1)));
        var reverse = new FixedSegment(query.End, query.Start);
        Assert.True(Vector3d.TryGetDistance(query.Start, query.End, out Fixed64 totalDistance));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)20,
            Fixed64.One,
            Fixed64.One,
            totalDistance,
            out Fixed64 forwardEntry,
            out Fixed64 forwardExit,
            out bool forwardStartContained,
            out bool forwardEndContainedStrict));
        Assert.True(forwardEntry > (Fixed64)6);
        Assert.True(forwardEntry < (Fixed64)7);
        Assert.Equal(totalDistance, forwardExit);
        Assert.False(forwardStartContained);
        Assert.True(forwardEndContainedStrict);

        Assert.True(reverse.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)20,
            Fixed64.One,
            Fixed64.One,
            totalDistance,
            out Fixed64 reverseEntry,
            out Fixed64 reverseExit,
            out bool reverseStartContained,
            out bool reverseEndContainedStrict));
        Assert.Equal(Fixed64.Zero, reverseEntry);
        Assert.Equal(totalDistance - forwardEntry, reverseExit);
        Assert.True(reverseStartContained);
        Assert.False(reverseEndContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_RepeatedLinearRemainderPreservesCapCoreInterval()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-28), (Fixed64)31, Fixed64.Zero),
            new Vector3d((Fixed64)(-12), (Fixed64)19, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)20,
            (Fixed64)15,
            Fixed64.One,
            (Fixed64)20,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.FromFraction(50, 3), entry);
        Assert.Equal((Fixed64)20, exit);
        Assert.False(startContained);
        Assert.True(endContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_AxialCapCoreEntryPrecedesRoundedRimEntry()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)10, Fixed64.Zero),
            Vector3d.Zero);

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)3,
            (Fixed64)2,
            (Fixed64)5,
            (Fixed64)10,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal((Fixed64)2, entry);
        Assert.Equal((Fixed64)10, exit);
        Assert.False(startContained);
        Assert.True(endContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_DisjointCapCoreIntervalsRemainAMiss()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)20, Fixed64.Zero),
            new Vector3d((Fixed64)20, Fixed64.Zero, Fixed64.Zero));
        Assert.True(Vector3d.TryGetDistance(query.Start, query.End, out Fixed64 totalDistance));

        Assert.False(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)3,
            (Fixed64)2,
            (Fixed64)5,
            totalDistance,
            out _,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void SphericallyExpandedCylinder_StationaryPointsDistinguishInteriorBoundaryAndRoundedCornerMiss()
    {
        var interior = new FixedSegment(
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2), Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2), Fixed64.Zero));
        var boundary = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.Zero));
        var roundedCornerMiss = new FixedSegment(
            new Vector3d(Fixed64.Two, Fixed64.Two, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.Two, Fixed64.Zero));

        Assert.True(interior.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One,
            out Fixed64 interiorEntry, out Fixed64 interiorExit,
            out bool interiorStartContained, out bool interiorEndContainedStrict));
        Assert.Equal(Fixed64.Zero, interiorEntry);
        Assert.Equal(Fixed64.One, interiorExit);
        Assert.True(interiorStartContained);
        Assert.True(interiorEndContainedStrict);

        Assert.True(boundary.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One,
            out Fixed64 boundaryEntry, out Fixed64 boundaryExit,
            out bool boundaryStartContained, out bool boundaryEndContainedStrict));
        Assert.Equal(Fixed64.Zero, boundaryEntry);
        Assert.Equal(Fixed64.One, boundaryExit);
        Assert.True(boundaryStartContained);
        Assert.False(boundaryEndContainedStrict);

        Assert.False(roundedCornerMiss.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One,
            out _, out _, out _, out _));

        Assert.True(boundary.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.Zero,
            out boundaryEntry, out boundaryExit,
            out boundaryStartContained, out boundaryEndContainedStrict));
        Assert.Equal(Fixed64.Zero, boundaryEntry);
        Assert.Equal(Fixed64.Zero, boundaryExit);
        Assert.True(boundaryStartContained);
        Assert.False(boundaryEndContainedStrict);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    public void SphericallyExpandedCylinder_HalfRawRimRootsRoundToEven(
        long totalDistanceRaw,
        long expectedRootRaw)
    {
        var entering = new FixedSegment(
            new Vector3d((Fixed64)3, Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));
        var exiting = new FixedSegment(entering.End, entering.Start);
        Fixed64 totalDistance = Fixed64.FromRaw(totalDistanceRaw);
        Fixed64 expectedRoot = Fixed64.FromRaw(expectedRootRaw);

        Assert.True(entering.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, Fixed64.One, totalDistance,
            out Fixed64 entry, out Fixed64 enteringExit,
            out bool enteringStartContained, out bool enteringEndContainedStrict));
        Assert.Equal(expectedRoot, entry);
        Assert.Equal(totalDistance, enteringExit);
        Assert.False(enteringStartContained);
        Assert.True(enteringEndContainedStrict);

        Assert.True(exiting.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, Fixed64.One, totalDistance,
            out Fixed64 exitingEntry, out Fixed64 exit,
            out bool exitingStartContained, out bool exitingEndContainedStrict));
        Assert.Equal(Fixed64.Zero, exitingEntry);
        Assert.Equal(expectedRoot, exit);
        Assert.True(exitingStartContained);
        Assert.False(exitingEndContainedStrict);
    }

    [Fact]
    public void SphericallyExpandedCylinder_CapCoreIntervalsThatOnlyRoundTogether_DoNotCreateAHit()
    {
        Fixed64 radius = Fixed64.FromRaw(5L);
        var query = new FixedSegment(
            new Vector3d(
                Fixed64.FromRaw(9L),
                radius,
                Fixed64.Zero),
            new Vector3d(
                Fixed64.FromRaw(2L),
                Fixed64.FromRaw(7L),
                Fixed64.Zero));
        Assert.True(Vector3d.TryGetDistance(query.Start, query.End, out Fixed64 totalDistance));
        Assert.Equal(Fixed64.FromRaw(7L), totalDistance);

        Assert.False(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            radius,
            radius,
            Fixed64.FromRaw(1L),
            totalDistance,
            out _,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void SphericallyExpandedCylinder_InteriorRimTangency_IsNotLostBetweenRawDistances()
    {
        Fixed64 radius = Fixed64.FromRaw(5L);
        var query = new FixedSegment(
            new Vector3d(Fixed64.FromRaw(8L), Fixed64.FromRaw(9L), Fixed64.FromRaw(-1L)),
            new Vector3d(Fixed64.FromRaw(8L), Fixed64.FromRaw(9L), Fixed64.FromRaw(1L)));
        Fixed64 totalDistance = Fixed64.FromRaw(2L);

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            radius,
            radius,
            radius,
            totalDistance,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.FromRaw(1L), entry);
        Assert.Equal(entry, exit);
        Assert.False(startContained);
        Assert.False(endContainedStrict);

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            radius,
            radius,
            radius,
            totalDistance,
            out Fixed64 repeatedEntry,
            out Fixed64 repeatedExit,
            out _,
            out _));
        Assert.Equal(entry, repeatedEntry);
        Assert.Equal(exit, repeatedExit);
    }

    [Fact]
    public void SphericallyExpandedCylinder_StartOnRimMovingOutward_ReturnsZeroLengthBoundaryInterval()
    {
        Fixed64 radius = Fixed64.FromRaw(5L);
        var query = new FixedSegment(
            new Vector3d(Fixed64.FromRaw(8L), Fixed64.FromRaw(9L), Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(9L), Fixed64.FromRaw(9L), Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            radius,
            radius,
            radius,
            Fixed64.FromRaw(1L),
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
    public void SphericallyExpandedCylinder_SymmetricRimDepartureRetainsBoundaryContact()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)3, (Fixed64)3, Fixed64.Zero),
            new Vector3d((Fixed64)3, (Fixed64)3, Fixed64.One));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)3,
            (Fixed64)2,
            Fixed64.One,
            Fixed64.One,
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
    public void SphericallyExpandedCylinder_FirstDistanceMatchesIntervalEntry()
    {
        Fixed64 y = Fixed64.FromFraction(9, 10);
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, y, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, y, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistance(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Two,
            out Fixed64 firstDistance));
        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Two,
            out Fixed64 intervalEntry,
            out _,
            out _,
            out _));
        Assert.Equal(intervalEntry, firstDistance);
    }

    [Fact]
    public void SphericallyExpandedCylinder_ZeroRadiusReducesToCapsule()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit,
            out _,
            out _));
        Assert.Equal(Fixed64.FromFraction(3, 2), entry);
        Assert.Equal(Fixed64.FromFraction(5, 2), exit);
    }

    [Fact]
    public void SphericallyExpandedCylinder_ZeroExpansionRetainsFlatCylinderContract()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit,
            out _,
            out _));
        Assert.Equal(Fixed64.FromFraction(3, 2), entry);
        Assert.Equal(Fixed64.FromFraction(5, 2), exit);
    }

    [Fact]
    public void SphericallyExpandedCylinder_FullDomainTranslationPreservesRimInterval()
    {
        Fixed64 centerX = Fixed64.MaxValue - (Fixed64)4;
        Vector3d center = new(centerX, Fixed64.Zero, Fixed64.Zero);
        Fixed64 y = Fixed64.FromFraction(9, 10);
        var query = new FixedSegment(
            new Vector3d(centerX + (Fixed64)2, y, Fixed64.Zero),
            new Vector3d(centerX, y, Fixed64.Zero));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            center,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit,
            out _,
            out _));
        Assert.Equal(Fixed64.FromFraction(6, 5), entry);
        Assert.Equal(Fixed64.Two, exit);
    }

    [Fact]
    public void SphericallyExpandedCylinder_NearUnitRotatedAxisPreservesRimEntry()
    {
        Vector3d axis = new(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5),
            Fixed64.Zero);
        Vector3d perpendicular = new(-axis.Y, axis.X, Fixed64.Zero);
        Assert.True(axis.IsNormalized());
        Assert.NotEqual(
            (BigInteger)Fixed64.One.m_rawValue * Fixed64.One.m_rawValue,
            (BigInteger)axis.X.m_rawValue * axis.X.m_rawValue
            + (BigInteger)axis.Y.m_rawValue * axis.Y.m_rawValue);
        Vector3d Transform(Vector3d point) =>
            perpendicular * point.X + axis * point.Y + Vector3d.Forward * point.Z;

        var query = new FixedSegment(
            Transform(new Vector3d((Fixed64)2, Fixed64.FromFraction(9, 10), Fixed64.Zero)),
            Transform(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(9, 10), Fixed64.Zero)));

        Assert.True(query.TryGetSweptSphereFiniteCylinderIntersectionDistance(
            Vector3d.Zero,
            axis,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Two,
            out Fixed64 entry));
        Assert.InRange(
            (entry - Fixed64.FromFraction(6, 5)).Abs().m_rawValue,
            0L,
            4L);
    }

    [Fact]
    public void SphericallyExpandedCylinder_OppositeScalarExtremesRemainASeparatedMiss()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.MinValue + (Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue + (Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        Vector3d center = new(Fixed64.MaxValue - (Fixed64)4, Fixed64.Zero, Fixed64.Zero);

        Assert.False(query.TryGetSweptSphereFiniteCylinderIntersectionDistance(
            center,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            (Fixed64)4,
            out _));
    }

    [Fact]
    public void SphericallyExpandedCylinder_RejectsInvalidAuthoredParameters()
    {
        var query = new FixedSegment(Vector3d.Zero, Vector3d.One);

        Assert.Throws<ArgumentException>(() => query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One, Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One, Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));
    }
}
