using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void CenteredFiniteCylinderContainment_DistinguishesInclusiveAndStrictBoundaries()
    {
        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One));
        Assert.False(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            strict: true));
        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One));
        Assert.False(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            strict: true));
        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            strict: true));
    }

    [Fact]
    public void CenteredFiniteCylinderContainment_FullDomainConceptualAxisDoesNotNarrowCaps()
    {
        Vector3d axis = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        var center = new Vector3d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero, Fixed64.Zero);

        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(Fixed64.MaxValue, (Fixed64)7, Fixed64.Zero),
            center,
            axis,
            (Fixed64)20,
            (Fixed64)2,
            strict: true));
        Assert.False(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(Fixed64.MaxValue, (Fixed64)9, Fixed64.Zero),
            center,
            axis,
            (Fixed64)20,
            (Fixed64)2));
    }

    [Fact]
    public void CenteredFiniteCylinderContainment_RejectsInvalidAuthoredParameters()
    {
        Assert.Throws<ArgumentException>(() => FixedSegment.ContainsPointInCenteredFiniteCylinder(
            Vector3d.Zero, Vector3d.Zero, Vector3d.One, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.ContainsPointInCenteredFiniteCylinder(
            Vector3d.Zero, Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.ContainsPointInCenteredFiniteCylinder(
            Vector3d.Zero, Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One));
    }

    [Fact]
    public void CenteredFiniteCylinder_OrdinaryCrossing_MatchesRepresentableEndpointContract()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit));
        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 endpointEntry,
            out Fixed64 endpointExit));
        Assert.Equal(endpointEntry, centeredEntry);
        Assert.Equal(endpointExit, centeredExit);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CenteredFiniteCylinder_ScalarEdgeCap_RemainsRepresentable(bool positive)
    {
        Fixed64 centerY = positive
            ? Fixed64.MaxValue - Fixed64.One
            : Fixed64.MinValue + Fixed64.One;
        Fixed64 outsideY = positive
            ? Fixed64.MaxValue - (Fixed64)5
            : Fixed64.MinValue + (Fixed64)5;
        Fixed64 capY = positive
            ? Fixed64.MaxValue - (Fixed64)4
            : Fixed64.MinValue + (Fixed64)4;
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, outsideY, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, capY, Fixed64.Zero));

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            new Vector3d(Fixed64.Zero, centerY, Fixed64.Zero),
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal(entry, exit);
    }

    [Fact]
    public void CenteredFiniteCylinder_RadialTangentAndAdjacentMiss_AreDistinct()
    {
        var tangent = new FixedSegment(
            new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)(-2)),
            new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)2));
        var miss = new FixedSegment(
            new Vector3d(Fixed64.One + Fixed64.FromRaw(1), Fixed64.Zero, (Fixed64)(-2)),
            new Vector3d(Fixed64.One + Fixed64.FromRaw(1), Fixed64.Zero, (Fixed64)2));

        Assert.True(tangent.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(entry, exit);
        Assert.False(miss.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _));
    }

    [Fact]
    public void CenteredFiniteCylinder_AdvancedClassification_UsesExactBoundaries()
    {
        var query = new FixedSegment(
            Vector3d.Right,
            Vector3d.Zero);

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.One, exit);
        Assert.True(startContained);
        Assert.True(endContainedStrict);
    }

    [Fact]
    public void CenteredFiniteCylinder_AxialParallelAndDisjointBranchesRemainExact()
    {
        var axial = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        var disjoint = new FixedSegment(
            new Vector3d((Fixed64)(-2), (Fixed64)3, Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)3, Fixed64.Zero));

        Assert.True(axial.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 3), entry);
        Assert.Equal(Fixed64.FromFraction(2, 3), exit);
        Assert.False(disjoint.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _));
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    public void CenteredFiniteCylinder_AxialHalfRawTieRoundsToEven(long capStartRaw, long expectedEntryRaw)
    {
        var query = new FixedSegment(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, Fixed64.Two, Fixed64.Zero));
        Vector3d center = new(
            Fixed64.Zero,
            Fixed64.One + Fixed64.FromRaw(capStartRaw),
            Fixed64.Zero);

        Assert.True(query.TryGetFiniteCylinderIntersectionInterval(
            center,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out _));
        Assert.Equal(Fixed64.FromRaw(expectedEntryRaw), entry);
    }

    [Fact]
    public void CenteredFiniteCylinder_RejectsInvalidDirectionAndDimensions()
    {
        var query = new FixedSegment(Vector3d.Zero, Vector3d.One);

        Assert.Throws<ArgumentException>(() => query.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.Zero, out _, out _));
        Assert.Throws<ArgumentException>(() => query.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up * Fixed64.Two, Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.Zero, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One, Fixed64.Zero, Fixed64.Zero, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, -Fixed64.One, Fixed64.Zero, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, Fixed64.Zero, -Fixed64.One, out _, out _));
    }
}
