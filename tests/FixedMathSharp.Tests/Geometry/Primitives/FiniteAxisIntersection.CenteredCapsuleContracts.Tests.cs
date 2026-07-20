using System;
using FixedMathSharp.Bounds;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void CenteredCapsuleContainment_StrictModeExcludes2dAnd3dBoundaries()
    {
        var side2d = new Vector2d(Fixed64.One, Fixed64.Zero);
        var tip2d = new Vector2d(Fixed64.Zero, (Fixed64)2);
        var interior2d = new Vector2d(Fixed64.Half, Fixed64.Zero);
        var side3d = new Vector3d(side2d.X, side2d.Y, Fixed64.Zero);
        var tip3d = new Vector3d(tip2d.X, tip2d.Y, Fixed64.Zero);
        var interior3d = new Vector3d(interior2d.X, interior2d.Y, Fixed64.Zero);

        Assert.True(FixedSegment2d.ContainsPointInCenteredCapsule(
            side2d, Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One));
        Assert.False(FixedSegment2d.ContainsPointInCenteredCapsule(
            side2d, Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One, strict: true));
        Assert.False(FixedSegment2d.ContainsPointInCenteredCapsule(
            tip2d, Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One, strict: true));
        Assert.True(FixedSegment2d.ContainsPointInCenteredCapsule(
            interior2d, Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One, strict: true));

        Assert.True(FixedSegment.ContainsPointInCenteredCapsule(
            side3d, Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One));
        Assert.False(FixedSegment.ContainsPointInCenteredCapsule(
            side3d, Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, strict: true));
        Assert.False(FixedSegment.ContainsPointInCenteredCapsule(
            tip3d, Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, strict: true));
        Assert.True(FixedSegment.ContainsPointInCenteredCapsule(
            interior3d, Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, strict: true));
    }

    [Fact]
    public void CenteredCapsuleContainment_StrictModeRetainsRadialExpansionOverload()
    {
        var boundary2d = new Vector2d((Fixed64)2, Fixed64.Zero);
        var boundary3d = new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero);

        Assert.True(FixedSegment2d.ContainsPointInCenteredCapsule(
            boundary2d,
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            strict: false));
        Assert.False(FixedSegment2d.ContainsPointInCenteredCapsule(
            boundary2d,
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            strict: true));
        Assert.True(FixedSegment.ContainsPointInCenteredCapsule(
            boundary3d,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            strict: false));
        Assert.False(FixedSegment.ContainsPointInCenteredCapsule(
            boundary3d,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            strict: true));
    }

    [Fact]
    public void CenteredCapsule2d_ConvenienceOverloadsPreserveExactIntervals()
    {
        var query = new FixedSegment2d(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero));

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 3), entry);
        Assert.Equal(Fixed64.FromFraction(2, 3), exit);

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            out entry,
            out exit));
        Assert.Equal(Fixed64.FromFraction(1, 6), entry);
        Assert.Equal(Fixed64.FromFraction(5, 6), exit);
    }

    [Fact]
    public void CenteredCapsule2d_RejectsEveryInvalidAuthoredParameter()
    {
        var query = new FixedSegment2d(Vector2d.Zero, Vector2d.One);

        Assert.Throws<ArgumentException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Zero, Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Forward, -Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, -Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.One,
            -Fixed64.One,
            out _,
            out _));
    }

    [Fact]
    public void CenteredCapsuleHelpers2d_RejectInvalidGeometryContracts()
    {
        Assert.Throws<ArgumentException>(() => FixedSegment2d.GetDirectionFromCenteredAxis(
            Vector2d.One, Vector2d.Zero, Vector2d.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.GetDirectionFromCenteredAxis(
            Vector2d.One, Vector2d.Zero, Vector2d.Forward, -Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.TryGetDistanceToCenteredCapsule(
            Vector2d.One, Vector2d.Zero, Vector2d.Forward, Fixed64.One, -Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.ContainsPointInCenteredCapsule(
            Vector2d.One, Vector2d.Zero, Vector2d.Forward, Fixed64.One, -Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.ContainsPointInCenteredCapsule(
            Vector2d.One, Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One, -Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            Vector2d.One,
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            -Fixed64.One,
            Vector2d.Right,
            out _));
        Assert.Throws<ArgumentException>(() => FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            Vector2d.One,
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.One,
            Vector2d.Zero,
            out _));
    }

    [Fact]
    public void CenteredCapsuleHelpers3d_RejectInvalidGeometryContracts()
    {
        Assert.Throws<ArgumentException>(() => FixedSegment.GetDirectionFromCenteredAxis(
            Vector3d.One, Vector3d.Zero, Vector3d.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.GetDirectionFromCenteredAxis(
            Vector3d.One, Vector3d.Zero, Vector3d.Up, -Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetDistanceToCenteredCapsule(
            Vector3d.One, Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.ContainsPointInCenteredCapsule(
            Vector3d.One, Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.ContainsPointInCenteredCapsule(
            Vector3d.One, Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, -Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetSurfacePointOnCenteredCapsule(
            Vector3d.One,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            -Fixed64.One,
            Vector3d.Right,
            out _));
        Assert.Throws<ArgumentException>(() => FixedSegment.TryGetSurfacePointOnCenteredCapsule(
            Vector3d.One,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Vector3d.Zero,
            out _));
    }

    [Fact]
    public void CenteredCapsuleSurfacePoint_ReportsEachUnrepresentableCoordinate()
    {
        Assert.False(FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue - Fixed64.One, Fixed64.Zero),
            Vector2d.Right,
            Fixed64.One,
            Fixed64.One,
            Vector2d.Right,
            out Vector2d surface2d));
        Assert.Equal(Vector2d.Zero, surface2d);
        Assert.False(FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            new Vector2d(Fixed64.Zero, Fixed64.MaxValue),
            new Vector2d(Fixed64.Zero, Fixed64.MaxValue - Fixed64.One),
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.One,
            Vector2d.Forward,
            out surface2d));
        Assert.Equal(Vector2d.Zero, surface2d);

        Assert.False(TryGetUnrepresentableSurface3d(Vector3d.Right, out Vector3d surface3d));
        Assert.Equal(Vector3d.Zero, surface3d);
        Assert.False(TryGetUnrepresentableSurface3d(Vector3d.Up, out surface3d));
        Assert.Equal(Vector3d.Zero, surface3d);
        Assert.False(TryGetUnrepresentableSurface3d(Vector3d.Forward, out surface3d));
        Assert.Equal(Vector3d.Zero, surface3d);
    }

    [Fact]
    public void CenteredCapsuleClosestFeature_SelectsBothCapsAndThe3dSide()
    {
        Assert.True(FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.One,
            Fixed64.One,
            Vector2d.Left,
            out Vector2d negativeCapSurface));
        Assert.Equal(new Vector2d((Fixed64)(-2), Fixed64.Zero), negativeCapSurface);

        Assert.Equal(Vector3d.Right, FixedSegment.GetDirectionFromCenteredAxis(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One));
        Assert.True(FixedSegment.ContainsPointInCenteredCapsule(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One));
    }

    [Fact]
    public void CenteredCapsuleDistance_ReportsUnrepresentable3dDistance()
    {
        var point = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        var center = new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);

        Assert.False(FixedSegment.TryGetDistanceToCenteredCapsule(
            point,
            center,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance));
        Assert.Equal(Fixed64.MaxValue, distance);
    }

    [Fact]
    public void CenteredCapsule_AxialClipRejectsIntervalsOutsideTheQueryDomain()
    {
        Assert.False(IntersectsCenteredCapsule2d(
            new Vector2d((Fixed64)(-1), (Fixed64)(-2)),
            new Vector2d(Fixed64.One, (Fixed64)(-2)),
            out _));
        Assert.False(IntersectsCenteredCapsule2d(
            new Vector2d((Fixed64)(-1), (Fixed64)2),
            new Vector2d(Fixed64.One, (Fixed64)2),
            out _));
        Assert.False(IntersectsCenteredCapsule2d(
            new Vector2d(Fixed64.Zero, (Fixed64)2),
            new Vector2d(Fixed64.Zero, (Fixed64)3),
            out _));
        Assert.False(IntersectsCenteredCapsule2d(
            new Vector2d(Fixed64.Zero, (Fixed64)(-3)),
            new Vector2d(Fixed64.Zero, (Fixed64)(-2)),
            out _));
        Assert.True(IntersectsCenteredCapsule2d(
            new Vector2d(Fixed64.Zero, (Fixed64)(-2)),
            Vector2d.Zero,
            out Fixed64 entry));
        Assert.Equal(Fixed64.Half, entry);
        Assert.True(IntersectsCenteredCapsule2d(
            Vector2d.Zero,
            new Vector2d(Fixed64.Zero, (Fixed64)2),
            out entry));
        Assert.Equal(Fixed64.Zero, entry);

        var query3d = new FixedSegment(
            new Vector3d((Fixed64)(-1), (Fixed64)2, Fixed64.Zero),
            new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero));
        Assert.False(query3d.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            out _,
            out _));
    }

    private static bool TryGetUnrepresentableSurface3d(Vector3d axis, out Vector3d surface)
    {
        var center = new Vector3d(
            axis.X == Fixed64.One ? Fixed64.MaxValue - Fixed64.One : Fixed64.Zero,
            axis.Y == Fixed64.One ? Fixed64.MaxValue - Fixed64.One : Fixed64.Zero,
            axis.Z == Fixed64.One ? Fixed64.MaxValue - Fixed64.One : Fixed64.Zero);
        Vector3d point = center + axis;

        return FixedSegment.TryGetSurfacePointOnCenteredCapsule(
            point,
            center,
            axis,
            Fixed64.One,
            Fixed64.One,
            axis,
            out surface);
    }

    private static bool IntersectsCenteredCapsule2d(
        Vector2d start,
        Vector2d end,
        out Fixed64 entry)
    {
        var query = new FixedSegment2d(start, end);
        return query.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Zero,
            out entry,
            out _);
    }
}
