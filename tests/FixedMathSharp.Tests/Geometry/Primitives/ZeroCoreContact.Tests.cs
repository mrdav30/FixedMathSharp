using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ZeroCoreContactTests
{
    [Fact]
    public void NonzeroCoreControl_RoundsDepthJustBelowAnIntegerAfterEndpointClamping()
    {
        // Parallel cores end one raw unit apart vertically. Their gap is
        // sqrt((4*Q32)^2 + 1) raw, just above 4 units; depth rounds back to 4.
        Vector2d center = new((Fixed64)4, Fixed64.Two + Fixed64.MinIncrement);
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero, Fixed64.Zero, Vector2d.Forward, Fixed64.Two, (Fixed64)4,
            center, Fixed64.Zero, Vector2d.Forward, Fixed64.Two, (Fixed64)4,
            Vector2d.Right, out FixedContactAnchors2d planar));
        Assert.Equal((Fixed64)4, planar.Depth);
        Assert.Equal(Vector2d.Right, planar.Normal);
        Assert.False(planar.DepthIsClamped);
        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, (Fixed64)4,
            new Vector3d(center.X, center.Y, Fixed64.Zero), FixedQuaternion.Identity,
            Vector3d.Up, Fixed64.Two, (Fixed64)4, Vector3d.Right, out FixedContactAnchors spatial));
        Assert.Equal(planar.Depth, spatial.Depth);
        Assert.Equal(Vector3d.Right, spatial.Normal);
        Assert.False(spatial.DepthIsClamped);
    }

    [Theory]
    [InlineData(1L, false, 0L)]
    [InlineData(2L, true, 0L)]
    [InlineData(3L, true, 1L)]
    public void SphereContact_IncludesEverySpatialComponent(long secondRadius, bool hit, long depth)
    {
        Vector3d center = new(Fixed64.FromRaw(1L), Fixed64.FromRaw(2L), Fixed64.FromRaw(2L));
        Assert.Equal(hit, FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.FromRaw(1L),
            center, FixedQuaternion.Identity, Vector3d.Forward, Fixed64.Zero, Fixed64.FromRaw(secondRadius),
            Vector3d.Right, out FixedContactAnchors contact));
        Assert.Equal(depth, contact.Depth.m_rawValue);
        Assert.False(contact.DepthIsClamped);
        if (hit)
            Assert.Equal(new Vector3d(Fixed64.FromFraction(1, 3), Fixed64.FromFraction(2, 3),
                Fixed64.FromFraction(2, 3)), contact.Normal);
        else
            Assert.Equal(default, contact);
    }

    [Theory]
    [InlineData(2L, false, false)]
    [InlineData(3L, true, false)]
    [InlineData(4L, true, true)]
    public void SphereOverlap_DistinguishesClosedTouchFromStrictOverlap(
        long secondRadius, bool closed, bool strict)
    {
        Vector3d center = new(Fixed64.FromRaw(3L), Fixed64.FromRaw(4L), Fixed64.Zero);
        Assert.Equal(closed, FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.FromRaw(2L),
            center, FixedQuaternion.Identity, Vector3d.Forward, Fixed64.Zero, Fixed64.FromRaw(secondRadius),
            Vector3d.Right, out _));
        Assert.Equal(strict, FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.FromRaw(2L),
            center, FixedQuaternion.Identity, Vector3d.Forward, Fixed64.Zero, Fixed64.FromRaw(secondRadius)));
    }

    // All values are raw Q32.32 units. Expectations come from rA + rB -
    // sqrt(dx^2 + dy^2); classify and clamp before rounding that expression.
    [Theory]
    [InlineData(0L, 0L, 0L, 0L, 0L, 0L, true, 0L, false)]
    [InlineData(0L, 0L, 1L, 0L, 0L, 0L, false, 0L, false)]
    [InlineData(0L, 0L, 3L, 4L, 2L, 2L, false, 0L, false)]
    [InlineData(0L, 0L, 3L, 4L, 2L, 3L, true, 0L, false)]
    [InlineData(0L, 0L, 3L, 4L, 2L, 4L, true, 1L, false)]
    [InlineData(0L, 0L, 1L, 1L, 1L, 1L, true, 1L, false)]
    [InlineData(0L, 0L, 2L, 2L, 1L, 1L, false, 0L, false)]
    [InlineData(0L, 0L, 2L, 2L, 2L, 2L, true, 1L, false)]
    [InlineData(0L, 0L, 1L, 1L, 2L, 2L, true, 3L, false)]
    [InlineData(0L, 0L, 1L, 1L, long.MaxValue, 1L, true, long.MaxValue, false)]
    [InlineData(0L, 0L, 2L, 2L, long.MaxValue, 3L, true, long.MaxValue, true)]
    [InlineData(0L, 0L, 3L, 1L, long.MaxValue, 3L, true, long.MaxValue, false)]
    [InlineData(0L, 0L, 3L, 0L, long.MaxValue, 3L, true, long.MaxValue, false)]
    [InlineData(0L, 0L, 1L, 0L, long.MaxValue, 1L, true, long.MaxValue, false)]
    [InlineData(0L, 0L, 0L, 0L, long.MaxValue, 1L, true, long.MaxValue, true)]
    [InlineData(long.MinValue, 0L, long.MaxValue, 0L, long.MaxValue, long.MaxValue, false, 0L, false)]
    [InlineData(long.MinValue, 0L, long.MaxValue - 1, 0L, long.MaxValue, long.MaxValue, true, 0L, false)]
    [InlineData(long.MinValue, 0L, long.MaxValue - 2, 0L, long.MaxValue, long.MaxValue, true, 1L, false)]
    public void Contact_PreservesFullDomainClassificationDepthAndClamping(
        long ax, long ay, long bx, long by, long ra, long rb,
        bool expectedHit, long expectedDepth, bool expectedClamp)
    {
        Vector2d a = new(Fixed64.FromRaw(ax), Fixed64.FromRaw(ay));
        Vector2d b = new(Fixed64.FromRaw(bx), Fixed64.FromRaw(by));
        Fixed64 firstRadius = Fixed64.FromRaw(ra);
        Fixed64 secondRadius = Fixed64.FromRaw(rb);
        Assert.Equal(expectedHit, FixedSegment2d.TryGetCenteredCapsulesContact(
            a, Fixed64.HalfPi, Vector2d.Right, Fixed64.Zero, firstRadius,
            b, -Fixed64.HalfPi, Vector2d.Forward, Fixed64.Zero, secondRadius,
            Vector2d.Right, out FixedContactAnchors2d planar));
        Assert.Equal(expectedDepth, planar.Depth.m_rawValue);
        Assert.Equal(expectedClamp, planar.DepthIsClamped);

        FixedQuaternion turn = new(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero);
        Assert.Equal(expectedHit, FixedSegment.TryGetCenteredCapsulesContact(
            new Vector3d(a.X, a.Y, Fixed64.Zero), turn, Vector3d.Up, Fixed64.Zero, firstRadius,
            new Vector3d(b.X, b.Y, Fixed64.Zero), turn, Vector3d.Forward, Fixed64.Zero, secondRadius,
            Vector3d.Right, out FixedContactAnchors spatial));
        Assert.Equal(expectedDepth, spatial.Depth.m_rawValue);
        Assert.Equal(expectedClamp, spatial.DepthIsClamped);
        if (!expectedHit)
        {
            Assert.Equal(default, planar);
            Assert.Equal(default, spatial);
            return;
        }

        Assert.Equal(Vector2d.Zero, planar.FirstAnchor.LocalPoint);
        Assert.Equal(Vector2d.Zero, planar.SecondAnchor.LocalPoint);
        Assert.Equal(Vector3d.Zero, spatial.FirstAnchor.LocalPoint);
        Assert.Equal(Vector3d.Zero, spatial.SecondAnchor.LocalPoint);
        Assert.Equal(Fixed64.HalfPi, planar.FirstAnchor.Rotation);
        Assert.Equal(-Fixed64.HalfPi, planar.SecondAnchor.Rotation);
        Assert.Equal(turn, spatial.FirstAnchor.Rotation);
        Assert.Equal(turn, spatial.SecondAnchor.Rotation);
    }

    [Fact]
    public void Contact_RetainsSubRawRadialWitnessesInBothDimensions()
    {
        Fixed64 unit = Fixed64.FromRaw(1L);
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero, Fixed64.Zero, Vector2d.Forward, Fixed64.Zero, unit,
            new Vector2d(unit, unit), Fixed64.Zero, Vector2d.Right, Fixed64.Zero, unit,
            Vector2d.Right, out FixedContactAnchors2d planar));
        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, unit,
            new Vector3d(unit, unit, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.Forward,
            Fixed64.Zero, unit, Vector3d.Right, out FixedContactAnchors spatial));

        Assert.True(planar.FirstAnchor.TryGetPoint(out Vector2d first));
        Assert.True(planar.SecondAnchor.TryGetPoint(out Vector2d second));
        Assert.Equal(new Vector2d(unit, unit), first);
        Assert.Equal(Vector2d.Zero, second);
        Assert.True(planar.FirstAnchor.TryGetOffsetFrom(planar.SecondAnchor, out Vector2d planarOffset));
        Assert.Equal(Vector2d.Zero, planarOffset);
        Assert.True(spatial.FirstAnchor.TryGetPoint(out Vector3d first3d));
        Assert.True(spatial.SecondAnchor.TryGetPoint(out Vector3d second3d));
        Assert.Equal(new Vector3d(unit, unit, Fixed64.Zero), first3d);
        Assert.Equal(Vector3d.Zero, second3d);
        Assert.True(spatial.FirstAnchor.TryGetOffsetFrom(spatial.SecondAnchor, out Vector3d spatialOffset));
        Assert.Equal(Vector3d.Zero, spatialOffset);
    }
}
