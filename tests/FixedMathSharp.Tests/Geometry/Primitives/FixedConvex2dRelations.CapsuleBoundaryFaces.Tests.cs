using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvex2dCapsuleBoundaryFaceTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CircleArcBeyondTheOriginalPole_ShouldSupplyTheFiniteCut(bool reverse, bool nearLimit)
    {
        Vector2d target = nearLimit ? new(Fixed64.MaxValue - (Fixed64)8, Fixed64.MinValue + (Fixed64)8) : Vector2d.Zero;
        Vector2d mover = target + new Vector2d(-Fixed64.Quarter, Fixed64.Quarter);
        var edge = new FixedSegment2d(new(-1, 0), new(1, 0));
        RationalSegmentInterval clip = Half(reverse);
        if (reverse) edge = new(edge.End, edge.Start);
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        int count = WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(mover, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Half, target, Fixed64.Zero, edge, new(0, -1), clip, contacts);
        Assert.Equal(1, count);
        var contact = contacts[0];
        Assert.Equal(-Vector2d.Forward, contact.Normal);
        Assert.Equal(Vector2d.Zero, contact.SecondAnchor.LocalPoint);
        Fixed64 expected = FixedMath.Sqrt((Fixed64)3) / (Fixed64)4 - Fixed64.Quarter;
        Assert.InRange(contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.Quarter, contact.FirstAnchor.LocalPoint.X);
        Vector2d local = contact.FirstAnchor.LocalPoint;
        Fixed64 radiusSquared = local.X * local.X + local.Y * local.Y;
        Assert.InRange(radiusSquared.m_rawValue, Fixed64.Quarter.m_rawValue - 2, Fixed64.Quarter.m_rawValue + 2);
        Assert.True(contact.FirstAnchor.TryGetPoint(out _));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CapsuleSide_ShouldRetainBothEndsOfTheExposedSupport(bool reverse, bool reverseAxis)
    {
        var edge = new FixedSegment2d(new(-2, 0), new(2, 0));
        if (reverse) edge = new(edge.End, edge.Start);
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        int count = WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(new(-Fixed64.Quarter, Fixed64.Quarter),
            reverseAxis ? Fixed64.HalfPi : -Fixed64.HalfPi, Fixed64.One, Fixed64.Half,
            Vector2d.Zero, Fixed64.Zero, edge, new(0, -1), Half(reverse), contacts);
        Assert.Equal(2, count);
        Fixed64 lower = Fixed64.MaxValue, upper = Fixed64.MinValue;
        foreach (var contact in contacts[..count])
        {
            Assert.Equal(Fixed64.Quarter, contact.Depth);
            Assert.Equal(-Vector2d.Forward, contact.Normal);
            Assert.Equal(Fixed64.Zero, contact.SecondAnchor.LocalPoint.Y);
            Assert.Equal(-Fixed64.Half, contact.FirstAnchor.LocalPoint.Y);
            lower = FixedMath.Min(lower, contact.SecondAnchor.LocalPoint.X);
            upper = FixedMath.Max(upper, contact.SecondAnchor.LocalPoint.X);
        }
        Assert.Equal(Fixed64.Zero, lower);
        Assert.Equal(Fixed64.Quarter, upper);
    }

    [Theory]
    [InlineData(0, -3, 0)]
    [InlineData(0, 3, 0)]
    [InlineData(3, 0, 0)]
    [InlineData(0, -3, 1)]
    [InlineData(0, 3, 1)]
    [InlineData(3, 0, 1)]
    public void RemoteOrWhollyBuriedBody_ShouldNotTurnItsSupportingPlaneIntoAContact(int x, int y, int length)
    {
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        Assert.Equal(0, WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(new(x, y), Fixed64.Zero,
            (Fixed64)length, Fixed64.Half, Vector2d.Zero, Fixed64.Zero,
            new(new(-1, 0), new(1, 0)), new(0, -1), default, contacts));
    }

    [Fact]
    public void OddCoreLength_ShouldNotRoundBeforeSelectingTheCap()
    {
        Fixed64 length = Fixed64.FromRaw(Fixed64.One.m_rawValue + 1);
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        Assert.Equal(1, WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(new(Fixed64.Zero, Fixed64.Quarter), Fixed64.Zero,
            length, Fixed64.Half, Vector2d.Zero, Fixed64.Zero,
            new(new(-2, 0), new(2, 0)), new(0, -1), default, contacts));
        // 1/2 + (1 + one raw)/2 - 1/4 ties to the even raw depth.
        Assert.Equal((Fixed64)0.75, contacts[0].Depth);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AdmittedCoplanarNeighbor_ShouldSupplyItsClippedFiniteSupport(bool planeAdmitted)
    {
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(7)),
            Signed320.ExtendValue(Signed192.Signed(8)), Signed320.One, Signed320.One);
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        int count = WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(new(Fixed64.Quarter, -Fixed64.Quarter),
            Fixed64.Zero, Fixed64.Zero, Fixed64.Half, Vector2d.Zero, Fixed64.Zero,
            new(new(-1, 0), new(1, 0)), new(0, -1), clip, contacts, planeAdmitted);
        Assert.Equal(planeAdmitted ? 1 : 0, count);
        if (!planeAdmitted) return;
        Assert.Equal(new Vector2d((Fixed64)0.75, Fixed64.Zero), contacts[0].SecondAnchor.LocalPoint);
        Assert.Equal(new Vector2d(Fixed64.Half, Fixed64.Zero), contacts[0].FirstAnchor.LocalPoint);
        Assert.Equal(Fixed64.Quarter, contacts[0].Depth);
    }

    [Fact]
    public void ObliqueCapsule_ShouldKeepTheAcceptedScalarAxisMetric()
    {
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        int count = WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(new(Fixed64.Zero, Fixed64.Quarter),
            Fixed64.PiOver4, (Fixed64)128, Fixed64.Half, Vector2d.Zero, Fixed64.Zero,
            new(new(-128, 0), new(128, 0)), new(0, -1), default, contacts);
        Assert.Equal(1, count);
        Fixed64 halfCoreProjection = (Fixed64)64 * FixedMath.Cos(Fixed64.PiOver4);
        Assert.Equal(Fixed64.Quarter + halfCoreProjection, contacts[0].Depth);
        Assert.Equal(-halfCoreProjection - Fixed64.Half, contacts[0].FirstAnchor.LocalPoint.Y);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DegenerateFace_ShouldNotProduceAContact(bool collinearInward)
    {
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        Assert.Equal(0, WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Half, Vector2d.Zero, Fixed64.Zero,
            new(Vector2d.Zero, collinearInward ? Vector2d.Right : Vector2d.Zero), Vector2d.Right, default, contacts));
    }

    [Fact]
    public void MissingCapacity_ShouldFailBeforeWriting() => Assert.Throws<ArgumentException>(() =>
        WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Half, Vector2d.Zero, Fixed64.Zero, new(Vector2d.Zero, Vector2d.Right),
            Vector2d.Forward, default, Span<FixedContactAnchors2d>.Empty));

    [Fact]
    public void WideCoverageBounds_ShouldFailBeforeMaterialization()
    {
        var n = new Signed832(0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
        var d = new Signed832(0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
        var clip = new RationalSegmentInterval(n, d, d, d);
        Assert.True(clip.HasWideBounds);
        Assert.Throws<ArgumentException>(() => WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(Vector2d.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Half, Vector2d.Zero, Fixed64.Zero,
            new(Vector2d.Zero, Vector2d.Right), Vector2d.Forward, clip, new FixedContactAnchors2d[2]));
    }

    [Fact]
    public void RepresentableLocalWitness_WithAnUnrepresentablePenetration_ShouldKeepTheClampFlag()
    {
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        int count = WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(new(Fixed64.Zero, -Fixed64.MaxValue / Fixed64.Two),
            Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue, Vector2d.Zero, Fixed64.Zero,
            new(new(-1, 0), new(1, 0)), new(0, -1), default, contacts);
        Assert.Equal(1, count);
        Assert.True(contacts[0].DepthIsClamped);
        Assert.Equal(Fixed64.MaxValue, contacts[0].Depth);
        Assert.Equal(new Vector2d(Fixed64.Zero, -Fixed64.MaxValue), contacts[0].FirstAnchor.LocalPoint);
        Assert.Equal(Vector2d.Zero, contacts[0].SecondAnchor.LocalPoint);
    }

    private static RationalSegmentInterval Half(bool reverse) => reverse
        ? new(default, Signed320.One, Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(2)))
        : new(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(2)), Signed320.One, Signed320.One);
}
