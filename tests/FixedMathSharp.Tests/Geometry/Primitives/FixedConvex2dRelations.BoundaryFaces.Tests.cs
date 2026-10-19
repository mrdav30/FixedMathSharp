using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvex2dBoundaryFaceTests
{
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(true, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    public void RetainedFace_ShouldRegenerateTheCutBeforeFinalMaterialization(bool reversePolygon, bool reverseEdge, bool nearLimit)
    {
        Vector2d origin = nearLimit ? new(Fixed64.MaxValue - (Fixed64)16, Fixed64.MinValue + (Fixed64)16) : Vector2d.Zero;
        Vector2d[] polygon = { new(-Fixed64.Quarter, -Fixed64.Quarter), new((Fixed64)0.75, -Fixed64.Quarter),
            new((Fixed64)0.75, (Fixed64)0.75), new(-Fixed64.Quarter, (Fixed64)0.75) };
        if (reversePolygon) Array.Reverse(polygon);
        var edge = new FixedSegment2d(new(-1, 0), new(1, 0));
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(2)),
            Signed320.One, Signed320.One);
        if (reverseEdge)
        {
            edge = new FixedSegment2d(edge.End, edge.Start);
            clip = new RationalSegmentInterval(default, Signed320.One, Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(2)));
        }
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        int count = WideConvex2dRelations.GetBoundaryFaceContacts(origin, Fixed64.Zero, polygon,
            origin, Fixed64.Zero, edge, new(0, -1), clip, contacts);
        Assert.Equal(2, count);
        Fixed64 minX = Fixed64.MaxValue, maxX = Fixed64.MinValue;
        foreach (var contact in contacts[..count])
        {
            Assert.Equal(-Vector2d.Forward, contact.Normal);
            Assert.Equal(Fixed64.Quarter, contact.Depth);
            Assert.False(contact.DepthIsClamped);
            Assert.Equal(origin, contact.FirstAnchor.Origin);
            Assert.Equal(origin, contact.SecondAnchor.Origin);
            Assert.Equal(-Fixed64.Quarter, contact.FirstAnchor.LocalPoint.Y);
            Assert.Equal(Fixed64.Zero, contact.SecondAnchor.LocalPoint.Y);
            Assert.Equal(contact.FirstAnchor.LocalPoint.X, contact.SecondAnchor.LocalPoint.X);
            minX = FixedMath.Min(minX, contact.SecondAnchor.LocalPoint.X);
            maxX = FixedMath.Max(maxX, contact.SecondAnchor.LocalPoint.X);
        }
        Assert.Equal(Fixed64.Zero, minX);
        Assert.Equal((Fixed64)0.75, maxX);
    }

    [Fact]
    public void SlopedIncidentEdge_ShouldSelectItsExactClippedMaximum()
    {
        Vector2d[] polygon = { new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Quarter), new(Fixed64.Half, Fixed64.Half) };
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(2)), Signed320.One, Signed320.One);
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        Assert.Equal(1, WideConvex2dRelations.GetBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero, polygon,
            Vector2d.Zero, Fixed64.Zero, new(new(-1, 0), new(1, 0)), new(0, -1), clip, contacts));
        Assert.Equal(new Vector2d(Fixed64.Zero, -(Fixed64)0.375), contacts[0].FirstAnchor.LocalPoint);
        Assert.Equal(Vector2d.Zero, contacts[0].SecondAnchor.LocalPoint);
        Assert.Equal((Fixed64)0.375, contacts[0].Depth);
    }

    [Theory]
    [InlineData(-3, 0)]
    [InlineData(0, -3)]
    [InlineData(0, 3)]
    public void RemoteFiniteFaceOrSupportingPlane_ShouldNotBecomeAContact(int x, int y)
    {
        Vector2d[] polygon = { new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half) };
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        Assert.Equal(0, WideConvex2dRelations.GetBoundaryFaceContacts(new(x, y), Fixed64.Zero, polygon,
            Vector2d.Zero, Fixed64.Zero, new(new(-1, 0), new(1, 0)), new(0, -1), default, contacts));
    }

    [Fact]
    public void ClippedAwayIntersection_ShouldRemainSeparated()
    {
        Vector2d[] polygon = { new(-2, -1), new(-1, -1), new(-1, 1), new(-2, 1) };
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(2)), Signed320.One, Signed320.One);
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        Assert.Equal(0, WideConvex2dRelations.GetBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero, polygon,
            Vector2d.Zero, Fixed64.Zero, new(new(-3, 0), new(3, 0)), new(0, -1), clip, contacts));
    }

    [Fact]
    public void MissingOutputCapacity_ShouldFailBeforeWriting()
    {
        Assert.Throws<ArgumentException>(() => WideConvex2dRelations.GetBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero,
            new[] { Vector2d.Zero, Vector2d.Right, Vector2d.Forward }, Vector2d.Zero, Fixed64.Zero,
            new(Vector2d.Zero, Vector2d.Right), Vector2d.Forward, default, Span<FixedContactAnchors2d>.Empty));
    }

    [Fact]
    public void AdmittedConnectedPlane_ShouldRetainAnObliqueSupportOutsideTheInitialFiniteCrossing()
    {
        Vector2d[] polygon = { new(-2, 1), new(-2, -1), new(1, -1) };
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        var supplier = new FixedSegment2d(new(0, 0), new(2, 0));
        Assert.Equal(0, WideConvex2dRelations.GetBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero, polygon,
            Vector2d.Zero, Fixed64.Zero, supplier, new(0, -1), default, contacts));
        Assert.True(WideConvex2dRelations.SameBoundaryPlane(Vector2d.Zero, Fixed64.Zero,
            new(new(-2, 0), new(0, 0)), new(-1, -1), Vector2d.Zero, Fixed64.Zero, supplier, new(1, -1)));
        Assert.Equal(2, WideConvex2dRelations.GetBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero, polygon,
            Vector2d.Zero, Fixed64.Zero, supplier, new(0, -1), default, contacts, planeAdmitted: true));
        Assert.Equal(Fixed64.One, contacts[0].Depth);
        Assert.Equal(new Vector2d(0, -1), contacts[0].FirstAnchor.LocalPoint);
        Assert.Equal(new Vector2d(1, -1), contacts[1].FirstAnchor.LocalPoint);
        Assert.Equal(Vector2d.Zero, contacts[0].SecondAnchor.LocalPoint);
        Assert.Equal(Vector2d.Right, contacts[1].SecondAnchor.LocalPoint);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void AdmittedPlane_ShouldStillRejectMissingFiniteSuppliers(bool collapsed)
    {
        Vector2d[] polygon = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        var edge = collapsed ? new FixedSegment2d(Vector2d.Zero, Vector2d.Zero) : new(new(4, 0), new(6, 0));
        Assert.Equal(0, WideConvex2dRelations.GetBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero, polygon,
            Vector2d.Zero, Fixed64.Zero, edge, -Vector2d.Forward, default, contacts, planeAdmitted: true));
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void FullFiniteEdgeCoverage_ShouldAgreeWithClippedOrUnclippedAdmission(bool clipped)
    {
        Vector2d[] polygon = { new(-2, -1), new(2, -1), new(2, 1), new(-2, 1) };
        RationalSegmentInterval interval = clipped ? new(Signed320.ExtendValue(Signed192.Signed(1)),
            Signed320.ExtendValue(Signed192.Signed(4)), Signed320.ExtendValue(Signed192.Signed(3)),
            Signed320.ExtendValue(Signed192.Signed(4))) : default;
        Span<FixedContactAnchors2d> contacts = stackalloc FixedContactAnchors2d[2];
        Assert.Equal(2, WideConvex2dRelations.GetBoundaryFaceContacts(Vector2d.Zero, Fixed64.Zero, polygon,
            Vector2d.Zero, Fixed64.Zero, new(new(-1, 0), new(1, 0)), -Vector2d.Forward, interval, contacts));
        Assert.Equal(Fixed64.One, contacts[0].Depth);
        Assert.Equal(clipped ? -Fixed64.Half : -Fixed64.One, contacts[0].SecondAnchor.LocalPoint.X);
        Assert.Equal(clipped ? Fixed64.Half : Fixed64.One, contacts[1].SecondAnchor.LocalPoint.X);
    }
}
