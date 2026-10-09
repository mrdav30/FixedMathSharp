//=======================================================================
// FixedSegment.FiniteCone.SurfaceFamilyDomains.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedSegmentFiniteConeSurfaceFamilyDomainsTests
{
    [Fact]
    public void SideCircle_IrrationalSingletonSurvivesExactOpposingHalfspaces()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle);
        // n=(-4 cos(theta),-2,-4 sin(theta)). The first two inequalities
        // require cos(theta)=1/2; the last selects sin(theta)=sqrt(3)/2.
        // No rational Vector2d direction can represent this singleton.
        WideAxis3[] fan = { Covector(1, -1, 0), Covector(-1, 1, 0), Covector(0, 0, 1) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Start);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out Vector3d first, out Vector3d second));
            Assert.Equal(Vector3d.Zero, first);
            Assert.Equal(new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(2, 5),
                Fixed64.FromRaw(2975640629L)), second); // z=2*sqrt(3)/5, rounded once.
            Assert.Equal(Fixed64.FromRaw(3841535534L), item.GetContact(candidate, fan).Depth);
        }
    }

    [Fact]
    public void SideCircle_ArcWithoutCanonicalAxesIsAdmitted()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle);
        // 1/2 <= cos(theta) <= 3/4, sin(theta) >= 0: neither an axis
        // sample nor the opposite root of either boundary is admissible.
        WideAxis3[] fan = { Covector(1, -1, 0), Covector(-2, 3, 0), Covector(0, 0, 1) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Start);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out _, out Vector3d point));
            Assert.Contains(point, new[]
            {
                new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(2, 5), Fixed64.FromRaw(2975640629L)),
                new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(2, 5), Fixed64.FromRaw(2272683071L))
            });
        }
    }

    [Fact]
    public void SideCircle_IndividuallyFeasibleHalfspacesCanHaveEmptyIntersection()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle);
        // cos(theta)>=3/4 and sin(theta)>=3/4 cannot both hold on the circle.
        WideAxis3[] fan = { Covector(2, -3, 0), Covector(0, -3, 2) };
        Assert.NotEmpty(Clip(candidate, fan.AsSpan(0, 1), ConeSurfacePointLocation.Start));
        Assert.NotEmpty(Clip(candidate, fan.AsSpan(1, 1), ConeSurfacePointLocation.Start));
        Assert.Empty(Clip(candidate, fan, ConeSurfacePointLocation.Start));
    }

    [Fact]
    public void BaseInterval_ClippedInteriorEndpointsRemainInteriorToTheAuthoredSegment()
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(new Vector3d(-2, 0, 0), new Vector3d(2, 0, 0)),
            ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval);
        WideAxis3[] fan = { Covector(0, -1, 0) };
        Assert.Empty(Clip(candidate, fan, ConeSurfacePointLocation.Start));
        Assert.Empty(Clip(candidate, fan, ConeSurfacePointLocation.End));
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Interior);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Interior);
        Assert.True(ContainsFirst(candidate, fan, events, -Vector3d.Right));
        Assert.True(ContainsFirst(candidate, fan, events, Vector3d.Zero));
        Assert.True(ContainsFirst(candidate, fan, events, Vector3d.Right));
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(1, 0)]
    [InlineData(2, 1)]
    public void BaseInterval_EndpointAndOpenInteriorFansDoNotLeakAcrossStrata(int stratum, int expectedX)
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(-Vector3d.Right, Vector3d.Right),
            ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval);
        ConeSurfacePointLocation location = stratum == 0 ? ConeSurfacePointLocation.Start
            : stratum == 1 ? ConeSurfacePointLocation.Interior : ConeSurfacePointLocation.End;
        WideAxis3[] allowed = { Covector(0, -1, 0) };
        WideAxis3[] rejected = { Covector(0, 1, 0) };
        Assert.Empty(Clip(candidate, rejected, location));
        ConeSurfaceFamilyEvent[] events = Clip(candidate, allowed, location);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, allowed, events, location);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, allowed, out Vector3d first, out _));
            Assert.Equal(new Vector3d(expectedX, 0, 0), first);
        }
    }

    [Fact]
    public void ApexNormalDisk_PolygonStrictlyInsideCircleRequiresLinePairEvents()
    {
        var apex = new Vector3d(0, 2, 0);
        SegmentConeSurfaceCandidate candidate = PointFamily(apex, ConeSurfaceFeature.Apex, ConeSurfaceFamily.NormalCone);
        // Normalize ny=-2. The cone permits x^2+z^2<=16; this fan cuts
        // 1<=x<=2, 1<=z<=2 entirely inside that disk. The disk center,
        // cardinal directions, and all line/circle intersections fail.
        WideAxis3[] fan = { Covector(-2, -1, 0), Covector(1, 1, 0), Covector(0, -1, -2), Covector(0, 1, 1) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Start);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            FixedContactAnchors contact = item.GetContact(candidate, fan);
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.Contains(contact.Normal, new[]
            {
                RawVector(1753413056L, -3506826112L, 1753413056L),
                RawVector(1431655765L, -2863311531L, 2863311531L),
                RawVector(2863311531L, -2863311531L, 1431655765L),
                RawVector(2479700525L, -2479700525L, 2479700525L)
            });
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out Vector3d first, out Vector3d second));
            Assert.Equal(apex, first);
            Assert.Equal(apex, second);
        }
    }

    [Fact]
    public void ApexNormalDisk_ExactTangentSurvivesButRawOutsideHalfspaceIsEmpty()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(new Vector3d(0, 2, 0), ConeSurfaceFeature.Apex, ConeSurfaceFamily.NormalCone);
        WideAxis3[] tangent = { Covector(-1, -2, 0) }; // x>=4 when ny=-2.
        WideAxis3[] outside = { Covector(-4294967296L, -8589934593L, 0) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, tangent, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, tangent, events, ConeSurfacePointLocation.Start);
        foreach (ConeSurfaceFamilyEvent item in events)
            Assert.Equal(RawVector(3841535534L, -1920767767L, 0), item.GetContact(candidate, tangent).Normal);
        Assert.Empty(Clip(candidate, outside, ConeSurfacePointLocation.Start));
    }

    [Fact]
    public void RimNormalInterval_InteriorMixtureSurvivesWhenBothGeneratorsFail()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(new Vector3d(2, -2, 0), ConeSurfaceFeature.Rim, ConeSurfaceFamily.NormalCone);
        // The side (-2,-1,0) and cap (0,1,0) generators both fail nx+ny=0.
        // Their positive combination (-1,1,0) is the surviving normal ray.
        WideAxis3[] fan = { Covector(1, 1, 0), Covector(-1, -1, 0) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Start);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.Equal(RawVector(-3037000500L, 3037000500L, 0), item.GetContact(candidate, fan).Normal);
        }
    }

    [Theory]
    [InlineData(2, -3)]
    [InlineData(2, -1)]
    public void RimNormalInterval_OutsideBoundaryDoesNotExtrapolatePastEitherGenerator(int x, int y)
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(new Vector3d(2, -2, 0), ConeSurfaceFeature.Rim, ConeSurfaceFamily.NormalCone);
        // For side S=(-2,-1,0), cap C=(0,1,0), the halfspace boundary
        // meets (1-t)S+tC at t=-1/2 or t=3/2. Both generators are strictly
        // admitted, but extrapolating to that root would leave the normal cone.
        WideAxis3[] fan = { Covector(x, y, 0) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.Equal(2, events.Length);
        Vector3d side = RawVector(-3841535534L, -1920767767L, 0);
        Assert.Contains(events, item => item.GetContact(candidate, fan).Normal == side);
        Assert.Contains(events, item => item.GetContact(candidate, fan).Normal == Vector3d.Up);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out Vector3d first, out Vector3d second));
            Assert.Equal(new Vector3d(2, -2, 0), first);
            Assert.Equal(first, second);
            Assert.Equal(Fixed64.Zero, item.GetContact(candidate, fan).Depth);
        }
    }

    [Fact]
    public void FamilyEvent_RejectsReconstructionAfterItsBoundaryHalfspaceChanges()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle);
        WideAxis3[] original = { Covector(1, -1, 0) }; // cos(theta)>=1/2.
        WideAxis3[] changed = { Covector(1, -3, 0) }; // cos(theta)>=3/2 has no circle intersection.
        ConeSurfaceFamilyEvent[] events = Clip(candidate, original, ConeSurfacePointLocation.Start);
        bool foundBoundary = false;
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            if (item.Kind != ConeSurfaceFamilyEventKind.CircleBoundary) continue;
            foundBoundary = true;
            Assert.True(item.TryGetWorldAnchors(candidate, original, out _, out _));
            // Events borrow their original ordered halfspaces. A stale event
            // cannot reconstruct this root after that resource is replaced.
            Assert.False(item.TryGetWorldAnchors(candidate, changed, out Vector3d first, out Vector3d second));
            Assert.Equal(Vector3d.Zero, first);
            Assert.Equal(Vector3d.Zero, second);
            Assert.Throws<InvalidOperationException>(() => item.GetContact(candidate, changed));
        }
        Assert.True(foundBoundary);
    }

    [Fact]
    public void ZeroRadiusInterval_RequiresAnAdmittedEventBeforeChoosingANormal()
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(Vector3d.Down, Vector3d.Up), ConeSurfaceFeature.Side,
            ConeSurfaceFamily.SegmentInterval, Fixed64.Zero);
        Assert.False(candidate.TryGetNormalDotSign(Covector(1, 0, 0), out int sign));
        Assert.Equal(0, sign);
        WideAxis3[] fan = { Covector(-1, 0, 0), Covector(0, 0, 1), Covector(0, 0, -1) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Interior);
        Assert.NotEmpty(events);
        foreach (ConeSurfaceFamilyEvent item in events)
            Assert.Equal(Vector3d.Right, item.GetContact(candidate, fan).Normal);
    }

    [Fact]
    public void ZeroRadiusNormalCone_ObliqueRayRequiresConstraintIntersection()
    {
        var edge = new Vector3d(1, 0, -2);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(-edge, edge), ConeSurfaceFeature.Side,
            ConeSurfaceFamily.NormalCone, Fixed64.Zero);
        // Interior segment tangency requires nx=2*nz; finite-axis tangency
        // requires ny=0. nz>=0 selects (2,0,1), excluding every axis sample.
        WideAxis3[] fan = { Covector(0, 0, -1) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Interior);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Interior);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            FixedContactAnchors contact = item.GetContact(candidate, fan);
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.Equal(new Vector3d(Fixed64.FromRaw(3841535534L), Fixed64.Zero, Fixed64.FromRaw(1920767767L)), contact.Normal);
        }
    }

    [Fact]
    public void BaseInterval_WorldAnchorsRoundTheExactIrrationalPointOnlyOnce()
    {
        // Normalized Q32 components with an exact 1:2 ratio retain cos=3/5,
        // sin=4/5 in the rational quaternion basis.
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(1920767767L), Fixed64.FromRaw(3841535534L));
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(new Vector3d(-Fixed64.Two, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Half)), ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval,
            rotation: rotation);
        WideAxis3[] fan = { Covector(0, -1, 0) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Interior);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Interior);
        bool foundPositiveEndpoint = false;
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out Vector3d first, out Vector3d second));
            if (first.X <= Fixed64.Zero) continue;
            foundPositiveEndpoint = true;
            // p=(sqrt(3)/2,0,1/2), q=(sqrt(3)/2,-2,1/2), before rotation.
            // Rounding p.x first yields world p.y=2975640630, one raw too high.
            Assert.Equal(new Vector3d(Fixed64.FromRaw(2231730472L), Fixed64.FromRaw(2975640629L), Fixed64.Half), first);
            Assert.Equal(new Vector3d(Fixed64.FromRaw(9103678146L), Fixed64.FromRaw(-2178320126L), Fixed64.Half), second);
        }
        Assert.True(foundPositiveEndpoint);
    }

    [Fact]
    public void RimCircle_IrrationalSingletonRetainsItsDistinctRadiusAndDepth()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Rim, ConeSurfaceFamily.RotationCircle);
        WideAxis3[] fan = { Covector(2, 1, 0), Covector(-2, -1, 0), Covector(0, 0, 1) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Start);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out Vector3d first, out Vector3d second));
            Assert.Equal(Vector3d.Zero, first);
            Assert.Equal(new Vector3d(Fixed64.One, -(Fixed64)2, Fixed64.FromRaw(7439101574L)), second);
            Assert.Equal(Fixed64.FromRaw(12148002000L), item.GetContact(candidate, fan).Depth);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SideInterval_ReversedSegmentRetainsTheSameFiniteSlice(bool reverse)
    {
        var a = new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero);
        var b = new Vector3d(Fixed64.Half, -Fixed64.Quarter, Fixed64.Zero);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(reverse ? b : a, reverse ? a : b),
            ConeSurfaceFeature.Side, ConeSurfaceFamily.SegmentInterval);
        WideAxis3[] fan = { Covector(1, 0, 0) };
        ConeSurfaceFamilyEvent[] middle = Clip(candidate, fan, ConeSurfacePointLocation.Interior);
        Assert.NotEmpty(middle);
        AssertAdmitted(candidate, fan, middle, ConeSurfacePointLocation.Interior);
        foreach (ConeSurfaceFamilyEvent item in middle)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out Vector3d first, out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), first);
            Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Half, Fixed64.Zero), second);
            Assert.Equal(Fixed64.FromRaw(2400959709L), item.GetContact(candidate, fan).Depth);
        }
        foreach (ConeSurfacePointLocation location in new[] { ConeSurfacePointLocation.Start, ConeSurfacePointLocation.End })
        {
            ConeSurfaceFamilyEvent[] endpoints = Clip(candidate, fan, location);
            Assert.NotEmpty(endpoints);
            AssertAdmitted(candidate, fan, endpoints, location);
        }
        Assert.Empty(Clip(candidate, new[] { Covector(-1, 0, 0) }, ConeSurfacePointLocation.Interior));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ZeroRadiusInterval_ClippedCapsAndInteriorRetainTheSameRadialRay(bool reverse)
    {
        var a = new Vector3d(0, -4, 0); var b = new Vector3d(0, 4, 0);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(reverse ? b : a, reverse ? a : b),
            ConeSurfaceFeature.Side, ConeSurfaceFamily.SegmentInterval, Fixed64.Zero);
        WideAxis3[] fan = { Covector(-1, 0, 0), Covector(0, 0, 1), Covector(0, 0, -1) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Interior);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Interior);
        Assert.True(ContainsFirst(candidate, fan, events, new Vector3d(0, -2, 0)));
        Assert.True(ContainsFirst(candidate, fan, events, Vector3d.Zero));
        Assert.True(ContainsFirst(candidate, fan, events, new Vector3d(0, 2, 0)));
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            FixedContactAnchors contact = item.GetContact(candidate, fan);
            Assert.Equal(Vector3d.Right, contact.Normal);
            Assert.Equal(Fixed64.Zero, contact.Depth);
        }
        Assert.Empty(Clip(candidate, fan, ConeSurfacePointLocation.Start));
        Assert.Empty(Clip(candidate, fan, ConeSurfacePointLocation.End));
    }

    [Theory]
    [InlineData(-2, 1)]
    [InlineData(0, 0)]
    [InlineData(2, -1)]
    public void ZeroRadiusPoint_CapHalfspaceControlsAxialNormals(int y, int axialDirection)
    {
        var point = new Vector3d(0, y, 0);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(point, point), ConeSurfaceFeature.Side,
            ConeSurfaceFamily.SegmentInterval, Fixed64.Zero);
        WideAxis3[] axisOnly = { Covector(1, 0, 0), Covector(-1, 0, 0), Covector(0, 0, 1), Covector(0, 0, -1) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, axisOnly, ConeSurfacePointLocation.Start);
        if (axialDirection == 0) Assert.Empty(events);
        else
        {
            Assert.NotEmpty(events);
            AssertAdmitted(candidate, axisOnly, events, ConeSurfacePointLocation.Start);
            foreach (ConeSurfaceFamilyEvent item in events)
                Assert.Equal(new Vector3d(0, axialDirection, 0), item.GetContact(candidate, axisOnly).Normal);
        }
        Assert.Empty(Clip(candidate, axisOnly, ConeSurfacePointLocation.End));
    }

    [Fact]
    public void MeridionalRimTouch_UsesTheActualInteriorSegmentTangent()
    {
        var rim = new Vector3d(2, -2, 0); var offset = new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(rim - offset, rim + offset),
            ConeSurfaceFeature.Rim, ConeSurfaceFamily.NormalCone);
        ConeSurfaceFamilyEvent[] events = Clip(candidate, ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.Interior);
        Assert.NotEmpty(events);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, ReadOnlySpan<WideAxis3>.Empty, out Vector3d first, out Vector3d second));
            Assert.Equal(rim, first); Assert.Equal(rim, second);
            Assert.Equal(RawVector(-3037000500L, 3037000500L, 0), item.GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty).Normal);
        }
    }

    [Fact]
    public void ApexNormalDisk_OutsideLinePairDoesNotCreateAFalseFeasibleVertex()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(new Vector3d(0, 2, 0), ConeSurfaceFeature.Apex, ConeSurfaceFamily.NormalCone);
        WideAxis3[] fan = { Covector(-2, -3, 0), Covector(0, -3, -2) }; // x,z>=3 in the radius-4 disk.
        Assert.NotEmpty(Clip(candidate, fan.AsSpan(0, 1), ConeSurfacePointLocation.Start));
        Assert.NotEmpty(Clip(candidate, fan.AsSpan(1, 1), ConeSurfacePointLocation.Start));
        Assert.Empty(Clip(candidate, fan, ConeSurfacePointLocation.Start));
    }

    [Fact]
    public void ZeroRadiusBaseInterval_PreservesTheDistinctAxialDepthFamily()
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(Vector3d.Zero, Vector3d.Zero), ConeSurfaceFeature.Base,
            ConeSurfaceFamily.SegmentInterval, Fixed64.Zero);
        ConeSurfaceFamilyEvent[] events = Clip(candidate, ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            FixedContactAnchors contact = item.GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty);
            Assert.Equal(Vector3d.Up, contact.Normal);
            Assert.Equal(Fixed64.Two, contact.Depth);
            Assert.True(item.TryGetWorldAnchors(candidate, ReadOnlySpan<WideAxis3>.Empty, out Vector3d first, out Vector3d second));
            Assert.Equal(Vector3d.Zero, first); Assert.Equal(new Vector3d(0, -2, 0), second);
        }
    }

    [Fact]
    public void LargeRimCircle_ClampsDepthWithoutClampingTheExactAnchors()
    {
        Fixed64 dimension = Fixed64.MaxValue - Fixed64.MinIncrement;
        Fixed64 half = Fixed64.FromRaw(dimension.m_rawValue / 2);
        var apex = new Vector3d(Fixed64.Zero, half, Fixed64.Zero);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(apex, apex), ConeSurfaceFeature.Rim,
            ConeSurfaceFamily.RotationCircle, dimension, height: dimension);
        ConeSurfaceFamilyEvent[] events = Clip(candidate, ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            FixedContactAnchors contact = item.GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty);
            Assert.True(contact.DepthIsClamped); Assert.Equal(Fixed64.MaxValue, contact.Depth);
            Assert.True(item.TryGetWorldAnchors(candidate, ReadOnlySpan<WideAxis3>.Empty, out Vector3d first, out Vector3d second));
            Assert.Equal(apex, first); Assert.Equal(-half, second.Y);
            Assert.Equal(dimension, second.X == Fixed64.Zero ? FixedMath.Abs(second.Z) : FixedMath.Abs(second.X));
        }
    }

    [Fact]
    public void FamilyWorldAnchors_RejectFirstAnchorOverflow()
    {
        var point = new Vector3d(0, 1, 0);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(point, point), ConeSurfaceFeature.Side,
            ConeSurfaceFamily.RotationCircle, center: new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero));
        ConeSurfaceFamilyEvent[] events = Clip(candidate, ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.False(item.TryGetWorldAnchors(candidate, ReadOnlySpan<WideAxis3>.Empty, out _, out _));
            Assert.Equal(Fixed64.FromRaw(1920767767L), item.GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty).Depth);
        }
    }

    [Fact]
    public void CircleHalfspaces_FullWidthPositiveRescalingPreservesTheExactSingleton()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle);
        Signed320 large = new(1UL << 54, 0, 0, 0, 0); // 2^310, with no narrowing to Q32.
        Signed320 negative = WideArithmetic.Negate(large);
        WideAxis3[] fan = { new(large, negative, default), new(negative, large, default), new(default, default, large) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Start);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out _, out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(2, 5), Fixed64.FromRaw(2975640629L)), second);
        }
    }

    [Fact]
    public void RelativeRotation_ClipsAuthoredHalfspacesBeforeMaterializingWorldAnchors()
    {
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.FromRaw(1920767767L), Fixed64.Zero, Fixed64.FromRaw(3841535534L));
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(Vector3d.Zero, Vector3d.Zero), ConeSurfaceFeature.Side,
            ConeSurfaceFamily.RotationCircle, coneRotation: rotation);
        // Exact authored images of 5*(1,-1,0), 5*(-1,1,0), 5*(0,0,1).
        WideAxis3[] fan = { Covector(3, -5, -4), Covector(-3, 5, 4), Covector(4, 0, 3) };
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        AssertAdmitted(candidate, fan, events, ConeSurfacePointLocation.Start);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.True(item.TryGetWorldAnchors(candidate, fan, out Vector3d first, out Vector3d second));
            Assert.Equal(Vector3d.Zero, first);
            Assert.Equal(new Vector3d(Fixed64.FromRaw(3411304655L), Fixed64.FromRaw(1717986918L), Fixed64.FromRaw(410994843L)), second);
        }
    }

    [Fact]
    public void FamilyDomain_RejectsUnspecifiedStrataAndExhaustedStorageExplicitly()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle);
        Assert.Throws<ArgumentException>(() => Clip(candidate, ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.SegmentIntervalUnspecified));
        Assert.Throws<ArgumentOutOfRangeException>(() => candidate.GetMaximumFamilyEventCount(-1));
        Assert.Throws<InvalidOperationException>(() => AccumulateEmpty(candidate));
        SegmentConeSurfaceCandidate isolated = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Apex, ConeSurfaceFamily.None);
        Assert.Empty(Clip(isolated, ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.Start));
        Assert.Empty(Clip(candidate, ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.End));
    }

    [Fact]
    public void FamilyWorldAnchors_RejectRoundedOverflowWithoutLosingSplitAnchors()
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(Vector3d.Zero, Vector3d.Zero), ConeSurfaceFeature.Side,
            ConeSurfaceFamily.RotationCircle, center: new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        WideAxis3[] fan = { Covector(1, -2, 0) }; // cos(theta)=1.
        ConeSurfaceFamilyEvent[] events = Clip(candidate, fan, ConeSurfacePointLocation.Start);
        Assert.NotEmpty(events);
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.False(item.TryGetWorldAnchors(candidate, fan, out _, out _));
            Assert.Equal(Fixed64.FromRaw(3841535534L), item.GetContact(candidate, fan).Depth);
        }
    }

    [Fact]
    public void FamilyEvents_WarmedCallerOwnedStorageAndMaterializationAllocateNothing()
    {
        SegmentConeSurfaceCandidate candidate = PointFamily(Vector3d.Zero, ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle);
        WideAxis3[] fan = { Covector(1, -1, 0), Covector(-1, 1, 0), Covector(0, 0, 1) };
        var storage = new ConeSurfaceFamilyEvent[candidate.GetMaximumFamilyEventCount(fan.Length)];
        int count = ExerciseFamily(candidate, fan, storage);
        ExerciseFamily(candidate, fan, storage);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 8; iteration++) ExerciseFamily(candidate, fan, storage);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.True(count > 0);
        Assert.Equal(0, allocated);
    }

    private static int ExerciseFamily(SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> fan, Span<ConeSurfaceFamilyEvent> storage)
    {
        var selection = new ConeSurfaceFamilySelection(storage);
        candidate.AccumulateFamilyEvents(fan, ConeSurfacePointLocation.Start, ref selection);
        for (int index = 0; index < selection.Count; index++)
        {
            selection[index].GetContact(candidate, fan);
            selection[index].TryGetWorldAnchors(candidate, fan, out _, out _);
        }
        return selection.Count;
    }

    private static void AccumulateEmpty(SegmentConeSurfaceCandidate candidate)
    {
        var selection = new ConeSurfaceFamilySelection(Span<ConeSurfaceFamilyEvent>.Empty);
        candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.Start, ref selection);
    }

    private static ConeSurfaceFamilyEvent[] Clip(SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> fan,
        ConeSurfacePointLocation location)
    {
        var storage = new ConeSurfaceFamilyEvent[candidate.GetMaximumFamilyEventCount(fan.Length)];
        var selection = new ConeSurfaceFamilySelection(storage);
        bool admitted = candidate.AccumulateFamilyEvents(fan, location, ref selection);
        Assert.Equal(selection.Count != 0, admitted);
        return storage[..selection.Count];
    }

    private static void AssertAdmitted(SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> fan,
        ReadOnlySpan<ConeSurfaceFamilyEvent> events, ConeSurfacePointLocation location)
    {
        foreach (ConeSurfaceFamilyEvent item in events)
        {
            Assert.Equal(location, item.PointLocation);
            Assert.NotEqual(Vector3d.Zero, item.GetContact(candidate, fan).Normal);
        }
    }

    private static bool ContainsFirst(SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> fan,
        ReadOnlySpan<ConeSurfaceFamilyEvent> events, Vector3d expected)
    {
        foreach (ConeSurfaceFamilyEvent item in events)
            if (item.TryGetWorldAnchors(candidate, fan, out Vector3d first, out _) && first == expected) return true;
        return false;
    }

    private static SegmentConeSurfaceCandidate PointFamily(Vector3d point, ConeSurfaceFeature feature, ConeSurfaceFamily family) =>
        Find(new FixedSegment(point, point), feature, family);

    private static SegmentConeSurfaceCandidate Find(FixedSegment segment, ConeSurfaceFeature feature,
        ConeSurfaceFamily family, Fixed64? radius = null, FixedQuaternion? rotation = null,
        FixedQuaternion? coneRotation = null, Vector3d? center = null, Fixed64? height = null)
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        FixedQuaternion frame = rotation ?? FixedQuaternion.Identity;
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            center ?? Vector3d.Zero, frame, center ?? Vector3d.Zero, coneRotation ?? frame, height ?? (Fixed64)4, radius ?? Fixed64.Two, ref selection));
        for (int index = 0; index < selection.Count; index++)
            if (selection[index].Feature == feature && selection[index].Family == family) return selection[index];
        throw new InvalidOperationException($"Expected admitted {feature}/{family} descriptor.");
    }

    private static WideAxis3 Covector(long x, long y, long z) => new(
        Signed320.ExtendValue(Signed192.Signed(x)), Signed320.ExtendValue(Signed192.Signed(y)), Signed320.ExtendValue(Signed192.Signed(z)));

    private static Vector3d RawVector(long x, long y, long z) => new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
}
