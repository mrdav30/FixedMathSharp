using System;
using System.Collections.Generic;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class CapsulePrismDomainTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, true)]
    public void SideClippingRetainsTheNativeTangentHeightAndCoreIntersection(bool yawed, bool reversedHorizontalEdge)
    {
        Fixed64 yaw = yawed ? Fixed64.Deg2Rad * (Fixed64)31 : Fixed64.Zero;
        Vector2d start = reversedHorizontalEdge ? new(2, 0) : new(0, -2);
        Vector2d end = reversedHorizontalEdge ? new(-2, 0) : new(0, 2);
        Vector3d first = reversedHorizontalEdge ? new(-4, -2, 0) : new(0, -2, -4);
        Vector3d second = reversedHorizontalEdge ? new(4, 2, 0) : new(0, 2, 4);
        var frame = new NativePrismYawFrame3d(Vector3d.Zero, yaw);
        Signed832[] projected = Projected(frame.GetPoint(first), frame.GetPoint(second), out Signed832 pointD);
        Assert.True(WideConvexPrismRelations.TryClipProjectedPrismFace(projected, pointD, yaw, start, end,
            Interval(1, 4, 1, 2), -Fixed64.Half, Fixed64.Half, Interval(7, 16, 1, 1), out var retained));
        // The source crosses -4..4. The retained edge is [-1,0] or [0,1],
        // intersected with the independently supplied core window [7/16,1].
        if (reversedHorizontalEdge) AssertInterval(retained, 1, 2, 5, 8);
        else AssertInterval(retained, 7, 16, 1, 2);
    }

    [Fact]
    public void CrossingTheSidePlaneRetainsOnlyItsExactIntersection()
    {
        Signed832[] projected = Projected(RationalPoint3d.FromScalarPoint(Vector3d.Left),
            RationalPoint3d.FromScalarPoint(Vector3d.Right), out Signed832 pointD);
        Assert.True(WideConvexPrismRelations.TryClipProjectedPrismFace(projected, pointD, Fixed64.Zero,
            new(0, -2), new(0, 2), default, -Fixed64.One, Fixed64.One, default, out var retained));
        AssertInterval(retained, 1, 2, 1, 2);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void SideClippingRejectsOffPlaneTangentAndHeightMisses(int geometry)
    {
        Vector3d first = new(0, 0, -1), second = new(0, 0, 1);
        if (geometry < 2) first.X = second.X = geometry == 0 ? Fixed64.One : -Fixed64.One;
        else if (geometry < 4)
        {
            first.Z = geometry == 2 ? (Fixed64)3 : (Fixed64)(-4);
            second.Z = geometry == 2 ? (Fixed64)4 : (Fixed64)(-3);
        }
        else first.Y = second.Y = geometry == 4 ? Fixed64.Two : -Fixed64.Two;
        Fixed64 yaw = Fixed64.Deg2Rad * (Fixed64)31;
        var frame = new NativePrismYawFrame3d(Vector3d.Zero, yaw);
        Signed832[] projected = Projected(frame.GetPoint(first), frame.GetPoint(second), out Signed832 pointD);
        Assert.False(WideConvexPrismRelations.TryClipProjectedPrismFace(projected, pointD, yaw,
            new(0, -2), new(0, 2), default, -Fixed64.One, Fixed64.One, default, out _));
    }

    [Fact]
    public void CollapsedSideEdgeCannotSupplyAFaceInterval()
    {
        Signed832[] projected = Projected(RationalPoint3d.FromScalarPoint(Vector3d.Down),
            RationalPoint3d.FromScalarPoint(Vector3d.Up), out Signed832 pointD);
        Assert.False(WideConvexPrismRelations.TryClipProjectedPrismFace(projected, pointD, Fixed64.Zero,
            Vector2d.Zero, Vector2d.Zero, default, -Fixed64.One, Fixed64.One, default, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapClippingPreservesTheAlreadyAdmittedFootprintWindow(bool coreExcludesPlane)
    {
        Fixed64 yaw = Fixed64.Deg2Rad * (Fixed64)31;
        var frame = new NativePrismYawFrame3d(Vector3d.Zero, yaw);
        Signed832[] projected = Projected(frame.GetPoint(new(0, -2, 0)), frame.GetPoint(new(0, 2, 0)), out Signed832 pointD);
        // Footprint admission belongs to the caller. A cap uses only Y, so
        // its edge arguments may be zero without inventing a side rectangle.
        bool found = WideConvexPrismRelations.TryClipProjectedPrismFace(projected, pointD, yaw,
            Vector2d.Zero, Vector2d.Zero, default, Fixed64.Half, Fixed64.Half,
            coreExcludesPlane ? Interval(0, 1, 1, 2) : default, out var retained, cap: true);
        Assert.Equal(!coreExcludesPlane, found);
        if (found) AssertInterval(retained, 5, 8, 5, 8);
    }

    [Theory]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(2, true)]
    [InlineData(3, true)]
    [InlineData(4, false)]
    [InlineData(5, true)]
    [InlineData(6, false)]
    public void CompleteRotatedCapsuleClassifiesBarrelSeamsAndTipsExactly(int feature, bool expected)
    {
        Fixed64 radius = feature >= 5 ? Fixed64.FromFraction(5, 8) : Fixed64.Half;
        Vector3d local = feature switch
        {
            0 => new(Fixed64.Half, Fixed64.One, Fixed64.Zero),
            1 => new(Fixed64.Half + Fixed64.MinIncrement, Fixed64.One, Fixed64.Zero),
            2 => new(Fixed64.Half, Fixed64.Two, Fixed64.Zero),
            3 => new(Fixed64.Zero, 5 * Fixed64.Half, Fixed64.Zero),
            4 => new(Fixed64.Zero, 5 * Fixed64.Half + Fixed64.MinIncrement, Fixed64.Zero),
            5 => new(Fixed64.FromFraction(3, 8), 5 * Fixed64.Half, Fixed64.Zero),
            _ => new(Fixed64.FromFraction(3, 8) + Fixed64.MinIncrement, 5 * Fixed64.Half, Fixed64.Zero)
        };
        Vector3d relativeCenter = new(13, -7, 5);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, Fixed64.Zero, (Fixed64)31);
        if ((feature & 1) != 0) rotation = new(-rotation.X, -rotation.Y, -rotation.Z, -rotation.W);
        // The exact frame generates the world point from a known local point;
        // no rounded quaternion-vector product weakens one-raw-unit admission.
        var frame = ScaledRigidFrame3d.Rigid(relativeCenter, rotation);
        RationalPoint3d point = frame.GetPoint(local);
        Signed832[] projected = Projected(point, point, out Signed832 pointD);
        Assert.Equal(expected, WideConvexPrismRelations.IsProjectedPrismPointWithinCapsule(projected, pointD,
            Integer(1), Integer(3), relativeCenter, rotation, (Fixed64)4, radius));
    }

    [Fact]
    public void InteriorCoreCrossingIsNotClassifiedAgainstOnlyASupportEndpoint()
    {
        Vector3d relativeCenter = new(13, -7, 5);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, Fixed64.Zero, (Fixed64)31);
        var frame = ScaledRigidFrame3d.Rigid(relativeCenter, rotation);
        Signed832[] projected = Projected(frame.GetPoint(new(Fixed64.Quarter, -5 * Fixed64.Half, Fixed64.Zero)),
            frame.GetPoint(new(Fixed64.Quarter, 5 * Fixed64.Half, Fixed64.Zero)), out Signed832 pointD);
        Assert.False(WideConvexPrismRelations.IsProjectedPrismPointWithinCapsule(projected, pointD,
            Integer(0), Integer(1), relativeCenter, rotation, (Fixed64)4, Fixed64.Half));
        Assert.False(WideConvexPrismRelations.IsProjectedPrismPointWithinCapsule(projected, pointD,
            Integer(1), Integer(1), relativeCenter, rotation, (Fixed64)4, Fixed64.Half));
        // Both endpoints are outside their caps. The midpoint is (1/4,0,0)
        // in the capsule chart, inside the barrel and far from either support tip.
        Assert.True(WideConvexPrismRelations.IsProjectedPrismPointWithinCapsule(projected, pointD,
            Integer(1), Integer(2), relativeCenter, rotation, (Fixed64)4, Fixed64.Half));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void BoundaryStrataPreserveExactFiniteAnchorsAfterSharedChartReduction(int stratum)
    {
        bool vertical = stratum == 1;
        Fixed64 radius = vertical ? Fixed64.One : Fixed64.Half;
        Vector3d center = new(-Fixed64.Quarter, Fixed64.Zero, vertical ? Fixed64.Zero : -Fixed64.Quarter);
        FixedQuaternion rotation = FixedQuaternion.Identity;
        if (stratum == 2)
        {
            // Equal normalized Y/W components give an exact quarter turn about
            // the capsule's unchanged axis, with a nontrivial authored denominator.
            Fixed64 halfRoot = Fixed64.FromRaw(3_037_000_500);
            rotation = new(Fixed64.Zero, halfRoot, Fixed64.Zero, halfRoot);
            Assert.True(rotation.IsNormalized());
        }
        RationalSegmentInterval clip = vertical ? Interval(1, 3, 1, 3)
            : stratum == 2 ? Interval(1, 2, 1, 2) : Interval(1, 2, 1, 1);
        var worker = new CapsulePrismBoundaryContacts();
        Assert.True(worker.TryGetContact(center, rotation, Fixed64.Two, radius, Vector3d.Zero, Fixed64.Zero,
            new(0, -2), new(0, 2), new(-2, 0), clip,
            vertical ? -Fixed64.One : Fixed64.Zero, vertical ? Fixed64.One : Fixed64.Zero, 0, out var contact));
        // For the vertical cut, z=-2/3 gives a radial ray height sqrt(5)/3.
        // Otherwise z=0 gives sqrt(3)/4. These integer half-cell checks are
        // independent nearest-even oracles for the unrounded geometric roots.
        long root = vertical ? 3_201_279_612 : 1_859_775_393;
        BigInteger q = Fixed64.One.m_rawValue;
        BigInteger numerator = (vertical ? 5 : 3) * q * q;
        int divisor = vertical ? 9 : 16;
        Assert.True(divisor * BigInteger.Pow(2 * (BigInteger)root - 1, 2) < 4 * numerator);
        Assert.True(4 * numerator < divisor * BigInteger.Pow(2 * (BigInteger)root + 1, 2));
        Assert.Equal(Fixed64.FromRaw(root + Fixed64.Quarter.m_rawValue), contact.Depth);
        Assert.Equal(Vector3d.Left, contact.Normal); Assert.False(contact.DepthIsClamped);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(Fixed64.Zero, target.X);
        Assert.Equal(vertical ? -Fixed64.FromFraction(2, 3) : Fixed64.Zero, target.Z);
        if (vertical) Assert.InRange(target.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
        else Assert.Equal(Fixed64.Zero, target.Y);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d moving));
        Assert.Equal(-contact.Depth, moving.X); Assert.Equal(target.Y, moving.Y); Assert.Equal(target.Z, moving.Z);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RetainedOrdinarySupportKeepsTheFiniteFaceContact(bool includeRecovery)
    {
        var domain = new CaptureDomain(new RationalSegmentInterval[] { default }) { IncludeRecovery = includeRecovery };
        Assert.True(Visit(new(Fixed64.Zero, Fixed64.Zero, 5 * Fixed64.Quarter), FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps);
        Assert.Equal(3, domain.Supports.Count);
        for (int index = 0; index < domain.Supports.Count; index++)
        {
            var support = domain.Supports[index];
            Assert.True(support.Physical);
            Assert.Equal(Fixed64.Quarter, support.Contact.Depth);
            Assert.Equal(Vector3d.Backward, support.Contact.Normal);
            Assert.True(support.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
            Assert.Equal(new Vector3d(Fixed64.Zero, index == 0 ? Fixed64.Zero : index == 1 ? -Fixed64.Half : Fixed64.Half, Fixed64.One), target);
        }
        Assert.Empty(domain.Edges);
    }

    [Fact]
    public void CoveredCoreMidpointKeepsBothDisconnectedSupportIntervals()
    {
        var domain = new CaptureDomain(new[] { Interval(0, 1, 1, 4), Interval(3, 4, 1, 1) });
        Assert.True(Visit(new(Fixed64.Zero, Fixed64.Zero, 5 * Fixed64.Quarter), FixedQuaternion.Identity,
            (Fixed64)4, Fixed64.Half, (Fixed64)3, domain, out bool overlaps));
        Assert.True(overlaps);
        Assert.Equal(4, domain.Supports.Count);
        for (int index = 0; index < domain.Supports.Count; index++)
        {
            var support = domain.Supports[index];
            Assert.True(support.Physical);
            Assert.Equal(index / 2, support.Interval);
            Assert.Equal(Fixed64.Quarter, support.Contact.Depth);
            Assert.True(support.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
            Assert.Equal(new Vector3d(0, index == 0 ? -1 : index == 1 ? -2 : index == 2 ? 1 : 2, 1), target);
        }
        Assert.Empty(domain.Edges);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EmptyFaceDomainRetainsTheActualClosestEdgeOrCanonicalCorner(bool corner)
    {
        var domain = new CaptureDomain(Array.Empty<RationalSegmentInterval>());
        Assert.True(Visit(new(5 * Fixed64.Quarter, Fixed64.Two, corner ? 5 * Fixed64.Quarter : Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.One, Fixed64.One, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps);
        Assert.Empty(domain.Supports);
        var edge = Assert.Single(domain.Edges);
        Assert.Equal(corner ? 2 : 1, edge.Kind);
        Assert.Equal(corner ? 2 : 1, edge.Vertex);
        Assert.True(edge.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(new Vector3d(1, 1, corner ? 1 : 0), target);
        // The source core endpoint is (5/4,3/2,z). Its exact distance
        // to the target is sqrt(5)/4, or sqrt(6)/4 at the cap corner.
        Fixed64 expected = Fixed64.One - FixedMath.Sqrt((Fixed64)(corner ? 6 : 5)) * Fixed64.Quarter;
        Assert.InRange(edge.Contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
        Assert.True(edge.Contact.Normal.X < Fixed64.Zero);
        Assert.True(edge.Contact.Normal.Y < edge.Contact.Normal.X);
        if (corner) Assert.Equal(edge.Contact.Normal.X, edge.Contact.Normal.Z);
        else Assert.Equal(Fixed64.Zero, edge.Contact.Normal.Z);
    }

    [Fact]
    public void ParallelClosestEdgeFamilyRetainsBothOpenClippedIntervals()
    {
        var domain = new CaptureDomain(Array.Empty<RationalSegmentInterval>())
        {
            EdgeVertex = 2, EdgeKind = 2, RejectEdgeEndpoints = true,
            EdgeIntervals = new[] { Interval(0, 1, 1, 4), Interval(3, 4, 1, 1) }
        };
        Assert.True(Visit(new(5 * Fixed64.Quarter, Fixed64.Zero, 5 * Fixed64.Quarter), FixedQuaternion.Identity,
            (Fixed64)4, Fixed64.Half, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps);
        Assert.Equal(2, domain.Edges.Count);
        Fixed64 expected = Fixed64.Half - FixedMath.Sqrt(Fixed64.Two) * Fixed64.Quarter;
        for (int index = 0; index < domain.Edges.Count; index++)
        {
            var edge = domain.Edges[index];
            Assert.Equal(index, edge.Interval);
            Assert.True(edge.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
            Assert.Equal(Fixed64.One, target.X); Assert.Equal(Fixed64.One, target.Z);
            Assert.True(index == 0 ? target.Y < -Fixed64.Half : target.Y > Fixed64.Half);
            Assert.InRange(edge.Contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
            Assert.Equal(Fixed64.Zero, edge.Contact.Normal.Y);
            Assert.Equal(edge.Contact.Normal.X, edge.Contact.Normal.Z);
        }
    }

    [Theory]
    [InlineData(-3)]
    [InlineData(0)]
    [InlineData(3)]
    public void ClippedVerticalEdgeClampsTheFiniteCoreWithoutInventingCutNormals(int centerY)
    {
        var domain = new CaptureDomain(Array.Empty<RationalSegmentInterval>())
        { EdgeVertex = 2, EdgeKind = 2, EdgeIntervals = new[] { Interval(1, 4, 3, 4) } };
        Assert.Equal(centerY == 0, Visit(new(5 * Fixed64.Quarter, (Fixed64)centerY, 5 * Fixed64.Quarter),
            FixedQuaternion.Identity, (Fixed64)4, (Fixed64)4, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps); Assert.Empty(domain.Supports);
        // The retained edge spans Y=[-1/2,1/2]. Only the centered core meets
        // its parallel closest family. Cores above/below require their finite
        // endpoint; a cut-height normal would not support the complete prism.
        if (centerY != 0) { Assert.Empty(domain.Edges); return; }
        var edge = Assert.Single(domain.Edges);
        Assert.True(edge.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(new Vector3d(1, 0, 1), target);
        Fixed64 expected = (Fixed64)4 - FixedMath.Sqrt(Fixed64.Two) * Fixed64.Quarter;
        Assert.InRange(edge.Contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
    }

    [Fact]
    public void AdjacentClosedEdgeBandsShareOneClosestPoint()
    {
        var domain = new CaptureDomain(Array.Empty<RationalSegmentInterval>())
        {
            EdgeVertex = 2, EdgeKind = 2,
            EdgeIntervals = new[] { Interval(0, 1, 1, 2), Interval(1, 2, 1, 1) }
        };
        Fixed64 halfRoot = Fixed64.FromRaw(3_037_000_500);
        // Equal Z/W components represent an exact quarter turn even though
        // the quaternion's normalized Q32.32 components have been rounded.
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, halfRoot, halfRoot);
        Assert.True(Visit(new(Fixed64.Two, Fixed64.Zero, 5 * Fixed64.Quarter), rotation,
            Fixed64.One, Fixed64.One, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps);
        var edge = Assert.Single(domain.Edges);
        Assert.True(edge.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(new Vector3d(1, 0, 1), target);
        Fixed64 expected = Fixed64.One - FixedMath.Sqrt((Fixed64)5) * Fixed64.Quarter;
        Assert.InRange(edge.Contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
    }

    [Fact]
    public void FiniteBoundaryConnectsOnlyTouchingCoreIntervalsAndTestsActualIntersections()
    {
        var worker = new CapsulePrismBoundaryContacts();
        Assert.True(worker.TryGetContact(new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            (Fixed64)4, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, new(0, -2), new(0, 2), new(-2, 0),
            Interval(1, 2, 1, 2), -Fixed64.One, Fixed64.One, 0, out _));
        // The retained vertical line spans core parameters [1/4,3/4].
        // Closed touching endpoints connect; disjoint intervals on either side do not.
        Assert.True(worker.ConnectsSupport(default));
        Assert.True(worker.ConnectsSupport(Interval(1, 8, 1, 4)));
        Assert.True(worker.ConnectsSupport(Interval(3, 4, 7, 8)));
        Assert.False(worker.ConnectsSupport(Interval(0, 1, 1, 8)));
        Assert.False(worker.ConnectsSupport(Interval(7, 8, 1, 1)));
        Signed320 one = Signed320.ExtendValue(Signed192.Signed(1));
        Signed320 two = Signed320.ExtendValue(Signed192.Signed(2));
        Assert.True(worker.ContainsPoint(one, two, Fixed64.Zero));
        Assert.False(worker.ContainsPoint(default, one, Fixed64.Zero));
        Assert.False(worker.ContainsPoint(one, two, (Fixed64)3));
    }

    [Fact]
    public void RoundedCapBoundaryCanBePhysicalWithoutConnectingTheStraightCore()
    {
        var worker = new CapsulePrismBoundaryContacts();
        Assert.True(worker.TryGetContact(new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, Vector3d.Zero, Fixed64.Zero, new(0, -2), new(0, 2), new(-2, 0),
            Interval(1, 2, 1, 2), 5 * Fixed64.Quarter, 3 * Fixed64.Half, 0, out var contact));
        Assert.False(worker.ConnectsSupport(default));
        Assert.False(worker.ConnectsSupport(Interval(0, 1, 1, 1)));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(Fixed64.Zero, target.X); Assert.Equal(Fixed64.Zero, target.Z);
        Assert.Equal(5 * Fixed64.Quarter, target.Y);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void BoundaryNormalsRetainTheDeclaredInwardSideOrCap(int vertical)
    {
        var worker = new CapsulePrismBoundaryContacts();
        Assert.True(worker.TryGetContact(new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, new(0, -2), new(0, 2), new(2, 0),
            default, Fixed64.Zero, Fixed64.Zero, vertical, out var contact));
        Assert.Equal(vertical == 0 ? Vector3d.Right : vertical > 0 ? Vector3d.Up : Vector3d.Down, contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(Vector3d.Zero, target);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void BoundaryOwnerRejectsEmptyOrInvalidLineGeometry(int invalid)
    {
        var worker = new CapsulePrismBoundaryContacts();
        Assert.False(worker.TryGetContact(new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, new(0, -2), invalid == 0 ? new(0, -2) : new(0, 2),
            invalid == 1 ? Vector2d.Zero : new(-2, 0), default,
            invalid == 2 ? Fixed64.One : invalid == 3 ? (Fixed64)4 : Fixed64.Zero,
            invalid == 3 ? (Fixed64)4 : Fixed64.Zero, 0, out _));
    }

    [Fact]
    public void BoundaryOwnerRejectsAnUnprovenTwoDimensionalRectangle()
    {
        var worker = new CapsulePrismBoundaryContacts();
        Assert.Throws<ArgumentException>(() => worker.TryGetContact(Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, new(0, -2), new(0, 2), new(-2, 0),
            default, -Fixed64.One, Fixed64.One, 0, out _));
        Assert.Throws<ArgumentException>(() => worker.TryGetContact(Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, new(0, -2), new(0, 2), new(-2, 0),
            Interval(1, 4, 3, 4), -Fixed64.One, Fixed64.One, 0, out _));
    }

    [Theory]
    [InlineData(200)]
    [InlineData(256)]
    public void WideLocalCapsuleChartPreservesCoordinateScaleAndPhysicalAdmission(int denominatorBit)
    {
        // A retained native rational cut and quaternion can require a local
        // denominator beyond Signed192. Mapping still means exactly 2*D*raw(p).
        Signed320 denominator = denominatorBit == 256 ? new(1, 0, 0, 0, 1) : new(0, 256, 0, 0, 1);
        BigInteger exactD = (BigInteger.One << denominatorBit) + 1;
        var frame = new CapsuleFiniteSectionFrame(Fixed64.Two, Fixed64.Half, denominator);
        Vector3d point = new(Fixed64.Quarter, -Fixed64.Half, Fixed64.Zero);
        WideAxis3 mapped = frame.Transform(point);
        Assert.Equal(2 * exactD * point.X.m_rawValue, Integer(mapped.X));
        Assert.Equal(2 * exactD * point.Y.m_rawValue, Integer(mapped.Y));
        Assert.Equal(BigInteger.Zero, Integer(mapped.Z));
        Assert.True(frame.ContainsTransformedPoint(mapped));
        Assert.True(frame.ContainsPoint(new(Fixed64.Zero, 3 * Fixed64.Half, Fixed64.Zero)));
        Assert.False(frame.ContainsPoint(Vector3d.Right));
        Assert.False(frame.ContainsPoint(new(Fixed64.Zero, 3 * Fixed64.Half + Fixed64.MinIncrement, Fixed64.Zero)));
    }

    [Fact]
    public void TinyNativeBoundaryPerturbationPreservesTheRotatedCapsuleWitness()
    {
        // This legal native cut differs from 1/2 by less than 2^-150.
        // Its coprime denominator and general quaternion require a wide local
        // chart, while the actual physical placement stays near the origin.
        Signed832 numerator = Signed832.ExtendValue(new Signed320(0, 0, 1UL << 22, 0, 2));
        Signed832 denominator = Signed832.ExtendValue(new Signed320(0, 0, 1UL << 23, 0, 2));
        var exactCut = new RationalSegmentInterval(numerator, denominator, numerator, denominator);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)31, (Fixed64)23);
        Vector3d center = new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero);
        var worker = new CapsulePrismBoundaryContacts();
        Assert.True(worker.TryGetContact(center, rotation, Fixed64.Two, Fixed64.Half, Vector3d.Zero,
            Fixed64.Zero, new(0, -1), new(0, 1), new(-1, 0), exactCut,
            -Fixed64.Quarter, Fixed64.Quarter, 0, out var contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d moving));
        Assert.Equal(Fixed64.Zero, target.X); Assert.Equal(Fixed64.Zero, target.Z);
        Assert.InRange(target.Y.m_rawValue, (-Fixed64.Quarter).m_rawValue, Fixed64.Quarter.m_rawValue);
        Assert.True(contact.Depth > Fixed64.Zero); Assert.False(contact.DepthIsClamped);
        Assert.InRange(moving.X.m_rawValue + contact.Depth.m_rawValue, -1L, 1L);
        Assert.Equal(target.Y, moving.Y); Assert.Equal(target.Z, moving.Z);
        // An independent capsule-distance oracle checks the rounded world
        // witness against the authored rigid frame, allowing two raw units
        // for final three-coordinate lattice materialization.
        RationalPoint3d.FromScalarPoint(moving).GetLocalPointNumerators(center, rotation,
            out Signed576 x, out Signed576 y, out Signed576 z, out Signed320 d);
        BigInteger dx = Integer(Signed320.NarrowValue(x)), dy = BigInteger.Abs(Integer(Signed320.NarrowValue(y)));
        BigInteger dz = Integer(Signed320.NarrowValue(z)), exactD = Integer(d);
        dy = BigInteger.Max(BigInteger.Zero, dy - Fixed64.One.m_rawValue * exactD);
        BigInteger distanceSquared = dx * dx + dy * dy + dz * dz;
        Assert.InRange(distanceSquared, BigInteger.Pow((Fixed64.Half.m_rawValue - 2) * exactD, 2),
            BigInteger.Pow((Fixed64.Half.m_rawValue + 2) * exactD, 2));
        // Reusing the worker exercises the retained scratch after a wider query.
        var reference = worker;
        Assert.True(reference.TryGetContact(center, rotation, Fixed64.Two, Fixed64.Half, Vector3d.Zero,
            Fixed64.Zero, new(0, -1), new(0, 1), new(-1, 0), Interval(1, 2, 1, 2),
            -Fixed64.Quarter, Fixed64.Quarter, 0, out var baseline));
        Assert.Equal(baseline.Depth, contact.Depth);
        Assert.True(baseline.FirstAnchor.TryGetPoint(out Vector3d referenceMoving));
        Assert.True(baseline.SecondAnchor.TryGetPoint(out Vector3d referenceTarget));
        Assert.Equal(referenceMoving, moving); Assert.Equal(referenceTarget, target);
    }

    [Fact]
    public void FullyCoveredSurfaceRetainsOverlapWithoutInventingAResponse()
    {
        var domain = new CaptureDomain(new RationalSegmentInterval[] { default })
        { EdgeIntervals = Array.Empty<RationalSegmentInterval>() };
        Assert.False(WideConvexPrismRelations.VisitCapsuleContacts(new(Fixed64.Zero, Fixed64.Zero, 5 * Fixed64.Quarter),
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, Vector3d.Zero, Fixed64.Zero,
            new Vector2d[] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) }, Fixed64.One,
            Reject, domain, out bool overlaps));
        Assert.True(overlaps);
        Assert.Empty(domain.Supports); Assert.Empty(domain.Edges);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedCapsuleSeparatesPhysicalBoundaryFromOptionalRecovery(bool includeRecovery)
    {
        var domain = new CaptureDomain(new RationalSegmentInterval[] { default }) { IncludeRecovery = includeRecovery };
        Assert.Equal(includeRecovery, Visit(Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Half, Fixed64.Quarter, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps);
        Assert.Empty(domain.Edges);
        if (includeRecovery)
        {
            var support = Assert.Single(domain.Supports);
            Assert.False(support.Physical);
            Assert.Equal(5 * Fixed64.Quarter, support.Contact.Depth);
        }
        else Assert.Empty(domain.Supports);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void CapsuleEdgeAndCornerContactsMatchIndependentAxisAlignedDistance(int coreAxis)
    {
        Fixed64 halfRoot = Fixed64.FromRaw(3_037_000_500);
        FixedQuaternion rotation = coreAxis == 0 ? new(Fixed64.Zero, Fixed64.Zero, halfRoot, halfRoot)
            : coreAxis == 2 ? new(halfRoot, Fixed64.Zero, Fixed64.Zero, halfRoot) : FixedQuaternion.Identity;
        for (int axial = -3; axial <= 3; axial += 3)
        for (int firstSign = -1; firstSign <= 1; firstSign += 2)
        for (int secondSign = -1; secondSign <= 1; secondSign += 2)
        for (int size = 0; size < 3; size++)
        for (int explicitWindow = 0; explicitWindow < 2; explicitWindow++)
        {
            Vector3d center = Vector3d.Zero;
            center[coreAxis] = (Fixed64)axial;
            center[(coreAxis + 1) % 3] = firstSign * 5 * Fixed64.Quarter;
            center[(coreAxis + 2) % 3] = secondSign * 5 * Fixed64.Quarter;
            Fixed64 radius = size == 0 ? Fixed64.Quarter : size == 1 ? Fixed64.Two : (Fixed64)4;
            // The two transverse gaps are exactly 1/4. A core centered at
            // +/-3 adds a unit axial gap; a centered core overlaps the edge.
            int squaredNumerator = axial == 0 ? 2 : 18;
            var domain = new CaptureDomain(Array.Empty<RationalSegmentInterval>())
            { EdgeIntervals = explicitWindow == 0 ? new RationalSegmentInterval[] { default } : new[] { Interval(0, 1, 1, 1) } };
            Assert.Equal(size != 0, Visit(center, rotation, Fixed64.Two, radius, Fixed64.One, domain, out bool overlaps));
            Assert.Equal(size != 0, overlaps);
            Assert.Empty(domain.Supports);
            if (size == 0) { Assert.Empty(domain.Edges); continue; }
            if (axial != 0) Assert.Single(domain.Edges);
            else Assert.NotEmpty(domain.Edges);
            var targets = new HashSet<Vector3d>();
            foreach (var edge in domain.Edges)
            {
                Assert.True(edge.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
                // Parallel horizontal edges and their two endpoint vertices
                // are distinct finite features of the same closest family.
                // This math domain does not promise cross-feature grouping.
                Assert.True(targets.Add(target));
                Assert.Equal((Fixed64)firstSign, target[(coreAxis + 1) % 3]);
                Assert.Equal((Fixed64)secondSign, target[(coreAxis + 2) % 3]);
                if (axial != 0) Assert.Equal((Fixed64)(axial / 3), target[coreAxis]);
                else Assert.InRange(target[coreAxis].m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
                Fixed64 expected = radius - FixedMath.Sqrt((Fixed64)squaredNumerator) * Fixed64.Quarter;
                Assert.InRange(edge.Contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
            }
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void NonparallelEdgeWindowRetainsOnlyTheGenuineStationaryClosestPair(int window)
    {
        RationalSegmentInterval clip = window == 0 ? Interval(0, 1, 1, 4)
            : window == 1 ? Interval(1, 4, 3, 4) : Interval(3, 4, 1, 1);
        var domain = new CaptureDomain(Array.Empty<RationalSegmentInterval>())
        { EdgeVertex = 2, EdgeKind = 1, EdgeIntervals = new[] { clip } };
        // The vertical core and horizontal cap edge have their unique closest
        // pair at s=t=1/2. A retained endpoint on either side is an artificial
        // cut, and must not replace that full-prism supporting feature.
        Assert.Equal(window == 1, Visit(new(Fixed64.Zero, Fixed64.One, 5 * Fixed64.Quarter),
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.Half, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps); Assert.Empty(domain.Supports);
        if (window != 1) { Assert.Empty(domain.Edges); return; }
        var edge = Assert.Single(domain.Edges);
        Assert.Equal(Fixed64.Quarter, edge.Contact.Depth);
        Assert.Equal(Vector3d.Backward, edge.Contact.Normal);
        Assert.True(edge.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(new Vector3d(0, 1, 1), target);
    }

    [Fact]
    public void ACoreCrossingARealPrismEdgeCannotInventANonzeroSeparatingEdgeContact()
    {
        Fixed64 halfRoot = Fixed64.FromRaw(3_037_000_500);
        var domain = new CaptureDomain(Array.Empty<RationalSegmentInterval>());
        Assert.False(Visit(new Vector3d(2, 0, 1), new(Fixed64.Zero, Fixed64.Zero, halfRoot, halfRoot),
            (Fixed64)4, Fixed64.Quarter, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps); Assert.Empty(domain.Edges); Assert.Empty(domain.Supports);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CoveredMinimumSupportSelectsTheNearestRetainedRecoveryPlane(int side)
    {
        var domain = new CaptureDomain(new RationalSegmentInterval[] { default })
        { RejectNormalZ = -1 };
        Assert.True(Visit(new(side * Fixed64.Quarter, Fixed64.Zero, 3 * Fixed64.Quarter), FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps);
        var support = Assert.Single(domain.Supports);
        Assert.False(support.Physical);
        Assert.Equal(5 * Fixed64.Quarter, support.Contact.Depth);
        Assert.Equal(side > 0 ? Vector3d.Left : Vector3d.Right, support.Contact.Normal);
        Assert.True(support.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(new Vector3d((Fixed64)side, Fixed64.Zero, 3 * Fixed64.Quarter), target);
        Assert.Empty(domain.Edges);
    }

    [Fact]
    public void RecoveryOnlyContactOutsideThePrismStillChecksRealEdges()
    {
        Fixed64 halfRoot = Fixed64.FromRaw(3_037_000_500);
        var domain = new CaptureDomain(new RationalSegmentInterval[] { default });
        Assert.True(Visit(new(Fixed64.Zero, Fixed64.Zero, 5 * Fixed64.Quarter),
            new(halfRoot, Fixed64.Zero, Fixed64.Zero, halfRoot), (Fixed64)4,
            Fixed64.Half, Fixed64.One, domain, out bool overlaps));
        Assert.True(overlaps);
        var support = Assert.Single(domain.Supports);
        Assert.False(support.Physical); Assert.Equal(3 * Fixed64.Half, support.Contact.Depth);
        Assert.Empty(domain.Edges);
    }

    [Fact]
    public void PointFanCanRejectOneWholeIntervalWhileRetainingTheOther()
    {
        var domain = new CaptureDomain(new[] { Interval(0, 1, 1, 4), Interval(3, 4, 1, 1) })
        { IncludeRecovery = false };
        Assert.True(WideConvexPrismRelations.VisitCapsuleContacts(new(Fixed64.Zero, Fixed64.Zero, 5 * Fixed64.Quarter),
            FixedQuaternion.Identity, (Fixed64)4, Fixed64.Half, Vector3d.Zero, Fixed64.Zero,
            new Vector2d[] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) }, (Fixed64)3,
            AdmitPositiveHeight, domain, out bool overlaps));
        Assert.True(overlaps);
        Assert.Equal(2, domain.Supports.Count);
        for (int index = 0; index < domain.Supports.Count; index++)
        {
            var support = domain.Supports[index];
            Assert.Equal(1, support.Interval); Assert.True(support.Physical);
            Assert.True(support.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
            Assert.Equal(new Vector3d(0, index + 1, 1), target);
        }
        Assert.Empty(domain.Edges);
    }

    [Fact]
    public void RetainedCoreWindowSuppliesItsPreferredPointAndPhysicalExtremes()
    {
        var domain = new CaptureDomain(new[] { Interval(1, 4, 3, 4) });
        Assert.True(Visit(new(Fixed64.Zero, Fixed64.Zero, 5 * Fixed64.Quarter), FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.Half, Fixed64.Two, domain, out bool overlaps));
        Assert.True(overlaps);
        Assert.Equal(3, domain.Supports.Count);
        for (int index = 0; index < domain.Supports.Count; index++)
        {
            Assert.True(domain.Supports[index].Contact.SecondAnchor.TryGetPoint(out Vector3d target));
            Assert.Equal(new Vector3d(Fixed64.Zero, index == 0 ? Fixed64.Zero : index == 1 ? -Fixed64.Half : Fixed64.Half, Fixed64.One), target);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void BlockedOrCollapsedBarrelExtremesRetainOnlyDistinctAdmittedSamples(int family)
    {
        RationalSegmentInterval interval = family == 2 ? Interval(0, 1, 1, 2)
            : family == 3 ? Interval(1, 2, 1, 2) : family == 4 ? Interval(1, 2, 1, 1) : default;
        var domain = new CaptureDomain(new[] { interval }) { IncludeRecovery = false };
        PrismPointAdmission admit = family == 1 ? AdmitZeroHeight : family == 3 ? Admit
            : family == 4 ? AdmitPositiveHeight : AdmitNonnegativeHeight;
        Assert.True(WideConvexPrismRelations.VisitCapsuleContacts(new(Fixed64.Zero, Fixed64.Zero, 5 * Fixed64.Quarter),
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero,
            new Vector2d[] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) }, Fixed64.Two,
            admit, domain, out bool overlaps));
        Assert.True(overlaps); Assert.Empty(domain.Edges);
        Assert.Equal(family == 0 || family == 4 ? 2 : 1, domain.Supports.Count);
        for (int index = 0; index < domain.Supports.Count; index++)
        {
            var support = domain.Supports[index];
            Assert.True(support.Physical); Assert.Equal(Fixed64.Quarter, support.Contact.Depth);
            Assert.True(support.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
            Assert.Equal(new Vector3d(Fixed64.Zero, family == 4 && index == 0 ? Fixed64.FromFraction(1, 3) : (Fixed64)index, Fixed64.One), target);
        }
    }

    [Fact]
    public void RotatedCapsuleReselectionClassifiesAgainstItsAuthoredCoreAxis()
    {
        Fixed64 halfRoot = Fixed64.FromRaw(3_037_000_500);
        var domain = new CaptureDomain(new RationalSegmentInterval[] { default })
        { IncludeRecovery = false };
        // The X-aligned core spans [0,4]. Its nearest front support is covered.
        // Side planes at Y=+/-1 miss its radius, while X=1 intersects its core.
        // Treating it as upright reverses those physical classifications.
        Assert.True(WideConvexPrismRelations.VisitCapsuleContacts(new(Fixed64.Two, Fixed64.Zero, 3 * Fixed64.Quarter),
            new(Fixed64.Zero, Fixed64.Zero, halfRoot, halfRoot), (Fixed64)4, Fixed64.Half, Vector3d.Zero, Fixed64.Zero,
            new Vector2d[] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) }, Fixed64.One,
            AdmitNonBack, domain, out bool overlaps));
        Assert.True(overlaps); Assert.Empty(domain.Edges);
        var support = Assert.Single(domain.Supports);
        Assert.True(support.Physical); Assert.Equal(3 * Fixed64.Half, support.Contact.Depth);
        Assert.Equal(Vector3d.Left, support.Contact.Normal);
        Assert.True(support.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Zero, 3 * Fixed64.Quarter), target);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueCapPlaneIntersectionDoesNotMakeTheProjectedFarTipPhysical(bool includeRecovery)
    {
        var domain = new CaptureDomain(new RationalSegmentInterval[] { default }) { IncludeRecovery = includeRecovery };
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45);
        Assert.Equal(includeRecovery, WideConvexPrismRelations.VisitCapsuleContacts(Vector3d.Zero, rotation,
            (Fixed64)10, Fixed64.Half, Vector3d.Zero, Fixed64.Zero,
            new Vector2d[] { new(-10, -10), new(10, -10), new(10, 10), new(-10, 10) }, Fixed64.Two,
            Admit, domain, out bool overlaps));
        Assert.True(overlaps); Assert.Empty(domain.Edges);
        // The long diagonal core crosses both caps. Projecting its opposite
        // support tip onto a cap leaves |X| about 3.536 at |Y|=2, nearly four
        // units from the actual core. Plane intersection alone therefore cannot
        // classify that witness as physical for a radius-1/2 capsule.
        if (!includeRecovery) { Assert.Empty(domain.Supports); return; }
        var support = Assert.Single(domain.Supports);
        Assert.False(support.Physical);
        Assert.True(support.Contact.SecondAnchor.TryGetPoint(out Vector3d target));
        Assert.InRange(FixedMath.Abs(target.X).m_rawValue, ((Fixed64)7 * Fixed64.Half).m_rawValue,
            Fixed64.FromFraction(18, 5).m_rawValue);
        Assert.Equal(Fixed64.Two, FixedMath.Abs(target.Y)); Assert.Equal(Fixed64.Zero, target.Z);
        Assert.InRange(support.Contact.Depth.m_rawValue, ((Fixed64)6).m_rawValue,
            Fixed64.FromFraction(61, 10).m_rawValue);
    }

    private static bool AdmitNonnegativeHeight(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator) => y.Signs[0] >= 0;

    private static bool AdmitZeroHeight(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator) => y.Signs[0] == 0;

    private static bool AdmitNonBack(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator) => normal.Z.Sign >= 0;

    private static bool AdmitPositiveHeight(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator) => y.Signs[0] > 0;

    private static bool Reject(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator) => false;

    private static BigInteger Integer(Signed320 value)
    {
        BigInteger unsigned = ((BigInteger)value.Word4 << 256) + ((BigInteger)value.Word3 << 192)
            + ((BigInteger)value.Word2 << 128) + ((BigInteger)value.Word1 << 64) + value.Word0;
        return value.Sign < 0 ? unsigned - (BigInteger.One << 320) : unsigned;
    }

    private static bool Visit(Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius,
        Fixed64 halfThickness, CaptureDomain domain, out bool overlaps) =>
        WideConvexPrismRelations.VisitCapsuleContacts(center, rotation, length, radius, Vector3d.Zero,
            Fixed64.Zero, new Vector2d[] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) },
            halfThickness, Admit, domain, out overlaps);

    private static bool Admit(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator) => true;

    private sealed class CaptureDomain : IPrismCapsuleContactDomain
    {
        private readonly RationalSegmentInterval[] _supportIntervals;
        internal readonly List<(FixedContactAnchors Contact, bool Physical, int Interval)> Supports = new();
        internal readonly List<(FixedContactAnchors Contact, int Vertex, int Kind, int Interval)> Edges = new();
        internal RationalSegmentInterval[] EdgeIntervals = { default };
        internal int EdgeVertex = -1, EdgeKind = -1;
        internal bool RejectEdgeEndpoints;
        internal int RejectNormalZ;
        private readonly RationalSegmentInterval[] _retainedSupport;
        public bool IncludeRecovery { get; set; } = true;

        internal CaptureDomain(RationalSegmentInterval[] supportIntervals)
        {
            _supportIntervals = supportIntervals;
            _retainedSupport = new RationalSegmentInterval[supportIntervals.Length];
        }

        public ReadOnlySpan<RationalSegmentInterval> GetSupportIntervals(in WideDirection3d normal,
            ReadOnlySpan<Signed832> projected, Signed832 denominator, in RationalSegmentInterval solidClip)
        {
            if (RejectNormalZ != 0 && normal.Z.Sign == RejectNormalZ) return ReadOnlySpan<RationalSegmentInterval>.Empty;
            int count = 0;
            Signed832 one = Integer(1);
            // Every candidate axis has its own finite prism window. Intersect
            // the fixture's exposure masks with that actual window on each call.
            foreach (RationalSegmentInterval mask in _supportIntervals)
            {
                Signed832 ln = mask.IsClipped ? mask.WideLowerNumerator : default, ld = mask.IsClipped ? mask.WideLowerDenominator : one;
                Signed832 un = mask.IsClipped ? mask.WideUpperNumerator : one, ud = mask.IsClipped ? mask.WideUpperDenominator : one;
                if (solidClip.IsClipped)
                {
                    if (RationalSegmentInterval.CompareParameters(ln, ld, solidClip.WideLowerNumerator, solidClip.WideLowerDenominator) < 0)
                    { ln = solidClip.WideLowerNumerator; ld = solidClip.WideLowerDenominator; }
                    if (RationalSegmentInterval.CompareParameters(un, ud, solidClip.WideUpperNumerator, solidClip.WideUpperDenominator) > 0)
                    { un = solidClip.WideUpperNumerator; ud = solidClip.WideUpperDenominator; }
                }
                if (RationalSegmentInterval.CompareParameters(ln, ld, un, ud) <= 0)
                    _retainedSupport[count++] = new RationalSegmentInterval(ln, ld, un, ud);
            }
            return _retainedSupport.AsSpan(0, count);
        }

        public void AddSupportContact(in FixedContactAnchors contact, in WideDirection3d normal,
            in WideConvexPrismRelations.ProjectionDepth depth, bool physical, int interval) => Supports.Add((contact, physical, interval));

        public ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(int vertex, int kind) =>
            EdgeVertex < 0 || vertex == EdgeVertex && kind == EdgeKind ? EdgeIntervals : ReadOnlySpan<RationalSegmentInterval>.Empty;

        public bool AdmitEdgePoint(int vertex, int kind, in Signed832 edgeN, in Signed832 edgeD, in WideDirection3d normal)
        {
            if (!RejectEdgeEndpoints) return true;
            foreach (RationalSegmentInterval interval in EdgeIntervals)
                if (RationalSegmentInterval.CompareParameters(edgeN, edgeD, interval.WideLowerNumerator, interval.WideLowerDenominator) > 0
                    && RationalSegmentInterval.CompareParameters(edgeN, edgeD, interval.WideUpperNumerator, interval.WideUpperDenominator) < 0)
                    return true;
            return false;
        }

        public void AddEdgeContact(in FixedContactAnchors contact, in WideDirection3d normal, int vertex, int kind, int interval) =>
            Edges.Add((contact, vertex, kind, interval));
    }

    private static Signed832[] Projected(in RationalPoint3d first, in RationalPoint3d second, out Signed832 pointD)
    {
        Assert.True(first.Denominator.Equals(second.Denominator));
        pointD = Signed832.ExtendValue(first.Denominator);
        return new[] { Signed832.ExtendValue(first.X), Signed832.ExtendValue(first.Y), Signed832.ExtendValue(first.Z),
            Signed832.ExtendValue(second.X), Signed832.ExtendValue(second.Y), Signed832.ExtendValue(second.Z) };
    }

    private static Signed832 Integer(int value) => Signed832.ExtendValue(Signed192.Signed(value));

    private static RationalSegmentInterval Interval(int ln, int ld, int un, int ud) =>
        new(Integer(ln), Integer(ld), Integer(un), Integer(ud));

    private static void AssertInterval(in RationalSegmentInterval interval, int ln, int ld, int un, int ud)
    {
        Signed832 lowerN = interval.IsClipped ? interval.WideLowerNumerator : default;
        Signed832 lowerD = interval.IsClipped ? interval.WideLowerDenominator : Integer(1);
        Signed832 upperN = interval.IsClipped ? interval.WideUpperNumerator : Integer(1);
        Signed832 upperD = interval.IsClipped ? interval.WideUpperDenominator : Integer(1);
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(lowerN, lowerD, Integer(ln), Integer(ld)));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(upperN, upperD, Integer(un), Integer(ud)));
    }
}
