//=======================================================================
// FixedTriangle.FiniteCone.PlaneSection.Scopes.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Collections.Generic;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleFiniteConePlaneSectionScopesTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedRigidFrame_ShouldKeepAPrimitiveCanonicalPlaneNormal(bool tilted)
    {
        FixedQuaternion rotation = tilted
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5))
            : FixedQuaternion.Identity;
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, rotation,
            Vector3d.Zero, rotation, (Fixed64)4, (Fixed64)2);
        // A common rigid pose does not change this plane in cone coordinates.
        // Its primitive covector avoids retaining arbitrary quaternion scale
        // in every subsequent homogeneous critical-event construction.
        Assert.True(frame.Normal.X.IsZero && frame.Normal.Z.IsZero);
        Assert.Equal(Signed320.ExtendValue(Signed192.Signed(-1)), frame.Normal.Y);
        Assert.True(ConePlaneRayEvents.IntersectsTriangle(frame, triangle));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void TriangleAdmission_UsesEveryClosedEdgeBeforeSectionContainment(int first)
    {
        Vector3d[] vertices = { new(-2, 0, 0), new(2, 0, 0), new(0, 0, 3) };
        var triangle = new FixedTriangle(vertices[first], vertices[(first + 1) % 3], vertices[(first + 2) % 3]);
        Assert.True(ConePlaneRayEvents.IntersectsTriangle(Frame(triangle), triangle));
    }

    [Fact]
    public void TriangleAdmission_UsesOneCertifiedSectionPointWhenAllEdgesMiss()
    {
        var containing = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var outside = new FixedTriangle(new Vector3d(2, 0, 2), new Vector3d(3, 0, 2), new Vector3d(2, 0, 3));
        var frame = Frame(containing);
        for (int edge = 0; edge < 3; edge++) Assert.False(frame.IntersectsSegment(containing.GetEdge(edge)));
        Assert.True(ConePlaneRayEvents.IntersectsTriangle(frame, containing));
        Assert.False(ConePlaneRayEvents.IntersectsTriangle(Frame(new FixedTriangle()), containing));
        Assert.False(ConePlaneRayEvents.IntersectsTriangle(frame, outside));
        Assert.False(ConePlaneRayEvents.IntersectsTriangle(frame,
            new FixedTriangle(-Vector3d.Right, Vector3d.Zero, Vector3d.Right)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void TriangleAdmission_HandlesNonaxialSectionAndExactRimTangency(int fixture)
    {
        Fixed64 x = fixture == 0 ? (Fixed64)3 / 2 : fixture == 1 ? Fixed64.Two : Fixed64.Two + Fixed64.FromRaw(1);
        var triangle = new FixedTriangle(new Vector3d(x, (Fixed64)(-8), (Fixed64)(-8)), new Vector3d(x, (Fixed64)8, (Fixed64)(-8)), new Vector3d(x, Fixed64.Zero, (Fixed64)8));
        var frame = Frame(triangle);
        for (int edge = 0; edge < 3; edge++) Assert.False(frame.IntersectsSegment(triangle.GetEdge(edge)));
        Assert.Equal(fixture != 2, ConePlaneRayEvents.IntersectsTriangle(frame, triangle));
    }

    [Fact]
    public void TriangleAdmission_AxisFreeSectionOutsideTheFiniteDomainRemainsSeparated()
    {
        Fixed64 x = (Fixed64)3 / 2;
        var containing = new FixedTriangle(new Vector3d(x, (Fixed64)(-8), (Fixed64)(-8)),
            new Vector3d(x, (Fixed64)8, (Fixed64)(-8)), new Vector3d(x, Fixed64.Zero, (Fixed64)8));
        var outside = new FixedTriangle(new Vector3d(x, Fixed64.Zero, (Fixed64)5),
            new Vector3d(x, Fixed64.One, (Fixed64)5), new Vector3d(x, Fixed64.Zero, (Fixed64)6));
        var frame = Frame(containing);
        Assert.True(ConePlaneRayEvents.IntersectsTriangle(frame, containing));
        Assert.False(ConePlaneRayEvents.IntersectsTriangle(frame, outside));
    }

    [Fact]
    public void TriangleAdmission_ZeroRadiusPlaneSectionRetainsTheWholeAxisOrRejectsAnExactGap()
    {
        var containing = new FixedTriangle(new Vector3d(0, -8, -8), new Vector3d(0, 8, -8), new Vector3d(0, 0, 8));
        var frame = new ConePlaneRayFrame(containing, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Zero);
        for (int edge = 0; edge < 3; edge++) Assert.False(frame.IntersectsSegment(containing.GetEdge(edge)));
        Assert.True(ConePlaneRayEvents.IntersectsTriangle(frame, containing));
        Fixed64 gap = Fixed64.FromRaw(1);
        var separated = new FixedTriangle(new Vector3d(gap, (Fixed64)(-8), (Fixed64)(-8)),
            new Vector3d(gap, (Fixed64)8, (Fixed64)(-8)), new Vector3d(gap, Fixed64.Zero, (Fixed64)8));
        frame = new ConePlaneRayFrame(separated, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Zero);
        Assert.False(ConePlaneRayEvents.IntersectsTriangle(frame, separated));
    }

    [Theory]
    [InlineData(-3, 0, false)]
    [InlineData(-2, 0, true)]
    [InlineData(0, 0, true)]
    [InlineData(2, 0, true)]
    [InlineData(3, 0, false)]
    [InlineData(-3, 2, false)]
    [InlineData(-2, 2, true)]
    [InlineData(0, 2, true)]
    [InlineData(2, 2, true)]
    [InlineData(3, 2, false)]
    public void TriangleAdmission_HandlesAxialCapApexAndZeroRadius(int y, int radius, bool expected)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, y, -8), new Vector3d(8, y, -8), new Vector3d(0, y, 8));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)radius);
        Assert.Equal(expected, ConePlaneRayEvents.IntersectsTriangle(frame, triangle));
    }

    [Fact]
    public void CanonicalWorldNormal_UsesExactAuthoredSignBeforeFinalRotationAndRounding()
    {
        var plane = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Forward);
        var reverse = new FixedTriangle(plane.C, plane.B, plane.A);
        WideAxis3 canonical = Frame(plane).AuthoredNormal;
        WideAxis3 reversed = Frame(reverse).AuthoredNormal;
        Assert.True(canonical.X.IsZero && canonical.Z.IsZero);
        Assert.Equal(1, canonical.Y.Sign);
        Assert.Equal(canonical.X, reversed.X); Assert.Equal(canonical.Y, reversed.Y); Assert.Equal(canonical.Z, reversed.Z);
        Assert.Equal(Vector3d.Up, Frame(plane).GetWorldNormal(FixedQuaternion.Identity));
        Assert.Equal(Vector3d.Up, Frame(reverse).GetWorldNormal(FixedQuaternion.Identity));
        Fixed64 halfRoot = FixedMath.Sqrt(Fixed64.Half);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, halfRoot, halfRoot);
        var rotated = new ConePlaneRayFrame(plane, Vector3d.Zero, rotation,
            Vector3d.Zero, rotation, (Fixed64)4, Fixed64.Two);
        Assert.Equal(Vector3d.Left, rotated.GetWorldNormal(rotation));
        // The leading exact component fixes orientation even when it rounds
        // away. Re-canonicalizing the materialized normal would reverse Z.
        var subRaw = new FixedTriangle(Vector3d.Zero,
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.FromRaw(1)), Vector3d.Up);
        Assert.Equal(-Vector3d.Forward, Frame(subRaw).GetWorldNormal(FixedQuaternion.Identity));
        Assert.Equal(Vector3d.Zero, Frame(new FixedTriangle()).GetWorldNormal(FixedQuaternion.Identity));
    }

    [Fact]
    public void DegeneratePlane_RetainsGeneralInventoryButAdmitsNoPointOrDirectionalCertificate()
    {
        var frame = Frame(new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Right * 2));
        Assert.True(frame.Normal.IsZero);
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        int count = ConePlaneRayEvents.GetIntrinsicEvents(frame, events);
        Assert.Equal(102, count);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        foreach (ConePlaneRayEvent descriptor in events[..count])
        {
            Assert.False(ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame, descriptor,
                point, root, ref positive, ref negative));
            Assert.False(positive.HasValue); Assert.False(negative.HasValue);
        }
    }

    [Theory]
    [InlineData(-3, 2, false, false)]
    [InlineData(3, 2, false, false)]
    [InlineData(0, 0, false, false)]
    [InlineData(2, 2, false, false)]
    [InlineData(0, 2, true, false)]
    [InlineData(0, 0, true, false)]
    [InlineData(-2, 2, true, false)]
    [InlineData(0, 2, false, true)]
    public void TargetedAxialCardinals_PreserveClosedAdmissionAndRequestedReplay(
        int planeY, int radius, bool invertAxis, bool extremeOddHeight)
    {
        var plane = new FixedTriangle(new Vector3d(-4, planeY, -4),
            new Vector3d(4, planeY, -4), new Vector3d(0, planeY, 4));
        Vector3d center = extremeOddHeight ? new(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero) : Vector3d.Zero;
        Fixed64 height = (Fixed64)4 + (extremeOddHeight ? Fixed64.FromRaw(1) : Fixed64.Zero);
        FixedQuaternion rotation = invertAxis
            ? new(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero) : FixedQuaternion.Identity;
        var frame = new ConePlaneRayFrame(plane, center, FixedQuaternion.Identity,
            center, rotation, height, (Fixed64)radius);
        Assert.Equal(Signed320.ExtendValue(Signed192.Signed(invertAxis ? 1 : -1)), frame.Normal.Y);
        var right = new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 0, 0, 0);
        var left = new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 1, 0, 0);
        var axis = new ConePlaneRayEvent(ConePlaneRayEventKind.Axis);
        long planeRaw = ((Fixed64)planeY).m_rawValue;
        bool admitted = 2 * Math.Abs(planeRaw) <= height.m_rawValue;
        bool collapsed = radius == 0 || planeY == (invertAxis ? -2 : 2);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        // Targeted replay must remain valid even when streaming prunes the
        // radius-zero/apex cardinal as a duplicate of the axis construction.
        for (int orientation = -1; orientation <= 1; orientation += 2)
        {
            Assert.Equal(admitted, ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame,
                right, point, root, ref positive, ref negative, requestedOrientation: orientation));
            if (!admitted)
            {
                Assert.False(positive.HasValue); Assert.False(negative.HasValue);
                continue;
            }
            Assert.True(ConePlaneRayPointExits.ContainsPoint(ConePlaneRayEventSource.Plane, frame, point, root));
            ConePlaneRaySelection selected = orientation > 0 ? positive : negative;
            Assert.True(selected.TryMaterialize(frame, orientation, center, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Fixed64 radial = collapsed ? Fixed64.Zero : Fixed64.One;
            Assert.Equal(new Vector3d(radial, (Fixed64)planeY, Fixed64.Zero), p);
            Assert.Equal(p.X, q.X); Assert.Equal(p.Z, q.Z);
            bool endpointExit = collapsed || orientation * (invertAxis ? -1 : 1) < 0;
            long endpointRaw = orientation * (height.m_rawValue / 2 + (extremeOddHeight ? 1 : 0));
            Assert.Equal(endpointExit ? endpointRaw : planeRaw, q.Y.m_rawValue);
            // The odd translated origin changes endpoint parity, while depth
            // independently rounds the half-height to the even raw integer.
            long depthRaw = endpointExit ? Math.Abs(orientation * (height.m_rawValue / 2) - planeRaw) : 0;
            Assert.Equal(depthRaw, depth.m_rawValue);
        }
        if (!admitted)
            Assert.Throws<InvalidOperationException>(() => ConePlaneRayEvents.CompareEventAnchors(
                ConePlaneRayEventSource.Plane, right, ConePlaneRayEventSource.Plane, axis, frame,
                includeCoincidentProvenance: false));
        else
            Assert.Equal(collapsed ? 0 : 1, Math.Sign(ConePlaneRayEvents.CompareEventAnchors(
                ConePlaneRayEventSource.Plane, right, ConePlaneRayEventSource.Plane, collapsed ? axis : left, frame,
                includeCoincidentProvenance: false)));
    }

    [Theory]
    [InlineData(-3, 0)]
    [InlineData(-2, 0)]
    [InlineData(0, 0)]
    [InlineData(2, 0)]
    [InlineData(3, 0)]
    [InlineData(-3, 2)]
    [InlineData(-2, 2)]
    [InlineData(0, 2)]
    [InlineData(2, 2)]
    [InlineData(3, 2)]
    public void AxialIntrinsicCohort_PreservesEveryPointAndBothExitMaxima(int height, int radius)
    {
        var plane = new FixedTriangle(new Vector3d(-2, height, -2), new Vector3d(2, height, -2), new Vector3d(0, height, 2));
        var frame = new ConePlaneRayFrame(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)radius);
        // This independent plane has X=0 but Z!=0 in its exact normal, so
        // it must retain the full inventory despite the first axial test.
        var general = Frame(new FixedTriangle(new Vector3d(-2, -2, 0), new Vector3d(2, -2, 0), new Vector3d(0, 2, 0)));
        Assert.True(general.Normal.X.IsZero); Assert.False(general.Normal.Z.IsZero);
        Span<ConePlaneRayEvent> complete = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        Span<ConePlaneRayEvent> compact = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        int fullCount = ConePlaneRayEvents.GetIntrinsicEvents(general, complete);
        int compactCount = ConePlaneRayEvents.GetIntrinsicEvents(frame, compact);
        bool singlePoint = radius == 0 || height == 2;
        Assert.Equal(102, fullCount); Assert.Equal(singlePoint ? 1 : 5, compactCount);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        int omitted = 0, admittedAxis = 0;
        Span<ulong> compactWords = stackalloc ulong[ConePlaneRayPoint.StorageWords], compactRoot = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> compactSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var compactPoint = new ConePlaneRayPoint(compactWords, compactSigns);
        Span<ulong> cpv = stackalloc ulong[ConePlaneRaySelection.StorageWords], cnv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> cps = stackalloc int[ConePlaneRaySelection.SignCount], cns = stackalloc int[ConePlaneRaySelection.SignCount];
        var compactPositive = new ConePlaneRaySelection(cpv, cps); var compactNegative = new ConePlaneRaySelection(cnv, cns);
        foreach (ConePlaneRayEvent descriptor in complete[..fullCount])
        {
            bool retained = false;
            foreach (ConePlaneRayEvent candidate in compact[..compactCount])
                retained |= descriptor.Matches(candidate);
            bool admitted = ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame, descriptor,
                point, root, ref positive, ref negative);
            if (!retained)
                omitted++;
            if (admitted)
            {
                // Omitted constructions may duplicate an admitted anchor or
                // supply a shorter certificate. Preserve every exact anchor
                // and dominate both directional values at that same point.
                bool witnessed = false, positiveCovered = !positive.HasValue, negativeCovered = !negative.HasValue;
                foreach (ConePlaneRayEvent candidate in compact[..compactCount])
                {
                    if (!ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame, candidate,
                        compactPoint, compactRoot, ref compactPositive, ref compactNegative)
                        || point.CompareTo(compactPoint, root, compactRoot) != 0) continue;
                    witnessed = true;
                    if (positive.HasValue && compactPositive.HasValue)
                        positiveCovered |= ContactQuadratic.CompareRatios(
                            ContactQuadratic.At(cpv, cps, 0, ConePlaneRaySelection.FieldWords),
                            ContactQuadratic.At(cpv, cps, 1, ConePlaneRaySelection.FieldWords), compactPositive.Root,
                            ContactQuadratic.At(pv, ps, 0, ConePlaneRaySelection.FieldWords),
                            ContactQuadratic.At(pv, ps, 1, ConePlaneRaySelection.FieldWords), positive.Root) >= 0;
                    if (negative.HasValue && compactNegative.HasValue)
                        negativeCovered |= ContactQuadratic.CompareRatios(
                            ContactQuadratic.At(cnv, cns, 0, ConePlaneRaySelection.FieldWords),
                            ContactQuadratic.At(cnv, cns, 1, ConePlaneRaySelection.FieldWords), compactNegative.Root,
                            ContactQuadratic.At(nv, ns, 0, ConePlaneRaySelection.FieldWords),
                            ContactQuadratic.At(nv, ns, 1, ConePlaneRaySelection.FieldWords), negative.Root) >= 0;
                }
                Assert.True(witnessed && positiveCovered && negativeCovered);
                if (descriptor.Kind == ConePlaneRayEventKind.LowerCircle)
                {
                    Fixed64 radial = (Fixed64)radius * (Fixed64)(2 - height) / 4;
                    Fixed64 x = descriptor.Branch == 0 ? ((descriptor.Quadrant & 1) == 0 ? radial : -radial) : Fixed64.Zero;
                    Fixed64 z = descriptor.Branch == 1 ? ((descriptor.Quadrant & 2) == 0 ? radial : -radial) : Fixed64.Zero;
                    Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, root, out Vector3d seamPoint));
                    Assert.Equal(new Vector3d(x, (Fixed64)height, z), seamPoint);
                    Assert.Equal(radius == 0 ? (Fixed64)(2 - height) : Fixed64.Zero, positive.GetRoundedMaximumDepth());
                    Assert.Equal((Fixed64)(height + 2), negative.GetRoundedMaximumDepth());
                }
            }
            if (!admitted || descriptor.Kind != ConePlaneRayEventKind.Axis) continue;
            admittedAxis++;
            Assert.True(retained);
            Assert.True(positive.TryMaterialize(frame, 1, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(new Vector3d(0, height, 0), p); Assert.Equal(Vector3d.Up * 2, q);
            Assert.Equal((Fixed64)(2 - height), depth);
            Assert.True(negative.TryMaterialize(frame, -1, out p, out q, out depth));
            Assert.Equal(new Vector3d(0, height, 0), p); Assert.Equal(Vector3d.Down * 2, q);
            Assert.Equal((Fixed64)(height + 2), depth);
        }
        Assert.Equal(singlePoint ? 101 : 97, omitted);
        Assert.Equal(height >= -2 && height <= 2 ? 1 : 0, admittedAxis);
    }

    [Fact]
    public void IntrinsicEvents_AdmitTheSharedPlaneWithoutClippingToItsSeedTriangle()
    {
        var seed = new FixedTriangle(new Vector3d(2, 0, 0), new Vector3d(3, 0, 0), new Vector3d(2, 0, 1));
        var containing = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        var frame = Frame(seed);
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        int count = ConePlaneRayEvents.GetIntrinsicEvents(frame, events);
        Span<ulong> pointWords = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRayCharts.RootWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointWords, pointSigns);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        bool foundAxis = false;
        for (int index = 0; index < count; index++)
        {
            ConePlaneRayEvent item = events[index];
            if (item.Kind != ConePlaneRayEventKind.Axis || item.Endpoint != -1) continue;
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame, item,
                point, root, ref positive, ref negative));
            Assert.False(ConePlaneRayPointExits.ContainsTrianglePoint(seed, frame, point, root));
            Assert.True(ConePlaneRayPointExits.ContainsTrianglePoint(containing, frame, point, root));
            Assert.True(positive.TryMaterialize(frame, 1, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(Vector3d.Zero, p); Assert.Equal(Vector3d.Up * 2, q); Assert.Equal(Fixed64.Two, depth);
            Assert.True(negative.TryMaterialize(frame, -1, out p, out q, out depth));
            Assert.Equal(Vector3d.Zero, p); Assert.Equal(Vector3d.Down * 2, q); Assert.Equal(Fixed64.Two, depth);
            foundAxis = true;
        }
        Assert.True(foundAxis);
    }

    [Fact]
    public void BoundaryEvents_RejectSideRootsBeyondBothFiniteEndpoints()
    {
        var seed = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        var frame = Frame(seed);
        var shortSource = new ConePlaneRayEventSource(new FixedSegment(-Vector3d.Right * Fixed64.Half, Vector3d.Right * Fixed64.Half));
        var longSource = new ConePlaneRayEventSource(new FixedSegment(-Vector3d.Right * 2, Vector3d.Right * 2));
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.BoundaryCapacity];
        int count = ConePlaneRayEvents.GetBoundaryEvents(events);
        Span<ulong> pointWords = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRayCharts.RootWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointWords, pointSigns);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        int sideRoots = 0;
        foreach (ConePlaneRayEvent item in events[..count])
        {
            if (item.Kind != ConePlaneRayEventKind.EdgeSide) continue;
            Assert.False(ConePlaneRayEvents.TryEvaluateEvent(shortSource, frame, item, point, root, ref positive, ref negative));
            Assert.False(positive.HasValue); Assert.False(negative.HasValue);
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(longSource, frame, item, point, root, ref positive, ref negative));
            Assert.True(negative.TryMaterialize(frame, -1, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(Fixed64.One, FixedMath.Abs(p.X)); Assert.Equal(Fixed64.Zero, p.Y); Assert.Equal(Fixed64.Zero, p.Z);
            Assert.Equal(new Vector3d(p.X, -(Fixed64)2, Fixed64.Zero), q); Assert.Equal(Fixed64.Two, depth);
            sideRoots++;
        }
        Assert.Equal(2, sideRoots);
    }

    [Fact]
    public void EvaluatedSelection_RetainsRationalPointUnderIrrationalDepthRootAndRestoresBorrowedStorage()
    {
        var seed = new FixedTriangle(new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, 0, 2));
        var frame = Frame(seed);
        var source = new ConePlaneRayEventSource(new FixedSegment(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Half)));
        var item = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0);
        Span<ulong> pointWords = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRayCharts.RootWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointWords, pointSigns);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, frame, item, point, root, ref positive, ref negative));
        Assert.Equal(0, WideArithmetic.GetActiveMagnitudeLength(root));
        Assert.True(WideArithmetic.GetActiveMagnitudeLength(positive.Root) > 0);
        Span<ulong> retainedWords = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> retainedSigns = stackalloc int[ConePlaneRaySelection.SignCount];
        var winner = new ConePlaneRaySelection(retainedWords, retainedSigns);
        Assert.True(winner.KeepEvaluated(positive, frame));
        Assert.False(winner.KeepEvaluated(positive, frame));
        var restored = new ConePlaneRaySelection(retainedWords, retainedSigns);
        restored.Restore(frame, winner.MaximumSource, winner.MaximumEvent);
        Assert.True(restored.TryMaterialize(frame, 1, out Vector3d p, out Vector3d q, out Fixed64 depth));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), p);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(3719550787L), Fixed64.Zero, Fixed64.Half), q);
        Assert.Equal(Fixed64.FromRaw(3719550787L), depth);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(3, true)]
    [InlineData(4, true)]
    [InlineData(5, true)]
    [InlineData(6, true)]
    [InlineData(7, false)]
    [InlineData(8, false)]
    public void SegmentDomain_RequiresTheExactFiniteLineOrPoint(int example, bool expected)
    {
        var seed = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        var frame = Frame(seed);
        Vector3d quarter = Vector3d.Right * Fixed64.Quarter;
        FixedSegment segment = example >= 6 ? new FixedSegment(quarter, quarter)
            : new FixedSegment(-Vector3d.Right * Fixed64.Half, Vector3d.Right * Fixed64.Half);
        Vector3d query = example switch
        {
            0 => -Vector3d.Right * Fixed64.FromFraction(3, 4),
            1 => Vector3d.Right * Fixed64.FromFraction(3, 4),
            2 => Vector3d.Forward * Fixed64.Quarter,
            3 => segment.Start,
            4 => segment.End,
            6 => quarter,
            8 => quarter + Vector3d.Forward * Fixed64.Quarter,
            _ => Vector3d.Zero
        };
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs); point.Set(frame.Transform(query));
        Assert.Equal(expected, ConePlaneRayPointExits.ContainsPoint(new ConePlaneRayEventSource(segment), frame, point, ReadOnlySpan<ulong>.Empty));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void ScopedSource_RejectsDescriptorsOutsideItsCohort(int example)
    {
        var frame = Frame(new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2)));
        var source = example == 3 ? ConePlaneRayEventSource.Plane
            : new ConePlaneRayEventSource(new FixedSegment(Vector3d.Left, Vector3d.Right));
        ConePlaneRayEvent item = example switch
        {
            0 => new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 0, 0, 0),
            1 => new ConePlaneRayEvent(ConePlaneRayEventKind.UpperRim, 8, 0, 0, 0),
            2 => new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeSide, feature: 1, branch: 1),
            _ => new ConePlaneRayEvent(ConePlaneRayEventKind.Apex, endpoint: 0)
        };
        if (example == 3)
            Assert.True(ConePlaneRayTestQueries.Materialize(source, frame, new ConePlaneRayEvent(ConePlaneRayEventKind.Apex), 1,
                out _, out _, out _));
        Assert.False(ConePlaneRayTestQueries.Materialize(source, frame, item, 1, out _, out _, out _));
    }

    [Fact]
    public void BoundaryRimAtZeroDepth_StillRequiresTheFiniteSegment()
    {
        var frame = Frame(new FixedTriangle(new Vector3d(-3, -2, -3), new Vector3d(3, -2, -3), new Vector3d(0, -2, 3)));
        var complete = new ConePlaneRayEventSource(new FixedSegment(new Vector3d(-2, -2, 1), new Vector3d(2, -2, 1)));
        var shortSegment = new ConePlaneRayEventSource(new FixedSegment(new Vector3d(-Fixed64.Half, -Fixed64.Two, Fixed64.One),
            new Vector3d(Fixed64.Half, -Fixed64.Two, Fixed64.One)));
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.BoundaryCapacity];
        int count = ConePlaneRayEvents.GetBoundaryEvents(events), admitted = 0;
        foreach (ConePlaneRayEvent item in events[..count])
        {
            if (item.Kind != ConePlaneRayEventKind.UpperRim) continue;
            if (ConePlaneRayTestQueries.Materialize(complete, frame, item, 1, out Vector3d p, out Vector3d q, out Fixed64 depth))
            {
                admitted++;
                Assert.Equal(p, q); Assert.Equal(Fixed64.Zero, depth);
                Assert.Equal(-Fixed64.Two, p.Y); Assert.Equal(Fixed64.One, p.Z);
                Assert.Equal(7439101574L, FixedMath.Abs(p.X).m_rawValue);
            }
            Assert.False(ConePlaneRayTestQueries.Materialize(shortSegment, frame, item, 1, out _, out _, out _));
        }
        Assert.Equal(2, admitted);
    }

    [Fact]
    public void StationaryOppositeExits_CompareTheirExactSharedPointWithoutExitReplay()
    {
        var frame = Frame(new FixedTriangle(new Vector3d(0, -3, -3), new Vector3d(0, -3, 3), new Vector3d(0, 3, 0)));
        var source = new ConePlaneRayEventSource(new FixedSegment(-Vector3d.Forward * Fixed64.Half, Vector3d.Forward * Fixed64.Half));
        var first = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeStationary, branch: -1);
        var second = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeStationary, branch: 1);
        Assert.Equal(0, ConePlaneRayEvents.CompareEventAnchors(source, first, source, second, frame, includeCoincidentProvenance: false));
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(source, first, source, second, frame) < 0);
        Assert.True(ConePlaneRayTestQueries.Materialize(source, frame, first, -1, out Vector3d p, out Vector3d q, out Fixed64 depth));
        Assert.Equal(Vector3d.Zero, p); Assert.Equal(Vector3d.Left, q); Assert.Equal(Fixed64.One, depth);
        Assert.True(ConePlaneRayTestQueries.Materialize(source, frame, second, 1, out p, out q, out depth));
        Assert.Equal(Vector3d.Zero, p); Assert.Equal(Vector3d.Right, q); Assert.Equal(Fixed64.One, depth);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    [InlineData(9)]
    [InlineData(10)]
    [InlineData(11)]
    [InlineData(12)]
    [InlineData(13)]
    public void StreamedEvents_PreserveTheScopedInventoryAndEachExactDirectionalCertificate(int fixture)
    {
        FixedTriangle plane = fixture switch
        {
            1 => new(new Vector3d(-4, 2, -4), new Vector3d(4, 2, -4), new Vector3d(0, 2, 4)),
            2 or 10 => new(new Vector3d(-4, -2, -4), new Vector3d(4, -2, -4), new Vector3d(0, -2, 4)),
            3 => new(new Vector3d(-4, 3, -4), new Vector3d(4, 3, -4), new Vector3d(0, 3, 4)),
            5 or 9 => new(new Vector3d(0, -4, -4), new Vector3d(0, 4, -4), new Vector3d(0, 0, 4)),
            6 => new(new Vector3d(-4, -2, -4), new Vector3d(4, 6, -4), new Vector3d(0, 2, 4)),
            7 => new(new Vector3d(-4, -1, -4), new Vector3d(4, 3, -4), new Vector3d(0, 1, 4)),
            12 => new(Vector3d.Zero, Vector3d.Right, Vector3d.Right * 2),
            13 => new(new Vector3d(-4, -3, -4), new Vector3d(4, 1, -4), new Vector3d(0, 3, 4)),
            _ => new(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4))
        };
        var frame = new ConePlaneRayFrame(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, fixture == 4 ? Fixed64.Zero : Fixed64.Two);
        ConePlaneRayEventSource source = fixture switch
        {
            8 => new(new FixedSegment(-Vector3d.Right * 2, Vector3d.Right * 2)),
            9 => new(new FixedSegment(-Vector3d.Forward * Fixed64.Half, Vector3d.Forward * Fixed64.Half)),
            10 => new(new FixedSegment(new Vector3d(-2, -2, 1), new Vector3d(2, -2, 1))),
            11 => new(new FixedSegment(Vector3d.Zero, Vector3d.Zero)),
            13 => new(new FixedSegment(new Vector3d(-2, -1, -2), new Vector3d(2, 3, 2))),
            _ => ConePlaneRayEventSource.Plane
        };
        var streamed = new List<ConePlaneRayEvent>();
        void Capture(in ConePlaneRayEventSource currentSource, in ConePlaneRayFrame currentFrame,
            ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        {
            Assert.Equal(source.Scope, currentSource.Scope);
            Assert.Equal(source.A, currentSource.A); Assert.Equal(source.B, currentSource.B);
            AssertStreamedCertificate(currentSource, currentFrame, descriptor, point, root, positive, negative);
            streamed.Add(descriptor);
        }
        bool found = ConePlaneRayEvents.VisitEvents(source, frame, Capture);
        Assert.Equal(streamed.Count != 0, found);
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        int count = source.Scope == ConePlaneRayEventScope.Plane
            ? ConePlaneRayEvents.GetIntrinsicEvents(frame, events) : ConePlaneRayEvents.GetBoundaryEvents(events);
        Assert.Equal(source.Scope == ConePlaneRayEventScope.Segment ? 14 : fixture is 1 or 4 ? 1 : fixture <= 3 ? 5 : 102, count);
        if (fixture == 13) Assert.True(!frame.Normal.X.IsZero && !frame.Normal.Z.IsZero);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        int admitted = 0;
        foreach (ConePlaneRayEvent descriptor in events[..count])
        {
            bool exists = ConePlaneRayEvents.TryEvaluateEvent(source, frame, descriptor, point, root, ref positive, ref negative);
            for (int orientation = -1; orientation <= 1; orientation += 2)
                Assert.Equal(exists, ConePlaneRayEvents.TryEvaluateEvent(source, frame, descriptor,
                    point, root, ref positive, ref negative, requestedOrientation: orientation));
            if (!exists) continue;
            Assert.True(admitted < streamed.Count);
            Assert.True(descriptor.Matches(streamed[admitted]));
            admitted++;
        }
        Assert.Equal(admitted, streamed.Count);
        if (fixture == 3 || fixture == 12) Assert.False(found);
        else Assert.True(found);
    }

    private static void AssertStreamedCertificate(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint streamedPoint, scoped ReadOnlySpan<ulong> streamedRoot,
        scoped in ConePlaneRaySelection streamedPositive, scoped in ConePlaneRaySelection streamedNegative)
    {
        // Full solid admission remains an independent check of certificates
        // whose constructing side polynomial already proves F(point)=0.
        Assert.True(ConePlaneRayPointExits.ContainsPoint(source, frame, streamedPoint, streamedRoot));
        Assert.Equal(0, ConePlaneRayEvents.CompareEventAnchors(streamedPoint, streamedRoot, source, descriptor, frame));
        // Replay each compact descriptor independently: the bulk construction
        // must preserve exact anchors and both exits, including rational points
        // whose selected ray exits use a different quadratic root.
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, frame, descriptor, point, root, ref positive, ref negative));
        Assert.Equal(0, point.CompareTo(streamedPoint, root, streamedRoot));
        AssertStreamedExit(frame, 1, positive, streamedPositive);
        AssertStreamedExit(frame, -1, negative, streamedNegative);
        // Once the direction is fixed, replay can omit the opposite general
        // exit solve without changing admission, provenance or materialization.
        for (int orientation = -1; orientation <= 1; orientation += 2)
        {
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, frame, descriptor, point, root,
                ref positive, ref negative, requestedOrientation: orientation));
            Assert.Equal(0, point.CompareTo(streamedPoint, root, streamedRoot));
            if (orientation > 0) AssertStreamedExit(frame, orientation, positive, streamedPositive);
            else AssertStreamedExit(frame, orientation, negative, streamedNegative);
        }
    }

    [Theory]
    [InlineData(-2)]
    [InlineData(2)]
    public void RequestedReplay_RejectsInvalidOrientations(int requestedOrientation)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
        {
            var frame = Frame(new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4)));
            Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
            Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
            var point = new ConePlaneRayPoint(words, signs);
            Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
            Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
            var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
            ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame, default, point, root,
                ref positive, ref negative, requestedOrientation: requestedOrientation);
        });
    }

    private static void AssertStreamedExit(in ConePlaneRayFrame frame, int orientation,
        scoped in ConePlaneRaySelection replayed, scoped in ConePlaneRaySelection streamed)
    {
        Assert.Equal(replayed.HasValue, streamed.HasValue);
        if (!replayed.HasValue) return;
        Assert.True(replayed.MaximumEvent.Matches(streamed.MaximumEvent));
        Assert.Equal(replayed.MaximumEvent.Orientation, streamed.MaximumEvent.Orientation);
        Assert.Equal(0, ContactQuadratic.CompareRatios(
            ContactQuadratic.At(replayed.Values, replayed.Signs, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(replayed.Values, replayed.Signs, 1, ConePlaneRaySelection.FieldWords), replayed.Root,
            ContactQuadratic.At(streamed.Values, streamed.Signs, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(streamed.Values, streamed.Signs, 1, ConePlaneRaySelection.FieldWords), streamed.Root));
        Assert.Equal(0, replayed.Point.CompareTo(streamed.Point, replayed.Root, streamed.Root));
        Assert.Equal(replayed.TryMaterialize(frame, orientation, out Vector3d p, out Vector3d q, out Fixed64 depth),
            streamed.TryMaterialize(frame, orientation, out Vector3d actualP, out Vector3d actualQ, out Fixed64 actualDepth));
        Assert.Equal(p, actualP); Assert.Equal(q, actualQ); Assert.Equal(depth, actualDepth);
    }

    private static ConePlaneRayFrame Frame(FixedTriangle triangle) => new(triangle, Vector3d.Zero, FixedQuaternion.Identity,
        Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
}

internal readonly struct ConePlaneRayTestEvent
{
    internal readonly ConePlaneRayEventSource Source;
    internal readonly ConePlaneRayEvent Event;
    internal ConePlaneRayTestEvent(ConePlaneRayEventSource source, ConePlaneRayEvent item) { Source = source; Event = item; }
}

// Test composition of the production plane/segment cohorts. It performs no
// geometry beyond the actual exact triangle membership and maximum APIs.
internal static class ConePlaneRayTestQueries
{
    internal const int Capacity = ConePlaneRayEvents.IntrinsicCapacity + 3 * ConePlaneRayEvents.BoundaryCapacity;
    internal static bool AccumulateTriangle(FixedTriangle triangle, in ConePlaneRayFrame frame,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative) =>
        CollectTriangle(triangle, frame, Span<ConePlaneRayTestEvent>.Empty, ref positive, ref negative) != 0;

    internal static int CollectTriangle(FixedTriangle triangle, in ConePlaneRayFrame frame, Span<ConePlaneRayTestEvent> admitted,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative)
    {
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var p = new ConePlaneRaySelection(pv, ps); var n = new ConePlaneRaySelection(nv, ns);
        scoped ConePlaneRaySelection winnerPositive = new(positive.Values, positive.Signs);
        scoped ConePlaneRaySelection winnerNegative = new(negative.Values, negative.Signs);
        if (positive.HasValue) winnerPositive.Restore(frame, positive.MaximumSource, positive.MaximumEvent);
        if (negative.HasValue) winnerNegative.Restore(frame, negative.MaximumSource, negative.MaximumEvent);
        int count = 0;
        for (int domain = -1; domain < 3; domain++)
        {
            ConePlaneRayEventSource source = domain < 0 ? ConePlaneRayEventSource.Plane : new ConePlaneRayEventSource(triangle.GetEdge(domain));
            int possible = domain < 0 ? ConePlaneRayEvents.GetIntrinsicEvents(frame, events) : ConePlaneRayEvents.GetBoundaryEvents(events);
            foreach (ConePlaneRayEvent item in events[..possible])
            {
                if (!ConePlaneRayEvents.TryEvaluateEvent(source, frame, item, point, root, ref p, ref n)
                    || !ConePlaneRayPointExits.ContainsTrianglePoint(triangle, frame, point, root)) continue;
                if (!admitted.IsEmpty) admitted[count] = new ConePlaneRayTestEvent(source, item);
                count++;
                winnerPositive.KeepEvaluated(p, frame); winnerNegative.KeepEvaluated(n, frame);
            }
        }
        if (winnerPositive.HasValue) positive.Restore(frame, winnerPositive.MaximumSource, winnerPositive.MaximumEvent);
        if (winnerNegative.HasValue) negative.Restore(frame, winnerNegative.MaximumSource, winnerNegative.MaximumEvent);
        return count;
    }

    internal static bool Materialize(ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEvent item,
        int orientation, out Vector3d lower, out Vector3d upper, out Fixed64 depth) =>
        Materialize(source, frame, item, orientation, out lower, out upper, out depth, out _);

    internal static bool Materialize(ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEvent item,
        int orientation, out Vector3d lower, out Vector3d upper, out Fixed64 depth, out bool admitted)
    {
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var p = new ConePlaneRaySelection(pv, ps); var n = new ConePlaneRaySelection(nv, ns);
        bool found = ConePlaneRayEvents.TryEvaluateEvent(source, frame, item, point, root, ref p, ref n);
        ConePlaneRaySelection selected = orientation > 0 ? p : n;
        admitted = found && selected.HasValue;
        return selected.TryMaterialize(frame, orientation, out lower, out upper, out depth);
    }
}
