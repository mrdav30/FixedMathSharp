using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;
using static FixedMathSharp.Tests.ConePlaneRayTestQueries;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedTriangleFiniteConePlaneSectionEventTests
{
    [Fact]
    public void PlaneEvents_GeometricComparison_DeduplicatesCoincidentProvenance()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var segment = new ConePlaneRayEventSource(new FixedSegment(Vector3d.Left, Vector3d.Zero));
        var endpoint = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 1);
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        var axis = new ConePlaneRayEvent(ConePlaneRayEventKind.Axis);
        Assert.Equal(0, ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, axis, segment, endpoint, frame,
            includeCoincidentProvenance: false));
        Assert.NotEqual(0, ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, axis, segment, endpoint, frame));
        var circle = new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 0, 0, 0);
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, axis, ConePlaneRayEventSource.Plane, circle, frame,
            includeCoincidentProvenance: false) < 0);
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, circle, ConePlaneRayEventSource.Plane, axis, frame,
            includeCoincidentProvenance: false) > 0);
        Assert.Equal(0, ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, axis, ConePlaneRayEventSource.Plane, axis, frame,
            includeCoincidentProvenance: false));
    }

    [Fact]
    public void PlaneEvents_RegionAssignment_UsesExactClosedTriangleAdmission()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        var axis = new ConePlaneRayEvent(ConePlaneRayEventKind.Axis);
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<ulong> root = stackalloc ulong[ConePlaneRayCharts.Words];
        var point = new ConePlaneRayPoint(values, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame,
            axis, point, root, ref positive, ref negative));
        Assert.True(ConePlaneRayPointExits.ContainsTrianglePoint(triangle, frame, point, root));
        var touching = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Forward);
        Assert.True(ConePlaneRayPointExits.ContainsTrianglePoint(touching, frame, point, root));
        var outside = new FixedTriangle(new Vector3d(5, 0, 0), new Vector3d(6, 0, 0), new Vector3d(5, 0, 1));
        Assert.False(ConePlaneRayPointExits.ContainsTrianglePoint(outside, frame, point, root));
        var degenerate = new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Assert.False(ConePlaneRayPointExits.ContainsTrianglePoint(degenerate, frame, point, root));
    }

    [Fact]
    public void PlaneEvents_GeometricComparison_PreservesDistinctSubrawAnchors()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.FromRaw(1));
        var right = new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 0, 0, 0);
        var left = new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 1, 0, 0);
        Assert.True(Materialize(ConePlaneRayEventSource.Plane, frame, left, 1,
            out Vector3d leftAnchor, out _, out _, out bool admitted));
        Assert.True(admitted);
        Assert.True(Materialize(ConePlaneRayEventSource.Plane, frame, right, 1,
            out Vector3d rightAnchor, out _, out _, out admitted));
        Assert.True(admitted);
        Assert.Equal(Vector3d.Zero, leftAnchor);
        Assert.Equal(leftAnchor, rightAnchor);
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, left, ConePlaneRayEventSource.Plane, right, frame,
            includeCoincidentProvenance: false) < 0);
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, right, ConePlaneRayEventSource.Plane, left, frame,
            includeCoincidentProvenance: false) > 0);
    }

    [Fact]
    public void PlaneEvents_Reconstruction_SeparatesAdmissionFromOutputRange()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        var axis = new ConePlaneRayEvent(ConePlaneRayEventKind.Axis);
        Assert.True(Materialize(ConePlaneRayEventSource.Plane, frame, axis, 1,
            out Vector3d lower, out Vector3d cone, out Fixed64 depth, out bool admitted));
        Assert.True(admitted);
        Assert.Equal(Vector3d.Zero, lower);
        Assert.Equal(new Vector3d(0, 2, 0), cone);
        Assert.Equal(Fixed64.Two, depth);
        Assert.False(Materialize(ConePlaneRayEventSource.Plane, frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.Apex), -1, out _, out _, out _, out admitted));
        Assert.False(admitted);
        var center = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        frame = new ConePlaneRayFrame(triangle, center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Assert.False(Materialize(ConePlaneRayEventSource.Plane, frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 0, 0, 0), 1,
            out _, out _, out _, out admitted));
        Assert.True(admitted);
        Fixed64 left = -Fixed64.MaxValue;
        triangle = new FixedTriangle(new Vector3d(left, -Fixed64.Two, Fixed64.Zero),
            new Vector3d(left, -Fixed64.Two, Fixed64.One), new Vector3d(left, -Fixed64.One, Fixed64.Zero));
        frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.MaxValue);
        Assert.False(Materialize(new ConePlaneRayEventSource(triangle.GetEdge(0)), frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0), 1,
            out _, out _, out _, out admitted));
        Assert.True(admitted);
    }

    [Fact]
    public void PlaneEvents_LargeCoprimeAuthoredNormal_PreservesFiniteRayCertificates()
    {
        long m = long.MaxValue;
        var b = new Vector3d(Fixed64.FromRaw(m), Fixed64.FromRaw(m - 1), Fixed64.FromRaw(1));
        var c = new Vector3d(Fixed64.FromRaw(m - 2), Fixed64.FromRaw(3), Fixed64.FromRaw(m - 4));
        var triangle = new FixedTriangle(Vector3d.Zero, b, c);
        BigInteger nx = (BigInteger)(m - 1) * (m - 4) - 3;
        BigInteger ny = (BigInteger)(m - 2) - (BigInteger)m * (m - 4);
        BigInteger nz = (BigInteger)m * 3 - (BigInteger)(m - 1) * (m - 2);
        Assert.Equal(BigInteger.One, BigInteger.GreatestCommonDivisor(nx, BigInteger.GreatestCommonDivisor(ny, nz)));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(AccumulateTriangle(triangle, frame, ref positive, ref negative));
        for (int orientation = -1; orientation <= 1; orientation += 2)
        {
            ConePlaneRaySelection selected = orientation > 0 ? positive : negative;
            Assert.True(selected.TryMaterialize(frame, orientation, out Vector3d lower, out Vector3d upper, out Fixed64 depth));
            // The admitted origin contains a radius-1/2 ball in the cone;
            // its diameter is max(2R,sqrt(H*H+R*R))=sqrt(20), below 5.
            Assert.InRange(depth.m_rawValue, Fixed64.Half.m_rawValue, ((Fixed64)5).m_rawValue);
            Assert.True(BigInteger.Abs(nx * lower.X.m_rawValue + ny * lower.Y.m_rawValue + nz * lower.Z.m_rawValue)
                <= BigInteger.Abs(nx) + BigInteger.Abs(ny) + BigInteger.Abs(nz));
            BigInteger dx = upper.X.m_rawValue - lower.X.m_rawValue;
            BigInteger dy = upper.Y.m_rawValue - lower.Y.m_rawValue;
            BigInteger dz = upper.Z.m_rawValue - lower.Z.m_rawValue;
            Assert.True(BigInteger.Abs(dx * ny - dy * nx) <= BigInteger.Abs(nx) + BigInteger.Abs(ny));
            Assert.True(BigInteger.Abs(dx * nz - dz * nx) <= BigInteger.Abs(nx) + BigInteger.Abs(nz));
            Assert.True(orientation * dx >= -1);
            BigInteger u = Fixed64.One.m_rawValue;
            BigInteger x = upper.X.m_rawValue, y = 2 * u - upper.Y.m_rawValue, z = upper.Z.m_rawValue;
            Assert.True(BigInteger.Abs(upper.Y.m_rawValue + 2 * u) <= 1
                || BigInteger.Abs(16 * u * u * (x * x + z * z) - 4 * u * u * y * y) <= 128 * u * u * u);
        }
    }

    [Fact]
    public void PlaneEvents_Reconstruction_ReportsAnchorAndDepthRangeFailures()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var centered = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Assert.False(Materialize(ConePlaneRayEventSource.Plane, centered,
            new ConePlaneRayEvent(ConePlaneRayEventKind.Apex), -1, out _, out _, out _));
        var center = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        var frame = new ConePlaneRayFrame(triangle, center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Assert.False(Materialize(ConePlaneRayEventSource.Plane, frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 0, 0, 0), 1, out _, out _, out _));
        Fixed64 left = -Fixed64.MaxValue;
        triangle = new FixedTriangle(new Vector3d(left, -Fixed64.Two, Fixed64.Zero),
            new Vector3d(left, -Fixed64.Two, Fixed64.One), new Vector3d(left, -Fixed64.One, Fixed64.Zero));
        frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.MaxValue);
        Assert.False(Materialize(new ConePlaneRayEventSource(triangle.GetEdge(0)), frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0), 1, out _, out _, out _));
    }

    [Fact]
    public void PlaneEvents_CoincidentAxisEndpoints_CompareOnlyEndpointProvenance()
    {
        var triangle = new FixedTriangle(new Vector3d(0, 2, 0), new Vector3d(0, 0, 1), new Vector3d(0, 0, -1));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane,
            new ConePlaneRayEvent(ConePlaneRayEventKind.Axis, endpoint: 0), new ConePlaneRayEventSource(triangle.GetEdge(0)),
            new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0), frame) < 0);
        // A zero-depth apex projection is represented by its axis family.
        Assert.False(Materialize(ConePlaneRayEventSource.Plane, frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.Apex), 1, out _, out _, out _));
    }

    [Fact]
    public void PlaneEvents_PointAboveApex_FailsFiniteAxialAdmission()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 3, -4), new Vector3d(4, 3, -4), new Vector3d(0, 3, 4));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs);
        point.Set(frame.Transform(new Vector3d(0, 3, 0)));
        Assert.False(ConePlaneRayPointExits.ContainsPoint(ConePlaneRayEventSource.Plane, frame, point, ReadOnlySpan<ulong>.Empty));
        point.Set(frame.Transform(Vector3d.Zero));
        Assert.False(ConePlaneRayPointExits.ContainsPoint(ConePlaneRayEventSource.Plane, frame, point, ReadOnlySpan<ulong>.Empty));
    }

    [Theory]
    [InlineData(true, 1, true)]
    [InlineData(true, 2, false)]
    [InlineData(true, 3, false)]
    [InlineData(false, -1, true)]
    [InlineData(false, -2, true)]
    [InlineData(false, -3, false)]
    public void PlaneEvents_AnchorRange_UsesRoundedHalfThresholds(bool maximum, int quarterRaws, bool represented)
    {
        Fixed64 center = Fixed64.FromRaw(maximum ? long.MaxValue : long.MinValue);
        var triangle = new FixedTriangle(new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(0, 0, 1));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(center, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs);
        point.X.Set(Signed576.ExtendValue(WideArithmetic.MultiplySigned192(
            frame.Finite.ShapeFrame.Denominator, Signed192.Signed(quarterRaws))));
        point.Y.Set(Signed576.ExtendValue(WideArithmetic.AddSigned320(frame.Finite.ShapeFrame.Cap, frame.Finite.ShapeFrame.Cap)));
        point.Z.Clear();
        point.Denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(2))));
        Assert.Equal(represented, ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point,
            ReadOnlySpan<ulong>.Empty, out Vector3d anchor));
        if (represented) Assert.Equal(center, anchor.X);
        Span<ulong> fields = stackalloc ulong[4 * ConePlaneRayCharts.Words];
        Span<int> fieldSigns = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, fieldSigns, 0, ConePlaneRayCharts.Words);
        ContactQuadratic d = ContactQuadratic.At(fields, fieldSigns, 1, ConePlaneRayCharts.Words);
        n.Clear(); d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
        // An exact zero exit is the original point, including its rounded
        // range boundary. Both output modes must preserve the half-tie policy.
        Assert.Equal(represented, ConePlaneRayPointMaterialization.IsExitWorldPointRepresentable(frame, point,
            ReadOnlySpan<ulong>.Empty, n, d, 1));
        Assert.Equal(represented, ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, point,
            ReadOnlySpan<ulong>.Empty, n, d, 1, out Vector3d exit));
        if (represented) Assert.Equal(anchor, exit);
    }

    [Fact]
    public void PlaneEvents_IntegerPlaneCorpus_MatchesIndependentSupportAndPairedRayBounds()
    {
        for (int b = -3; b <= 3; b++)
        for (int c = -3; c <= 3; c++)
        for (int k = -3; k <= 3; k++)
        {
            // x+b*y+c*z=k. This triangle contains the complete Y/Z projection
            // of the cone, so support interval membership is an independent
            // exact oracle for whether its finite plane section exists.
            var triangle = new FixedTriangle(new Vector3d(k + 8 * b + 8 * c, -8, -8),
                new Vector3d(k - 8 * b + 8 * c, 8, -8), new Vector3d(k - 8 * c, 0, 8));
            int apex = 2 * b, shifted = k + 2 * b;
            bool expected = k == apex || k > apex && (shifted <= 0 || shifted * shifted <= 4 * (1 + c * c))
                || k < apex && (shifted >= 0 || shifted * shifted <= 4 * (1 + c * c));
            CheckPlaneCorpusCase(triangle, b, c, k, expected);
        }
    }

    private static void CheckPlaneCorpusCase(FixedTriangle triangle, int b, int c, int k, bool expected)
    {
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        bool intersects = AccumulateTriangle(triangle, frame, ref positive, ref negative);
        Assert.True(expected == intersects, $"Plane ({b}, {c}, {k}) support admission mismatch.");
        Assert.Equal(expected, positive.HasValue); Assert.Equal(expected, negative.HasValue);
        if (!expected) return;
        for (int orientation = -1; orientation <= 1; orientation += 2)
        {
            ConePlaneRaySelection selected = orientation > 0 ? positive : negative;
            Assert.True(selected.TryMaterialize(frame, orientation, out Vector3d lower, out Vector3d upper, out Fixed64 depth));
            Assert.True(depth >= Fixed64.Zero);
            long unit = Fixed64.One.m_rawValue;
            Assert.InRange(Math.Abs(lower.X.m_rawValue + b * lower.Y.m_rawValue + c * lower.Z.m_rawValue - k * unit),
                0L, 1L + Math.Abs(b) + Math.Abs(c));
            long dx = upper.X.m_rawValue - lower.X.m_rawValue;
            long dy = upper.Y.m_rawValue - lower.Y.m_rawValue;
            long dz = upper.Z.m_rawValue - lower.Z.m_rawValue;
            Assert.InRange(Math.Abs(dy - b * dx), 0L, 1L + Math.Abs(b));
            Assert.InRange(Math.Abs(dz - c * dx), 0L, 1L + Math.Abs(c));
            Assert.True(orientation * dx >= -1);
            Assert.InRange(upper.Y.m_rawValue, -2 * unit - 1, 2 * unit + 1);
            BigInteger x = upper.X.m_rawValue, y = 2 * unit - upper.Y.m_rawValue, z = upper.Z.m_rawValue;
            BigInteger u = unit;
            BigInteger side = 16 * u * u * (x * x + z * z) - 4 * u * u * y * y;
            // Independent boundary checks allow only the propagated final
            // half-raw coordinate rounding, never a geometry-admission slack.
            Assert.True(Math.Abs(upper.Y.m_rawValue + 2 * unit) <= 1 || BigInteger.Abs(side) <= 128 * u * u * u);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void PlaneEvents_TangentGeneratorExcludingBothVertices_RetainsInteriorRimSwitch(bool reverse, bool alignedEdge)
    {
        // The plane 2x+y=2 touches the cone in one complete generator.
        // Its triangle-clipped interval excludes apex and base. The inward
        // maximum is at y=-2/5, where side and cap exits meet: 8*sqrt(5)/5.
        var triangle = new FixedTriangle(new Vector3d(1, 0, alignedEdge ? 0 : -1), new Vector3d(1, 0, 1),
            new Vector3d((Fixed64)3 / 2, -Fixed64.One, Fixed64.Zero));
        if (reverse) triangle = new FixedTriangle(triangle.A, triangle.C, triangle.B);
        var result = Evaluate(triangle, Fixed64.Two);
        Assert.True(result.Intersects);
        Assert.Equal(Fixed64.Zero, result.Positive);
        Assert.Equal(Fixed64.FromRaw(15366142136L), result.Negative);
    }

    [Fact]
    public void PlaneEvents_LowerBaseLineInterior_RetainsStationarySideExit()
    {
        // 3x+y=-5 cuts the lower base at x=-1. The maximum positive ray
        // starts at (-1,-2,0), strictly inside both the triangle and base disk,
        // and ends on the side at (11/7,-8/7,0): distance 6*sqrt(10)/7.
        var triangle = new FixedTriangle(new Vector3d(0, -5, -3), new Vector3d(0, -5, 3),
            new Vector3d(-2, 1, 0));
        var result = Evaluate(triangle, Fixed64.Two);
        Assert.True(result.Intersects);
        Assert.Equal(Fixed64.FromRaw(11641610684L), result.Positive);
    }

    [Fact]
    public void PlaneEvents_CoincidentBasePlane_RetainsDiskSupportContinuum()
    {
        var triangle = new FixedTriangle(new Vector3d(-5, -2, -5), new Vector3d(5, -2, -5),
            new Vector3d(0, -2, 5));
        var result = Evaluate(triangle, Fixed64.Two);
        Assert.True(result.Intersects);
        Assert.Equal((Fixed64)4, result.Positive);
        Assert.Equal(Fixed64.Zero, result.Negative);
    }

    [Fact]
    public void PlaneEvents_ClippedAxialSection_RetainsFiniteBaseDiameter()
    {
        var triangle = new FixedTriangle(new Vector3d(0, -2, -3), new Vector3d(0, -2, 3),
            new Vector3d(0, -1, 0));
        var result = Evaluate(triangle, Fixed64.Two);
        Assert.True(result.Intersects);
        Assert.Equal(Fixed64.Two, result.Positive);
        Assert.Equal(Fixed64.Two, result.Negative);
    }

    [Fact]
    public void PlaneEvents_ZeroRadiusClippedAxisInterval_ExcludesBothConeVertices()
    {
        var triangle = new FixedTriangle(new Vector3d(0, -1, -1), new Vector3d(0, -1, 1),
            new Vector3d(0, 1, 0));
        var result = Evaluate(triangle, Fixed64.Zero);
        Assert.True(result.Intersects);
        Assert.Equal(Fixed64.Zero, result.Positive);
        Assert.Equal(Fixed64.Zero, result.Negative);
    }

    [Fact]
    public void PlaneEvents_RegularAndDegenerateStrata_AllocateNoWarmedBytes()
    {
        var triangle = new FixedTriangle(new Vector3d(0, -3, -3), new Vector3d(0, -3, 3),
            new Vector3d(-3, 0, 0));
        long bytes = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            var result = Evaluate(triangle, Fixed64.Two);
            Assert.True(result.Intersects);
        });
        Assert.Equal(0, bytes);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void PlaneEvents_EqualDepthWinner_UsesExactAnchorAcrossOrderAndTriangulation(bool alternate, bool reverse)
    {
        Vector3d a = new(-2, 0, -2), b = new(2, 0, -2), c = new(2, 0, 2), d = new(-2, 0, 2);
        FixedTriangle first = alternate ? new FixedTriangle(a, b, d) : new FixedTriangle(a, b, c);
        FixedTriangle second = alternate ? new FixedTriangle(b, c, d) : new FixedTriangle(a, c, d);
        if (reverse) (first, second) = (second, first);
        var frame = new ConePlaneRayFrame(first, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(AccumulateTriangle(first, frame, ref positive, ref negative));
        Assert.True(AccumulateTriangle(second, frame, ref positive, ref negative));
        Assert.True(negative.TryMaterialize(frame, -1, out Vector3d lower, out Vector3d upper, out Fixed64 depth));
        Assert.Equal(new Vector3d(-1, 0, 0), lower);
        Assert.Equal(new Vector3d(-1, -2, 0), upper);
        Assert.Equal(Fixed64.Two, depth);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlaneEvents_ScopedPool_ReconstructsRegularEventsAndClippedGeneratorEndpoints(bool tangent)
    {
        var triangle = new FixedTriangle(new Vector3d(1, 0, -1), new Vector3d(1, 0, 1),
            new Vector3d((Fixed64)3 / 2, -Fixed64.One, Fixed64.Zero));
        if (!tangent) triangle = new FixedTriangle(Vector3d.Zero, new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        Span<ConePlaneRayTestEvent> events = stackalloc ConePlaneRayTestEvent[Capacity];
        int count = CollectTriangle(triangle, frame, events, ref positive, ref negative);
        Assert.InRange(count, 1, Capacity);
        bool lowerEndpoint = false, upperEndpoint = false;
        for (int i = 0; i < count; i++)
        {
            ConePlaneRayTestEvent item = events[i];
            bool reconstructed = Materialize(item.Source, frame, item.Event, 1, out Vector3d anchor, out _, out _);
            if (!reconstructed)
                reconstructed = Materialize(item.Source, frame, item.Event, -1, out anchor, out _, out _);
            Assert.True(reconstructed);
            lowerEndpoint |= anchor == new Vector3d(1, 0, 0);
            upperEndpoint |= anchor == new Vector3d((Fixed64)3 / 2, -Fixed64.One, Fixed64.Zero);
        }
        if (tangent) { Assert.True(lowerEndpoint); Assert.True(upperEndpoint); }
    }

    [Fact]
    public void PlaneEvents_ReusedStorage_BeginsANewTriangleWhileSelectionsRetainTheRegion()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var outside = new FixedTriangle(new Vector3d(100, 0, -4), new Vector3d(108, 0, -4), new Vector3d(104, 0, 4));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Span<ConePlaneRayTestEvent> events = stackalloc ConePlaneRayTestEvent[Capacity];
        Assert.True(CollectTriangle(triangle, frame, events, ref positive, ref negative) > 0);
        Assert.Equal(0, CollectTriangle(outside, frame, events, ref positive, ref negative));
        Assert.Equal(Fixed64.Two, positive.GetRoundedMaximumDepth());
        Assert.Equal(Fixed64.Two, negative.GetRoundedMaximumDepth());
    }

    [Fact]
    public void PlaneEvents_DegeneratePlane_AdmitsNoFace()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Right * 2);
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Assert.False(AccumulateTriangle(triangle, frame, ref positive, ref negative));
        Assert.False(positive.HasValue); Assert.False(negative.HasValue);
    }

    [Fact]
    public void PlaneEvents_ZNormalAndNoncoplanarSegmentEndpoints_PreserveClosedPlaneIdentity()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, -4, 0), new Vector3d(4, -4, 0), new Vector3d(0, 4, 0));
        var result = Evaluate(triangle, Fixed64.Two);
        Assert.Equal(Fixed64.Two, result.Positive); Assert.Equal(Fixed64.Two, result.Negative);
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Assert.False(frame.IntersectsSegment(new FixedSegment(Vector3d.Zero, Vector3d.Forward)));
        Assert.False(frame.IntersectsSegment(new FixedSegment(Vector3d.Forward, Vector3d.Zero)));
    }

    [Fact]
    public void PlaneEvents_DescriptorValidation_RejectsMissingEventsAndInvalidOrientations()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        var axis = new ConePlaneRayEvent(ConePlaneRayEventKind.Axis);
        var missing = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeSide, 99);
        Assert.Throws<ArgumentOutOfRangeException>(() => Materialize(ConePlaneRayEventSource.Plane, frame, axis,
            0, out _, out _, out _));
        Assert.Throws<ArgumentException>(() => { Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[1];
            ConePlaneRayEvents.GetIntrinsicEvents(frame, events); });
        Assert.Throws<ArgumentException>(() => { Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[1];
            ConePlaneRayEvents.GetBoundaryEvents(events); });
        Assert.False(Materialize(ConePlaneRayEventSource.Plane, frame, missing, 1, out _, out _, out _));
        Assert.False(Materialize(ConePlaneRayEventSource.Plane, frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.Generator, 8, 0, 0, 0), 1, out _, out _, out _));
        Assert.False(Materialize(ConePlaneRayEventSource.Plane, frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.Axis, endpoint: 0), 1, out _, out _, out _));
        Assert.Equal(0, ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, axis, ConePlaneRayEventSource.Plane, axis, frame));
        Assert.Throws<InvalidOperationException>(() => ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, missing, ConePlaneRayEventSource.Plane, axis, frame));
        Assert.Throws<InvalidOperationException>(() => ConePlaneRayEvents.CompareEventAnchors(ConePlaneRayEventSource.Plane, axis, ConePlaneRayEventSource.Plane, missing, frame));
    }

    [Fact]
    public void PlaneEvents_CoincidentOppositeRimBranches_UseProvenanceOnlyAfterExactAnchorEquality()
    {
        Fixed64 x = (Fixed64)5 / 4;
        var triangle = new FixedTriangle(new Vector3d(x, (Fixed64)(-3), -x), new Vector3d(x, Fixed64.Zero, -x), new Vector3d(0, -3, 0));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        var first = new ConePlaneRayEvent(ConePlaneRayEventKind.UpperRim, 0, 2, -1);
        var second = new ConePlaneRayEvent(ConePlaneRayEventKind.UpperRim, 0, 2, 1);
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(new ConePlaneRayEventSource(triangle.GetEdge(0)), first, new ConePlaneRayEventSource(triangle.GetEdge(0)), second, frame) < 0);
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(new ConePlaneRayEventSource(triangle.GetEdge(0)), second, new ConePlaneRayEventSource(triangle.GetEdge(0)), first, frame) > 0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlaneEvents_BorrowedStorage_RequiresBothValueAndSignCapacity(bool signsTooSmall)
    {
        Assert.Throws<ArgumentException>(() =>
        {
            Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords - (signsTooSmall ? 0 : 1)];
            Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount - (signsTooSmall ? 1 : 0)];
            _ = new ConePlaneRayPoint(values, signs);
        });
        Assert.Throws<ArgumentException>(() =>
        {
            Span<ulong> values = stackalloc ulong[ConePlaneRaySelection.StorageWords - (signsTooSmall ? 0 : 1)];
            Span<int> signs = stackalloc int[ConePlaneRaySelection.SignCount - (signsTooSmall ? 1 : 0)];
            _ = new ConePlaneRaySelection(values, signs);
        });
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void PlaneEvents_Selection_RejectsInvalidDepthSigns(int mode) =>
        Assert.Throws<ArgumentException>(() => ExerciseSelectionGuard(mode));

    [Fact]
    public void PlaneEvents_Selection_DistinguishesNoValueFromUnrepresentableDepth()
    {
        Assert.Throws<InvalidOperationException>(() => ExerciseSelectionGuard(2));
        Assert.Throws<OverflowException>(() => ExerciseSelectionGuard(3));
    }

    private static void ExerciseSelectionGuard(int mode)
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        Fixed64 radius = mode == 3 ? Fixed64.MaxValue : Fixed64.Two;
        if (mode == 3)
            triangle = new FixedTriangle(new Vector3d(-radius, (Fixed64)(-4), (Fixed64)(-2)), new Vector3d(-radius, (Fixed64)(-4), Fixed64.Two),
                new Vector3d(-radius, Fixed64.Zero, Fixed64.Zero));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, radius);
        Span<ulong> values = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRaySelection.SignCount];
        var selection = new ConePlaneRaySelection(values, signs);
        Assert.False(selection.TryGetRoundedMaximumDepth(out _));
        if (mode == 2) { selection.GetRoundedMaximumDepth(); return; }
        Span<ulong> fields = stackalloc ulong[4 * ConePlaneRayCharts.Words];
        Span<int> fieldSigns = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, fieldSigns, 0, ConePlaneRayCharts.Words);
        ContactQuadratic d = ContactQuadratic.At(fields, fieldSigns, 1, ConePlaneRayCharts.Words);
        n.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(mode == 0 ? -1 : mode == 3 ? long.MaxValue : 0))));
        d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(mode == 1 ? 0 : 1))));
        // At the base's -X rim, the +X exit crosses diameter 2*MaxValue.
        // The frame uses doubled raw coordinates, hence numerator 4*MaxRaw.
        if (mode == 3) { n.Add(n); n.Add(n); }
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns); point.Set(frame.Transform(triangle.A));
        selection.Keep(n, d, ReadOnlySpan<ulong>.Empty, frame, point);
        Assert.False(selection.TryGetRoundedMaximumDepth(out _));
        selection.GetRoundedMaximumDepth();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void PlaneEvents_PointAdmission_RejectsMissingHomogeneousOrPlaneIdentity(int mode)
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var degenerate = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Right * 2);
        var frame = new ConePlaneRayFrame(mode == 1 ? degenerate : triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs); point.Set(frame.Transform(Vector3d.Zero));
        if (mode == 0) point.Denominator.Clear();
        Assert.False(mode == 2
            ? ConePlaneRayPointExits.ContainsTrianglePoint(degenerate, frame, point, ReadOnlySpan<ulong>.Empty)
            : ConePlaneRayPointExits.ContainsPoint(ConePlaneRayEventSource.Plane, frame, point, ReadOnlySpan<ulong>.Empty));
        if (mode == 0) Assert.False(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point,
            ReadOnlySpan<ulong>.Empty, out _));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void PlaneEvents_ExitCertificates_RejectNegativeDepthZeroDenominatorAndOppositeNappe(int mode)
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs); point.Set(frame.Transform(Vector3d.Right));
        Span<ulong> fields = stackalloc ulong[4 * ConePlaneRayCharts.Words];
        Span<int> fieldSigns = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, fieldSigns, 0, ConePlaneRayCharts.Words);
        ContactQuadratic d = ContactQuadratic.At(fields, fieldSigns, 1, ConePlaneRayCharts.Words);
        n.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(mode == 0 ? -1 : mode == 3 ? 0 : 8 * Fixed64.One.m_rawValue))));
        d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(mode == 1 || mode == 3 ? 0 : 1))));
        Assert.False(ConePlaneRayPointExits.AccumulateStationarySideExit(ConePlaneRayEventSource.Plane, frame, point,
            ReadOnlySpan<ulong>.Empty, n, d, 1, ref positive, ref negative));
        if (mode < 2 || mode == 3)
            Assert.False(ConePlaneRayPointExits.AccumulateCertifiedExit(ConePlaneRayEventSource.Plane, frame, point,
                ReadOnlySpan<ulong>.Empty, n, d, 1, ref positive, ref negative));
        if (mode == 2) point.Denominator.Clear();
        Assert.False(ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, point, ReadOnlySpan<ulong>.Empty,
            n, d, 1, out _));
        Assert.False(ConePlaneRayPointMaterialization.IsExitWorldPointRepresentable(frame, point, ReadOnlySpan<ulong>.Empty,
            n, d, 1));
    }

    private static (bool Intersects, Fixed64 Positive, Fixed64 Negative) Evaluate(FixedTriangle triangle, Fixed64 radius)
    {
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, radius);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        bool intersects = AccumulateTriangle(triangle, frame,
            ref positive, ref negative);
        return (intersects, positive.GetRoundedMaximumDepth(), negative.GetRoundedMaximumDepth());
    }
}
