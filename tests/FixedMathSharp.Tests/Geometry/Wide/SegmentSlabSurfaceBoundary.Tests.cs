using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class SegmentSlabSurfaceBoundaryTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void NegativeStadiumEndpoint_ShouldRetainItsAuthoredFiniteArc(bool reverse, bool oddCore)
    {
        Fixed64 length = oddCore ? Fixed64.FromRaw(3) : Fixed64.Two;
        Fixed64 radius = oddCore ? Fixed64.FromRaw(2) : Fixed64.One;
        Fixed64 z = oddCore ? Fixed64.FromRaw(-3) : -Fixed64.One - Fixed64.Half;
        Vector3d a = new(Fixed64.Zero, -Fixed64.Half, z), b = new(Fixed64.Zero, Fixed64.Half, z);
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1) };
        Assert.True(Contact(new(a, b), length, radius, Fixed64.One, Array.Empty<Vector2d>(), fan, true, false, out var contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab));
        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.Equal(z, source.Z); Assert.Equal(Fixed64.Zero, source.Y);
        Assert.Equal(oddCore ? Fixed64.FromRaw(-4) : -Fixed64.Two, slab.Z);
        Assert.Equal(Fixed64.Zero, slab.Y);
        Assert.Equal(oddCore ? Fixed64.Zero : Fixed64.Half, contact.Depth);
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void AdmittedVerticalFeatureOutsideBothCaps_ShouldRejectAnImpossibleSideWitness(int side, bool zeroRadius)
    {
        var segment = new FixedSegment(new Vector3d(0, 2 * side, 0), new Vector3d(0, 3 * side, 0));
        WideAxis3[] fan = { Axis(0, 1, 0), Axis(0, -1, 0) };
        Assert.False(Contact(segment, Fixed64.Zero, zeroRadius ? Fixed64.Zero : Fixed64.One,
            Fixed64.One, Array.Empty<Vector2d>(), fan, true, true, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapEndpointOnlyIntersection_ShouldKeepTheClosedWitnessAndRejectStrictInterior(bool reverse)
    {
        Vector3d a = new(0, 1, 0), b = new(0, 2, 0);
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(new(a, b), Fixed64.Zero, Fixed64.One, Fixed64.One,
            Array.Empty<Vector2d>(), fan, false, false, out var contact));
        Assert.Equal(Fixed64.Zero, contact.Depth); Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.False(Contact(new(a, b), Fixed64.Zero, Fixed64.One, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClockwisePrism_ShouldRetainTheSameFiniteCornerCone(bool reverse)
    {
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        if (reverse) Array.Reverse(offsets);
        Vector3d point = new(Fixed64.Half, Fixed64.Half, Fixed64.Half);
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0), Axis(1, 0, -2), Axis(-2, 0, 1) };
        Assert.True(Contact(new(point, point), Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            offsets, fan, false, false, out var contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab));
        Assert.Equal(Vector3d.One, slab);
        Fixed64 depth = FixedMath.Sqrt(Fixed64.One * 3 / 4);
        Assert.InRange(contact.Depth.m_rawValue, depth.m_rawValue - 1, depth.m_rawValue + 1);
        Assert.Equal(contact.Normal.X, contact.Normal.Y); Assert.Equal(contact.Normal.X, contact.Normal.Z);
    }

    [Theory]
    [InlineData(-1, -1, false)]
    [InlineData(-1, 1, false)]
    [InlineData(1, -1, false)]
    [InlineData(1, 1, false)]
    [InlineData(-1, -1, true)]
    [InlineData(-1, 1, true)]
    [InlineData(1, -1, true)]
    [InlineData(1, 1, true)]
    public void ObliqueTangentRim_ShouldRetainAnExactZeroDepthAndClippedNormal(int cap, int region, bool reverse)
    {
        Vector3d point = new((Fixed64)3, (Fixed64)cap, (Fixed64)(5 * region));
        Vector3d edge = new((Fixed64)4, (Fixed64)(-8 * cap), (Fixed64)(-region));
        Vector3d a = point - edge / 16, b = point + edge / 16;
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -6 * cap, 0), Axis(-1, cap, 0) };
        Assert.True(Contact(new(a, b), Fixed64.Two, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out var contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab));
        Assert.Equal(point, source); Assert.Equal(point, slab);
        Assert.True(contact.Normal.X < Fixed64.Zero);
        Assert.True(contact.Normal.Y * cap < Fixed64.Zero && contact.Normal.Z * region < Fixed64.Zero);
        Assert.InRange(Vector3d.Dot(contact.Normal, edge).m_rawValue, -8, 8);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullRangeFeaturePenetration_ShouldClampOnlyDepthAndRetainFiniteOppositeAnchors(bool prism)
    {
        Fixed64 extent = Fixed64.MaxValue;
        Vector3d point = new(-extent, Fixed64.Zero, Fixed64.Zero);
        Vector2d[] offsets = prism ? new[] { new Vector2d(-extent, -extent), new Vector2d(extent, -extent),
            new Vector2d(extent, extent), new Vector2d(-extent, extent) } : Array.Empty<Vector2d>();
        WideAxis3[] fan = { Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(new(point, point), Fixed64.Zero, extent, extent, offsets, fan, false, true, out var contact));
        Assert.True(contact.DepthIsClamped); Assert.Equal(extent, contact.Depth); Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(new Vector3d(extent, Fixed64.Zero, Fixed64.Zero), slab);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CollapsedRadialPointBeyondACap_ShouldRetainOnlyTheFarOutwardCap(int cap)
    {
        Vector3d point = new(0, 2 * cap, 0);
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, cap, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(new(point, point), Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Array.Empty<Vector2d>(), fan, false, true, out var contact));
        Assert.Equal((Fixed64)3, contact.Depth); Assert.Equal(Vector3d.Up * cap, contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(Vector3d.Down * cap, slab);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void EndpointAxisFamily_ShouldClipTheOuterHemisphereBeforeItsAuthoredSector(int region)
    {
        Vector3d point = Vector3d.Forward * region;
        WideAxis3[] fan = { Axis(-1, 0, region), Axis(1, 0, -2 * region), Axis(0, 1, 0), Axis(0, -1, 0) };
        Assert.True(Contact(new(point, point), Fixed64.Two, Fixed64.One, Fixed64.One,
            Array.Empty<Vector2d>(), fan, false, false, out var contact));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab));
        Fixed64 radialZ = (slab.Z - point.Z) * region;
        Assert.True(slab.X > Fixed64.Zero && radialZ > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, slab.Y);
        Assert.True(radialZ <= slab.X + Fixed64.FromRaw(1) && slab.X <= 2 * radialZ + Fixed64.FromRaw(1));
        Assert.InRange((slab.X * slab.X + radialZ * radialZ).m_rawValue, Fixed64.One.m_rawValue - 2, Fixed64.One.m_rawValue + 2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CrossingLowerCap_ShouldRejectEverySourceFootBelowTheClosedSection(bool prism)
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, (Fixed64)(-3), Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        Vector2d[] offsets = prism ? new[] { new Vector2d(-1, -1), new Vector2d(1, -1),
            new Vector2d(1, 1), new Vector2d(-1, 1) } : Array.Empty<Vector2d>();
        WideAxis3[] fan = { Axis(0, 1, 0), Axis(0, -1, 0), Axis(-1, 0, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(segment, Fixed64.Zero, Fixed64.One, Fixed64.One, offsets, fan, true, false, out var contact));
        Assert.Equal(Fixed64.Half, contact.Depth); Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab));
        Assert.True(source.Y >= -Fixed64.One && source.Y <= Fixed64.Zero);
        Assert.Equal(source.Y, slab.Y); Assert.Equal(Fixed64.One, slab.X);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RepeatedPrismOffsets_ShouldPreserveDistinctCornerAdjacency(bool reverse, bool feature)
    {
        Vector2d[] clean = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Vector2d[] repeated = { clean[0], clean[0], clean[0], clean[1], clean[1], clean[2], clean[2], clean[3], clean[3], clean[0] };
        if (reverse) { Array.Reverse(clean); Array.Reverse(repeated); }
        Vector3d point = feature ? new Vector3d(2, 2, 2) : Vector3d.One * Fixed64.Half;
        int sign = feature ? -1 : 1;
        WideAxis3[] fan = { Axis(sign, -2 * sign, 0), Axis(-2 * sign, sign, 0),
            Axis(sign, 0, -2 * sign), Axis(-2 * sign, 0, sign) };
        Assert.True(Contact(new(point, point), Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            clean, fan, false, feature, out var expected));
        Assert.True(Contact(new(point, point), Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            repeated, fan, false, feature, out var actual));
        Assert.Equal(expected.Depth, actual.Depth); Assert.Equal(expected.Normal, actual.Normal);
        Assert.True(expected.SecondAnchor.TryGetPoint(out Vector3d cleanPoint));
        Assert.True(actual.SecondAnchor.TryGetPoint(out Vector3d repeatedPoint));
        Assert.Equal(cleanPoint, repeatedPoint); Assert.Equal(Vector3d.One * sign, repeatedPoint);
    }

    [Theory]
    [InlineData(1, 1, 1)]
    [InlineData(1, -1, 1)]
    [InlineData(-1, 1, 1)]
    [InlineData(1, 1, -1)]
    [InlineData(0, 1, 1)]
    [InlineData(1, 1, 0)]
    [InlineData(2, 1, 1)]
    [InlineData(1, 2, 1)]
    [InlineData(1, 2, 3)]
    public void CrossingCenterLine_ShouldRetainItsExactPerpendicularRadialWitness(int x, int y, int z)
    {
        Vector3d direction = new(x, y, z);
        Assert.True(Contact(new(-direction * 4, direction * 4), Fixed64.Zero, Fixed64.One, Fixed64.One,
            Array.Empty<Vector2d>(), Array.Empty<WideAxis3>(), true, false, out var contact));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(Vector3d.Zero, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(Fixed64.Zero, slab.Y);
        Assert.InRange((slab.X * slab.X + slab.Z * slab.Z).m_rawValue, Fixed64.One.m_rawValue - 2, Fixed64.One.m_rawValue + 2);
        Assert.InRange(Vector3d.Dot(contact.Normal, direction).m_rawValue, -3, 3);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SinglePermittedObliqueNormal_ShouldSelectAChartWithoutAnAnalyticWitness(bool feature)
    {
        Vector3d point = new(Fixed64.One * 3 / 2, Fixed64.Half, Fixed64.Two);
        Vector3d edge = new(Fixed64.Half, Fixed64.Half, -Fixed64.Half);
        WideAxis3[] fan = { Axis(1, -3, 0), Axis(-1, 3, 0), Axis(0, -4, 1), Axis(0, 4, -1), Axis(-1, 0, 0) };
        Assert.True(Contact(new(point - edge / 2, point + edge / 2), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, feature, out var contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(new Vector3d(3, 1, 4), slab);
        Fixed64 depth = FixedMath.Sqrt(Fixed64.One * 13 / 2);
        Assert.InRange(contact.Depth.m_rawValue, depth.m_rawValue - 1, depth.m_rawValue + 1);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CollinearPrismOffsets_ShouldPreserveTheWallOutwardCone(bool reverse, bool feature)
    {
        Vector2d[] clean = { new(-1, -1), new(2, -1), new(2, 1), new(-1, 1) };
        Vector2d[] collinear = { new(-1, -1), new(2, -1), new(2, 0), new(2, 1), new(-1, 1) };
        if (reverse) { Array.Reverse(clean); Array.Reverse(collinear); }
        Vector3d point = feature ? new Vector3d(3, 0, 0) : new Vector3d(Fixed64.One + Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        int sign = feature ? 1 : -1;
        WideAxis3[] fan = { Axis(sign, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(new(point, point), Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            clean, fan, false, feature, out var expected));
        Assert.True(Contact(new(point, point), Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            collinear, fan, false, feature, out var actual));
        Assert.Equal(feature ? (Fixed64)4 : Fixed64.Half, actual.Depth);
        Assert.Equal(expected.Depth, actual.Depth); Assert.Equal(expected.Normal, actual.Normal);
        Assert.True(expected.SecondAnchor.TryGetPoint(out Vector3d cleanPoint));
        Assert.True(actual.SecondAnchor.TryGetPoint(out Vector3d collinearPoint));
        Assert.Equal(cleanPoint, collinearPoint);
        Assert.Equal(feature ? Vector3d.Left : Vector3d.Right * 2, collinearPoint);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AcutePrismCorner_ShouldRetainBothIncidentFaceNormalDirections(bool reverse)
    {
        Vector2d[] offsets = { new(0, 0), new(2, 0), new(2, 1) };
        if (reverse) Array.Reverse(offsets);
        Vector3d point = new(1, 0, -1);
        WideAxis3[] fan = { Axis(1, 0, 1), Axis(-1, 0, -1), Axis(0, 1, 0), Axis(0, -1, 0), Axis(1, 0, 0) };
        Assert.True(Contact(new(point, point), Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            offsets, fan, false, true, out var contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(Vector3d.Zero, slab);
        Fixed64 depth = FixedMath.Sqrt(Fixed64.Two);
        Assert.InRange(contact.Depth.m_rawValue, depth.m_rawValue - 1, depth.m_rawValue + 1);
        Assert.True(contact.Normal.X > Fixed64.Zero); Assert.Equal(-contact.Normal.X, contact.Normal.Z);
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueFootAtAClosedEndpoint_ShouldRespectTheStrictInteriorContract(bool reverse)
    {
        Vector3d point = new(Fixed64.One * 3 / 2, Fixed64.Half, Fixed64.Two);
        Vector3d edge = new(Fixed64.Half, Fixed64.Half, -Fixed64.Half);
        Vector3d a = point, b = point + edge;
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -3, 0), Axis(-1, 3, 0), Axis(0, -4, 1), Axis(0, 4, -1), Axis(-1, 0, 0) };
        Assert.True(Contact(new(a, b), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, false, false, out var closed));
        Assert.True(closed.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(closed.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(new Vector3d(3, 1, 4), slab);
        Assert.False(Contact(new(a, b), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueFootBeyondTheAuthoredEndpoint_ShouldNotUseTheInfiniteLine(bool reverse)
    {
        Vector3d point = new(Fixed64.One * 3 / 2, Fixed64.Half, Fixed64.Two);
        Vector3d edge = new(Fixed64.Half, Fixed64.Half, -Fixed64.Half);
        Vector3d a = point + edge / 4, b = point + edge / 2;
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -3, 0), Axis(-1, 3, 0), Axis(0, -4, 1), Axis(0, 4, -1), Axis(-1, 0, 0) };
        Assert.False(Contact(new(a, b), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, false, false, out _));
        Assert.False(Contact(new(a, b), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out _));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ObliqueFootBeyondHeight_ShouldRequireTheIndependentlyAdmittedFeatureEntry(int cap)
    {
        Vector3d point = new(Fixed64.One * 3 / 2, (Fixed64)(2 * cap), Fixed64.Two);
        Vector3d edge = new((Fixed64)4, (Fixed64)(4 * cap), (Fixed64)3);
        WideAxis3[] fan = { Axis(2, cap, 0), Axis(-2, -cap, 0), Axis(0, 2 * cap, 3), Axis(0, -2 * cap, -3), Axis(-1, 0, 0) };
        var segment = new FixedSegment(point - edge / 2, point + edge / 2);
        Assert.False(Contact(segment, Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out _));
        Assert.True(Contact(segment, Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, true, out var contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(new Vector3d((Fixed64)3, (Fixed64)(-cap), (Fixed64)4), slab);
        Fixed64 depth = FixedMath.Sqrt(Fixed64.One * 61 / 4);
        Assert.InRange(contact.Depth.m_rawValue, depth.m_rawValue - 1, depth.m_rawValue + 1);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void StadiumObliqueRimFootInsideItsCentralStrip_ShouldUseTheCompleteFootprint(int region)
    {
        Vector3d point = new(-Fixed64.One * 3 / 2, Fixed64.Half, (Fixed64)(3 * region));
        Vector3d edge = new((Fixed64)1, (Fixed64)3, (Fixed64)(-region));
        WideAxis3[] fan = { Axis(1, -9, 0), Axis(-1, 9, 0), Axis(0, -12, region), Axis(0, 12, -region), Axis(-1, 0, 0) };
        Assert.True(Contact(new(point - edge / 16, point + edge / 16), (Fixed64)10, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out var contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(new Vector3d((Fixed64)3, Fixed64.One, (Fixed64)(9 * region)), slab);
        Fixed64 depth = FixedMath.Sqrt(Fixed64.One * 113 / 2);
        Assert.InRange(contact.Depth.m_rawValue, depth.m_rawValue - 1, depth.m_rawValue + 1);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliquePrincipalRimTouch_ShouldKeepItsAlreadyCertifiedAnalyticZero(bool reverse)
    {
        Vector3d point = new(1, 1, 0), edge = new(1, -1, 1);
        Vector3d a = point - edge / 4, b = point + edge / 4;
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0) };
        Assert.True(Contact(new(a, b), Fixed64.Zero, Fixed64.One, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out var contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(point, slab);
        Assert.Equal(contact.Normal.X, contact.Normal.Y); Assert.True(contact.Normal.X < Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
    }

    [Fact]
    public void SlantedZeroRadiusSection_ShouldUseItsAdmittedFreeNormalWithoutRimCharts()
    {
        Vector3d edge = new(Fixed64.One / 4, Fixed64.One / 4, Fixed64.Zero);
        WideAxis3[] fan = { Axis(0, 0, -1) };
        Assert.True(Contact(new(-edge, edge), Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out var contact));
        Assert.Equal(Fixed64.Zero, contact.Depth); Assert.Equal(Vector3d.Backward, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(Vector3d.Zero, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(Vector3d.Zero, slab);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueRimTouchOnTwoChartBoundaries_ShouldKeepTheFirstExactZero(bool reverse)
    {
        Vector3d point = new(3, 1, 4), edge = new(4, -8, 5);
        Vector3d a = point - edge / 16, b = point + edge / 16;
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -16, 0), Axis(-16, 1, 0) };
        Assert.True(Contact(new(a, b), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, false, out var contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(point, slab);
        Assert.Equal(contact.Normal.Y, contact.Normal.Z); Assert.True(contact.Normal.X < Fixed64.Zero);
        Assert.InRange((contact.Normal.X * 4 - contact.Normal.Y * 3).m_rawValue, -4, 4);
        Assert.InRange(Vector3d.Dot(contact.Normal, edge).m_rawValue, -12, 12);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BroadObliqueNormalCone_ShouldSelectAMatchedFiniteRimWitness(bool feature)
    {
        Vector3d point = new((Fixed64)3, Fixed64.Half, (Fixed64)4), edge = new(4, -8, 5);
        Vector3d a = point - edge * 4, b = point + edge * 4;
        WideAxis3[] fan = { Axis(1, -16, 0), Axis(-16, 1, 0) };
        Assert.True(Contact(new(a, b), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, feature, out var contact));
        Assert.True(Contact(new(b, a), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, true, feature, out var reversed));
        Assert.True(contact.Depth > Fixed64.Zero); Assert.Equal(contact.Depth, reversed.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(Fixed64.One, slab.Y);
        Assert.InRange((slab.X * slab.X + slab.Z * slab.Z).m_rawValue, ((Fixed64)25).m_rawValue - 20, ((Fixed64)25).m_rawValue + 20);
        Assert.True(source.X > a.X && source.X < b.X);
        Assert.InRange(Vector3d.Cross(source - point, edge).Magnitude.m_rawValue, 0, 20);
        Assert.InRange(Vector3d.Dot(contact.Normal, edge).m_rawValue, -20, 20);
        Assert.True(contact.Normal.X < Fixed64.Zero && contact.Normal.Y < Fixed64.Zero);
        Assert.True(-contact.Normal.X <= -contact.Normal.Y * 16 && -contact.Normal.Y <= -contact.Normal.X * 16);
        Assert.InRange(((slab - source).Magnitude - contact.Depth).m_rawValue, -3, 3);
    }

    [Theory]
    [InlineData(-1, false, false)]
    [InlineData(-1, false, true)]
    [InlineData(-1, true, false)]
    [InlineData(-1, true, true)]
    [InlineData(1, false, false)]
    [InlineData(1, false, true)]
    [InlineData(1, true, false)]
    [InlineData(1, true, true)]
    public void ShallowSlantedRimTouch_ShouldRetainZeroAmongOtherFiniteFeatureExits(int cap, bool reverse, bool feature)
    {
        Vector3d point = new((Fixed64)3, (Fixed64)cap, (Fixed64)4), edge = new((Fixed64)4, (Fixed64)(-cap), (Fixed64)5);
        Vector3d a = point - edge * 4, b = point + edge * 4;
        if (reverse) (a, b) = (b, a);
        Assert.True(Contact(new(a, b), Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), Array.Empty<WideAxis3>(), true, feature, out var contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(point, source);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d slab)); Assert.Equal(point, slab);
        Assert.True(contact.Normal.X < Fixed64.Zero && contact.Normal.Y * cap < Fixed64.Zero && contact.Normal.Z < Fixed64.Zero);
        Assert.InRange((contact.Normal.X * 4 - contact.Normal.Z * 3).m_rawValue, -4, 4);
        Assert.InRange((contact.Normal.Y * (3 * cap) - contact.Normal.X * 32).m_rawValue, -32, 32);
        Assert.InRange(Vector3d.Dot(contact.Normal, edge).m_rawValue, -12, 12);
    }

    private static bool Contact(FixedSegment segment, Fixed64 length, Fixed64 radius, Fixed64 height,
        ReadOnlySpan<Vector2d> offsets, ReadOnlySpan<WideAxis3> fan, bool interior, bool feature, out FixedContactAnchors contact)
        => feature ? SegmentSlabSurfaceContact.TryGetFeaturePenetration(segment, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, length, radius, height, offsets, fan, interior, out contact)
            : SegmentSlabSurfaceContact.TryGetContact(segment, Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, length, radius, height, offsets, fan, interior, out contact);

    private static WideAxis3 Axis(long x, long y, long z) => new(Signed320.ExtendValue(Signed192.Signed(x)),
        Signed320.ExtendValue(Signed192.Signed(y)), Signed320.ExtendValue(Signed192.Signed(z)));
}
