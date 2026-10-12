using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class SegmentSlabSurfaceContactTests
{
    [Theory]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    public void PrismPointAndEdge_ShouldRetainTheExactFanSelectedFaceRimOrCorner(int dimensions, bool edge)
    {
        Vector3d point = new(Fixed64.Half, Fixed64.Half, dimensions == 3 ? Fixed64.Half : Fixed64.Zero);
        Vector3d end = edge ? point - new Vector3d(Fixed64.One / 4, Fixed64.One / 4,
            dimensions == 3 ? Fixed64.One / 4 : Fixed64.Zero) : point;
        WideAxis3[] fan = dimensions switch
        {
            1 => new[] { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) },
            2 => new[] { Axis(1, -1, 0), Axis(-1, 1, 0), Axis(0, 0, 1), Axis(0, 0, -1) },
            _ => new[] { Axis(1, -1, 0), Axis(-1, 1, 0), Axis(0, 1, -1), Axis(0, -1, 1) }
        };
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Assert.True(Contact(point, end, Fixed64.Zero, fan, offsets, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point, first);
        Assert.Equal(new Vector3d(dimensions >= 2 ? Fixed64.One : Fixed64.Half,
            Fixed64.One, dimensions == 3 ? Fixed64.One : Fixed64.Zero), second);
        Fixed64 expectedDepth = FixedMath.Sqrt((Fixed64)dimensions) / 2;
        Assert.InRange(contact.Depth.m_rawValue, expectedDepth.m_rawValue - 1, expectedDepth.m_rawValue + 1);
        Fixed64 expectedComponent = -Fixed64.One / FixedMath.Sqrt((Fixed64)dimensions);
        Assert.InRange(contact.Normal.Y.m_rawValue, expectedComponent.m_rawValue - 1, expectedComponent.m_rawValue + 1);
        Assert.Equal(dimensions >= 2 ? contact.Normal.Y : Fixed64.Zero, contact.Normal.X);
        Assert.Equal(dimensions == 3 ? contact.Normal.Y : Fixed64.Zero, contact.Normal.Z);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AdmittedOwnerPrismPointAndEdge_ShouldRetainTheFarFiniteSide(bool edge)
    {
        Vector3d point = new(3, 0, 0), end = edge ? new Vector3d(4, 0, 0) : point;
        Vector2d[] offsets = { new(-1, -1), new(2, -1), new(2, 1), new(-1, 1) };
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetFeaturePenetration(new(point, end), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, offsets, fan, interiorOnly: false, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point, first); Assert.Equal(Vector3d.Left, second);
        Assert.Equal((Fixed64)4, contact.Depth); Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CornerFan_ShouldRetainMatchedFiniteRim_WhenBothFaceNormalsAreExcluded(bool prism)
    {
        // The two incident rays exclude +X and +Y individually while admitting
        // their sum. The finite rim, rather than either unpermitted face, owns
        // q=(1,1,0) from p=(1/2,1/2,0).
        Vector3d point = new(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0), Axis(0, 0, 1) };
        Vector2d[] offsets = prism
            ? new[] { new Vector2d(-1, -1), new Vector2d(1, -1), new Vector2d(1, 1), new Vector2d(-1, 1) }
            : Array.Empty<Vector2d>();
        Assert.True(Contact(point, point, Fixed64.Zero, fan, offsets, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point, first);
        Assert.Equal(new Vector3d(1, 1, 0), second);
        Fixed64 expected = FixedMath.Sqrt(Fixed64.Half);
        Assert.InRange(contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
        Assert.InRange(contact.Normal.X.m_rawValue, -expected.m_rawValue - 1, -expected.m_rawValue + 1);
        Assert.Equal(contact.Normal.X, contact.Normal.Y);
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
    }

    [Fact]
    public void AxisCorner_ShouldClipItsFreeSideNormals_BeforeChoosingTheMinimum()
    {
        WideAxis3[] fan = { Axis(0, 1, 0), Axis(0, -1, 0), Axis(1, 0, 0), Axis(0, 0, 1) };
        Assert.True(Contact(Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, fan,
            Array.Empty<Vector2d>(), out FixedContactAnchors contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.Equal(Fixed64.Zero, second.Y);
        Assert.True(second.X <= Fixed64.Zero && second.Z <= Fixed64.Zero);
        Assert.InRange((second.X * second.X + second.Z * second.Z).m_rawValue,
            Fixed64.One.m_rawValue - 2, Fixed64.One.m_rawValue + 2);
    }

    [Fact]
    public void StadiumPoint_ShouldRejectCoveredEndpointArcs()
    {
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, -1) };
        Assert.True(Contact(Vector3d.Zero, Vector3d.Zero, Fixed64.Two, fan,
            Array.Empty<Vector2d>(), out FixedContactAnchors contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(0, 0, 2), second);
        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.Equal(Vector3d.Backward, contact.Normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PrismParallelRimFamily_ShouldKeepAnInteriorFiniteFoot(bool reverse)
    {
        Vector3d first = new(Fixed64.Half, Fixed64.Half, -Fixed64.Two);
        Vector3d second = new(Fixed64.Half, Fixed64.Half, Fixed64.Two);
        if (reverse) (first, second) = (second, first);
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(first, second), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero,
            Fixed64.One, Fixed64.One, offsets, fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(Fixed64.Half, p.X); Assert.Equal(Fixed64.Half, p.Y);
        Assert.Equal(Fixed64.One, q.X); Assert.Equal(Fixed64.One, q.Y);
        Assert.Equal(p.Z, q.Z);
        Assert.InRange(q.Z.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
        Fixed64 expected = FixedMath.Sqrt(Fixed64.Half);
        Assert.InRange(contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StadiumCentralRim_ShouldRetainMatchedStraightFoot(bool reverse)
    {
        Vector3d first = new(Fixed64.Half, Fixed64.Half, -Fixed64.Half);
        Vector3d second = new(Fixed64.Half, Fixed64.Half, Fixed64.Half);
        if (reverse) (first, second) = (second, first);
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(first, second, Fixed64.Two, fan, Array.Empty<Vector2d>(), out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.One, p.Z), q);
        Assert.Equal(Fixed64.Half, p.X); Assert.Equal(Fixed64.Half, p.Y);
        Fixed64 expected = FixedMath.Sqrt(Fixed64.Half);
        Assert.InRange(contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
    }

    [Fact]
    public void StadiumCentralCap_ShouldKeepTheFiniteRectangularCap()
    {
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Vector3d point = new(Fixed64.Half, Fixed64.Half, Fixed64.Two);
        Assert.True(Contact(point, point, (Fixed64)10, fan, Array.Empty<Vector2d>(), out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Two), q);
        Assert.Equal(Vector3d.Down, contact.Normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliquePrincipalRim_ShouldRetainTheChartAxisFoot(bool reverse)
    {
        Vector3d a = new(Fixed64.Half, Fixed64.One, -Fixed64.One / 4);
        Vector3d b = new(Fixed64.One, Fixed64.Half, Fixed64.One / 4);
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(new Vector3d(Fixed64.One * 3 / 4, Fixed64.One * 3 / 4, Fixed64.Zero), p);
        Assert.Equal(new Vector3d(1, 1, 0), q);
        Fixed64 expected = FixedMath.Sqrt(Fixed64.One / 8);
        Assert.InRange(contact.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ObliqueRim_ShouldSelectTheExactFiniteStationaryFoot(bool stadium, bool reverse)
    {
        Fixed64 core = stadium ? Fixed64.One : Fixed64.Zero;
        Vector3d a = new(Fixed64.One * 5 / 4, Fixed64.One / 4, core + Fixed64.One * 9 / 4);
        Vector3d b = new(Fixed64.One * 7 / 4, Fixed64.One * 3 / 4, core + Fixed64.One * 7 / 4);
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -4, 0), Axis(-1, 2, 0) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, stadium ? Fixed64.Two : Fixed64.Zero,
            (Fixed64)5, Fixed64.One, Array.Empty<Vector2d>(), fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Vector3d expectedP = new(Fixed64.One * 3 / 2, Fixed64.Half, core + Fixed64.Two);
        Vector3d expectedQ = new((Fixed64)3, Fixed64.One, core + (Fixed64)4);
        for (int axis = 0; axis < 3; axis++)
        {
            Assert.InRange(p[axis].m_rawValue, expectedP[axis].m_rawValue - 1, expectedP[axis].m_rawValue + 1);
            Assert.InRange(q[axis].m_rawValue, expectedQ[axis].m_rawValue - 1, expectedQ[axis].m_rawValue + 1);
        }
        Fixed64 expectedDepth = FixedMath.Sqrt(Fixed64.One * 13 / 2);
        Assert.InRange(contact.Depth.m_rawValue, expectedDepth.m_rawValue - 1, expectedDepth.m_rawValue + 1);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullRangeRim_ShouldRetainFiniteAnchorsAndNearestEvenDepth(bool prism)
    {
        Fixed64 extent = Fixed64.MaxValue, half = Fixed64.FromRaw(long.MaxValue / 2);
        Vector3d p = new(half, half, Fixed64.Zero);
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0), Axis(0, 0, 1) };
        Vector2d[] offsets = prism ? new[]
        { new Vector2d(-extent, -extent), new Vector2d(extent, -extent), new Vector2d(extent, extent), new Vector2d(-extent, extent) }
            : Array.Empty<Vector2d>();
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(p, p), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, extent, extent,
            offsets, fan, interiorOnly: false, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(p, first); Assert.Equal(new Vector3d(extent, extent, Fixed64.Zero), second);
        Assert.False(contact.DepthIsClamped);
        BigInteger delta = (BigInteger)extent.m_rawValue - half.m_rawValue;
        BigInteger squared = 8 * delta * delta, twice = 2 * (BigInteger)contact.Depth.m_rawValue;
        Assert.True((twice - 1) * (twice - 1) <= squared && squared <= (twice + 1) * (twice + 1));
        Assert.Equal(contact.Normal.X, contact.Normal.Y); Assert.Equal(Fixed64.Zero, contact.Normal.Z);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TouchingRim_ShouldClipItsActualNormalCone(bool prism)
    {
        Vector3d point = new(1, 1, 0);
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Vector2d[] offsets = prism ? new[] { new Vector2d(-1, -1), new Vector2d(1, -1), new Vector2d(1, 1), new Vector2d(-1, 1) }
            : Array.Empty<Vector2d>();
        Assert.True(Contact(point, point, Fixed64.Zero, fan, offsets, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p)); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(point, p); Assert.Equal(point, q);
        Assert.True(contact.Normal.X < Fixed64.Zero && contact.Normal.Y < Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
        Assert.True(contact.Normal.X - 2 * contact.Normal.Y >= -Fixed64.FromRaw(2));
        Assert.True(-2 * contact.Normal.X + contact.Normal.Y >= -Fixed64.FromRaw(2));
    }

    [Fact]
    public void FreeRadialFamily_ShouldFindAnExactFanWall_WhenAllPrincipalRaysAreExcluded()
    {
        WideAxis3[] fan = { Axis(-1, 0, 1), Axis(1, 0, -2), Axis(0, 1, 0), Axis(0, -1, 0) };
        Assert.True(Contact(Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, fan, Array.Empty<Vector2d>(), out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.True(q.X > Fixed64.Zero && q.Z > Fixed64.Zero); Assert.Equal(Fixed64.Zero, q.Y);
        Assert.InRange((q.X * q.X + q.Z * q.Z).m_rawValue, Fixed64.One.m_rawValue - 2, Fixed64.One.m_rawValue + 2);
        Assert.True(q.Z <= q.X + Fixed64.FromRaw(1)); Assert.True(q.X <= 2 * q.Z + Fixed64.FromRaw(1));
    }

    [Fact]
    public void ZeroRadius_ShouldRetainAnAdmittedNormalFamily()
    {
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(Vector3d.Zero, Vector3d.Zero), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Zero,
            Fixed64.One, Array.Empty<Vector2d>(), fan, interiorOnly: false, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth); Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q)); Assert.Equal(Vector3d.Zero, q);
        Assert.True(contact.Normal.X > Fixed64.Zero || contact.Normal.Z > Fixed64.Zero);
    }

    [Fact]
    public void ImpossibleFanOrOutsideSegment_ShouldNotInventAnExit()
    {
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.False(Contact(Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, fan, Array.Empty<Vector2d>(), out _));
        Assert.False(Contact(new Vector3d(3, 0, 0), new Vector3d(4, 0, 0), Fixed64.Zero,
            Array.Empty<WideAxis3>(), Array.Empty<Vector2d>(), out _));
        Assert.False(SegmentSlabSurfaceContact.TryGetContact(new(Vector3d.Zero, Vector3d.Zero), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One,
            Fixed64.One, Array.Empty<Vector2d>(), Array.Empty<WideAxis3>(), interiorOnly: true, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MeridionalRim_ShouldKeepTheExactFiniteFoot(bool reverse)
    {
        Vector3d p = new(Fixed64.One * 3 / 2, Fixed64.Half, Fixed64.Two);
        Vector3d edge = new((Fixed64)3, (Fixed64)(-25), (Fixed64)4);
        Vector3d a = p - edge / 64, b = p + edge / 64;
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -4, 0), Axis(-1, 2, 0) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first)); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(p, first); Assert.Equal(new Vector3d(3, 1, 4), second);
        Fixed64 depth = FixedMath.Sqrt(Fixed64.One * 13 / 2);
        Assert.InRange(contact.Depth.m_rawValue, depth.m_rawValue - 1, depth.m_rawValue + 1);
    }

    [Fact]
    public void VerticalSegmentFamily_ShouldKeepAnInteriorSideWitness()
    {
        WideAxis3[] fan = { Axis(0, 1, 0), Axis(0, -1, 0), Axis(1, 0, 0), Axis(0, 0, 1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero)), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p)); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(Vector3d.Zero, p); Assert.Equal(Fixed64.Zero, q.Y);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StadiumCentralCapSegment_ShouldChooseTheCapBeforeTheEndpointPole(bool reverse)
    {
        Vector3d a = new(-Fixed64.Half, Fixed64.Half, Fixed64.Two);
        Vector3d b = new(Fixed64.Half, Fixed64.Half, Fixed64.Two);
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, -1, 0), Axis(0, 0, -1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, Fixed64.One, Fixed64.One,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Half, contact.Depth); Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p)); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(Fixed64.Half, p.Y); Assert.Equal(Fixed64.Two, p.Z);
        Assert.Equal(new Vector3d(p.X, Fixed64.One, p.Z), q);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullRangeRotatedRadialSegment_ShouldKeepItsFiniteStationaryFoot(bool reverse)
    {
        Fixed64 extent = Fixed64.MaxValue, half = Fixed64.FromRaw(long.MaxValue / 2),
            quarter = Fixed64.FromRaw(long.MaxValue / 4);
        Vector3d a = new(half, Fixed64.Zero, -quarter), b = new(half, Fixed64.Zero, quarter);
        if (reverse) (a, b) = (b, a);
        FixedQuaternion sourceRotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)35, Fixed64.Zero);
        FixedQuaternion slabRotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)(-17), Fixed64.Zero);
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), Vector3d.Zero, sourceRotation,
            Vector3d.Zero, slabRotation, Fixed64.Zero, extent, extent, Array.Empty<Vector2d>(),
            Array.Empty<WideAxis3>(), interiorOnly: true, out FixedContactAnchors contact));
        Assert.Equal(extent - half, contact.Depth); Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p)); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.True(new FixedPointAnchor(Vector3d.Zero, sourceRotation, new Vector3d(half, Fixed64.Zero, Fixed64.Zero))
            .TryGetPoint(out Vector3d expectedP));
        Assert.True(new FixedPointAnchor(Vector3d.Zero, sourceRotation, new Vector3d(extent, Fixed64.Zero, Fixed64.Zero))
            .TryGetPoint(out Vector3d expectedQ));
        for (int axis = 0; axis < 3; axis++)
        {
            Assert.Equal(expectedP[axis], p[axis]);
            Assert.InRange(q[axis].m_rawValue, expectedQ[axis].m_rawValue - 2, expectedQ[axis].m_rawValue + 2);
        }
    }

    [Fact]
    public void ZeroRadiusFreeNormal_ShouldClipAPlaneSectorWithoutPrincipalRays()
    {
        WideAxis3[] fan = { Axis(-1, 0, 1), Axis(1, 0, -2), Axis(0, 1, 0), Axis(0, -1, 0) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(Vector3d.Zero, Vector3d.Zero), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Zero,
            Fixed64.One, Array.Empty<Vector2d>(), fan, interiorOnly: false, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth); Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.True(contact.Normal.X < Fixed64.Zero && contact.Normal.Z < Fixed64.Zero);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q)); Assert.Equal(Vector3d.Zero, q);
        Assert.True(-contact.Normal.X + contact.Normal.Z >= -Fixed64.FromRaw(2));
        Assert.True(contact.Normal.X - 2 * contact.Normal.Z >= -Fixed64.FromRaw(2));
    }

    [Fact]
    public void ZeroRadiusStadiumEndpoint_ShouldRejectItsInwardFreeRay()
    {
        Vector3d point = Vector3d.Forward;
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(point, point), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero,
            Fixed64.One, Array.Empty<Vector2d>(), fan, interiorOnly: false, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Two, contact.Depth); Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q)); Assert.Equal(Vector3d.Backward, q);
    }

    [Theory]
    [InlineData(0, 4)]
    [InlineData(1, 3)]
    public void OddRawCore_ShouldRoundTheCompleteRadialEndpointOnce(long xRaw, long expectedZRaw)
    {
        Vector3d point = new(Fixed64.FromRaw(xRaw), Fixed64.Zero, Fixed64.FromRaw(3));
        WideAxis3[] fan = { Axis(0, 1, 0), Axis(0, -1, 0) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(point, point), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.FromRaw(3),
            Fixed64.FromRaw(2), Fixed64.One, Array.Empty<Vector2d>(), fan, interiorOnly: false, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth); Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.True(contact.Normal.Z < Fixed64.Zero);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p)); Assert.Equal(point, p);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(xRaw), Fixed64.Zero, Fixed64.FromRaw(expectedZRaw)), q);
    }

    [Fact]
    public void FullRangeRim_ShouldClampDepthWhileRetainingBothFiniteAnchors()
    {
        WideAxis3[] fan = { Axis(1, -2, 0), Axis(-2, 1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(Vector3d.Zero, Vector3d.Zero), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.MaxValue,
            Fixed64.MaxValue, Array.Empty<Vector2d>(), fan, interiorOnly: false, out FixedContactAnchors contact));
        Assert.True(contact.DepthIsClamped); Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p)); Assert.Equal(Vector3d.Zero, p);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
        Assert.Equal(new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero), q);
        Assert.Equal(contact.Normal.X, contact.Normal.Y); Assert.Equal(Fixed64.Zero, contact.Normal.Z);
    }

    [Fact]
    public void AdmittedOwnerFeature_ShouldRetainATallCircleExitBeyondItsOriginalSection()
    {
        var segment = new FixedSegment(new Vector3d(-2, 2, -2), new Vector3d(-2, 2, 2));
        WideAxis3[] fan = { Axis(-1, 1, 0), Axis(-1, -1, 0) };
        Assert.False(SegmentSlabSurfaceContact.TryGetContact(segment, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half, (Fixed64)100,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out _));
        Assert.True(SegmentSlabSurfaceContact.TryGetFeaturePenetration(segment, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half, (Fixed64)100,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Two + Fixed64.Half, contact.Depth); Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d q)); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d p));
        Assert.Equal(new Vector3d(-2, 2, 0), q); Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Two, Fixed64.Zero), p);
    }

    [Fact]
    public void AdmittedOwnerCapFeature_ShouldRejectTheNearInwardCapFoot()
    {
        Vector3d a = new(Fixed64.One / 4, (Fixed64)101, -Fixed64.One / 4),
            b = new(Fixed64.One / 4, (Fixed64)101, Fixed64.One / 4);
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetFeaturePenetration(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half, (Fixed64)100,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.Equal((Fixed64)201, contact.Depth); Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d p));
        Assert.Equal(Fixed64.One / 4, p.X); Assert.Equal((Fixed64)(-100), p.Y); Assert.Equal(Fixed64.Zero, p.Z);
    }

    [Fact]
    public void AdmittedOwnerPrismFeature_ShouldRejectTheNearInwardSideFoot()
    {
        Vector3d q = new(3, 0, 0);
        Vector2d[] offsets = { new(-1, -1), new(2, -1), new(2, 1), new(-1, 1) };
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.False(SegmentSlabSurfaceContact.TryGetContact(new(q, q), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            offsets, fan, interiorOnly: false, out _));
        Assert.True(SegmentSlabSurfaceContact.TryGetFeaturePenetration(new(q, q), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            offsets, fan, interiorOnly: false, out FixedContactAnchors contact));
        Assert.Equal((Fixed64)4, contact.Depth); Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first)); Assert.Equal(q, first);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d p)); Assert.Equal(Vector3d.Left, p);
    }

    [Fact]
    public void AdmittedOwnerFeature_ShouldRejectAFanThatCannotSupportTheWholeOwner()
    {
        var segment = new FixedSegment(new Vector3d(-2, 2, -2), new Vector3d(-2, 2, 2));
        // An additional owner vertex beyond this edge excludes its local
        // left-facing exit; the edge cannot certify a whole-owner separation.
        WideAxis3[] fan = { Axis(-1, 1, 0), Axis(-1, -1, 0), Axis(1, 0, 0) };
        Assert.False(SegmentSlabSurfaceContact.TryGetFeaturePenetration(segment, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half, (Fixed64)100,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AdmittedOwnerCapFeature_ShouldClipItsFiniteProjectedInterval(bool prism, bool reverse)
    {
        Vector3d a = new(0, 2, -2), b = new(0, 2, 0);
        if (reverse) (a, b) = (b, a);
        Vector2d[] offsets = prism
            ? new[] { new Vector2d(-Fixed64.Half, -Fixed64.Half), new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half), new Vector2d(-Fixed64.Half, Fixed64.Half) }
            : Array.Empty<Vector2d>();
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        var segment = new FixedSegment(a, b);
        Assert.False(SegmentSlabSurfaceContact.TryGetContact(segment, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half, Fixed64.One,
            offsets, fan, interiorOnly: true, out _));
        Assert.True(SegmentSlabSurfaceContact.TryGetFeaturePenetration(segment, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half, Fixed64.One,
            offsets, fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.Equal((Fixed64)3, contact.Depth); Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d q));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d p));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Two, -Fixed64.Half), q);
        Assert.Equal(new Vector3d(Fixed64.Zero, -Fixed64.One, -Fixed64.Half), p);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AdmittedOwnerVerticalSideFeature_ShouldClipOnlyItsFiniteHeightInterval(bool reverse)
    {
        Vector3d a = new(2, 0, 0), b = new(2, 10, 0);
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetFeaturePenetration(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half, Fixed64.One,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Two + Fixed64.Half, contact.Depth); Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d q));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d p));
        Assert.Equal(new Vector3d(Fixed64.Two, Fixed64.Half, Fixed64.Zero), q);
        Assert.Equal(new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Zero), p);
    }

    private static bool Contact(Vector3d first, Vector3d second, Fixed64 length,
        ReadOnlySpan<WideAxis3> fan, ReadOnlySpan<Vector2d> offsets, out FixedContactAnchors contact) =>
        SegmentSlabSurfaceContact.TryGetContact(new FixedSegment(first, second),
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity,
            length, Fixed64.One, Fixed64.One, offsets, fan, interiorOnly: false, out contact);

    private static WideAxis3 Axis(long x, long y, long z) => new(
        Signed320.ExtendValue(Signed192.Signed(x)), Signed320.ExtendValue(Signed192.Signed(y)),
        Signed320.ExtendValue(Signed192.Signed(z)));
}
