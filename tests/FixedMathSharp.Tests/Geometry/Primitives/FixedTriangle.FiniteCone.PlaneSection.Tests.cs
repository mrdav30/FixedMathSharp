using System;
using FixedMathSharp.Geometry;
using Xunit;
using static FixedMathSharp.Tests.ConePlaneRayTestQueries;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedTriangleFiniteConePlaneSectionTests
{
    [Fact]
    public void PlaneRays_CenteredHorizontalSection_RetainBothFiniteMaxima()
    {
        FixedTriangle triangle = Horizontal(Fixed64.Zero);
        var frame = Frame(triangle);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);

        Assert.True(AccumulateTriangle(
            triangle, frame, ref positive, ref negative));
        Assert.True(positive.HasValue);
        Assert.True(negative.HasValue);
        Assert.Equal(Fixed64.Two, positive.GetRoundedMaximumDepth());
        Assert.Equal(Fixed64.Two, negative.GetRoundedMaximumDepth());
    }

    [Fact]
    public void PlaneRays_TiltedInteriorSection_AdmitsClippedLateralExtrema()
    {
        var first = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(2, 0, 2));
        var second = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, 2), new Vector3d(-2, 0, 2));
        var tilt = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
        var frame = new ConePlaneRayFrame(first, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, tilt, (Fixed64)1000, Fixed64.One);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);

        Assert.True(AccumulateTriangle(first, frame, ref positive, ref negative));
        Assert.True(AccumulateTriangle(second, frame, ref positive, ref negative));
        // Both maxima lie between 1 and 1.1. The projected global apex and
        // base supports are hundreds of units outside the authored quad.
        Assert.InRange(positive.GetRoundedMaximumDepth().m_rawValue, Fixed64.One.m_rawValue, ((Fixed64)11 / 10).m_rawValue);
        Assert.InRange(negative.GetRoundedMaximumDepth().m_rawValue, Fixed64.One.m_rawValue, ((Fixed64)11 / 10).m_rawValue);
        Assert.True(positive.GetRoundedMaximumDepth() < negative.GetRoundedMaximumDepth());
    }

    [Fact]
    public void PlaneRays_ObliqueSection_RetainsInteriorSideBaseSwitch()
    {
        var triangle = new FixedTriangle(new Vector3d(3, -3, -3), new Vector3d(3, -3, 3), new Vector3d(-3, 3, 0));
        var frame = Frame(triangle);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);

        Assert.True(AccumulateTriangle(triangle, frame, ref positive, ref negative));
        // Along N=(1,1,0), the positive maximum is 8/9*sqrt(2).
        // Along -N, side and base meet at p=(0,0,0), depth 2*sqrt(2).
        Assert.Equal(Fixed64.FromRaw(5399112000L), positive.GetRoundedMaximumDepth());
        Assert.Equal(Fixed64.FromRaw(12148002000L), negative.GetRoundedMaximumDepth());
    }

    [Theory]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void PlaneRays_ApexTouchAndOneRawGap_ClassifyBeforeRounding(long gapRaw, bool intersects)
    {
        FixedTriangle triangle = Horizontal(Fixed64.Two + Fixed64.FromRaw(gapRaw));
        var frame = Frame(triangle);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);

        Assert.Equal(intersects, AccumulateTriangle(triangle, frame, ref positive, ref negative));
        if (intersects)
        {
            Assert.Equal(Fixed64.Zero, positive.GetRoundedMaximumDepth());
            Assert.Equal((Fixed64)4, negative.GetRoundedMaximumDepth());
        }
    }

    [Fact]
    public void PlaneRays_ZeroRadius_RetainsAxisSegment()
    {
        FixedTriangle triangle = Horizontal(Fixed64.Zero);
        var frame = new ConePlaneRayFrame(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Zero);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);

        Assert.True(AccumulateTriangle(triangle, frame, ref positive, ref negative));
        Assert.Equal(Fixed64.Two, positive.GetRoundedMaximumDepth());
        Assert.Equal(Fixed64.Two, negative.GetRoundedMaximumDepth());
    }

    [Theory]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void PlaneConnectivity_SegmentTangentAndOneRawGap_UsesFiniteAdmission(long gapRaw, bool intersects)
    {
        var frame = Frame(Horizontal(Fixed64.Zero));
        Fixed64 x = Fixed64.One + Fixed64.FromRaw(gapRaw);
        var segment = new FixedSegment(new Vector3d(x, Fixed64.Zero, -Fixed64.One), new Vector3d(x, Fixed64.Zero, Fixed64.One));

        Assert.Equal(intersects, frame.IntersectsSegment(segment));
        Assert.Equal(intersects, frame.ContainsPoint(new Vector3d(x, Fixed64.Zero, Fixed64.Zero)));
    }

    [Fact]
    public void PlaneConnectivity_PointOnAnotherPlane_IsNotAdmittedToSurface()
    {
        var frame = Frame(Horizontal(Fixed64.Zero));
        Assert.False(frame.ContainsPoint(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Zero)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void PlaneFrame_NonpositiveHeight_RejectsInvalidFiniteCone(int height)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new ConePlaneRayFrame(Horizontal(Fixed64.Zero),
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)height, Fixed64.One));
    }

    [Theory]
    [InlineData(-2, 2, true)]
    [InlineData(-3, -2, false)]
    [InlineData(2, 3, false)]
    [InlineData(1, 1, true)]
    [InlineData(2, 2, false)]
    public void PlaneConnectivity_ClosedSegmentMinimum_UsesWholeFiniteInterval(int startX, int endX, bool intersects)
    {
        var frame = Frame(Horizontal(Fixed64.Zero));
        Assert.Equal(intersects, frame.IntersectsSegment(new FixedSegment(
            new Vector3d(startX, 0, 0), new Vector3d(endX, 0, 0))));
    }

    [Theory]
    [InlineData(0, -3, 3, true)]
    [InlineData(2, -3, -1, true)]
    [InlineData(3, -3, 3, false)]
    [InlineData(0, 3, 4, false)]
    [InlineData(0, -4, -3, false)]
    public void PlaneConnectivity_AxialClip_PreservesFiniteCaps(int z, int startY, int endY, bool intersects)
    {
        var plane = new FixedTriangle(new Vector3d(0, -4, -4), new Vector3d(0, 4, -4), new Vector3d(0, 0, 4));
        var frame = Frame(plane);
        var forward = new FixedSegment(new Vector3d(0, startY, z), new Vector3d(0, endY, z));
        var reverse = new FixedSegment(forward.End, forward.Start);
        Assert.Equal(intersects, frame.IntersectsSegment(forward));
        Assert.Equal(intersects, frame.IntersectsSegment(reverse));
    }

    [Fact]
    public void PlaneConnectivity_NonzeroOneRawTriangleNormal_IsRetained()
    {
        Fixed64 raw = Fixed64.FromRaw(1);
        var triangle = new FixedTriangle(Vector3d.Zero, new Vector3d(raw, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, raw));
        Assert.True(Frame(triangle).ContainsPoint(Vector3d.Zero));
    }

    [Fact]
    public void PlaneChart_QuadraticParameter_AdmitsExactIrrationalUnitRoot()
    {
        Span<ulong> values = stackalloc ulong[10 * ConePlaneRayCharts.Words];
        Span<int> signs = stackalloc int[10];
        Span<ulong> root = stackalloc ulong[ConePlaneRayCharts.RootWords];
        ContactQuadratic a = ContactQuadratic.At(values, signs, 0, ConePlaneRayCharts.Words);
        ContactQuadratic b = ContactQuadratic.At(values, signs, 1, ConePlaneRayCharts.Words);
        ContactQuadratic c = ContactQuadratic.At(values, signs, 2, ConePlaneRayCharts.Words);
        ContactQuadratic n = ContactQuadratic.At(values, signs, 3, ConePlaneRayCharts.Words);
        ContactQuadratic d = ContactQuadratic.At(values, signs, 4, ConePlaneRayCharts.Words);
        a.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(2))));
        b.Clear(); c.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(-1))));

        Assert.True(ConePlaneRayCharts.TryGetParameter(a, b, c, 1, root, n, d));
        Assert.True(ConePlaneRayCharts.IsUnitParameter(n, d, root));
        ContactQuadratic.Scale(n, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(Fixed64.One))), a);
        Assert.Equal(Fixed64.FromRaw(3037000500L), ContactQuadratic.RoundRatio(a, d, root));
        a.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(2))));
        Assert.True(ConePlaneRayCharts.TryGetParameter(a, b, c, -1, root, n, d));
        Assert.False(ConePlaneRayCharts.IsUnitParameter(n, d, root));
    }

    [Theory]
    [InlineData(0, true, 2, 2)]
    [InlineData(1, true, 0, 2)]
    [InlineData(2, false, 0, 0)]
    public void PlanePoint_RationalFirstExit_UsesClosedFiniteCone(int x, bool admitted, int positiveDepth, int negativeDepth)
    {
        FixedTriangle triangle = Horizontal(Fixed64.Zero);
        var frame = Frame(triangle);
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        point.Set(frame.Transform(new Vector3d(x, 0, 0)));
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);

        Assert.Equal(admitted, ConePlaneRayPointExits.AccumulateRationalPoint(ConePlaneRayEventSource.Plane, frame, point, ref positive, ref negative));
        Assert.Equal(admitted, positive.HasValue);
        Assert.Equal(admitted, negative.HasValue);
        if (admitted)
        {
            Assert.Equal((Fixed64)positiveDepth, positive.GetRoundedMaximumDepth());
            Assert.Equal((Fixed64)negativeDepth, negative.GetRoundedMaximumDepth());
        }
    }

    [Fact]
    public void PlanePoint_SideFirstExit_RejectsInwardZeroAndOppositeNappe()
    {
        FixedTriangle triangle = Horizontal(Fixed64.Zero);
        var frame = Frame(triangle);
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        point.Set(frame.Transform(new Vector3d(1, 0, 0)));
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);

        Assert.True(ConePlaneRayPointExits.AccumulateSidePoint(ConePlaneRayEventSource.Plane, frame, point,
            ReadOnlySpan<ulong>.Empty, ref positive, ref negative));
        Assert.Equal(Fixed64.Zero, positive.GetRoundedMaximumDepth());
        Assert.Equal(Fixed64.Two, negative.GetRoundedMaximumDepth());
    }

    [Fact]
    public void PlanePoint_GeneratorFirstExit_RetainsBothAxialEndpoints()
    {
        var triangle = new FixedTriangle(new Vector3d(-3, -2, -4), new Vector3d(5, 2, -4), new Vector3d(1, 0, 4));
        var frame = Frame(triangle);
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        point.Set(frame.Transform(new Vector3d(1, 0, 0)));
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);

        Assert.True(ConePlaneRayPointExits.AccumulateSidePoint(ConePlaneRayEventSource.Plane, frame, point,
            ReadOnlySpan<ulong>.Empty, ref positive, ref negative));
        Assert.Equal(Fixed64.FromRaw(9603838835L), positive.GetRoundedMaximumDepth());
        Assert.Equal(Fixed64.FromRaw(9603838835L), negative.GetRoundedMaximumDepth());
    }

    [Fact]
    public void PlanePoint_Materialization_InvertsExactConeFrameBeforeRounding()
    {
        var triangle = Horizontal(Fixed64.Zero);
        var tilt = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
        var frame = new ConePlaneRayFrame(triangle, new Vector3d(4, 5, -3), FixedQuaternion.Identity,
            new Vector3d(-2, 4, 3), tilt, (Fixed64)4, Fixed64.Two);
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs);
        point.Set(frame.Transform(new Vector3d(1, -1, 2)));

        Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, ReadOnlySpan<ulong>.Empty, out Vector3d world));
        Assert.Equal(new Vector3d(5, 4, -1), world);
    }

    [Fact]
    public void PlanePoint_Materialization_RoundsHalfRawCoordinatesToEven()
    {
        var frame = Frame(Horizontal(Fixed64.Zero));
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs);
        Signed192 scale = frame.Finite.ShapeFrame.Denominator;
        point.X.Set(Signed576.ExtendValue(Signed320.ExtendValue(scale)));
        point.Y.Set(Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap));
        point.Z.Set(Signed576.ExtendValue(WideArithmetic.MultiplySigned192(scale, Signed192.Signed(-3))));
        point.Denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));

        Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, ReadOnlySpan<ulong>.Empty, out Vector3d world));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-2)), world);
    }

    [Theory]
    [InlineData(long.MaxValue, 1L)]
    [InlineData(long.MinValue, -1L)]
    public void PlanePoint_Materialization_OutOfRangeWorldCoordinate_ReturnsFalse(long centerRaw, long pointRaw)
    {
        var center = new Vector3d(Fixed64.FromRaw(centerRaw), Fixed64.Zero, Fixed64.Zero);
        var frame = new ConePlaneRayFrame(Horizontal(Fixed64.Zero), center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs);
        point.Set(frame.Transform(new Vector3d(Fixed64.FromRaw(pointRaw), Fixed64.Zero, Fixed64.Zero)));

        Assert.False(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, ReadOnlySpan<ulong>.Empty, out _));
    }

    [Fact]
    public void PlanePoint_ExitMaterialization_PreservesSamePointFiniteWitnesses()
    {
        FixedTriangle triangle = Horizontal(Fixed64.Zero);
        var frame = Frame(triangle);
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        point.Set(frame.Transform(Vector3d.Zero));
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(ConePlaneRayPointExits.AccumulateRationalPoint(ConePlaneRayEventSource.Plane, frame, point, ref positive, ref negative));

        Assert.True(ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, point, positive.Root,
            ContactQuadratic.At(pv, ps, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(pv, ps, 1, ConePlaneRaySelection.FieldWords), 1, out Vector3d positivePoint));
        Assert.True(ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, point, negative.Root,
            ContactQuadratic.At(nv, ns, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(nv, ns, 1, ConePlaneRaySelection.FieldWords), -1, out Vector3d negativePoint));
        Assert.Equal(new Vector3d(0, 2, 0), positivePoint);
        Assert.Equal(new Vector3d(0, -2, 0), negativePoint);
    }

    private static FixedTriangle Horizontal(Fixed64 y) => new(
        new Vector3d((Fixed64)(-4), y, (Fixed64)(-4)),
        new Vector3d((Fixed64)4, y, (Fixed64)(-4)),
        new Vector3d(Fixed64.Zero, y, (Fixed64)4));

    private static ConePlaneRayFrame Frame(FixedTriangle triangle) => new(triangle,
        Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
}
