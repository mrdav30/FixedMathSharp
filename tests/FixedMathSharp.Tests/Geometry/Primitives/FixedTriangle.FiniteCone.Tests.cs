using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedTriangleFiniteConeTests
{
    [Fact]
    public void MinimumAxialPoint_FaceInteriorTangencyWithoutEdgeCrossing_IsRetained()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.One, (Fixed64)(-100), (Fixed64)(-100)),
            new Vector3d(Fixed64.One, (Fixed64)(-100), (Fixed64)100),
            new Vector3d(Fixed64.One, (Fixed64)100, Fixed64.Zero));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Zero), point);
    }

    [Fact]
    public void MinimumAxialPoint_ObliqueFaceInteriorPrecedesAxisIntersection()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-100, 103, -100),
            new Vector3d(-100, 103, 100),
            new Vector3d(100, -97, 0));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            (Fixed64)3,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Zero), point);
    }

    [Fact]
    public void MinimumAxialPoint_InteriorTangencyPrecedesLaterEdgeCrossings()
    {
        var triangle = new FixedTriangle(
            new Vector3d(2, 1, -2),
            new Vector3d(2, 1, 2),
            new Vector3d(-2, 5, 0));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)14,
            (Fixed64)7,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Zero), point);
        Assert.True(triangle.GetEdge(1).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)14,
            (Fixed64)7,
            out Vector3d edgePoint));
        Assert.True(edgePoint.Y > point.Y);
    }

    [Fact]
    public void MinimumAxialPoint_RootCellCrossesTriangleBoundary_PreservesEdgeWitness()
    {
        long rootFloorRaw = Fixed64.MaxValue.m_rawValue / 3L;
        Fixed64 rootFloor = Fixed64.FromRaw(rootFloorRaw);
        Fixed64 rootCeiling = Fixed64.FromRaw(rootFloorRaw + 1L);
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.One, rootFloor, -Fixed64.One),
            new Vector3d(Fixed64.One, Fixed64.FromRaw(rootFloorRaw + 3L), (Fixed64)3),
            new Vector3d(Fixed64.One, rootFloor + Fixed64.One, Fixed64.Zero));
        Assert.False(triangle.ContainsProjection(new Vector3d(Fixed64.One, rootFloor, Fixed64.Zero)));
        Assert.True(triangle.ContainsProjection(new Vector3d(Fixed64.One, rootCeiling, Fixed64.Zero)));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            (Fixed64)3,
            out Vector3d point);

        Assert.True(found);
        Assert.True(triangle.GetEdge(0).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            (Fixed64)3,
            out Vector3d edgePoint));
        Assert.Equal(edgePoint, point);
        Assert.NotEqual(new Vector3d(Fixed64.One, rootFloor, Fixed64.Zero), point);
    }

    [Fact]
    public void MinimumAxialPoint_RootRoundsUpWithinBoundaryCell_PreservesEdgeWitness()
    {
        long rootFloorRaw = Fixed64.MaxValue.m_rawValue >> 1;
        Fixed64 rootFloor = Fixed64.FromRaw(rootFloorRaw);
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.One, rootFloor, -Fixed64.One),
            new Vector3d(Fixed64.One, Fixed64.FromRaw(rootFloorRaw + 3L), (Fixed64)3),
            new Vector3d(Fixed64.One, rootFloor + Fixed64.One, Fixed64.Zero));

        Assert.True(triangle.GetEdge(0).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.Two,
            out Vector3d edgePoint));
        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.Two,
            out Vector3d point));
        Assert.Equal(edgePoint, point);
    }

    [Fact]
    public void MinimumAxialPoint_RootCellExitsTriangle_PreservesEdgeWitness()
    {
        long rootFloorRaw = Fixed64.MaxValue.m_rawValue / 3L;
        Fixed64 rootFloor = Fixed64.FromRaw(rootFloorRaw);
        Fixed64 rootCeiling = Fixed64.FromRaw(rootFloorRaw + 1L);
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.One, rootFloor, -Fixed64.One),
            new Vector3d(Fixed64.One, Fixed64.FromRaw(rootFloorRaw + 3L), (Fixed64)3),
            new Vector3d(Fixed64.One, rootFloor - Fixed64.One, Fixed64.Zero));
        Assert.True(triangle.ContainsProjection(new Vector3d(Fixed64.One, rootFloor, Fixed64.Zero)));
        Assert.False(triangle.ContainsProjection(new Vector3d(Fixed64.One, rootCeiling, Fixed64.Zero)));

        Assert.True(triangle.GetEdge(0).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            (Fixed64)3,
            out Vector3d edgePoint));
        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            (Fixed64)3,
            out Vector3d point));
        Assert.Equal(edgePoint, point);
    }

    [Fact]
    public void MinimumAxialPoint_FaceAndFirstEdgeTie_PreservesEdgeWitness()
    {
        var triangle = new FixedTriangle(
            new Vector3d(1, 0, 0),
            new Vector3d(1, 10, 0),
            new Vector3d(1, 5, 10));

        Assert.True(triangle.GetEdge(0).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out Vector3d edgePoint));
        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out Vector3d point));
        Assert.Equal(edgePoint, point);
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Zero), point);
    }

    [Fact]
    public void MinimumAxialPoint_DisjointTriangleInIntersectingPlaneDoesNotHit()
    {
        var triangle = new FixedTriangle(
            new Vector3d(1, -100, 100),
            new Vector3d(1, 100, 100),
            new Vector3d(1, 0, 200));

        Assert.False(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out _));
    }

    [Fact]
    public void MinimumAxialPoint_PerpendicularPlaneUsesAxisPoint()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-100, 3, -100),
            new Vector3d(100, 3, -100),
            new Vector3d(0, 3, 100));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            (Fixed64)3,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero), point);
    }

    [Fact]
    public void MinimumAxialPoint_CollapsedConeRetainsFaceInteriorAxisIntersection()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-100, 3, -100),
            new Vector3d(100, 3, -100),
            new Vector3d(0, 3, 100));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            Fixed64.Zero,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero), point);
    }

    [Fact]
    public void MinimumAxialPoint_PlaneThroughApexRetainsApex()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, -100, -100),
            new Vector3d(0, 100, -100),
            new Vector3d(0, 0, 100));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(Vector3d.Zero, point);
    }

    [Fact]
    public void MinimumAxialPoint_DegenerateTriangleUsesCollapsedEdgesOnly()
    {
        var line = new FixedTriangle(
            new Vector3d(-2, 3, 0),
            new Vector3d(2, 3, 0),
            new Vector3d(6, 3, 0));
        var miss = new FixedTriangle(
            new Vector3d(10, 3, 0),
            new Vector3d(12, 3, 0),
            new Vector3d(14, 3, 0));

        Assert.True(line.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            (Fixed64)3,
            out Vector3d point));
        Assert.Equal(new Vector3d(-Fixed64.FromDouble(1.5), (Fixed64)3, Fixed64.Zero), point);
        Assert.False(miss.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            (Fixed64)3,
            out _));
    }

    [Fact]
    public void MinimumAxialPoint_NearUnitAxisUsesExactParametricGeometry()
    {
        var axis = new Vector3d(Fixed64.One, Fixed64.Epsilon, Fixed64.Zero);
        var triangle = new FixedTriangle(
            new Vector3d(-100, -100, 1),
            new Vector3d(100, -100, 1),
            new Vector3d(0, 100, 1));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            axis,
            (Fixed64)10,
            (Fixed64)5,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(new Vector3d(Fixed64.Two, Fixed64.Epsilon * Fixed64.Two, Fixed64.One), point);
    }

    [Fact]
    public void MinimumAxialPoint_ExtremeTriangleCoordinatesDoNotSaturatePlaneReduction()
    {
        var triangle = new FixedTriangle(
            RawVector(Fixed64.One.m_rawValue, long.MinValue, long.MinValue),
            RawVector(Fixed64.One.m_rawValue, long.MinValue, long.MaxValue),
            RawVector(Fixed64.One.m_rawValue, long.MaxValue, 0L));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Zero), point);
    }

    [Fact]
    public void MinimumAxialPoint_ExtremeTriangleCoordinatesRetainHalfwayRootRounding()
    {
        var triangle = new FixedTriangle(
            RawVector(Fixed64.One.m_rawValue, long.MinValue, long.MinValue),
            RawVector(Fixed64.One.m_rawValue, long.MinValue, long.MaxValue),
            RawVector(Fixed64.One.m_rawValue, long.MaxValue, 0L));

        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            Fixed64.Two,
            out Vector3d point);

        Assert.True(found);
        Assert.Equal(new Vector3d(Fixed64.One, (Fixed64)5, Fixed64.Zero), point);
    }

    [Fact]
    public void MinimumAxialPoint_ExtremeObliqueFaceUsesInRangeAxisBound()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        Vector3d apex = new(Fixed64.Zero, (Fixed64)(-5), Fixed64.Zero);
        var triangle = new FixedTriangle(
            RawVector(long.MinValue, long.MaxValue, long.MinValue),
            RawVector(long.MaxValue, long.MinValue, long.MinValue),
            RawVector(0L, -oneRaw.m_rawValue, long.MaxValue));

        Assert.False(triangle.GetEdge(0).TryGetFiniteConeIntersectionMinimumAxialPoint(
            apex, Vector3d.Up, (Fixed64)10, (Fixed64)5, out _));
        Assert.False(triangle.GetEdge(1).TryGetFiniteConeIntersectionMinimumAxialPoint(
            apex, Vector3d.Up, (Fixed64)10, (Fixed64)5, out _));
        Assert.False(triangle.GetEdge(2).TryGetFiniteConeIntersectionMinimumAxialPoint(
            apex, Vector3d.Up, (Fixed64)10, (Fixed64)5, out _));
        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            apex,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out Vector3d point));
        Assert.True(triangle.ContainsProjection(point));
        Assert.True(point.Y - apex.Y > Fixed64.Zero);
        Assert.True(point.Y - apex.Y < (Fixed64)5);
    }

    [Fact]
    public void MinimumAxialPoint_ExtremeDisjointPlaneBeyondCapDoesNotIntersect()
    {
        var triangle = new FixedTriangle(
            RawVector(((Fixed64)6).m_rawValue, long.MinValue, long.MinValue),
            RawVector(((Fixed64)6).m_rawValue, long.MinValue, long.MaxValue),
            RawVector(((Fixed64)6).m_rawValue, long.MaxValue, 0L));

        Assert.False(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out _));
    }

    [Fact]
    public void MinimumAxialPoint_PerpendicularPlaneWithBoundaryHitsPreservesFirstEdge()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-2, 3, -2),
            new Vector3d(2, 3, -2),
            new Vector3d(0, 3, 2));

        Assert.True(triangle.GetEdge(1).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            (Fixed64)3,
            out Vector3d edgePoint));
        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            (Fixed64)3,
            out Vector3d point));
        Assert.Equal(edgePoint, point);
    }

    [Fact]
    public void MinimumAxialPoint_PerpendicularPlanesOutsideAxialRangeDoNotIntersect()
    {
        FixedTriangle belowApex = CreatePerpendicularTriangle((Fixed64)(-1));
        FixedTriangle beyondCap = CreatePerpendicularTriangle((Fixed64)7);

        Assert.False(belowApex.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            (Fixed64)3,
            out _));
        Assert.False(beyondCap.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)6,
            (Fixed64)3,
            out _));
    }

    [Fact]
    public void MinimumAxialPoint_PerpendicularDisjointFaceIsWindingInvariant()
    {
        Vector3d a = new(10, 3, -1);
        Vector3d b = new(12, 3, -1);
        Vector3d c = new(11, 3, 1);

        Assert.False(new FixedTriangle(a, b, c).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero, Vector3d.Up, (Fixed64)6, (Fixed64)3, out _));
        Assert.False(new FixedTriangle(b, c, a).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero, Vector3d.Up, (Fixed64)6, (Fixed64)3, out _));
        Assert.False(new FixedTriangle(c, a, b).TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero, Vector3d.Up, (Fixed64)6, (Fixed64)3, out _));
    }

    [Fact]
    public void MinimumAxialPoint_SeparatesFiniteCapContactFromMiss()
    {
        Fixed64 missOffset = (Fixed64)5 + Fixed64.Epsilon;
        FixedTriangle contact = CreateAxisParallelTriangle((Fixed64)5);
        FixedTriangle miss = CreateAxisParallelTriangle(missOffset);

        Assert.True(contact.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out Vector3d contactPoint));
        Assert.Equal(new Vector3d((Fixed64)5, (Fixed64)10, Fixed64.Zero), contactPoint);
        Assert.False(miss.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)10,
            (Fixed64)5,
            out _));
    }

    [Fact]
    public void MinimumAxialPoint_ValidatesFiniteConeContract()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Up);

        Assert.Throws<ArgumentException>(() => triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero, Vector3d.One, Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.Epsilon, out _));
    }

    private static FixedTriangle CreateAxisParallelTriangle(Fixed64 x) =>
        new(
            new Vector3d(x, (Fixed64)(-100), (Fixed64)(-100)),
            new Vector3d(x, (Fixed64)(-100), (Fixed64)100),
            new Vector3d(x, (Fixed64)100, Fixed64.Zero));

    private static FixedTriangle CreatePerpendicularTriangle(Fixed64 y) =>
        new(
            new Vector3d((Fixed64)(-100), y, (Fixed64)(-100)),
            new Vector3d((Fixed64)100, y, (Fixed64)(-100)),
            new Vector3d(Fixed64.Zero, y, (Fixed64)100));

    private static Vector3d RawVector(long x, long y, long z) =>
        new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
}
