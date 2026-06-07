using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedPlaneTests
{
    [Fact]
    public void Constructor_FromThreePoints_NormalizesNormalAndContainsInputPoints()
    {
        var a = new Vector3d(0, 0, 0);
        var b = new Vector3d(1, 0, 0);
        var c = new Vector3d(0, 1, 0);

        var plane = new FixedPlane(a, b, c);

        Assert.Equal(Vector3d.Forward, plane.Normal);
        Assert.Equal(Fixed64.Zero, plane.DotCoordinate(a));
        Assert.Equal(Fixed64.Zero, plane.DotCoordinate(b));
        Assert.Equal(Fixed64.Zero, plane.DotCoordinate(c));
    }

    [Fact]
    public void Constructor_FromPointAndNormal_ComputesDistance()
    {
        var plane = new FixedPlane(new Vector3d(2, 0, 0), Vector3d.Right);

        Assert.Equal(Vector3d.Right, plane.Normal);
        Assert.Equal(new Fixed64(-2), plane.D);
        Assert.Equal(Fixed64.Zero, plane.DotCoordinate(new Vector3d(2, 3, 4)));
    }

    [Fact]
    public void Normalize_ScalesNormalAndDistanceTogether()
    {
        var plane = FixedPlane.GetNormalized(new FixedPlane(new Vector3d(0, 0, 2), new Fixed64(-4)));

        Assert.Equal(Vector3d.Forward, plane.Normal);
        Assert.Equal(new Fixed64(-2), plane.D);
    }

    [Fact]
    public void Normalize_InstanceScalesPlaneInPlace()
    {
        var plane = new FixedPlane(new Vector3d(0, 3, 0), new Fixed64(-6));

        plane.NormalizeInPlace();

        Assert.True(plane.Normal.FuzzyEqual(Vector3d.Up, Fixed64.FromDouble(0.0001)));
        FixedMathTestHelper.AssertWithinRelativeTolerance(new Fixed64(-2), plane.D);
    }

    [Fact]
    public void Normalize_ZeroNormal_Throws()
    {
        Assert.Throws<ArithmeticException>(() => FixedPlane.GetNormalized(new FixedPlane(Vector3d.Zero, Fixed64.One)));
    }

    [Fact]
    public void Intersects_BoundingBox_ClassifiesRelativeToPlane()
    {
        var plane = new FixedPlane(Vector3d.Right, new Fixed64(-1));
        var front = new FixedBoundBox(new Vector3d(3, 0, 0), new Vector3d(1, 1, 1));
        var back = new FixedBoundBox(new Vector3d(-3, 0, 0), new Vector3d(1, 1, 1));
        var intersecting = new FixedBoundBox(new Vector3d(1, 0, 0), new Vector3d(2, 1, 1));

        Assert.Equal(FixedPlaneIntersectionType.Front, plane.Intersects(front));
        Assert.Equal(FixedPlaneIntersectionType.Back, plane.Intersects(back));
        Assert.Equal(FixedPlaneIntersectionType.Intersecting, plane.Intersects(intersecting));
    }

    [Fact]
    public void Intersects_BoundingArea_ClassifiesRelativeToPlane()
    {
        var plane = new FixedPlane(Vector3d.Right, new Fixed64(-1));
        var front = new FixedBoundArea(new Vector3d(2, -1, -1), new Vector3d(3, 1, 1));
        var back = new FixedBoundArea(new Vector3d(-3, -1, -1), new Vector3d(-2, 1, 1));
        var intersecting = new FixedBoundArea(new Vector3d(0, -1, -1), new Vector3d(2, 1, 1));

        Assert.Equal(FixedPlaneIntersectionType.Front, plane.Intersects(front));
        Assert.Equal(FixedPlaneIntersectionType.Back, plane.Intersects(back));
        Assert.Equal(FixedPlaneIntersectionType.Intersecting, plane.Intersects(intersecting));
    }

    [Fact]
    public void Intersects_BoundingSphere_ClassifiesRelativeToPlane()
    {
        var plane = new FixedPlane(Vector3d.Right, new Fixed64(-1));
        var front = new FixedBoundSphere(new Vector3d(3, 0, 0), Fixed64.Half);
        var back = new FixedBoundSphere(new Vector3d(-3, 0, 0), Fixed64.Half);
        var intersecting = new FixedBoundSphere(new Vector3d(1, 0, 0), Fixed64.Half);

        Assert.Equal(FixedPlaneIntersectionType.Front, plane.Intersects(front));
        Assert.Equal(FixedPlaneIntersectionType.Back, plane.Intersects(back));
        Assert.Equal(FixedPlaneIntersectionType.Intersecting, plane.Intersects(intersecting));
    }

    [Fact]
    public void DeconstructEqualityOperatorsAndHashCode_UseNormalAndDistance()
    {
        var plane = new FixedPlane(Vector3d.Forward, new Fixed64(-2));
        var same = new FixedPlane(Vector3d.Forward, new Fixed64(-2));
        var different = new FixedPlane(Vector3d.Up, new Fixed64(-2));

        plane.Deconstruct(out var normal, out var d);

        Assert.Equal(Vector3d.Forward, normal);
        Assert.Equal(new Fixed64(-2), d);
        Assert.True(plane == same);
        Assert.False(plane != same);
        Assert.False(plane == different);
        Assert.NotEqual(plane, different);
        Assert.True(plane.Equals((object)same));
        Assert.False(plane.Equals((object)new object()));
        Assert.Equal(plane.GetHashCode(), same.GetHashCode());
    }
}
