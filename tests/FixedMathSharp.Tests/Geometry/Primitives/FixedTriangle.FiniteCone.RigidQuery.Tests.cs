using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedTriangleFiniteConeRigidQueryTests
{
    [Fact]
    public void MinimumAxialPoint_EquivalentFramesRankNearlyUnitAxisCandidatesConsistently()
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.One, (Fixed64)(-1020), -Fixed64.FromFraction(1, 128)),
            new Vector3d(Fixed64.One, (Fixed64)(-1020), Fixed64.FromFraction(1, 128)), new Vector3d(1, 4, 0));
        var axis = new Vector3d(Fixed64.Zero, Fixed64.One - Fixed64.MinIncrement, Fixed64.Zero);
        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, axis,
            (Fixed64)10, (Fixed64)5, out Vector3d expected));
        Assert.Equal(new Vector3d(Fixed64.One, (Fixed64)2 - Fixed64.FromRaw(2), Fixed64.Zero), expected);
        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            Vector3d.Zero, axis, (Fixed64)10, (Fixed64)5, out FixedPointAnchor actual));
        Assert.Equal(expected, actual.LocalPoint);
    }

    [Fact]
    public void MinimumAxialPoint_UnrepresentableLocalApexRetainsIntersectingEdge()
    {
        var triangle = new FixedTriangle(new Vector3d(-3, -1, 0), new Vector3d(3, -1, 0), new Vector3d(0, 1, 0));
        var origin = new Vector3d(3, 0, 0);
        var apex = new Vector3d(Fixed64.MinValue + (Fixed64)2, Fixed64.Half, Fixed64.Zero);

        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(origin, FixedQuaternion.Identity,
            apex, Vector3d.Down, Fixed64.One, Fixed64.MaxValue, out FixedPointAnchor hit));
        Assert.Equal(origin, hit.Origin);
        FixedMathTestHelper.AssertWithinRange(hit.LocalPoint.X, Fixed64.FromFraction(-9, 4) - Fixed64.FromRaw(4), Fixed64.FromFraction(-9, 4) + Fixed64.FromRaw(4));
        FixedMathTestHelper.AssertWithinRange(hit.LocalPoint.Y, -Fixed64.Half, -Fixed64.Half + Fixed64.FromRaw(4));
        Assert.Equal(Fixed64.Zero, hit.LocalPoint.Z);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(37)]
    [InlineData(180)]
    public void MinimumAxialPoint_RotatedFaceRetainsLocalWitnessBeyondWorldDomain(int degrees)
    {
        var triangle = new FixedTriangle(new Vector3d(1, -100, -100), new Vector3d(1, -100, 100), new Vector3d(1, 100, 0));
        var origin = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.MaxValue);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)degrees, Fixed64.Zero);

        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(origin, rotation,
            origin, Vector3d.Up, (Fixed64)10, (Fixed64)5, out FixedPointAnchor hit));
        Assert.Equal(new Vector3d(1, 2, 0), hit.LocalPoint);
        Assert.Equal(origin, hit.Origin);
        Assert.Equal(rotation, hit.Rotation);
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(0, true)]
    [InlineData(3, true)]
    [InlineData(7, false)]
    public void MinimumAxialPoint_RotatedPerpendicularFaceRespectsFiniteCaps(int y, bool expected)
    {
        var triangle = new FixedTriangle(new Vector3d(-100, y, -100), new Vector3d(100, y, -100), new Vector3d(0, y, 100));
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);
        bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, rotation,
            Vector3d.Zero, Vector3d.Up, (Fixed64)6, (Fixed64)3, out FixedPointAnchor hit);
        Assert.Equal(expected, found);
        if (expected) Assert.Equal(new Vector3d(0, y, 0), hit.LocalPoint);
        else Assert.Equal(default, hit);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void MinimumAxialPoint_ObliqueRationalFrameRetainsAnalyticFaceMinimum(bool fullDomain, bool negateQuaternion)
    {
        Fixed64 low = fullDomain ? Fixed64.MinValue : (Fixed64)(-100);
        Fixed64 high = fullDomain ? Fixed64.MaxValue : (Fixed64)100;
        var triangle = new FixedTriangle(new Vector3d(Fixed64.One, low, low),
            new Vector3d(Fixed64.One, low, high), new Vector3d(Fixed64.One, high, Fixed64.Zero));
        Fixed64 z = Fixed64.FromRaw(2576980378), w = Fixed64.FromRaw(3435973837);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, negateQuaternion ? -z : z, negateQuaternion ? -w : w);

        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, rotation,
            Vector3d.Zero, Vector3d.Up, (Fixed64)10, (Fixed64)5, out FixedPointAnchor hit));
        // With c=w²-z² and s=2zw, the plane first touches the cone at
        // local x=1, local y=(2c-s)/(2s+c), z=0. Exact integer evaluation
        // rounds y to raw -780903145; the root lattice adds at most one raw unit.
        Assert.Equal(Fixed64.One, hit.LocalPoint.X);
        FixedMathTestHelper.AssertWithinRange(hit.LocalPoint.Y, Fixed64.FromRaw(-780903146), Fixed64.FromRaw(-780903144));
        Assert.Equal(Fixed64.Zero, hit.LocalPoint.Z);
    }

    [Fact]
    public void MinimumAxialPoint_RotationDoesNotLoseAnExactApexVertex()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, new Vector3d(4, -2, 3), new Vector3d(-3, 5, 7));
        var origin = new Vector3d(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)23, (Fixed64)41, (Fixed64)(-17));
        Assert.True(triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(origin, rotation,
            origin, new Vector3d(1, 2, 3).Normalized, (Fixed64)8, (Fixed64)3, out FixedPointAnchor hit));
        Assert.Equal(Vector3d.Zero, hit.LocalPoint);
    }

    [Fact]
    public void MinimumAxialPoint_RigidQueryValidatesArguments()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Up);
        Assert.Throws<ArgumentException>(() => triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, default,
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentException>(() => triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One, out _));
    }

    [Fact]
    public void MinimumAxialPoint_NegativeIdentityAndWindingPreserveCompactQueryResults()
    {
        var triangles = new[]
        {
            new FixedTriangle(new Vector3d(1, -100, -100), new Vector3d(1, -100, 100), new Vector3d(1, 100, 0)),
            new FixedTriangle(new Vector3d(2, 1, -2), new Vector3d(2, 1, 2), new Vector3d(-2, 5, 0)),
            new FixedTriangle(new Vector3d(-2, 3, -2), new Vector3d(2, 3, -2), new Vector3d(0, 3, 2)),
            new FixedTriangle(new Vector3d(1, -3, -1), new Vector3d(1, -3, 1), new Vector3d(2, -4, 0)),
            new FixedTriangle(new Vector3d(-3, 20, -1), new Vector3d(3, 20, -1), new Vector3d(0, 20, 1))
        };
        var negativeIdentity = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, -Fixed64.One);
        var axis = new Vector3d(Fixed64.MinIncrement, Fixed64.One, Fixed64.FromRaw(256));
        foreach (FixedTriangle authored in triangles)
        {
            foreach (FixedTriangle triangle in new[] { authored, new FixedTriangle(authored.C, authored.B, authored.A) })
            {
                bool expected = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, axis, (Fixed64)10, (Fixed64)5, out Vector3d point);
                bool found = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, negativeIdentity,
                    Vector3d.Zero, axis, (Fixed64)10, (Fixed64)5, out FixedPointAnchor anchor);
                Assert.Equal(expected, found);
                Assert.Equal(point, anchor.LocalPoint);
            }
        }
    }

    [Fact]
    public void MinimumAxialPoint_RigidQueryDoesNotAllocate()
    {
        var triangle = new FixedTriangle(new Vector3d(1, -100, -100), new Vector3d(1, -100, 100), new Vector3d(1, 100, 0));
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);
        FixedPointAnchor result = default;
        long bytes = FixedMathTestHelper.MeasureWarmedAllocations(() =>
            triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero, rotation,
                Vector3d.Zero, Vector3d.Up, (Fixed64)10, (Fixed64)5, out result));
        Assert.Equal(new Vector3d(1, 2, 0), result.LocalPoint);
        Assert.Equal(0, bytes);
    }
}
