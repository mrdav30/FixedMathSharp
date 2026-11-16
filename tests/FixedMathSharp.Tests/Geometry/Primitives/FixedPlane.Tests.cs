using System;
using FixedMathSharp.Geometry;
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

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void Normalize_OneRawUnitAxis_PreservesOrientationAndDistance(int sign)
    {
        var normalComponent = Fixed64.MinIncrement * sign;
        var plane = new FixedPlane(new Vector3d(normalComponent, Fixed64.Zero, Fixed64.Zero), -normalComponent);

        plane.NormalizeInPlace();

        Assert.Equal(new Vector3d(sign, 0, 0), plane.Normal);
        Assert.Equal(new Fixed64(-sign), plane.D);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Normalize_Diagonal_UsesTheUnroundedLength(bool useLargestComponent)
    {
        var component = useLargestComponent ? Fixed64.MaxValue : Fixed64.MinIncrement;
        var plane = FixedPlane.GetNormalized(new FixedPlane(new Vector3d(component, component, Fixed64.Zero), -component));
        var expectedComponent = FixedMath.Sqrt(Fixed64.Half);

        Assert.Equal(new Vector3d(expectedComponent, expectedComponent, Fixed64.Zero), plane.Normal);
        Assert.Equal(-expectedComponent, plane.D);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Normalize_DistanceOutOfRange_ThrowsWithoutChangingPlane(bool diagonal)
    {
        var plane = new FixedPlane(
            new Vector3d(Fixed64.MinIncrement, diagonal ? Fixed64.MinIncrement : Fixed64.Zero, Fixed64.Zero),
            Fixed64.One);
        var original = plane;

        Assert.Throws<OverflowException>(() => plane.NormalizeInPlace());
        Assert.Equal(original, plane);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Normalize_MinimumDistanceNearUnitLength_RoundsToMinimum(bool tilt)
    {
        var plane = new FixedPlane(new Vector3d(-Fixed64.One, tilt ? Fixed64.MinIncrement : Fixed64.Zero, Fixed64.Zero),
            Fixed64.MinValue);

        Assert.Equal(plane, FixedPlane.GetNormalized(plane));
    }

    [Fact]
    public void Normalize_MinimumDistanceRoundingOutOfRange_Throws()
    {
        // The normal's squared raw length is 2^64 - 2, so the magnitude of the
        // normalized raw offset exceeds 2^63 + 0.5 and rounds outside the signed range.
        var normal = new Vector3d(Fixed64.One - Fixed64.MinIncrement, Fixed64.FromRaw(12683), Fixed64.FromRaw(91810));
        var plane = new FixedPlane(normal, Fixed64.MinValue);

        Assert.Throws<OverflowException>(() => FixedPlane.GetNormalized(plane));
    }

    [Theory]
    [InlineData(1, 0)]
    [InlineData(3, 2)]
    [InlineData(-3, -2)]
    public void Normalize_HalfRawUnitDistance_RoundsToEven(long rawOffset, long expectedRawOffset)
    {
        var plane = new FixedPlane(new Vector3d(0, 2, 0), Fixed64.FromRaw(rawOffset));

        Assert.Equal(Fixed64.FromRaw(expectedRawOffset), FixedPlane.GetNormalized(plane).D);
    }

    [Theory]
    [InlineData(3L, 1L)]
    [InlineData(-3L, -1L)]
    [InlineData(1L, 0L)]
    public void Normalize_IrrationalLength_CorrectsRoundedEstimate(long rawOffset, long expectedRawOffset)
    {
        var plane = new FixedPlane(new Vector3d(Fixed64.Two, Fixed64.MinIncrement, Fixed64.Zero),
            Fixed64.FromRaw(rawOffset));

        // The exact length is just above two, so the magnitude falls below the midpoint.
        Assert.Equal(Fixed64.FromRaw(expectedRawOffset), FixedPlane.GetNormalized(plane).D);
    }

    [Fact]
    public void Normalize_IrrationalLength_RoundsTinyComponentToZero()
    {
        var plane = new FixedPlane(new Vector3d(new Fixed64(4), Fixed64.MinIncrement, Fixed64.Zero), Fixed64.Zero);
        Assert.Equal(Vector3d.Right, FixedPlane.GetNormalized(plane).Normal);
    }

    [Fact]
    public void Normalize_AxialMinimumAndIntegralDiagonal_RetainExactLength()
    {
        var axial = new FixedPlane(new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero), Fixed64.MaxValue);
        Assert.Equal(new FixedPlane(Vector3d.Down, Fixed64.One), FixedPlane.GetNormalized(axial));

        var diagonal = new FixedPlane(new Vector3d(3, 4, 0), new Fixed64(5));
        var unit = new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero);
        Assert.Equal(new FixedPlane(unit, Fixed64.One), FixedPlane.GetNormalized(diagonal));
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
        var front = FixedBoundBox.FromCenterAndSize(new Vector3d(3, 0, 0), new Vector3d(1, 1, 1));
        var back = FixedBoundBox.FromCenterAndSize(new Vector3d(-3, 0, 0), new Vector3d(1, 1, 1));
        var intersecting = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 0, 0), new Vector3d(2, 1, 1));

        Assert.Equal(FixedPlaneIntersectionType.Front, plane.Intersects(front));
        Assert.Equal(FixedPlaneIntersectionType.Back, plane.Intersects(back));
        Assert.Equal(FixedPlaneIntersectionType.Intersecting, plane.Intersects(intersecting));
    }

    [Fact]
    public void Intersects_MinMaxBoundingBox_ClassifiesRelativeToPlane()
    {
        var plane = new FixedPlane(Vector3d.Right, new Fixed64(-1));
        var front = FixedBoundBox.FromMinMax(new Vector3d(2, -1, -1), new Vector3d(3, 1, 1));
        var back = FixedBoundBox.FromMinMax(new Vector3d(-3, -1, -1), new Vector3d(-2, 1, 1));
        var intersecting = FixedBoundBox.FromMinMax(new Vector3d(0, -1, -1), new Vector3d(2, 1, 1));

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
