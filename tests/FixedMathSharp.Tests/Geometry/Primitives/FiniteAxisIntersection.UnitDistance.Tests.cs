using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Theory]
    [InlineData(4, 0, 6, 0, false, 0, 0, false, false)] // Outside, moving away.
    [InlineData(6, 0, 4, 0, false, 0, 0, false, false)] // Approaches, but stops outside.
    [InlineData(-1, 0, 1, 0, true, 0, 4, true, true)] // Entire chord is contained.
    [InlineData(-4, 2, 4, 2, true, 2, 2, false, false)] // Interior tangency.
    [InlineData(-4, 2, 0, 2, true, 4, 4, false, false)] // Tangency at the endpoint.
    [InlineData(2, 0, 4, 0, true, 0, 0, true, false)] // Starts on the boundary, exits immediately.
    [InlineData(4, 0, 0, 0, true, 2, 4, false, true)] // Enters, then ends inside.
    [InlineData(0, 0, 4, 0, true, 0, 2, true, false)] // Starts inside, then exits.
    [InlineData(-4, 4, 4, 4, false, 0, 0, false, false)] // Closest interior point is outside.
    [InlineData(0, 0, 0, 0, true, 0, 4, true, true)] // Stationary contained point.
    [InlineData(4, 0, 4, 0, false, 0, 0, false, false)] // Stationary outside point.
    public void RadialDistanceIntervals_ClassifyUnitEndpointSigns(
        int startXHalves,
        int startYHalves,
        int endXHalves,
        int endYHalves,
        bool expectedHit,
        int expectedEntry,
        int expectedExit,
        bool expectedStartContained,
        bool expectedEndContainedStrict)
    {
        var start = new Vector2d(
            Fixed64.FromFraction(startXHalves, 2), Fixed64.FromFraction(startYHalves, 2));
        var end = new Vector2d(
            Fixed64.FromFraction(endXHalves, 2), Fixed64.FromFraction(endYHalves, 2));
        var query2d = new FixedSegment2d(start, end);
        var query3d = new FixedSegment(
            new Vector3d(start.X, start.Y, Fixed64.Zero),
            new Vector3d(end.X, end.Y, Fixed64.Zero));

        bool hit2d = query2d.TryGetCircleIntersectionDistanceInterval(
            new FixedBoundCircle(Vector2d.Zero, Fixed64.One), Fixed64.Zero, (Fixed64)4,
            out Fixed64 entry2d, out Fixed64 exit2d,
            out bool startContained2d, out bool endContainedStrict2d);
        bool hit3d = query3d.TryGetSphereIntersectionDistanceInterval(
            new FixedBoundSphere(Vector3d.Zero, Fixed64.One), Fixed64.Zero, (Fixed64)4,
            out Fixed64 entry3d, out Fixed64 exit3d,
            out bool startContained3d, out bool endContainedStrict3d);

        Assert.Equal(expectedHit, hit2d);
        Assert.Equal(expectedHit, hit3d);
        Assert.Equal((Fixed64)expectedEntry, entry2d);
        Assert.Equal((Fixed64)expectedEntry, entry3d);
        Assert.Equal((Fixed64)expectedExit, exit2d);
        Assert.Equal((Fixed64)expectedExit, exit3d);
        Assert.Equal(expectedStartContained, startContained2d);
        Assert.Equal(expectedStartContained, startContained3d);
        Assert.Equal(expectedEndContainedStrict, endContainedStrict2d);
        Assert.Equal(expectedEndContainedStrict, endContainedStrict3d);
    }

    [Theory]
    [InlineData(false, 2, 4, false, true)]
    [InlineData(true, 0, 2, true, false)]
    public void RadialDistanceIntervals_PreserveEndpointSignsAcrossFullRawDomain(
        bool reverse,
        int expectedEntry,
        int expectedExit,
        bool expectedStartContained,
        bool expectedEndContainedStrict)
    {
        Fixed64 startX = reverse ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 endX = reverse ? Fixed64.MinValue : Fixed64.MaxValue;
        var query2d = new FixedSegment2d(
            new Vector2d(startX, Fixed64.Zero), new Vector2d(endX, Fixed64.Zero));
        var query3d = new FixedSegment(
            new Vector3d(startX, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(endX, Fixed64.Zero, Fixed64.Zero));

        // The full chord spans 2^64 - 1 raw units and the radius is 2^63 - 1.
        // The unrounded crossing is just beyond (or before) the midpoint;
        // its deviation is below half a raw unit in the four-unit distance domain.
        Assert.True(query2d.TryGetCircleIntersectionDistanceInterval(
            new FixedBoundCircle(new Vector2d(Fixed64.MaxValue, Fixed64.Zero), Fixed64.MaxValue),
            Fixed64.Zero, (Fixed64)4, out Fixed64 entry2d, out Fixed64 exit2d,
            out bool startContained2d, out bool endContainedStrict2d));
        Assert.True(query3d.TryGetSphereIntersectionDistanceInterval(
            new FixedBoundSphere(new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), Fixed64.MaxValue),
            Fixed64.Zero, (Fixed64)4, out Fixed64 entry3d, out Fixed64 exit3d,
            out bool startContained3d, out bool endContainedStrict3d));

        Assert.Equal((Fixed64)expectedEntry, entry2d);
        Assert.Equal((Fixed64)expectedEntry, entry3d);
        Assert.Equal((Fixed64)expectedExit, exit2d);
        Assert.Equal((Fixed64)expectedExit, exit3d);
        Assert.Equal(expectedStartContained, startContained2d);
        Assert.Equal(expectedStartContained, startContained3d);
        Assert.Equal(expectedEndContainedStrict, endContainedStrict2d);
        Assert.Equal(expectedEndContainedStrict, endContainedStrict3d);
    }
}
