using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class RaySlabTests
{
    [Fact]
    public void AabbSlabs_RejectBeyondRangeAndPreserveGenuineLimitHit()
    {
        AssertEntry(Vector2d.Zero, new Vector2d(Fixed64.MinIncrement, Fixed64.Zero),
            new Vector2d(1, -1), new Vector2d(2, 1), null);
        AssertEntry(Vector2d.Zero, Vector2d.Right,
            new Vector2d(Fixed64.MaxValue, -Fixed64.One), new Vector2d(Fixed64.MaxValue, Fixed64.One),
            Fixed64.MaxValue);
    }

    [Fact]
    public void AabbSlabs_RejectDisjointIntervalsBeforeRounding()
    {
        Fixed64 tick = Fixed64.MinIncrement;
        AssertEntry(Vector2d.Zero, new Vector2d(tick, tick),
            new Vector2d(1, 3), new Vector2d(2, 4), null);
        AssertEntry(Vector2d.Zero, -Vector2d.Right,
            new Vector2d(1, -1), new Vector2d(2, 1), null);
        AssertEntry(Vector2d.Zero, new Vector2d(8, 8),
            new Vector2d(tick, Fixed64.FromRaw(3)),
            new Vector2d(Fixed64.FromRaw(2), Fixed64.FromRaw(4)), null);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void AabbSlabs_PreserveWideEndpointDifferenceWhenEntryFits(int sign)
    {
        int coordinate = sign * 1_500_000_000;
        Vector2d position = new(-coordinate, 0);
        Vector2d direction = new(2 * sign, 0);
        Vector2d minimum = new(coordinate, -1);
        Vector2d maximum = new(coordinate, 1);

        AssertEntry(position, direction, minimum, maximum, new Fixed64(1_500_000_000));
    }

    [Fact]
    public void AabbSlabs_AcceptMinimumRawDirection()
    {
        AssertEntry(Vector2d.Zero, new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(-1_073_741_825, -1), new Vector2d(-1_073_741_824, 1), Fixed64.Half);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    public void AabbSlabs_RoundTheAdmittedEntryHalfToEven(long rawX, long expectedRawEntry)
    {
        Fixed64 x = Fixed64.FromRaw(rawX);
        Fixed64 expected = Fixed64.FromRaw(expectedRawEntry);
        var area = FixedBoundArea.FromMinMax(new Vector2d(x, -Fixed64.One), new Vector2d(x, Fixed64.One));
        var box = FixedBoundBox.FromMinMax(
            new Vector3d(x, -Fixed64.One, -Fixed64.One), new Vector3d(x, Fixed64.One, Fixed64.One));

        Assert.Equal(expected, new FixedRay2d(Vector2d.Zero, new Vector2d(2, 0)).Intersects(area));
        Assert.Equal(expected, new FixedRay(Vector3d.Zero, new Vector3d(2, 0, 0)).Intersects(box));
    }

    private static void AssertEntry(
        Vector2d position, Vector2d direction, Vector2d minimum, Vector2d maximum, Fixed64? expected)
    {
        var area = FixedBoundArea.FromMinMax(minimum, maximum);
        var box = FixedBoundBox.FromMinMax(
            new Vector3d(minimum.X, minimum.Y, -Fixed64.One),
            new Vector3d(maximum.X, maximum.Y, Fixed64.One));
        var ray2d = new FixedRay2d(position, direction);
        var ray3d = new FixedRay(
            new Vector3d(position.X, position.Y, Fixed64.Zero),
            new Vector3d(direction.X, direction.Y, Fixed64.Zero));

        Assert.Equal(expected, ray2d.Intersects(area));
        Assert.Equal(expected, ray3d.Intersects(box));
        if (expected is Fixed64 entry)
        {
            Assert.True(area.Contains(ray2d.GetPoint(entry)));
            Assert.True(box.Contains(ray3d.GetPoint(entry)));
        }
    }
}
