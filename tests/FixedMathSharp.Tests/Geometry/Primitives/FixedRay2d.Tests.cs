using FixedMathSharp.Bounds;
using MemoryPack;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedRay2dTests
{
    [Fact]
    public void Constructor_AssignsPositionAndDirectionWithoutNormalizing()
    {
        var ray = new FixedRay2d(new Vector2d(1, 2), new Vector2d(2, 0));

        Assert.Equal(new Vector2d(1, 2), ray.Position);
        Assert.Equal(new Vector2d(2, 0), ray.Direction);
        Assert.Equal(new Vector2d(7, 2), ray.GetPoint(new Fixed64(3)));
    }

    [Fact]
    public void Intersects_Area_ReturnsNearestForwardParameter()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));
        var ray = new FixedRay2d(new Vector2d(-5, 0), Vector2d.Right);

        Fixed64? hit = ray.Intersects(area);

        Assert.Equal(new Fixed64(4), hit);
    }

    [Fact]
    public void Intersects_Area_HandlesNegativeDirectionAndDiagonalNonUnitDirection()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));
        var negative = new FixedRay2d(new Vector2d(5, 0), Vector2d.Left);
        var diagonal = new FixedRay2d(new Vector2d(-5, -5), new Vector2d(2, 2));

        Assert.Equal(new Fixed64(4), negative.Intersects(area));
        Assert.Equal(new Fixed64(2), diagonal.Intersects(area));
    }

    [Fact]
    public void Intersects_Area_ReturnsZeroWhenRayStartsInsideOrOnBoundary()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));

        Assert.Equal(Fixed64.Zero, new FixedRay2d(Vector2d.Zero, Vector2d.Right).Intersects(area));
        Assert.Equal(Fixed64.Zero, new FixedRay2d(new Vector2d(-1, 0), Vector2d.Left).Intersects(area));
    }

    [Fact]
    public void Intersects_Area_ReturnsNullWhenParallelOutsideBehindOrZeroOutside()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));

        Assert.Null(new FixedRay2d(new Vector2d(-5, 2), Vector2d.Right).Intersects(area));
        Assert.Null(new FixedRay2d(new Vector2d(5, 0), Vector2d.Right).Intersects(area));
        Assert.Null(new FixedRay2d(new Vector2d(2, 2), Vector2d.Zero).Intersects(area));
        Assert.Equal(Fixed64.Zero, new FixedRay2d(Vector2d.Zero, Vector2d.Zero).Intersects(area));
    }

    [Fact]
    public void Intersects_Circle_ReturnsNearestForwardParameter()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var ray = new FixedRay2d(new Vector2d(-5, 0), Vector2d.Right);

        Fixed64? hit = ray.Intersects(circle);

        Assert.Equal(new Fixed64(4), hit);
    }

    [Fact]
    public void Intersects_Circle_HandlesNonUnitDirectionAndTangency()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var nonUnit = new FixedRay2d(new Vector2d(-10, 0), new Vector2d(2, 0));
        var tangent = new FixedRay2d(new Vector2d(-5, 1), Vector2d.Right);

        Assert.Equal(Fixed64.FromDouble(4.5), nonUnit.Intersects(circle));
        Assert.Equal(new Fixed64(5), tangent.Intersects(circle));
    }

    [Fact]
    public void Intersects_Circle_ReturnsZeroWhenRayStartsInsideOrOnBoundary()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, new Fixed64(2));

        Assert.Equal(Fixed64.Zero, new FixedRay2d(Vector2d.Zero, Vector2d.Right).Intersects(circle));
        Assert.Equal(Fixed64.Zero, new FixedRay2d(new Vector2d(2, 0), Vector2d.Right).Intersects(circle));
    }

    [Fact]
    public void Intersects_Circle_ReturnsNullWhenMissingPointingAwayOrZeroOutside()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);

        Assert.Null(new FixedRay2d(new Vector2d(-5, 2), Vector2d.Right).Intersects(circle));
        Assert.Null(new FixedRay2d(new Vector2d(-5, 0), Vector2d.Left).Intersects(circle));
        Assert.Null(new FixedRay2d(new Vector2d(2, 0), Vector2d.Zero).Intersects(circle));
        Assert.Equal(Fixed64.Zero, new FixedRay2d(Vector2d.Zero, Vector2d.Zero).Intersects(circle));
    }

    [Fact]
    public void EqualityDeconstructAndHashCode_UsePositionAndDirection()
    {
        var ray = new FixedRay2d(new Vector2d(1, 2), Vector2d.Forward);
        var same = new FixedRay2d(new Vector2d(1, 2), Vector2d.Forward);
        var different = new FixedRay2d(new Vector2d(1, 2), Vector2d.Right);

        ray.Deconstruct(out Vector2d position, out Vector2d direction);

        Assert.Equal(new Vector2d(1, 2), position);
        Assert.Equal(Vector2d.Forward, direction);
        Assert.True(ray == same);
        Assert.False(ray != same);
        Assert.Equal(ray.GetHashCode(), same.GetHashCode());
        Assert.NotEqual(ray, different);
        Assert.False(ray == different);
        Assert.True(ray != different);
        Assert.False(ray.Equals("not a ray"));
    }

    [Fact]
    public void JsonSerialization_RoundTripsState()
    {
        var ray = new FixedRay2d(new Vector2d(1, 2), Vector2d.Forward);

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(ray);
        var roundTrip = JsonSerializer.Deserialize<FixedRay2d>(json);

        Assert.Equal(ray, roundTrip);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_RoundTripsState()
    {
        var ray = new FixedRay2d(new Vector2d(1, 2), Vector2d.Forward);

        byte[] bytes = MemoryPackSerializer.Serialize(ray);
        var roundTrip = MemoryPackSerializer.Deserialize<FixedRay2d>(bytes);

        Assert.Equal(ray, roundTrip);
    }
#endif
}
