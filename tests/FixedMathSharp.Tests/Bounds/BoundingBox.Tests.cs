using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class BoundingBoxTests
{
    #region Test: Constructor and Property

    [Fact]
    public void Constructor_AssignsValuesCorrectly()
    {
        var center = new Vector3d(0, 0, 0);
        var size = new Vector3d(2, 2, 2);

        var box = new BoundingBox(center, size);

        Assert.Equal(center, box.Center);
        Assert.Equal(size, box.Proportions);
        Assert.Equal(new Vector3d(-1, -1, -1), box.Min);
        Assert.Equal(new Vector3d(1, 1, 1), box.Max);
    }

    #endregion

    #region Test: Containment

    [Fact]
    public void Contains_PointInside_ReturnsTrue()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(1, 1, 1);

        Assert.True(box.Contains(point));
    }

    [Fact]
    public void Contains_PointOutside_ReturnsFalse()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(5, 5, 5);

        Assert.False(box.Contains(point));
    }

    [Fact]
    public void Contains_PointOnBoundary_ReturnsTrue()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(2, 0, 0);

        Assert.True(box.Contains(point));
    }

    [Fact]
    public void Constructor_WithNegativeSize_StillContainsCenter()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(-4, -4, -4));
        Assert.True(box.Contains(new Vector3d(0, 0, 0)));
    }

    #endregion

    #region Test: Intersection

    [Fact]
    public void Intersects_WithOverlappingBox_ReturnsTrue()
    {
        var box1 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = new BoundingBox(new Vector3d(1, 1, 1), new Vector3d(4, 4, 4));

        Assert.True(box1.Intersects(box2));

        var area3 = new BoundingBox(new Vector3d(-2, -2, 0), new Vector3d(2, 2, 0));
        var area4 = new BoundingBox(new Vector3d(-1, -1, 0), new Vector3d(3, 3, 0));
        Assert.False(area3.Intersects(area4));
    }

    [Fact]
    public void Intersects_WithNonOverlappingBox_ReturnsFalse()
    {
        var box1 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var box2 = new BoundingBox(new Vector3d(5, 5, 5), new Vector3d(2, 2, 2));

        Assert.False(box1.Intersects(box2));
    }

    [Fact]
    public void Intersects_WithBoundingArea_ReturnsTrue()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var area = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(3, 3, 3));

        Assert.True(box.Intersects(area));
    }

    [Fact]
    public void Intersects_WithBoundingSphere_ReturnsTrue()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var sphere = new BoundingSphere(new Vector3d(1, 1, 1), Fixed64.One);

        Assert.True(box.Intersects(sphere));
    }

    [Fact]
    public void Intersects_TouchingEdges_ReturnsFalse()
    {
        var a = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var b = new BoundingBox(new Vector3d(2, 0, 0), new Vector3d(2, 2, 2));

        Assert.False(a.Intersects(b));
    }

    #endregion

    #region Test: Surface Distance and Closest Point

    [Fact]
    public void DistanceToSurface_WorksCorrectly()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(3, 0, 0);

        Assert.Equal(Fixed64.One, box.DistanceToSurface(point));
    }

    [Fact]
    public void ClosestPointOnSurface_ReturnsCorrectPoint()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(3, 3, 3);

        var closestPoint = box.ClosestPointOnSurface(point);
        Assert.Equal(new Vector3d(2, 2, 2), closestPoint);
    }

    #endregion

    #region Test: Equality

    [Fact]
    public void Equality_SameBox_ReturnsTrue()
    {
        var box1 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.True(box1 == box2);
    }

    [Fact]
    public void Equality_DifferentBox_ReturnsFalse()
    {
        var box1 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = new BoundingBox(new Vector3d(1, 1, 1), new Vector3d(4, 4, 4));

        Assert.False(box1 == box2);
    }

    [Fact]
    public void GetHashCode_SameBox_ReturnsSameHash()
    {
        var box1 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.Equal(box1.GetHashCode(), box2.GetHashCode());
    }

    [Fact]
    public void SetBoundingBox_KeepsSizeAndScopeInSync()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var newScope = new Vector3d(3, 3, 3);
        box.SetBoundingBox(new Vector3d(1, 1, 1), newScope);

        Assert.Equal(newScope * Fixed64.Two, box.Proportions);
    }

    #endregion

    #region Test: Edge Cases

    [Fact]
    public void Contains_ZeroSizeBox_ContainsPoint()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
        var point = new Vector3d(0, 0, 0);

        Assert.True(box.Contains(point));
    }

    [Fact]
    public void Intersects_ZeroSizeBox_ReturnsFalseForNonOverlapping()
    {
        var box1 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
        var box2 = new BoundingBox(new Vector3d(1, 1, 1), new Vector3d(2, 2, 2));

        Assert.False(box1.Intersects(box2));
    }

    #endregion

    #region Test: Mutation Invariance

    [Fact]
    public void Resize_PreservesCenter()
    {
        var box = new BoundingBox(new Vector3d(5, 5, 5), new Vector3d(4, 4, 4));

        box.Resize(new Vector3d(2, 2, 2));

        Assert.Equal(new Vector3d(5, 5, 5), box.Center);
    }

    [Fact]
    public void SetMinMax_SetsCorrectCenterAndSize()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));

        box.SetMinMax(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.Equal(new Vector3d(2, 2, 2), box.Center);
        Assert.Equal(new Vector3d(4, 4, 4), box.Proportions);
    }

    #endregion

    #region Test: Vertex Cache Safety

    [Fact]
    public void Vertices_UpdateAfterMutation()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var original = box.Vertices[0];

        box.Resize(new Vector3d(4, 4, 4));
        var updated = box.Vertices[0];

        Assert.NotEqual(original, updated);
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void BoundingBox_NetSerialization_RoundTripMaintainsData()
    {
        BoundingBox originalValue = new(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<BoundingBox>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    [Fact]
    public void BoundingBox_MemoryPackSerialization_RoundTripMaintainsData()
    {
        BoundingBox originalValue = new(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        BoundingBox deserializedValue = MemoryPackSerializer.Deserialize<BoundingBox>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    [Fact]
    public void MemoryPack_SerializedBox_RemainsMutable()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var bytes = MemoryPackSerializer.Serialize(box);
        var deserialized = MemoryPackSerializer.Deserialize<BoundingBox>(bytes);

        deserialized.Resize(new Vector3d(2, 2, 2));

        Assert.Equal(new Vector3d(2, 2, 2), deserialized.Proportions);
    }

    #endregion
}
