using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class BoundingBoxTests
{
    private readonly struct UnsupportedBound : IBound
    {
        public Vector3d Min { get; }
        public Vector3d Max { get; }

        public UnsupportedBound(Vector3d min, Vector3d max)
        {
            Min = min;
            Max = max;
        }

        public bool Contains(Vector3d point) => false;

        public bool Intersects(IBound other) => false;

        public Vector3d ProjectPoint(Vector3d point) => point;
    }

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
    public void DistanceToSurface_InsidePoint_ReturnsZero()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.Equal(Fixed64.Zero, box.DistanceToSurface(new Vector3d(1, 1, 1)));
    }

    [Fact]
    public void ClosestPointOnSurface_ReturnsCorrectPoint()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(3, 3, 3);

        var closestPoint = box.ClosestPointOnSurface(point);
        Assert.Equal(new Vector3d(2, 2, 2), closestPoint);
    }

    [Fact]
    public void ClosestPointOnSurface_InsidePoint_UsesNearestFace()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.Equal(new Vector3d(-2, 0.5, 0.25), box.ClosestPointOnSurface(new Vector3d(-1.75, 0.5, 0.25)));
        Assert.Equal(new Vector3d(0.25, 2, -0.75), box.ClosestPointOnSurface(new Vector3d(0.25, 1.9, -0.75)));
        Assert.Equal(new Vector3d(-0.25, 0.5, -2), box.ClosestPointOnSurface(new Vector3d(-0.25, 0.5, -1.8)));
    }

    [Fact]
    public void ProjectPoint_ClampsToBounds()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var projected = box.ProjectPoint(new Vector3d(5, -3, 1));

        Assert.Equal(new Vector3d(2, -2, 1), projected);
    }

    [Fact]
    public void GetPointOnSurfaceTowardsObject_UsesProjectedPoint()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var point = box.GetPointOnSurfaceTowardsObject(new Vector3d(5, 1, 0));

        Assert.Equal(new Vector3d(2, 1, 0), point);
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

    [Fact]
    public void Inequality_DifferentBox_ReturnsTrue()
    {
        var box1 = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = new BoundingBox(new Vector3d(1, 1, 1), new Vector3d(4, 4, 4));

        Assert.True(box1 != box2);
    }

    [Fact]
    public void Equals_WithDifferentObjectType_ReturnsFalse()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.False(box.Equals("not-a-box"));
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

    [Fact]
    public void Center_Setter_RepositionsBoundsWithoutChangingSize()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 6, 8));

        box.Center = new Vector3d(5, -1, 3);

        Assert.Equal(new Vector3d(5, -1, 3), box.Center);
        Assert.Equal(new Vector3d(4, 6, 8), box.Proportions);
        Assert.Equal(new Vector3d(3, -4, -1), box.Min);
        Assert.Equal(new Vector3d(7, 2, 7), box.Max);
    }

    [Fact]
    public void Proportions_Setter_ResizesBoundsWithoutChangingCenter()
    {
        var box = new BoundingBox(new Vector3d(1, 2, 3), new Vector3d(4, 4, 4));

        box.Proportions = new Vector3d(6, 8, 10);

        Assert.Equal(new Vector3d(1, 2, 3), box.Center);
        Assert.Equal(new Vector3d(-2, -2, -2), box.Min);
        Assert.Equal(new Vector3d(4, 6, 8), box.Max);
    }

    [Fact]
    public void Orient_WithNullSize_PreservesCurrentProportions()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 6, 8));

        box.Orient(new Vector3d(10, 20, 30), null);

        Assert.Equal(new Vector3d(10, 20, 30), box.Center);
        Assert.Equal(new Vector3d(4, 6, 8), box.Proportions);
    }

    [Fact]
    public void Orient_WithExplicitSize_UsesProvidedDimensions()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        box.Orient(new Vector3d(1, 2, 3), new Vector3d(2, 4, 6));

        Assert.Equal(new Vector3d(1, 2, 3), box.Center);
        Assert.Equal(new Vector3d(2, 4, 6), box.Proportions);
        Assert.Equal(new Vector3d(0, 0, 0), box.Min);
        Assert.Equal(new Vector3d(2, 4, 6), box.Max);
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

    [Fact]
    public void Union_EnclosesBothBoxes()
    {
        var a = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var b = new BoundingBox(new Vector3d(4, 0, 0), new Vector3d(4, 4, 4));

        var union = BoundingBox.Union(a, b);

        Assert.Equal(new Vector3d(-1, -2, -2), union.Min);
        Assert.Equal(new Vector3d(6, 2, 2), union.Max);
    }

    [Fact]
    public void FindClosestPointsBetweenBoxes_ReturnsClosestPointOnFirstBoxSurface()
    {
        var a = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var b = new BoundingBox(new Vector3d(5, 0, 0), new Vector3d(2, 2, 2));

        var closestPoint = BoundingBox.FindClosestPointsBetweenBoxes(a, b);

        Assert.Equal(new Vector3d(1, -1, -1), closestPoint);
    }

    [Fact]
    public void Intersects_WithUnsupportedBound_ReturnsFalse()
    {
        var box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var unsupported = new UnsupportedBound(new Vector3d(10, 10, 10), new Vector3d(12, 12, 12));

        Assert.False(box.Intersects(unsupported));
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
