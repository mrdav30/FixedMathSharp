using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class BoundingAreaTests
{
    #region Test: Constructor and Property

    [Fact]
    public void Constructor_AssignsCornersCorrectly()
    {
        var corner1 = new Vector3d(1, 2, 3);
        var corner2 = new Vector3d(4, 5, 6);

        var area = new BoundingArea(corner1, corner2);

        Assert.Equal(corner1, area.Corner1);
        Assert.Equal(corner2, area.Corner2);
    }

    [Fact]
    public void Constructor_WithScalarCorners_AssignsValuesCorrectly()
    {
        var area = new BoundingArea(Fixed64.One, new Fixed64(2), new Fixed64(3), new Fixed64(4), new Fixed64(5), new Fixed64(6));

        Assert.Equal(new Vector3d(1, 2, 3), area.Corner1);
        Assert.Equal(new Vector3d(4, 5, 6), area.Corner2);
    }

    [Fact]
    public void MinMaxProperties_AreCorrect()
    {
        var area = new BoundingArea(
            new Vector3d(1, 2, 3),
            new Vector3d(4, 5, 6)
        );

        Assert.Equal(Fixed64.One, area.MinX);
        Assert.Equal(new Fixed64(4), area.MaxX);
        Assert.Equal(new Fixed64(2), area.MinY);
        Assert.Equal(new Fixed64(5), area.MaxY);
        Assert.Equal(new Fixed64(3), area.MinZ);
        Assert.Equal(new Fixed64(6), area.MaxZ);
    }

    [Fact]
    public void MinMaxDimensionsAndBounds_AreCorrect_WhenCornersAreReversed()
    {
        var area = new BoundingArea(
            new Vector3d(5, 1, 6),
            new Vector3d(2, 4, 3)
        );

        Assert.Equal(new Vector3d(2, 1, 3), area.Min);
        Assert.Equal(new Vector3d(5, 4, 6), area.Max);
        Assert.Equal(new Fixed64(3), area.Width);
        Assert.Equal(new Fixed64(3), area.Height);
        Assert.Equal(new Fixed64(3), area.Depth);
    }

    [Fact]
    public void Center_ReturnsMidpointOfNormalizedBounds()
    {
        var area = new BoundingArea(
            new Vector3d(5, 1, 6),
            new Vector3d(1, 5, 2));

        Assert.Equal(new Vector3d(3, 3, 4), area.Center);
    }

    [Fact]
    public void Deconstruct_ReturnsStoredCorners()
    {
        var corner1 = new Vector3d(5, 1, 6);
        var corner2 = new Vector3d(1, 5, 2);
        var area = new BoundingArea(corner1, corner2);

        area.Deconstruct(out Vector3d actualCorner1, out Vector3d actualCorner2);

        Assert.Equal(corner1, actualCorner1);
        Assert.Equal(corner2, actualCorner2);
    }

    #endregion

    #region Test: Containment

    [Fact]
    public void Contains_PointInside_ReturnsTrue()
    {
        var area = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(5, 5, 5));
        var point = new Vector3d(3, 3, 3);

        Assert.True(area.Contains(point));
    }

    [Fact]
    public void Contains_PointOutside_ReturnsFalse()
    {
        var area = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(5, 5, 5));
        var point = new Vector3d(6, 6, 6);

        Assert.False(area.Contains(point));
    }

    [Fact]
    public void Contains_PointOnBoundary_ReturnsTrue()
    {
        var area = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(5, 5, 5));
        var point = new Vector3d(1, 3, 5);

        Assert.True(area.Contains(point));
    }

    #endregion

    #region Test: Intersection

    [Fact]
    public void Intersects_WithOverlappingArea_ReturnsTrue()
    {
        var area1 = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(5, 5, 5));
        var area2 = new BoundingArea(new Vector3d(3, 3, 3), new Vector3d(6, 6, 6));

        Assert.True(area1.Intersects(area2));

        var area3 = new BoundingArea(new Vector3d(-2, -2, 0), new Vector3d(2, 2, 0));
        var area4 = new BoundingArea(new Vector3d(-1, -1, 0), new Vector3d(3, 3, 0));
        Assert.True(area3.Intersects(area4));
    }

    [Fact]
    public void Intersects_WithNonOverlappingArea_ReturnsFalse()
    {
        var area1 = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(2, 2, 2));
        var area2 = new BoundingArea(new Vector3d(3, 3, 3), new Vector3d(4, 4, 4));

        Assert.False(area1.Intersects(area2));
    }

    [Fact]
    public void Intersects_WithBoundingBox_ReturnsTrue()
    {
        var area = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(5, 5, 5));
        var box = new BoundingBox(new Vector3d(4, 4, 4), new Vector3d(6, 6, 6));

        Assert.True(area.Intersects(box));
    }

    [Fact]
    public void Intersects_WithBoundingSphere_ReturnsTrue()
    {
        var area = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(5, 5, 5));
        var sphere = new BoundingSphere(new Vector3d(4, 4, 4), Fixed64.One);

        Assert.True(area.Intersects(sphere));
    }

    [Fact]
    public void Intersects_WithBoundingFrustum_ReturnsTrueOnlyWhenOverlapping()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var overlapping = new BoundingArea(new Vector3d(0, -2, 0), new Vector3d(2, 2, 2));
        var disjoint = new BoundingArea(new Vector3d(4, -1, 0), new Vector3d(5, 1, 1));

        Assert.True(overlapping.Intersects(frustum));
        Assert.False(disjoint.Intersects(frustum));
    }

    [Fact]
    public void Contains_TypedBounds_ReturnsContainmentClassification()
    {
        var area = new BoundingArea(new Vector3d(-2, -2, 0), new Vector3d(2, 2, 2));
        var containedArea = new BoundingArea(new Vector3d(-1, -1, 0), new Vector3d(1, 1, 1));
        var crossingBox = new BoundingBox(new Vector3d(2, 0, 1), new Vector3d(2, 1, 1));
        var containedSphere = new BoundingSphere(new Vector3d(0, 0, 1), Fixed64.Half);
        var crossingSphere = new BoundingSphere(new Vector3d(2, 0, 1), Fixed64.One);
        var disjointSphere = new BoundingSphere(new Vector3d(5, 0, 1), Fixed64.Half);
        var containedFrustum = new BoundingFrustum(Fixed4x4.Identity);
        var crossingFrustumMatrix = Fixed4x4.Identity;
        crossingFrustumMatrix.m30 = Fixed64.Two;
        var crossingFrustum = new BoundingFrustum(crossingFrustumMatrix);

        Assert.Equal(ContainmentType.Contains, area.Contains(containedArea));
        Assert.Equal(ContainmentType.Intersects, area.Contains(crossingBox));
        Assert.Equal(ContainmentType.Contains, area.Contains(containedSphere));
        Assert.Equal(ContainmentType.Intersects, area.Contains(crossingSphere));
        Assert.Equal(ContainmentType.Disjoint, area.Contains(disjointSphere));
        Assert.Equal(ContainmentType.Contains, area.Contains(containedFrustum));
        Assert.Equal(ContainmentType.Intersects, area.Contains(crossingFrustum));
    }

    [Fact]
    public void Intersects_ReturnsTrue_WhenOtherBoundIsFullyContained()
    {
        var containing = new BoundingArea(new Vector3d(0, 0, 0), new Vector3d(10, 10, 10));
        var contained = new BoundingArea(new Vector3d(2, 2, 2), new Vector3d(4, 4, 4));

        Assert.True(containing.Intersects(contained));
    }

    [Fact]
    public void Intersects_FlatXZAreas_ReturnsTrue()
    {
        var area1 = new BoundingArea(new Vector3d(0, 0, 0), new Vector3d(4, 0, 4));
        var area2 = new BoundingArea(new Vector3d(2, 0, 2), new Vector3d(6, 0, 6));

        Assert.True(area1.Intersects(area2));
    }

    [Fact]
    public void Intersects_FlatYZAreas_ReturnsTrue()
    {
        var area1 = new BoundingArea(new Vector3d(0, 0, 0), new Vector3d(0, 4, 4));
        var area2 = new BoundingArea(new Vector3d(0, 2, 2), new Vector3d(0, 6, 6));

        Assert.True(area1.Intersects(area2));
    }

    [Fact]
    public void ProjectPoint_ClampsPointWithinBounds()
    {
        var area = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));

        var projected = area.ProjectPoint(new Vector3d(10, 1, 5));

        Assert.Equal(new Vector3d(4, 2, 5), projected);
    }

    [Fact]
    public void ClampPoint_ClampsPointWithinBounds()
    {
        var area = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));

        var clamped = area.ClampPoint(new Vector3d(10, 1, 5));

        Assert.Equal(new Vector3d(4, 2, 5), clamped);
    }

    [Fact]
    public void Union_EnclosesBothAreasUsingNormalizedBounds()
    {
        var area1 = new BoundingArea(new Vector3d(4, 0, 2), new Vector3d(1, 3, 5));
        var area2 = new BoundingArea(new Vector3d(-2, 2, -1), new Vector3d(2, 6, 1));

        var union = BoundingArea.Union(area1, area2);

        Assert.Equal(new Vector3d(-2, 0, -1), union.Min);
        Assert.Equal(new Vector3d(4, 6, 5), union.Max);
        Assert.Equal(ContainmentType.Contains, union.Contains(area1));
        Assert.Equal(ContainmentType.Contains, union.Contains(area2));
    }

    #endregion

    #region Test: Equality

    [Fact]
    public void Equality_SameCorners_ReturnsTrue()
    {
        var area1 = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        var area2 = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));

        Assert.True(area1 == area2);
    }

    [Fact]
    public void Equality_DifferentCorners_ReturnsFalse()
    {
        var area1 = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        var area2 = new BoundingArea(new Vector3d(0, 2, 3), new Vector3d(4, 5, 7));

        Assert.False(area1 == area2);
    }

    [Fact]
    public void GetHashCode_SameArea_ReturnsSameHash()
    {
        var area1 = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        var area2 = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));

        Assert.Equal(area1.GetHashCode(), area2.GetHashCode());
    }

    [Fact]
    public void Inequality_AndObjectEqualityBehaveCorrectly()
    {
        var area1 = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        var area2 = new BoundingArea(new Vector3d(1, 2, 3), new Vector3d(4, 5, 7));

        Assert.True(area1 != area2);
        Assert.False(area1.Equals("not-an-area"));
    }

    #endregion

    #region Test: Edge Cases

    [Fact]
    public void Contains_ZeroSizeArea_ContainsPoint()
    {
        var area = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(1, 1, 1));
        var point = new Vector3d(1, 1, 1);

        Assert.True(area.Contains(point));
    }

    [Fact]
    public void Intersects_ZeroSizeArea_ReturnsFalseForNonOverlapping()
    {
        var area1 = new BoundingArea(new Vector3d(1, 1, 1), new Vector3d(1, 1, 1));
        var area2 = new BoundingArea(new Vector3d(2, 2, 2), new Vector3d(3, 3, 3));

        Assert.False(area1.Intersects(area2));
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void BoundingArea_NetSerialization_RoundTripMaintainsData()
    {
        BoundingArea originalValue = new(
            new Vector3d(1, 2, 3),
            new Vector3d(4, 5, 6)
        );

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles,
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<BoundingArea>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    [Fact]
    public void BoundingArea_MemoryPackSerialization_RoundTripMaintainsData()
    {
        BoundingArea originalValue = new(
            new Vector3d(1, 2, 3),
            new Vector3d(4, 5, 6)
        );

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        BoundingArea deserializedValue = MemoryPackSerializer.Deserialize<BoundingArea>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    #endregion
}
