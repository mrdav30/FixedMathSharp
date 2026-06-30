using FixedMathSharp.Bounds;
using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedBoundAreaTests
{
    #region Test: Construction And Normalization

    [Fact]
    public void FromCenterAndSize_AssignsValuesCorrectly()
    {
        var center = new Vector2d(4, -2);
        var size = new Vector2d(6, 10);

        var area = FixedBoundArea.FromCenterAndSize(center, size);

        Assert.Equal(center, area.Center);
        Assert.Equal(size, area.Size);
        Assert.Equal(size * Fixed64.Half, area.Scope);
        Assert.Equal(new Vector2d(1, -7), area.Min);
        Assert.Equal(new Vector2d(7, 3), area.Max);
    }

    [Fact]
    public void FromMinMax_NormalizesSwappedInputs()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(5, 3), new Vector2d(-1, -7));

        Assert.Equal(new Vector2d(-1, -7), area.Min);
        Assert.Equal(new Vector2d(5, 3), area.Max);
        Assert.Equal(new Vector2d(2, -2), area.Center);
        Assert.Equal(new Vector2d(6, 10), area.Size);
        Assert.Equal(new Vector2d(3, 5), area.Scope);
    }

    [Fact]
    public void FromCenterAndSize_NormalizesNegativeSize()
    {
        var area = FixedBoundArea.FromCenterAndSize(new Vector2d(2, 4), new Vector2d(-6, -8));

        Assert.Equal(new Vector2d(-1, 0), area.Min);
        Assert.Equal(new Vector2d(5, 8), area.Max);
        Assert.Equal(new Vector2d(6, 8), area.Size);
    }

    [Fact]
    public void FromCenterAndScope_NormalizesNegativeScope()
    {
        var area = FixedBoundArea.FromCenterAndScope(new Vector2d(2, 4), new Vector2d(-3, -4));

        Assert.Equal(new Vector2d(-1, 0), area.Min);
        Assert.Equal(new Vector2d(5, 8), area.Max);
        Assert.Equal(new Vector2d(6, 8), area.Size);
        Assert.Equal(new Vector2d(3, 4), area.Scope);
    }

    [Fact]
    public void State_WithSwappedMinMax_NormalizesBounds()
    {
        var state = new FixedBoundArea.BoundingAreaState(new Vector2d(5, 3), new Vector2d(-1, -7));
        var area = new FixedBoundArea(state);

        Assert.Equal(new Vector2d(-1, -7), state.Min);
        Assert.Equal(new Vector2d(5, 3), state.Max);
        Assert.Equal(new Vector2d(-1, -7), area.Min);
        Assert.Equal(new Vector2d(5, 3), area.Max);
        Assert.Equal(new Vector2d(6, 10), area.Size);
    }

    [Fact]
    public void SetMinMax_NormalizesSwappedInputs()
    {
        var area = FixedBoundArea.FromCenterAndSize(Vector2d.Zero, new Vector2d(2, 2));

        area.SetMinMax(new Vector2d(4, 6), new Vector2d(-2, -8));

        Assert.Equal(new Vector2d(-2, -8), area.Min);
        Assert.Equal(new Vector2d(4, 6), area.Max);
        Assert.Equal(new Vector2d(6, 14), area.Size);
    }

    [Fact]
    public void Center_Setter_RepositionsBoundsWithoutChangingSize()
    {
        var area = FixedBoundArea.FromCenterAndSize(Vector2d.Zero, new Vector2d(4, 6));

        area.Center = new Vector2d(5, -1);

        Assert.Equal(new Vector2d(5, -1), area.Center);
        Assert.Equal(new Vector2d(4, 6), area.Size);
        Assert.Equal(new Vector2d(3, -4), area.Min);
        Assert.Equal(new Vector2d(7, 2), area.Max);
    }

    [Fact]
    public void Size_Setter_ResizesBoundsWithoutChangingCenter()
    {
        var area = FixedBoundArea.FromCenterAndSize(new Vector2d(1, 2), new Vector2d(4, 4));

        area.Size = new Vector2d(-6, 8);

        Assert.Equal(new Vector2d(1, 2), area.Center);
        Assert.Equal(new Vector2d(6, 8), area.Size);
        Assert.Equal(new Vector2d(-2, -2), area.Min);
        Assert.Equal(new Vector2d(4, 6), area.Max);
    }

    [Fact]
    public void Deconstruct_ReturnsNormalizedMinAndMax()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(5, 4), new Vector2d(-1, -2));

        var (min, max) = area;

        Assert.Equal(new Vector2d(-1, -2), min);
        Assert.Equal(new Vector2d(5, 4), max);
    }

    #endregion

    #region Test: Containment And Intersection

    [Fact]
    public void Contains_Point_IsBoundaryInclusive()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -3), new Vector2d(4, 5));

        Assert.True(area.Contains(new Vector2d(-2, -3)));
        Assert.True(area.Contains(new Vector2d(4, 5)));
        Assert.True(area.Contains(new Vector2d(1, 0)));
        Assert.False(area.Contains(new Vector2d(5, 0)));
        Assert.False(area.Contains(new Vector2d(0, -4)));
    }

    [Fact]
    public void Contains_Area_ReturnsContainmentClassification()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -2), new Vector2d(2, 2));
        var contained = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(1, 1));
        var crossing = FixedBoundArea.FromMinMax(new Vector2d(1, 1), new Vector2d(4, 4));
        var disjoint = FixedBoundArea.FromMinMax(new Vector2d(5, 5), new Vector2d(6, 6));

        Assert.Equal(FixedEnclosureType.Contains, area.Contains(contained));
        Assert.Equal(FixedEnclosureType.Intersects, area.Contains(crossing));
        Assert.Equal(FixedEnclosureType.Disjoint, area.Contains(disjoint));
    }

    [Fact]
    public void Intersects_IsBoundaryInclusive()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -2), new Vector2d(2, 2));
        var touchingEdge = FixedBoundArea.FromMinMax(new Vector2d(2, -1), new Vector2d(4, 1));
        var touchingCorner = FixedBoundArea.FromMinMax(new Vector2d(2, 2), new Vector2d(3, 3));
        var overlapping = FixedBoundArea.FromMinMax(new Vector2d(1, 1), new Vector2d(4, 4));
        var disjoint = FixedBoundArea.FromMinMax(new Vector2d(3, 0), new Vector2d(4, 1));

        Assert.True(area.Intersects(touchingEdge));
        Assert.True(area.Intersects(touchingCorner));
        Assert.True(area.Intersects(overlapping));
        Assert.False(area.Intersects(disjoint));
    }

    [Fact]
    public void IntersectsStrict_RequiresPositiveAreaOverlap()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -2), new Vector2d(2, 2));
        var touchingEdge = FixedBoundArea.FromMinMax(new Vector2d(2, -1), new Vector2d(4, 1));
        var touchingCorner = FixedBoundArea.FromMinMax(new Vector2d(2, 2), new Vector2d(3, 3));
        var zeroSizeInside = FixedBoundArea.FromMinMax(Vector2d.Zero, Vector2d.Zero);
        var overlapping = FixedBoundArea.FromMinMax(new Vector2d(1, 1), new Vector2d(4, 4));

        Assert.False(area.IntersectsStrict(touchingEdge));
        Assert.False(area.IntersectsStrict(touchingCorner));
        Assert.False(area.IntersectsStrict(zeroSizeInside));
        Assert.True(area.IntersectsStrict(overlapping));
    }

    #endregion

    #region Test: Projection And Union

    [Fact]
    public void ClampPoint_ClampsToBounds()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -3), new Vector2d(4, 5));

        Assert.Equal(new Vector2d(4, -3), area.ClampPoint(new Vector2d(8, -9)));
        Assert.Equal(new Vector2d(1, 2), area.ClampPoint(new Vector2d(1, 2)));
    }

    [Fact]
    public void ProjectPoint_ClampsToBounds()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -3), new Vector2d(4, 5));

        Assert.Equal(new Vector2d(-2, 5), area.ProjectPoint(new Vector2d(-8, 9)));
    }

    [Fact]
    public void Union_ReturnsContainingArea()
    {
        var a = FixedBoundArea.FromMinMax(new Vector2d(-2, -1), new Vector2d(1, 3));
        var b = FixedBoundArea.FromMinMax(new Vector2d(0, -4), new Vector2d(6, 2));

        var union = FixedBoundArea.Union(a, b);

        Assert.Equal(new Vector2d(-2, -4), union.Min);
        Assert.Equal(new Vector2d(6, 3), union.Max);
    }

    #endregion

    #region Test: Equality

    [Fact]
    public void Equality_SameArea_ReturnsTrueAndSameHashCode()
    {
        var a = FixedBoundArea.FromCenterAndSize(new Vector2d(1, 2), new Vector2d(4, 6));
        var b = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(3, 5));

        Assert.True(a.Equals(b));
        Assert.True(a == b);
        Assert.False(a != b);
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
    }

    [Fact]
    public void GetHashCode_UsesDeterministicComponentHash()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-1, -2), new Vector2d(3, 5));

        Assert.Equal(CombineVectorPairHash(area.Min, area.Max), area.GetHashCode());
    }

    [Fact]
    public void Equality_DifferentArea_ReturnsFalse()
    {
        var a = FixedBoundArea.FromCenterAndSize(new Vector2d(1, 2), new Vector2d(4, 6));
        var b = FixedBoundArea.FromCenterAndSize(new Vector2d(1, 3), new Vector2d(4, 6));

        Assert.False(a.Equals(b));
        Assert.False(a == b);
        Assert.True(a != b);
        Assert.False(a.Equals("not an area"));
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void JsonSerialization_RoundTripMaintainsData()
    {
        var originalValue = FixedBoundArea.FromCenterAndSize(new Vector2d(1, 2), new Vector2d(4, 6));

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<FixedBoundArea>(json, jsonOptions);

        Assert.Equal(originalValue, deserializedValue);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_RoundTripMaintainsData()
    {
        var originalValue = FixedBoundArea.FromCenterAndSize(new Vector2d(1, 2), new Vector2d(4, 6));

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedBoundArea deserializedValue = MemoryPackSerializer.Deserialize<FixedBoundArea>(bytes);

        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion

    private static int CombineVectorPairHash(Vector2d min, Vector2d max)
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + min.StateHash;
            hash = (hash * 31) + max.StateHash;
            return hash;
        }
    }
}
