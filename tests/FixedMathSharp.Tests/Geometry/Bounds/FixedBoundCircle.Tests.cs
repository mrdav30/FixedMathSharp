using FixedMathSharp.Bounds;
using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedBoundCircleTests
{
    #region Test: Construction And Normalization

    [Fact]
    public void Constructor_NormalizesNegativeRadiusAndComputesDerivedBounds()
    {
        var circle = new FixedBoundCircle(new Vector2d(2, -3), new Fixed64(-5));

        Assert.Equal(new Vector2d(2, -3), circle.Center);
        Assert.Equal(new Fixed64(5), circle.Radius);
        Assert.Equal(new Fixed64(25), circle.RadiusSquared);
        Assert.Equal(FixedBoundArea.FromMinMax(new Vector2d(-3, -8), new Vector2d(7, 2)), circle.Bounds);
    }

    [Fact]
    public void Radius_Setter_NormalizesNegativeRadius()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);

        circle.Radius = new Fixed64(-3);

        Assert.Equal(new Fixed64(3), circle.Radius);
        Assert.Equal(new Fixed64(9), circle.RadiusSquared);
        Assert.Equal(FixedBoundArea.FromMinMax(new Vector2d(-3, -3), new Vector2d(3, 3)), circle.Bounds);
    }

    [Fact]
    public void State_WithNegativeRadius_NormalizesRadius()
    {
        var state = new FixedBoundCircle.BoundingCircleState(new Vector2d(4, 5), new Fixed64(-6));
        var circle = new FixedBoundCircle(state);

        Assert.Equal(new Vector2d(4, 5), state.Center);
        Assert.Equal(new Fixed64(6), state.Radius);
        Assert.Equal(new Vector2d(4, 5), circle.Center);
        Assert.Equal(new Fixed64(6), circle.Radius);
    }

    [Fact]
    public void Deconstruct_ReturnsCenterAndNormalizedRadius()
    {
        var circle = new FixedBoundCircle(new Vector2d(3, 4), new Fixed64(-7));

        var (center, radius) = circle;

        Assert.Equal(new Vector2d(3, 4), center);
        Assert.Equal(new Fixed64(7), radius);
    }

    #endregion

    #region Test: Containment And Intersection

    [Fact]
    public void Contains_Point_IsBoundaryInclusive()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, new Fixed64(5));

        Assert.True(circle.Contains(Vector2d.Zero));
        Assert.True(circle.Contains(new Vector2d(3, 4)));
        Assert.True(circle.Contains(new Vector2d(5, 0)));
        Assert.False(circle.Contains(new Vector2d(6, 0)));
    }

    [Fact]
    public void Contains_Circle_ReturnsContainmentClassification()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, new Fixed64(5));
        var contained = new FixedBoundCircle(new Vector2d(1, 0), new Fixed64(2));
        var largerSameCenter = new FixedBoundCircle(Vector2d.Zero, new Fixed64(6));
        var crossing = new FixedBoundCircle(new Vector2d(4, 0), new Fixed64(2));
        var touching = new FixedBoundCircle(new Vector2d(7, 0), new Fixed64(2));
        var disjoint = new FixedBoundCircle(new Vector2d(8, 0), Fixed64.One);

        Assert.Equal(FixedEnclosureType.Contains, circle.Contains(contained));
        Assert.Equal(FixedEnclosureType.Intersects, circle.Contains(largerSameCenter));
        Assert.Equal(FixedEnclosureType.Intersects, circle.Contains(crossing));
        Assert.Equal(FixedEnclosureType.Intersects, circle.Contains(touching));
        Assert.Equal(FixedEnclosureType.Disjoint, circle.Contains(disjoint));
    }

    [Fact]
    public void Intersects_Circle_IsBoundaryInclusive()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, new Fixed64(2));
        var touching = new FixedBoundCircle(new Vector2d(4, 0), new Fixed64(2));
        var overlapping = new FixedBoundCircle(new Vector2d(3, 0), new Fixed64(2));
        var disjoint = new FixedBoundCircle(new Vector2d(5, 0), new Fixed64(2));

        Assert.True(circle.Intersects(touching));
        Assert.True(circle.Intersects(overlapping));
        Assert.False(circle.Intersects(disjoint));
    }

    [Fact]
    public void IntersectsStrict_Circle_RequiresPositiveAreaOverlap()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, new Fixed64(2));
        var touching = new FixedBoundCircle(new Vector2d(4, 0), new Fixed64(2));
        var overlapping = new FixedBoundCircle(new Vector2d(3, 0), new Fixed64(2));
        var disjoint = new FixedBoundCircle(new Vector2d(5, 0), new Fixed64(2));
        var zeroRadiusInside = new FixedBoundCircle(Vector2d.Zero, Fixed64.Zero);

        Assert.True(circle.Intersects(touching));
        Assert.False(circle.IntersectsStrict(touching));
        Assert.True(circle.IntersectsStrict(overlapping));
        Assert.False(circle.Intersects(disjoint));
        Assert.False(circle.IntersectsStrict(zeroRadiusInside));
    }

    [Fact]
    public void Intersects_Area_UsesClosestPointAndBoundaryTouch()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, new Fixed64(2));
        var touchingEdge = FixedBoundArea.FromMinMax(new Vector2d(2, -1), new Vector2d(4, 1));
        var overlappingCorner = FixedBoundArea.FromMinMax(new Vector2d(1, 1), new Vector2d(4, 4));
        var disjoint = FixedBoundArea.FromMinMax(new Vector2d(3, 3), new Vector2d(4, 4));

        Assert.True(circle.Intersects(touchingEdge));
        Assert.True(circle.Intersects(overlappingCorner));
        Assert.False(circle.Intersects(disjoint));
    }

    [Fact]
    public void IntersectsStrict_Area_RequiresPositiveAreaOverlap()
    {
        var circle = new FixedBoundCircle(Vector2d.Zero, new Fixed64(2));
        var touchingEdge = FixedBoundArea.FromMinMax(new Vector2d(2, -1), new Vector2d(4, 1));
        var overlappingCorner = FixedBoundArea.FromMinMax(new Vector2d(1, 1), new Vector2d(4, 4));
        var zeroSizeInside = FixedBoundArea.FromMinMax(Vector2d.Zero, Vector2d.Zero);

        Assert.True(circle.Intersects(touchingEdge));
        Assert.False(circle.IntersectsStrict(touchingEdge));
        Assert.True(circle.IntersectsStrict(overlappingCorner));
        Assert.False(circle.IntersectsStrict(zeroSizeInside));
    }

    [Fact]
    public void ZeroRadiusCircle_BehavesAsBoundaryInclusivePoint()
    {
        var circle = new FixedBoundCircle(new Vector2d(1, 1), Fixed64.Zero);
        var samePoint = new FixedBoundCircle(new Vector2d(1, 1), Fixed64.Zero);
        var areaContainingPoint = FixedBoundArea.FromMinMax(Vector2d.Zero, new Vector2d(1, 1));
        var areaMissingPoint = FixedBoundArea.FromMinMax(new Vector2d(2, 2), new Vector2d(3, 3));

        Assert.True(circle.Contains(new Vector2d(1, 1)));
        Assert.False(circle.Contains(new Vector2d(1, 2)));
        Assert.Equal(FixedBoundArea.FromMinMax(new Vector2d(1, 1), new Vector2d(1, 1)), circle.Bounds);
        Assert.Equal(FixedEnclosureType.Contains, circle.Contains(samePoint));
        Assert.True(circle.Intersects(areaContainingPoint));
        Assert.False(circle.Intersects(areaMissingPoint));
    }

    #endregion

    #region Test: Projection

    [Fact]
    public void ClampPoint_ReturnsOriginalInsideAndSurfacePointOutside()
    {
        var circle = new FixedBoundCircle(new Vector2d(1, 2), new Fixed64(5));

        Assert.Equal(new Vector2d(3, 2), circle.ClampPoint(new Vector2d(3, 2)));
        Assert.Equal(new Vector2d(6, 2), circle.ClampPoint(new Vector2d(20, 2)));
        Assert.Equal(circle.Center, circle.ClampPoint(circle.Center));
    }

    [Fact]
    public void ProjectPoint_ReturnsSurfacePointOrCenterForDegenerateDirection()
    {
        var circle = new FixedBoundCircle(new Vector2d(1, 2), new Fixed64(5));

        Assert.Equal(new Vector2d(1, 7), circle.ProjectPoint(new Vector2d(1, 20)));
        Assert.Equal(circle.Center, circle.ProjectPoint(circle.Center));
    }

    #endregion

    #region Test: Equality

    [Fact]
    public void Equality_SameCircle_ReturnsTrueAndSameHashCode()
    {
        var a = new FixedBoundCircle(new Vector2d(1, 2), new Fixed64(4));
        var b = new FixedBoundCircle(new Vector2d(1, 2), new Fixed64(-4));

        Assert.True(a.Equals(b));
        Assert.True(a == b);
        Assert.False(a != b);
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
    }

    [Fact]
    public void GetHashCode_UsesDeterministicComponentHash()
    {
        var circle = new FixedBoundCircle(new Vector2d(-1, 2), new Fixed64(3));

        Assert.Equal(CombineCircleHash(circle.Center, circle.Radius), circle.GetHashCode());
    }

    [Fact]
    public void Equality_DifferentCircle_ReturnsFalse()
    {
        var a = new FixedBoundCircle(new Vector2d(1, 2), new Fixed64(4));
        var b = new FixedBoundCircle(new Vector2d(1, 3), new Fixed64(4));

        Assert.False(a.Equals(b));
        Assert.False(a == b);
        Assert.True(a != b);
        Assert.False(a.Equals("not a circle"));
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void JsonSerialization_RoundTripMaintainsData()
    {
        var originalValue = new FixedBoundCircle(new Vector2d(1, 2), new Fixed64(4));

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<FixedBoundCircle>(json, jsonOptions);

        Assert.Equal(originalValue, deserializedValue);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_RoundTripMaintainsData()
    {
        var originalValue = new FixedBoundCircle(new Vector2d(1, 2), new Fixed64(4));

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedBoundCircle deserializedValue = MemoryPackSerializer.Deserialize<FixedBoundCircle>(bytes);

        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion

    private static int CombineCircleHash(Vector2d center, Fixed64 radius)
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + center.StateHash;
            hash = (hash * 31) + radius.GetHashCode();
            return hash;
        }
    }
}
