using FixedMathSharp.Bounds;
using MemoryPack;
using System;
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
    public void DerivedMetadata_FullDomainAndRawUnitSpans_RemainsExactOrFailsHonestly()
    {
        Fixed64 sameSignMin = Fixed64.FromRaw(long.MaxValue - 3);
        Fixed64 sameSignMax = Fixed64.FromRaw(long.MaxValue - 1);
        var sameSign = FixedBoundArea.FromMinMax(
            new Vector2d(sameSignMin, Fixed64.FromRaw(1)),
            new Vector2d(sameSignMax, Fixed64.FromRaw(2)));

        Assert.Equal(
            new Vector2d(Fixed64.FromRaw(long.MaxValue - 2), Fixed64.FromRaw(2)),
            sameSign.Center);
        Assert.Equal(new Vector2d(Fixed64.FromRaw(2), Fixed64.MinIncrement), sameSign.Size);
        Assert.Equal(new Vector2d(Fixed64.MinIncrement, Fixed64.MinIncrement), sameSign.Scope);

        Fixed64 quarterDomain = Fixed64.FromRaw(1L << 62);
        var wide = FixedBoundArea.FromMinMax(
            new Vector2d(-quarterDomain, Fixed64.Zero),
            new Vector2d(quarterDomain, Fixed64.One));

        Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.Half), wide.Center);
        Assert.Equal(new Vector2d(quarterDomain, Fixed64.Half), wide.Scope);
        Assert.Throws<OverflowException>(() => _ = wide.Size);

        var fullDomain = FixedBoundArea.FromMinMax(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.One));
        fullDomain.Center = fullDomain.Center;

        Assert.Equal(Fixed64.Zero, fullDomain.Center.X);
        Assert.Throws<OverflowException>(() => _ = fullDomain.Scope);
    }

    [Fact]
    public void Center_Setter_RawUnitSpan_PreservesRequestedCenterWithConservativeScope()
    {
        var area = FixedBoundArea.FromMinMax(
            new Vector2d(Fixed64.FromRaw(1), Fixed64.FromRaw(3)),
            new Vector2d(Fixed64.FromRaw(2), Fixed64.FromRaw(6)));

        area.Center = new Vector2d(Fixed64.FromRaw(11), Fixed64.FromRaw(20));

        Assert.Equal(
            new Vector2d(Fixed64.FromRaw(10), Fixed64.FromRaw(18)),
            area.Min);
        Assert.Equal(
            new Vector2d(Fixed64.FromRaw(12), Fixed64.FromRaw(22)),
            area.Max);
        Assert.Equal(new Vector2d(Fixed64.FromRaw(11), Fixed64.FromRaw(20)), area.Center);
    }

    [Fact]
    public void CenterAndSizeMutators_UnrepresentableEndpoint_ThrowWithoutMutation()
    {
        var area = FixedBoundArea.FromCenterAndSize(Vector2d.Zero, new Vector2d(4, 6));
        FixedBoundArea original = area;

        Assert.Throws<OverflowException>(() => area.Center = new Vector2d(Fixed64.MaxValue, Fixed64.Zero));
        Assert.Equal(original, area);

        area = FixedBoundArea.FromCenterAndSize(
            new Vector2d(Fixed64.MaxValue - Fixed64.One, Fixed64.Zero),
            new Vector2d(2, 2));
        original = area;

        Assert.Throws<OverflowException>(() => area.Size = new Vector2d(4, 2));
        Assert.Equal(original, area);
    }

    [Fact]
    public void CenteredFactories_UnrepresentableEndpointOrMagnitude_Throw()
    {
        Assert.Throws<OverflowException>(() => FixedBoundArea.FromCenterAndSize(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(2, 2)));
        Assert.Throws<OverflowException>(() => FixedBoundArea.FromCenterAndSize(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(2, 2)));
        Assert.Throws<OverflowException>(() => FixedBoundArea.FromCenterAndScope(
            Vector2d.Zero,
            new Vector2d(Fixed64.MinValue, Fixed64.One)));

        FixedBoundArea rawUnit = FixedBoundArea.FromCenterAndSize(
            new Vector2d(Fixed64.FromRaw(10), Fixed64.Zero),
            new Vector2d(Fixed64.MinIncrement, Fixed64.MinValue));

        Assert.Equal(Fixed64.FromRaw(9), rawUnit.Min.X);
        Assert.Equal(Fixed64.FromRaw(11), rawUnit.Max.X);
        Assert.Equal(Fixed64.FromRaw(1L << 62), rawUnit.Scope.Y);
        Assert.Throws<OverflowException>(() => _ = rawUnit.Size);
    }

    [Fact]
    public void ClippedCenteredFactories_ExplicitlyIntersectWithScalarDomain()
    {
        FixedBoundArea fromSize = FixedBoundArea.FromCenterAndSizeClippedToDomain(
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(2, 2));
        FixedBoundArea fromScope = FixedBoundArea.FromCenterAndScopeClippedToDomain(
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(-Fixed64.One, -Fixed64.One));

        var expected = FixedBoundArea.FromMinMax(
            new Vector2d(Fixed64.MaxValue - Fixed64.One, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue + Fixed64.One));
        Assert.Equal(expected, fromSize);
        Assert.Equal(expected, fromScope);
    }

    [Fact]
    public void CenterOffsetFactory_PreservesAsymmetryAndClipsOnlyFinalEndpoints()
    {
        FixedBoundArea area = FixedBoundArea.FromCenterAndOffsetsClippedToDomain(
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(-3, -2),
            new Vector2d(5, 7));

        Assert.Equal(
            new Vector2d(Fixed64.MaxValue - (Fixed64)3, Fixed64.MinValue),
            area.Min);
        Assert.Equal(
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue + (Fixed64)7),
            area.Max);
        Assert.Throws<ArgumentException>(() =>
            FixedBoundArea.FromCenterAndOffsetsClippedToDomain(
                Vector2d.Zero,
                new Vector2d(2, 0),
                new Vector2d(1, 0)));
        Assert.Throws<ArgumentException>(() =>
            FixedBoundArea.FromCenterAndOffsetsClippedToDomain(
                Vector2d.Zero,
                new Vector2d(0, 2),
                new Vector2d(0, 1)));
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
    public void Contains_Circle_ReturnsContainmentClassification()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -2), new Vector2d(2, 2));
        var contained = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var crossing = new FixedBoundCircle(new Vector2d(2, 0), Fixed64.One);
        var touching = new FixedBoundCircle(new Vector2d(3, 0), Fixed64.One);
        var disjoint = new FixedBoundCircle(new Vector2d(4, 0), Fixed64.One);

        Assert.Equal(FixedEnclosureType.Contains, area.Contains(contained));
        Assert.Equal(FixedEnclosureType.Intersects, area.Contains(crossing));
        Assert.Equal(FixedEnclosureType.Intersects, area.Contains(touching));
        Assert.Equal(FixedEnclosureType.Disjoint, area.Contains(disjoint));
    }

    [Fact]
    public void Contains_CircleNearScalarLimit_DoesNotUseSaturatedDerivedBounds()
    {
        Fixed64 centerX = Fixed64.MaxValue - Fixed64.One;
        var area = FixedBoundArea.FromMinMax(
            new Vector2d(Fixed64.Zero, (Fixed64)(-20)),
            new Vector2d(Fixed64.MaxValue, (Fixed64)20));
        var crossing = new FixedBoundCircle(new Vector2d(centerX, Fixed64.Zero), (Fixed64)10);

        Assert.Equal(FixedEnclosureType.Intersects, area.Contains(crossing));
    }

    [Fact]
    public void Contains_CircleCrossingEachExtent_RemainsIntersection()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-10, -10), new Vector2d(10, 10));

        Assert.Equal(
            FixedEnclosureType.Intersects,
            area.Contains(new FixedBoundCircle(new Vector2d(-11, 0), (Fixed64)2)));
        Assert.Equal(
            FixedEnclosureType.Intersects,
            area.Contains(new FixedBoundCircle(new Vector2d(11, 0), (Fixed64)2)));
        Assert.Equal(
            FixedEnclosureType.Intersects,
            area.Contains(new FixedBoundCircle(new Vector2d(-9, 0), (Fixed64)2)));
        Assert.Equal(
            FixedEnclosureType.Intersects,
            area.Contains(new FixedBoundCircle(new Vector2d(9, 0), (Fixed64)2)));
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
    public void Intersects_Circle_IsBoundaryInclusive()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -2), new Vector2d(2, 2));
        var contained = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var touching = new FixedBoundCircle(new Vector2d(3, 0), Fixed64.One);
        var disjoint = new FixedBoundCircle(new Vector2d(4, 0), Fixed64.One);

        Assert.True(area.Intersects(contained));
        Assert.True(area.Intersects(touching));
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

    [Fact]
    public void IntersectsStrict_Circle_RequiresPositiveAreaOverlap()
    {
        var area = FixedBoundArea.FromMinMax(new Vector2d(-2, -2), new Vector2d(2, 2));
        var touching = new FixedBoundCircle(new Vector2d(3, 0), Fixed64.One);
        var overlapping = new FixedBoundCircle(new Vector2d(2, 0), Fixed64.One);
        var contained = new FixedBoundCircle(Vector2d.Zero, Fixed64.One);
        var zeroRadiusInside = new FixedBoundCircle(Vector2d.Zero, Fixed64.Zero);

        Assert.True(area.Intersects(touching));
        Assert.False(area.IntersectsStrict(touching));
        Assert.True(area.IntersectsStrict(overlapping));
        Assert.True(area.IntersectsStrict(contained));
        Assert.False(area.IntersectsStrict(zeroRadiusInside));
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
        Assert.True(a.Equals((object)b));
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

    [Fact]
    public void JsonSerialization_FullDomainState_PreservesExactDerivedCenter()
    {
        var originalValue = FixedBoundArea.FromMinMax(
            new Vector2d(Fixed64.FromRaw(long.MaxValue - 3), Fixed64.MinValue),
            new Vector2d(Fixed64.FromRaw(long.MaxValue - 1), Fixed64.MaxValue));

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(originalValue);
        FixedBoundArea deserializedValue = JsonSerializer.Deserialize<FixedBoundArea>(json);

        Assert.Equal(originalValue, deserializedValue);
        Assert.Equal(
            new Vector2d(Fixed64.FromRaw(long.MaxValue - 2), Fixed64.Zero),
            deserializedValue.Center);
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
