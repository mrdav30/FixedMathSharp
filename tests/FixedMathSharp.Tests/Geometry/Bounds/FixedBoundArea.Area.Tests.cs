using System;
using System.Numerics;
using System.Text.Json;
using FixedMathSharp.Geometry;
using MemoryPack;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedBoundAreaAreaTests
{
    [Theory]
    [InlineData(0UL, 0UL, 0L)]
    [InlineData(1UL, 0x7FFF_FFFFUL, 0L)]
    [InlineData(1UL, 0x8000_0000UL, 0L)]
    [InlineData(1UL, 0x8000_0001UL, 1L)]
    [InlineData(1UL, 0x1_8000_0000UL, 2L)]
    [InlineData(1UL, 0x2_8000_0000UL, 2L)]
    [InlineData(0x1_0000_0000UL, 0x1_0000_0000UL, 0x1_0000_0000L)]
    [InlineData(0x7FFF_FFFF_FFFF_FFFFUL, 0x1_0000_0000UL, long.MaxValue)]
    [InlineData(ulong.MaxValue, 0UL, 0L)]
    [InlineData(0UL, ulong.MaxValue, 0L)]
    public void Area_RoundsExactSpansOnceToNearestEven(ulong width, ulong height, long expectedRaw)
    {
        FixedBoundArea area = FromRawSpans(width, height);

        Assert.Equal(expectedRaw, area.Area.m_rawValue);
        AssertAreaOracle(area);
    }

    [Fact]
    public void Area_RejectsOverflowBeforeNarrowingAndAfterRoundingCarry()
    {
        FixedBoundArea fullDomain = FromRawSpans(ulong.MaxValue, ulong.MaxValue);
        // Exact raw result is long.MaxValue + 1/2: nearest-even must carry into an unrepresentable value.
        FixedBoundArea roundingCarry = FromRawSpans(ulong.MaxValue, 0x8000_0000UL);

        Assert.Throws<OverflowException>(() => fullDomain.Area);
        Assert.Throws<OverflowException>(() => roundingCarry.Area);
        AssertAreaOracle(fullDomain);
        AssertAreaOracle(roundingCarry);
    }

    [Fact]
    public void Area_DoesNotRequireRepresentableSizeAndPreservesNormalizedState()
    {
        FixedBoundArea area = FromRawSpans(ulong.MaxValue, 1UL);
        FixedBoundArea normalized = FixedBoundArea.FromMinMax(area.Max, area.Min);
        FixedBoundArea line = FromRawSpans(ulong.MaxValue, 0UL);

        Assert.Throws<OverflowException>(() => area.Size);
        Assert.Equal(Fixed64.FromRaw(0x1_0000_0000L), area.Area);
        Assert.Equal(area.Area, normalized.Area);
        Assert.Equal(Fixed64.Zero, line.Area);
        Assert.Equal(Fixed64.Zero, default(FixedBoundArea).Area);
        Assert.Equal(area.State.Min, normalized.State.Min);
        Assert.Equal(area.State.Max, normalized.State.Max);
    }

    [Fact]
    public void GetAreaExpansionCost_SubtractsBeforeRoundingOrClamping()
    {
        FixedBoundArea unit = FixedBoundArea.FromMinMax(Vector2d.Zero, Vector2d.One);
        FixedBoundArea expanded = FixedBoundArea.FromMinMax(Vector2d.Zero, new Vector2d(3, 3));
        FixedBoundArea almostUnit = FromRawSpans(0xFFFF_FFFFUL, 0xFFFF_FFFFUL);
        FixedBoundArea slightlyLarger = FromRawSpans(0x1_0000_0001UL, 0x1_0000_0001UL);
        FixedBoundArea large = FromRawSpans(ulong.MaxValue, 0x1_0000_0000UL);
        FixedBoundArea larger = FromRawSpans(ulong.MaxValue, 0x1_0000_0002UL);
        FixedBoundArea fullDomain = FromRawSpans(ulong.MaxValue, ulong.MaxValue);

        Assert.Equal(0L, unit.GetAreaExpansionCost(default));
        Assert.Equal(0L, unit.GetAreaExpansionCost(unit));
        Assert.Equal(8L, unit.GetAreaExpansionCost(expanded));
        // Union/current products straddle 2^64; subtracting their integer parts would incorrectly return one.
        Assert.Equal(0L, almostUnit.GetAreaExpansionCost(slightlyLarger));
        Assert.Throws<OverflowException>(() => large.Area);
        Assert.Throws<OverflowException>(() => larger.Area);
        Assert.Equal(1L, large.GetAreaExpansionCost(larger));
        Assert.Equal(long.MaxValue, default(FixedBoundArea).GetAreaExpansionCost(fullDomain));
        Assert.Equal(0L, fullDomain.GetAreaExpansionCost(default));

        AssertCostOracle(unit, expanded);
        AssertCostOracle(almostUnit, slightlyLarger);
        AssertCostOracle(large, larger);
        AssertCostOracle(default, fullDomain);
        AssertCostOracle(fullDomain, default);
    }

    [Fact]
    public void AreaAndExpansionCost_MatchBigIntegerOracleAcrossEndpointDomain()
    {
        var random = new System.Random(27783);
        var bytes = new byte[8];
        for (int i = 0; i < 512; i++)
        {
            ulong width = NextUInt64(random, bytes);
            ulong height = NextUInt64(random, bytes);
            if ((i & 1) == 0)
            {
                width &= 0xFF_FFFF_FFFFUL;
                height &= 0xFF_FFFF_FFFFUL;
            }

            FixedBoundArea area = FromRawSpans(width, height);
            FixedBoundArea other = FixedBoundArea.FromMinMax(
                new Vector2d(Fixed64.FromRaw((long)NextUInt64(random, bytes)), Fixed64.FromRaw((long)NextUInt64(random, bytes))),
                new Vector2d(Fixed64.FromRaw((long)NextUInt64(random, bytes)), Fixed64.FromRaw((long)NextUInt64(random, bytes))));

            AssertAreaOracle(area);
            AssertAreaOracle(other);
            AssertCostOracle(area, other);
            AssertCostOracle(other, area);
        }
    }

    [Fact]
    public void AreaAndExpansionCost_WarmReadsAllocateNothingAndDoNotMutateBounds()
    {
        FixedBoundArea area = FixedBoundArea.FromMinMax(Vector2d.Zero, new Vector2d(2, 3));
        FixedBoundArea other = FixedBoundArea.FromMinMax(new Vector2d(-1, -1), new Vector2d(4, 5));
        FixedBoundArea before = area;
        long checksum = 0L;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            checksum = 0L;
            for (int i = 0; i < 10000; i++)
                checksum += area.Area.m_rawValue + area.GetAreaExpansionCost(other);
        });

        Assert.Equal(0L, allocated);
        Assert.True(checksum > 0L);
        Assert.Equal(before, area);
        Assert.Equal((Fixed64)6, area.Area);
        Assert.Equal(24L, area.GetAreaExpansionCost(other));
    }

    [Fact]
    public void JsonSerialization_StillWritesOnlyStateAndCanRestoreOverflowingArea()
    {
        FixedBoundArea original = FromRawSpans(ulong.MaxValue, ulong.MaxValue);

        byte[] bytes = JsonSerializer.SerializeToUtf8Bytes(original);
        using JsonDocument json = JsonDocument.Parse(bytes);
        JsonElement.ObjectEnumerator properties = json.RootElement.EnumerateObject();
        Assert.True(properties.MoveNext());
        Assert.Equal("State", properties.Current.Name);
        Assert.False(properties.MoveNext());

        FixedBoundArea restored = JsonSerializer.Deserialize<FixedBoundArea>(bytes);
        Assert.Equal(original, restored);
        Assert.Equal(0L, restored.GetAreaExpansionCost(original));
        Assert.Throws<OverflowException>(() => restored.Area);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_PreservesEndpointPayloadAndCanRestoreOverflowingArea()
    {
        FixedBoundArea original = FromRawSpans(ulong.MaxValue, ulong.MaxValue);

        byte[] bytes = MemoryPackSerializer.Serialize(original);
        byte[] stateBytes = MemoryPackSerializer.Serialize(original.State);
        Assert.Equal(32, bytes.Length);
        Assert.Equal(stateBytes, bytes);

        FixedBoundArea restored = MemoryPackSerializer.Deserialize<FixedBoundArea>(bytes);
        Assert.Equal(original, restored);
        Assert.Equal(0L, restored.GetAreaExpansionCost(original));
        Assert.Throws<OverflowException>(() => restored.Area);
    }
#endif

    private static FixedBoundArea FromRawSpans(ulong width, ulong height)
    {
        const ulong minimumBits = 0x8000_0000_0000_0000UL;
        return FixedBoundArea.FromMinMax(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(
                Fixed64.FromRaw(unchecked((long)(minimumBits + width))),
                Fixed64.FromRaw(unchecked((long)(minimumBits + height)))));
    }

    private static ulong NextUInt64(System.Random random, byte[] bytes)
    {
        random.NextBytes(bytes);
        return BitConverter.ToUInt64(bytes, 0);
    }

    private static BigInteger ExactArea(FixedBoundArea area)
    {
        BigInteger width = (BigInteger)area.Max.X.m_rawValue - area.Min.X.m_rawValue;
        BigInteger height = (BigInteger)area.Max.Y.m_rawValue - area.Min.Y.m_rawValue;
        return width * height;
    }

    private static void AssertAreaOracle(FixedBoundArea area)
    {
        BigInteger scale = BigInteger.One << 32;
        BigInteger expected = BigInteger.DivRem(ExactArea(area), scale, out BigInteger remainder);
        int halfComparison = (remainder * 2).CompareTo(scale);
        if (halfComparison > 0 || (halfComparison == 0 && !expected.IsEven))
            expected++;

        if (expected > long.MaxValue)
            Assert.Throws<OverflowException>(() => area.Area);
        else
            Assert.Equal((long)expected, area.Area.m_rawValue);
    }

    private static void AssertCostOracle(FixedBoundArea area, FixedBoundArea other)
    {
        BigInteger minX = Math.Min(area.Min.X.m_rawValue, other.Min.X.m_rawValue);
        BigInteger minY = Math.Min(area.Min.Y.m_rawValue, other.Min.Y.m_rawValue);
        BigInteger maxX = Math.Max(area.Max.X.m_rawValue, other.Max.X.m_rawValue);
        BigInteger maxY = Math.Max(area.Max.Y.m_rawValue, other.Max.Y.m_rawValue);
        BigInteger expected = ((maxX - minX) * (maxY - minY) - ExactArea(area)) / (BigInteger.One << 64);

        Assert.Equal((long)BigInteger.Min(expected, long.MaxValue), area.GetAreaExpansionCost(other));
    }
}
