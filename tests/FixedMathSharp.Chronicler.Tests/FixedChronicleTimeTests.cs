using System;
using Chronicler.Timing;
using FixedMathSharp.Tests;
using Xunit;

namespace FixedMathSharp.Chronicler.Tests;

public sealed class FixedChronicleTimeTests
{
    [Theory]
    [InlineData(long.MinValue, int.MinValue, 0U)]
    [InlineData(long.MinValue + 1, int.MinValue, 1U)]
    [InlineData(-4294967297L, -2L, uint.MaxValue)]
    [InlineData(-4294967296L, -1L, 0U)]
    [InlineData(-2147483648L, -1L, 2147483648U)]
    [InlineData(-1L, -1L, uint.MaxValue)]
    [InlineData(0L, 0L, 0U)]
    [InlineData(1L, 0L, 1U)]
    [InlineData(4294967295L, 0L, uint.MaxValue)]
    [InlineData(4294967296L, 1L, 0U)]
    [InlineData(long.MaxValue - 1, int.MaxValue, uint.MaxValue - 1)]
    [InlineData(long.MaxValue, int.MaxValue, uint.MaxValue)]
    public void Conversions_PreserveExactCanonicalComponents(long raw, long seconds, uint fraction)
    {
        var duration = new ChronicleDuration(seconds, fraction);
        Assert.Equal(duration, FixedChronicleTime.FromFixed64(Fixed64.FromRaw(raw)));
        Assert.True(FixedChronicleTime.TryToFixed64(duration, out Fixed64 converted));
        Assert.Equal(raw, converted.m_rawValue);
        Assert.Equal(raw, FixedChronicleTime.ToFixed64(duration).m_rawValue);
    }

    [Fact]
    public void Conversions_RoundTripSeededRawPayloads()
    {
        ulong bits = 0x6A09E667F3BCC909UL;
        for (int i = 0; i < 4096; i++)
        {
            bits = unchecked(bits * 6364136223846793005UL + 1442695040888963407UL);
            long raw = unchecked((long)bits);
            ChronicleDuration duration = FixedChronicleTime.FromFixed64(Fixed64.FromRaw(raw));
            Assert.True(FixedChronicleTime.TryToFixed64(duration, out Fixed64 converted));
            Assert.Equal(raw, converted.m_rawValue);
            Assert.Equal(raw, FixedChronicleTime.ToFixed64(duration).m_rawValue);
        }
    }

    [Theory]
    [InlineData((long)int.MinValue - 1, uint.MaxValue)]
    [InlineData((long)int.MaxValue + 1, 0U)]
    [InlineData(long.MinValue, 0U)]
    [InlineData(long.MaxValue, uint.MaxValue)]
    public void Conversions_RejectOutOfRangeDurationsWithoutSaturation(long seconds, uint fraction)
    {
        var duration = new ChronicleDuration(seconds, fraction);
        Assert.False(FixedChronicleTime.TryToFixed64(duration, out Fixed64 result));
        Assert.Equal(Fixed64.Zero, result);
        Assert.Throws<OverflowException>(() => FixedChronicleTime.ToFixed64(duration));
    }

    [Fact]
    public void LongRunningTimeline_NarrowsTheDifferenceNotTheEndpoints()
    {
        // Executable counterpart of the companion README's complete example.
        var start = new ChronicleTimestamp(3_155_760_000, 0);
        var end = start + FixedChronicleTime.FromFixed64(Fixed64.FromRaw(0x40000000));
        Assert.Equal(Fixed64.FromRaw(0x40000000), FixedChronicleTime.ToFixed64(end - start));
        Assert.Equal(Fixed64.FromRaw(-0x40000000), FixedChronicleTime.ToFixed64(start - end));
        Assert.False(FixedChronicleTime.TryToFixed64(end - ChronicleTimestamp.Zero, out Fixed64 result));
        Assert.Equal(Fixed64.Zero, result);
        Assert.Throws<OverflowException>(() => FixedChronicleTime.ToFixed64(end - ChronicleTimestamp.Zero));
    }

    [Theory]
    [InlineData(0L, 1L, 0L)]
    [InlineData(1L, 1L, 1L)]
    [InlineData(1L, 2L, 0L)]
    [InlineData(134217727L, 134217728L, 0L)]
    [InlineData(134217728L, 134217728L, 1L)]
    [InlineData(134217729L, 134217728L, 1L)]
    [InlineData(288230376151711744L, 134217728L, 2147483648L)]
    [InlineData(long.MaxValue, 1L, long.MaxValue)]
    [InlineData(long.MaxValue, long.MaxValue, 1L)]
    // A rounded 1/30 second is 143165577 raw units: 1 second contains only 29 complete steps.
    [InlineData(4294967296L, 143165577L, 29L)]
    [InlineData(4294967309L, 143165577L, 29L)]
    [InlineData(4294967310L, 143165577L, 30L)]
    [InlineData(4294967311L, 143165577L, 30L)]
    // A rounded 1/60 second is slightly short: 60 complete steps fit within one second.
    [InlineData(4294967279L, 71582788L, 59L)]
    [InlineData(4294967280L, 71582788L, 60L)]
    [InlineData(4294967296L, 71582788L, 60L)]
    public void FrameCount_CountsCompleteStepsUsingRawIntegerDivision(long duration, long step, long expected)
    {
        Assert.Equal(expected, FixedChronicleTime.GetFrameCountForDuration(
            Fixed64.FromRaw(duration), Fixed64.FromRaw(step)));
    }

    [Theory]
    [InlineData(-1L, 1L, "duration")]
    [InlineData(long.MinValue, 1L, "duration")]
    [InlineData(1L, 0L, "stepDuration")]
    [InlineData(0L, 0L, "stepDuration")]
    [InlineData(1L, -1L, "stepDuration")]
    [InlineData(1L, long.MinValue, "stepDuration")]
    public void FrameCount_RejectsInvalidInputs(long duration, long step, string parameter)
    {
        Assert.Throws<ArgumentOutOfRangeException>(parameter, () =>
            FixedChronicleTime.GetFrameCountForDuration(Fixed64.FromRaw(duration), Fixed64.FromRaw(step)));
    }

    [Fact]
    public void SuccessfulConversionsAndCounting_AllocateNothingWhenWarmed()
    {
        long sum = 0;
        int convertedCount = 0;
        Action operation = () =>
        {
            sum = 0;
            convertedCount = 0;
            for (int i = 1; i <= 1024; i++)
            {
                var duration = FixedChronicleTime.FromFixed64(Fixed64.FromRaw(i));
                if (FixedChronicleTime.TryToFixed64(duration, out Fixed64 value))
                {
                    convertedCount++;
                    sum += FixedChronicleTime.GetFrameCountForDuration(value, Fixed64.FromRaw(1));
                    sum += FixedChronicleTime.ToFixed64(duration).m_rawValue;
                }
            }
        };

        Assert.Equal(0, FixedMathTestHelper.MeasureWarmedAllocations(operation));
        Assert.Equal(1024, convertedCount);
        Assert.Equal(1049600L, sum);
    }
}
