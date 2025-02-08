#if NET48_OR_GREATER
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
#endif

#if NET8_0_OR_GREATER
using System.Text.Json;
using System.Text.Json.Serialization;

#endif

using Xunit;

namespace FixedMathSharp.Tests
{
    public class FixedRangeTests
    {
        [Fact]
        public void FixedRange_InitializesCorrectly()
        {
            var min = new Fixed64(-10);
            var max = new Fixed64(10);
            var range = new FixedRange(min, max);

            Assert.Equal(min, range.Min);
            Assert.Equal(max, range.Max);
        }

        [Fact]
        public void FixedRange_Constructor_EnforcesOrder()
        {
            var range = new FixedRange(new Fixed64(10), new Fixed64(-10));

            Assert.Equal(new Fixed64(-10), range.Min);
            Assert.Equal(new Fixed64(10), range.Max);
        }

        [Fact]
        public void FixedRange_Length_ComputesCorrectly()
        {
            var range = new FixedRange(new Fixed64(-5), new Fixed64(15));
            Assert.Equal(new Fixed64(20), range.Length); // Length = 15 - (-5) = 20
        }

        [Fact]
        public void FixedRange_MidPoint_ComputesCorrectly()
        {
            var range = new FixedRange(new Fixed64(-5), new Fixed64(15));
            Assert.Equal(new Fixed64(5), range.MidPoint); // Midpoint = (-5 + 15) / 2 = 5
        }

        [Fact]
        public void FixedRange_InRange_Fixed64Value_ReturnsTrue()
        {
            var range = new FixedRange(new Fixed64(0), new Fixed64(10));
            Assert.True(range.InRange(new Fixed64(5))); // 5 is in [0, 10)
        }

        [Fact]
        public void FixedRange_InRange_Fixed64Value_ReturnsFalse()
        {
            var range = new FixedRange(new Fixed64(0), new Fixed64(10));
            Assert.False(range.InRange(new Fixed64(10))); // 10 is not included in [0, 10)
        }

        [Fact]
        public void FixedRange_InRange_IntValue_ReturnsTrue()
        {
            var range = new FixedRange(new Fixed64(0), new Fixed64(10));
            Assert.True(range.InRange((Fixed64)5)); // 5 is in [0, 10)
        }

        [Fact]
        public void FixedRange_InRange_FloatValue_ReturnsTrue()
        {
            var range = new FixedRange(new Fixed64(0), new Fixed64(10));
            Assert.True(range.InRange((Fixed64)5.5f)); // 5.5 is in [0, 10)
        }

        [Fact]
        public void FixedRange_Overlaps_ReturnsTrue()
        {
            var range1 = new FixedRange(new Fixed64(0), new Fixed64(10));
            var range2 = new FixedRange(new Fixed64(5), new Fixed64(15));
            Assert.True(range1.Overlaps(range2)); // Ranges [0, 10) and [5, 15) overlap
        }

        [Fact]
        public void FixedRange_Overlaps_ReturnsFalse()
        {
            var range1 = new FixedRange(new Fixed64(0), new Fixed64(10));
            var range2 = new FixedRange(new Fixed64(10), new Fixed64(20));
            Assert.False(range1.Overlaps(range2)); // No overlap between [0, 10) and [10, 20)
        }

        [Fact]
        public void FixedRange_AddInPlace_AddsToRangeCorrectly()
        {
            var range = new FixedRange(new Fixed64(0), new Fixed64(10));
            range.AddInPlace(new Fixed64(5));

            Assert.Equal(new Fixed64(5), range.Min);
            Assert.Equal(new Fixed64(15), range.Max); // Range should now be [5, 15)
        }

        [Fact]
        public void FixedRange_GetDirection_ReturnsCorrectDirection()
        {
            var range1 = new FixedRange(new Fixed64(0), new Fixed64(5));
            var range2 = new FixedRange(new Fixed64(10), new Fixed64(15));

            Fixed64? sign;
            var result = FixedRange.GetDirection(range1, range2, out sign);
            Assert.True(result);
            Assert.Equal(-Fixed64.One, sign); // range1 is to the left of range2
        }

        [Fact]
        public void FixedRange_ComputeOverlapDepth_ComputesCorrectly()
        {
            var range1 = new FixedRange(new Fixed64(0), new Fixed64(10));
            var range2 = new FixedRange(new Fixed64(5), new Fixed64(15));

            var overlapDepth = FixedRange.ComputeOverlapDepth(range1, range2);
            Assert.Equal(new Fixed64(5), overlapDepth); // Overlap depth is 5
        }

        [Fact]
        public void FixedRange_Serialization_RoundTripMaintainsData()
        {
            var originalRange = new FixedRange(new Fixed64(-10), new Fixed64(10));

            // Serialize the FixedRange object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, originalRange);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserializedRange = (FixedRange)formatter.Deserialize(stream);
#endif

#if NET8_0_OR_GREATER
            var jsonOptions = new JsonSerializerOptions
            {
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
                ReferenceHandler = ReferenceHandler.IgnoreCycles,
                IncludeFields = true,
                IgnoreReadOnlyProperties = true
            };
            var json = JsonSerializer.SerializeToUtf8Bytes(originalRange, jsonOptions);
            var deserializedRange = JsonSerializer.Deserialize<FixedRange>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(originalRange.Min, deserializedRange.Min);
            Assert.Equal(originalRange.Max, deserializedRange.Max);
        }

        [Fact]
        public void FixedRange_EqualMinMax_DoesNotOverlapWithAnyOtherRange()
        {
            var pointRange = new FixedRange(new Fixed64(5), new Fixed64(5)); // Zero-length range at 5
            var range = new FixedRange(new Fixed64(0), new Fixed64(4)); // Does not contain 5

            Assert.False(pointRange.Overlaps(range)); // Point range (5-5) should not overlap with (0-4)
        }

        [Fact]
        public void FixedRange_EqualMinMax_ContainedInLargerRange()
        {
            var range1 = new FixedRange(new Fixed64(5), new Fixed64(5)); // Zero-length range
            var range2 = new FixedRange(new Fixed64(0), new Fixed64(10));

            Assert.True(range2.Overlaps(range1)); // Zero-length range (5-5) is contained within (0-10)
        }


        [Fact]
        public void FixedRange_SmallRanges_OverlapCorrectly()
        {
            var smallRange1 = new FixedRange(new Fixed64(0.0001), new Fixed64(0.0002));
            var smallRange2 = new FixedRange(new Fixed64(0.00015), new Fixed64(0.00025));

            Assert.True(smallRange1.Overlaps(smallRange2));
        }


        [Fact]
        public void FixedRange_LargeRanges_OverlapCorrectly()
        {
            var largeRange1 = new FixedRange(new Fixed64(long.MinValue), new Fixed64(0));
            var largeRange2 = new FixedRange(new Fixed64(-1000), new Fixed64(long.MaxValue));

            Assert.True(largeRange1.Overlaps(largeRange2));
        }
    }
}
