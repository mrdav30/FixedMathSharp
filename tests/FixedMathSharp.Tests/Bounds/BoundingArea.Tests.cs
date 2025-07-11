using MessagePack;

#if NET48_OR_GREATER
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
#endif

#if NET8_0_OR_GREATER
using System.Text.Json;
using System.Text.Json.Serialization;
#endif

using Xunit;

namespace FixedMathSharp.Tests.Bounds
{
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

            // Serialize the BoundingArea object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, originalValue);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserializedValue = (BoundingArea)formatter.Deserialize(stream);
#endif

#if NET8_0_OR_GREATER
            var jsonOptions = new JsonSerializerOptions
            {
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
                ReferenceHandler = ReferenceHandler.IgnoreCycles,
                IncludeFields = true,
                IgnoreReadOnlyProperties = true
            };
            var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
            var deserializedValue = JsonSerializer.Deserialize<BoundingArea>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(originalValue, deserializedValue);
        }

        [Fact]
        public void BoundingArea_MsgPackSerialization_RoundTripMaintainsData()
        {
            BoundingArea originalValue = new(
                new Vector3d(1, 2, 3),
                new Vector3d(4, 5, 6)
            );

            byte[] bytes = MessagePackSerializer.Serialize(originalValue);
            BoundingArea deserializedValue = MessagePackSerializer.Deserialize<BoundingArea>(bytes);

            // Check that deserialized values match the original
            Assert.Equal(originalValue, deserializedValue);
        }

        #endregion
    }
}
