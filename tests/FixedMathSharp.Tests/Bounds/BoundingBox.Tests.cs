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

        #region Test: Serialization

        [Fact]
        public void BoundingBox_NetSerialization_RoundTripMaintainsData()
        {
            BoundingBox originalValue = new(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

            // Serialize the BoundingArea object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, originalValue);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserializedValue = (BoundingBox)formatter.Deserialize(stream);
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
            var deserializedValue = JsonSerializer.Deserialize<BoundingBox>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(originalValue, deserializedValue);
        }

        [Fact]
        public void BoundingBox_MsgPackSerialization_RoundTripMaintainsData()
        {
            BoundingBox originalValue = new(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

            byte[] bytes = MessagePackSerializer.Serialize(originalValue);
            BoundingBox deserializedValue = MessagePackSerializer.Deserialize<BoundingBox>(bytes);

            // Check that deserialized values match the original
            Assert.Equal(originalValue, deserializedValue);
        }

        #endregion
    }
}
