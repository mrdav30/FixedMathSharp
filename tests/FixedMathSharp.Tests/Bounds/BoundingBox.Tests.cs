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
            Assert.Equal(size, box.Size);
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
    }
}
