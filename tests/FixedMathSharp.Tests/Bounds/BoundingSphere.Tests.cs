using Xunit;

namespace FixedMathSharp.Tests.Bounds
{
    public class BoundingSphereTests
    {
        #region Test: Constructor and Property

        [Fact]
        public void Constructor_AssignsValuesCorrectly()
        {
            var center = new Vector3d(1, 2, 3);
            var radius = new Fixed64(5);

            var sphere = new BoundingSphere(center, radius);

            Assert.Equal(center, sphere.Center);
            Assert.Equal(radius, sphere.Radius);
            Assert.Equal(new Vector3d(-4, -3, -2), sphere.Min);
            Assert.Equal(new Vector3d(6, 7, 8), sphere.Max);
        }

        #endregion

        #region Test: Containment

        [Fact]
        public void Contains_PointInside_ReturnsTrue()
        {
            var sphere = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var point = new Vector3d(2, 2, 2);

            Assert.True(sphere.Contains(point));
        }

        [Fact]
        public void Contains_PointOutside_ReturnsFalse()
        {
            var sphere = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var point = new Vector3d(6, 0, 0);

            Assert.False(sphere.Contains(point));
        }

        [Fact]
        public void Contains_PointOnSurface_ReturnsTrue()
        {
            var sphere = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var point = new Vector3d(5, 0, 0);

            Assert.True(sphere.Contains(point));
        }

        #endregion

        #region Test: Intersection

        [Fact]
        public void Intersects_WithOverlappingSphere_ReturnsTrue()
        {
            var sphere1 = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var sphere2 = new BoundingSphere(new Vector3d(3, 0, 0), new Fixed64(4));

            Assert.True(sphere1.Intersects(sphere2));
        }

        [Fact]
        public void Intersects_WithNonOverlappingSphere_ReturnsFalse()
        {
            var sphere1 = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var sphere2 = new BoundingSphere(new Vector3d(11, 0, 0), new Fixed64(4));

            Assert.False(sphere1.Intersects(sphere2));
        }

        [Fact]
        public void Intersects_WithBoundingBox_ReturnsTrue()
        {
            var sphere = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var box = new BoundingBox(new Vector3d(-3, -3, -3), new Vector3d(3, 3, 3));

            Assert.True(sphere.Intersects(box));
        }

        [Fact]
        public void Intersects_WithBoundingArea_ReturnsTrue()
        {
            var sphere = new BoundingSphere(new Vector3d(2, 2, 0), new Fixed64(3));
            var area = new BoundingArea(new Vector3d(0, 0, 0), new Vector3d(4, 4, 0));

            Assert.True(sphere.Intersects(area));
        }

        #endregion

        #region Test: Distance to Surface

        [Fact]
        public void DistanceToSurface_PointOutside_ReturnsPositiveDistance()
        {
            var sphere = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var point = new Vector3d(10, 0, 0);

            Assert.Equal(new Fixed64(5), sphere.DistanceToSurface(point));
        }

        [Fact]
        public void DistanceToSurface_PointOnSurface_ReturnsZero()
        {
            var sphere = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var point = new Vector3d(5, 0, 0);

            Assert.Equal(Fixed64.Zero, sphere.DistanceToSurface(point));
        }

        [Fact]
        public void DistanceToSurface_PointInside_ReturnsNegativeDistance()
        {
            var sphere = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var point = new Vector3d(3, 0, 0);

            Assert.Equal(new Fixed64(-2), sphere.DistanceToSurface(point));
        }

        #endregion

        #region Test: Equality

        [Fact]
        public void Equality_SameValues_ReturnsTrue()
        {
            var sphere1 = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var sphere2 = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));

            Assert.True(sphere1 == sphere2);
        }

        [Fact]
        public void Equality_DifferentValues_ReturnsFalse()
        {
            var sphere1 = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var sphere2 = new BoundingSphere(new Vector3d(1, 1, 1), new Fixed64(5));

            Assert.False(sphere1 == sphere2);
        }

        [Fact]
        public void GetHashCode_SameValues_ReturnsSameHash()
        {
            var sphere1 = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));
            var sphere2 = new BoundingSphere(new Vector3d(0, 0, 0), new Fixed64(5));

            Assert.Equal(sphere1.GetHashCode(), sphere2.GetHashCode());
        }

        #endregion

        #region Test: Edge Cases

        [Fact]
        public void Intersects_ZeroRadiusSphere_ReturnsTrueForOverlapping()
        {
            var sphere1 = new BoundingSphere(new Vector3d(0, 0, 0), Fixed64.Zero);
            var sphere2 = new BoundingSphere(new Vector3d(1, 1, 1), new Fixed64(2));

            // The distance between centers is less than or equal to the sum of radii, so they intersect
            Assert.True(sphere1.Intersects(sphere2));
        }

        [Fact]
        public void DistanceToSurface_ZeroRadiusSphere_ReturnsDistanceToCenter()
        {
            var sphere = new BoundingSphere(new Vector3d(0, 0, 0), Fixed64.Zero);
            var point = new Vector3d(3, 0, 0);

            Assert.Equal(new Fixed64(3), sphere.DistanceToSurface(point));
        }

        #endregion
    }
}
