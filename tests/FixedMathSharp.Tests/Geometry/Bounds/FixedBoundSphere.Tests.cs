using FixedMathSharp.Bounds;
using MemoryPack;
using System;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedBoundSphereTests
{
    #region Test: Constructor and Property

    [Fact]
    public void Constructor_AssignsValuesCorrectly()
    {
        var center = new Vector3d(1, 2, 3);
        var radius = new Fixed64(5);

        var sphere = new FixedBoundSphere(center, radius);

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
        var sphere = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var point = new Vector3d(2, 2, 2);

        Assert.True(sphere.Contains(point));
    }

    [Fact]
    public void Contains_PointOutside_ReturnsFalse()
    {
        var sphere = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var point = new Vector3d(6, 0, 0);

        Assert.False(sphere.Contains(point));
    }

    [Fact]
    public void Contains_PointOnSurface_ReturnsTrue()
    {
        var sphere = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var point = new Vector3d(5, 0, 0);

        Assert.True(sphere.Contains(point));
    }

    [Fact]
    public void Contains_BoundingSphere_ReturnsContainmentClassification()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, new Fixed64(5));
        var contained = new FixedBoundSphere(new Vector3d(1, 0, 0), new Fixed64(2));
        var largerSameCenter = new FixedBoundSphere(Vector3d.Zero, new Fixed64(6));
        var crossing = new FixedBoundSphere(new Vector3d(4, 0, 0), new Fixed64(2));
        var disjoint = new FixedBoundSphere(new Vector3d(8, 0, 0), Fixed64.One);

        Assert.Equal(FixedEnclosureType.Contains, sphere.Contains(contained));
        Assert.Equal(FixedEnclosureType.Intersects, sphere.Contains(largerSameCenter));
        Assert.Equal(FixedEnclosureType.Intersects, sphere.Contains(crossing));
        Assert.Equal(FixedEnclosureType.Disjoint, sphere.Contains(disjoint));
    }

    [Fact]
    public void Contains_BoundingBox_ReturnsContainmentClassification()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, new Fixed64(5));
        var contained = new FixedBoundBox(Vector3d.Zero, new Vector3d(2, 2, 2));
        var crossing = new FixedBoundBox(new Vector3d(5, 0, 0), new Vector3d(2, 2, 2));
        var disjoint = new FixedBoundBox(new Vector3d(8, 0, 0), new Vector3d(1, 1, 1));

        Assert.Equal(FixedEnclosureType.Contains, sphere.Contains(contained));
        Assert.Equal(FixedEnclosureType.Intersects, sphere.Contains(crossing));
        Assert.Equal(FixedEnclosureType.Disjoint, sphere.Contains(disjoint));
    }

    [Fact]
    public void Contains_BoundingArea_ReturnsContainmentClassification()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, new Fixed64(5));
        var contained = new FixedBoundArea(new Vector3d(-1, -1, 0), new Vector3d(1, 1, 0));
        var crossing = new FixedBoundArea(new Vector3d(4, -1, 0), new Vector3d(6, 1, 0));
        var disjoint = new FixedBoundArea(new Vector3d(8, -1, 0), new Vector3d(9, 1, 0));

        Assert.Equal(FixedEnclosureType.Contains, sphere.Contains(contained));
        Assert.Equal(FixedEnclosureType.Intersects, sphere.Contains(crossing));
        Assert.Equal(FixedEnclosureType.Disjoint, sphere.Contains(disjoint));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsContainmentClassification()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var containing = new FixedBoundSphere(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), new Fixed64(2));
        var crossing = new FixedBoundSphere(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Half), Fixed64.Half);
        var disjoint = new FixedBoundSphere(new Vector3d(new Fixed64(4), Fixed64.Zero, Fixed64.Half), Fixed64.Half);

        Assert.Equal(FixedEnclosureType.Contains, containing.Contains(frustum));
        Assert.Equal(FixedEnclosureType.Intersects, crossing.Contains(frustum));
        Assert.Equal(FixedEnclosureType.Disjoint, disjoint.Contains(frustum));
    }

    #endregion

    #region Test: Intersection

    [Fact]
    public void Intersects_WithOverlappingSphere_ReturnsTrue()
    {
        var sphere1 = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var sphere2 = new FixedBoundSphere(new Vector3d(3, 0, 0), new Fixed64(4));

        Assert.True(sphere1.Intersects(sphere2));
    }

    [Fact]
    public void Intersects_WithNonOverlappingSphere_ReturnsFalse()
    {
        var sphere1 = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var sphere2 = new FixedBoundSphere(new Vector3d(11, 0, 0), new Fixed64(4));

        Assert.False(sphere1.Intersects(sphere2));
    }

    [Fact]
    public void Intersects_WithBoundingBox_ReturnsTrue()
    {
        var sphere = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var box = new FixedBoundBox(new Vector3d(-3, -3, -3), new Vector3d(3, 3, 3));

        Assert.True(sphere.Intersects(box));
    }

    [Fact]
    public void Intersects_WithBoundingArea_ReturnsTrue()
    {
        var sphere = new FixedBoundSphere(new Vector3d(2, 2, 0), new Fixed64(3));
        var area = new FixedBoundArea(new Vector3d(0, 0, 0), new Vector3d(4, 4, 0));

        Assert.True(sphere.Intersects(area));
    }

    [Fact]
    public void Intersects_TypedBounds_UseSphereGeometry()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, new Fixed64(2));
        var box = new FixedBoundBox(new Vector3d(3, 0, 0), new Vector3d(2, 2, 2));
        var area = new FixedBoundArea(new Vector3d(2, -1, 0), new Vector3d(4, 1, 0));
        var otherSphere = new FixedBoundSphere(new Vector3d(4, 0, 0), new Fixed64(2));
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);

        Assert.True(sphere.Intersects(box));
        Assert.True(sphere.Intersects(area));
        Assert.True(sphere.Intersects(otherSphere));
        Assert.True(sphere.Intersects(frustum));
    }

    [Fact]
    public void Intersects_FixedPlaneAndRay_ReturnPrototypeStyleResults()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, new Fixed64(2));
        var plane = new FixedPlane(Vector3d.Right, new Fixed64(-1));
        var ray = new FixedRay(new Vector3d(-5, 0, 0), Vector3d.Right);

        Assert.Equal(FixedPlaneIntersectionType.Intersecting, sphere.Intersects(plane));
        Assert.Equal(new Fixed64(3), sphere.Intersects(ray));
    }

    #endregion

    #region Test: Creation

    [Fact]
    public void CreateFromBoundingBox_UsesBoxCenterAndFarthestCorner()
    {
        var box = new FixedBoundBox(new Vector3d(1, 2, 3), new Vector3d(2, 4, 6));

        FixedBoundSphere sphere = FixedBoundSphere.CreateFromBoundingBox(box);

        Assert.Equal(box.Center, sphere.Center);
        Assert.Equal(Vector3d.Distance(box.Center, box.Max), sphere.Radius);
    }

    [Fact]
    public void CreateFromFrustum_ContainsEveryCorner()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);

        FixedBoundSphere sphere = FixedBoundSphere.CreateFromFrustum(frustum);

        foreach (Vector3d corner in frustum.GetCorners())
            Assert.True(sphere.Contains(corner));
    }

    [Fact]
    public void CreateFromFrustum_DoesNotAllocate()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);

        _ = FixedBoundSphere.CreateFromFrustum(frustum);

        long before = GC.GetAllocatedBytesForCurrentThread();
        _ = FixedBoundSphere.CreateFromFrustum(frustum);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);
    }

    [Fact]
    public void CreateFromPoints_ContainsEveryPoint()
    {
        Vector3d[] points =
        {
            new(-1, 0, 0),
            new(1, 0, 0),
            new(0, 2, 0),
            new(0, 0, -3)
        };

        FixedBoundSphere sphere = FixedBoundSphere.CreateFromPoints(points);

        foreach (Vector3d point in points)
            Assert.True(sphere.Contains(point));
    }

    [Fact]
    public void CreateFromPoints_MaterializesEnumerableAndChoosesLargestYAxisPair()
    {
        static System.Collections.Generic.IEnumerable<Vector3d> Points()
        {
            yield return new Vector3d(0, -5, 0);
            yield return new Vector3d(0, 5, 0);
            yield return new Vector3d(2, 0, 0);
            yield return new Vector3d(0, 0, 1);
        }

        FixedBoundSphere sphere = FixedBoundSphere.CreateFromPoints(Points());

        Assert.Equal(Vector3d.Zero, sphere.Center);
        Assert.Equal(new Fixed64(5), sphere.Radius);
    }

    [Fact]
    public void CreateFromPoints_ChoosesLargestXAxisPair_WhenLaterPointExtendsMinimumX()
    {
        Vector3d[] points =
        {
            Vector3d.Zero,
            new(-5, 0, 0),
            new(5, 0, 0),
            new(0, 2, 0)
        };

        FixedBoundSphere sphere = FixedBoundSphere.CreateFromPoints(points);

        Assert.Equal(Vector3d.Zero, sphere.Center);
        Assert.Equal(new Fixed64(5), sphere.Radius);
    }

    [Fact]
    public void CreateFromPoints_NullInput_Throws()
    {
        Assert.Throws<ArgumentNullException>(() => FixedBoundSphere.CreateFromPoints(null!));
    }

    [Fact]
    public void CreateFromPoints_RejectsEmptyInput()
    {
        Assert.Throws<ArgumentException>(() => FixedBoundSphere.CreateFromPoints(Array.Empty<Vector3d>()));
    }

    [Fact]
    public void CreateMerged_ReturnsMinimalSphereContainingInputs()
    {
        var left = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var right = new FixedBoundSphere(new Vector3d(4, 0, 0), Fixed64.One);
        var contained = new FixedBoundSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), Fixed64.Half);

        FixedBoundSphere merged = FixedBoundSphere.CreateMerged(left, right);

        Assert.Equal(new Vector3d(2, 0, 0), merged.Center);
        Assert.Equal(new Fixed64(3), merged.Radius);
        Assert.Equal(left, FixedBoundSphere.CreateMerged(left, contained));
    }

    [Fact]
    public void CreateMerged_HandlesFractionalOffsetAndRadius()
    {
        var left = new FixedBoundSphere(
            Vector3d.FromDouble(-1.5, 0.25, 2.0),
            Fixed64.FromDouble(1.25));
        var right = new FixedBoundSphere(
            Vector3d.FromDouble(2.5, 0.25, 2.0),
            Fixed64.FromDouble(0.75));

        FixedBoundSphere merged = FixedBoundSphere.CreateMerged(left, right);

        Assert.Equal(Vector3d.FromDouble(0.25, 0.25, 2.0), merged.Center);
        Assert.Equal(new Fixed64(3), merged.Radius);
    }

    [Fact]
    public void CreateMerged_ReturnsContainingOrLargerSameCenterSphere()
    {
        var small = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var largerSameCenter = new FixedBoundSphere(Vector3d.Zero, new Fixed64(2));
        var largerOffset = new FixedBoundSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), new Fixed64(3));

        Assert.Equal(largerOffset, FixedBoundSphere.CreateMerged(small, largerOffset));
        Assert.Equal(largerSameCenter, FixedBoundSphere.CreateMerged(small, largerSameCenter));
        Assert.Equal(largerSameCenter, FixedBoundSphere.CreateMerged(largerSameCenter, small));
    }

    #endregion

    #region Test: Distance to Surface

    [Fact]
    public void ProjectPoint_ReturnsCenter_WhenPointIsCenter()
    {
        var sphere = new FixedBoundSphere(new Vector3d(1, 2, 3), new Fixed64(5));

        Assert.Equal(sphere.Center, sphere.ProjectPoint(sphere.Center));
    }

    [Fact]
    public void ProjectPoint_ReturnsPointOnSurface_WhenPointIsAwayFromCenter()
    {
        var sphere = new FixedBoundSphere(new Vector3d(1, 2, 3), new Fixed64(5));

        Assert.Equal(new Vector3d(6, 2, 3), sphere.ProjectPoint(new Vector3d(20, 2, 3)));
    }

    [Fact]
    public void ClampPoint_ReturnsOriginalInsideAndSurfacePointOutside()
    {
        var sphere = new FixedBoundSphere(new Vector3d(1, 2, 3), new Fixed64(5));

        Assert.Equal(new Vector3d(3, 2, 3), sphere.ClampPoint(new Vector3d(3, 2, 3)));
        Assert.Equal(new Vector3d(6, 2, 3), sphere.ClampPoint(new Vector3d(20, 2, 3)));
        Assert.Equal(sphere.Center, sphere.ClampPoint(sphere.Center));
    }

    [Fact]
    public void DistanceToSurface_PointOutside_ReturnsPositiveDistance()
    {
        var sphere = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var point = new Vector3d(10, 0, 0);

        Assert.Equal(new Fixed64(5), sphere.DistanceToSurface(point));
    }

    [Fact]
    public void DistanceToSurface_PointOnSurface_ReturnsZero()
    {
        var sphere = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var point = new Vector3d(5, 0, 0);

        Assert.Equal(Fixed64.Zero, sphere.DistanceToSurface(point));
    }

    [Fact]
    public void DistanceToSurface_PointInside_ReturnsNegativeDistance()
    {
        var sphere = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var point = new Vector3d(3, 0, 0);

        Assert.Equal(new Fixed64(-2), sphere.DistanceToSurface(point));
    }

    #endregion

    #region Test: Equality

    [Fact]
    public void Equality_SameValues_ReturnsTrue()
    {
        var sphere1 = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var sphere2 = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));

        Assert.True(sphere1 == sphere2);
    }

    [Fact]
    public void Equality_DifferentValues_ReturnsFalse()
    {
        var sphere1 = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var sphere2 = new FixedBoundSphere(new Vector3d(1, 1, 1), new Fixed64(5));

        Assert.False(sphere1 == sphere2);
    }

    [Fact]
    public void GetHashCode_SameValues_ReturnsSameHash()
    {
        var sphere1 = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var sphere2 = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));

        Assert.Equal(sphere1.GetHashCode(), sphere2.GetHashCode());
    }

    [Fact]
    public void Inequality_AndObjectEqualityBehaveCorrectly()
    {
        var sphere1 = new FixedBoundSphere(new Vector3d(0, 0, 0), new Fixed64(5));
        var sphere2 = new FixedBoundSphere(new Vector3d(0, 0, 1), new Fixed64(5));

        Assert.True(sphere1 != sphere2);
        Assert.True(sphere1.Equals((object)new FixedBoundSphere(Vector3d.Zero, new Fixed64(5))));
        Assert.False(sphere1.Equals("not-a-sphere"));
    }

    [Fact]
    public void DeconstructAndToString_ExposeCenterAndRadius()
    {
        var sphere = new FixedBoundSphere(new Vector3d(1, 2, 3), new Fixed64(5));

        sphere.Deconstruct(out Vector3d center, out Fixed64 radius);

        Assert.Equal(new Vector3d(1, 2, 3), center);
        Assert.Equal(new Fixed64(5), radius);
        Assert.Equal("{Center:(1, 2, 3) Radius:5}", sphere.ToString());
    }

    #endregion

    #region Test: Edge Cases

    [Fact]
    public void Intersects_ZeroRadiusSphere_ReturnsTrueForOverlapping()
    {
        var sphere1 = new FixedBoundSphere(new Vector3d(0, 0, 0), Fixed64.Zero);
        var sphere2 = new FixedBoundSphere(new Vector3d(1, 1, 1), new Fixed64(2));

        // The distance between centers is less than or equal to the sum of radii, so they intersect
        Assert.True(sphere1.Intersects(sphere2));
    }

    [Fact]
    public void DistanceToSurface_ZeroRadiusSphere_ReturnsDistanceToCenter()
    {
        var sphere = new FixedBoundSphere(new Vector3d(0, 0, 0), Fixed64.Zero);
        var point = new Vector3d(3, 0, 0);

        Assert.Equal(new Fixed64(3), sphere.DistanceToSurface(point));
    }

    #endregion

    #region Test: Transform

    [Fact]
    public void Transform_TransformsCenterAndUsesLargestBasisScaleForRadius()
    {
        var sphere = new FixedBoundSphere(new Vector3d(1, 1, 1), new Fixed64(2));
        Fixed4x4 matrix = Fixed4x4.CreateTransform(
            new Vector3d(10, 20, 30),
            FixedQuaternion.Identity,
            new Vector3d(2, 3, 4));

        FixedBoundSphere transformed = sphere.Transform(matrix);

        Assert.Equal(new Vector3d(12, 23, 34), transformed.Center);
        Assert.Equal(new Fixed64(8), transformed.Radius);
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void BoundingSphere_NetSerialization_RoundTripMaintainsData()
    {
        FixedBoundSphere originalValue = new(new Vector3d(1, 1, 1), new Fixed64(2));

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<FixedBoundSphere>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void BoundingSphere_MemoryPackSerialization_RoundTripMaintainsData()
    {
        FixedBoundSphere originalValue = new(new Vector3d(1, 1, 1), new Fixed64(2));

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedBoundSphere deserializedValue = MemoryPackSerializer.Deserialize<FixedBoundSphere>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion
}
