using FixedMathSharp.Bounds;
using MemoryPack;
using System;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedRayTests
{
    [Fact]
    public void Constructor_AssignsPositionAndDirection()
    {
        var position = new Vector3d(1, 2, 3);
        var direction = Vector3d.Forward;

        var ray = new FixedRay(position, direction);

        Assert.Equal(position, ray.Position);
        Assert.Equal(direction, ray.Direction);
    }

    [Fact]
    public void Intersects_Plane_ReturnsHitParameter()
    {
        var ray = new FixedRay(new Vector3d(0, 0, -5), Vector3d.Forward);
        var plane = new FixedPlane(Vector3d.Forward, new Fixed64(-2));

        Fixed64? hit = ray.Intersects(plane);

        Assert.Equal(new Fixed64(7), hit);
    }

    [Fact]
    public void Intersects_Plane_ReturnsNullWhenParallelOrBehind()
    {
        var plane = new FixedPlane(Vector3d.Forward, new Fixed64(-2));
        var parallel = new FixedRay(Vector3d.Zero, Vector3d.Right);
        var behind = new FixedRay(Vector3d.Zero, Vector3d.Backward);

        Assert.Null(parallel.Intersects(plane));
        Assert.Null(behind.Intersects(plane));
    }

    [Fact]
    public void Intersects_BoundingBox_ReturnsNearestForwardHit()
    {
        var box = new FixedBoundBox(Vector3d.Zero, new Vector3d(2, 2, 2));
        var ray = new FixedRay(new Vector3d(-5, 0, 0), Vector3d.Right);

        Fixed64? hit = ray.Intersects(box);

        Assert.Equal(new Fixed64(4), hit);
    }

    [Fact]
    public void Intersects_BoundingBox_HandlesNegativeDirectionAndSwappedSlabDistances()
    {
        var box = new FixedBoundBox(Vector3d.Zero, new Vector3d(2, 2, 2));
        var ray = new FixedRay(new Vector3d(5, 0, 0), Vector3d.Left);

        Fixed64? hit = ray.Intersects(box);

        Assert.Equal(new Fixed64(4), hit);

        var diagonal = new FixedRay(new Vector3d(-5, -5, 0), new Vector3d(1, 1, 0));
        Assert.Equal(new Fixed64(4), diagonal.Intersects(box));
    }

    [Fact]
    public void Intersects_BoundingBox_ReturnsZeroWhenRayStartsInside()
    {
        var box = new FixedBoundBox(Vector3d.Zero, new Vector3d(2, 2, 2));
        var ray = new FixedRay(Vector3d.Zero, Vector3d.Right);

        Fixed64? hit = ray.Intersects(box);

        Assert.Equal(Fixed64.Zero, hit);
    }

    [Fact]
    public void Intersects_BoundingBox_ReturnsNullWhenParallelOutsideOrBehind()
    {
        var box = new FixedBoundBox(Vector3d.Zero, new Vector3d(2, 2, 2));
        var parallelOutside = new FixedRay(new Vector3d(-5, 2, 0), Vector3d.Right);
        var parallelBelow = new FixedRay(new Vector3d(-5, -2, 0), Vector3d.Right);
        var parallelOutsideZ = new FixedRay(new Vector3d(0, 0, 3), Vector3d.Right);
        var behind = new FixedRay(new Vector3d(5, 0, 0), Vector3d.Right);
        var behindOnZ = new FixedRay(new Vector3d(0, 0, 5), Vector3d.Forward);

        Assert.Null(parallelOutside.Intersects(box));
        Assert.Null(parallelBelow.Intersects(box));
        Assert.Null(parallelOutsideZ.Intersects(box));
        Assert.Null(behind.Intersects(box));
        Assert.Null(behindOnZ.Intersects(box));
    }

    [Fact]
    public void Intersects_BoundingArea_UsesBoxLikeBounds()
    {
        var area = new FixedBoundArea(new Vector3d(-1, -1, -1), new Vector3d(1, 1, 1));
        var ray = new FixedRay(new Vector3d(0, 0, -5), Vector3d.Forward);

        Fixed64? hit = ray.Intersects(area);

        Assert.Equal(new Fixed64(4), hit);
    }

    [Fact]
    public void Intersects_BoundingSphere_ReturnsNearestForwardHit()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var ray = new FixedRay(new Vector3d(-5, 0, 0), Vector3d.Right);

        Fixed64? hit = ray.Intersects(sphere);

        Assert.Equal(new Fixed64(4), hit);
    }

    [Fact]
    public void Intersects_BoundingSphere_HandlesNonUnitDirection()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var ray = new FixedRay(new Vector3d(-10, 0, 0), new Vector3d(2, 0, 0));

        Fixed64? hit = ray.Intersects(sphere);

        Assert.Equal(Fixed64.FromDouble(4.5), hit);
    }

    [Fact]
    public void Intersects_BoundingSphere_ReturnsZeroWhenRayStartsInside()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var ray = new FixedRay(Vector3d.Zero, Vector3d.Right);

        Fixed64? hit = ray.Intersects(sphere);

        Assert.Equal(Fixed64.Zero, hit);
    }

    [Fact]
    public void Intersects_BoundingSphere_ReturnsNullWhenMissingOrPointingAway()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var miss = new FixedRay(new Vector3d(-5, 2, 0), Vector3d.Right);
        var away = new FixedRay(new Vector3d(-5, 0, 0), Vector3d.Left);

        Assert.Null(miss.Intersects(sphere));
        Assert.Null(away.Intersects(sphere));
    }

    [Fact]
    public void Intersects_BoundingSphere_ZeroDirectionReturnsZeroOnlyWhenOriginIsInside()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var inside = new FixedRay(Vector3d.Zero, Vector3d.Zero);
        var outside = new FixedRay(new Vector3d(2, 0, 0), Vector3d.Zero);

        Assert.Equal(Fixed64.Zero, inside.Intersects(sphere));
        Assert.Null(outside.Intersects(sphere));
    }

    [Fact]
    public void Intersects_BoundingFrustum_ReturnsNearestForwardHit()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var ray = new FixedRay(new Vector3d(0, 0, -5), Vector3d.Forward);

        Fixed64? hit = ray.Intersects(frustum);

        Assert.Equal(new Fixed64(5), hit);
        Assert.Equal(hit, frustum.Intersects(ray));
    }

    [Fact]
    public void Intersects_BoundingFrustum_ReturnsZeroWhenRayStartsInside()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var ray = new FixedRay(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), Vector3d.Forward);

        Fixed64? hit = ray.Intersects(frustum);

        Assert.Equal(Fixed64.Zero, hit);
    }

    [Fact]
    public void Intersects_BoundingFrustum_ReturnsNullWhenMissingOrPointingAway()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var miss = new FixedRay(new Vector3d(2, 0, -5), Vector3d.Forward);
        var away = new FixedRay(new Vector3d(0, 0, -5), Vector3d.Backward);

        Assert.Null(miss.Intersects(frustum));
        Assert.Null(away.Intersects(frustum));
    }

    [Fact]
    public void Intersects_ZeroDirection_ReturnsZeroOnlyWhenOriginIsInsideVolume()
    {
        var box = new FixedBoundBox(Vector3d.Zero, new Vector3d(2, 2, 2));
        var inside = new FixedRay(Vector3d.Zero, Vector3d.Zero);
        var outside = new FixedRay(new Vector3d(3, 0, 0), Vector3d.Zero);

        Assert.Equal(Fixed64.Zero, inside.Intersects(box));
        Assert.Null(outside.Intersects(box));
    }

    [Fact]
    public void Equality_UsesPositionAndDirection()
    {
        var ray = new FixedRay(Vector3d.Zero, Vector3d.Forward);
        var same = new FixedRay(Vector3d.Zero, Vector3d.Forward);
        var different = new FixedRay(Vector3d.Zero, Vector3d.Right);

        Assert.True(ray == same);
        Assert.False(ray != same);
        Assert.False(ray == different);
        Assert.NotEqual(ray, different);
        Assert.False(ray.Equals(different));
        Assert.False(ray.Equals(new FixedRay(Vector3d.One, Vector3d.Forward)));
    }

    [Fact]
    public void DeconstructHashCodeAndObjectEquality_UsePositionAndDirection()
    {
        var ray = new FixedRay(new Vector3d(1, 2, 3), Vector3d.Forward);
        var same = new FixedRay(new Vector3d(1, 2, 3), Vector3d.Forward);

        ray.Deconstruct(out var position, out var direction);

        Assert.Equal(new Vector3d(1, 2, 3), position);
        Assert.Equal(Vector3d.Forward, direction);
        Assert.Equal(ray.GetHashCode(), same.GetHashCode());
        Assert.True(ray.Equals((object)same));
        Assert.False(ray.Equals((object)new object()));
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void Serialization_RoundTripsState()
    {
        var ray = new FixedRay(new Vector3d(1, 2, 3), Vector3d.Forward);

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(ray);
        var jsonRoundTrip = JsonSerializer.Deserialize<FixedRay>(json);

        byte[] memoryPack = MemoryPackSerializer.Serialize(ray);
        var memoryPackRoundTrip = MemoryPackSerializer.Deserialize<FixedRay>(memoryPack);

        Assert.Equal(ray, jsonRoundTrip);
        Assert.Equal(ray, memoryPackRoundTrip);
    }
#endif
}
