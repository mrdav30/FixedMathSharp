using System;
using System.Text.Json;
using FixedMathSharp.Geometry;
using MemoryPack;
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
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        var ray = new FixedRay(new Vector3d(-5, 0, 0), Vector3d.Right);

        Fixed64? hit = ray.Intersects(box);

        Assert.Equal(new Fixed64(4), hit);
    }

    [Fact]
    public void Intersects_BoundingBox_HandlesNegativeDirectionAndSwappedSlabDistances()
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        var ray = new FixedRay(new Vector3d(5, 0, 0), Vector3d.Left);

        Fixed64? hit = ray.Intersects(box);

        Assert.Equal(new Fixed64(4), hit);

        var diagonal = new FixedRay(new Vector3d(-5, -5, 0), new Vector3d(1, 1, 0));
        Assert.Equal(new Fixed64(4), diagonal.Intersects(box));
    }

    [Theory]
    [InlineData(1L)]
    [InlineData(-1L)]
    public void Intersects_BoundingBox_TreatsOneRawDirectionAsMotion(long rawDirection)
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 positionX = rawDirection > 0 ? -oneRaw : Fixed64.One + oneRaw;
        var box = FixedBoundBox.FromMinMax(Vector3d.Zero, Vector3d.One);
        var ray = new FixedRay(
            new Vector3d(positionX, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(rawDirection), Fixed64.One, Fixed64.Zero));

        Assert.Equal(Fixed64.One, ray.Intersects(box));
    }

    [Fact]
    public void Intersects_BoundingBox_ZeroDirectionOutsideSlabReturnsNull()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        var box = FixedBoundBox.FromMinMax(Vector3d.Zero, Vector3d.One);
        var ray = new FixedRay(
            new Vector3d(-oneRaw, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        Assert.Null(ray.Intersects(box));
    }

    [Fact]
    public void Intersects_BoundingBox_ReturnsZeroWhenRayStartsInside()
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        var ray = new FixedRay(Vector3d.Zero, Vector3d.Right);

        Fixed64? hit = ray.Intersects(box);

        Assert.Equal(Fixed64.Zero, hit);
    }

    [Fact]
    public void Intersects_BoundingBox_ReturnsNullWhenParallelOutsideOrBehind()
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
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
    public void Intersects_MinMaxBoundingBox_UsesBoxLikeBounds()
    {
        var box = FixedBoundBox.FromMinMax(new Vector3d(-1, -1, -1), new Vector3d(1, 1, 1));
        var ray = new FixedRay(new Vector3d(0, 0, -5), Vector3d.Forward);

        Fixed64? hit = ray.Intersects(box);

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
    public void Intersects_BoundingSphere_OrdersExtremeRangeQuadraticWithoutSaturation()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var crossing = new FixedRay(
            new Vector3d((Fixed64)(-100_000), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)200_000, Fixed64.Zero, Fixed64.Zero));
        var miss = new FixedRay(
            new Vector3d((Fixed64)(-100_000), (Fixed64)2, Fixed64.Zero),
            new Vector3d((Fixed64)200_000, Fixed64.Zero, Fixed64.Zero));

        Assert.Equal(Fixed64.FromFraction(99_999, 200_000), crossing.Intersects(sphere));
        Assert.Null(miss.Intersects(sphere));
    }

    [Fact]
    public void Intersects_BoundingSphere_TreatsOneRawDirectionAsMotion()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var ray = new FixedRay(
            new Vector3d(-Fixed64.One - oneRaw, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(oneRaw, Fixed64.Zero, Fixed64.Zero));

        Assert.Equal(Fixed64.One, ray.Intersects(sphere));
    }

    [Fact]
    public void Intersects_BoundingSphere_OrdersUnrepresentableOffsetWithinBound()
    {
        var sphere = new FixedBoundSphere(new Vector3d(-1, 0, 0), Fixed64.One);
        var ray = new FixedRay(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(-Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));

        Assert.Equal(Fixed64.One, ray.Intersects(sphere, Fixed64.One));
        Assert.Null(ray.Intersects(sphere, Fixed64.One - Fixed64.FromRaw(1L)));
    }

    [Fact]
    public void Intersects_BoundingSphere_ExpandsRadiusWithoutSaturatingTheSum()
    {
        var sphere = new FixedBoundSphere(new Vector3d(-1, 0, 0), Fixed64.MaxValue);
        var ray = new FixedRay(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero);

        Assert.Null(ray.Intersects(sphere, Fixed64.Zero));
        Assert.Equal(
            Fixed64.Zero,
            ray.Intersects(sphere, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            ray.Intersects(sphere, -Fixed64.One, Fixed64.One));
    }

    [Fact]
    public void Intersects_BoundingSphere_BoundedExpansionCoversFastAndInvalidBounds()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var ray = new FixedRay(new Vector3d(-5, 0, 0), Vector3d.Right);

        Assert.Equal((Fixed64)3, ray.Intersects(sphere, Fixed64.One, (Fixed64)3));
        Assert.Null(ray.Intersects(sphere, Fixed64.One, (Fixed64)2));
        Assert.Equal((Fixed64)4, ray.Intersects(sphere, Fixed64.Zero, (Fixed64)4));
        Assert.Null(ray.Intersects(sphere, -Fixed64.One));
        Assert.Null(ray.Intersects(sphere, Fixed64.One, -Fixed64.One));
    }

    [Fact]
    public void Intersects_BoundingSphere_RoundsWideRootsHalfToEven()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var sphere = new FixedBoundSphere(Vector3d.Zero, (Fixed64)65);
        var evenLower = new FixedRay(
            new Vector3d(-(Fixed64)65 - oneRaw, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(2, 0, 0));
        var oddLower = new FixedRay(
            new Vector3d(-(Fixed64)65 - Fixed64.FromRaw(3L), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(2, 0, 0));

        Assert.Equal(Fixed64.Zero, evenLower.Intersects(sphere));
        Assert.Equal(Fixed64.FromRaw(2L), oddLower.Intersects(sphere));
    }

    [Fact]
    public void Intersects_BoundingSphere_RoundsWideRootsOnEitherSideOfMidpoint()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, (Fixed64)65);
        var belowMidpoint = new FixedRay(
            new Vector3d(-(Fixed64)65 - Fixed64.FromRaw(1L), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(3, 0, 0));
        var aboveMidpoint = new FixedRay(
            new Vector3d(-(Fixed64)65 - Fixed64.FromRaw(2L), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(3, 0, 0));

        Assert.Equal(Fixed64.Zero, belowMidpoint.Intersects(sphere));
        Assert.Equal(Fixed64.FromRaw(1L), aboveMidpoint.Intersects(sphere));
    }

    [Fact]
    public void Intersects_BoundingSphere_RoundsWideTangentAndSubRawParameters()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, (Fixed64)65);
        var tangent = new FixedRay(
            new Vector3d(-100, 65, 0),
            new Vector3d(199, 0, 0));
        var subRaw = new FixedRay(
            new Vector3d(-Fixed64.FromRaw(1L), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));

        Assert.Equal(Fixed64.FromFraction(100, 199), tangent.Intersects(sphere));
        Assert.Equal(Fixed64.Zero, subRaw.Intersects(new FixedBoundSphere(Vector3d.Zero, Fixed64.Zero)));
    }

    [Fact]
    public void Intersects_BoundingSphere_PreservesExactRootAcrossEquivalentScale()
    {
        const long positionX = -21_474_923_131L;
        const long positionY = 4_294_881_327L;
        const long directionX = 4_295_022_249L;
        const long directionY = 2_228L;
        const long radius = 8_589_994_090L;
        var ray = new FixedRay(
            new Vector3d(Fixed64.FromRaw(positionX), Fixed64.FromRaw(positionY), Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(directionX), Fixed64.FromRaw(directionY), Fixed64.Zero));
        var scaledRay = new FixedRay(
            new Vector3d(Fixed64.FromRaw(positionX * 100), Fixed64.FromRaw(positionY * 100), Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(directionX * 100), Fixed64.FromRaw(directionY * 100), Fixed64.Zero));

        Fixed64? hit = ray.Intersects(new FixedBoundSphere(Vector3d.Zero, Fixed64.FromRaw(radius)));
        Fixed64? scaledHit = scaledRay.Intersects(
            new FixedBoundSphere(Vector3d.Zero, Fixed64.FromRaw(radius * 100)));

        Assert.Equal(Fixed64.FromRaw(14_035_527_845L), hit);
        Assert.Equal(hit, scaledHit);
    }

    [Fact]
    public void Intersects_BoundingSphere_RejectsExactRootBeyondRepresentableBound()
    {
        var ray = new FixedRay(
            new Vector3d(Fixed64.FromRaw(-17_179_869_185L), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(17_179_869_184L), Fixed64.Zero, Fixed64.Zero));

        Assert.Null(ray.Intersects(
            new FixedBoundSphere(Vector3d.Zero, Fixed64.Zero),
            Fixed64.One));
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
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        var inside = new FixedRay(Vector3d.Zero, Vector3d.Zero);
        var outside = new FixedRay(new Vector3d(3, 0, 0), Vector3d.Zero);

        Assert.Equal(Fixed64.Zero, inside.Intersects(box));
        Assert.Null(outside.Intersects(box));
    }

    [Fact]
    public void TryGetIntersectionInterval_SphereReturnsOrderedClippedParameters()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var ray = new FixedRay(new Vector3d(-10, 0, 0), new Vector3d(2, 0, 0));

        Assert.True(ray.TryGetIntersectionInterval(sphere, (Fixed64)10, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(9, 2), entry);
        Assert.Equal(Fixed64.FromFraction(11, 2), exit);

        Assert.True(ray.TryGetIntersectionInterval(sphere, (Fixed64)5, out entry, out exit));
        Assert.Equal(Fixed64.FromFraction(9, 2), entry);
        Assert.Equal((Fixed64)5, exit);

        Assert.False(ray.TryGetIntersectionInterval(sphere, (Fixed64)4, out entry, out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Zero, exit);
    }

    [Fact]
    public void TryGetIntersectionInterval_SpherePreservesClosedBoundaryAndZeroDirectionSemantics()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);

        Assert.True(new FixedRay(Vector3d.Zero, Vector3d.Zero)
            .TryGetIntersectionInterval(sphere, (Fixed64)3, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal((Fixed64)3, exit);

        Assert.True(new FixedRay(Vector3d.Right, Vector3d.Right)
            .TryGetIntersectionInterval(sphere, (Fixed64)3, out entry, out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Zero, exit);

        Assert.True(new FixedRay(Vector3d.Right, Vector3d.Left)
            .TryGetIntersectionInterval(sphere, (Fixed64)3, out entry, out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal((Fixed64)2, exit);

        Assert.False(new FixedRay((Fixed64)2 * Vector3d.Right, Vector3d.Zero)
            .TryGetIntersectionInterval(sphere, (Fixed64)3, out _, out _));
        Assert.False(new FixedRay(Vector3d.Zero, Vector3d.Right)
            .TryGetIntersectionInterval(sphere, -Fixed64.One, out _, out _));
    }

    [Fact]
    public void TryGetIntersectionInterval_SphereHandlesTangencyAndExactBoundBeforeRounding()
    {
        var sphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var tangent = new FixedRay(new Vector3d(-5, 1, 0), Vector3d.Right);

        Assert.True(tangent.TryGetIntersectionInterval(sphere, (Fixed64)10, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal((Fixed64)5, entry);
        Assert.Equal(entry, exit);

        var justBeyond = new FixedRay(
            new Vector3d(Fixed64.FromRaw(-17_179_869_185L), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(17_179_869_184L), Fixed64.Zero, Fixed64.Zero));
        var justInside = new FixedRay(
            new Vector3d(Fixed64.FromRaw(-17_179_869_183L), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(17_179_869_184L), Fixed64.Zero, Fixed64.Zero));
        var point = new FixedBoundSphere(Vector3d.Zero, Fixed64.Zero);

        Assert.False(justBeyond.TryGetIntersectionInterval(point, Fixed64.One, out _, out _));
        Assert.True(justInside.TryGetIntersectionInterval(point, Fixed64.One, out entry, out exit));
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal(entry, exit);
    }

    [Fact]
    public void TryGetIntersectionInterval_SphereExpandsRadiusWithoutSaturating()
    {
        var sphere = new FixedBoundSphere(new Vector3d(-1, 0, 0), Fixed64.MaxValue);
        var ray = new FixedRay(new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), Vector3d.Zero);

        Assert.False(ray.TryGetIntersectionInterval(sphere, Fixed64.One, out _, out _));
        Assert.True(ray.TryGetIntersectionInterval(
            sphere,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.One, exit);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            ray.TryGetIntersectionInterval(sphere, -Fixed64.One, Fixed64.One, out _, out _));
    }

    [Fact]
    public void TryGetIntersectionInterval_SphereRoundsBothRootsToEven()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var sphere = new FixedBoundSphere(Vector3d.Zero, (Fixed64)65);
        var evenCandidates = new FixedRay(
            new Vector3d(-(Fixed64)65 - oneRaw, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(2, 0, 0));
        var oddCandidates = new FixedRay(
            new Vector3d(-(Fixed64)65 - Fixed64.FromRaw(3L), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(2, 0, 0));

        Assert.True(evenCandidates.TryGetIntersectionInterval(sphere, (Fixed64)100, out Fixed64 entry, out Fixed64 exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal((Fixed64)65, exit);

        Assert.True(oddCandidates.TryGetIntersectionInterval(sphere, (Fixed64)100, out entry, out exit));
        Assert.Equal(Fixed64.FromRaw(2L), entry);
        Assert.Equal((Fixed64)65 + Fixed64.FromRaw(2L), exit);
    }

    [Fact]
    public void TryGetIntersectionInterval_SpherePreservesScaleAndSubRawOverlap()
    {
        var scaledSphere = new FixedBoundSphere(Vector3d.Zero, (Fixed64)100_000);
        var scaledRay = new FixedRay(
            new Vector3d(-1_000_000, 0, 0),
            new Vector3d(200_000, 0, 0));

        Assert.True(scaledRay.TryGetIntersectionInterval(
            scaledSphere,
            (Fixed64)10,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(9, 2), entry);
        Assert.Equal(Fixed64.FromFraction(11, 2), exit);

        Fixed64 oneRaw = Fixed64.FromRaw(1L);
        var subRawSphere = new FixedBoundSphere(new Vector3d(oneRaw, Fixed64.Zero, Fixed64.Zero), oneRaw);
        var subRawRay = new FixedRay(Vector3d.Zero, new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));

        Assert.True(subRawRay.TryGetIntersectionInterval(
            subRawSphere,
            Fixed64.One,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Zero, exit);
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
