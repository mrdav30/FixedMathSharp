using FixedMathSharp.Bounds;
using MemoryPack;
using System;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedBoundBoxTests
{
    [Fact]
    public void FromFiniteConeClippedToDomain_NearUnitAxisConservativelyContainsBaseDiskSupports()
    {
        var axis = new Vector3d(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5) + Fixed64.MinIncrement,
            Fixed64.Zero);
        var apex = new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        var baseCenter = new Vector3d(Fixed64.One, Fixed64.Two, (Fixed64)3);

        FixedBoundBox box = FixedBoundBox.FromFiniteConeClippedToDomain(
            apex,
            baseCenter,
            axis,
            Fixed64.One);

        Vector3d xSupport = baseCenter
            + Vector3d.GetNormalizedProjectionOnPlane(Vector3d.Right, axis);
        Vector3d ySupport = baseCenter
            + Vector3d.GetNormalizedProjectionOnPlane(Vector3d.Up, axis);
        Vector3d zSupport = baseCenter
            + Vector3d.GetNormalizedProjectionOnPlane(Vector3d.Forward, axis);
        Assert.True(box.Contains(apex));
        Assert.True(box.Contains(xSupport));
        Assert.True(box.Contains(ySupport));
        Assert.True(box.Contains(zSupport));
        Assert.True(box.Max.X > baseCenter.X + Fixed64.FromFraction(4, 5));
    }

    [Fact]
    public void FromFiniteConeClippedToDomain_ValidatesGeometryAndClipsScalarLimits()
    {
        Assert.Throws<ArgumentException>(() => FixedBoundBox.FromFiniteConeClippedToDomain(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedBoundBox.FromFiniteConeClippedToDomain(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Right,
            -Fixed64.One));

        FixedBoundBox point = FixedBoundBox.FromFiniteConeClippedToDomain(
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Right,
            Fixed64.Zero);
        Assert.Equal(Vector3d.Zero, point.Min);
        Assert.Equal(Vector3d.One, point.Max);

        FixedBoundBox clipped = FixedBoundBox.FromFiniteConeClippedToDomain(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.One);
        Assert.Equal(Fixed64.MaxValue, clipped.Max.Y);
    }

    [Fact]
    public void FromFiniteConeClippedToDomain_LargeRadiusUsesLeastOutwardRawExtents()
    {
        Fixed64 radius = (Fixed64)2_000_000_000;
        var axes = new[]
        {
            new Vector3d(
                Fixed64.FromFraction(3, 5),
                Fixed64.FromFraction(4, 5) + Fixed64.MinIncrement,
                Fixed64.Zero),
            new Vector3d(
                Fixed64.FromFraction(3, 5),
                Fixed64.FromFraction(4, 5) - Fixed64.MinIncrement,
                Fixed64.Zero)
        };

        foreach (Vector3d axis in axes)
        {
            FixedBoundBox box = FixedBoundBox.FromFiniteConeClippedToDomain(
                Vector3d.Zero,
                Vector3d.Zero,
                axis,
                radius);

            AssertLeastOutwardDiskExtent(box.Max.X, radius, axis, axis.X);
            AssertLeastOutwardDiskExtent(box.Max.Y, radius, axis, axis.Y);
            AssertLeastOutwardDiskExtent(box.Max.Z, radius, axis, axis.Z);
        }
    }

    private static void AssertLeastOutwardDiskExtent(
        Fixed64 extent,
        Fixed64 radius,
        Vector3d axis,
        Fixed64 component)
    {
        BigInteger x = axis.X.m_rawValue;
        BigInteger y = axis.Y.m_rawValue;
        BigInteger z = axis.Z.m_rawValue;
        BigInteger q = x * x + y * y + z * z;
        BigInteger componentRaw = component.m_rawValue;
        BigInteger capacity = q - componentRaw * componentRaw;
        BigInteger radiusRaw = radius.m_rawValue;
        BigInteger target = radiusRaw * radiusRaw * capacity;
        BigInteger extentRaw = extent.m_rawValue;

        Assert.True(extentRaw * extentRaw * q >= target);
        if (extentRaw > BigInteger.Zero)
            Assert.True((extentRaw - BigInteger.One) * (extentRaw - BigInteger.One) * q < target);
    }
    #region Test: Constructor and Property

    [Fact]
    public void FromCenterAndSize_AssignsValuesCorrectly()
    {
        var center = new Vector3d(0, 0, 0);
        var size = new Vector3d(2, 2, 2);

        var box = FixedBoundBox.FromCenterAndSize(center, size);

        Assert.Equal(center, box.Center);
        Assert.Equal(size, box.Proportions);
        Assert.Equal(size * Fixed64.Half, box.Scope);
        Assert.Equal(new Vector3d(-1, -1, -1), box.Min);
        Assert.Equal(new Vector3d(1, 1, 1), box.Max);
    }

    [Fact]
    public void FromMinMax_NormalizesSwappedInputs()
    {
        var box = FixedBoundBox.FromMinMax(
            new Vector3d(4, -2, 8),
            new Vector3d(-6, 10, 1));

        Assert.Equal(new Vector3d(-6, -2, 1), box.Min);
        Assert.Equal(new Vector3d(4, 10, 8), box.Max);
        Assert.Equal(Vector3d.FromDouble(-1, 4, 4.5), box.Center);
        Assert.Equal(new Vector3d(10, 12, 7), box.Proportions);
    }

    [Fact]
    public void FromCenterAndSize_NormalizesNegativeSize()
    {
        var box = FixedBoundBox.FromCenterAndSize(
            new Vector3d(1, 2, 3),
            new Vector3d(-4, 6, -8));

        Assert.Equal(new Vector3d(-1, -1, -1), box.Min);
        Assert.Equal(new Vector3d(3, 5, 7), box.Max);
        Assert.Equal(new Vector3d(4, 6, 8), box.Proportions);
    }

    [Fact]
    public void FromCenterAndScope_NormalizesNegativeScope()
    {
        var box = FixedBoundBox.FromCenterAndScope(
            new Vector3d(1, 2, 3),
            new Vector3d(-2, 3, -4));

        Assert.Equal(new Vector3d(-1, -1, -1), box.Min);
        Assert.Equal(new Vector3d(3, 5, 7), box.Max);
        Assert.Equal(new Vector3d(2, 3, 4), box.Scope);
    }

    [Fact]
    public void DerivedMetadata_FullDomainAndRawUnitSpans_RemainsExactOrFailsHonestly()
    {
        Fixed64 sameSignMin = Fixed64.FromRaw(long.MaxValue - 3);
        Fixed64 sameSignMax = Fixed64.FromRaw(long.MaxValue - 1);
        var sameSign = FixedBoundBox.FromMinMax(
            new Vector3d(sameSignMin, Fixed64.FromRaw(1), Fixed64.FromRaw(3)),
            new Vector3d(sameSignMax, Fixed64.FromRaw(2), Fixed64.FromRaw(6)));

        Assert.Equal(
            new Vector3d(
                Fixed64.FromRaw(long.MaxValue - 2),
                Fixed64.FromRaw(2),
                Fixed64.FromRaw(4)),
            sameSign.Center);
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(2), Fixed64.MinIncrement, Fixed64.FromRaw(3)),
            sameSign.Proportions);
        Assert.Equal(
            new Vector3d(Fixed64.MinIncrement, Fixed64.MinIncrement, Fixed64.FromRaw(2)),
            sameSign.Scope);

        Fixed64 quarterDomain = Fixed64.FromRaw(1L << 62);
        var wide = FixedBoundBox.FromMinMax(
            new Vector3d(-quarterDomain, Fixed64.Zero, -Fixed64.One),
            new Vector3d(quarterDomain, Fixed64.One, Fixed64.One));

        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero), wide.Center);
        Assert.Equal(new Vector3d(quarterDomain, Fixed64.Half, Fixed64.One), wide.Scope);
        Assert.Throws<OverflowException>(() => _ = wide.Proportions);

        var fullDomain = FixedBoundBox.FromMinMax(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.One));
        fullDomain.Center = fullDomain.Center;

        Assert.Equal(Fixed64.Zero, fullDomain.Center.X);
        Assert.Throws<OverflowException>(() => _ = fullDomain.Scope);
    }

    #endregion

    #region Test: Containment

    [Fact]
    public void Contains_PointInside_ReturnsTrue()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(1, 1, 1);

        Assert.True(box.Contains(point));
    }

    [Fact]
    public void Contains_PointOutside_ReturnsFalse()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(5, 5, 5);

        Assert.False(box.Contains(point));
    }

    [Fact]
    public void Contains_PointOnBoundary_ReturnsTrue()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(2, 0, 0);

        Assert.True(box.Contains(point));
    }

    [Theory]
    [InlineData(-2, -2, -2)]
    [InlineData(2, -2, -2)]
    [InlineData(-2, 2, -2)]
    [InlineData(2, 2, -2)]
    [InlineData(-2, -2, 2)]
    [InlineData(2, -2, 2)]
    [InlineData(-2, 2, 2)]
    [InlineData(2, 2, 2)]
    public void Contains_BoxCornerBoundary_ReturnsTrue(int x, int y, int z)
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(4, 4, 4));

        Assert.True(box.Contains(new Vector3d(x, y, z)));
    }

    [Fact]
    public void Constructor_WithNegativeSize_StillContainsCenter()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(-4, -4, -4));
        Assert.True(box.Contains(new Vector3d(0, 0, 0)));
    }

    #endregion

    #region Test: Intersection

    [Fact]
    public void Intersects_WithOverlappingBox_ReturnsTrue()
    {
        var box1 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 1, 1), new Vector3d(4, 4, 4));
        var contained = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));

        Assert.True(box1.Intersects(box2));
        Assert.True(box1.Intersects(contained));

        var area3 = FixedBoundBox.FromCenterAndSize(new Vector3d(-2, -2, 0), new Vector3d(2, 2, 0));
        var area4 = FixedBoundBox.FromCenterAndSize(new Vector3d(-1, -1, 0), new Vector3d(3, 3, 0));
        Assert.True(area3.Intersects(area4));
        Assert.False(area3.IntersectsStrict(area4));
    }

    [Fact]
    public void Intersects_WithNonOverlappingBox_ReturnsFalse()
    {
        var box1 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var box2 = FixedBoundBox.FromCenterAndSize(new Vector3d(5, 5, 5), new Vector3d(2, 2, 2));

        Assert.False(box1.Intersects(box2));
    }

    [Fact]
    public void Intersects_BoxFullyContainedOnBoundary_ReturnsTrue()
    {
        var containing = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(4, 4, 4));
        var contained = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 0, 0), new Vector3d(2, 4, 4));

        Assert.Equal(new Vector3d(0, -2, -2), contained.Min);
        Assert.Equal(new Vector3d(2, 2, 2), contained.Max);
        Assert.Equal(FixedEnclosureType.Contains, containing.Contains(contained));
        Assert.True(containing.Intersects(contained));
    }

    [Theory]
    [InlineData(2, 0, 0)]
    [InlineData(0, 2, 0)]
    [InlineData(0, 0, 2)]
    [InlineData(2, 2, 2)]
    public void Intersects_TouchingBoxes_IsBoundaryInclusive(int x, int y, int z)
    {
        var a = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        var b = FixedBoundBox.FromCenterAndSize(new Vector3d(x, y, z), new Vector3d(2, 2, 2));

        Assert.True(a.Intersects(b));
        Assert.False(a.IntersectsStrict(b));
    }

    [Fact]
    public void Intersects_WithMinMaxBoundingBox_ReturnsTrue()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var other = FixedBoundBox.FromMinMax(new Vector3d(1, 1, 1), new Vector3d(3, 3, 3));

        Assert.True(box.Intersects(other));
    }

    [Fact]
    public void Intersects_WithBoundingSphere_ReturnsTrue()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var sphere = new FixedBoundSphere(new Vector3d(1, 1, 1), Fixed64.One);

        Assert.True(box.Intersects(sphere));
    }

    [Fact]
    public void IntersectsStrict_BoundingSphere_RequiresPositiveVolumeOverlap()
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        var touching = new FixedBoundSphere(new Vector3d(2, 0, 0), Fixed64.One);
        var overlapping = new FixedBoundSphere(new Vector3d(Fixed64.One + Fixed64.Half, Fixed64.Zero, Fixed64.Zero), Fixed64.One);
        var disjoint = new FixedBoundSphere(new Vector3d(3, 0, 0), Fixed64.One);
        var zeroRadiusInside = new FixedBoundSphere(Vector3d.Zero, Fixed64.Zero);

        Assert.True(box.Intersects(touching));
        Assert.False(box.IntersectsStrict(touching));
        Assert.True(box.IntersectsStrict(overlapping));
        Assert.False(box.Intersects(disjoint));
        Assert.False(box.IntersectsStrict(zeroRadiusInside));
    }

    [Fact]
    public void SpherePredicates_LargeValuesAvoidSquaredAndExtentSaturation()
    {
        var distantSphere = new FixedBoundSphere(new Vector3d(200_000, 0, 0), (Fixed64)100_000);
        var unitBox = FixedBoundBox.FromMinMax(Vector3d.Zero, Vector3d.One);
        Fixed64 centerX = Fixed64.MaxValue - Fixed64.One;
        var limitBox = FixedBoundBox.FromMinMax(
            new Vector3d(Fixed64.Zero, (Fixed64)(-20), (Fixed64)(-20)),
            new Vector3d(Fixed64.MaxValue, (Fixed64)20, (Fixed64)20));
        var crossingLimit = new FixedBoundSphere(
            new Vector3d(centerX, Fixed64.Zero, Fixed64.Zero),
            (Fixed64)10);

        Assert.False(unitBox.Intersects(distantSphere));
        Assert.Equal(FixedEnclosureType.Intersects, limitBox.Contains(crossingLimit));
    }

    [Fact]
    public void Intersects_WithBoundingFrustum_ReturnsTrueOnlyWhenOverlapping()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var overlapping = FixedBoundBox.FromCenterAndSize(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), new Vector3d(2, 2, 2));
        var disjoint = FixedBoundBox.FromCenterAndSize(new Vector3d(new Fixed64(4), Fixed64.Zero, Fixed64.Half), new Vector3d(1, 1, 1));

        Assert.True(overlapping.Intersects(frustum));
        Assert.False(disjoint.Intersects(frustum));
    }

    [Fact]
    public void Contains_TypedBounds_ReturnsContainmentClassification()
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(4, 4, 4));
        var containedBox = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(1, 1, 1));
        var crossingBox = FixedBoundBox.FromCenterAndSize(new Vector3d(2, 2, 0), new Vector3d(2, 2, 2));
        var disjointBox = FixedBoundBox.FromMinMax(new Vector3d(5, 5, 5), new Vector3d(6, 6, 6));
        var containedSphere = new FixedBoundSphere(Vector3d.Zero, Fixed64.One);
        var crossingSphere = new FixedBoundSphere(new Vector3d(2, 0, 0), Fixed64.One);
        var disjointSphere = new FixedBoundSphere(new Vector3d(5, 0, 0), Fixed64.One);
        var containedFrustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var crossingFrustumMatrix = Fixed4x4.Identity;
        crossingFrustumMatrix.M41 = Fixed64.Two;
        var crossingFrustum = new FixedBoundFrustum(crossingFrustumMatrix);
        var disjointFrustumMatrix = Fixed4x4.Identity;
        disjointFrustumMatrix.M41 = new Fixed64(10);
        var disjointFrustum = new FixedBoundFrustum(disjointFrustumMatrix);

        Assert.Equal(FixedEnclosureType.Contains, box.Contains(containedBox));
        Assert.Equal(FixedEnclosureType.Intersects, box.Contains(crossingBox));
        Assert.Equal(FixedEnclosureType.Disjoint, box.Contains(disjointBox));
        Assert.Equal(FixedEnclosureType.Contains, box.Contains(containedSphere));
        Assert.Equal(FixedEnclosureType.Intersects, box.Contains(crossingSphere));
        Assert.Equal(FixedEnclosureType.Disjoint, box.Contains(disjointSphere));
        Assert.Equal(FixedEnclosureType.Contains, box.Contains(containedFrustum));
        Assert.Equal(FixedEnclosureType.Intersects, box.Contains(crossingFrustum));
        Assert.Equal(FixedEnclosureType.Disjoint, box.Contains(disjointFrustum));
    }

    [Fact]
    public void Intersects_TouchingEdges_IsBoundaryInclusive()
    {
        var a = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var b = FixedBoundBox.FromCenterAndSize(new Vector3d(2, 0, 0), new Vector3d(2, 2, 2));

        Assert.True(a.Intersects(b));
        Assert.False(a.IntersectsStrict(b));
    }

    [Fact]
    public void IntersectsStrict_RequiresPositiveVolumeOverlap()
    {
        var a = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        var touchingFace = FixedBoundBox.FromCenterAndSize(new Vector3d(2, 0, 0), new Vector3d(2, 2, 2));
        var touchingEdge = FixedBoundBox.FromCenterAndSize(new Vector3d(2, 2, 0), new Vector3d(2, 2, 2));
        var touchingCorner = FixedBoundBox.FromCenterAndSize(new Vector3d(2, 2, 2), new Vector3d(2, 2, 2));
        var zeroSizeInside = FixedBoundBox.FromMinMax(Vector3d.Zero, Vector3d.Zero);
        var overlapping = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 0, 0), new Vector3d(2, 2, 2));

        Assert.True(a.Intersects(touchingFace));
        Assert.True(a.Intersects(touchingEdge));
        Assert.True(a.Intersects(touchingCorner));
        Assert.True(a.Intersects(zeroSizeInside));
        Assert.False(a.IntersectsStrict(touchingFace));
        Assert.False(a.IntersectsStrict(touchingEdge));
        Assert.False(a.IntersectsStrict(touchingCorner));
        Assert.False(a.IntersectsStrict(zeroSizeInside));
        Assert.True(a.IntersectsStrict(overlapping));
    }

    [Fact]
    public void Intersects_Overlapping3DVolumeBoxes_ReturnsTrue()
    {
        var a = FixedBoundBox.FromCenterAndSize(new Vector3d(3, 3, 3), new Vector3d(4, 4, 4));
        var b = FixedBoundBox.FromCenterAndSize(new Vector3d(5, 5, 5), new Vector3d(4, 4, 4));

        Assert.True(a.Intersects(b));
        Assert.Equal(FixedEnclosureType.Intersects, a.Contains(b));
    }

    [Fact]
    public void Intersects_Separated3DVolumeBoxes_ReturnsFalse()
    {
        var a = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 1, 1), new Vector3d(2, 2, 2));
        var b = FixedBoundBox.FromCenterAndSize(new Vector3d(4, 4, 4), new Vector3d(2, 2, 2));

        Assert.False(a.Intersects(b));
        Assert.Equal(FixedEnclosureType.Disjoint, a.Contains(b));
    }

    #endregion

    #region Test: Surface Distance and Closest Point

    [Fact]
    public void DistanceToSurface_WorksCorrectly()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(3, 0, 0);

        Assert.Equal(Fixed64.One, box.DistanceToSurface(point));
    }

    [Fact]
    public void DistanceToSurface_InsidePoint_ReturnsZero()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.Equal(Fixed64.Zero, box.DistanceToSurface(new Vector3d(1, 1, 1)));
    }

    [Fact]
    public void ClosestPointOnSurface_ReturnsCorrectPoint()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var point = new Vector3d(3, 3, 3);

        var closestPoint = box.ClosestPointOnSurface(point);
        Assert.Equal(new Vector3d(2, 2, 2), closestPoint);
    }

    [Fact]
    public void ClosestPointOnSurface_InsidePoint_UsesNearestFace()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.Equal(Vector3d.FromDouble(-2, 0.5, 0.25), box.ClosestPointOnSurface(Vector3d.FromDouble(-1.75, 0.5, 0.25)));
        Assert.Equal(Vector3d.FromDouble(2, 0.5, 0.25), box.ClosestPointOnSurface(Vector3d.FromDouble(1.75, 0.5, 0.25)));
        Assert.Equal(Vector3d.FromDouble(0.25, -2, -0.75), box.ClosestPointOnSurface(Vector3d.FromDouble(0.25, -1.9, -0.75)));
        Assert.Equal(Vector3d.FromDouble(0.25, 2, -0.75), box.ClosestPointOnSurface(Vector3d.FromDouble(0.25, 1.9, -0.75)));
        Assert.Equal(Vector3d.FromDouble(-0.25, 0.5, -2), box.ClosestPointOnSurface(Vector3d.FromDouble(-0.25, 0.5, -1.8)));
        Assert.Equal(Vector3d.FromDouble(-0.25, 0.5, 2), box.ClosestPointOnSurface(Vector3d.FromDouble(-0.25, 0.5, 1.8)));
    }

    [Fact]
    public void ProjectPoint_ClampsToBounds()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var projected = box.ProjectPoint(new Vector3d(5, -3, 1));

        Assert.Equal(new Vector3d(2, -2, 1), projected);
    }

    [Fact]
    public void ClampPoint_ClampsToBounds()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var clamped = box.ClampPoint(new Vector3d(5, -3, 1));

        Assert.Equal(new Vector3d(2, -2, 1), clamped);
    }

    [Fact]
    public void GetPointOnSurfaceTowardsObject_UsesProjectedPoint()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var point = box.GetPointOnSurfaceTowardsObject(new Vector3d(5, 1, 0));

        Assert.Equal(new Vector3d(2, 1, 0), point);
    }

    #endregion

    #region Test: Equality

    [Fact]
    public void Equality_SameBox_ReturnsTrue()
    {
        var box1 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.True(box1 == box2);
    }

    [Fact]
    public void Equality_DifferentBox_ReturnsFalse()
    {
        var box1 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 1, 1), new Vector3d(4, 4, 4));

        Assert.False(box1 == box2);
    }

    [Fact]
    public void GetHashCode_SameBox_ReturnsSameHash()
    {
        var box1 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.Equal(box1.GetHashCode(), box2.GetHashCode());
    }

    [Fact]
    public void GetHashCode_UsesDeterministicComponentHash()
    {
        var box = FixedBoundBox.FromMinMax(new Vector3d(-1, -2, -3), new Vector3d(3, 5, 7));

        Assert.Equal(CombineVectorPairHash(box.Min, box.Max), box.GetHashCode());
    }

    [Fact]
    public void SetBoundingBox_KeepsSizeAndScopeInSync()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var newScope = new Vector3d(3, 3, 3);
        box.SetBoundingBox(new Vector3d(1, 1, 1), newScope);

        Assert.Equal(newScope * Fixed64.Two, box.Proportions);
    }

    [Fact]
    public void Inequality_DifferentBox_ReturnsTrue()
    {
        var box1 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));
        var box2 = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 1, 1), new Vector3d(4, 4, 4));

        Assert.True(box1 != box2);
    }

    [Fact]
    public void Equals_WithDifferentObjectType_ReturnsFalse()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.False(box.Equals("not-a-box"));
        Assert.True(box.Equals((object)FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4))));
    }

    #endregion

    #region Test: Edge Cases

    [Fact]
    public void Contains_ZeroSizeBox_ContainsPoint()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
        var point = new Vector3d(0, 0, 0);

        Assert.True(box.Contains(point));
    }

    [Fact]
    public void Intersects_ZeroSizeBox_IsBoundaryInclusive()
    {
        var box1 = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
        var box2 = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 1, 1), new Vector3d(2, 2, 2));
        var disjoint = FixedBoundBox.FromCenterAndSize(new Vector3d(3, 3, 3), new Vector3d(1, 1, 1));

        Assert.True(box1.Intersects(box2));
        Assert.False(box1.IntersectsStrict(box2));
        Assert.False(box1.Intersects(disjoint));
    }

    #endregion

    #region Test: Mutation Invariance

    [Fact]
    public void Resize_PreservesCenter()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(5, 5, 5), new Vector3d(4, 4, 4));

        box.Resize(new Vector3d(2, 2, 2));

        Assert.Equal(new Vector3d(5, 5, 5), box.Center);
    }

    [Fact]
    public void SetMinMax_SetsCorrectCenterAndSize()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));

        box.SetMinMax(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        Assert.Equal(new Vector3d(2, 2, 2), box.Center);
        Assert.Equal(new Vector3d(4, 4, 4), box.Proportions);
    }

    [Fact]
    public void SetMinMax_SwappedInputs_NormalizesBounds()
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));

        box.SetMinMax(new Vector3d(4, 4, 4), new Vector3d(-2, -2, -2));

        Assert.Equal(new Vector3d(-2, -2, -2), box.Min);
        Assert.Equal(new Vector3d(4, 4, 4), box.Max);
        Assert.Equal(new Vector3d(6, 6, 6), box.Proportions);
        Assert.True(box.Contains(Vector3d.Zero));
    }

    [Fact]
    public void State_WithSwappedMinMax_NormalizesBounds()
    {
        var state = new FixedBoundBox.BoundingBoxState(
            new Vector3d(3, 3, 3),
            new Vector3d(-1, -1, -1));
        var box = new FixedBoundBox(state);

        Assert.Equal(new Vector3d(-1, -1, -1), state.Min);
        Assert.Equal(new Vector3d(3, 3, 3), state.Max);
        Assert.Equal(new Vector3d(-1, -1, -1), box.Min);
        Assert.Equal(new Vector3d(3, 3, 3), box.Max);
        Assert.Equal(new Vector3d(4, 4, 4), box.Proportions);
    }

    [Fact]
    public void Center_Setter_RepositionsBoundsWithoutChangingSize()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 6, 8));

        box.Center = new Vector3d(5, -1, 3);

        Assert.Equal(new Vector3d(5, -1, 3), box.Center);
        Assert.Equal(new Vector3d(4, 6, 8), box.Proportions);
        Assert.Equal(new Vector3d(3, -4, -1), box.Min);
        Assert.Equal(new Vector3d(7, 2, 7), box.Max);
    }

    [Fact]
    public void Center_Setter_RawUnitSpan_PreservesRequestedCenterWithConservativeScope()
    {
        var box = FixedBoundBox.FromMinMax(
            new Vector3d(Fixed64.FromRaw(1), Fixed64.FromRaw(3), Fixed64.FromRaw(5)),
            new Vector3d(Fixed64.FromRaw(2), Fixed64.FromRaw(6), Fixed64.FromRaw(10)));

        box.Center = new Vector3d(
            Fixed64.FromRaw(11),
            Fixed64.FromRaw(20),
            Fixed64.FromRaw(30));

        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(10), Fixed64.FromRaw(18), Fixed64.FromRaw(27)),
            box.Min);
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(12), Fixed64.FromRaw(22), Fixed64.FromRaw(33)),
            box.Max);
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(11), Fixed64.FromRaw(20), Fixed64.FromRaw(30)),
            box.Center);
    }

    [Fact]
    public void CenterResizeAndOrient_UnrepresentableEndpoint_ThrowWithoutMutation()
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(4, 6, 8));
        FixedBoundBox original = box;

        Assert.Throws<OverflowException>(() => box.Center = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(original, box);

        box = FixedBoundBox.FromCenterAndSize(
            new Vector3d(Fixed64.MaxValue - Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(2, 2, 2));
        original = box;

        Assert.Throws<OverflowException>(() => box.Resize(new Vector3d(4, 2, 2)));
        Assert.Equal(original, box);
        Assert.Throws<OverflowException>(() => box.Orient(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            null));
        Assert.Equal(original, box);
    }

    [Fact]
    public void CenteredFactories_UnrepresentableEndpointOrMagnitude_Throw()
    {
        Assert.Throws<OverflowException>(() => FixedBoundBox.FromCenterAndSize(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(2, 2, 2)));
        Assert.Throws<OverflowException>(() => FixedBoundBox.FromCenterAndSize(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(2, 2, 2)));
        Assert.Throws<OverflowException>(() => FixedBoundBox.FromCenterAndScope(
            Vector3d.Zero,
            new Vector3d(Fixed64.MinValue, Fixed64.One, Fixed64.One)));

        FixedBoundBox rawUnit = FixedBoundBox.FromCenterAndSize(
            new Vector3d(Fixed64.FromRaw(10), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinIncrement, Fixed64.MinValue, Fixed64.MinIncrement));

        Assert.Equal(Fixed64.FromRaw(9), rawUnit.Min.X);
        Assert.Equal(Fixed64.FromRaw(11), rawUnit.Max.X);
        Assert.Equal(Fixed64.FromRaw(1L << 62), rawUnit.Scope.Y);
        Assert.Throws<OverflowException>(() => _ = rawUnit.Proportions);
    }

    [Fact]
    public void ClippedCenteredFactories_ExplicitlyIntersectWithScalarDomain()
    {
        FixedBoundBox fromSize = FixedBoundBox.FromCenterAndSizeClippedToDomain(
            new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.Zero),
            new Vector3d(2, 2, 2));
        FixedBoundBox fromScope = FixedBoundBox.FromCenterAndScopeClippedToDomain(
            new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.Zero),
            new Vector3d(-Fixed64.One, -Fixed64.One, -Fixed64.One));

        var expected = FixedBoundBox.FromMinMax(
            new Vector3d(Fixed64.MaxValue - Fixed64.One, Fixed64.MinValue, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.MinValue + Fixed64.One, Fixed64.One));
        Assert.Equal(expected, fromSize);
        Assert.Equal(expected, fromScope);
    }

    [Fact]
    public void Proportions_Setter_ResizesBoundsWithoutChangingCenter()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(1, 2, 3), new Vector3d(4, 4, 4));

        box.Proportions = new Vector3d(6, 8, 10);

        Assert.Equal(new Vector3d(1, 2, 3), box.Center);
        Assert.Equal(new Vector3d(-2, -2, -2), box.Min);
        Assert.Equal(new Vector3d(4, 6, 8), box.Max);
    }

    [Fact]
    public void Orient_WithNullSize_PreservesCurrentProportions()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 6, 8));

        box.Orient(new Vector3d(10, 20, 30), null);

        Assert.Equal(new Vector3d(10, 20, 30), box.Center);
        Assert.Equal(new Vector3d(4, 6, 8), box.Proportions);
    }

    [Fact]
    public void Orient_WithExplicitSize_UsesProvidedDimensions()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        box.Orient(new Vector3d(1, 2, 3), new Vector3d(2, 4, 6));

        Assert.Equal(new Vector3d(1, 2, 3), box.Center);
        Assert.Equal(new Vector3d(2, 4, 6), box.Proportions);
        Assert.Equal(new Vector3d(0, 0, 0), box.Min);
        Assert.Equal(new Vector3d(2, 4, 6), box.Max);
    }

    #endregion

    #region Test: Vertex Cache Safety

    [Fact]
    public void GetCorner_ReturnsStableCornerOrderAfterMutation()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var original = box.GetCorner(0);

        box.Resize(new Vector3d(4, 4, 4));
        var updated = box.GetCorner(0);

        Assert.Equal(new Vector3d(-1, -1, -1), original);
        Assert.Equal(new Vector3d(-2, -2, -2), updated);
    }

    [Theory]
    [InlineData(0, -1, -2, -3)]
    [InlineData(1, 1, -2, -3)]
    [InlineData(2, -1, 2, -3)]
    [InlineData(3, 1, 2, -3)]
    [InlineData(4, -1, -2, 3)]
    [InlineData(5, 1, -2, 3)]
    [InlineData(6, -1, 2, 3)]
    [InlineData(7, 1, 2, 3)]
    public void GetCorner_ReturnsAllEightCornersInStableOrder(int index, int x, int y, int z)
    {
        var box = FixedBoundBox.FromMinMax(new Vector3d(-1, -2, -3), new Vector3d(1, 2, 3));

        Assert.Equal(new Vector3d(x, y, z), box.GetCorner(index));
    }

    [Fact]
    public void GetCorner_InvalidIndex_Throws()
    {
        FixedBoundBox box = default;

        Assert.Throws<ArgumentOutOfRangeException>(() => box.GetCorner(-1));
        Assert.Throws<ArgumentOutOfRangeException>(() => box.GetCorner(8));
    }

    [Fact]
    public void CopyCorners_CopiesStableOrder()
    {
        var box = FixedBoundBox.FromMinMax(new Vector3d(-1, -2, -3), new Vector3d(1, 2, 3));
        Span<Vector3d> corners = stackalloc Vector3d[8];

        box.CopyCorners(corners);

        for (int i = 0; i < corners.Length; i++)
            Assert.Equal(box.GetCorner(i), corners[i]);
    }

    [Fact]
    public void CopyCorners_ShortDestination_Throws()
    {
        FixedBoundBox box = default;
        Assert.Throws<ArgumentException>(() => box.CopyCorners(new Vector3d[7]));
    }

    [Fact]
    public void GetCornerAndCopyCorners_DoNotAllocate()
    {
        var box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        Span<Vector3d> corners = stackalloc Vector3d[8];

        box.CopyCorners(corners);
        _ = box.GetCorner(0);

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++)
        {
            _ = box.GetCorner(i & 7);
            box.CopyCorners(corners);
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(0, allocated);
    }

    [Fact]
    public void Union_EnclosesBothBoxes()
    {
        var a = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var b = FixedBoundBox.FromCenterAndSize(new Vector3d(4, 0, 0), new Vector3d(4, 4, 4));

        var union = FixedBoundBox.Union(a, b);

        Assert.Equal(new Vector3d(-1, -2, -2), union.Min);
        Assert.Equal(new Vector3d(6, 2, 2), union.Max);
    }

    [Fact]
    public void GetVolumeExpansionCost_UsesExactFullDomainVolumeAndClampsOnlyThePublicMetric()
    {
        var unit = FixedBoundBox.FromMinMax(Vector3d.Zero, new Vector3d(2, 2, 2));
        var contained = FixedBoundBox.FromMinMax(Vector3d.Zero, Vector3d.One);
        var expanded = FixedBoundBox.FromMinMax(Vector3d.Zero, new Vector3d(3, 3, 3));
        var point = FixedBoundBox.FromMinMax(Vector3d.Zero, Vector3d.Zero);
        var largeRepresentableCost = FixedBoundBox.FromMinMax(
            Vector3d.Zero,
            new Vector3d((Fixed64)2_000_000, (Fixed64)2_000_000, (Fixed64)2));
        var fullDomain = FixedBoundBox.FromMinMax(
            new Vector3d(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue),
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue));

        Assert.Equal(0L, unit.GetVolumeExpansionCost(contained));
        Assert.Equal(19L, unit.GetVolumeExpansionCost(expanded));
        Assert.Equal(8_000_000_000_000L, point.GetVolumeExpansionCost(largeRepresentableCost));
        Assert.Equal(long.MaxValue, unit.GetVolumeExpansionCost(fullDomain));
        Assert.Equal(0L, fullDomain.GetVolumeExpansionCost(unit));
    }

    [Fact]
    public void FindClosestPointsBetweenBoxes_ReturnsClosestPointOnFirstBoxSurface()
    {
        var a = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(2, 2, 2));
        var b = FixedBoundBox.FromCenterAndSize(new Vector3d(5, 0, 0), new Vector3d(2, 2, 2));

        var closestPoint = FixedBoundBox.FindClosestPointsBetweenBoxes(a, b);

        Assert.Equal(new Vector3d(1, -1, -1), closestPoint);
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void BoundingBox_NetSerialization_RoundTripMaintainsData()
    {
        FixedBoundBox originalValue = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<FixedBoundBox>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    [Fact]
    public void JsonSerialization_FullDomainState_PreservesExactDerivedCenter()
    {
        var originalValue = FixedBoundBox.FromMinMax(
            new Vector3d(Fixed64.FromRaw(long.MaxValue - 3), Fixed64.MinValue, -Fixed64.One),
            new Vector3d(Fixed64.FromRaw(long.MaxValue - 1), Fixed64.MaxValue, Fixed64.One));

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(originalValue);
        FixedBoundBox deserializedValue = JsonSerializer.Deserialize<FixedBoundBox>(json);

        Assert.Equal(originalValue, deserializedValue);
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(long.MaxValue - 2), Fixed64.Zero, Fixed64.Zero),
            deserializedValue.Center);
    }

    [Fact]
    public void BoundingBox_SetBoundingBox_JsonRoundTripPreservesBounds()
    {
        var originalValue = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        originalValue.SetBoundingBox(new Vector3d(1, 2, 3), new Vector3d(3, 4, 5));

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<FixedBoundBox>(json, jsonOptions);

        Assert.Equal(originalValue, deserializedValue);
        Assert.Equal(new Vector3d(-2, -2, -2), deserializedValue.Min);
        Assert.Equal(new Vector3d(4, 6, 8), deserializedValue.Max);
        Assert.Equal(new Vector3d(6, 8, 10), deserializedValue.Proportions);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void BoundingBox_MemoryPackSerialization_RoundTripMaintainsData()
    {
        FixedBoundBox originalValue = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedBoundBox deserializedValue = MemoryPackSerializer.Deserialize<FixedBoundBox>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    [Fact]
    public void BoundingBox_SetBoundingBox_MemoryPackRoundTripPreservesBounds()
    {
        var originalValue = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(2, 2, 2));
        originalValue.SetBoundingBox(new Vector3d(1, 2, 3), new Vector3d(3, 4, 5));

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedBoundBox deserializedValue = MemoryPackSerializer.Deserialize<FixedBoundBox>(bytes);

        Assert.Equal(originalValue, deserializedValue);
        Assert.Equal(new Vector3d(-2, -2, -2), deserializedValue.Min);
        Assert.Equal(new Vector3d(4, 6, 8), deserializedValue.Max);
        Assert.Equal(new Vector3d(6, 8, 10), deserializedValue.Proportions);
    }

    [Fact]
    public void MemoryPack_SerializedBox_RemainsMutable()
    {
        var box = FixedBoundBox.FromCenterAndSize(new Vector3d(0, 0, 0), new Vector3d(4, 4, 4));

        var bytes = MemoryPackSerializer.Serialize(box);
        var deserialized = MemoryPackSerializer.Deserialize<FixedBoundBox>(bytes);

        deserialized.Resize(new Vector3d(2, 2, 2));

        Assert.Equal(new Vector3d(2, 2, 2), deserialized.Proportions);
    }
#endif

    #endregion

    private static int CombineVectorPairHash(Vector3d min, Vector3d max)
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + min.StateHash;
            hash = (hash * 31) + max.StateHash;
            return hash;
        }
    }
}
