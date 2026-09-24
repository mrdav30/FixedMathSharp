using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedTriangleStrictClearanceTests
{
    private static readonly int[] TriangleIndices = { 0, 1, 2 };
    private static readonly int[] TriangleEdges = { 0, 1, 1, 2, 2, 0 };
    private static readonly Vector3d[] CornerTriangle =
    {
        new(0, 0, 0), new(-10, 0, 0), new(0, -10, 0)
    };
    private static readonly Vector3d[] InteriorTriangle =
    {
        new(-2, -2, 0), new(2, -2, 0), new(0, 2, 0)
    };

    [Theory]
    [InlineData(0, -1L, false)]
    [InlineData(0, 0L, false)]
    [InlineData(0, 1L, true)]
    [InlineData(2, -1L, false)]
    [InlineData(2, 0L, false)]
    [InlineData(2, 1L, true)]
    public void StrictOverlap_ClassifiesTriangleVertexSubrawPenetration(
        int axisLength,
        long rawStep,
        bool expected)
    {
        Fixed64 step = Fixed64.FromRaw(rawStep);
        Vector3d center = new((Fixed64)3 + step, (Fixed64)4 - step, Fixed64.Zero);
        // The closest source core point has z=0 and the nearest triangle
        // vertex is zero. Its exact squared distance is 25 - 2*u + 2*u^2.
        Assert.Equal(expected, WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            Vector3d.Zero, FixedQuaternion.Identity,
            CornerTriangle, TriangleIndices, TriangleEdges,
            center, FixedQuaternion.Identity, Vector3d.Forward,
            (Fixed64)axisLength, (Fixed64)5));
    }

    [Theory]
    [InlineData(0, -1L, false)]
    [InlineData(0, 0L, false)]
    [InlineData(0, 1L, true)]
    [InlineData(2, -1L, false)]
    [InlineData(2, 0L, false)]
    [InlineData(2, 1L, true)]
    public void StrictOverlap_PreservesFaceTangencyInArbitraryRigidFrame(
        int axisLength,
        long radiusRawStep,
        bool expected)
    {
        Vector3d[] points = { new(1, -8, -8), new(1, 8, -8), new(1, 0, 8) };
        FixedQuaternion rotation = new FixedQuaternion(
            Fixed64.One, Fixed64.Two, (Fixed64)3, (Fixed64)4).Normalized;
        Vector3d origin = new(5, 7, 11);
        Fixed64 radius = Fixed64.One + Fixed64.FromRaw(radiusRawStep);
        // Both frames share the exact rational rotation. In that frame the
        // core is on x=0 and the triangle is on x=1, with its nearest face
        // projection in the triangle interior; no world witness is rounded.
        Assert.Equal(expected, WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            origin, rotation, points, TriangleIndices, TriangleEdges,
            origin, rotation, Vector3d.Up, (Fixed64)axisLength, radius));
    }

    [Fact]
    public void StrictOverlap_ZeroRadiusPointOnTriangleInteriorIsOnlyTouching()
    {
        Assert.False(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            Vector3d.Zero, FixedQuaternion.Identity,
            InteriorTriangle, TriangleIndices, TriangleEdges,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Forward,
            Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void StrictOverlap_ZeroRadiusCoplanarSegmentIsOnlyTouching()
    {
        Assert.False(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            Vector3d.Zero, FixedQuaternion.Identity,
            InteriorTriangle, TriangleIndices, TriangleEdges,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Fixed64.Two, Fixed64.Zero));
    }

    [Fact]
    public void StrictOverlap_ZeroRadiusTransverseSegmentPiercesTriangleInterior()
    {
        // Triangle-minus-segment has a 3D interior here: unlike a coplanar
        // segment, this crossing requires positive translation to separate.
        Assert.True(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            Vector3d.Zero, FixedQuaternion.Identity,
            InteriorTriangle, TriangleIndices, TriangleEdges,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Forward,
            Fixed64.Two, Fixed64.Zero));
    }

    [Fact]
    public void StrictOverlap_ZeroRadiusTransverseSegmentAtVertexIsOnlyTouching()
    {
        Assert.False(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            Vector3d.Zero, FixedQuaternion.Identity,
            InteriorTriangle, TriangleIndices, TriangleEdges,
            new Vector3d(2, -2, 0), FixedQuaternion.Identity, Vector3d.Forward,
            Fixed64.Two, Fixed64.Zero));
    }
}
