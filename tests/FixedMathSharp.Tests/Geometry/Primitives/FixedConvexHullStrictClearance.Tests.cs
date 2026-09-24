using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedConvexHullStrictClearanceTests
{
    private static readonly Vector3d[] CubePoints =
    {
        new(-1, -1, -1), new(1, -1, -1), new(1, 1, -1), new(-1, 1, -1),
        new(-1, -1, 1), new(1, -1, 1), new(1, 1, 1), new(-1, 1, 1)
    };

    private static readonly int[] CubeTriangles =
    {
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
        0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5,
        0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2
    };

    private static readonly int[] CubeEdges =
    {
        0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7
    };

    [Theory]
    [InlineData(0, -1L, false)]
    [InlineData(0, 0L, false)]
    [InlineData(0, 1L, true)]
    [InlineData(2, -1L, false)]
    [InlineData(2, 0L, false)]
    [InlineData(2, 1L, true)]
    public void StrictOverlap_ClassifiesSubrawEdgePenetrationBeforeRounding(
        int axisLength,
        long rawStep,
        bool expected)
    {
        Fixed64 step = Fixed64.FromRaw(rawStep);
        Vector3d center = new((Fixed64)4 + step, (Fixed64)5 - step, Fixed64.Zero);

        // Distance squared to the cube edge is 25 - 2*step + 2*step^2.
        // The positive-step penetration is less than half a raw unit.
        Assert.Equal(expected, StrictlyOverlaps(
            Vector3d.Zero, FixedQuaternion.Identity, center, (Fixed64)axisLength, (Fixed64)5));
        if (rawStep >= 0)
        {
            Assert.True(GetContact(Vector3d.Zero, FixedQuaternion.Identity, center,
                (Fixed64)axisLength, (Fixed64)5, out FixedContactAnchors contact));
            Assert.Equal(Fixed64.Zero, contact.Depth);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void StrictOverlap_AdmitsShapeWhollyInsideConvexHull(int axisLength)
    {
        Assert.True(StrictlyOverlaps(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, (Fixed64)axisLength, Fixed64.FromFraction(1, 4)));
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, false)]
    [InlineData(1L, true)]
    public void StrictOverlap_PreservesExactQuarterTurn(long rawStep, bool expected)
    {
        Fixed64 step = Fixed64.FromRaw(rawStep);
        FixedQuaternion rotation = new FixedQuaternion(
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        // The normalized equal Z/W components represent the exact rational
        // quarter-turn (-y, x, z), independently of their common raw magnitude.
        Vector3d center = new(-(Fixed64)5 + step, (Fixed64)4 + step, Fixed64.Zero);
        Assert.Equal(expected, StrictlyOverlaps(
            Vector3d.Zero, rotation, center, Fixed64.Two, (Fixed64)5));
    }

    [Theory]
    [InlineData(false, -1L, false)]
    [InlineData(false, 0L, false)]
    [InlineData(false, 1L, true)]
    [InlineData(true, -1L, false)]
    [InlineData(true, 0L, false)]
    [InlineData(true, 1L, true)]
    public void StrictOverlap_DoesNotMaterializeOutOfDomainHullVertices(
        bool maximumFace,
        long rawStep,
        bool expected)
    {
        Fixed64 limit = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 sign = maximumFace ? -Fixed64.One : Fixed64.One;
        Fixed64 step = Fixed64.FromRaw(rawStep);
        Vector3d origin = new(limit, limit, Fixed64.Zero);
        Vector3d center = new(
            limit + sign * ((Fixed64)4 + step),
            limit + sign * ((Fixed64)5 - step),
            Fixed64.Zero);
        Assert.Equal(expected, StrictlyOverlaps(
            origin, FixedQuaternion.Identity, center, Fixed64.Two, (Fixed64)5));
    }

    [Fact]
    public void StrictOverlap_DoesNotChangeOrdinaryContactAxisOrDepth()
    {
        Vector3d center = new(Fixed64.One + Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        Assert.True(StrictlyOverlaps(Vector3d.Zero, FixedQuaternion.Identity,
            center, Fixed64.Two, Fixed64.One));
        Assert.True(GetContact(Vector3d.Zero, FixedQuaternion.Identity,
            center, Fixed64.Two, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
    }

    [Fact]
    public void StrictOverlap_DoesNotAllocateAfterWarmup()
    {
        Vector3d center = new(Fixed64.One + Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        for (int index = 0; index < 8; index++)
            _ = StrictlyOverlaps(Vector3d.Zero, FixedQuaternion.Identity,
                center, Fixed64.Two, Fixed64.One);

        long before = GC.GetAllocatedBytesForCurrentThread();
        int overlaps = 0;
        for (int index = 0; index < 8; index++)
        {
            if (StrictlyOverlaps(Vector3d.Zero, FixedQuaternion.Identity,
                    center, Fixed64.Two, Fixed64.One))
                overlaps++;
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(8, overlaps);
        Assert.Equal(0L, allocated);
    }

    private static bool StrictlyOverlaps(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d center,
        Fixed64 axisLength,
        Fixed64 radius) =>
        WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            origin, rotation, CubePoints, CubeTriangles, CubeEdges,
            center, rotation, Vector3d.Forward, axisLength, radius);

    private static bool GetContact(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d center,
        Fixed64 axisLength,
        Fixed64 radius,
        out FixedContactAnchors contact) =>
        WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(
            origin, rotation, CubePoints, CubeTriangles, CubeEdges,
            center, rotation, Vector3d.Forward, axisLength, radius, out contact);
}
