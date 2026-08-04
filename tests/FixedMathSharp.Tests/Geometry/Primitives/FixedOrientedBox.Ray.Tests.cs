using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxRayTests
{
    [Fact]
    public void RayInterval_ReturnsClosedEntryAndExit()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var ray = new FixedRay(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right);

        Assert.True(box.TryGetRayIntersectionInterval(
            ray,
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal((Fixed64)3, exit);
    }

    [Fact]
    public void RayInterval_UsesCanonicalRotationAtScalarBoundary()
    {
        Fixed64 centerX =
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4);
        var box = new FixedOrientedBox(
            new Vector3d(centerX, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)45,
                Fixed64.Zero),
            Vector3d.One);
        var ray = new FixedRay(
            new Vector3d(
                centerX - (Fixed64)2,
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Right);

        Assert.True(box.TryGetRayIntersectionInterval(
            ray,
            Fixed64.Two,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.True(entry > Fixed64.Zero);
        Assert.Equal(Fixed64.Two, exit);
        Assert.True(ray.TryGetPoint(entry, out Vector3d entryPoint));
        Assert.True(box.Contains(entryPoint));
        Assert.True(ray.TryGetPoint(exit, out Vector3d exitPoint));
        Assert.True(box.Contains(exitPoint));
    }

    [Fact]
    public void RayInterval_HandlesPointQueriesAndClosedTangency()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetRayIntersectionInterval(
            new FixedRay(Vector3d.One, Vector3d.Zero),
            Fixed64.Zero,
            out Fixed64 containedEntry,
            out Fixed64 containedExit));
        Assert.Equal(Fixed64.Zero, containedEntry);
        Assert.Equal(Fixed64.Zero, containedExit);

        Assert.False(box.TryGetRayIntersectionInterval(
            new FixedRay(Vector3d.One * Fixed64.Two, Vector3d.Zero),
            Fixed64.One,
            out Fixed64 missedEntry,
            out Fixed64 missedExit));
        Assert.Equal(default, missedEntry);
        Assert.Equal(default, missedExit);

        Assert.True(box.TryGetRayIntersectionInterval(
            new FixedRay(
                new Vector3d((Fixed64)(-2), Fixed64.One, Fixed64.Zero),
                Vector3d.Right),
            (Fixed64)4,
            out Fixed64 tangentEntry,
            out Fixed64 tangentExit));
        Assert.Equal(Fixed64.One, tangentEntry);
        Assert.Equal((Fixed64)3, tangentExit);
    }

    [Fact]
    public void RayInterval_RejectsSeparatedAndNegativeRange()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetRayIntersectionInterval(
            new FixedRay(
                new Vector3d((Fixed64)(-2), (Fixed64)2, Fixed64.Zero),
                Vector3d.Right),
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(default, entry);
        Assert.Equal(default, exit);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetRayIntersectionInterval(
                new FixedRay(Vector3d.Zero, Vector3d.Right),
                -Fixed64.MinIncrement,
                out _,
                out _));
    }

    [Fact]
    public void RayInterval_HandlesNegativeDirectionsAndEverySeparatedAxis()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetRayIntersectionInterval(
            new FixedRay(
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                -Vector3d.Right),
            (Fixed64)4,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.One, entry);
        Assert.Equal((Fixed64)3, exit);

        Assert.False(box.TryGetRayIntersectionInterval(
            new FixedRay(
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero),
                Vector3d.Right),
            (Fixed64)4,
            out _,
            out _));
        Assert.False(box.TryGetRayIntersectionInterval(
            new FixedRay(
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2),
                Vector3d.Right),
            (Fixed64)4,
            out _,
            out _));
    }

    [Fact]
    public void RayInterval_RejectsIntersectionsOutsideTheRequestedRange()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetRayIntersectionInterval(
            new FixedRay(
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right),
            (Fixed64)4,
            out _,
            out _));
        Assert.False(box.TryGetRayIntersectionInterval(
            new FixedRay(
                new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right),
            Fixed64.Half,
            out _,
            out _));
    }
}
