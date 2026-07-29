//=======================================================================
// FixedPointAnchor2d.Distance.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class FixedPointAnchor2dDistanceTests
{
    [Fact]
    public void CompareSquaredDistance_OrdersOrdinaryAndTiedPoints()
    {
        var reference = new FixedPointAnchor2d(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero);
        var near = new FixedPointAnchor2d(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right);
        var far = new FixedPointAnchor2d(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right * Fixed64.Two);
        var tied = new FixedPointAnchor2d(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Left);

        Assert.True(reference.CompareSquaredDistance(near, far) < 0);
        Assert.True(reference.CompareSquaredDistance(far, near) > 0);
        Assert.Equal(0, reference.CompareSquaredDistance(near, tied));
    }

    [Fact]
    public void CompareSquaredDistance_RetainsUnrepresentableRelativeOffsets()
    {
        var reference = new FixedPointAnchor2d(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Zero);
        var near = new FixedPointAnchor2d(
            new Vector2d(
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Zero);
        var far = new FixedPointAnchor2d(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Zero);

        Assert.True(reference.CompareSquaredDistance(near, far) < 0);
    }

    [Fact]
    public void CompareSquaredDistance_RetainsExactFeatureTerms()
    {
        var reference = new FixedPointAnchor2d(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero);
        var representable = new FixedPointAnchor2d(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Forward * Fixed64.MinIncrement);
        FixedPointAnchor2d halfRaw =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Forward,
                Fixed64.MinIncrement,
                Fixed64.Zero,
                Vector2d.Forward);

        Assert.True(
            reference.CompareSquaredDistance(representable, halfRaw) > 0);
        Assert.True(
            reference.CompareSquaredDistance(halfRaw, representable) < 0);
    }
}
