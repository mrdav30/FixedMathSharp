//=======================================================================
// FixedPointAnchor.Distance.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedPointAnchorDistanceTests
{
    [Fact]
    public void CompareSquaredDistance_OrdersOrdinaryAndTiedPoints()
    {
        var reference = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var near = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right);
        var far = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right * (Fixed64)2);
        var tied = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Left);

        Assert.True(reference.CompareSquaredDistance(near, far) < 0);
        Assert.True(reference.CompareSquaredDistance(far, near) > 0);
        Assert.Equal(0, reference.CompareSquaredDistance(near, tied));
    }

    [Fact]
    public void CompareSquaredDistance_RetainsUnrepresentableRelativeOffsets()
    {
        var reference = new FixedPointAnchor(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var near = new FixedPointAnchor(
            new Vector3d(
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var far = new FixedPointAnchor(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.True(reference.CompareSquaredDistance(near, far) < 0);
    }

    [Fact]
    public void CompareSquaredDistance_RetainsExactSecondFeatureTerm()
    {
        var reference = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var representable = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up * Fixed64.MinIncrement);
        FixedPointAnchor halfRaw = FixedSegment
            .GetCenteredCapsuleSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.MinIncrement,
                Fixed64.Zero,
                Vector3d.Up);

        Assert.True(
            reference.CompareSquaredDistance(representable, halfRaw) > 0);
        Assert.True(
            reference.CompareSquaredDistance(halfRaw, representable) < 0);
    }

    [Fact]
    public void CompareSquaredDistance_RetainsExactReferenceFeatureTerm()
    {
        FixedPointAnchor halfRaw = FixedSegment
            .GetCenteredCapsuleSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.MinIncrement,
                Fixed64.Zero,
                Vector3d.Up);
        var near = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up * Fixed64.MinIncrement);
        var far = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Down * Fixed64.MinIncrement);

        Assert.True(halfRaw.CompareSquaredDistance(near, far) < 0);
    }

    [Fact]
    public void CompareSquaredDistance_ValidatesEveryAnchor()
    {
        var valid = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.Throws<InvalidOperationException>(() =>
            default(FixedPointAnchor).CompareSquaredDistance(valid, valid));
        Assert.Throws<ArgumentException>(() =>
            valid.CompareSquaredDistance(default, valid));
        Assert.Throws<ArgumentException>(() =>
            valid.CompareSquaredDistance(valid, default));
    }
}
