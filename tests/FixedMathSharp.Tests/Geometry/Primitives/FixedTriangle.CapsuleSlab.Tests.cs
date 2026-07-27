//=======================================================================
// FixedTriangle.CapsuleSlab.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleCapsuleSlabTests
{
    [Fact]
    public void CircleSlab_ReturnsCanonicalContact()
    {
        FixedTriangle triangle = CreateVerticalTriangle();

        Assert.True(triangle.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.Origin);
        Assert.Equal(
            new Vector3d(
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Zero),
            contact.SecondAnchor.Origin);
    }

    [Fact]
    public void CircleSlab_FaceNormalUsesTangentiallyClosestTriangleWitness()
    {
        var triangle = new FixedTriangle(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)(-2)),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Two),
            new Vector3d(Fixed64.Two, Fixed64.Zero, (Fixed64)(-2)));

        Assert.True(triangle.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
    }

    [Fact]
    public void CircleSlab_RejectsRadialAndVerticalSeparation()
    {
        FixedTriangle triangle = CreateVerticalTriangle();

        Assert.False(triangle.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.FromFraction(5, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(triangle.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.Zero,
                (Fixed64)3,
                Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CircleSlab_ScalarFaceRetainsUnmaterializedTriangleWitness(
        bool maximumFace)
    {
        Fixed64 face =
            maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 outward = maximumFace
            ? Fixed64.One
            : -Fixed64.One;
        var triangle = new FixedTriangle(
            new Vector3d(outward, -Fixed64.One, -Fixed64.One),
            new Vector3d(outward, Fixed64.One, -Fixed64.One),
            new Vector3d(outward, Fixed64.Zero, Fixed64.One));

        Assert.True(triangle.TryGetCircleSlabContact(
            new Vector3d(face, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(face, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Two,
            out FixedContactAnchors contact));
        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.Equal(
            new Vector3d(face, Fixed64.Zero, Fixed64.Zero),
            contact.FirstAnchor.Origin);
    }

    [Fact]
    public void CenteredCapsuleSlab_OddAxisRetainsExactEndpointFeature()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        Fixed64 oddAxisLength =
            Fixed64.Two + Fixed64.MinIncrement;

        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Fixed64.Zero,
            Vector2d.Forward,
            oddAxisLength,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors contact));
        FixedPointAnchor capsuleAnchor = contact.SecondAnchor;
        var roundedOnly = new FixedPointAnchor(
            capsuleAnchor.Origin,
            capsuleAnchor.Rotation,
            capsuleAnchor.LocalPoint,
            capsuleAnchor.LocalDisplacement);

        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.NotEqual(
            0,
            capsuleAnchor.CompareLocalFeature(roundedOnly));
    }

    [Fact]
    public void ReversedTriangleOrder_PreservesCircleSlabResponse()
    {
        FixedTriangle forward = CreateVerticalTriangle();
        var reversed = new FixedTriangle(
            forward.A,
            forward.C,
            forward.B);

        Assert.True(forward.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors first));
        Assert.True(reversed.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors second));

        Assert.Equal(first.Normal, second.Normal);
        Assert.Equal(first.Depth, second.Depth);
        Assert.Equal(first.DepthIsClamped, second.DepthIsClamped);
    }

    [Fact]
    public void DeepDiagonalCircleSlab_ExposesClampedDepth()
    {
        var triangle = new FixedTriangle(
            new Vector3d(
                -Fixed64.MaxValue,
                Fixed64.MaxValue,
                -Fixed64.One),
            new Vector3d(
                Fixed64.MaxValue,
                -Fixed64.MaxValue,
                -Fixed64.One),
            new Vector3d(
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One));

        Assert.True(triangle.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void WarmedCircleAndCapsuleSlabContacts_DoNotAllocate()
    {
        FixedTriangle triangle = CreateVerticalTriangle();
        Vector3d center =
            new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        _ = triangle.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            center,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _);
        _ = triangle.TryGetCenteredCapsuleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            center,
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.One,
            out _);

        long before = GC.GetAllocatedBytesForCurrentThread();
        bool allSucceeded = true;
        for (int iteration = 0; iteration < 32; iteration++)
        {
            allSucceeded &= triangle.TryGetCircleSlabContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                center,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _);
            allSucceeded &= triangle.TryGetCenteredCapsuleSlabContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                center,
                Fixed64.Zero,
                Vector2d.Forward,
                Fixed64.Two,
                Fixed64.One,
                Fixed64.One,
                out _);
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.True(allSucceeded);
        Assert.Equal(before, after);
    }

    [Fact]
    public void CapsuleSlab_RejectsSeparationOnItsPlanarNormal()
    {
        var triangle = new FixedTriangle(
            new Vector3d((Fixed64)3, Fixed64.Zero, (Fixed64)(-2)),
            new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)(-2)),
            new Vector3d((Fixed64)3, Fixed64.Zero, (Fixed64)2));

        Assert.False(triangle.TryGetCenteredCapsuleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CapsuleSlab_RejectsSeparationBeyondItsEndpoint()
    {
        var triangle = new FixedTriangle(
            new Vector3d((Fixed64)3, Fixed64.Zero, -Fixed64.One),
            new Vector3d((Fixed64)4, Fixed64.Zero, -Fixed64.One),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.One));

        Assert.False(triangle.TryGetCenteredCapsuleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.Half,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CircleSlab_RejectsSeparationOnTriangleEdgeNormal()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Two));

        Assert.False(triangle.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.FromFraction(3, 2)),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.FromFraction(1, 4),
            out _));
    }

    [Fact]
    public void CapsuleSlab_RejectsSkewEdgeSeparation()
    {
        FixedTriangle triangle = CreateVerticalTriangle();

        Assert.False(triangle.TryGetCenteredCapsuleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(11, 10)),
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.FromFraction(1, 4),
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CapsuleSlab_RejectsEndpointVertexRadialGap()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Two));

        Assert.False(triangle.TryGetCenteredCapsuleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.FromFraction(16, 5),
                Fixed64.Zero,
                -Fixed64.FromFraction(1, 5)),
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.FromFraction(1, 4),
            Fixed64.One,
            out _));
    }

    [Fact]
    public void DegenerateTriangle_ReturnsNoCapsuleSlabContact()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Right);

        Assert.False(triangle.TryGetCircleSlabContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CapsuleSlabContracts_RejectInvalidGeometry()
    {
        FixedTriangle triangle = CreateVerticalTriangle();

        Assert.Throws<ArgumentException>(() =>
            triangle.TryGetCenteredCapsuleSlabContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetCenteredCapsuleSlabContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                -Fixed64.MinIncrement,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetCircleSlabContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                -Fixed64.MinIncrement,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetCircleSlabContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out _));
    }

    private static FixedTriangle CreateVerticalTriangle() =>
        new(
            new Vector3d(Fixed64.Zero, -Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One));
}
