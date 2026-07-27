//=======================================================================
// FixedOrientedBox.CapsuleSlab.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxCapsuleSlabTests
{
    [Fact]
    public void ContactAnchors_CenterAxialTiesAndRetainRadialSupport()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCenteredCapsuleSlabContact(
            new Vector3d(
                Fixed64.FromFraction(5, 4),
                Fixed64.Zero,
                Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Forward,
            (Fixed64)2,
            Fixed64.Half,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(new Vector3d(1, -1, -1), contact.FirstAnchor.LocalPoint);
        Assert.Equal(
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero),
            contact.SecondAnchor.LocalPoint);
        Assert.Equal(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactAnchors_LiftOddPlanarEndpointResidualInto3D()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCenteredCapsuleSlabContact(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2),
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Two + Fixed64.MinIncrement,
            Fixed64.Half,
            Fixed64.One,
            out FixedContactAnchors contact));
        FixedPointAnchor capsuleAnchor = contact.SecondAnchor;
        var roundedOnly = new FixedPointAnchor(
            capsuleAnchor.Origin,
            capsuleAnchor.Rotation,
            capsuleAnchor.LocalPoint,
            capsuleAnchor.LocalDisplacement);

        Assert.NotEqual(
            0,
            capsuleAnchor.CompareLocalFeature(roundedOnly));
    }

    [Fact]
    public void ContactAnchors_RejectEndpointCornerSeparation()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.FromFraction(5, 2)),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));

        Assert.False(box.TryGetCenteredCapsuleSlabContact(
            Vector3d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            (Fixed64)2,
            Fixed64.Half,
            Fixed64.Half,
            out FixedContactAnchors separated));
        Assert.Equal(default, separated);
    }

    [Fact]
    public void ContactAnchors_RejectVerticalAndAxialSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetCenteredCapsuleSlabContact(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.Half,
            Fixed64.Half,
            out _));
        Assert.False(box.TryGetCenteredCapsuleSlabContact(
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.Half,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void ContactAnchors_RejectEndpointCornerGapAfterFaceOverlap()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetCenteredCapsuleSlabContact(
            new Vector3d(
                Fixed64.FromFraction(11, 5),
                Fixed64.Zero,
                Fixed64.FromFraction(6, 5)),
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.FromFraction(1, 4),
            Fixed64.One,
            out _));
    }

    [Fact]
    public void Contact_RejectsAnObliqueBoxFaceGapHiddenByWorldAxisProjections()
    {
        FixedQuaternion orientation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)20,
                (Fixed64)45,
                Fixed64.Zero);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            new Vector3d(
                Fixed64.Two,
                Fixed64.Two,
                Fixed64.FromFraction(1, 4)));
        Vector3d slabCenter = orientation * new Vector3d(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Two);

        Assert.False(box.TryGetCenteredCapsuleSlabContact(
            slabCenter,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 10),
            Fixed64.One,
            out _));
    }

    [Fact]
    public void Contact_RejectsARotatedBoxFaceGapForADegenerateCapsule()
    {
        FixedQuaternion orientation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)45,
                Fixed64.Zero,
                Fixed64.Zero);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            new Vector3d(
                Fixed64.Two,
                Fixed64.FromFraction(1, 4),
                Fixed64.Two));
        Vector3d slabCenter = orientation * new Vector3d(
            Fixed64.Zero,
            Fixed64.Two,
            Fixed64.Zero);

        Assert.False(box.TryGetCenteredCapsuleSlabContact(
            slabCenter,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.FromFraction(1, 10),
            Fixed64.One,
            out _));
    }

    [Fact]
    public void Contact_RejectsTheSeparatingCrossAxisBetweenSkewFiniteEdges()
    {
        FixedQuaternion orientation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)30,
                (Fixed64)35,
                (Fixed64)10);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            new Vector3d(
                Fixed64.Two,
                Fixed64.FromFraction(1, 10),
                Fixed64.FromFraction(1, 10)));
        Vector3d boxEdgeDirection = orientation * Vector3d.Right;
        Vector3d separationDirection =
            Vector3d.Cross(boxEdgeDirection, Vector3d.Right).Normalized;
        Vector3d slabCenter = separationDirection * Fixed64.Half;

        Assert.False(box.TryGetCenteredCapsuleSlabContact(
            slabCenter,
            Fixed64.Zero,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 10),
            out _));
    }

    [Fact]
    public void ContactAnchors_OrientTowardNegativeVerticalSlab()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCenteredCapsuleSlabContact(
            new Vector3d(
                Fixed64.Zero,
                -Fixed64.FromFraction(3, 2),
                Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.Half,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
    }

    [Fact]
    public void ContactAnchors_RemainRelativeAtScalarFaceAndClampDepth()
    {
        var scalarFace = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(scalarFace.TryGetCenteredCapsuleSlabContact(
            scalarFace.Center,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.One,
            out FixedContactAnchors boundary));
        Assert.True(boundary.Normal.MagnitudeSquared > Fixed64.Zero);

        var huge = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));
        Assert.True(huge.TryGetCenteredCapsuleSlabContact(
            Vector3d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out FixedContactAnchors clamped));
        Assert.Equal(Fixed64.MaxValue, clamped.Depth);
        Assert.True(clamped.DepthIsClamped);
    }

    [Fact]
    public void ContactAnchors_RejectInvalidCapsuleSlabInputs()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<ArgumentException>(() =>
            box.TryGetCenteredCapsuleSlabContact(
                Vector3d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCenteredCapsuleSlabContact(
                Vector3d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                -Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCenteredCapsuleSlabContact(
                Vector3d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                -Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCenteredCapsuleSlabContact(
                Vector3d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.One,
                Fixed64.Zero,
                out _));
    }
}
