//=======================================================================
// FixedOrientedBox.CircleSlab.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxCircleSlabTests
{
    [Fact]
    public void ContactAnchors_UseNearestRelativeBoxFeatureAndRadialSupport()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabContact(
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, contact.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(0, -1, 0), contact.SecondAnchor.LocalPoint);
        Assert.Equal(Vector3d.Left, contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-1, 0, 0)]
    [InlineData(0, -1, 0)]
    [InlineData(0, 0, -1)]
    [InlineData(0, 0, 1)]
    public void ContactAnchors_OrientTowardAxisAlignedSlabCenters(
        int x,
        int y,
        int z)
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d direction = new(
            (Fixed64)x,
            (Fixed64)y,
            (Fixed64)z);

        Assert.True(box.TryGetCircleSlabContact(
            direction * Fixed64.FromFraction(3, 2),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(direction, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
    }

    [Theory]
    [InlineData(-1, 0, 0)]
    [InlineData(0, -1, 0)]
    [InlineData(0, 0, -1)]
    [InlineData(0, 0, 1)]
    public void ContactAnchors_ForContainedSlabCenter_SelectSignedNearestFace(
        int x,
        int y,
        int z)
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var direction = new Vector3d(
            (Fixed64)x,
            (Fixed64)y,
            (Fixed64)z);

        Assert.True(box.TryGetCircleSlabContact(
            direction * Fixed64.Half,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(direction, contact.Normal);
        Assert.Equal(direction, contact.FirstAnchor.LocalPoint);
        Assert.Equal(Fixed64.FromFraction(3, 2), contact.Depth);
    }

    [Fact]
    public void ContactAnchors_UseCornerAxisForDiagonalSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetCircleSlabContact(
            new Vector3d(2, 0, 2),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors separated));
        Assert.Equal(default, separated);
        Assert.True(box.TryGetCircleSlabContact(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.FromFraction(3, 2)),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors diagonal));
        Assert.True(diagonal.Normal.X > Fixed64.Zero);
        Assert.True(diagonal.Normal.Z > Fixed64.Zero);
        Assert.True(diagonal.Depth > Fixed64.Zero);
    }

    [Fact]
    public void ContactAnchors_NearTangentialFaceAxisShouldRetainNearestBoxFeature()
    {
        var orientation = new FixedQuaternion(
            Fixed64.Zero,
            (Fixed64)(-0.03022970794700086),
            Fixed64.Zero,
            (Fixed64)0.9995429799892008);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            new Vector3d(
                (Fixed64)3,
                Fixed64.Half,
                Fixed64.FromFraction(1, 10)));
        var slabCenter = new Vector3d(
            (Fixed64)2.8977774793747813,
            Fixed64.Zero,
            (Fixed64)0.7764571344014257);

        Assert.True(box.TryGetCircleSlabContact(
            slabCenter,
            Fixed64.Zero,
            Fixed64.Half,
            Fixed64.Half,
            out FixedContactAnchors contact));

        Assert.True(contact.FirstAnchor.LocalPoint.X > Fixed64.Zero);
        var slabCenterAnchor = new FixedPointAnchor(
            slabCenter,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var oppositeBoxFeature = new FixedPointAnchor(
            Vector3d.Zero,
            orientation,
            new Vector3d(
                (Fixed64)(-3),
                -Fixed64.Half,
                Fixed64.FromFraction(1, 10)));
        Assert.True(
            slabCenterAnchor.CompareSquaredDistance(
                contact.FirstAnchor,
                oppositeBoxFeature) < 0);
    }

    [Fact]
    public void RotatedDiagonalTangency_DoesNotReportNegativeDepth()
    {
        FixedQuaternion orientation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)13,
                (Fixed64)(-29),
                (Fixed64)7);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            Vector3d.One);
        Vector3d slabCenter =
            orientation * new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2);
        Fixed64 tangentRadius =
            FixedMath.Sqrt(Fixed64.Two) + Fixed64.MinIncrement;

        Assert.True(box.TryGetCircleSlabContact(
            slabCenter,
            Fixed64.Zero,
            Fixed64.One,
            tangentRadius,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth >= Fixed64.Zero);
    }

    [Fact]
    public void DiagonalRadialSupport_RetainsItsExactLocalFeature()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabContact(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.FromFraction(3, 2)),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One + Fixed64.MinIncrement,
            out FixedContactAnchors contact));
        FixedPointAnchor radialSupport = contact.SecondAnchor;
        var roundedOnly = new FixedPointAnchor(
            radialSupport.Origin,
            radialSupport.Rotation,
            radialSupport.LocalPoint,
            radialSupport.LocalDisplacement);

        Assert.NotEqual(
            0,
            radialSupport.CompareLocalFeature(roundedOnly));
    }

    [Fact]
    public void ContactAnchors_RejectClosestFeatureDiagonalGap()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromFraction(9, 10),
                Fixed64.Zero,
                Fixed64.FromFraction(9, 10)),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));

        Assert.False(box.TryGetCircleSlabContact(
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.Half,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ContactAnchors_RejectVerticalSeparationAndRemainRelativeAtScalarFace()
    {
        var origin = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.False(origin.TryGetCircleSlabContact(
            new Vector3d(0, 3, 0),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));

        var scalarFace = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(scalarFace.TryGetCircleSlabContact(
            scalarFace.Center,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Vector3d.Up, contact.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(0, -1, 0), contact.SecondAnchor.LocalPoint);
        Assert.Equal(scalarFace.Center, contact.FirstAnchor.Origin);
    }

    [Fact]
    public void ContactAnchors_ExposeClampedDepth()
    {
        var huge = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));

        Assert.True(huge.TryGetCircleSlabContact(
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactAnchors_AcrossOppositeScalarFaces_RetainTangency()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromRaw(long.MaxValue - 1L),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One));

        Assert.True(box.TryGetCircleSlabContact(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d firstPoint));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));
        Assert.Equal(
            -Fixed64.MinIncrement,
            firstPoint.X);
        Assert.True(contact.SecondAnchor.TryGetProjectedOffsetFrom(
            contact.FirstAnchor,
            contact.Normal,
            out Fixed64 separation));
        Assert.Equal(Fixed64.Zero, separation);
    }

    [Fact]
    public void ContactAnchors_RejectInvalidSlabInputs()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCircleSlabContact(
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCircleSlabContact(
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                -Fixed64.One,
                out _));
    }
}
