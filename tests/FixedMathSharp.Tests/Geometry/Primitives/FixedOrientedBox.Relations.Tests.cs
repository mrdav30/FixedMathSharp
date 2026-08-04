//=======================================================================
// FixedOrientedBox.Relations.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxRelationTests
{
    [Fact]
    public void SphereContact_UsesCenterRelativeSurfaceOffsets()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetSphereContact(
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, GetOffset(contact.FirstAnchor));
        Assert.Equal(-Vector3d.Right, GetOffset(contact.SecondAnchor));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void SphereContact_IsBoundaryInclusiveAndRejectsExactSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetSphereContact(
            new Vector3d(2, 0, 0),
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors tangent));
        Assert.Equal(Fixed64.Zero, tangent.Depth);
        Assert.False(box.TryGetSphereContact(
            new Vector3d(Fixed64.FromRaw((2L << 32) + 1L), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors separated));
        Assert.Equal(default, separated);
    }

    [Fact]
    public void SphereContact_RotatedThinBox_PreservesSubRawProjectedOverlap()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.FromFraction(5, 2),
            Fixed64.Zero);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            rotation,
            new Vector3d(
                (Fixed64)3,
                Fixed64.Half,
                Fixed64.FromFraction(1, 10)));
        Vector3d sphereCenter = rotation * new Vector3d(
            Fixed64.FromFraction(16, 5),
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(box.TryGetSphereContact(
            sphereCenter,
            FixedQuaternion.Identity,
            Fixed64.FromFraction(401, 2000),
            out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void SphereContact_ContainedSphereUsesNearestFaceAndClampsConceptualDepth()
    {
        var ordinary = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, 3, 4));

        Assert.True(ordinary.TryGetSphereContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors contained));
        Assert.Equal(new Vector3d(2, 0, 0), GetOffset(contained.FirstAnchor));
        Assert.Equal(-Vector3d.Right, GetOffset(contained.SecondAnchor));
        Assert.Equal(Vector3d.Right, contained.Normal);
        Assert.Equal((Fixed64)3, contained.Depth);
        Assert.False(contained.DepthIsClamped);

        var fullDomain = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));
        Assert.True(fullDomain.TryGetSphereContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Fixed64.MaxValue,
            out FixedContactAnchors clamped));
        Assert.Equal(Fixed64.MaxValue, clamped.Depth);
        Assert.True(clamped.DepthIsClamped);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void SphereContact_ContainedSphereUsesEverySignedNearestFace(
        int axis,
        bool negative)
    {
        Vector3d halfExtents = axis switch
        {
            0 => new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3),
            1 => new Vector3d((Fixed64)3, Fixed64.One, (Fixed64)2),
            _ => new Vector3d((Fixed64)2, (Fixed64)3, Fixed64.One)
        };
        Fixed64 coordinate = negative ? -Fixed64.Half : Fixed64.Half;
        Vector3d sphereCenter = axis switch
        {
            0 => new Vector3d(coordinate, Fixed64.Zero, Fixed64.Zero),
            1 => new Vector3d(Fixed64.Zero, coordinate, Fixed64.Zero),
            _ => new Vector3d(Fixed64.Zero, Fixed64.Zero, coordinate)
        };
        Vector3d expectedNormal = axis switch
        {
            0 => negative ? -Vector3d.Right : Vector3d.Right,
            1 => negative ? -Vector3d.Up : Vector3d.Up,
            _ => negative ? -Vector3d.Forward : Vector3d.Forward
        };
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            halfExtents);

        Assert.True(box.TryGetSphereContact(
            sphereCenter,
            FixedQuaternion.Identity,
            Fixed64.FromFraction(1, 4),
            out FixedContactAnchors contact));

        Assert.Equal(expectedNormal, contact.Normal);
        Assert.Equal(expectedNormal * halfExtents[axis], GetOffset(contact.FirstAnchor));
        Assert.Equal(-expectedNormal * Fixed64.FromFraction(1, 4), GetOffset(contact.SecondAnchor));
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void SphereContact_OutsideCornerUsesRadialDepth()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var sphereCenter = new Vector3d(
            Fixed64.FromFraction(3, 2),
            Fixed64.FromFraction(3, 2),
            Fixed64.Zero);

        Assert.True(box.TryGetSphereContact(
            sphereCenter,
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors contact));

        Fixed64 expectedDepth = Fixed64.One
            - FixedMath.Sqrt(Fixed64.Half);
        Assert.True((contact.Depth - expectedDepth).Abs() <= Fixed64.MinIncrement);
        Assert.True(contact.Normal.X > Fixed64.Zero);
        Assert.True(contact.Normal.Y > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
    }

    [Fact]
    public void SphereContact_RemainsAuthoritativeAtTheScalarFace()
    {
        var box = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetSphereContact(
            box.Center,
            FixedQuaternion.Identity,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Vector3d.Right, GetOffset(contact.FirstAnchor));
        Assert.Equal(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero), GetOffset(contact.SecondAnchor));
        Assert.Equal(Fixed64.FromFraction(3, 2), contact.Depth);
    }

    [Fact]
    public void SphereContact_UsesStableFaceTiesAndRetainsUnmaterializableAnchors()
    {
        var tied = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(tied.TryGetSphereContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Fixed64.Zero,
            out FixedContactAnchors zeroRadius));
        Assert.Equal(Vector3d.Right, zeroRadius.Normal);
        Assert.Equal(Vector3d.Right, GetOffset(zeroRadius.FirstAnchor));
        Assert.Equal(Vector3d.Zero, GetOffset(zeroRadius.SecondAnchor));

        FixedQuaternion yaw = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero);
        var outside = new FixedOrientedBox(
            Vector3d.Zero,
            yaw,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));
        Assert.True(outside.TryGetSphereContact(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.Zero,
            out FixedContactAnchors fullDomain));
        Assert.False(fullDomain.FirstAnchor.TryGetPoint(out _));
        Assert.True(fullDomain.SecondAnchor.TryGetPoint(out _));
        Assert.NotEqual(Vector3d.Zero, fullDomain.Normal);
    }

    [Fact]
    public void SphereContact_RejectsNegativeRadius()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetSphereContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                -Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetSphereContact(
                Vector3d.Zero,
                default,
                Fixed64.One,
                out _));
    }

    [Fact]
    public void SphereContact_PreservesSphereLocalFeatureAcrossRigidPoses()
    {
        FixedQuaternion relativeRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)13,
                (Fixed64)(-27),
                (Fixed64)9);
        var ordinary = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d ordinarySphereCenter = new(
            Fixed64.FromFraction(3, 2),
            Fixed64.Zero,
            Fixed64.Zero);
        Assert.True(ordinary.TryGetSphereContact(
            ordinarySphereCenter,
            relativeRotation,
            Fixed64.One,
            out FixedContactAnchors ordinaryContact));

        FixedQuaternion commonRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)(-11),
                (Fixed64)31,
                (Fixed64)17);
        Vector3d translation = new(7, -5, 3);
        var transformed = new FixedOrientedBox(
            translation,
            commonRotation,
            Vector3d.One);
        Assert.True(commonRotation.TryRotate(
            ordinarySphereCenter,
            out Vector3d rotatedSphereCenter));
        Assert.True(transformed.TryGetSphereContact(
            translation + rotatedSphereCenter,
            commonRotation * relativeRotation,
            Fixed64.One,
            out FixedContactAnchors transformedContact));

        Assert.True(Vector3d.Distance(
            ordinaryContact.SecondAnchor.LocalPoint,
            transformedContact.SecondAnchor.LocalPoint)
            <= Fixed64.Epsilon);
        Assert.True(Vector3d.Distance(
            ordinaryContact.SecondAnchor.LocalDisplacement,
            transformedContact.SecondAnchor.LocalDisplacement)
            <= Fixed64.Epsilon);
    }

    private static Vector3d GetOffset(FixedPointAnchor anchor)
    {
        var origin = new FixedPointAnchor(
            anchor.Origin,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Assert.True(anchor.TryGetOffsetFrom(origin, out Vector3d offset));
        return offset;
    }
}

