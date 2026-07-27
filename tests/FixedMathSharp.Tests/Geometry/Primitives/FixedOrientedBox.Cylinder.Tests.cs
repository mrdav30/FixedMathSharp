//=======================================================================
// FixedOrientedBox.Cylinder.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxCylinderTests
{
    [Fact]
    public void ContactAnchors_UseFiniteCylinderSupport()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(
                Fixed64.FromFraction(5, 4),
                Fixed64.Zero,
                Fixed64.Zero),
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Forward),
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.Half,
            out FixedContactAnchors contact));

        Assert.Equal(new Vector3d(1, -1, -1), contact.FirstAnchor.LocalPoint);
        Assert.Equal(
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero),
            contact.SecondAnchor.LocalPoint);
        Assert.Equal(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            contact.SecondAnchor.LocalDisplacement);
        Assert.True(contact.SecondAnchor.TryGetPoint(
            out Vector3d cylinderPoint));
        Assert.Equal(
            new Vector3d(
                Fixed64.FromFraction(3, 4),
                Fixed64.Zero,
                -Fixed64.One),
            cylinderPoint);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactAnchors_RejectCornerSeparationAndIncludeTangency()
    {
        var separated = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(5, 2)),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Assert.False(separated.TryGetCenteredCylinderContact(
            Vector3d.Zero,
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Forward),
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.Half,
            out FixedContactAnchors miss));
        Assert.Equal(default, miss);

        var tangent = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(tangent.TryGetCenteredCylinderContact(
            new Vector3d(2, 0, 0),
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Forward),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            out FixedContactAnchors touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
    }

    [Fact]
    public void ContactAnchors_RejectFiniteCapCornerSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Fixed64 centerCoordinate = Fixed64.FromFraction(7, 5);

        Assert.False(box.TryGetCenteredCylinderContact(
            new Vector3d(
                centerCoordinate,
                centerCoordinate,
                centerCoordinate),
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Forward),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ContactAnchors_RejectCapSeparationInsideCapsuleProxy()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.FromFraction(1, 10),
                Fixed64.FromFraction(1, 10),
                Fixed64.FromFraction(1, 10)));

        Assert.False(box.TryGetCenteredCylinderContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ContactAnchors_RejectLateralBoxAxisSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetCenteredCylinderContact(
            new Vector3d(4, 0, 0),
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Forward),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ContactAnchors_RejectObliqueSideSeparationInsideCapsuleProxy()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d axis = new(
            FixedMath.Sqrt(Fixed64.Half),
            FixedMath.Sqrt(Fixed64.Half),
            Fixed64.Zero);
        FixedQuaternion rotation =
            WideGeometry.GetCanonicalAxisRotation(axis);
        Vector3d center = new(
            Fixed64.FromFraction(21, 10),
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(box.TryGetCenteredCapsuleContact(
            center,
            rotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out _));
        Assert.False(box.TryGetCenteredCylinderContact(
            center,
            rotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ContactAnchors_RemainRelativeAtScalarFaceAndClampDepth()
    {
        var scalarFace = new FixedOrientedBox(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(scalarFace.TryGetCenteredCylinderContact(
            scalarFace.Center,
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Right),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            out FixedContactAnchors boundary));
        Assert.True(boundary.Normal.MagnitudeSquared > Fixed64.Zero);

        var huge = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));
        Assert.True(huge.TryGetCenteredCylinderContact(
            Vector3d.Zero,
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Right),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.MaxValue,
            out FixedContactAnchors clamped));
        Assert.Equal(Fixed64.MaxValue, clamped.Depth);
        Assert.True(clamped.DepthIsClamped);
    }

    [Fact]
    public void ContactAnchors_RejectInvalidCylinderInputs()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<ArgumentException>(() =>
            box.TryGetCenteredCylinderContact(
                Vector3d.Zero,
                FixedQuaternion.Zero,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetCenteredCylinderContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCenteredCylinderContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCenteredCylinderContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                out _));
    }

    [Fact]
    public void CapFaceContactAnchors_ReturnFourStableMatchedConstraints()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out FixedContactAnchors primary,
            out int count));

        Assert.Equal(4, count);
        Assert.Equal(Fixed64.FromFraction(1, 4), primary.Depth);
        Assert.Equal(
            new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Zero),
            contacts[0].FirstLocalPoint);
        Assert.Equal(
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero),
            contacts[1].FirstLocalPoint);
        Assert.Equal(
            new Vector3d(Fixed64.Zero, Fixed64.Half, -Fixed64.Half),
            contacts[2].FirstLocalPoint);
        Assert.Equal(
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Half),
            contacts[3].FirstLocalPoint);
        for (int index = 0; index < count; index++)
        {
            Assert.Equal(
                contacts[index].FirstLocalPoint - Vector3d.Up * primary.Depth
                    - new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero),
                contacts[index].SecondLocalPoint);
        }
    }

    [Fact]
    public void CapFaceContactAnchors_ClipDiskRectangleAndDeduplicateTangency()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(
                Fixed64.FromFraction(3, 4),
                Fixed64.FromFraction(3, 4),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out _,
            out int clippedCount));
        Assert.InRange(clippedCount, 2, 4);
        for (int index = 0; index < clippedCount; index++)
        {
            Assert.True(
                contacts[index].FirstLocalPoint.X >= -Fixed64.Half
                && contacts[index].FirstLocalPoint.X <= Fixed64.Half);
            Assert.True(
                contacts[index].FirstLocalPoint.Z >= -Fixed64.Half
                && contacts[index].FirstLocalPoint.Z <= Fixed64.Half);
            Vector3d radial = new(
                contacts[index].SecondLocalPoint.X,
                Fixed64.Zero,
                contacts[index].SecondLocalPoint.Z);
            Assert.True(radial.MagnitudeSquared <= Fixed64.Quarter);
        }

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(
                -Fixed64.FromFraction(3, 4),
                Fixed64.FromFraction(3, 4),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out _,
            out int mirroredClippedCount));
        Assert.InRange(mirroredClippedCount, 2, 4);
        Assert.Contains(
            contacts[..mirroredClippedCount].ToArray(),
            contact => contact.FirstLocalPoint.X == -Fixed64.Half);

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(
                Fixed64.One,
                Fixed64.FromFraction(3, 4),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out FixedContactAnchors tangentPrimary,
            out int tangentCount));
        Assert.Equal(0, tangentCount);
        Assert.Equal(Vector3d.Right, tangentPrimary.Normal);

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out FixedContactAnchors capTangent,
            out int capTangentCount));
        Assert.Equal(4, capTangentCount);
        Assert.Equal(Fixed64.Zero, capTangent.Depth);
    }

    [Fact]
    public void CapFaceContactAnchors_HandleRotationReversalAndScalarFaces()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)45);
        Vector3d axis = (rotation * Vector3d.Up).Normalized;
        var rotated = new FixedOrientedBox(
            Vector3d.Zero,
            rotation,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Span<FixedContactLocalPoints> first = stackalloc FixedContactLocalPoints[4];
        Span<FixedContactLocalPoints> second = stackalloc FixedContactLocalPoints[4];

        Assert.True(rotated.TryGetCenteredCylinderContact(
            axis * Fixed64.FromFraction(3, 4),
            rotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            first,
            out FixedContactAnchors rotatedPrimary,
            out int firstCount));
        Assert.True(rotated.TryGetCenteredCylinderContact(
            axis * Fixed64.FromFraction(3, 4),
            rotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            second,
            out _,
            out int secondCount));
        Assert.True(
            firstCount == 4,
            $"Expected four rotated contacts; primary normal was {rotatedPrimary.Normal} and depth {rotatedPrimary.Depth}.");
        Assert.Equal(firstCount, secondCount);
        for (int index = 0; index < firstCount; index++)
        {
            Assert.Equal(first[index].FirstLocalPoint, second[index].FirstLocalPoint);
            Assert.Equal(first[index].SecondLocalPoint, second[index].SecondLocalPoint);
        }

        Assert.True(rotated.TryGetCenteredCylinderContact(
            -axis * Fixed64.FromFraction(3, 4),
            rotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            first,
            out FixedContactAnchors reversed,
            out int reversedCount));
        Assert.Equal(4, reversedCount);
        Assert.True(Vector3d.Dot(reversed.Normal, axis) < Fixed64.Zero);

        var scalarFace = new FixedOrientedBox(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Assert.True(scalarFace.TryGetCenteredCylinderContact(
            scalarFace.Center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            first,
            out _,
            out int scalarFaceCount));
        Assert.Equal(4, scalarFaceCount);
        Assert.Contains(
            first[..scalarFaceCount].ToArray(),
            contact => !Vector3d.TryAdd(
                scalarFace.Center,
                contact.FirstLocalPoint,
                out _));
    }

    [Fact]
    public void CapFaceContactAnchors_UseZeroAllocationsAndRequireCapacity()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Vector3d cylinderCenter =
            new(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero);
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];
        Assert.True(box.TryGetCenteredCylinderContact(
            cylinderCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out _,
            out _));

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 1_000; iteration++)
        {
            Assert.True(box.TryGetCenteredCylinderContact(
                cylinderCenter,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.Half,
                contacts,
                out _,
                out _));
        }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);

        Assert.Throws<ArgumentException>(() =>
        {
            Span<FixedContactLocalPoints> insufficient =
                stackalloc FixedContactLocalPoints[3];
            box.TryGetCenteredCylinderContact(
                cylinderCenter,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.Half,
                insufficient,
                out _,
                out _);
        });

        Assert.False(box.TryGetCenteredCylinderContact(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out FixedContactAnchors separated,
            out int separatedCount));
        Assert.Equal(default, separated);
        Assert.Equal(0, separatedCount);
    }

    [Fact]
    public void CapFaceContactAnchors_SupportEveryBoxFaceAndDegenerateDisks()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Right),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out FixedContactAnchors xContact,
            out int xCount));
        Assert.Equal(4, xCount);
        Assert.Equal(Vector3d.Right, xContact.Normal);

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(3, 4)),
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Forward),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out FixedContactAnchors zContact,
            out int zCount));
        Assert.Equal(4, zCount);
        Assert.Equal(Vector3d.Forward, zContact.Normal);

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            contacts,
            out FixedContactAnchors pointCap,
            out int pointCount));
        Assert.Equal(1, pointCount);
        Assert.Equal(Vector3d.Up, pointCap.Normal);

        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d skewAxis =
            new(diagonal, diagonal, Fixed64.Zero);
        Assert.True(box.TryGetCenteredCylinderContact(
            skewAxis * Fixed64.FromFraction(3, 4),
            WideGeometry.GetCanonicalAxisRotation(skewAxis),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            contacts,
            out _,
            out int skewCount));
        Assert.Equal(0, skewCount);
    }

    [Fact]
    public void CapFaceContactAnchors_RetainAllSemanticLocalConstraints()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)45);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            rotation,
            new Vector3d(
                Fixed64.MaxValue * Fixed64.FromFraction(4, 5),
                Fixed64.MaxValue * Fixed64.FromFraction(17, 20),
                Fixed64.Half));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetCenteredCylinderContact(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.MaxValue * Fixed64.FromFraction(99, 100),
                Fixed64.FromFraction(99, 100)),
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Forward),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.MaxValue * Fixed64.FromFraction(4, 5),
            contacts,
            out FixedContactAnchors primary,
            out int count));
        Assert.Equal(Vector3d.Forward, primary.Normal);
        Assert.Equal(4, count);
        for (int index = 0; index < count; index++)
        {
            var firstAnchor = new FixedPointAnchor(
                primary.FirstAnchor.Origin,
                primary.FirstAnchor.Rotation,
                contacts[index].FirstLocalPoint);
            var secondAnchor = new FixedPointAnchor(
                primary.SecondAnchor.Origin,
                primary.SecondAnchor.Rotation,
                contacts[index].SecondLocalPoint);
            Assert.True(firstAnchor.TryGetOffsetFrom(
                secondAnchor,
                out Vector3d separation));
            Assert.Equal(
                primary.Depth,
                Vector3d.Dot(separation, primary.Normal));
        }
    }

    [Fact]
    public void ContactAnchors_RejectCapRimSeparationInsideCapsuleProxy()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)15),
            new Vector3d(
                Fixed64.One,
                Fixed64.FromFraction(3, 4),
                Fixed64.FromFraction(5, 4)));
        Vector3d axis = new Vector3d(3, 1, -2).Normalized;
        FixedQuaternion rotation =
            WideGeometry.GetCanonicalAxisRotation(axis);
        Vector3d center = new(
            Fixed64.FromFraction(7, 4),
            Fixed64.FromFraction(11, 5),
            -Fixed64.FromFraction(7, 5));

        Assert.True(box.TryGetCenteredCapsuleContact(
            center,
            rotation,
            Vector3d.Up,
            Fixed64.FromFraction(5, 2),
            Fixed64.FromFraction(9, 10),
            out _));
        Assert.False(box.TryGetCenteredCylinderContact(
            center,
            rotation,
            Vector3d.Up,
            Fixed64.FromFraction(5, 2),
            Fixed64.FromFraction(9, 10),
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }
}

