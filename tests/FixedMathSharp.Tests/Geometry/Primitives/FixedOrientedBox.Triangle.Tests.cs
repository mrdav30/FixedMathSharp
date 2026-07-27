//=======================================================================
// FixedOrientedBox.Triangle.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxTriangleTests
{
    [Fact]
    public void ContactAnchors_AreOriginRelativeAndRejectSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d triangleOrigin = new(0, 0, 1);
        var overlapping = new FixedTriangle(
            new Vector3d(
                (Fixed64)(-1),
                (Fixed64)(-1),
                -Fixed64.Half),
            new Vector3d(
                Fixed64.One,
                (Fixed64)(-1),
                -Fixed64.Half),
            new Vector3d(
                Fixed64.Zero,
                Fixed64.One,
                -Fixed64.Half));
        var separated = new FixedTriangle(
            new Vector3d(-1, -1, 1),
            new Vector3d(1, -1, 1),
            new Vector3d(0, 1, 1));

        Assert.True(box.TryGetTriangleContact(
            triangleOrigin,
            FixedQuaternion.Identity,
            overlapping,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(triangleOrigin, contact.SecondAnchor.Origin);
        Assert.Equal(FixedQuaternion.Identity, contact.SecondAnchor.Rotation);

        Assert.False(box.TryGetTriangleContact(
            triangleOrigin,
            FixedQuaternion.Identity,
            separated,
            out FixedContactAnchors miss));
        Assert.Equal(default, miss);
    }

    [Fact]
    public void ContactAnchors_UseExactRotatedEdgeAxes()
    {
        var box = new FixedOrientedBox(
            new Vector3d(2, -1, 3),
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)15,
                (Fixed64)25,
                (Fixed64)5),
            new Vector3d(2, 1, 1));
        Vector3d triangleOrigin = new(2, -1, 3);
        var triangle = new FixedTriangle(
            new Vector3d(0, -2, 3),
            new Vector3d(4, 0, 4),
            new Vector3d(3, 2, 2));

        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            triangle,
            out FixedContactAnchors first));
        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            triangle,
            out FixedContactAnchors second));
        Assert.Equal(first.Normal, second.Normal);
        Assert.Equal(first.Depth, second.Depth);
        Assert.True(first.Normal.MagnitudeSquared > Fixed64.Zero);
    }

    [Fact]
    public void ContactAnchors_RejectTriangleFaceAndEdgeCrossSeparations()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var faceSeparated = new FixedTriangle(
            new Vector3d(3, 2, -1),
            new Vector3d(3, -1, 2),
            new Vector3d(-1, 3, 2));
        var edgeCrossSeparated = new FixedTriangle(
            new Vector3d(
                Fixed64.FromFraction(9, 10),
                (Fixed64)2,
                Fixed64.Zero),
            new Vector3d(
                (Fixed64)2,
                Fixed64.FromFraction(9, 10),
                Fixed64.Zero),
            new Vector3d(2, 2, 0));

        Assert.False(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            faceSeparated,
            out FixedContactAnchors faceContact));
        Assert.Equal(default, faceContact);

        FixedTriangle[] cyclicEdgeOrderings =
        {
            edgeCrossSeparated,
            new(
                edgeCrossSeparated.B,
                edgeCrossSeparated.C,
                edgeCrossSeparated.A),
            new(
                edgeCrossSeparated.C,
                edgeCrossSeparated.A,
                edgeCrossSeparated.B),
        };
        foreach (FixedTriangle ordering in cyclicEdgeOrderings)
        {
            Assert.False(box.TryGetTriangleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                ordering,
                out FixedContactAnchors edgeContact));
            Assert.Equal(default, edgeContact);
        }
    }

    [Fact]
    public void FaceContactAnchors_ClipStableBoxCornersToTriangle()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d triangleOrigin = new(0, 0, 1);
        var triangle = new FixedTriangle(
            new Vector3d(-2, -2, 0),
            new Vector3d(2, -2, 0),
            new Vector3d(-2, 2, 0));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            triangle,
            contacts,
            out FixedContactAnchors primary,
            out int count));
        Assert.Equal(Vector3d.Forward, primary.Normal);
        Assert.Equal(3, count);
        for (int index = 0; index < count; index++)
        {
            FixedContactLocalPoints contact = contacts[index];
            Assert.True(triangle.ContainsProjection(
                contact.SecondLocalPoint));
            var firstAnchor = new FixedPointAnchor(
                primary.FirstAnchor.Origin,
                primary.FirstAnchor.Rotation,
                contact.FirstLocalPoint);
            var secondAnchor = new FixedPointAnchor(
                primary.SecondAnchor.Origin,
                primary.SecondAnchor.Rotation,
                contact.SecondLocalPoint);
            Assert.True(firstAnchor.TryGetOffsetFrom(
                secondAnchor,
                out Vector3d separation));
            Assert.Equal(
                primary.Depth,
                Vector3d.Dot(separation, primary.Normal));
        }
    }

    [Fact]
    public void FaceContactAnchors_HandleEveryBoxFaceAxisAndOrientation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var negativeXFace = new FixedTriangle(
            new Vector3d(
                -Fixed64.Half,
                (Fixed64)(-2),
                (Fixed64)(-2)),
            new Vector3d(
                -Fixed64.Half,
                (Fixed64)(-2),
                (Fixed64)2),
            new Vector3d(
                -Fixed64.Half,
                (Fixed64)2,
                (Fixed64)(-2)));
        var positiveYFace = new FixedTriangle(
            new Vector3d(-2, 0, -2),
            new Vector3d(-2, 0, 2),
            new Vector3d(2, 0, -2));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            negativeXFace,
            contacts,
            out FixedContactAnchors negativeXPrimary,
            out int negativeXCount));
        Assert.Equal(-Vector3d.Right, negativeXPrimary.Normal);
        Assert.Equal(3, negativeXCount);

        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            positiveYFace,
            contacts,
            out FixedContactAnchors positiveYPrimary,
            out int positiveYCount));
        Assert.Equal(Vector3d.Up, positiveYPrimary.Normal);
        Assert.Equal(3, positiveYCount);
    }

    [Fact]
    public void FaceContactAnchors_DoNotInventBoxFaceContactsForSlantedTriangle()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var triangle = new FixedTriangle(
            new Vector3d(4, -4, 0),
            new Vector3d(-4, 0, 4),
            new Vector3d(0, 4, -4));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            triangle,
            contacts,
            out FixedContactAnchors primary,
            out int count));
        Assert.Equal(0, count);
        Assert.True(
            Vector3d.Dot(triangle.Normal, primary.Normal).Abs()
                > Fixed64.FromFraction(99, 100));
    }

    [Fact]
    public void FaceContactAnchors_RecognizeSharedRigidFrameAtScalarBoundary()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.Zero);
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)23,
                (Fixed64)31);
        var box = new FixedOrientedBox(
            origin,
            rotation,
            Vector3d.One);
        var triangle = new FixedTriangle(
            new Vector3d((Fixed64)(-2), (Fixed64)(-2), Fixed64.Half),
            new Vector3d((Fixed64)2, (Fixed64)(-2), Fixed64.Half),
            new Vector3d((Fixed64)(-2), (Fixed64)2, Fixed64.Half));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetTriangleContact(
            origin,
            rotation,
            triangle,
            contacts,
            out FixedContactAnchors primary,
            out int count));

        Assert.Equal(3, count);
        Assert.Equal(origin, primary.FirstAnchor.Origin);
        Assert.Equal(origin, primary.SecondAnchor.Origin);
        Assert.Equal(rotation, primary.FirstAnchor.Rotation);
        Assert.Equal(rotation, primary.SecondAnchor.Rotation);
    }

    [Fact]
    public void FaceContactAnchors_SelectTheDeepestTriangleWitnessBeforeRejectingAnEdgeFeature()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var triangle = new FixedTriangle(
            new Vector3d(-3, -3, -3),
            new Vector3d(-3, -3, -2),
            new Vector3d(3, 3, 2));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            triangle,
            contacts,
            out FixedContactAnchors primary,
            out int count));

        Assert.Equal(triangle.B, primary.SecondAnchor.LocalPoint);
        Assert.Equal(0, count);
        Assert.True(
            Vector3d.Dot(triangle.Normal, primary.Normal).Abs()
                < Fixed64.FromFraction(99, 100));
    }

    [Fact]
    public void FaceContactAnchors_RejectAParallelPlaneWhenThePrimaryAxisIsInPlane()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var triangle = new FixedTriangle(
            new Vector3d(
                Fixed64.FromFraction(9, 10),
                (Fixed64)(-2),
                Fixed64.Zero),
            new Vector3d(2, -2, 0),
            new Vector3d(
                Fixed64.FromFraction(9, 10),
                (Fixed64)2,
                Fixed64.Zero));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            triangle,
            contacts,
            out FixedContactAnchors primary,
            out int count));

        Assert.Equal(Vector3d.Right, primary.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 10), primary.Depth);
        Assert.Equal(0, count);
    }

    [Fact]
    public void FaceContactAnchors_SkipUnrepresentableRotatedCornersWithoutLosingPrimary()
    {
        Fixed64 extent =
            Fixed64.MaxValue * Fixed64.FromFraction(4, 5);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)45),
            new Vector3d(extent, extent, Fixed64.One));
        var triangle = new FixedTriangle(
            new Vector3d(-2, -2, 0),
            new Vector3d(2, -2, 0),
            new Vector3d(-2, 2, 0));
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];

        Assert.True(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            triangle,
            contacts,
            out FixedContactAnchors primary,
            out int count));
        Assert.Equal(Vector3d.Forward, primary.Normal);
        Assert.Equal(0, count);
    }

    [Fact]
    public void ContactAnchors_RemainRelativeAtScalarFace()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        var box = new FixedOrientedBox(
            origin,
            FixedQuaternion.Identity,
            Vector3d.One);
        var triangle = new FixedTriangle(
            new Vector3d(
                -Fixed64.Half,
                (Fixed64)(-1),
                (Fixed64)(-1)),
            new Vector3d(
                -Fixed64.Half,
                Fixed64.Zero,
                Fixed64.One),
            new Vector3d(
                -Fixed64.Half,
                Fixed64.One,
                (Fixed64)(-1)));

        Assert.True(box.TryGetTriangleContact(
            origin,
            FixedQuaternion.Identity,
            triangle,
            out FixedContactAnchors contact));
        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(-Fixed64.One, contact.FirstAnchor.LocalPoint.X);
    }

    [Fact]
    public void ContactAnchors_RejectDegenerateTriangleAndRetainFullDomainFrame()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var degenerate = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Right * 2);
        var scalarFaceTriangle = new FixedTriangle(
            new Vector3d(
                Fixed64.MaxValue,
                (Fixed64)(-1),
                Fixed64.Zero),
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                (Fixed64)(-1)),
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One));

        Assert.False(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            degenerate,
            out _));
        Assert.True(box.TryGetTriangleContact(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            scalarFaceTriangle,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MinValue, contact.SecondAnchor.Origin.X);
        Assert.Contains(
            contact.SecondAnchor.LocalPoint,
            new[]
            {
                scalarFaceTriangle.A,
                scalarFaceTriangle.B,
                scalarFaceTriangle.C,
            });
    }

    [Fact]
    public void TriangleContact_WarmedPathDoesNotAllocate()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)7,
                (Fixed64)19,
                (Fixed64)(-11)),
            new Vector3d(2, 1, 3));
        var triangle = new FixedTriangle(
            new Vector3d(-3, 0, -3),
            new Vector3d(3, 0, -3),
            new Vector3d(0, 0, 3));
        FixedQuaternion triangleRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.PiOver4);
        Vector3d triangleOrigin = new(
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero);
        _ = box.TryGetTriangleContact(
            triangleOrigin,
            triangleRotation,
            triangle,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        bool allContacts = true;

        for (int iteration = 0; iteration < 64; iteration++)
        {
            allContacts &= box.TryGetTriangleContact(
                triangleOrigin,
                triangleRotation,
                triangle,
                out _);
        }

        long after = GC.GetAllocatedBytesForCurrentThread();
        Assert.True(allContacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void ContactAnchors_RejectInvalidRotationAndManifoldCapacity()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var triangle = new FixedTriangle(
            new Vector3d(-1, 0, -1),
            new Vector3d(1, 0, -1),
            new Vector3d(0, 0, 1));
        var invalidRotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)2);
        var contacts = new FixedContactLocalPoints[4];

        Assert.Throws<ArgumentException>(() =>
            box.TryGetTriangleContact(
                Vector3d.Zero,
                invalidRotation,
                triangle,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetTriangleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                triangle,
                new FixedContactLocalPoints[3],
                out _,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetTriangleContact(
                Vector3d.Zero,
                invalidRotation,
                triangle,
                contacts,
                out _,
                out _));
    }

    [Fact]
    public void ManifoldContact_RejectsSeparatedTriangle()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var separated = new FixedTriangle(
            new Vector3d(-1, -1, 4),
            new Vector3d(1, -1, 4),
            new Vector3d(0, 1, 4));
        var contacts = new FixedContactLocalPoints[4];

        Assert.False(box.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            separated,
            contacts,
            out FixedContactAnchors contact,
            out int contactCount));
        Assert.Equal(default, contact);
        Assert.Equal(0, contactCount);
    }
}

