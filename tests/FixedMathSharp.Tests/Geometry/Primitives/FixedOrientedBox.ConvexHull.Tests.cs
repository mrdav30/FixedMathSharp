//=======================================================================
// FixedOrientedBox.ConvexHull.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxConvexHullTests
{
    private static readonly int[] CubeTriangles =
    {
        0, 2, 1, 1, 2, 3,
        4, 5, 6, 5, 7, 6,
        0, 4, 2, 2, 4, 6,
        1, 3, 5, 3, 7, 5,
        0, 1, 4, 1, 5, 4,
        2, 6, 3, 3, 6, 7,
    };

    private static readonly int[] CubeEdges =
    {
        0, 1, 0, 2, 0, 4,
        1, 3, 1, 5, 2, 3,
        2, 6, 3, 7, 4, 5,
        4, 6, 5, 7, 6, 7,
    };

    [Fact]
    public void ContactAnchors_EqualFaceDepthKeepsFirstBoxAxisAndFirstSupportPoint()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d[] hullOffsets = CreateCubeOffsets((Fixed64)3);

        Assert.True(box.TryGetConvexHullContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            hullOffsets,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal((Fixed64)4, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.One, contact.FirstAnchor.LocalPoint.X);
        Assert.Equal(new Vector3d(-3, -3, -3), contact.SecondAnchor.LocalPoint);
    }

    [Fact]
    public void ContactAnchors_RejectBoxHullFaceAndEdgeCrossSeparations()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d[] hullOffsets = CreateCubeOffsets(Fixed64.One);

        Assert.False(box.TryGetConvexHullContact(
            new Vector3d(3, 0, 0),
            FixedQuaternion.Identity,
            hullOffsets,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors boxContact));
        Assert.Equal(default, boxContact);

        Assert.False(box.TryGetConvexHullContact(
            new Vector3d(
                Fixed64.FromFraction(23, 10),
                Fixed64.FromFraction(23, 10),
                Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)45),
            hullOffsets,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors faceContact));
        Assert.Equal(default, faceContact);

        Assert.False(box.TryGetConvexHullContact(
            new Vector3d(
                Fixed64.FromFraction(-27, 10),
                Fixed64.FromFraction(-3, 2),
                Fixed64.FromFraction(-19, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)15,
                (Fixed64)30,
                (Fixed64)45),
            hullOffsets,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors edgeContact));
        Assert.Equal(default, edgeContact);
    }

    [Fact]
    public void ContactAnchors_RemainOriginRelativeAtScalarFace()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.MaxValue - Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d[] hullOffsets = CreateCubeOffsets(Fixed64.One);

        Assert.True(box.TryGetConvexHullContact(
            origin,
            FixedQuaternion.Identity,
            hullOffsets,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(box.Center, contact.FirstAnchor.Origin);
        Assert.Equal(box.Orientation, contact.FirstAnchor.Rotation);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.Equal(FixedQuaternion.Identity, contact.SecondAnchor.Rotation);
    }

    [Fact]
    public void ContactAnchors_ClampTrueDepthOverflow()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));
        Vector3d[] hullOffsets = CreateCubeOffsets(Fixed64.MaxValue);

        Assert.True(box.TryGetConvexHullContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            hullOffsets,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void TinyRotatedHullInsideWideBox_UsesFiniteNearestExit()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue / Fixed64.Two,
                Fixed64.One,
                Fixed64.MaxValue / Fixed64.Two));
        Vector3d[] hullOffsets =
            CreateCubeOffsets(Fixed64.MinIncrement);

        Assert.True(box.TryGetConvexHullContact(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)23,
                (Fixed64)31),
            hullOffsets,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));

        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.Depth >= Fixed64.One);
        Assert.True(contact.Depth <= Fixed64.One + Fixed64.FromRaw(2));
    }

    [Fact]
    public void ContactAnchors_RetainSelectedRotatedPointWithoutMaterializingIt()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d[] hullOffsets = CreateCubeOffsets(Fixed64.MaxValue);
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)45);

        Assert.True(box.TryGetConvexHullContact(
            Vector3d.Zero,
            rotation,
            hullOffsets,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));
        Assert.Equal(rotation, contact.SecondAnchor.Rotation);
        Assert.Contains(
            contact.SecondAnchor.LocalPoint,
            hullOffsets);
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
    }

    [Fact]
    public void ContactAnchors_HandleRotatedHullAtScalarFace()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.MaxValue - Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)5,
                (Fixed64)10,
                (Fixed64)15),
            Vector3d.One);
        FixedQuaternion hullRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)(-5),
                (Fixed64)15,
                (Fixed64)25);

        Assert.True(box.TryGetConvexHullContact(
            origin,
            hullRotation,
            CreateCubeOffsets((Fixed64)2),
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.Normal.MagnitudeSquared > Fixed64.Zero);
    }

    [Fact]
    public void ContactAnchors_PreserveRotatedSpanSliceUnderScalarTranslation()
    {
        FixedQuaternion boxRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)7,
                (Fixed64)19,
                (Fixed64)(-11));
        FixedQuaternion hullRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)(-5),
                (Fixed64)13,
                (Fixed64)17);
        Vector3d hullOrigin = new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            boxRotation,
            new Vector3d(2, 1, 3));
        Vector3d[] storage = new Vector3d[10];
        CreateCubeOffsets(Fixed64.One).CopyTo(storage, 1);
        ReadOnlySpan<Vector3d> points = storage.AsSpan(1, 8);

        Assert.True(box.TryGetConvexHullContact(
            hullOrigin,
            hullRotation,
            points,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors ordinary));

        Vector3d translation = new(
            Fixed64.MaxValue - (Fixed64)10,
            Fixed64.Zero,
            Fixed64.Zero);
        var translatedBox = new FixedOrientedBox(
            translation,
            boxRotation,
            box.HalfExtents);
        Assert.True(translatedBox.TryGetConvexHullContact(
            translation + hullOrigin,
            hullRotation,
            points,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors translated));

        Assert.Equal(ordinary.Normal, translated.Normal);
        Assert.Equal(ordinary.Depth, translated.Depth);
        Assert.Equal(ordinary.DepthIsClamped, translated.DepthIsClamped);
        Assert.Equal(
            ordinary.FirstAnchor.LocalPoint,
            translated.FirstAnchor.LocalPoint);
        Assert.Equal(
            ordinary.FirstAnchor.LocalDisplacement,
            translated.FirstAnchor.LocalDisplacement);
        Assert.Equal(
            ordinary.SecondAnchor.LocalPoint,
            translated.SecondAnchor.LocalPoint);
        Assert.Equal(
            ordinary.SecondAnchor.LocalDisplacement,
            translated.SecondAnchor.LocalDisplacement);
    }

    [Fact]
    public void ConvexHullContact_WarmedPathDoesNotAllocate()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)7,
                (Fixed64)19,
                (Fixed64)(-11)),
            new Vector3d(2, 1, 3));
        Vector3d[] points = CreateCubeOffsets(Fixed64.One);
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)(-5),
                (Fixed64)13,
                (Fixed64)17);
        _ = box.TryGetConvexHullContact(
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            rotation,
            points,
            CubeTriangles,
            CubeEdges,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        bool allContacts = true;

        for (int iteration = 0; iteration < 64; iteration++)
        {
            allContacts &= box.TryGetConvexHullContact(
                new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
                rotation,
                points,
                CubeTriangles,
                CubeEdges,
                out _);
        }

        long after = GC.GetAllocatedBytesForCurrentThread();
        Assert.True(allContacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void ContactAnchors_RejectInvalidAuthoredHullData()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d[] hullOffsets = CreateCubeOffsets(Fixed64.One);
        var invalidRotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)2);

        Assert.Throws<ArgumentException>(() =>
            box.TryGetConvexHullContact(
                Vector3d.Zero,
                invalidRotation,
                hullOffsets,
                CubeTriangles,
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetConvexHullContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d[3],
                CubeTriangles,
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetConvexHullContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                hullOffsets,
                Array.Empty<int>(),
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetConvexHullContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                hullOffsets,
                new[] { 0 },
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetConvexHullContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                hullOffsets,
                CubeTriangles,
                Array.Empty<int>(),
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetConvexHullContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                hullOffsets,
                CubeTriangles,
                new[] { 0, 1, 2 },
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetConvexHullContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                hullOffsets,
                new[] { 0, 1, hullOffsets.Length },
                CubeEdges,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetConvexHullContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                hullOffsets,
                CubeTriangles,
                new[] { 0, hullOffsets.Length },
                out _));
    }

    private static Vector3d[] CreateCubeOffsets(Fixed64 extent) =>
        new[]
        {
            new Vector3d(-extent, -extent, -extent),
            new Vector3d(extent, -extent, -extent),
            new Vector3d(-extent, extent, -extent),
            new Vector3d(extent, extent, -extent),
            new Vector3d(-extent, -extent, extent),
            new Vector3d(extent, -extent, extent),
            new Vector3d(-extent, extent, extent),
            new Vector3d(extent, extent, extent),
        };
}
