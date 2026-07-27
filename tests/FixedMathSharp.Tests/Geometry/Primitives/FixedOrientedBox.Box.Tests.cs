//=======================================================================
// FixedOrientedBox.Box.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxBoxTests
{
    [Fact]
    public void ContactAnchors_AreCenterRelativeAndReverseCleanly()
    {
        var first = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var second = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(first.TryGetContact(
            second,
            out FixedContactAnchors forward));
        Assert.Equal(new Vector3d(1, 0, 0), GetOffset(forward.FirstAnchor));
        Assert.Equal(new Vector3d(-1, 0, 0), GetOffset(forward.SecondAnchor));
        Assert.Equal(Vector3d.Right, forward.Normal);
        Assert.Equal(Fixed64.Half, forward.Depth);
        Assert.False(forward.DepthIsClamped);

        Assert.True(second.TryGetContact(
            first,
            out FixedContactAnchors reverse));
        Assert.Equal(
            GetOffset(forward.SecondAnchor),
            GetOffset(reverse.FirstAnchor));
        Assert.Equal(
            GetOffset(forward.FirstAnchor),
            GetOffset(reverse.SecondAnchor));
        Assert.Equal(-forward.Normal, reverse.Normal);
        Assert.Equal(forward.Depth, reverse.Depth);
    }

    [Fact]
    public void ContactAnchors_AreBoundaryInclusiveAndRejectSeparation()
    {
        var first = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var tangent = new FixedOrientedBox(
            new Vector3d(2, 0, 0),
            FixedQuaternion.Identity,
            Vector3d.One);
        var separated = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromRaw((2L << 32) + 1L),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(first.TryGetContact(
            tangent,
            out FixedContactAnchors touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Assert.False(first.TryGetContact(separated, out FixedContactAnchors miss));
        Assert.Equal(default, miss);
    }

    [Fact]
    public void ContactAnchors_UseExactCrossAxesForRotatedBoxes()
    {
        var first = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)15,
                (Fixed64)25,
                (Fixed64)5),
            new Vector3d(2, 1, 1));
        var second = new FixedOrientedBox(
            new Vector3d(2, 0, 1),
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)(-10),
                (Fixed64)40,
                (Fixed64)20),
            new Vector3d(1, 2, 1));

        Assert.True(first.TryGetContact(
            second,
            out FixedContactAnchors firstRun));
        Assert.True(first.TryGetContact(
            second,
            out FixedContactAnchors secondRun));
        Assert.Equal(firstRun.Normal, secondRun.Normal);
        Assert.Equal(firstRun.Depth, secondRun.Depth);
        Assert.True(firstRun.Normal.MagnitudeSquared > Fixed64.Zero);
    }

    [Fact]
    public void ContactAnchors_RejectSkewEdgeSeparationAfterFaceAxesOverlap()
    {
        var first = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)20,
                (Fixed64)30,
                (Fixed64)40),
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(3, 10),
                Fixed64.Half));
        var second = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromFraction(-8, 5),
                Fixed64.FromFraction(3, 10),
                Fixed64.FromFraction(-11, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)(-45),
                (Fixed64)(-60),
                (Fixed64)40),
            new Vector3d(
                Fixed64.Two,
                Fixed64.Quarter,
                Fixed64.Quarter));

        first.GetAxes(
            out Vector3d firstX,
            out Vector3d firstY,
            out Vector3d firstZ);
        second.GetAxes(
            out Vector3d secondX,
            out Vector3d secondY,
            out Vector3d secondZ);
        Assert.True(OverlapsOnAxis(first, second, firstX));
        Assert.True(OverlapsOnAxis(first, second, firstY));
        Assert.True(OverlapsOnAxis(first, second, firstZ));
        Assert.True(OverlapsOnAxis(first, second, secondX));
        Assert.True(OverlapsOnAxis(first, second, secondY));
        Assert.True(OverlapsOnAxis(first, second, secondZ));

        Assert.False(first.TryGetContact(
            second,
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
        Assert.True(scalarFace.TryGetContact(
            scalarFace,
            out FixedContactAnchors boundary));
        Assert.Equal(new Vector3d(1, 0, 0), boundary.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(-1, 0, 0), boundary.SecondAnchor.LocalPoint);
        Assert.False(boundary.FirstAnchor.TryGetPoint(out _));

        var huge = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));
        Assert.True(huge.TryGetContact(
            huge,
            out FixedContactAnchors clamped));
        Assert.Equal(Fixed64.MaxValue, clamped.Depth);
        Assert.True(clamped.DepthIsClamped);
    }

    [Fact]
    public void ContactAnchors_RejectInvalidOtherGeometry()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<InvalidOperationException>(() =>
            box.TryGetContact(default, out _));
    }

    private static Vector3d GetOffset(FixedPointAnchor anchor)
    {
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.True(Vector3d.TrySubtract(
            point,
            anchor.Origin,
            out Vector3d offset));
        return offset;
    }

    private static bool OverlapsOnAxis(
        FixedOrientedBox first,
        FixedOrientedBox second,
        Vector3d axis)
    {
        first.GetAxes(
            out Vector3d firstX,
            out Vector3d firstY,
            out Vector3d firstZ);
        second.GetAxes(
            out Vector3d secondX,
            out Vector3d secondY,
            out Vector3d secondZ);
        Fixed64 centerDistance = Vector3d.Dot(
            second.Center - first.Center,
            axis).Abs();
        Fixed64 firstRadius =
            (Vector3d.Dot(firstX, axis) * first.HalfExtents.X).Abs()
            + (Vector3d.Dot(firstY, axis) * first.HalfExtents.Y).Abs()
            + (Vector3d.Dot(firstZ, axis) * first.HalfExtents.Z).Abs();
        Fixed64 secondRadius =
            (Vector3d.Dot(secondX, axis) * second.HalfExtents.X).Abs()
            + (Vector3d.Dot(secondY, axis) * second.HalfExtents.Y).Abs()
            + (Vector3d.Dot(secondZ, axis) * second.HalfExtents.Z).Abs();
        return centerDistance <= firstRadius + secondRadius;
    }
}
