//=======================================================================
// FixedOrientedBox.ConvexPrism.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxConvexPrismTests
{
    private static readonly Vector2d[] UnitSquare =
    {
        new(-1, -1),
        new(1, -1),
        new(1, 1),
        new(-1, 1),
    };

    [Fact]
    public void ContactOffsets_UseExactPolytopeAxesAndStableSupports()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetConvexPrismContact(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(new Vector3d(1, -1, -1), contact.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(-1, -1, -1), contact.SecondAnchor.LocalPoint);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactOffsets_CoincidentOriginsOrientTowardTheSmallerExactPush()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector2d[] offsets =
        {
            new(Fixed64.FromFraction(-5, 2), -Fixed64.One),
            new(Fixed64.FromFraction(-5, 2), Fixed64.One),
            new(-Fixed64.Half, Fixed64.Zero),
        };
        Vector2d[][] permutations =
        {
            offsets,
            new[] { offsets[2], offsets[1], offsets[0] },
            new[] { offsets[1], offsets[2], offsets[0] },
        };

        foreach (Vector2d[] permutation in permutations)
        {
            Assert.True(box.TryGetConvexPrismContact(
                Vector3d.Zero,
                Fixed64.Zero,
                permutation,
                Fixed64.One,
                out FixedContactAnchors contact));

            Assert.Equal(Vector3d.Left, contact.Normal);
            Assert.Equal(
                new Vector3d(-1, -1, -1),
                contact.FirstAnchor.LocalPoint);
            Assert.Equal(
                new Vector3d(-Fixed64.Half, -Fixed64.One, Fixed64.Zero),
                contact.SecondAnchor.LocalPoint);
            Assert.Equal(Fixed64.Half, contact.Depth);
            Assert.False(contact.DepthIsClamped);
        }
    }

    [Fact]
    public void ContactOffsets_EqualPlanarPushUsesCanonicalNormalAndSupportsAcrossBoundaryOrder()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)20,
                Fixed64.Zero),
            new Vector3d(Fixed64.One, (Fixed64)4, Fixed64.One));
        Vector2d[] narrowPrism =
        {
            new(-Fixed64.FromFraction(1, 4), (Fixed64)(-2)),
            new(Fixed64.FromFraction(1, 4), (Fixed64)(-2)),
            new(Fixed64.FromFraction(1, 4), (Fixed64)2),
            new(-Fixed64.FromFraction(1, 4), (Fixed64)2),
        };
        Vector2d[] reversed =
        {
            narrowPrism[0],
            narrowPrism[3],
            narrowPrism[2],
            narrowPrism[1],
        };
        Vector2d[] cyclic =
        {
            narrowPrism[2],
            narrowPrism[3],
            narrowPrism[0],
            narrowPrism[1],
        };

        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            narrowPrism,
            (Fixed64)4,
            out FixedContactAnchors baseline));
        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            reversed,
            (Fixed64)4,
            out FixedContactAnchors reversedContact));
        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            cyclic,
            (Fixed64)4,
            out FixedContactAnchors cyclicContact));

        Assert.Equal(Vector3d.Right, baseline.Normal);
        Assert.Equal(baseline.Normal, reversedContact.Normal);
        Assert.Equal(baseline.Normal, cyclicContact.Normal);
        Assert.Equal(baseline.FirstAnchor, reversedContact.FirstAnchor);
        Assert.Equal(baseline.FirstAnchor, cyclicContact.FirstAnchor);
        Assert.Equal(baseline.SecondAnchor, reversedContact.SecondAnchor);
        Assert.Equal(baseline.SecondAnchor, cyclicContact.SecondAnchor);
        Assert.Equal(baseline.Depth, reversedContact.Depth);
        Assert.Equal(baseline.Depth, cyclicContact.Depth);

        Vector2d[] narrowZPrism =
        {
            new((Fixed64)(-2), -Fixed64.FromFraction(1, 4)),
            new((Fixed64)2, -Fixed64.FromFraction(1, 4)),
            new((Fixed64)2, Fixed64.FromFraction(1, 4)),
            new((Fixed64)(-2), Fixed64.FromFraction(1, 4)),
        };
        Vector2d[] reversedZ =
        {
            narrowZPrism[0],
            narrowZPrism[3],
            narrowZPrism[2],
            narrowZPrism[1],
        };
        Vector2d[] cyclicZ =
        {
            narrowZPrism[1],
            narrowZPrism[2],
            narrowZPrism[3],
            narrowZPrism[0],
        };
        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            narrowZPrism,
            (Fixed64)4,
            out FixedContactAnchors baselineZ));
        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            reversedZ,
            (Fixed64)4,
            out FixedContactAnchors reversedZContact));
        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            cyclicZ,
            (Fixed64)4,
            out FixedContactAnchors cyclicZContact));
        Assert.Equal(Vector3d.Forward, baselineZ.Normal);
        Assert.Equal(baselineZ.Normal, reversedZContact.Normal);
        Assert.Equal(baselineZ.Normal, cyclicZContact.Normal);
        Assert.Equal(baselineZ.FirstAnchor, reversedZContact.FirstAnchor);
        Assert.Equal(baselineZ.FirstAnchor, cyclicZContact.FirstAnchor);
        Assert.Equal(baselineZ.SecondAnchor, reversedZContact.SecondAnchor);
        Assert.Equal(baselineZ.SecondAnchor, cyclicZContact.SecondAnchor);
        Assert.Equal(baselineZ.Depth, reversedZContact.Depth);
        Assert.Equal(baselineZ.Depth, cyclicZContact.Depth);

        Assert.False(box.TryGetConvexPrismContact(
            new Vector3d(4, 0, 0),
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors separated));
        Assert.Equal(default, separated);
    }

    [Fact]
    public void ContactOffsets_RejectVerticalSeparationBeforePlanarAxes()
    {
        var box = new FixedOrientedBox(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void ContactOffsets_RejectBoxAxisAndBoxUpCrossSeparations()
    {
        var axisSeparated = new FixedOrientedBox(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One * Fixed64.FromFraction(1, 4));
        var crossSeparated = new FixedOrientedBox(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3),
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.PiOver4),
            Vector3d.One * Fixed64.FromFraction(1, 4));

        Assert.False(axisSeparated.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
        Assert.False(crossSeparated.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void ContactOffsets_RejectPrismFaceSeparationAfterBoxAxesOverlap()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.PiOver4),
            new Vector3d(
                Fixed64.Two,
                Fixed64.Half,
                Fixed64.One));

        Assert.False(box.TryGetConvexPrismContact(
            new Vector3d(
                Fixed64.FromFraction(16, 5),
                Fixed64.Zero,
                Fixed64.Zero),
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void ContactOffsets_RejectSkewEdgeSeparation()
    {
        Vector3d rodDirection =
            new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One)
                .Normalized;
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(27, 25),
                -Fixed64.FromFraction(27, 25)),
            FixedQuaternion.FromDirection(rodDirection),
            new Vector3d(
                Fixed64.FromFraction(1, 50),
                Fixed64.FromFraction(1, 50),
                Fixed64.FromFraction(1, 5)));

        Assert.False(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void ContactOffsets_FromAboveUseThePositivePrismFace()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.One, contact.SecondAnchor.LocalPoint.Y);
        Assert.Equal(Vector3d.Down, contact.Normal);
    }

    [Fact]
    public void TinyRotatedPrismInsideWideBox_UsesTheVerticalExitDepth()
    {
        Fixed64 raw = Fixed64.MinIncrement;
        Vector2d[] tinySquare =
        {
            new(-raw, -raw),
            new(raw, -raw),
            new(raw, raw),
            new(-raw, raw),
        };
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue / Fixed64.Two,
                Fixed64.One,
                Fixed64.MaxValue / Fixed64.Two));

        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.PiOver4,
            tinySquare,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.Equal(Vector3d.Up, contact.Normal);
    }

    [Fact]
    public void VerticalFaceTouch_ReportsZeroDepth()
    {
        var box = new FixedOrientedBox(
            new Vector3d(Fixed64.Zero, Fixed64.Two, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactOffsets_RemainRelativeAtTheScalarFaceAndClampDepth()
    {
        var scalarFace = new FixedOrientedBox(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(scalarFace.TryGetConvexPrismContact(
            scalarFace.Center,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors boundary));
        Assert.Equal(
            new Vector3d(-1, 1, -1),
            boundary.FirstAnchor.LocalPoint);
        Assert.Equal(Vector3d.Up, boundary.Normal);

        Vector2d[] fullDomain =
        {
            new(Fixed64.MinValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue),
        };
        var huge = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));
        Assert.True(huge.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            fullDomain,
            Fixed64.MaxValue,
            out FixedContactAnchors clamped));
        Assert.Equal(Fixed64.MaxValue, clamped.Depth);
        Assert.True(clamped.DepthIsClamped);
    }

    [Fact]
    public void ContactOffsets_RejectInvalidPrismInputs()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<ArgumentException>(() =>
            box.TryGetConvexPrismContact(
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare.AsSpan(0, 2),
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetConvexPrismContact(
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Zero,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetConvexPrismContact(
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                -Fixed64.One,
                out _));
    }

    [Fact]
    public void RotatedContact_RetainsBothRigidFramesAtTheScalarFace()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector2d[] asymmetricRectangle =
        {
            new(-Fixed64.One, Fixed64.FromFraction(-1, 4)),
            new(Fixed64.One, Fixed64.FromFraction(-1, 4)),
            new(Fixed64.One, Fixed64.Two),
            new(-Fixed64.One, Fixed64.Two),
        };

        Assert.True(box.TryGetConvexPrismContact(
            box.Center,
            Fixed64.HalfPi,
            asymmetricRectangle,
            (Fixed64)2,
            out FixedContactAnchors contact));

        Assert.Equal(box.Center, contact.FirstAnchor.Origin);
        Assert.Equal(box.Orientation, contact.FirstAnchor.Rotation);
        Assert.Equal(
            new Vector3d(-Fixed64.One, -Fixed64.One, -Fixed64.One),
            contact.FirstAnchor.LocalPoint);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d firstPoint));
        Assert.Equal(
            new Vector3d(
                Fixed64.MaxValue - Fixed64.One,
                -Fixed64.One,
                -Fixed64.One),
            firstPoint);
        Assert.Equal(box.Center, contact.SecondAnchor.Origin);
        Assert.Equal(
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                -Fixed64.HalfPi),
            contact.SecondAnchor.Rotation);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.True(Vector3d.Dot(separation, contact.Normal) > Fixed64.Zero);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void RotatedContact_UsesTheRotatedPrismForSeparation()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromFraction(5, 4),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Half));
        Vector2d[] rectangle =
        {
            new(-Fixed64.Two, -Fixed64.Half),
            new(Fixed64.Two, -Fixed64.Half),
            new(Fixed64.Two, Fixed64.Half),
            new(-Fixed64.Two, Fixed64.Half),
        };

        Assert.True(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.Zero,
            rectangle,
            Fixed64.One,
            out _));
        Assert.False(box.TryGetConvexPrismContact(
            Vector3d.Zero,
            Fixed64.HalfPi,
            rectangle,
            Fixed64.One,
            out FixedContactAnchors separated));
        Assert.Equal(default, separated);
    }
}
