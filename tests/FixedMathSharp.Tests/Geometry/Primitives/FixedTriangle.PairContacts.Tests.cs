//=======================================================================
// FixedTriangle.PairContacts.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTrianglePairContactsTests
{
    [Fact]
    public void PairContact_ValidatesBothRotationsAndRejectsExactZeroNormals()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(2, 0, 0),
            new Vector3d(0, 2, 0));
        var zeroNormal = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(1, 1, 1),
            new Vector3d(2, 2, 2));

        ArgumentException firstError = Assert.Throws<ArgumentException>(() =>
            triangle.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Zero,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                triangle,
                out _));
        Assert.Equal("firstRotation", firstError.ParamName);

        ArgumentException secondError = Assert.Throws<ArgumentException>(() =>
            triangle.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                FixedQuaternion.Zero,
                triangle,
                out _));
        Assert.Equal("secondRotation", secondError.ParamName);

        Assert.False(zeroNormal.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            triangle,
            out FixedContactAnchors firstDegenerate));
        Assert.Equal(default, firstDegenerate);
        Assert.False(triangle.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            zeroNormal,
            out FixedContactAnchors secondDegenerate));
        Assert.Equal(default, secondDegenerate);
    }

    [Fact]
    public void PairContact_RejectsCoplanarInPlaneSeparationAndIncludesTouching()
    {
        var first = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(0, 4, 0));
        var separated = new FixedTriangle(
            new Vector3d(3, 3, 0),
            new Vector3d(5, 3, 0),
            new Vector3d(3, 5, 0));
        var touching = new FixedTriangle(
            new Vector3d(2, 2, 0),
            new Vector3d(4, 2, 0),
            new Vector3d(2, 4, 0));

        Assert.False(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            separated,
            out _));
        Assert.False(separated.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            first,
            out _));
        Assert.True(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            touching,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void PairContact_RejectsSeparationAlongEitherFaceNormal()
    {
        var first = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(2, 0, 0),
            new Vector3d(0, 2, 0));
        var parallel = new FixedTriangle(
            new Vector3d(0, 0, 1),
            new Vector3d(2, 0, 1),
            new Vector3d(0, 2, 1));
        var orthogonal = new FixedTriangle(
            new Vector3d(3, 0, -1),
            new Vector3d(3, 2, -1),
            new Vector3d(3, 0, 1));

        Assert.False(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            parallel,
            out _));
        Assert.False(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            orthogonal,
            out _));
    }

    [Fact]
    public void PairContact_RejectsCoplanarSeparatorOwnedBySecondEdge()
    {
        var first = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(0, 4, 0));
        var second = new FixedTriangle(
            new Vector3d(3, -2, 0),
            new Vector3d(5, 0, 0),
            new Vector3d(5, -1, 0));

        Assert.False(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            second,
            out _));
    }

    [Fact]
    public void PairContact_RejectsTinyNonzeroInPlaneSeparatingAxis()
    {
        Fixed64 m = Fixed64.MinIncrement;
        var first = new FixedTriangle(
            new Vector3d(-m, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(m, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero));
        var second = new FixedTriangle(
            new Vector3d(-m, -m, Fixed64.Zero),
            new Vector3d(m, -m, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero));

        Assert.False(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            second,
            out _));
        Assert.False(second.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            first,
            out _));
    }

    [Fact]
    public void PairContact_HandlesOrdinaryCoplanarAndTranslatedRigidFrames()
    {
        var first = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(0, 4, 0));
        var crossing = new FixedTriangle(
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.FromFraction(-1, 4)),
            new Vector3d(1, 1, 1),
            new Vector3d(3, 1, 1));
        var coplanar = new FixedTriangle(
            new Vector3d(1, 1, 0),
            new Vector3d(3, 1, 0),
            new Vector3d(1, 3, 0));

        Assert.True(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            crossing,
            out FixedContactAnchors ordinary));
        Assert.Equal(Fixed64.FromFraction(1, 4), ordinary.Depth);
        Assert.Equal(Vector3d.Forward, ordinary.Normal);
        Assert.False(ordinary.DepthIsClamped);

        Assert.True(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            coplanar,
            out FixedContactAnchors coplanarContact));
        Assert.Equal(Fixed64.Zero, coplanarContact.Depth);

        var translation = new Vector3d(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.MaxValue);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            Fixed64.PiOver4);
        Assert.True(first.TryGetContact(
            translation,
            rotation,
            translation,
            rotation,
            crossing,
            out FixedContactAnchors translated));
        Assert.True(rotation.TryRotate(
            ordinary.Normal,
            out Vector3d expectedNormal));
        Assert.True(Vector3d.Distance(
            expectedNormal,
            translated.Normal) <= Fixed64.Epsilon);
        Assert.Equal(ordinary.Depth, translated.Depth);
        Assert.Equal(ordinary.DepthIsClamped, translated.DepthIsClamped);
        Assert.Equal(translation, translated.FirstAnchor.Origin);
        Assert.Equal(rotation, translated.FirstAnchor.Rotation);
        Assert.Equal(translation, translated.SecondAnchor.Origin);
        Assert.Equal(rotation, translated.SecondAnchor.Rotation);
        Assert.Equal(ordinary.FirstAnchor.LocalPoint, translated.FirstAnchor.LocalPoint);
        Assert.Equal(ordinary.SecondAnchor.LocalPoint, translated.SecondAnchor.LocalPoint);
    }

    [Fact]
    public void PairContact_DistinctRotationsPreserveContactAndFrameOwnedAnchors()
    {
        var first = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(0, 3, 0));
        var second = new FixedTriangle(
            new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.FromFraction(-1, 4)),
            new Vector3d(-1, 0, 1),
            new Vector3d(-1, -2, 1));
        var halfTurn = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero);

        Assert.True(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            halfTurn,
            second,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(Vector3d.Distance(
            new Vector3d(1, 1, 0),
            contact.FirstAnchor.LocalPoint) <= Fixed64.Epsilon);
        Assert.True(Vector3d.Distance(
            new Vector3d(-1, -1, 0),
            contact.SecondAnchor.LocalPoint) <= Fixed64.Epsilon);
        Assert.Equal(FixedQuaternion.Identity, contact.FirstAnchor.Rotation);
        Assert.Equal(halfTurn, contact.SecondAnchor.Rotation);

        Assert.True(second.TryGetContact(
            Vector3d.Zero,
            halfTurn,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            first,
            out FixedContactAnchors reverse));
        Assert.Equal(-contact.Normal, reverse.Normal);
        Assert.Equal(contact.Depth, reverse.Depth);
        Assert.Equal(contact.DepthIsClamped, reverse.DepthIsClamped);
        Assert.Equal(halfTurn, reverse.FirstAnchor.Rotation);
        Assert.Equal(FixedQuaternion.Identity, reverse.SecondAnchor.Rotation);
        Assert.True(Vector3d.Distance(
            new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero),
            reverse.FirstAnchor.LocalPoint) <= Fixed64.Epsilon,
            $"Unexpected reversed first anchor: {reverse.FirstAnchor.LocalPoint}");
        Assert.True(Vector3d.Distance(
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
            reverse.SecondAnchor.LocalPoint) <= Fixed64.Epsilon,
            $"Unexpected reversed second anchor: {reverse.SecondAnchor.LocalPoint}");

        Assert.False(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(0, 0, 2),
            halfTurn,
            second,
            out _));
    }

    [Fact]
    public void PairContact_RejectsFullDomainEdgeCrossSeparationInBothOrders()
    {
        Fixed64 scale = Fixed64.MaxValue / new Fixed64(3);
        var first = new FixedTriangle(
            new Vector3d(1, -2, 2) * scale,
            new Vector3d(0, 3, 1) * scale,
            new Vector3d(-2, 2, 1) * scale);
        var second = new FixedTriangle(
            new Vector3d(2, 2, -2) * scale,
            new Vector3d(-2, 0, -3) * scale,
            new Vector3d(-2, -1, 2) * scale);

        Assert.False(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            second,
            out _));
        Assert.False(second.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            first,
            out _));
    }

    [Fact]
    public void PairContact_CrossingReversalSwapsFramesAndNegatesNormal()
    {
        var first = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(0, 4, 0));
        var second = new FixedTriangle(
            new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.FromFraction(-1, 4)),
            new Vector3d(-1, 1, 1),
            new Vector3d(1, 1, 1));
        var firstOrigin = new Vector3d(7, -3, 2);
        var secondOrigin = new Vector3d(9, -3, 2);

        Assert.True(first.TryGetContact(
            firstOrigin,
            FixedQuaternion.Identity,
            secondOrigin,
            FixedQuaternion.Identity,
            second,
            out FixedContactAnchors forward));
        Assert.True(second.TryGetContact(
            secondOrigin,
            FixedQuaternion.Identity,
            firstOrigin,
            FixedQuaternion.Identity,
            first,
            out FixedContactAnchors reverse));

        Assert.Equal(forward.Depth, reverse.Depth);
        Assert.Equal(-forward.Normal, reverse.Normal);
        Assert.Equal(firstOrigin, forward.FirstAnchor.Origin);
        Assert.Equal(secondOrigin, forward.SecondAnchor.Origin);
        Assert.Equal(secondOrigin, reverse.FirstAnchor.Origin);
        Assert.Equal(firstOrigin, reverse.SecondAnchor.Origin);
    }

    [Fact]
    public void PairContact_CentroidTieRetainsFirstFaceNormal()
    {
        var first = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(2, 0, 0),
            new Vector3d(0, 2, 0));
        var reverseWound = new FixedTriangle(first.A, first.C, first.B);

        Assert.True(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            reverseWound,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void PairContact_MirroredExtremeFramesRetainSemanticAnchors()
    {
        Fixed64 maximum = Fixed64.MaxValue;
        Fixed64 minimum = Fixed64.MinValue;
        var first = new FixedTriangle(
            new Vector3d(maximum, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(maximum, Fixed64.Two, Fixed64.Zero),
            new Vector3d(maximum, Fixed64.Zero, Fixed64.Two));
        var second = new FixedTriangle(
            new Vector3d(minimum, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(minimum, Fixed64.Two, Fixed64.Zero),
            new Vector3d(minimum, Fixed64.Zero, Fixed64.Two));

        Assert.True(first.TryGetContact(
            new Vector3d(minimum, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(maximum, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            second,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetOffsetFrom(
            contact.FirstAnchor,
            out Vector3d anchorOffset));
        Assert.Equal(Vector3d.Zero, anchorOffset);
    }

    [Fact]
    public void PairContact_ClampsOnlyTheFinalFullDomainDepth()
    {
        Fixed64 maximum = Fixed64.MaxValue;
        var first = new FixedTriangle(
            new Vector3d(-maximum, maximum, -maximum),
            new Vector3d(-maximum, -maximum, maximum),
            new Vector3d(maximum, maximum, maximum));
        var second = new FixedTriangle(
            new Vector3d(-maximum, maximum, maximum),
            new Vector3d(maximum, -maximum, maximum),
            new Vector3d(maximum, maximum, -maximum));

        Assert.True(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            second,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void PairContact_WarmedPathIsStableAndAllocatesNothing()
    {
        var first = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(0, 4, 0));
        var second = new FixedTriangle(
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.FromFraction(-1, 4)),
            new Vector3d(1, 1, 1),
            new Vector3d(3, 1, 1));

        Assert.True(first.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            second,
            out FixedContactAnchors expected));
        long before = GC.GetAllocatedBytesForCurrentThread();
        bool stable = true;
        for (int index = 0; index < 64; index++)
        {
            bool hit = first.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                second,
                out FixedContactAnchors actual);
            stable &= hit
                && actual.FirstAnchor == expected.FirstAnchor
                && actual.SecondAnchor == expected.SecondAnchor
                && actual.Normal == expected.Normal
                && actual.Depth == expected.Depth
                && actual.DepthIsClamped == expected.DepthIsClamped;
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.True(stable);
        Assert.Equal(0L, allocated);
    }
}
