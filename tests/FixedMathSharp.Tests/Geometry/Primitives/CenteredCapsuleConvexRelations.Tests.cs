//=======================================================================
// CenteredCapsuleConvexRelations.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCapsuleConvexRelationsTests
{
    [Fact]
    public void MinimumTranslation_ReturnsOrdinaryHorizontalFeature()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.FromFraction(5, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), Fixed64.One),
            new(Fixed64.FromFraction(5, 2), Fixed64.One),
        };

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            Vector2d.Zero,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.One,
            Vector2d.Zero,
            vertices,
            out Vector2d normal,
            out Fixed64 depth));
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
    }

    [Fact]
    public void Contacts_ProjectOntoTheSupportingFace()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.FromFraction(5, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), Fixed64.One),
            new(Fixed64.FromFraction(5, 2), Fixed64.One),
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(1, contactCount);
        Assert.Equal(
            new Vector2d(Fixed64.Two, Fixed64.Zero),
            capsuleContacts[0].LocalPoint);
        Assert.Equal(Vector2d.Right, capsuleContacts[0].LocalDisplacement);
        Assert.Equal(
            new Vector2d(Fixed64.FromFraction(5, 2), Fixed64.Zero),
            convexContacts[0].LocalPoint);
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
    }

    [Fact]
    public void Contacts_ReturnStableCapsuleSidePair()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.FromFraction(5, 2), (Fixed64)(-2)),
            new(Fixed64.FromFraction(9, 2), (Fixed64)(-2)),
            new(Fixed64.FromFraction(9, 2), (Fixed64)2),
            new(Fixed64.FromFraction(5, 2), (Fixed64)2),
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(2, contactCount);
        Assert.Equal(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            capsuleContacts[0].LocalPoint);
        Assert.Equal(
            new Vector2d(Fixed64.Zero, Fixed64.One),
            capsuleContacts[1].LocalPoint);
        Assert.Equal(Vector2d.Right, capsuleContacts[0].LocalDisplacement);
        Assert.Equal(Vector2d.Right, capsuleContacts[1].LocalDisplacement);
        Assert.Equal(
            new Vector2d(Fixed64.FromFraction(5, 2), -Fixed64.One),
            convexContacts[0].LocalPoint);
        Assert.Equal(
            new Vector2d(Fixed64.FromFraction(5, 2), Fixed64.One),
            convexContacts[1].LocalPoint);
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
    }

    [Fact]
    public void Contacts_OddLengthSidePairRetainsExactEndpointSeparation()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.FromFraction(5, 2), (Fixed64)(-2)),
            new(Fixed64.FromFraction(9, 2), (Fixed64)(-2)),
            new(Fixed64.FromFraction(9, 2), (Fixed64)2),
            new(Fixed64.FromFraction(5, 2), (Fixed64)2),
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        Fixed64 oddAxisLength =
            Fixed64.Two + Fixed64.MinIncrement;

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Forward,
            oddAxisLength,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out _,
            out _,
            out _));
        Assert.Equal(2, contactCount);
        Assert.True(capsuleContacts[1].TryGetOffsetFrom(
            capsuleContacts[0],
            out Vector2d endpointSeparation));
        Assert.Equal(
            Vector2d.Forward * oddAxisLength,
            endpointSeparation);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MinimumTranslation_DoesNotMaterializeOutwardEndpointAtScalarFace(bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 inward = maximumFace ? -Fixed64.One : Fixed64.One;
        Vector2d axis = maximumFace ? Vector2d.Left : Vector2d.Right;
        Vector2d[] vertices =
        {
            new(inward * Fixed64.FromFraction(3, 2), (Fixed64)(-2)),
            new(inward * Fixed64.FromFraction(3, 4), (Fixed64)(-2)),
            new(inward * Fixed64.FromFraction(3, 4), (Fixed64)2),
            new(inward * Fixed64.FromFraction(3, 2), (Fixed64)2),
        };

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            new Vector2d(face, Fixed64.Zero),
            axis,
            Fixed64.Two,
            Fixed64.FromFraction(1, 4),
            new Vector2d(face, Fixed64.Zero),
            vertices,
            out Vector2d normal,
            out Fixed64 depth));
        Assert.Equal(maximumFace ? Vector2d.Left : Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
    }

    [Fact]
    public void Contacts_RemainAuthoritativeAtTheScalarFace()
    {
        Vector2d scalarFace = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d[] vertices =
        {
            new(Fixed64.FromFraction(3, 2), (Fixed64)(-2)),
            new(Fixed64.FromFraction(3, 4), (Fixed64)(-2)),
            new(Fixed64.FromFraction(3, 4), (Fixed64)2),
            new(Fixed64.FromFraction(3, 2), (Fixed64)2),
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            scalarFace,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.FromFraction(1, 4),
            scalarFace,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(1, contactCount);
        Assert.Equal(Vector2d.Right, capsuleContacts[0].LocalPoint);
        Assert.Equal(
            new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero),
            capsuleContacts[0].LocalDisplacement);
        Assert.Equal(
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            convexContacts[0].LocalPoint);
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
        Assert.False(FixedSegment2d.TryGetCenteredAxisEndpoint(
            scalarFace,
            Vector2d.Right,
            Fixed64.Two,
            positive: false,
            out _));
        Assert.True(capsuleContacts[0].TryGetPoint(out _));
    }

    [Fact]
    public void Contacts_ExposeClampedDepthWithoutLosingOverlap()
    {
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One),
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(1, contactCount);
        Assert.Equal(Vector2d.Zero, capsuleContacts[0].LocalPoint);
        Assert.Equal(
            new Vector2d(Fixed64.Zero, Fixed64.MaxValue),
            capsuleContacts[0].LocalDisplacement);
        Assert.Equal(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            convexContacts[0].LocalPoint);
        Assert.Equal(Vector2d.Forward, normal);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.True(depthIsClamped);
    }

    [Fact]
    public void Contacts_ReturnFalseOnlyForSeparation()
    {
        Vector2d[] vertices =
        {
            new(4, -1),
            new(6, -1),
            new(6, 1),
            new(4, 1),
        };
        FixedPointAnchor2d[] capsuleContacts = new FixedPointAnchor2d[2];
        FixedPointAnchor2d[] convexContacts = new FixedPointAnchor2d[2];

        Assert.False(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(0, contactCount);
        Assert.Equal(Vector2d.Zero, normal);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(depthIsClamped);
    }

    [Fact]
    public void Contacts_RetainRadialSupportOnAxialTie()
    {
        Vector2d[] vertices =
        {
            Vector2d.Left,
            Vector2d.Backward,
            Vector2d.Right,
            Vector2d.Forward,
        };
        FixedPointAnchor2d[] capsuleContacts = new FixedPointAnchor2d[2];
        FixedPointAnchor2d[] convexContacts = new FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.One.Normalized,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(1, contactCount);
        Assert.Equal(Vector2d.Zero, capsuleContacts[0].LocalPoint);
        Assert.NotEqual(Vector2d.Zero, capsuleContacts[0].LocalDisplacement);
        Assert.True(capsuleContacts[0].TryGetPoint(out Vector2d point));
        Assert.Equal(capsuleContacts[0].LocalDisplacement, point);
        Assert.NotEqual(Vector2d.Zero, normal);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.True(depthIsClamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Contacts_CollapseClippedSideToOneFeatureContact(bool reverseAxis)
    {
        Vector2d[] vertices =
        {
            new(Fixed64.FromFraction(5, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), Fixed64.One),
            new(Fixed64.FromFraction(5, 2), Fixed64.One),
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.Zero,
            reverseAxis ? Vector2d.Backward : Vector2d.Forward,
            (Fixed64)10,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(1, contactCount);
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
    }

    [Fact]
    public void Contacts_UseTheClosestVertexFeatureAtACorner()
    {
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One),
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            new Vector2d(Fixed64.Two, Fixed64.Two),
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.Two,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(1, contactCount);
        Assert.Equal(
            new Vector2d(Fixed64.One, Fixed64.One),
            convexContacts[0].LocalPoint);
        Assert.True(normal.X < Fixed64.Zero);
        Assert.True(normal.Y < Fixed64.Zero);
        Assert.True(normal.IsNormalized());
        Assert.False(depthIsClamped);
        Assert.True(depth > Fixed64.Zero);
    }

    [Fact]
    public void Contacts_HandleRepeatedClosingVertexWithoutChangingTheFace()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.FromFraction(5, 2), -Fixed64.One),
            new(Fixed64.FromFraction(5, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), Fixed64.One),
            new(Fixed64.FromFraction(5, 2), Fixed64.One),
            new(Fixed64.FromFraction(5, 2), -Fixed64.One),
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(1, contactCount);
        Assert.Equal(
            new Vector2d(Fixed64.Two, Fixed64.Zero),
            capsuleContacts[0].LocalPoint);
        Assert.Equal(Vector2d.Right, capsuleContacts[0].LocalDisplacement);
        Assert.Equal(
            new Vector2d(Fixed64.FromFraction(5, 2), Fixed64.Zero),
            convexContacts[0].LocalPoint);
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
    }

    [Fact]
    public void Contacts_RetainLocalFeaturesUnderCommonRigidRotation()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.FromFraction(5, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), -Fixed64.One),
            new(Fixed64.FromFraction(9, 2), Fixed64.One),
            new(Fixed64.FromFraction(5, 2), Fixed64.One),
        };
        Span<FixedPointAnchor2d> baselineCapsule =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> baselineConvex =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> rotatedCapsule =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> rotatedConvex =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            baselineCapsule,
            baselineConvex,
            out int baselineCount,
            out Vector2d baselineNormal,
            out Fixed64 baselineDepth,
            out bool baselineDepthIsClamped));
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.HalfPi,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.HalfPi,
            vertices,
            rotatedCapsule,
            rotatedConvex,
            out int rotatedCount,
            out Vector2d rotatedNormal,
            out Fixed64 rotatedDepth,
            out bool rotatedDepthIsClamped));

        Assert.Equal(baselineCount, rotatedCount);
        Assert.Equal(baselineDepth, rotatedDepth);
        Assert.Equal(baselineDepthIsClamped, rotatedDepthIsClamped);
        Assert.True(Vector2d.TryRotate(
            baselineNormal,
            Fixed64.HalfPi,
            out Vector2d expectedRotatedNormal));
        Assert.Equal(expectedRotatedNormal, rotatedNormal);
        Assert.True(
            FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                Vector2d.Zero,
                Vector2d.Forward,
                (Fixed64)4,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.HalfPi,
                vertices,
                out Vector2d rotatedMinimumTranslationNormal,
                out Fixed64 rotatedMinimumTranslationDepth));
        Assert.Equal(expectedRotatedNormal, rotatedMinimumTranslationNormal);
        Assert.Equal(baselineDepth, rotatedMinimumTranslationDepth);
        for (int i = 0; i < baselineCount; i++)
        {
            Assert.Equal(
                baselineCapsule[i].LocalPoint,
                rotatedCapsule[i].LocalPoint);
            Assert.Equal(
                baselineCapsule[i].LocalDisplacement,
                rotatedCapsule[i].LocalDisplacement);
            Assert.Equal(
                baselineConvex[i].LocalPoint,
                rotatedConvex[i].LocalPoint);
            Assert.Equal(
                baselineConvex[i].LocalDisplacement,
                rotatedConvex[i].LocalDisplacement);
        }
    }

    [Fact]
    public void Contacts_RejectDegenerateConvexBoundary()
    {
        Vector2d[] vertices =
        {
            Vector2d.Zero,
            Vector2d.Zero,
            Vector2d.Zero,
        };
        FixedPointAnchor2d[] capsuleContacts = new FixedPointAnchor2d[2];
        FixedPointAnchor2d[] convexContacts = new FixedPointAnchor2d[2];

        Assert.False(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.One,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(0, contactCount);
        Assert.Equal(Vector2d.Zero, normal);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(depthIsClamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Contacts_UseVertexAxisForDiagonalSeparation(bool negativeSide)
    {
        Fixed64 sign = negativeSide ? -Fixed64.One : Fixed64.One;
        Vector2d[] vertices =
        {
            new(sign * Fixed64.FromFraction(4, 5), sign * Fixed64.FromFraction(4, 5)),
            new(sign * Fixed64.FromFraction(6, 5), sign * Fixed64.FromFraction(4, 5)),
            new(sign * Fixed64.FromFraction(6, 5), sign * Fixed64.FromFraction(6, 5)),
            new(sign * Fixed64.FromFraction(4, 5), sign * Fixed64.FromFraction(6, 5)),
        };
        FixedPointAnchor2d[] capsuleContacts = new FixedPointAnchor2d[2];
        FixedPointAnchor2d[] convexContacts = new FixedPointAnchor2d[2];

        Assert.False(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            vertices,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(0, contactCount);
        Assert.Equal(Vector2d.Zero, normal);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(depthIsClamped);
    }

    [Fact]
    public void MinimumTranslation_PreservesFirstAuthoredAxisAndVertexOnExactTies()
    {
        Vector2d[] vertices =
        {
            new(-Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half),
            new(-Fixed64.Half, Fixed64.Half),
        };

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Vector2d.Zero,
            vertices,
            out Vector2d normal,
            out Fixed64 depth));
        Assert.Equal(Vector2d.Forward, normal);
        Assert.Equal(Fixed64.FromFraction(3, 2), depth);
    }

    [Fact]
    public void MinimumTranslation_ReturnsFalseOnlyForSeparation()
    {
        Vector2d[] separated =
        {
            new(4, -1),
            new(6, -1),
            new(6, 1),
            new(4, 1),
        };

        Assert.False(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Zero,
            separated,
            out Vector2d normal,
            out Fixed64 depth));
        Assert.Equal(Vector2d.Zero, normal);
        Assert.Equal(Fixed64.Zero, depth);
    }

    [Fact]
    public void MinimumTranslation_RejectsInvalidShapeInputs()
    {
        Vector2d[] triangle =
        {
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.Forward,
        };

        Assert.Equal(
            "axisDirection",
            Assert.Throws<ArgumentException>(() =>
                FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                    Vector2d.Zero,
                    Vector2d.One,
                    Fixed64.One,
                    Fixed64.One,
                    Vector2d.Zero,
                    triangle,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "convexVertexOffsets",
            Assert.Throws<ArgumentException>(() =>
                FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                    Vector2d.Zero,
                    Vector2d.Right,
                    Fixed64.One,
                    Fixed64.One,
                    Vector2d.Zero,
                    triangle.AsSpan(0, 2),
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "radius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                    Vector2d.Zero,
                    Vector2d.Right,
                    Fixed64.One,
                    -Fixed64.One,
                    Vector2d.Zero,
                    triangle,
                    out _,
                    out _)).ParamName);

        Assert.Equal(
            "radius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                    Vector2d.Zero,
                    Vector2d.Right,
                    Fixed64.One,
                    -Fixed64.One,
                    Vector2d.Zero,
                    Fixed64.PiOver4,
                    triangle,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "convexVertexOffsets",
            Assert.Throws<ArgumentException>(() =>
                FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                    Vector2d.Zero,
                    Vector2d.Right,
                    Fixed64.One,
                    Fixed64.One,
                    Vector2d.Zero,
                    Fixed64.PiOver4,
                    triangle.AsSpan(0, 2),
                    out _,
                    out _)).ParamName);
    }

    [Fact]
    public void Contacts_RejectInvalidShapeAndOutputInputs()
    {
        Vector2d[] triangle =
        {
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.Forward,
        };
        Vector2d[] twoVertices = { Vector2d.Zero, Vector2d.Right };
        FixedPointAnchor2d[] twoContacts = new FixedPointAnchor2d[2];
        FixedPointAnchor2d[] oneContact = new FixedPointAnchor2d[1];

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                -Fixed64.One,
                Vector2d.Zero,
                Fixed64.Zero,
                triangle,
                twoContacts,
                twoContacts,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.Zero,
                twoVertices,
                twoContacts,
                twoContacts,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.Zero,
                triangle,
                oneContact,
                twoContacts,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.Zero,
                triangle,
                twoContacts,
                oneContact,
                out _,
                out _,
                out _,
                out _));
    }
}
