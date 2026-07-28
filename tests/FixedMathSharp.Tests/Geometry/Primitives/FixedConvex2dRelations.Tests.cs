//=======================================================================
// FixedConvex2dRelations.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using FluentAssertions;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedConvex2dRelationsTests
{
    [Fact]
    public void IsStrictlyConvex_ShouldClassifyFullDomainVerticesWithoutSaturation()
    {
        Span<Vector2d> convex = stackalloc Vector2d[4]
        {
            new(Fixed64.MinValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue),
        };
        Span<Vector2d> concave = stackalloc Vector2d[5]
        {
            new(Fixed64.MinValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MinValue),
            Vector2d.Zero,
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue),
        };
        Span<Vector2d> collinear = stackalloc Vector2d[4]
        {
            new(Fixed64.MinValue, Fixed64.MinValue),
            Vector2d.Zero,
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue),
        };

        FixedConvex2dRelations.IsStrictlyConvex(convex).Should().BeTrue();
        FixedConvex2dRelations.IsStrictlyConvex(concave).Should().BeFalse();
        FixedConvex2dRelations.IsStrictlyConvex(collinear).Should().BeFalse();
    }

    [Fact]
    public void IsStrictlyConvex_ShouldAcceptEitherBoundaryWinding()
    {
        Span<Vector2d> counterClockwise = stackalloc Vector2d[3]
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.Zero, Fixed64.One),
        };
        Span<Vector2d> clockwise = stackalloc Vector2d[3]
        {
            counterClockwise[2],
            counterClockwise[1],
            counterClockwise[0],
        };

        FixedConvex2dRelations.IsStrictlyConvex(counterClockwise).Should().BeTrue();
        FixedConvex2dRelations.IsStrictlyConvex(clockwise).Should().BeTrue();
    }

    [Fact]
    public void ContainsPoint_ShouldAcceptEitherBoundaryWinding()
    {
        Vector2d[] counterClockwise =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One),
        };
        Vector2d[] clockwise =
        {
            counterClockwise[3],
            counterClockwise[2],
            counterClockwise[1],
            counterClockwise[0],
        };

        Assert.True(FixedConvex2dRelations.ContainsPoint(
            Vector2d.Zero,
            Vector2d.Zero,
            counterClockwise));
        Assert.True(FixedConvex2dRelations.ContainsPoint(
            Vector2d.Zero,
            Vector2d.Zero,
            clockwise));
        Assert.True(FixedConvex2dRelations.ContainsPoint(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            Vector2d.Zero,
            counterClockwise));
    }

    private static readonly Vector2d[] UnitBoxOffsets =
    {
        new(-Fixed64.One, -Fixed64.One),
        new(Fixed64.One, -Fixed64.One),
        new(Fixed64.One, Fixed64.One),
        new(-Fixed64.One, Fixed64.One)
    };

    [Fact]
    public void SupportOffset_PreservesTheFirstAuthoredFeatureOnAnExactTie()
    {
        Assert.Equal(
            UnitBoxOffsets[0],
            FixedConvex2dRelations.GetSupportOffset(
                UnitBoxOffsets,
                Vector2d.Zero));
        Assert.Equal(
            UnitBoxOffsets[1],
            FixedConvex2dRelations.GetSupportOffset(
                UnitBoxOffsets,
                Vector2d.Right));
    }

    [Fact]
    public void RotatedBounds_TrackEveryAuthoredExtremum()
    {
        Vector2d[] offsets =
        {
            Vector2d.Zero,
            new(-Fixed64.Two, (Fixed64)(-3)),
            new((Fixed64)4, -Fixed64.One),
            new(Fixed64.One, (Fixed64)5),
        };

        FixedBoundArea bounds =
            FixedBoundArea.FromRotatedOffsetsClippedToDomain(
                Vector2d.Zero,
                Fixed64.Zero,
                offsets);

        Assert.Equal(new Vector2d(-Fixed64.Two, (Fixed64)(-3)), bounds.Min);
        Assert.Equal(new Vector2d((Fixed64)4, (Fixed64)5), bounds.Max);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CircleContacts_RemainExactAcrossScalarFaces(
        bool positiveFace)
    {
        Fixed64 centerX = positiveFace
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        Fixed64 inward = positiveFace ? -Fixed64.One : Fixed64.One;
        Vector2d convexOrigin = new(centerX, Fixed64.Zero);
        Vector2d[] convexOffsets =
        {
            new(inward * Fixed64.FromFraction(3, 2), -Fixed64.One),
            new(inward * Fixed64.FromFraction(1, 2), -Fixed64.One),
            new(inward * Fixed64.FromFraction(1, 2), Fixed64.One),
            new(inward * Fixed64.FromFraction(3, 2), Fixed64.One)
        };
        Vector2d circleCenter = new(
            centerX,
            Fixed64.Zero);

        Assert.True(FixedConvex2dRelations.TryGetCircleContact(
            circleCenter,
            Fixed64.Zero,
            Fixed64.One,
            convexOrigin,
            Fixed64.Zero,
            convexOffsets,
            out FixedPointAnchor2d circleContact,
            out FixedPointAnchor2d convexContact,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(
            positiveFace ? Vector2d.Left : Vector2d.Right,
            normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
        Assert.Equal(Vector2d.Zero, circleContact.LocalPoint);
        Assert.Equal(normal, circleContact.LocalDisplacement);
        Assert.Equal(
            new Vector2d(
                inward * Fixed64.FromFraction(1, 2),
                Fixed64.Zero),
            convexContact.LocalPoint);
    }

    [Fact]
    public void CircleContacts_RejectInvalidInputs()
    {
        Vector2d[] triangle =
        {
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.Forward
        };

        Assert.Equal(
            "circleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations.TryGetCircleContact(
                    Vector2d.Zero,
                    Fixed64.Zero,
                    -Fixed64.One,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    triangle,
                    out _,
                    out _,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "convexVertexOffsets",
            Assert.Throws<ArgumentException>(() =>
                FixedConvex2dRelations.TryGetCircleContact(
                    Vector2d.Zero,
                    Fixed64.Zero,
                    Fixed64.One,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    triangle.AsSpan(0, 2),
                    out _,
                    out _,
                    out _,
                    out _,
                    out _)).ParamName);
    }

    [Fact]
    public void ConvexContacts_ReturnStableOverlappingFacePair()
    {
        Span<FixedPointAnchor2d> firstContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> secondContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            UnitBoxOffsets,
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            Fixed64.Zero,
            UnitBoxOffsets,
            firstContacts,
            secondContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(2, contactCount);
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
        Assert.Equal(
            new Vector2d(Fixed64.One, Fixed64.One),
            firstContacts[0].LocalPoint);
        Assert.Equal(
            new Vector2d(Fixed64.One, -Fixed64.One),
            firstContacts[1].LocalPoint);
        Assert.Equal(
            new Vector2d(-Fixed64.One, Fixed64.One),
            secondContacts[0].LocalPoint);
        Assert.Equal(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            secondContacts[1].LocalPoint);
    }

    [Fact]
    public void ConvexContacts_IgnoreRepeatedBoundaryVertices()
    {
        Vector2d[] repeatedBoundaryVertex =
        {
            UnitBoxOffsets[0],
            UnitBoxOffsets[0],
            UnitBoxOffsets[1],
            UnitBoxOffsets[2],
            UnitBoxOffsets[3],
        };
        Span<FixedPointAnchor2d> firstContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> secondContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            repeatedBoundaryVertex,
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            Fixed64.Zero,
            UnitBoxOffsets,
            firstContacts,
            secondContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(2, contactCount);
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
    }

    [Fact]
    public void ConvexContacts_PreservePointAndFaceSupportFeatures()
    {
        Vector2d[] rightTriangle =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(-Fixed64.One, Fixed64.One),
            Vector2d.Right,
        };
        Vector2d[] leftTriangle =
        {
            new(Fixed64.One, -Fixed64.One),
            Vector2d.Left,
            new(Fixed64.One, Fixed64.One),
        };
        Span<FixedPointAnchor2d> first =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> second =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            rightTriangle,
            Vector2d.Right * Fixed64.Two,
            Fixed64.Zero,
            leftTriangle,
            first,
            second,
            out int pointCount,
            out Vector2d pointNormal,
            out Fixed64 pointDepth,
            out _));
        Assert.Equal(1, pointCount);
        Assert.True(pointNormal.IsNormalized());
        Assert.True(Vector2d.Dot(pointNormal, Vector2d.Right) > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, pointDepth);
        Assert.Equal(Vector2d.Right, first[0].LocalPoint);
        Assert.Equal(Vector2d.Left, second[0].LocalPoint);

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            rightTriangle,
            Vector2d.Right * Fixed64.Two,
            Fixed64.Zero,
            UnitBoxOffsets,
            first,
            second,
            out int firstPointCount,
            out _,
            out _,
            out _));
        Assert.Equal(1, firstPointCount);
        Assert.Equal(Vector2d.Right, first[0].LocalPoint);
        Assert.Equal(-Vector2d.Right, second[0].LocalPoint);

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            UnitBoxOffsets,
            Vector2d.Right * Fixed64.Two,
            Fixed64.Zero,
            leftTriangle,
            first,
            second,
            out int secondPointCount,
            out _,
            out _,
            out _));
        Assert.Equal(1, secondPointCount);
        Assert.Equal(Vector2d.Right, first[0].LocalPoint);
        Assert.Equal(Vector2d.Left, second[0].LocalPoint);
    }

    [Fact]
    public void ConvexContacts_ClampOnlyAnUnrepresentableFullDomainDepth()
    {
        Vector2d[] fullDomainBox =
        {
            new(Fixed64.MinValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue),
        };
        Span<FixedPointAnchor2d> first =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> second =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            fullDomainBox,
            Vector2d.Zero,
            Fixed64.Zero,
            fullDomainBox,
            first,
            second,
            out int contactCount,
            out _,
            out Fixed64 depth,
            out bool depthIsClamped));

        Assert.Equal(2, contactCount);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.True(depthIsClamped);

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            fullDomainBox,
            Vector2d.Zero,
            Fixed64.PiOver4,
            fullDomainBox,
            first,
            second,
            out contactCount,
            out _,
            out depth,
            out depthIsClamped));
        Assert.True(contactCount > 0);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.True(depthIsClamped);
    }

    [Fact]
    public void ConvexContacts_RoundFullDomainObliqueDepthToNearest()
    {
        const long expectedDepthRaw = 6_521_908_912_666_391_103L;
        Fixed64 along = (Fixed64)600_000_000;
        Fixed64 across = Fixed64.FromRaw(
            (Fixed64.MaxValue.m_rawValue - 3L) / 4L);
        Vector2d[] diamond =
        {
            new(-along - across, -along + across),
            new(along - across, along + across),
            new(along + across, along - across),
            new(-along + across, -along - across),
        };
        Span<FixedPointAnchor2d> first =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> second =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            diamond,
            Vector2d.Zero,
            Fixed64.Zero,
            diamond,
            first,
            second,
            out _,
            out _,
            out Fixed64 depth,
            out bool depthIsClamped));

        Assert.Equal(Fixed64.FromRaw(expectedDepthRaw), depth);
        Assert.False(depthIsClamped);
    }

    [Fact]
    public void ConvexContacts_ReturnFalseForSeparatedPolygons()
    {
        Span<FixedPointAnchor2d> firstContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> secondContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.False(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            UnitBoxOffsets,
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Zero,
            UnitBoxOffsets,
            firstContacts,
            secondContacts,
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
    public void ConvexContacts_RejectUndersizedOutputBuffers()
    {
        FixedPointAnchor2d[] full = new FixedPointAnchor2d[2];
        FixedPointAnchor2d[] shortBuffer = new FixedPointAnchor2d[1];

        Assert.Equal(
            "firstContacts",
            Assert.Throws<ArgumentException>(() =>
                FixedConvex2dRelations.TryGetConvexContacts(
                    Vector2d.Zero,
                    Fixed64.Zero,
                    UnitBoxOffsets,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    UnitBoxOffsets,
                    shortBuffer,
                    full,
                    out _,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "secondContacts",
            Assert.Throws<ArgumentException>(() =>
                FixedConvex2dRelations.TryGetConvexContacts(
                    Vector2d.Zero,
                    Fixed64.Zero,
                    UnitBoxOffsets,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    UnitBoxOffsets,
                    full,
                    shortBuffer,
                    out _,
                    out _,
                    out _,
                    out _)).ParamName);
    }

    [Fact]
    public void ConvexContacts_AtScalarFaceRetainLocalFeatures()
    {
        Span<FixedPointAnchor2d> baselineFirst =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> baselineSecond =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> scalarFirst =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> scalarSecond =
            stackalloc FixedPointAnchor2d[2];
        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            UnitBoxOffsets,
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            Fixed64.Zero,
            UnitBoxOffsets,
            baselineFirst,
            baselineSecond,
            out int baselineCount,
            out Vector2d baselineNormal,
            out Fixed64 baselineDepth,
            out bool baselineClamped));
        Vector2d firstOrigin = new(
            Fixed64.MaxValue - Fixed64.Two,
            Fixed64.Zero);
        Vector2d secondOrigin = new(
            Fixed64.MaxValue - Fixed64.Half,
            Fixed64.Zero);

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            firstOrigin,
            Fixed64.Zero,
            UnitBoxOffsets,
            secondOrigin,
            Fixed64.Zero,
            UnitBoxOffsets,
            scalarFirst,
            scalarSecond,
            out int scalarCount,
            out Vector2d scalarNormal,
            out Fixed64 scalarDepth,
            out bool scalarClamped));
        Assert.Equal(baselineCount, scalarCount);
        Assert.Equal(baselineNormal, scalarNormal);
        Assert.Equal(baselineDepth, scalarDepth);
        Assert.Equal(baselineClamped, scalarClamped);
        for (int i = 0; i < baselineCount; i++)
        {
            Assert.Equal(
                baselineFirst[i].LocalPoint,
                scalarFirst[i].LocalPoint);
            Assert.Equal(
                baselineSecond[i].LocalPoint,
                scalarSecond[i].LocalPoint);
        }
    }

    [Fact]
    public void ClosestPointOffset_AndContainmentRemainExactAtScalarFace()
    {
        Vector2d origin = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero);
        Vector2d[] offsets =
        {
            new(-Fixed64.Two, -Fixed64.One),
            new(-Fixed64.Half, -Fixed64.One),
            new(-Fixed64.Half, Fixed64.One),
            new(-Fixed64.Two, Fixed64.One)
        };
        Vector2d point = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 8),
            Fixed64.Zero);

        Assert.False(FixedConvex2dRelations.ContainsPoint(
            point,
            origin,
            offsets));
        Vector2d closestOffset =
            FixedConvex2dRelations.GetClosestPointOffset(
            point,
            origin,
            offsets);
        Assert.Equal(new Vector2d(-Fixed64.Half, Fixed64.Zero), closestOffset);
    }

    [Fact]
    public void SegmentAndCircleSweeps_ReturnFirstConvexBoundaryDistance()
    {
        Assert.True(FixedConvex2dRelations
            .TryGetSegmentFirstIntersectionDistance(
                new Vector2d((Fixed64)(-3), Fixed64.Zero),
                Vector2d.Right,
                (Fixed64)5,
                Vector2d.Zero,
                UnitBoxOffsets,
                out Fixed64 segmentDistance,
                out Vector2d segmentNormal,
                out Vector2d segmentContactOffset));
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            Vector2d.Right,
            (Fixed64)5,
            Vector2d.Zero,
            UnitBoxOffsets,
            out Fixed64 circleDistance,
            out Vector2d circleNormal,
            out Vector2d circleContactOffset));

        Assert.Equal((Fixed64)2, segmentDistance);
        Assert.Equal(Vector2d.Left, segmentNormal);
        Assert.Equal(new Vector2d(-Fixed64.One, Fixed64.Zero), segmentContactOffset);
        Assert.Equal(Fixed64.FromFraction(3, 2), circleDistance);
        Assert.Equal(Vector2d.Left, circleNormal);
        Assert.Equal(new Vector2d(-Fixed64.One, Fixed64.Zero), circleContactOffset);
    }

    [Fact]
    public void ConvexSweep_AtScalarFaceMatchesTranslatedDistanceAndNormal()
    {
        Vector2d[] halfBoxOffsets =
        {
            new(-Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half),
            new(-Fixed64.Half, Fixed64.Half)
        };
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            halfBoxOffsets,
            Vector2d.Right,
            (Fixed64)5,
            Vector2d.Zero,
            halfBoxOffsets,
            out Fixed64 baselineDistance,
            out Vector2d baselineNormal,
            out Vector2d baselineContactOffset));
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            new Vector2d(Fixed64.MaxValue - (Fixed64)3, Fixed64.Zero),
            halfBoxOffsets,
            Vector2d.Right,
            (Fixed64)5,
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            halfBoxOffsets,
            out Fixed64 scalarDistance,
            out Vector2d scalarNormal,
            out Vector2d scalarContactOffset));

        Assert.Equal((Fixed64)2, baselineDistance);
        Assert.Equal(Vector2d.Left, baselineNormal);
        Assert.Equal(baselineDistance, scalarDistance);
        Assert.Equal(baselineNormal, scalarNormal);
        Assert.Equal(baselineContactOffset, scalarContactOffset);
    }

    [Fact]
    public void CenteredCapsuleSweep_OnAxialTie_ReturnsCenteredTargetEdgeWitness()
    {
        Assert.True(FixedConvex2dRelations
            .TryGetSweptCenteredCapsuleFirstDistance(
                new Vector2d((Fixed64)(-4), Fixed64.Zero),
                Vector2d.Forward,
                Fixed64.Two,
                Fixed64.Half,
                Vector2d.Right,
                (Fixed64)6,
                Vector2d.Zero,
                UnitBoxOffsets,
                out Fixed64 distance,
                out Vector2d normal,
                out Vector2d contactOffset));

        Assert.Equal(Fixed64.FromFraction(5, 2), distance);
        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(new Vector2d(-Fixed64.One, Fixed64.Zero), contactOffset);
    }

    [Fact]
    public void TangentCircleSweep_SelectsTouchedCornerWitnessAndNormal()
    {
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            new Vector2d((Fixed64)(-2), Fixed64.One),
            Fixed64.Half,
            Vector2d.Right,
            (Fixed64)4,
            Vector2d.Zero,
            new[]
            {
                new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half)
            },
            out Fixed64 distance,
            out Vector2d normal,
            out Vector2d contactOffset));

        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
        Assert.Equal(Vector2d.Forward, normal);
        Assert.Equal(
            new Vector2d(-Fixed64.Half, Fixed64.Half),
            contactOffset);
    }

    [Fact]
    public void RotatedRelations_RetainLocalAnchorsAcrossScalarFace()
    {
        Vector2d origin = new(Fixed64.MaxValue, Fixed64.Zero);
        Fixed64 rotation = Fixed64.HalfPi;

        FixedBoundArea bounds =
            FixedBoundArea.FromRotatedOffsetsClippedToDomain(
                origin,
                rotation,
                UnitBoxOffsets);
        Assert.Equal(
            new Vector2d(
                Fixed64.MaxValue - Fixed64.One,
                -Fixed64.One),
            bounds.Min);
        Assert.Equal(
            new Vector2d(
                Fixed64.MaxValue,
                Fixed64.One),
            bounds.Max);
        Assert.True(FixedConvex2dRelations.ContainsPoint(
            origin,
            origin,
            rotation,
            UnitBoxOffsets));

        FixedPointAnchor2d support =
            FixedConvex2dRelations.GetSupportAnchor(
                origin,
                rotation,
                UnitBoxOffsets,
                Vector2d.Right);
        Assert.Equal(origin, support.Origin);
        Assert.Equal(rotation, support.Rotation);
        Assert.Equal(UnitBoxOffsets[0], support.LocalPoint);
        Assert.False(support.TryGetPoint(out _));

        FixedPointAnchor2d closest =
            FixedConvex2dRelations.GetClosestPointAnchor(
            new Vector2d(
                Fixed64.MaxValue - Fixed64.Two,
                Fixed64.Zero),
            origin,
            rotation,
            UnitBoxOffsets);
        Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.One), closest.LocalPoint);
        Assert.True(closest.TryGetPoint(out Vector2d closestPoint));
        Assert.Equal(
            new Vector2d(
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.Zero),
            closestPoint);
    }

    [Fact]
    public void RotatedConvexContacts_ReturnSuppliedFrameAnchors()
    {
        Span<FixedPointAnchor2d> firstContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> secondContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.HalfPi,
            UnitBoxOffsets,
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            Fixed64.Zero,
            UnitBoxOffsets,
            firstContacts,
            secondContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));

        Assert.Equal(2, contactCount);
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
        for (int i = 0; i < contactCount; i++)
        {
            Assert.Equal(Fixed64.HalfPi, firstContacts[i].Rotation);
            Assert.Equal(Fixed64.Zero, secondContacts[i].Rotation);
            Assert.True(firstContacts[i].TryGetOffsetFrom(
                secondContacts[i],
                out Vector2d separation));
            Assert.Equal(Fixed64.Half, separation.X);
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void RotatedConvexContacts_KeepFirstAuthoredAxisOnExactDepthTie(
        int verticalSign)
    {
        Vector2d[] innerOffsets =
        {
            new(-Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half),
            new(-Fixed64.Half, Fixed64.Half),
        };
        Span<FixedPointAnchor2d> firstContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> secondContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            UnitBoxOffsets,
            new Vector2d(
                -Fixed64.Half,
                verticalSign * Fixed64.Half),
            Fixed64.PiOver4,
            innerOffsets,
            firstContacts,
            secondContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));

        Assert.True(contactCount > 0);
        Assert.Equal((Fixed64)verticalSign * Vector2d.Forward, normal);
        Assert.True(depth > Fixed64.Zero);
        Assert.False(depthIsClamped);

        Vector2d[] reversedOuter =
        {
            UnitBoxOffsets[3],
            UnitBoxOffsets[2],
            UnitBoxOffsets[1],
            UnitBoxOffsets[0],
        };
        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(
            Vector2d.Zero,
            Fixed64.Zero,
            reversedOuter,
            new Vector2d(
                -Fixed64.Half,
                verticalSign * Fixed64.Half),
            Fixed64.PiOver4,
            innerOffsets,
            firstContacts,
            secondContacts,
            out _,
            out Vector2d reversedNormal,
            out Fixed64 reversedDepth,
            out bool reversedDepthIsClamped));
        Assert.Equal(normal, reversedNormal);
        Assert.Equal(depth, reversedDepth);
        Assert.Equal(depthIsClamped, reversedDepthIsClamped);
    }

    [Fact]
    public void RotatedBounds_RejectEmptyOffsetSet()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedBoundArea.FromRotatedOffsetsClippedToDomain(
                Vector2d.Zero,
                Fixed64.Zero,
                ReadOnlySpan<Vector2d>.Empty));
    }

    [Fact]
    public void RotatedCircleContact_RetainsConvexLocalWitness()
    {
        Vector2d[] rectangle =
        {
            new((Fixed64)(-2), -Fixed64.One),
            new((Fixed64)2, -Fixed64.One),
            new((Fixed64)2, Fixed64.One),
            new((Fixed64)(-2), Fixed64.One)
        };

        Assert.True(FixedConvex2dRelations.TryGetCircleContact(
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.HalfPi,
            rectangle,
            out FixedPointAnchor2d circleContact,
            out FixedPointAnchor2d convexContact,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));

        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(Fixed64.Half, depth);
        Assert.False(depthIsClamped);
        Assert.Equal(Vector2d.Zero, circleContact.LocalPoint);
        Assert.Equal(Vector2d.Left, circleContact.LocalDisplacement);
        Assert.Equal(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            convexContact.LocalPoint);
        Assert.True(circleContact.TryGetOffsetFrom(
            convexContact,
            out Vector2d separation));
        Assert.Equal(-Fixed64.Half, separation.X);
    }

    [Fact]
    public void CircleContacts_RetainLocalFeaturesUnderCommonRigidRotation()
    {
        Span<FixedPointAnchor2d> baselineCircle =
            stackalloc FixedPointAnchor2d[1];
        Span<FixedPointAnchor2d> baselineConvex =
            stackalloc FixedPointAnchor2d[1];
        Span<FixedPointAnchor2d> rotatedCircle =
            stackalloc FixedPointAnchor2d[1];
        Span<FixedPointAnchor2d> rotatedConvex =
            stackalloc FixedPointAnchor2d[1];

        Assert.True(FixedConvex2dRelations.TryGetCircleContact(
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            Fixed64.Zero,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.Zero,
            UnitBoxOffsets,
            out baselineCircle[0],
            out baselineConvex[0],
            out Vector2d baselineNormal,
            out Fixed64 baselineDepth,
            out bool baselineDepthIsClamped));
        Assert.True(FixedConvex2dRelations.TryGetCircleContact(
            new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)),
            Fixed64.HalfPi,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.HalfPi,
            UnitBoxOffsets,
            out rotatedCircle[0],
            out rotatedConvex[0],
            out Vector2d rotatedNormal,
            out Fixed64 rotatedDepth,
            out bool rotatedDepthIsClamped));

        Assert.Equal(baselineDepth, rotatedDepth);
        Assert.Equal(baselineDepthIsClamped, rotatedDepthIsClamped);
        Assert.True(Vector2d.TryRotate(
            baselineNormal,
            Fixed64.HalfPi,
            out Vector2d expectedRotatedNormal));
        Assert.Equal(expectedRotatedNormal, rotatedNormal);
        Assert.Equal(
            baselineCircle[0].LocalPoint,
            rotatedCircle[0].LocalPoint);
        Assert.Equal(
            baselineCircle[0].LocalDisplacement,
            rotatedCircle[0].LocalDisplacement);
        Assert.Equal(
            baselineConvex[0].LocalPoint,
            rotatedConvex[0].LocalPoint);
        Assert.Equal(
            baselineConvex[0].LocalDisplacement,
            rotatedConvex[0].LocalDisplacement);
    }

    [Fact]
    public void RotatedCapsuleContact_ReturnsTwoLocalFrameAnchors()
    {
        Vector2d[] rectangle =
        {
            new((Fixed64)(-2), -Fixed64.One),
            new((Fixed64)2, -Fixed64.One),
            new((Fixed64)2, Fixed64.One),
            new((Fixed64)(-2), Fixed64.One)
        };
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            new Vector2d(Fixed64.FromFraction(5, 4), Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.Half,
            Vector2d.Zero,
            Fixed64.HalfPi,
            rectangle,
            capsuleContacts,
            convexContacts,
            out int contactCount,
            out Vector2d normal,
            out Fixed64 depth,
            out bool depthIsClamped));

        Assert.Equal(2, contactCount);
        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), depth);
        Assert.False(depthIsClamped);
        Assert.All(
            convexContacts.ToArray(),
            contact => Assert.Equal(
                Fixed64.HalfPi,
                contact.Rotation));
    }

    [Fact]
    public void RotatedSweeps_ReturnCanonicalTargetAnchors()
    {
        Vector2d[] rectangle =
        {
            new((Fixed64)(-2), -Fixed64.One),
            new((Fixed64)2, -Fixed64.One),
            new((Fixed64)2, Fixed64.One),
            new((Fixed64)(-2), Fixed64.One)
        };
        Vector2d[] mover =
        {
            new(-Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half),
            new(-Fixed64.Half, Fixed64.Half)
        };

        Assert.True(FixedConvex2dRelations
            .TryGetSegmentFirstIntersectionDistance(
                new Vector2d((Fixed64)3, Fixed64.Zero),
                Vector2d.Left,
                (Fixed64)5,
                Vector2d.Zero,
                Fixed64.HalfPi,
                rectangle,
                out Fixed64 segmentDistance,
                out Vector2d segmentNormal,
                out FixedPointAnchor2d segmentContact));
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Vector2d.Left,
            (Fixed64)5,
            Vector2d.Zero,
            Fixed64.HalfPi,
            rectangle,
            out Fixed64 circleDistance,
            out Vector2d circleNormal,
            out FixedPointAnchor2d circleContact));
        Assert.True(FixedConvex2dRelations
            .TryGetSweptCenteredCapsuleFirstDistance(
                new Vector2d((Fixed64)3, Fixed64.Zero),
                Vector2d.Forward,
                Fixed64.One,
                Fixed64.Half,
                Vector2d.Left,
                (Fixed64)5,
                Vector2d.Zero,
                Fixed64.HalfPi,
                rectangle,
                out Fixed64 capsuleDistance,
                out Vector2d capsuleNormal,
                out FixedPointAnchor2d capsuleContact));
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.HalfPi,
            mover,
            Vector2d.Left,
            (Fixed64)5,
            Vector2d.Zero,
            Fixed64.HalfPi,
            rectangle,
            out Fixed64 convexDistance,
            out Vector2d convexNormal,
            out FixedPointAnchor2d convexContact));

        Assert.Equal((Fixed64)2, segmentDistance);
        Assert.Equal(Fixed64.FromFraction(3, 2), circleDistance);
        Assert.Equal(Fixed64.FromFraction(3, 2), capsuleDistance);
        Assert.Equal(Fixed64.FromFraction(3, 2), convexDistance);
        Assert.Equal(Vector2d.Right, segmentNormal);
        Assert.Equal(segmentNormal, circleNormal);
        Assert.Equal(segmentNormal, capsuleNormal);
        Assert.Equal(segmentNormal, convexNormal);
        Assert.Equal(Fixed64.HalfPi, segmentContact.Rotation);
        Assert.Equal(Fixed64.HalfPi, circleContact.Rotation);
        Assert.Equal(Fixed64.HalfPi, capsuleContact.Rotation);
        Assert.Equal(Fixed64.HalfPi, convexContact.Rotation);
    }
}
