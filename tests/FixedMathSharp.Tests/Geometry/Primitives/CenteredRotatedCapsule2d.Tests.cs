using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredRotatedCapsule2dTests
{
    [Fact]
    public void RotatedPointAndSupportRelations_UseTheScalarRigidFrame()
    {
        Fixed64 rotation = Fixed64.PiOver4;
        Vector2d center = new(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero);

        Assert.True(FixedSegment2d.ContainsPointInCenteredCapsule(
            center,
            center,
            rotation,
            Fixed64.Two,
            Fixed64.Half,
            Fixed64.Zero,
            strict: false));
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleSupport(
            center,
            rotation,
            Fixed64.Two,
            Fixed64.Half,
            Vector2d.Left,
            out Vector2d support));
        Assert.True(support.X < center.X);
    }

    [Fact]
    public void RotatedCapsuleContact_UsesBothScalarRigidFrames()
    {
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero,
            Fixed64.PiOver4,
            Fixed64.Two,
            Fixed64.Half,
            Vector2d.Right,
            -Fixed64.PiOver4,
            Fixed64.Two,
            Fixed64.Half,
            Vector2d.Right,
            out FixedContactAnchors2d contact));

        Assert.True(contact.Depth >= Fixed64.Zero);
        Assert.Equal(Fixed64.PiOver4, contact.FirstAnchor.Rotation);
        Assert.Equal(-Fixed64.PiOver4, contact.SecondAnchor.Rotation);
    }

    [Fact]
    public void RotatedCapsuleSweep_UsesScalarRigidFrames()
    {
        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Fixed64.PiOver4,
            Fixed64.Two,
            Fixed64.Half,
            Vector2d.Right,
            (Fixed64)8,
            new Vector2d((Fixed64)2, Fixed64.Zero),
            -Fixed64.PiOver4,
            Fixed64.Two,
            Fixed64.Half,
            out Fixed64 distance,
            out Vector2d normal));

        Assert.True(distance >= Fixed64.Zero);
        Assert.True(distance <= (Fixed64)8);
        Assert.True(normal.IsNormalized());
    }

    [Fact]
    public void RotatedCapsuleRelations_MatchExplicitWorldAxisContracts()
    {
        Fixed64 rotation = Fixed64.HalfPi;
        Vector2d axis = new(
            -FixedMath.Sin(rotation),
            FixedMath.Cos(rotation));
        Vector2d center = Vector2d.Zero;
        Vector2d point = new(Fixed64.Zero, Fixed64.Two);
        Fixed64 length = Fixed64.Two;
        Fixed64 radius = Fixed64.Half;

        Assert.Equal(
            FixedSegment2d.ContainsPointInCenteredCapsule(
                point,
                center,
                axis,
                length,
                radius),
            FixedSegment2d.ContainsPointInCenteredCapsule(
                point,
                center,
                rotation,
                length,
                radius));
        Assert.Equal(
            FixedSegment2d.ContainsPointInCenteredCapsule(
                point,
                center,
                axis,
                length,
                radius,
                Fixed64.Half),
            FixedSegment2d.ContainsPointInCenteredCapsule(
                point,
                center,
                rotation,
                length,
                radius,
                Fixed64.Half));
        Assert.Equal(
            FixedSegment2d.GetDirectionFromCenteredAxis(
                point,
                center,
                axis,
                length),
            FixedSegment2d.GetDirectionFromCenteredAxis(
                point,
                center,
                rotation,
                length));
        Assert.Equal(
            FixedSegment2d.GetDistanceToCenteredCapsule(
                point,
                center,
                axis,
                length,
                radius),
            FixedSegment2d.GetDistanceToCenteredCapsule(
                point,
                center,
                rotation,
                length,
                radius));
        Assert.Equal(
            FixedSegment2d.TryGetDistanceToCenteredCapsule(
                point,
                center,
                axis,
                length,
                radius,
                out Fixed64 axisDistance),
            FixedSegment2d.TryGetDistanceToCenteredCapsule(
                point,
                center,
                rotation,
                length,
                radius,
                out Fixed64 rotatedDistance));
        Assert.Equal(axisDistance, rotatedDistance);
        Assert.Equal(
            FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
                point,
                center,
                axis,
                length,
                radius,
                Vector2d.Forward,
                out Vector2d axisSurfacePoint),
            FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
                point,
                center,
                rotation,
                length,
                radius,
                Vector2d.Forward,
                out Vector2d rotatedSurfacePoint));
        Assert.Equal(axisSurfacePoint, rotatedSurfacePoint);
        Assert.Equal(
            FixedSegment2d.TryGetSurfaceOffsetOnCenteredCapsule(
                point,
                center,
                axis,
                length,
                radius,
                Vector2d.Forward,
                out Vector2d axisSurfaceOffset),
            FixedSegment2d.TryGetSurfaceOffsetOnCenteredCapsule(
                point,
                center,
                rotation,
                length,
                radius,
                Vector2d.Forward,
                out Vector2d rotatedSurfaceOffset));
        Assert.Equal(axisSurfaceOffset, rotatedSurfaceOffset);

        Vector2d secondCenter = Vector2d.Right;
        Fixed64 secondRotation = -Fixed64.PiOver4;
        Vector2d secondAxis = new(
            -FixedMath.Sin(secondRotation),
            FixedMath.Cos(secondRotation));
        Assert.Equal(
            FixedSegment2d.DoCenteredCapsulesOverlap(
                center,
                axis,
                length,
                radius,
                secondCenter,
                secondAxis,
                length,
                radius),
            FixedSegment2d.DoCenteredCapsulesOverlap(
                center,
                rotation,
                length,
                radius,
                secondCenter,
                secondRotation,
                length,
                radius));

        Vector2d[] square =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One),
        };
        Assert.Equal(
            FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                center,
                axis,
                length,
                radius,
                center,
                square,
                out Vector2d axisNormal,
                out Fixed64 axisDepth),
            FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                center,
                rotation,
                length,
                radius,
                center,
                square,
                out Vector2d rotatedNormal,
                out Fixed64 rotatedDepth));
        Assert.Equal(axisNormal, rotatedNormal);
        Assert.Equal(axisDepth, rotatedDepth);

        var targetAxis = new FixedSegment2d(
            new Vector2d((Fixed64)2, -Fixed64.One),
            new Vector2d((Fixed64)2, Fixed64.One));
        Assert.Equal(
            FixedSegment2d.TryGetSweptCenteredCapsuleSegmentFirstDistance(
                new Vector2d((Fixed64)(-4), Fixed64.Zero),
                axis,
                length,
                radius,
                Vector2d.Right,
                (Fixed64)8,
                targetAxis,
                radius,
                out Fixed64 axisSweepDistance,
                out Vector2d axisSweepNormal),
            FixedSegment2d.TryGetSweptCenteredCapsuleSegmentFirstDistance(
                new Vector2d((Fixed64)(-4), Fixed64.Zero),
                rotation,
                length,
                radius,
                Vector2d.Right,
                (Fixed64)8,
                targetAxis,
                radius,
                out Fixed64 rotatedSweepDistance,
                out Vector2d rotatedSweepNormal));
        Assert.Equal(axisSweepDistance, rotatedSweepDistance);
        Assert.Equal(axisSweepNormal, rotatedSweepNormal);

        var query = new FixedSegment2d(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero));
        Assert.Equal(
            query.TryGetCapsuleIntersectionDistanceInterval(
                center,
                axis,
                length,
                radius,
                Fixed64.Zero,
                (Fixed64)8,
                out Fixed64 axisEntry,
                out Fixed64 axisExit,
                out bool axisStartContained,
                out bool axisEndContained),
            query.TryGetCapsuleIntersectionDistanceInterval(
                center,
                rotation,
                length,
                radius,
                Fixed64.Zero,
                (Fixed64)8,
                out Fixed64 rotatedEntry,
                out Fixed64 rotatedExit,
                out bool rotatedStartContained,
                out bool rotatedEndContained));
        Assert.Equal(axisEntry, rotatedEntry);
        Assert.Equal(axisExit, rotatedExit);
        Assert.Equal(axisStartContained, rotatedStartContained);
        Assert.Equal(axisEndContained, rotatedEndContained);
    }

    [Fact]
    public void RotatedCapsuleContracts_RejectInvalidAuthoredGeometry()
    {
        Vector2d[] triangle =
        {
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.Forward,
        };
        var query = new FixedSegment2d(Vector2d.Left, Vector2d.Right);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment2d.ContainsPointInCenteredCapsule(
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                -Fixed64.Epsilon,
                strict: false));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment2d.TryGetCenteredCapsuleSupport(
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                -Fixed64.Epsilon,
                Vector2d.Right,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Zero,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetSurfaceOffsetOnCenteredCapsule(
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Zero,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetCenteredCapsulesContact(
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Zero,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Zero,
                triangle.AsSpan(0, 2),
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query.TryGetCapsuleIntersectionDistanceInterval(
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                -Fixed64.Epsilon,
                Fixed64.Two,
                out _,
                out _,
                out _,
                out _));
    }

    [Fact]
    public void RotatedCapsuleConvexSweep_MatchesExplicitAxisForOverlapHitAndMiss()
    {
        Fixed64 capsuleRotation = Fixed64.HalfPi;
        Vector2d capsuleAxis = new(
            -FixedMath.Sin(capsuleRotation),
            FixedMath.Cos(capsuleRotation));
        Fixed64 convexRotation = Fixed64.PiOver4;
        Vector2d[] square =
        {
            new(-Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half),
            new(-Fixed64.Half, Fixed64.Half),
        };

        AssertSweepParity(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Zero);
        AssertSweepParity(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            new Vector2d((Fixed64)2, Fixed64.Zero));
        AssertSweepParity(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Left,
            Fixed64.One,
            new Vector2d((Fixed64)2, Fixed64.Zero));

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(
                Vector2d.Zero,
                capsuleRotation,
                -Fixed64.Epsilon,
                Fixed64.Half,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                convexRotation,
                square,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(
                Vector2d.Zero,
                capsuleRotation,
                Fixed64.One,
                -Fixed64.Epsilon,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                convexRotation,
                square,
                out _,
                out _,
                out _));

        void AssertSweepParity(
            Vector2d capsuleCenter,
            Vector2d direction,
            Fixed64 maximumDistance,
            Vector2d convexOrigin)
        {
            bool explicitHit =
                FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(
                    capsuleCenter,
                    capsuleAxis,
                    Fixed64.One,
                    Fixed64.Half,
                    direction,
                    maximumDistance,
                    convexOrigin,
                    convexRotation,
                    square,
                    out Fixed64 explicitDistance,
                    out Vector2d explicitNormal,
                    out FixedPointAnchor2d explicitContact);
            bool rotatedHit =
                FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(
                    capsuleCenter,
                    capsuleRotation,
                    Fixed64.One,
                    Fixed64.Half,
                    direction,
                    maximumDistance,
                    convexOrigin,
                    convexRotation,
                    square,
                    out Fixed64 rotatedDistance,
                    out Vector2d rotatedNormal,
                    out FixedPointAnchor2d rotatedContact);

            Assert.Equal(explicitHit, rotatedHit);
            Assert.Equal(explicitDistance, rotatedDistance);
            Assert.Equal(explicitNormal, rotatedNormal);
            Assert.Equal(explicitContact, rotatedContact);
        }
    }
}
