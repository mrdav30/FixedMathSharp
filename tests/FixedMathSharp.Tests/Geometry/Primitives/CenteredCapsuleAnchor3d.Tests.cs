//=======================================================================
// CenteredCapsuleAnchor3d.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCapsuleAnchor3dTests
{
    [Fact]
    public void SupportAnchor_RejectsOnlyTheUnrepresentableFinalCoordinate()
    {
        FixedPointAnchor anchor =
            FixedSegment.GetCenteredCapsuleSupportAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Forward,
                    -Fixed64.HalfPi),
                Fixed64.FromRaw(1L),
                Fixed64.Zero,
                Vector3d.Right);
        FixedPointAnchor origin = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.False(anchor.TryGetPoint(out _));
        Assert.False(anchor.TryGetOffsetFrom(origin, out _));
        Assert.False(anchor.TryGetScaledOffsetFrom(
            origin,
            Fixed64.One,
            out _));
        Assert.False(anchor.TryGetLocalPointIn(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out _));
    }

    [Fact]
    public void SupportAnchor_RetainsOddRawFullLengthUntilWorldMaterialization()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        Vector3d center = new(Fixed64.Zero, rawUnit, Fixed64.Zero);

        FixedPointAnchor positive =
            FixedSegment.GetCenteredCapsuleSupportAnchor(
                center,
                FixedQuaternion.Identity,
                rawUnit,
                Fixed64.Zero,
                Vector3d.Up);
        FixedPointAnchor negative =
            FixedSegment.GetCenteredCapsuleSupportAnchor(
                center,
                FixedQuaternion.Identity,
                rawUnit,
                Fixed64.Zero,
                Vector3d.Down);

        Assert.Equal(Fixed64.One.m_rawValue, positive.ExactLocalTerm.Y);
        Assert.Equal(-Fixed64.One.m_rawValue, negative.ExactLocalTerm.Y);
        Assert.True(positive.TryGetPoint(out Vector3d positivePoint));
        Assert.True(negative.TryGetPoint(out Vector3d negativePoint));
        Assert.Equal(Fixed64.FromRaw(2L), positivePoint.Y);
        Assert.Equal(Fixed64.Zero, negativePoint.Y);
        var centerAnchor = new FixedPointAnchor(
            center,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Assert.True(positive.TryGetOffsetFrom(
            centerAnchor,
            out Vector3d positiveOffset));
        Assert.Equal(Fixed64.Zero, positiveOffset.Y);
        Assert.True(negative.TryGetOffsetFrom(
            centerAnchor,
            out Vector3d negativeOffset));
        Assert.Equal(Fixed64.Zero, negativeOffset.Y);
        Assert.True(positive.TryGetScaledOffsetFrom(
            centerAnchor,
            Fixed64.Zero,
            out Vector3d zeroScaledOffset));
        Assert.Equal(Vector3d.Zero, zeroScaledOffset);
        Assert.True(positive.TryGetScaledOffsetFrom(
            centerAnchor,
            Fixed64.Two,
            out Vector3d doubledOffset));
        Assert.Equal(rawUnit, doubledOffset.Y);
        Assert.True(positive.TryGetProjectedOffsetFrom(
            centerAnchor,
            Vector3d.Up,
            out Fixed64 projection));
        Assert.Equal(Fixed64.Zero, projection);
        Assert.Equal(
            Fixed64.Zero,
            positive.ProjectNonNegativeOffsetFrom(
                centerAnchor,
                Vector3d.Up));
        Assert.True(positive.TryGetLocalPointIn(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out Vector3d positiveLocal));
        Assert.Equal(Fixed64.FromRaw(2L), positiveLocal.Y);
        Assert.True(
            centerAnchor.CompareSquaredDistance(
                positive,
                centerAnchor) > 0);
        Assert.NotEqual(positive, negative);
        Assert.NotEqual(0, positive.CompareLocalFeature(negative));
        Assert.NotEqual(
            positive.GetLocalFeatureHash64(),
            negative.GetLocalFeatureHash64());
        Assert.InRange(
            positive.ExactLocalTerm.Y,
            -FixedPointAnchorTerm3d.MaximumResidualMagnitude,
            FixedPointAnchorTerm3d.MaximumResidualMagnitude);
        Assert.InRange(
            negative.ExactLocalTerm.Y,
            -FixedPointAnchorTerm3d.MaximumResidualMagnitude,
            FixedPointAnchorTerm3d.MaximumResidualMagnitude);
    }

    [Fact]
    public void SupportAnchor_RetainsTheAuthoritativeRigidFrame()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.FromFraction(-1, 4));
        Vector3d center = new(2_000_000_000, 2_000_000_000, 0);
        Vector3d direction = rotation.Rotate(Vector3d.Up).Normalized;

        FixedPointAnchor anchor =
            FixedSegment.GetCenteredCapsuleSupportAnchor(
                center,
                rotation,
                Fixed64.Two,
                Fixed64.Half,
                direction);

        Assert.Equal(center, anchor.Origin);
        Assert.Equal(rotation, anchor.Rotation);
        Assert.True(anchor.TryGetPoint(out Vector3d support));
        Assert.True(anchor.TryGetOffsetFrom(
            new FixedPointAnchor(
                center,
                FixedQuaternion.Identity,
                Vector3d.Zero),
            out Vector3d offset));
        Assert.Equal(support, center + offset);
        Assert.True(Vector3d.Dot(offset, direction) > Fixed64.Zero);
    }

    [Fact]
    public void SupportAnchor_ValidatesRigidFrameAndGeometry()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.GetCenteredCapsuleSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Up));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.GetCenteredCapsuleSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                -Fixed64.One,
                Fixed64.One,
                Vector3d.Up));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.GetCenteredCapsuleSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Up));
    }

    [Fact]
    public void WorldFrameSurfaceAnchor_DoesNotInverseRotateTheSurfaceNormal()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            Fixed64.FromFraction(1472, 997));
        Vector3d center = new(3, 4, 5);
        Vector3d axis = rotation.Rotate(Vector3d.Up).Normalized;
        Vector3d normal = rotation.Rotate(Vector3d.Right).Normalized;

        FixedPointAnchor anchor =
            FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                center,
                center,
                axis,
                Fixed64.Two,
                Fixed64.Half,
                normal);

        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(center + normal * Fixed64.Half, point);
    }

    [Fact]
    public void RigidFrameSurfaceAnchor_ProjectsAgainstTheExactQuaternionBasis()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.Pi / (Fixed64)7);
        Vector3d center = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d point = Vector3d.Zero;

        FixedPointAnchor anchor =
            FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                point,
                center,
                rotation,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.Zero,
                Vector3d.Right);

        Assert.Equal(
            GetExactInverseRotatedYRaw(point, center, rotation),
            anchor.LocalPoint.Y.m_rawValue);
    }

    [Fact]
    public void ClosestSurfaceAnchor_RetainsOddFullLengthCapFeature()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        Vector3d center = new(Fixed64.Zero, rawUnit, Fixed64.Zero);

        Assert.True(
            FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                new Vector3d(Fixed64.Zero, Fixed64.FromRaw(3L), Fixed64.Zero),
                center,
                FixedQuaternion.Identity,
                Vector3d.Up,
                rawUnit,
                Fixed64.Zero,
                Vector3d.Right,
                out FixedPointAnchor anchor,
                out Vector3d normal,
                out Fixed64 signedDistance));

        Assert.Equal(Fixed64.One.m_rawValue, anchor.ExactLocalTerm.Y);
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.FromRaw(2L), point.Y);
        Assert.Equal(Vector3d.Up, normal);
        Assert.Equal(Fixed64.FromRaw(2L), signedDistance);
    }

    [Fact]
    public void ClosestSurfaceAnchor_ReportsContainedCenterAsNegativeDistance()
    {
        Assert.True(
            FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                Vector3d.Zero,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                Vector3d.Right,
                out FixedPointAnchor anchor,
                out Vector3d normal,
                out Fixed64 signedDistance));

        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Vector3d.Right, point);
        Assert.Equal(Vector3d.Right, normal);
        Assert.Equal(-Fixed64.One, signedDistance);
    }

    internal static long GetExactInverseRotatedYRaw(
        Vector3d point,
        Vector3d center,
        FixedQuaternion rotation)
    {
        BigInteger x = rotation.X.m_rawValue;
        BigInteger y = rotation.Y.m_rawValue;
        BigInteger z = rotation.Z.m_rawValue;
        BigInteger w = rotation.W.m_rawValue;
        BigInteger denominator =
            (x * x)
            + (y * y)
            + (z * z)
            + (w * w);
        BigInteger basisYx = 2 * ((x * y) - (z * w));
        BigInteger basisYy =
            (y * y)
            - (x * x)
            - (z * z)
            + (w * w);
        BigInteger basisYz = 2 * ((y * z) + (x * w));
        BigInteger numerator =
            ((BigInteger)point.X.m_rawValue - center.X.m_rawValue) * basisYx
            + ((BigInteger)point.Y.m_rawValue - center.Y.m_rawValue) * basisYy
            + ((BigInteger)point.Z.m_rawValue - center.Z.m_rawValue) * basisYz;
        BigInteger quotient = BigInteger.DivRem(
            BigInteger.Abs(numerator),
            denominator,
            out BigInteger remainder);
        BigInteger twiceRemainder = remainder << 1;
        if (twiceRemainder > denominator
            || (twiceRemainder == denominator && !quotient.IsEven))
        {
            quotient++;
        }
        return (long)(numerator.Sign < 0 ? -quotient : quotient);
    }
}
