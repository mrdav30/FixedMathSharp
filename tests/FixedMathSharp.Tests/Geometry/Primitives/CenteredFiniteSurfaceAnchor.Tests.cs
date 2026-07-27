//=======================================================================
// CenteredFiniteSurfaceAnchor.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredFiniteSurfaceAnchorTests
{
    [Fact]
    public void CylinderSurfaceAnchor_RetainsTheAuthoritativeRigidFrame()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.Pi / (Fixed64)5);
        Vector3d center = new(
            (Fixed64)7,
            (Fixed64)(-3),
            (Fixed64)2);
        Assert.True(rotation.TryRotate(
            new Vector3d(
                Fixed64.FromFraction(1, 2),
                (Fixed64)3,
                (Fixed64)2),
            out Vector3d pointOffset));
        Assert.True(Vector3d.TryAdd(center, pointOffset, out Vector3d point));

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                point,
                center,
                rotation,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                Vector3d.Right,
                out FixedPointAnchor anchor,
                out Vector3d normal,
                out Fixed64 signedDistance));

        Assert.Equal(center, anchor.Origin);
        Assert.Equal(rotation, anchor.Rotation);
        Assert.Equal((Fixed64)2, anchor.LocalPoint.Y);
        Assert.True(normal.IsNormalized());
        Assert.True(signedDistance > Fixed64.Zero);
    }

    [Fact]
    public void CylinderSurfaceAnchor_PreservesOddRawCapAndRadialProducts()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.HalfPi);
        Vector3d point = new(
            (Fixed64)(-2),
            Fixed64.Zero,
            rawUnit);

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                point,
                Vector3d.Zero,
                rotation,
                Vector3d.Up,
                rawUnit,
                rawUnit,
                Vector3d.Right,
                out FixedPointAnchor anchor,
                out _,
                out _));

        Assert.False(anchor.ExactLocalTerm.IsZero);
        Assert.Equal(rotation, anchor.Rotation);
        Assert.True(anchor.TryGetPoint(out _));
    }

    [Fact]
    public void CylinderSurfaceAnchor_ProjectsAgainstTheExactQuaternionBasis()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.Pi / (Fixed64)7);
        Vector3d center = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d point = Vector3d.Zero;

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                point,
                center,
                rotation,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.Zero,
                Vector3d.Right,
                out FixedPointAnchor anchor,
                out _,
                out _));

        Assert.Equal(
            CenteredCapsuleAnchor3dTests.GetExactInverseRotatedYRaw(
                point,
                center,
                rotation),
            anchor.LocalPoint.Y.m_rawValue);
    }

    [Fact]
    public void ConeSurfaceAnchor_RetainsSideFeatureInTheAuthoritativeFrame()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Right,
            Fixed64.Pi / (Fixed64)7);
        Vector3d center = new(
            (Fixed64)(-5),
            (Fixed64)4,
            (Fixed64)3);
        Assert.True(rotation.TryRotate(
            new Vector3d(
                (Fixed64)3,
                Fixed64.Zero,
                Fixed64.Zero),
            out Vector3d pointOffset));
        Assert.True(Vector3d.TryAdd(center, pointOffset, out Vector3d point));

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                point,
                center,
                rotation,
                Vector3d.Up,
                (Fixed64)4,
                (Fixed64)2,
                Vector3d.Right,
                out FixedPointAnchor anchor,
                out Vector3d normal,
                out Fixed64 signedDistance));

        Assert.Equal(center, anchor.Origin);
        Assert.Equal(rotation, anchor.Rotation);
        Assert.True(normal.IsNormalized());
        Assert.True(signedDistance > Fixed64.Zero);
        Assert.True(anchor.TryGetPoint(out _));
    }

    [Fact]
    public void ConeSurfaceAnchor_ClassifiesTheExactRotatedBasePlane()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.Pi / (Fixed64)7);
        Vector3d center = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                Vector3d.Zero,
                center,
                rotation,
                Vector3d.Up,
                Fixed64.FromRaw(8_003_742_285_754_490_109L),
                Fixed64.MaxValue,
                Vector3d.Right,
                out _,
                out _,
                out Fixed64 signedDistance));

        Assert.True(signedDistance < Fixed64.Zero);
    }

    [Fact]
    public void SurfaceAnchors_ValidateRigidLocalFrameContract()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                Vector3d.Zero,
                Vector3d.Zero,
                default,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Right,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                Vector3d.Zero,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.One,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Right,
                out _,
                out _,
                out _));
    }
}
