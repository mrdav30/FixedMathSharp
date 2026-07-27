//=======================================================================
// CenteredConeAnchor.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredConeAnchorTests
{
    [Fact]
    public void SupportAnchor_PreservesRigidFrameCancellationAtScalarFaces()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.HalfPi / Fixed64.Two);
        Vector3d direction = rotation.Rotate(
            new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero));
        FixedPointAnchor support =
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                Vector3d.Zero,
                rotation,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                direction);

        Assert.False(support.TryGetPoint(out _));
        Assert.True(support.TryGetOffsetFrom(
            new FixedPointAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.Zero),
            out Vector3d relative));
        Assert.True(relative.X > Fixed64.Zero);
        Assert.True(relative.Y > Fixed64.Zero);
    }

    [Fact]
    public void SupportAnchor_UsesBaseForZeroDirectionAndValidatesGeometry()
    {
        FixedPointAnchor support =
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                new Vector3d(3, 4, 5),
                FixedQuaternion.Identity,
                (Fixed64)6,
                (Fixed64)2,
                Vector3d.Zero);

        Assert.Equal(new Vector3d(3, 4, 5), support.Origin);
        Assert.Equal(
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            support.LocalPoint);
        Assert.Equal(new Vector3d(2, 0, 0), support.LocalDisplacement);

        Assert.Throws<ArgumentException>(() =>
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Right));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Right));
    }

    [Fact]
    public void ClosestSurfaceAnchor_PreservesQuantizedBaseRim()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                Vector3d.Right,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                rawUnit,
                rawUnit,
                Vector3d.Right,
                out FixedPointAnchor anchor,
                out _,
                out _));

        Assert.Equal(-Fixed64.One.m_rawValue, anchor.ExactLocalTerm.Y);
        Assert.Equal(rawUnit, anchor.LocalDisplacement.X);
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(new Vector3d(rawUnit, Fixed64.Zero, Fixed64.Zero), point);
    }

    [Theory]
    [MemberData(nameof(CanonicalAxes))]
    public void AxisSupportAnchor_UsesCanonicalFrameAndCorrectFeature(
        Vector3d axis,
        bool apex)
    {
        Vector3d center = new(3, 4, 5);
        Vector3d direction = new(7, -2, 3);

        FixedPointAnchor anchor =
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                center,
                axis,
                (Fixed64)6,
                (Fixed64)2,
                direction);

        Assert.True(anchor.TryGetPoint(out _));
        Assert.True(
            Vector3d.Dot(
                anchor.Rotation.Rotate(Vector3d.Up).Normalized,
                axis)
            >= Fixed64.One - Fixed64.MinIncrement * (Fixed64)4);
        Assert.Equal(apex ? (Fixed64)3 : (Fixed64)(-3), anchor.LocalPoint.Y);
        Assert.Equal(Fixed64.Zero, anchor.LocalDisplacement.Y);
        Assert.Equal(apex, anchor.LocalDisplacement == Vector3d.Zero);
    }

    public static TheoryData<Vector3d, bool> CanonicalAxes => new()
    {
        { Vector3d.Up, false },
        { Vector3d.Down, false },
        { new Vector3d(3, 4, 0).Normalized, true },
    };
}
