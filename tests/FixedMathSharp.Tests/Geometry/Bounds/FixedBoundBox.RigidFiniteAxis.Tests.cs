//=======================================================================
// FixedBoundBox.RigidFiniteAxis.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedBoundBoxRigidFiniteAxisTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RigidFiniteAxisBounds_ClipOddLengthsAtMirroredScalarFaces(
        bool maximumFace)
    {
        Fixed64 face = maximumFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Fixed64 inwardTwo = Fixed64.FromRaw(maximumFace ? -2L : 2L);
        Vector3d center = new(face, Fixed64.Zero, Fixed64.Zero);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            maximumFace ? -Fixed64.HalfPi : Fixed64.HalfPi);
        Vector3d expectedMin = maximumFace
            ? new Vector3d(face + inwardTwo, Fixed64.Zero, Fixed64.Zero)
            : center;
        Vector3d expectedMax = maximumFace
            ? center
            : new Vector3d(face + inwardTwo, Fixed64.Zero, Fixed64.Zero);

        FixedBoundBox capsule =
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                center,
                rotation,
                Vector3d.Up,
                Fixed64.FromRaw(3L),
                Fixed64.Zero);
        FixedBoundBox cylinder =
            FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                center,
                rotation,
                Vector3d.Up,
                Fixed64.FromRaw(3L),
                Fixed64.Zero);
        FixedBoundBox cone =
            FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                center,
                rotation,
                Vector3d.Up,
                Fixed64.FromRaw(3L),
                Fixed64.Zero);

        Assert.Equal(expectedMin, capsule.Min);
        Assert.Equal(expectedMax, capsule.Max);
        Assert.Equal(capsule, cylinder);
        Assert.Equal(capsule, cone);
    }

    [Fact]
    public void RigidConeBounds_RoundCombinedBaseRimOnlyOnce()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            -FixedMath.Acos(Fixed64.FromFraction(3, 5)));

        FixedBoundBox bounds =
            FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                Vector3d.Zero,
                rotation,
                Vector3d.Up,
                Fixed64.FromRaw(2L),
                Fixed64.FromRaw(4L));

        Assert.Equal(Fixed64.FromRaw(-4L), bounds.Min.X);
        Assert.Equal(Fixed64.FromRaw(2L), bounds.Max.X);

        FixedBoundBox mirrored =
            FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                Vector3d.Zero,
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Forward,
                    FixedMath.Acos(Fixed64.FromFraction(3, 5))),
                Vector3d.Up,
                Fixed64.FromRaw(2L),
                Fixed64.FromRaw(4L));

        Assert.Equal(-bounds.Max.X, mirrored.Min.X);
        Assert.Equal(-bounds.Min.X, mirrored.Max.X);
    }

    [Fact]
    public void RigidCapsuleBounds_RoundBothFractionalFacesOutward()
    {
        FixedQuaternion rotation = new(
            Fixed64.FromRaw(-2_454_267_026L),
            Fixed64.FromRaw(-2_454_267_026L),
            Fixed64.FromRaw(-2_454_267_026L),
            Fixed64.FromRaw(613_566_757L));
        Fixed64 oneRaw = Fixed64.FromRaw(1L);

        FixedBoundBox bounds =
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                Vector3d.Zero,
                rotation,
                Vector3d.Up,
                oneRaw,
                Fixed64.Zero);

        Assert.Equal(new Vector3d(-oneRaw, -oneRaw, -oneRaw), bounds.Min);
        Assert.Equal(new Vector3d(oneRaw, oneRaw, oneRaw), bounds.Max);
    }

    [Fact]
    public void RigidCylinderBounds_CarryAcrossCombinedAxialAndDiskExtent()
    {
        var rotation = new FixedQuaternion(
            Fixed64.FromRaw(-2_454_267_026L),
            Fixed64.FromRaw(-2_454_267_026L),
            Fixed64.FromRaw(-2_454_267_026L),
            Fixed64.FromRaw(613_566_757L));

        FixedBoundBox bounds =
            FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                Vector3d.Zero,
                rotation,
                Vector3d.Up,
                Fixed64.FromRaw(1L),
                Fixed64.FromRaw(3L));

        Assert.Equal(Fixed64.FromRaw(-3L), bounds.Min.X);
        Assert.Equal(Fixed64.FromRaw(3L), bounds.Max.X);
    }

    [Fact]
    public void RigidFiniteAxisBounds_RejectInvalidAuthoredFramesAndDimensions()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                Vector3d.Zero,
                default,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.Zero));
        Assert.Throws<ArgumentException>(() =>
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                -Fixed64.One,
                Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                -Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.Zero));
    }
}
