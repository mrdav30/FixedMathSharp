//=======================================================================
// FixedBoundBox.RelativeRotation.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedBoundBoxRelativeRotationTests
{
    [Fact]
    public void RelativeBounds_IdentityFramesPreserveNormalizedEndpoints()
    {
        FixedBoundBox bounds =
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                new Vector3d(7, -5, 3),
                FixedQuaternion.Identity,
                new Vector3d(4, 6, 8),
                new Vector3d(-2, -3, -5),
                new Vector3d(7, -5, 3),
                FixedQuaternion.Identity);

        Assert.Equal(new Vector3d(-2, -3, -5), bounds.Min);
        Assert.Equal(new Vector3d(4, 6, 8), bounds.Max);
    }

    [Fact]
    public void RelativeBounds_CancelExtremeOriginsBeforeFinalNarrowing()
    {
        Vector3d scalarFace = new(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.MaxValue);

        FixedBoundBox bounds =
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                scalarFace,
                FixedQuaternion.Identity,
                -Vector3d.One,
                Vector3d.One,
                scalarFace,
                FixedQuaternion.Identity);

        Assert.Equal(-Vector3d.One, bounds.Min);
        Assert.Equal(Vector3d.One, bounds.Max);
    }

    [Fact]
    public void RelativeBounds_IdentityFramesRebaseWorldBoundsIntoTargetOrigin()
    {
        FixedBoundBox bounds =
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(
                    Fixed64.FromFraction(-7, 2),
                    Fixed64.FromFraction(-7, 2),
                    (Fixed64)(-4)),
                new Vector3d(
                    Fixed64.FromFraction(9, 2),
                    Fixed64.FromFraction(9, 2),
                    (Fixed64)4),
                new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero),
                FixedQuaternion.Identity);

        Assert.Equal(new Vector3d(-4, -4, -4), bounds.Min);
        Assert.Equal(new Vector3d(4, 4, 4), bounds.Max);
    }

    [Fact]
    public void RelativeBounds_ClipOnlyFinalEndpoints()
    {
        FixedBoundBox bounds =
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.MinValue,
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                FixedQuaternion.Identity);

        Assert.Equal(Fixed64.MaxValue, bounds.Min.X);
        Assert.Equal(Fixed64.MaxValue, bounds.Max.X);
        Assert.Equal(Fixed64.MinValue, bounds.Min.Y);
        Assert.Equal(Fixed64.MinValue + Fixed64.One, bounds.Max.Y);
        Assert.Equal(Fixed64.Zero, bounds.Min.Z);
        Assert.Equal(Fixed64.One, bounds.Max.Z);
    }

    [Fact]
    public void RelativeBounds_ConservativelyContainEveryRotatedCorner()
    {
        Vector3d sourceOrigin = new(11, -7, 5);
        FixedQuaternion sourceRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)(-31),
                (Fixed64)9);
        Vector3d sourceMin = new(-2, -3, -5);
        Vector3d sourceMax = new(4, 6, 8);
        Vector3d targetOrigin = new(-13, 3, 17);
        FixedQuaternion targetRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)(-7),
                (Fixed64)23,
                (Fixed64)41);
        FixedBoundBox bounds =
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                sourceOrigin,
                sourceRotation,
                sourceMin,
                sourceMax,
                targetOrigin,
                targetRotation);

        for (int index = 0; index < FixedBoundBox.CornerCount; index++)
        {
            Vector3d sourceLocal = FixedBoundBox
                .FromMinMax(sourceMin, sourceMax)
                .GetCorner(index);
            Assert.True(new FixedPointAnchor(
                sourceOrigin,
                sourceRotation,
                sourceLocal).TryGetLocalPointIn(
                    targetOrigin,
                    targetRotation,
                    out Vector3d targetLocal));

            Assert.True(bounds.Contains(targetLocal));
        }
    }

    [Fact]
    public void RelativeBounds_ValidateBothRigidFrames()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                Vector3d.Zero,
                default,
                -Vector3d.One,
                Vector3d.One,
                Vector3d.Zero,
                FixedQuaternion.Identity));
        Assert.Throws<ArgumentException>(() =>
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                -Vector3d.One,
                Vector3d.One,
                Vector3d.Zero,
                default));
    }

    [Fact]
    public void RelativeBounds_WarmedPathDoesNotAllocate()
    {
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)29,
                (Fixed64)(-11));
        _ = FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
            new Vector3d(3, -5, 7),
            rotation,
            -Vector3d.One,
            Vector3d.One,
            new Vector3d(-7, 11, 13),
            rotation);
        long before = GC.GetAllocatedBytesForCurrentThread();

        for (int iteration = 0; iteration < 64; iteration++)
        {
            _ = FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                new Vector3d(3, -5, 7),
                rotation,
                -Vector3d.One,
                Vector3d.One,
                new Vector3d(-7, 11, 13),
                rotation);
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }
}
