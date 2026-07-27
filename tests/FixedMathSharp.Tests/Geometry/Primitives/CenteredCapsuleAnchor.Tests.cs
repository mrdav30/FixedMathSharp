//=======================================================================
// CenteredCapsuleAnchor.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class CenteredCapsuleAnchorTests
{
    [Fact]
    public void SupportAnchor_RejectsOnlyTheUnrepresentableFinalCoordinate()
    {
        FixedPointAnchor2d anchor =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.FromRaw(1L),
                Fixed64.Zero,
                Vector2d.Right);
        FixedPointAnchor2d origin = new(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero);

        Assert.False(anchor.TryGetPoint(out _));
        Assert.False(anchor.TryGetOffsetFrom(origin, out _));
        Assert.False(anchor.TryGetLocalPointIn(
            Vector2d.Zero,
            Fixed64.Zero,
            out _));
    }

    [Fact]
    public void SupportAnchor_RetainsOddRawFullLengthUntilWorldMaterialization()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        Vector2d center = new(rawUnit, Fixed64.Zero);

        FixedPointAnchor2d positive =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                center,
                Fixed64.Zero,
                Vector2d.Right,
                rawUnit,
                Fixed64.Zero,
                Vector2d.Right);
        FixedPointAnchor2d negative =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                center,
                Fixed64.Zero,
                Vector2d.Right,
                rawUnit,
                Fixed64.Zero,
                Vector2d.Left);

        Assert.True(positive.TryGetPoint(out Vector2d positivePoint));
        Assert.True(negative.TryGetPoint(out Vector2d negativePoint));
        Assert.Equal(Fixed64.FromRaw(2L), positivePoint.X);
        Assert.Equal(Fixed64.Zero, negativePoint.X);
        var centerAnchor = new FixedPointAnchor2d(
            center,
            Fixed64.Zero,
            Vector2d.Zero);
        Assert.True(positive.TryGetOffsetFrom(
            centerAnchor,
            out Vector2d positiveOffset));
        Assert.Equal(Fixed64.Zero, positiveOffset.X);
        Assert.True(negative.TryGetOffsetFrom(
            centerAnchor,
            out Vector2d negativeOffset));
        Assert.Equal(Fixed64.Zero, negativeOffset.X);
        Assert.True(positive.TryGetLocalPointIn(
            Vector2d.Zero,
            Fixed64.Zero,
            out Vector2d positiveLocal));
        Assert.Equal(Fixed64.FromRaw(2L), positiveLocal.X);
        Assert.NotEqual(positive, negative);
        Assert.NotEqual(0, positive.CompareLocalFeature(negative));
        Assert.NotEqual(
            positive.GetLocalFeatureHash64(),
            negative.GetLocalFeatureHash64());
        Assert.InRange(
            positive.ExactLocalTerm.X,
            -FixedPointAnchorTerm3d.MaximumResidualMagnitude,
            FixedPointAnchorTerm3d.MaximumResidualMagnitude);
        Assert.InRange(
            negative.ExactLocalTerm.X,
            -FixedPointAnchorTerm3d.MaximumResidualMagnitude,
            FixedPointAnchorTerm3d.MaximumResidualMagnitude);
    }

    [Fact]
    public void SupportAnchor_RetainsRadialProductUntilRigidRotation()
    {
        FixedPointAnchor2d exact =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.PiOver4,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.FromRaw(2L),
                Vector2d.One);
        var rounded = new FixedPointAnchor2d(
            exact.Origin,
            exact.Rotation,
            exact.LocalPoint,
            exact.LocalDisplacement);

        Assert.True(exact.TryGetPoint(out Vector2d exactPoint));
        Assert.True(rounded.TryGetPoint(out Vector2d roundedPoint));
        Assert.Equal(Fixed64.FromRaw(2L), exactPoint.Y);
        Assert.Equal(Fixed64.FromRaw(1L), roundedPoint.Y);
    }

    [Fact]
    public void SupportAnchor_PreservesAxialAndRadialTermsBeyondTheScalarDomain()
    {
        FixedPointAnchor2d anchor =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Vector2d.Right);

        Assert.Equal(
            Fixed64.MaxValue / Fixed64.Two,
            anchor.LocalPoint.X);
        Assert.Equal(Fixed64.MaxValue, anchor.LocalDisplacement.X);
        Assert.False(anchor.TryGetPoint(out _));
    }

    [Fact]
    public void SupportAnchor_NormalizesEveryRepresentableNonzeroDirection()
    {
        FixedPointAnchor2d tiny =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.Two,
                Fixed64.One,
                new Vector2d(Fixed64.FromRaw(1L), Fixed64.Zero));
        FixedPointAnchor2d zero =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.Two,
                Fixed64.One,
                Vector2d.Zero);

        Assert.Equal(Vector2d.Right, tiny.LocalPoint);
        Assert.Equal(Vector2d.Right, tiny.LocalDisplacement);
        Assert.Equal(Vector2d.Zero, zero.LocalPoint);
        Assert.Equal(Vector2d.Zero, zero.LocalDisplacement);
    }

    [Fact]
    public void SurfaceAnchors_PreserveCompositeFeaturesInBothDimensions()
    {
        FixedPointAnchor2d planar =
            FixedSegment2d.GetSurfaceAnchorOnCenteredCapsule(
                new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Vector2d.Right);
        FixedPointAnchor spatial =
            FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Right,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Vector3d.Right);

        Assert.Equal(
            Fixed64.MaxValue / Fixed64.Two,
            planar.LocalPoint.X);
        Assert.Equal(Fixed64.MaxValue, planar.LocalDisplacement.X);
        Assert.False(planar.TryGetPoint(out _));
        Assert.Equal(
            Fixed64.MaxValue / Fixed64.Two,
            spatial.LocalPoint.X);
        Assert.Equal(Fixed64.MaxValue, spatial.LocalDisplacement.X);
        Assert.False(spatial.TryGetPoint(out _));
    }

    [Fact]
    public void PlanarAnchors_PreserveFeatureIdentityAcrossRigidPoses()
    {
        FixedPointAnchor2d firstSupport =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.Two,
                Fixed64.One,
                Vector2d.Forward);
        FixedPointAnchor2d rotatedSupport =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                new Vector2d(7, -11),
                Fixed64.HalfPi,
                Vector2d.Right,
                Fixed64.Two,
                Fixed64.One,
                Vector2d.Forward);
        FixedPointAnchor2d firstSurface =
            FixedSegment2d.GetSurfaceAnchorOnCenteredCapsule(
                Vector2d.Right,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.Two,
                Fixed64.One,
                Vector2d.Forward);
        Vector2d rotatedCenter = new(7, -11);
        Vector2d rotatedPoint =
            rotatedCenter + Vector2d.Rotate(
                Vector2d.Right,
                Fixed64.HalfPi);
        FixedPointAnchor2d rotatedSurface =
            FixedSegment2d.GetSurfaceAnchorOnCenteredCapsule(
                rotatedPoint,
                rotatedCenter,
                Fixed64.HalfPi,
                Vector2d.Right,
                Fixed64.Two,
                Fixed64.One,
                Vector2d.Forward);

        Assert.Equal(firstSupport.LocalPoint, rotatedSupport.LocalPoint);
        Assert.Equal(
            firstSupport.LocalDisplacement,
            rotatedSupport.LocalDisplacement);
        Assert.Equal(firstSurface.LocalPoint, rotatedSurface.LocalPoint);
        Assert.Equal(
            firstSurface.LocalDisplacement,
            rotatedSurface.LocalDisplacement);
    }

    [Fact]
    public void AnchorFactories_ValidateGeometryContracts()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector2d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                -Fixed64.One,
                Vector2d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment2d.TryGetSurfaceOffsetOnCenteredCapsule(
                Vector2d.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                -Fixed64.One,
                Vector2d.Right,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetSurfaceOffsetOnCenteredCapsule(
                Vector2d.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.One,
                Vector2d.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment2d.GetSurfaceAnchorOnCenteredCapsule(
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                -Fixed64.One,
                Vector2d.Right));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.GetSurfaceAnchorOnCenteredCapsule(
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.One,
                Vector2d.One));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                Vector3d.Zero,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Right + Vector3d.Up));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                Vector3d.Zero,
                Vector3d.Zero,
                FixedQuaternion.Zero,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Right));
    }

    [Fact]
    public void AnchorFactories_WarmedPathsDoNotAllocate()
    {
        _ = FixedSegment2d.GetCenteredCapsuleSupportAnchor(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Forward);
        _ = FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
            Vector3d.Zero,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right);
        long before = GC.GetAllocatedBytesForCurrentThread();

        for (int iteration = 0; iteration < 64; iteration++)
        {
            _ = FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.Two,
                Fixed64.One,
                Vector2d.Forward);
            _ = FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                Vector3d.Zero,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                Vector3d.Right);
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }
}
