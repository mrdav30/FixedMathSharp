//=======================================================================
// WidePlanarProjection.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WidePlanarProjectionTests
{
    [Fact]
    public void QuarticRootCounter_CountsDistinctRealRoots()
    {
        Signed832 one = Signed832.ExtendValue(Signed192.One);
        Signed832 negativeOne =
            WideArithmetic.SubtractSigned832(default, one);

        Assert.Equal(0, WideFiniteAxisIntersection.CountProjectionDomainQuarticRoots(
            one, default, default, default, one));
        Assert.Equal(2, WideFiniteAxisIntersection.CountProjectionDomainQuarticRoots(
            negativeOne, default, default, default, one));
        Assert.Equal(0, WideFiniteAxisIntersection.CountProjectionDomainQuarticRoots(
            one, default, WideArithmetic.AddSigned832(one, one), default, one));
        Assert.Equal(2, WideFiniteAxisIntersection.CountProjectionDomainQuarticRoots(
            one, default, WideArithmetic.AddSigned832(negativeOne, negativeOne), default, one));
        Assert.Equal(2, WideFiniteAxisIntersection.CountProjectionDomainQuarticRoots(
            one, default, default, default, negativeOne));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            WideFiniteAxisIntersection.CountProjectionDomainQuarticRoots(
                one, default, default, default, default));
    }

    [Fact]
    public void SphereRelation_RetainsScalarFaceContainmentAndSeparation()
    {
        Vector3d center = new(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.Zero);

        Assert.True(WidePlanarProjection.TryGetSphereRelation(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Fixed64.Zero,
            center,
            (Fixed64)2,
            out PlanarProjectionRelation contained));
        Assert.Equal(Fixed64.Zero, contained.Distance);
        Assert.Equal(Vector2d.Zero, contained.Offset);

        Assert.True(WidePlanarProjection.TryGetSphereRelation(
            new Vector2d(Fixed64.MaxValue - (Fixed64)3, Fixed64.Zero),
            Fixed64.One,
            center,
            (Fixed64)2,
            out PlanarProjectionRelation separated));
        Assert.Equal(Fixed64.One, separated.Distance);
        Assert.Equal(Vector2d.Right, separated.Offset);
    }

    [Fact]
    public void CapsuleRelation_UsesTheCompleteProjectedAxis()
    {
        Vector3d center = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(WidePlanarProjection.TryGetCenteredCapsuleRelation(
            new Vector2d(Fixed64.MaxValue - (Fixed64)4, Fixed64.Zero),
            Fixed64.One,
            center,
            FixedQuaternion.Identity,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation relation));

        Assert.Equal(Fixed64.One, relation.Distance);
        Assert.Equal(Vector2d.Right, relation.Offset);
    }

    [Fact]
    public void CapsuleRelation_ClassifiesContainedRoundedAndRejectedGaps()
    {
        Assert.True(WidePlanarProjection.TryGetCenteredCapsuleRelation(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation contained));
        Assert.Equal(Fixed64.Zero, contained.Distance);

        Assert.True(WidePlanarProjection.TryGetCenteredCapsuleRelation(
            new Vector2d((Fixed64)2, (Fixed64)2),
            Fixed64.Two,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation rounded));
        Assert.True(
            rounded.Distance >= Fixed64.FromFraction(9, 5));
        Assert.True(
            rounded.Distance <= Fixed64.FromFraction(19, 10));

        Assert.False(WidePlanarProjection.TryGetCenteredCapsuleRelation(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CylinderRelation_HandlesVerticalHorizontalAndRotatedAxes()
    {
        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.One,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            (Fixed64)4,
            (Fixed64)2,
            out PlanarProjectionRelation vertical));
        Assert.Equal(Fixed64.One, vertical.Distance);
        Assert.Equal(Vector2d.Left, vertical.Offset);

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d((Fixed64)3, (Fixed64)2),
            (Fixed64)2,
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                -Fixed64.HalfPi),
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation horizontal));
        Assert.Equal(FixedMath.Sqrt((Fixed64)2), horizontal.Distance);
        Assert.Equal(new Vector2d(-Fixed64.One, -Fixed64.One), horizontal.Offset);

        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.PiOver4);
        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d(Fixed64.Zero, (Fixed64)2),
            Fixed64.One,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation rotated));
        Assert.Equal(Fixed64.One, rotated.Distance);
        Assert.Equal(new Vector2d(Fixed64.Zero, -Fixed64.One), rotated.Offset);

        Vector3d axis = rotation * Vector3d.Up;
        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d(
                axis.X * Fixed64.Two,
                axis.Z * Fixed64.Two),
            Fixed64.Zero,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation endCap));
        Assert.Equal(Fixed64.Zero, endCap.Distance);

        Assert.False(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d(Fixed64.Zero, (Fixed64)4),
            Fixed64.Half,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CylinderRelation_RetainsTheClosestEllipseDirection()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            -Fixed64.PiOver4);
        Vector2d query = new((Fixed64)3, (Fixed64)2);

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            query,
            (Fixed64)3,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation relation));
        Assert.True(relation.Distance > Fixed64.Zero);
        Assert.NotEqual(Vector2d.Zero, relation.Offset);

        Vector3d axis = rotation * Vector3d.Up;
        var capCenter = new Vector2d(
            axis.X * Fixed64.Two,
            axis.Z * Fixed64.Two);
        Vector2d radial = (capCenter - query).Normalized;
        Assert.True(
            FixedMath.Abs(
                relation.Offset.Normalized.X * radial.Y
                - relation.Offset.Normalized.Y * radial.X)
            > Fixed64.FromFraction(1, 100));
    }

    [Fact]
    public void CylinderRelation_ClassifiesExactEllipticalCapBoundaries()
    {
        var rotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Two);

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Zero,
            Vector3d.Zero,
            rotation,
            (Fixed64)10,
            (Fixed64)5,
            out PlanarProjectionRelation contained));
        Assert.Equal(Fixed64.Zero, contained.Distance);

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.One,
            Vector3d.Zero,
            rotation,
            (Fixed64)10,
            (Fixed64)5,
            out PlanarProjectionRelation containingCircle));
        Assert.Equal(Fixed64.Zero, containingCircle.Distance);

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d((Fixed64)9, Fixed64.Zero),
            Fixed64.Two,
            Vector3d.Zero,
            rotation,
            (Fixed64)10,
            (Fixed64)5,
            out PlanarProjectionRelation tangent));
        Assert.Equal(Fixed64.Two, tangent.Distance);
        Assert.Equal(new Vector2d((Fixed64)(-2), Fixed64.Zero), tangent.Offset);

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d((Fixed64)(-9), Fixed64.Zero),
            Fixed64.Two,
            Vector3d.Zero,
            rotation,
            (Fixed64)10,
            (Fixed64)5,
            out PlanarProjectionRelation mirroredTangent));
        Assert.Equal(Fixed64.Two, mirroredTangent.Distance);
        Assert.Equal(new Vector2d(Fixed64.Two, Fixed64.Zero), mirroredTangent.Offset);

        Assert.False(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d((Fixed64)10, Fixed64.Zero),
            Fixed64.Two,
            Vector3d.Zero,
            rotation,
            (Fixed64)10,
            (Fixed64)5,
            out _));
    }

    [Fact]
    public void CylinderRelation_RoundsPositiveSubrawEllipticalCapGaps()
    {
        Fixed64 Raw(long value) => Fixed64.FromRaw(value);
        var rotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Raw(1L),
            Raw(2L));

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d(Raw(2L), Fixed64.Zero),
            Raw(4L),
            Vector3d.Zero,
            rotation,
            Raw(1L),
            Raw(2L),
            out PlanarProjectionRelation relation));
        Assert.Equal(Fixed64.Zero, relation.Distance);
        Assert.Equal(Vector2d.Zero, relation.Offset);

    }

    [Fact]
    public void CylinderRelation_RetainsASubscaleGapDirection()
    {
        var rotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.FromRaw(1L),
            Fixed64.FromRaw(3L));
        Fixed64 extent = Fixed64.FromRaw(
            3_000_000_000_000_000_000L);
        Fixed64 boundary = Fixed64.FromRaw(
            3_300_000_000_000_000_000L);

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d(
                Fixed64.FromRaw(boundary.m_rawValue + 1L),
                Fixed64.Zero),
            Fixed64.MinIncrement,
            Vector3d.Zero,
            rotation,
            extent,
            extent,
            out PlanarProjectionRelation relation));
        Assert.Equal(Fixed64.MinIncrement, relation.Distance);
        Assert.Equal(
            new Vector2d(-Fixed64.MinIncrement, Fixed64.Zero),
            relation.Offset);
    }

    [Fact]
    public void CylinderRelation_RetainsAnOffPrincipalSubscaleGap()
    {
        Fixed64 Raw(long value) => Fixed64.FromRaw(value);
        var rotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Raw(1L),
            Raw(3L));

        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d(
                Raw(1_440_000_000_000_000_068L),
                Raw(2_400_000_000_000_000_073L)),
            Raw(1_000L),
            Vector3d.Zero,
            rotation,
            Raw(1L),
            Raw(3_000_000_000_000_000_000L),
            out PlanarProjectionRelation relation));
        Assert.True(relation.Distance >= Raw(90L));
        Assert.True(relation.Distance <= Raw(110L));
        Assert.True(relation.Offset.X < Fixed64.Zero);
        Assert.True(relation.Offset.Y < Fixed64.Zero);
    }

    [Fact]
    public void ConeRelation_HandlesVerticalAndHorizontalProjections()
    {
        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.One,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            (Fixed64)4,
            (Fixed64)2,
            out PlanarProjectionRelation vertical));
        Assert.Equal(Fixed64.One, vertical.Distance);
        Assert.Equal(Vector2d.Left, vertical.Offset);

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d((Fixed64)(-2), (Fixed64)2),
            Fixed64.One,
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                -Fixed64.HalfPi),
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation horizontal));
        Assert.Equal(Fixed64.One, horizontal.Distance);
        Assert.Equal(new Vector2d(Fixed64.Zero, -Fixed64.One), horizontal.Offset);
    }

    [Fact]
    public void ConeRelation_ClassifiesTheRotatedProjectedSideInterior()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            -Fixed64.PiOver4);

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Fixed64.Zero, Fixed64.Half),
            Fixed64.Zero,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation relation));
        Assert.Equal(Fixed64.Zero, relation.Distance);
        Assert.Equal(Vector2d.Zero, relation.Offset);

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Fixed64.Zero, Fixed64.Half),
            Fixed64.One,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation containingCircle));
        Assert.Equal(Fixed64.Zero, containingCircle.Distance);
    }

    [Fact]
    public void ConeRelation_ClassifiesRotatedProjectedTangentSeparation()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            -Fixed64.PiOver4);

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Fixed64.Zero, Fixed64.One),
            Fixed64.Half,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation relation));
        Assert.True(
            relation.Distance >= Fixed64.FromFraction(2, 5));
        Assert.True(relation.Distance <= Fixed64.Half);
        Assert.True(relation.Offset.Y < Fixed64.Zero);

        Assert.False(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Fixed64.Zero, Fixed64.One),
            Fixed64.FromFraction(2, 5),
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void ConeRelation_RetainsAnOffPrincipalSubscaleGap()
    {
        Fixed64 Raw(long value) => Fixed64.FromRaw(value);
        var rotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Raw(1L),
            Raw(3L));

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(
                Raw(1_440_000_000_000_000_068L),
                Raw(2_400_000_000_000_000_073L)),
            Raw(1_000L),
            Vector3d.Zero,
            rotation,
            Raw(1L),
            Raw(3_000_000_000_000_000_000L),
            out PlanarProjectionRelation relation));
        Assert.True(relation.Distance >= Raw(90L));
        Assert.True(relation.Distance <= Raw(110L));
        Assert.True(relation.Offset.X < Fixed64.Zero);
        Assert.True(relation.Offset.Y < Fixed64.Zero);
    }

    [Fact]
    public void ConeRelation_SelectsBaseApexAndBothTangentSides()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            -Fixed64.PiOver4);

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            Fixed64.Two,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation baseRelation));
        Assert.True(baseRelation.Distance > Fixed64.Zero);
        Assert.True(baseRelation.Offset.X > Fixed64.Zero);

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.One,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation apexRelation));
        Assert.True(apexRelation.Distance > Fixed64.Zero);
        Assert.True(apexRelation.Offset.X < Fixed64.Zero);

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            Fixed64.Half,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation lowerTangent));
        Assert.True(lowerTangent.Offset.Y > Fixed64.Zero);

        Assert.False(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Fixed64.Zero, (Fixed64)3),
            Fixed64.Half,
            Vector3d.Zero,
            rotation,
            (Fixed64)4,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void ConeRelation_ClassifiesAnOversizedCircleAgainstTheTangentHull()
    {
        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Fixed64.Zero, Fixed64.One),
            (Fixed64)10,
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                -Fixed64.PiOver4),
            (Fixed64)4,
            Fixed64.One,
            out PlanarProjectionRelation relation));
        Assert.True(relation.Distance > Fixed64.Zero);
        Assert.True(relation.Distance < Fixed64.One);
        Assert.True(relation.Offset.Y < Fixed64.Zero);
    }

    [Fact]
    public void OrientedBoxRelation_DoesNotMaterializeScalarFaceCorners()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MinValue,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));

        Assert.True(WidePlanarProjection.TryGetOrientedBoxRelation(
            new Vector2d(Fixed64.MaxValue - (Fixed64)3, Fixed64.Zero),
            Fixed64.One,
            box,
            out PlanarProjectionRelation relation));

        Assert.Equal(Fixed64.One, relation.Distance);
        Assert.Equal(Vector2d.Right, relation.Offset);
    }

    [Fact]
    public void OrientedBoxRelation_HandlesRotatedHullsAndRejection()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.PiOver4),
            new Vector3d(
                Fixed64.One,
                Fixed64.Two,
                Fixed64.One));
        Assert.True(WidePlanarProjection.TryGetOrientedBoxRelation(
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.One,
            box,
            out PlanarProjectionRelation relation));
        Assert.True(relation.Distance > Fixed64.Zero);

        Assert.True(WidePlanarProjection.TryGetOrientedBoxRelation(
            Vector2d.Zero,
            Fixed64.Zero,
            box,
            out PlanarProjectionRelation contained));
        Assert.Equal(Fixed64.Zero, contained.Distance);

        var reversed = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi),
            new Vector3d(
                Fixed64.One,
                Fixed64.Two,
                Fixed64.One));
        Assert.True(WidePlanarProjection.TryGetOrientedBoxRelation(
            Vector2d.Zero,
            Fixed64.Zero,
            reversed,
            out _));

        Assert.False(WidePlanarProjection.TryGetOrientedBoxRelation(
            new Vector2d((Fixed64)3, (Fixed64)3),
            Fixed64.One,
            box,
            out _));
    }

    [Fact]
    public void TriangleRelation_UsesProjectedInteriorEdgesAndPoints()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.Zero);
        var area = new FixedTriangle(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)(-1)),
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)(-1)),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2));

        Assert.True(WidePlanarProjection.TryGetTriangleRelation(
            new Vector2d(Fixed64.MaxValue - (Fixed64)3, (Fixed64)(-1)),
            Fixed64.One,
            area,
            origin,
            FixedQuaternion.Identity,
            out PlanarProjectionRelation edge));
        Assert.Equal(Fixed64.One, edge.Distance);
        Assert.Equal(Vector2d.Right, edge.Offset);

        var projectedPoint = new FixedTriangle(
            new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        Assert.True(WidePlanarProjection.TryGetTriangleRelation(
            new Vector2d(Fixed64.One, Fixed64.Zero),
            Fixed64.One,
            projectedPoint,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out PlanarProjectionRelation point));
        Assert.Equal(Fixed64.One, point.Distance);
        Assert.Equal(Vector2d.Left, point.Offset);

        Assert.True(WidePlanarProjection.TryGetTriangleRelation(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Fixed64.Zero,
            area,
            origin,
            FixedQuaternion.Identity,
            out PlanarProjectionRelation contained));
        Assert.Equal(Fixed64.Zero, contained.Distance);
    }

    [Fact]
    public void ProjectionRelations_RejectSeparatedCircles()
    {
        Assert.False(WidePlanarProjection.TryGetSphereRelation(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.One,
            out _));

        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Forward);
        Assert.False(WidePlanarProjection.TryGetTriangleRelation(
            new Vector2d((Fixed64)4, (Fixed64)4),
            Fixed64.One,
            triangle,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out _));
    }

    [Fact]
    public void ProjectionRelations_RoundExactHalfRawGapsToEven()
    {
        Fixed64 Raw(long value) => Fixed64.FromRaw(value);

        var capsuleRotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Raw(1L),
            Raw(1L));
        Assert.True(WidePlanarProjection.TryGetCenteredCapsuleRelation(
            new Vector2d(Raw(2L), Fixed64.Zero),
            Raw(2L),
            Vector3d.Zero,
            capsuleRotation,
            Vector3d.Up,
            Raw(1L),
            Raw(1L),
            out PlanarProjectionRelation capsuleEven));
        Assert.Equal(Fixed64.Zero, capsuleEven.Distance);
        Assert.True(WidePlanarProjection.TryGetCenteredCapsuleRelation(
            new Vector2d(Raw(3L), Fixed64.Zero),
            Raw(2L),
            Vector3d.Zero,
            capsuleRotation,
            Vector3d.Up,
            Raw(1L),
            Raw(1L),
            out PlanarProjectionRelation capsuleOdd));
        Assert.Equal(Raw(2L), capsuleOdd.Distance);

        var finiteAxisRotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Raw(1L),
            Raw(3L));
        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d(Raw(6L), Fixed64.Zero),
            Raw(2L),
            Vector3d.Zero,
            finiteAxisRotation,
            Raw(5L),
            Raw(5L),
            out PlanarProjectionRelation cylinderEven));
        Assert.Equal(Fixed64.Zero, cylinderEven.Distance);
        Assert.True(WidePlanarProjection.TryGetCenteredCylinderRelation(
            new Vector2d(Raw(7L), Fixed64.Zero),
            Raw(2L),
            Vector3d.Zero,
            finiteAxisRotation,
            Raw(5L),
            Raw(5L),
            out PlanarProjectionRelation cylinderOdd));
        Assert.Equal(Raw(2L), cylinderOdd.Distance);

        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Raw(6L), Fixed64.Zero),
            Raw(2L),
            Vector3d.Zero,
            finiteAxisRotation,
            Raw(5L),
            Raw(5L),
            out PlanarProjectionRelation coneEven));
        Assert.Equal(Fixed64.Zero, coneEven.Distance);
        Assert.True(WidePlanarProjection.TryGetCenteredConeRelation(
            new Vector2d(Raw(7L), Fixed64.Zero),
            Raw(2L),
            Vector3d.Zero,
            finiteAxisRotation,
            Raw(5L),
            Raw(5L),
            out PlanarProjectionRelation coneOdd));
        Assert.Equal(Raw(2L), coneOdd.Distance);

    }
}
