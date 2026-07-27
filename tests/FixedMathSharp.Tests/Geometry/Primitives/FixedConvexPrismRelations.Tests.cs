//=======================================================================
// FixedConvexPrismRelations.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismRelationsTests
{
    private static readonly Vector2d[] UnitSquare =
    {
        new(-1, -1),
        new(1, -1),
        new(1, 1),
        new(-1, 1),
    };

    private static readonly Vector3d[] VerticalTriangle =
    {
        new(0, -1, -1),
        new(0, 1, -1),
        new(0, 0, 1),
    };

    private static readonly FixedTriangle VerticalTriangleGeometry = new(
        VerticalTriangle[0],
        VerticalTriangle[1],
        VerticalTriangle[2]);

    [Fact]
    public void FiniteShapeContacts_RemainOriginRelativeAtTheScalarFace()
    {
        Vector3d center = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(
            center,
            Fixed64.One,
            center,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors sphere));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            center,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors capsule));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            center,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors cylinder));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            center,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors cone));

        Assert.NotEqual(default, sphere.Normal);
        Assert.NotEqual(default, capsule.Normal);
        Assert.NotEqual(default, cylinder.Normal);
        Assert.NotEqual(default, cone.Normal);
        Assert.False(sphere.DepthIsClamped);
        Assert.False(capsule.DepthIsClamped);
        Assert.False(cylinder.DepthIsClamped);
        Assert.False(cone.DepthIsClamped);
    }

    [Fact]
    public void CapsuleContact_PreservesRelativeAnchorsWhenWorldPointsAreOutsideTheScalarDomain()
    {
        Vector2d[] fullDomainSquare =
        {
            new(-Fixed64.MaxValue, -Fixed64.MaxValue),
            new(Fixed64.MaxValue, -Fixed64.MaxValue),
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(-Fixed64.MaxValue, Fixed64.MaxValue),
        };
        Vector3d prismOrigin = new(
            -Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            prismOrigin,
            Fixed64.Zero,
            fullDomainSquare,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d relativeOffset));
        Assert.Equal(
            new Vector3d(
                Fixed64.MinValue + Fixed64.One,
                Fixed64.MaxValue,
                Fixed64.MaxValue),
            relativeOffset);
    }

    [Fact]
    public void SphereContact_FromInsideSelectsNearestFaceWithMirroredOrientation()
    {
        AssertInsideSphereContact(
            new Vector3d(
                Fixed64.FromFraction(9, 10),
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(3, 5));
        AssertInsideSphereContact(
            new Vector3d(
                -Fixed64.FromFraction(9, 10),
                Fixed64.Zero,
                Fixed64.Zero),
            -Vector3d.Right,
            Fixed64.FromFraction(3, 5));
        AssertInsideSphereContact(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(9, 20),
                Fixed64.Zero),
            Vector3d.Up,
            Fixed64.FromFraction(11, 20));
        AssertInsideSphereContact(
            new Vector3d(
                Fixed64.Zero,
                -Fixed64.FromFraction(9, 20),
                Fixed64.Zero),
            Vector3d.Down,
            Fixed64.FromFraction(11, 20));
    }

    [Fact]
    public void RotatedFiniteShapeContacts_PreserveTheExactWinnerAtTheScalarFace()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAngles(
            Fixed64.FromFraction(1, 5),
            Fixed64.FromFraction(2, 5),
            Fixed64.FromFraction(1, 3));
        Vector3d localAxis = new Vector3d(1, 2, 3).Normalized;
        Fixed64 prismRotation = Fixed64.FromFraction(3, 10);
        Vector3d scalarFace = new(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.MaxValue);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            rotation,
            localAxis,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            prismRotation,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors baselineCapsule));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            scalarFace,
            rotation,
            localAxis,
            Fixed64.One,
            Fixed64.Half,
            scalarFace,
            prismRotation,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors boundaryCapsule));
        AssertEquivalentContactDecision(
            baselineCapsule,
            boundaryCapsule);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            Vector3d.Zero,
            rotation,
            localAxis,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            prismRotation,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors baselineCylinder));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            scalarFace,
            rotation,
            localAxis,
            Fixed64.One,
            Fixed64.Half,
            scalarFace,
            prismRotation,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors boundaryCylinder));
        AssertEquivalentContactDecision(
            baselineCylinder,
            boundaryCylinder);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(
            Vector3d.Zero,
            rotation,
            localAxis,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            prismRotation,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors baselineCone));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(
            scalarFace,
            rotation,
            localAxis,
            Fixed64.One,
            Fixed64.Half,
            scalarFace,
            prismRotation,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors boundaryCone));
        AssertEquivalentContactDecision(
            baselineCone,
            boundaryCone);
    }

    [Fact]
    public void FiniteShapeContacts_RejectFullDomainSeparationWithoutSaturation()
    {
        Vector3d source = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d prism = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.False(FixedConvexPrismRelations.TryGetSphereContact(
            source,
            Fixed64.One,
            prism,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            source,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            prism,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            source,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            prism,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
        Assert.False(FixedConvexPrismRelations.TryGetCenteredConeContact(
            source,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            prism,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Theory]
    [InlineData(0, 3, 0, false)]
    [InlineData(3, 0, 0, true)]
    [InlineData(0, 0, 3, true)]
    public void FiniteShapeContacts_RejectSeparationOnEveryRigidAxisFamily(
        int x,
        int y,
        int z,
        bool horizontalAxis)
    {
        FixedQuaternion rotation = horizontalAxis
            ? FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.HalfPi)
            : FixedQuaternion.Identity;

        Assert.False(FixedConvexPrismRelations
            .TryGetCenteredCylinderContact(
                new Vector3d(x, y, z),
                rotation,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.FromFraction(1, 4),
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
    }

    [Fact]
    public void HorizontalCapsule_BeyondPrismEdgeRadius_ShouldRejectEndpointAxisSeparation()
    {
        FixedQuaternion horizontal = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.HalfPi);

        Assert.False(FixedConvexPrismRelations
            .TryGetCenteredCapsuleContact(
                new Vector3d(
                    Fixed64.Zero,
                    Fixed64.FromFraction(27, 25),
                    Fixed64.FromFraction(27, 25)),
                horizontal,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.FromFraction(1, 10),
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void SphereCornerContact_UsesUniqueIncidentPrismVertex(
        int sign)
    {
        Fixed64 coordinate =
            sign * Fixed64.FromFraction(5, 4);

        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(
            new Vector3d(coordinate, Fixed64.Zero, coordinate),
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal((Fixed64)sign, contact.SecondAnchor.LocalPoint.X);
        Assert.Equal((Fixed64)sign, contact.SecondAnchor.LocalPoint.Z);
    }

    [Fact]
    public void SphereCornerContact_AcceptsAnExplicitClosingVertex()
    {
        Vector2d[] closedUnitSquare =
        {
            new(-1, -1),
            new(1, -1),
            new(1, 1),
            new(-1, 1),
            new(-1, -1),
        };
        Vector3d center = new(
            Fixed64.FromFraction(5, 4),
            Fixed64.Zero,
            Fixed64.FromFraction(5, 4));

        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(
            center,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors openContact));
        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(
            center,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            closedUnitSquare,
            Fixed64.One,
            out FixedContactAnchors closedContact));

        AssertEquivalent(openContact, closedContact);
    }

    [Fact]
    public void SphereBeyondPlanarCornerRadius_RejectsVertexAxisSeparation()
    {
        Fixed64 coordinate = Fixed64.FromFraction(7, 5);

        Assert.False(FixedConvexPrismRelations.TryGetSphereContact(
            new Vector3d(coordinate, Fixed64.Zero, coordinate),
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void SphereBeyondPlanarFaceRadius_RejectsFaceAxisSeparation()
    {
        Assert.False(FixedConvexPrismRelations.TryGetSphereContact(
            new Vector3d(2, 0, 0),
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void SkewCapsuleBeyondPrismEdgeRadius_RejectsCrossAxisSeparation()
    {
        Vector3d localAxis =
            new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One)
                .Normalized;

        Assert.False(FixedConvexPrismRelations
            .TryGetCenteredCapsuleContact(
                new Vector3d(
                    Fixed64.Zero,
                    Fixed64.FromFraction(26, 25),
                    -Fixed64.FromFraction(26, 25)),
                FixedQuaternion.Identity,
                localAxis,
                Fixed64.FromFraction(1, 5),
                Fixed64.FromFraction(1, 20),
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
    }

    [Fact]
    public void CapsuleContact_SelectsExactShallowAxisBeforeDepthClamping()
    {
        Vector2d[] fullDomainSquare =
        {
            new(-Fixed64.MaxValue, -Fixed64.MaxValue),
            new(Fixed64.MaxValue, -Fixed64.MaxValue),
            new(Fixed64.MaxValue, Fixed64.MaxValue),
            new(-Fixed64.MaxValue, Fixed64.MaxValue),
        };

        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Fixed64.Zero,
            fullDomainSquare,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
        Assert.Equal(
            -Fixed64.MaxValue,
            contact.FirstAnchor.LocalDisplacement.Z);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.Zero,
                -Fixed64.MaxValue),
            point);
        Assert.Equal(-Vector3d.Forward, contact.Normal);
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void RotatedFiniteShapeContacts_MatchMaterializedSmallDomainGeometry()
    {
        Vector2d[] rectangle =
        {
            new(-Fixed64.Two, -Fixed64.Half),
            new(Fixed64.Two, -Fixed64.Half),
            new(Fixed64.Two, Fixed64.Half),
            new(-Fixed64.Two, Fixed64.Half),
        };
        Vector2d[] rotated =
        {
            Vector2d.Rotate(rectangle[0], Fixed64.HalfPi),
            Vector2d.Rotate(rectangle[1], Fixed64.HalfPi),
            Vector2d.Rotate(rectangle[2], Fixed64.HalfPi),
            Vector2d.Rotate(rectangle[3], Fixed64.HalfPi),
        };
        Vector3d center = new(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(
            center,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.HalfPi,
            rectangle,
            Fixed64.One,
            out FixedContactAnchors sphere));
        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(
            center,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            rotated,
            Fixed64.One,
            out FixedContactAnchors materializedSphere));
        AssertEquivalent(sphere, materializedSphere);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.HalfPi,
            rectangle,
            Fixed64.One,
            out FixedContactAnchors capsule));
        Assert.True(FixedConvexPrismRelations
            .TryGetCenteredCapsuleContact(
                center,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.Half,
                Vector3d.Zero,
                Fixed64.Zero,
                rotated,
                Fixed64.One,
                out FixedContactAnchors materializedCapsule));
        AssertEquivalent(capsule, materializedCapsule);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.HalfPi,
            rectangle,
            Fixed64.One,
            out FixedContactAnchors cylinder));
        Assert.True(FixedConvexPrismRelations
            .TryGetCenteredCylinderContact(
                center,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.Half,
                Vector3d.Zero,
                Fixed64.Zero,
                rotated,
                Fixed64.One,
                out FixedContactAnchors materializedCylinder));
        AssertEquivalent(cylinder, materializedCylinder);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.HalfPi,
            rectangle,
            Fixed64.One,
            out FixedContactAnchors cone));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            rotated,
            Fixed64.One,
            out FixedContactAnchors materializedCone));
        AssertEquivalent(cone, materializedCone);
    }

    [Fact]
    public void FiniteShapeContacts_RetainTheSuppliedRigidAxisFrame()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.PiOver4);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            rotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors capsule));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            Vector3d.Zero,
            rotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors cylinder));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(
            Vector3d.Zero,
            rotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors cone));

        Assert.Equal(rotation, capsule.FirstAnchor.Rotation);
        Assert.Equal(rotation, cylinder.FirstAnchor.Rotation);
        Assert.Equal(rotation, cone.FirstAnchor.Rotation);
    }

    [Fact]
    public void FiniteShapeContacts_ValidateCanonicalShapeInputs()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                default,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexPrismRelations.TryGetCenteredCylinderContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexPrismRelations.TryGetCenteredConeContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare.AsSpan(0, 2),
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Zero,
                out _));
    }

    [Fact]
    public void TriangleContact_RemainsOriginRelativeAndWindingStableAtTheScalarFace()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector2d[] reversedPrism =
        {
            UnitSquare[3],
            UnitSquare[2],
            UnitSquare[1],
            UnitSquare[0],
        };

        Assert.True(FixedConvexPrismRelations.TryGetTriangleContact(
            origin,
            FixedQuaternion.Identity,
            VerticalTriangleGeometry,
            origin,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors clockwise));
        Assert.True(FixedConvexPrismRelations.TryGetTriangleContact(
            origin,
            FixedQuaternion.Identity,
            VerticalTriangleGeometry,
            origin,
            Fixed64.Zero,
            reversedPrism,
            Fixed64.One,
            out FixedContactAnchors counterclockwise));

        Assert.Equal(clockwise.Depth, counterclockwise.Depth);
        Assert.Equal(clockwise.DepthIsClamped, counterclockwise.DepthIsClamped);
        Assert.NotEqual(default, clockwise.Normal);
        Assert.Equal(origin, clockwise.FirstAnchor.Origin);
        Assert.Equal(origin, clockwise.SecondAnchor.Origin);
    }

    [Fact]
    public void TriangleContact_RejectsFullDomainSeparationAndInvalidTriangleFrame()
    {
        Assert.False(FixedConvexPrismRelations.TryGetTriangleContact(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            VerticalTriangleGeometry,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexPrismRelations.TryGetTriangleContact(
                Vector3d.Zero,
                default,
                VerticalTriangleGeometry,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
        Assert.False(FixedConvexPrismRelations.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new FixedTriangle(
                Vector3d.Zero,
                Vector3d.Right,
                (Fixed64)2 * Vector3d.Right),
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void TriangleContact_RejectsVerticalAndPrismFaceSeparation()
    {
        var horizontalTriangle = new FixedTriangle(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new Vector3d(Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half));
        Vector2d[] xFaceFirst =
        {
            UnitSquare[3],
            UnitSquare[0],
            UnitSquare[1],
            UnitSquare[2],
        };

        Assert.False(FixedConvexPrismRelations.TryGetTriangleContact(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            FixedQuaternion.Identity,
            horizontalTriangle,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
        Assert.False(FixedConvexPrismRelations.TryGetTriangleContact(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            horizontalTriangle,
            Vector3d.Zero,
            Fixed64.Zero,
            xFaceFirst,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void TriangleContact_FromAboveUsesTheOppositeOrientedAxis()
    {
        var horizontalTriangle = new FixedTriangle(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new Vector3d(Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half));

        Assert.True(FixedConvexPrismRelations.TryGetTriangleContact(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(3, 4),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            horizontalTriangle,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.One, contact.SecondAnchor.LocalPoint.Y);
    }

    [Fact]
    public void TriangleContact_RejectsTriangleEdgeUpCrossSeparation()
    {
        var horizontalTriangle = new FixedTriangle(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.Zero,
                -Fixed64.FromFraction(1, 10)),
            new Vector3d(
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.FromFraction(1, 10)),
            new Vector3d(
                Fixed64.FromFraction(1, 5),
                Fixed64.Zero,
                Fixed64.Zero));

        Assert.False(FixedConvexPrismRelations.TryGetTriangleContact(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            horizontalTriangle,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void TriangleContact_RejectsSkewEdgeSeparation()
    {
        Fixed64 twentieth = Fixed64.FromFraction(1, 20);
        var skewTriangle = new FixedTriangle(
            new Vector3d(-Fixed64.FromFraction(1, 10), -twentieth, -twentieth),
            new Vector3d(Fixed64.FromFraction(1, 10), twentieth, twentieth),
            new Vector3d(
                -Fixed64.FromFraction(1, 10),
                -twentieth + Fixed64.MinIncrement,
                -twentieth - Fixed64.MinIncrement));

        Assert.False(FixedConvexPrismRelations.TryGetTriangleContact(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(26, 25),
                -Fixed64.FromFraction(26, 25)),
            FixedQuaternion.Identity,
            skewTriangle,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void TriangleContact_PreservesBothRigidFrames()
    {
        FixedQuaternion triangleRotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            Fixed64.Pi / Fixed64.Two);
        Fixed64 prismRotation = Fixed64.PiOver4;

        Assert.True(FixedConvexPrismRelations.TryGetTriangleContact(
            Vector3d.Zero,
            triangleRotation,
            VerticalTriangleGeometry,
            Vector3d.Zero,
            prismRotation,
            UnitSquare,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(triangleRotation, contact.FirstAnchor.Rotation);
        Assert.Equal(
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                -prismRotation),
            contact.SecondAnchor.Rotation);
        Assert.True(contact.FirstAnchor.TryGetProjectedOffsetFrom(
            contact.SecondAnchor,
            contact.Normal,
            out Fixed64 projectedSeparation));
        Assert.True(projectedSeparation >= Fixed64.Zero);
    }

    [Fact]
    public void TriangleContact_WarmedPathDoesNotAllocate()
    {
        _ = FixedConvexPrismRelations.TryGetTriangleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            VerticalTriangleGeometry,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.One,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();

        for (int iteration = 0; iteration < 64; iteration++)
        {
            Assert.True(FixedConvexPrismRelations.TryGetTriangleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                VerticalTriangleGeometry,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.One,
                out _));
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }

    private static void AssertEquivalent(
        FixedContactAnchors anchored,
        FixedContactAnchors materialized)
    {
        Assert.Equal(materialized.Normal, anchored.Normal);
        Assert.Equal(materialized.Depth, anchored.Depth);
        Assert.Equal(materialized.DepthIsClamped, anchored.DepthIsClamped);
        Assert.True(anchored.FirstAnchor.TryGetOffsetFrom(
            anchored.SecondAnchor,
            out Vector3d anchoredSeparation));
        Assert.True(materialized.FirstAnchor.TryGetOffsetFrom(
            materialized.SecondAnchor,
            out Vector3d materializedSeparation));
        Assert.Equal(materializedSeparation, anchoredSeparation);
    }

    private static void AssertEquivalentContactDecision(
        FixedContactAnchors baseline,
        FixedContactAnchors boundary)
    {
        Assert.Equal(baseline.Normal, boundary.Normal);
        Assert.Equal(baseline.Depth, boundary.Depth);
        Assert.Equal(baseline.DepthIsClamped, boundary.DepthIsClamped);
    }

    private static void AssertInsideSphereContact(
        Vector3d sphereCenter,
        Vector3d expectedNormal,
        Fixed64 expectedDepth)
    {
        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(
            sphereCenter,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.Half,
            out FixedContactAnchors sphere));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            sphereCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare,
            Fixed64.Half,
            out FixedContactAnchors collapsedCapsule));

        Assert.Equal(expectedNormal, sphere.Normal);
        Assert.Equal(expectedDepth, sphere.Depth);
        AssertEquivalent(sphere, collapsedCapsule);
    }
}
