//=======================================================================
// FixedOrientedBox.Capsule.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxCapsuleTests
{
    [Fact]
    public void ContactOffsets_UseCenteredCapsuleSupport()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.FromFraction(5, 4),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Forward,
            (Fixed64)2,
            Fixed64.Half,
            out FixedContactAnchors contact));

        Assert.Equal(new Vector3d(1, 0, 0), GetOffset(contact.FirstAnchor));
        Assert.Equal(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero), GetOffset(contact.SecondAnchor));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True((contact.Depth - Fixed64.FromFraction(1, 4)).Abs()
            <= Fixed64.Epsilon);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactOffsets_RejectEndpointCornerSeparationAndIncludeTangency()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(5, 2)),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Assert.False(box.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Forward,
            (Fixed64)2,
            Fixed64.Half,
            out FixedContactAnchors separated));
        Assert.Equal(default, separated);

        var tangent = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(tangent.TryGetCenteredCapsuleContact(
            new Vector3d(2, 0, 0),
            FixedQuaternion.Identity,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
    }

    [Fact]
    public void ContactOffsets_RejectEndpointToEdgeSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.FromFraction(19, 10),
                Fixed64.FromFraction(7, 5),
                Fixed64.Zero),
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Right),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ContactOffsets_RejectLargeRepresentableSphereSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetCenteredCapsuleContact(
            new Vector3d(50_001, 0, 0),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            (Fixed64)48_000,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void PossibleOverlap_RejectsRadialCornerSeparationWithoutContactWitnesses()
    {
        Vector3d boxCenter = Vector3d.Zero;
        FixedQuaternion orientation = FixedQuaternion.Identity;
        Vector3d halfExtents = Vector3d.One;

        Assert.False(WideOrientedBox.CanOverlapCenteredCapsule(
            boxCenter,
            orientation,
            halfExtents,
            new Vector3d(
                Fixed64.FromFraction(7, 5),
                Fixed64.FromFraction(7, 5),
                Fixed64.FromFraction(7, 5)),
            Vector3d.Forward,
            Fixed64.Two,
            Fixed64.Half));
        Assert.True(WideOrientedBox.CanOverlapCenteredCapsule(
            boxCenter,
            orientation,
            halfExtents,
            new Vector3d(
                Fixed64.FromFraction(5, 4),
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Forward,
            Fixed64.Two,
            Fixed64.Half));
    }

    [Fact]
    public void ContactOffsets_MatchRotatedFaceToPerpendicularCapsuleCore()
    {
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)45,
                Fixed64.Zero);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            rotation,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Vector3d capsuleCenter = rotation * new Vector3d(
            Fixed64.Zero,
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero);
        FixedQuaternion capsuleRotation = rotation
            * FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)(-90));
        Assert.True(box.TryGetCenteredCapsuleContact(
            capsuleCenter,
            capsuleRotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.True(rotation.TryRotate(
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero),
            out Vector3d expectedBoxOffset));
        Assert.True(rotation.TryRotate(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            out Vector3d expectedCapsuleOffset));

        Assert.True(Vector3d.Distance(contact.Normal, Vector3d.Up)
            <= Fixed64.Epsilon);
        Assert.True((contact.Depth - Fixed64.FromFraction(1, 4)).Abs()
            <= Fixed64.Epsilon);
        Assert.True(Vector3d.Distance(
            expectedCapsuleOffset,
            GetOffset(contact.SecondAnchor)) <= Fixed64.Epsilon);
        Assert.True(Vector3d.Distance(
            expectedBoxOffset,
            GetOffset(contact.FirstAnchor)) <= Fixed64.Epsilon);
    }

    [Fact]
    public void ContactOffsets_OnAxialFace_UseTheCapsuleCap()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(7, 4),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Vector3d.Up, contact.FirstAnchor.LocalPoint);
        Assert.Equal(-Vector3d.Up, contact.SecondAnchor.LocalPoint);
        Assert.Equal(-Vector3d.Up * Fixed64.Half, contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
    }

    [Fact]
    public void ContactOffsets_PreserveExactCornerAxisConstraint()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Vector3d touchingEndpoint = new(
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Zero);
        Vector3d otherEndpoint = new(
            Fixed64.FromFraction(2, 5),
            Fixed64.FromFraction(51, 100),
            Fixed64.Zero);
        Vector3d segment = otherEndpoint - touchingEndpoint;
        Fixed64 segmentLength = segment.Magnitude;
        Vector3d segmentDirection = segment / segmentLength;
        Vector3d rotationAxis = Vector3d.Cross(
            Vector3d.Up,
            segmentDirection);
        FixedQuaternion rotation = new FixedQuaternion(
            rotationAxis.X,
            rotationAxis.Y,
            rotationAxis.Z,
            Fixed64.One + Vector3d.Dot(
                Vector3d.Up,
                segmentDirection)).Normalized;
        Vector3d capsuleCenter =
            (touchingEndpoint + otherEndpoint) * Fixed64.Half;
        Fixed64 radius = Fixed64.FromFraction(1, 10);

        Assert.True(box.TryGetCenteredCapsuleContact(
            capsuleCenter,
            rotation,
            Vector3d.Up,
            segmentLength,
            radius,
            out FixedContactAnchors contact));

        Assert.True(contact.Normal.IsNormalized());
        Assert.True(Vector3d.Dot(
            contact.Normal,
            capsuleCenter - box.Center) > Fixed64.Zero);
        Assert.True(contact.SecondAnchor.TryGetProjectedOffsetFrom(
            contact.FirstAnchor,
            contact.Normal,
            out Fixed64 signedSeparation));
        Assert.True((signedSeparation + contact.Depth).Abs()
            <= Fixed64.Epsilon);
    }

    [Fact]
    public void ContactOffsets_RoundIrrationalCornerDepthToNearestRaw()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Fixed64 gap = Fixed64.Half;

        Assert.True(box.TryGetCenteredCapsuleContact(
            new Vector3d(gap + Fixed64.One, gap + Fixed64.One, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors contact));

        Fixed64 expected =
            Fixed64.One - FixedMath.Sqrt(gap * gap + gap * gap);
        Assert.True((contact.Depth - expected).Abs() <= Fixed64.MinIncrement);
        Assert.Equal(new Vector3d(1, 1, 0), contact.FirstAnchor.LocalPoint);
        Assert.True(contact.Normal.X > Fixed64.Zero);
        Assert.Equal(contact.Normal.X, contact.Normal.Y);
    }

    [Fact]
    public void ContactOffsets_RoundSubLatticeCornerDepthToNearestRaw()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Fixed64 oneRaw = Fixed64.MinIncrement;
        Vector3d center = new(
            Fixed64.One + oneRaw,
            Fixed64.One + oneRaw,
            Fixed64.Zero);

        Assert.True(box.TryGetCenteredCapsuleContact(
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.FromRaw(2),
            out FixedContactAnchors contact));

        Assert.Equal(oneRaw, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(new Vector3d(1, 1, 0), contact.FirstAnchor.LocalPoint);
        Assert.True(contact.Normal.X > Fixed64.Zero);
        Assert.Equal(contact.Normal.X, contact.Normal.Y);
    }

    [Fact]
    public void ContactOffsets_AcrossOppositeScalarFacesRetainWideSupport()
    {
        var box = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.PiOver4),
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.One));

        Assert.True(box.TryGetCenteredCapsuleContact(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));
        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.True(Vector3d.Dot(separation, contact.Normal) >= Fixed64.Zero);
    }

    [Fact]
    public void ContactOffsets_RetainUnrepresentableCombinedAxialSupport()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.MaxValue));

        Assert.True(box.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.False(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out _));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.Equal(
            contact.Depth,
            contact.FirstAnchor.ProjectNonNegativeOffsetFrom(
                contact.SecondAnchor,
                contact.Normal));
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactOffsets_RemainRelativeAtScalarFaceAndClampDepth()
    {
        var scalarFace = new FixedOrientedBox(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(scalarFace.TryGetCenteredCapsuleContact(
            scalarFace.Center,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.One,
            Fixed64.Half,
            out FixedContactAnchors boundary));
        Assert.True(boundary.Normal.MagnitudeSquared > Fixed64.Zero);

        var huge = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));
        Assert.True(huge.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out FixedContactAnchors clamped));
        Assert.Equal(Fixed64.MaxValue, clamped.Depth);
        Assert.True(clamped.DepthIsClamped);
    }

    [Fact]
    public void ContactOffsets_KeepTheSameFeatureUnderAnExactRigidPose()
    {
        var baselineBox = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d localCapsuleCenter = new(
            Fixed64.FromFraction(5, 4),
            Fixed64.Zero,
            Fixed64.Zero);
        Assert.True(baselineBox.TryGetCenteredCapsuleContact(
            localCapsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors baseline));

        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.Pi / (Fixed64)7);
        Vector3d scalarFaceCenter = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Assert.True(rotation.TryRotate(
            localCapsuleCenter,
            out Vector3d rotatedOffset));
        Assert.True(Vector3d.TryAdd(
            scalarFaceCenter,
            rotatedOffset,
            out Vector3d capsuleCenter));
        var transformedBox = new FixedOrientedBox(
            scalarFaceCenter,
            rotation,
            Vector3d.One);
        Assert.True(transformedBox.TryGetCenteredCapsuleContact(
            capsuleCenter,
            rotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors transformed));
        Assert.True(rotation.TryRotate(
            baseline.Normal,
            out Vector3d expectedNormal));

        Assert.Equal(baseline.Depth, transformed.Depth);
        Assert.True(Vector3d.Distance(
            expectedNormal,
            transformed.Normal) <= Fixed64.Epsilon);
        Assert.Equal(
            baseline.FirstAnchor.LocalPoint,
            transformed.FirstAnchor.LocalPoint);
        Assert.Equal(
            baseline.SecondAnchor.LocalPoint,
            transformed.SecondAnchor.LocalPoint);
    }

    [Fact]
    public void ContactOffsets_KeepCornerFeatureUnderRelativeRigidPose()
    {
        var baselineBox = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Vector3d touchingEndpoint = new(
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Zero);
        Vector3d otherEndpoint = new(
            Fixed64.FromFraction(2, 5),
            Fixed64.FromFraction(51, 100),
            Fixed64.Zero);
        Vector3d segment = otherEndpoint - touchingEndpoint;
        Fixed64 segmentLength = segment.Magnitude;
        Vector3d segmentDirection = segment / segmentLength;
        Vector3d rotationAxis = Vector3d.Cross(
            Vector3d.Up,
            segmentDirection);
        FixedQuaternion capsuleRotation = new FixedQuaternion(
            rotationAxis.X,
            rotationAxis.Y,
            rotationAxis.Z,
            Fixed64.One + Vector3d.Dot(
                Vector3d.Up,
                segmentDirection)).Normalized;
        Vector3d capsuleCenter =
            (touchingEndpoint + otherEndpoint) * Fixed64.Half;
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        Assert.True(baselineBox.TryGetCenteredCapsuleContact(
            capsuleCenter,
            capsuleRotation,
            Vector3d.Up,
            segmentLength,
            radius,
            out FixedContactAnchors baseline));

        var rigidRotation = new FixedQuaternion(
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half);
        Vector3d scalarFaceCenter = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Assert.True(Vector3d.TryAdd(
            scalarFaceCenter,
            rigidRotation * capsuleCenter,
            out Vector3d transformedCenter));
        var transformedBox = new FixedOrientedBox(
            scalarFaceCenter,
            rigidRotation,
            baselineBox.HalfExtents);
        Assert.True(transformedBox.TryGetCenteredCapsuleContact(
            transformedCenter,
            rigidRotation * capsuleRotation,
            Vector3d.Up,
            segmentLength,
            radius,
            out FixedContactAnchors transformed));
        Vector3d expectedNormal = rigidRotation * baseline.Normal;

        Assert.Equal(baseline.Depth, transformed.Depth);
        Assert.Equal(baseline.DepthIsClamped, transformed.DepthIsClamped);
        Assert.Equal(baseline.FirstAnchor.LocalPoint, transformed.FirstAnchor.LocalPoint);
        Assert.Equal(baseline.SecondAnchor.LocalPoint, transformed.SecondAnchor.LocalPoint);
        Assert.True(Vector3d.Distance(expectedNormal, transformed.Normal)
            <= Fixed64.Epsilon);
    }

    [Fact]
    public void ContactOffsets_EqualRankTieKeepsFirstFaceAxis()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        FixedQuaternion capsuleRotation =
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Right);
        Assert.True(box.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            capsuleRotation,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors expected));

        Assert.Equal(Vector3d.Forward, expected.Normal);
        Assert.Equal(new Vector3d(0, 0, 1), expected.FirstAnchor.LocalPoint);
        for (int iteration = 0; iteration < 8; iteration++)
        {
            Assert.True(box.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                capsuleRotation,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                out FixedContactAnchors actual));
            Assert.Equal(expected, actual);
        }
    }

    [Fact]
    public void ContactOffsets_EqualDepthLaterFaceReplacesCrossAxis()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(3, 3, 5));
        Vector3d capsuleAxis = new(
            Fixed64.FromFraction(4, 5),
            Fixed64.FromFraction(3, 5),
            Fixed64.Zero);
        FixedQuaternion capsuleRotation =
            WideGeometry.GetCanonicalAxisRotation(capsuleAxis);

        Assert.True(box.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            capsuleRotation,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.Half,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(
            new Vector3d(
                -Fixed64.FromFraction(8, 5),
                (Fixed64)3,
                Fixed64.Zero),
            contact.FirstAnchor.LocalPoint);
        Assert.Equal(Fixed64.FromFraction(47, 10), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactOffsets_KeepEndpointEdgeFeatureUnderRigidPose()
    {
        var baselineBox = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d capsuleCenter = new(
            Fixed64.FromFraction(8, 5),
            Fixed64.FromFraction(13, 10),
            Fixed64.Zero);
        FixedQuaternion capsuleRotation =
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Right);
        Assert.True(baselineBox.TryGetCenteredCapsuleContact(
            capsuleCenter,
            capsuleRotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            out FixedContactAnchors baseline));
        Assert.True(baseline.Normal.X > Fixed64.Zero);
        Assert.True(baseline.Normal.Y > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, baseline.Normal.Z);
        Assert.True((baseline.Normal.Y - (baseline.Normal.X * (Fixed64)3)).Abs()
            <= Fixed64.Epsilon);
        Assert.Equal(Fixed64.Zero, baseline.FirstAnchor.LocalPoint.Z);

        var rigidRotation = new FixedQuaternion(
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half);
        var transformedBox = new FixedOrientedBox(
            Vector3d.Zero,
            rigidRotation,
            Vector3d.One);
        Assert.True(transformedBox.TryGetCenteredCapsuleContact(
            rigidRotation * capsuleCenter,
            rigidRotation * capsuleRotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            out FixedContactAnchors transformed));

        Assert.Equal(baseline.Depth, transformed.Depth);
        Assert.Equal(baseline.DepthIsClamped, transformed.DepthIsClamped);
        Assert.Equal(baseline.FirstAnchor.LocalPoint, transformed.FirstAnchor.LocalPoint);
        Assert.Equal(baseline.SecondAnchor.LocalPoint, transformed.SecondAnchor.LocalPoint);
        Assert.True(Vector3d.Distance(
            rigidRotation * baseline.Normal,
            transformed.Normal) <= Fixed64.Epsilon);
    }

    [Fact]
    public void ContactOffsets_PerpendicularAxisUsesSegmentSurfaceAndFreeCoordinates()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Vector3d capsuleCenter = new(
            Fixed64.FromFraction(1, 4),
            Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(1, 8));
        FixedQuaternion capsuleRotation =
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Right);

        Assert.True(box.TryGetCenteredCapsuleContact(
            capsuleCenter,
            capsuleRotation,
            Vector3d.Up,
            Fixed64.FromFraction(1, 4),
            Fixed64.Half,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(
            new Vector3d(
                Fixed64.FromFraction(1, 8),
                Fixed64.Half,
                capsuleCenter.Z),
            contact.FirstAnchor.LocalPoint);
        Assert.Equal(
            new Vector3d(
                -Fixed64.FromFraction(1, 8),
                -Fixed64.Half,
                Fixed64.Zero),
            GetOffset(contact.SecondAnchor));
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
    }

    [Fact]
    public void ContactOffsets_WarmedPathDoesNotAllocate()
    {
        FixedQuaternion boxRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)29,
                (Fixed64)11);
        var box = new FixedOrientedBox(
            new Vector3d(3, -2, 5),
            boxRotation,
            Vector3d.One);
        Vector3d capsuleCenter = box.Center + boxRotation * new Vector3d(
            Fixed64.FromFraction(5, 4),
            Fixed64.Zero,
            Fixed64.Zero);
        FixedQuaternion capsuleRotation = boxRotation
            * FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)37);
        Assert.True(box.TryGetCenteredCapsuleContact(
            capsuleCenter,
            capsuleRotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors expected));

        long before = GC.GetAllocatedBytesForCurrentThread();
        bool stable = true;
        for (int iteration = 0; iteration < 32; iteration++)
        {
            bool hit = box.TryGetCenteredCapsuleContact(
                capsuleCenter,
                capsuleRotation,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Half,
                out FixedContactAnchors actual);
            stable &= hit
                && actual.FirstAnchor == expected.FirstAnchor
                && actual.SecondAnchor == expected.SecondAnchor
                && actual.Normal == expected.Normal
                && actual.Depth == expected.Depth
                && actual.DepthIsClamped == expected.DepthIsClamped;
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.True(stable);
        Assert.Equal(0L, allocated);
    }

    [Fact]
    public void ContactOffsets_RetainFullDomainRotatedEdgeWitnesses()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.PiOver4);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            rotation,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue));

        Assert.True(WideOrientedBox.CanOverlapCenteredCapsule(
            box.Center,
            box.Orientation,
            box.HalfExtents,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One));
        Assert.True(box.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.FirstAnchor.TryGetPoint(out _));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));
    }

    [Fact]
    public void ContactOffsets_RetainTranslatedRotatedFullDomainCornerWitness()
    {
        Fixed64 oppositeCenter = -Fixed64.MaxValue
            * Fixed64.FromFraction(7, 10);
        var box = new FixedOrientedBox(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4),
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.One));
        FixedQuaternion capsuleRotation = box.Orientation;

        Assert.True(box.TryGetCenteredCapsuleContact(
            new Vector3d(oppositeCenter, Fixed64.Zero, Fixed64.Zero),
            capsuleRotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));

        Assert.NotEqual(Fixed64.Zero, contact.Normal.X);
        Assert.NotEqual(Fixed64.Zero, contact.Normal.Y);
        Assert.Equal(Fixed64.MaxValue, contact.FirstAnchor.LocalPoint.X.Abs());
        Assert.Equal(Fixed64.MaxValue, contact.FirstAnchor.LocalPoint.Y.Abs());
        Assert.True(contact.Normal.IsNormalized());
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ContactOffsets_RejectInvalidCapsuleInputs()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<ArgumentException>(() =>
            box.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                default,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            box.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Right,
                -Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Right,
                Fixed64.One,
                -Fixed64.One,
                out _));
    }

    private static Vector3d GetOffset(FixedPointAnchor anchor)
    {
        var origin = new FixedPointAnchor(
            anchor.Origin,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Assert.True(anchor.TryGetOffsetFrom(origin, out Vector3d offset));
        return offset;
    }

}
