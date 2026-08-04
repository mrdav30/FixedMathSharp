using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleRigidContactsTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FiniteSurfaceContact_WithUnrepresentableWorldSupport_ShouldRetainSemanticAnchors(
        bool cone)
    {
        var triangle = new FixedTriangle(
            new Vector3d(2, -2, -2),
            new Vector3d(2, 2, -2),
            new Vector3d(2, 0, 2));
        Vector3d triangleOrigin = new(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d shapeCenter = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        bool hit = cone
            ? triangle.TryGetCenteredFiniteConeSupportContact(
                triangleOrigin,
                FixedQuaternion.Identity,
                shapeCenter,
                FixedQuaternion.Identity,
                (Fixed64)2,
                Fixed64.One,
                Vector3d.Right,
                -Vector3d.Right,
                out FixedContactAnchors contact)
            : triangle.TryGetCenteredFiniteCylinderSupportContact(
                triangleOrigin,
                FixedQuaternion.Identity,
                shapeCenter,
                FixedQuaternion.Identity,
                (Fixed64)2,
                Fixed64.One,
                Vector3d.Right,
                -Vector3d.Right,
                out contact);

        Assert.True(hit);
        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(Vector3d.Zero, separation);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FiniteSurfaceContact_WhenSupportProjectionMissesTriangle_ShouldReturnFalse(
        bool cone)
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Forward);

        bool hit = cone
            ? triangle.TryGetCenteredFiniteConeSupportContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(4, 1, 4),
                FixedQuaternion.Identity,
                (Fixed64)2,
                Fixed64.One,
                Vector3d.Down,
                Vector3d.Up,
                out FixedContactAnchors contact)
            : triangle.TryGetCenteredFiniteCylinderSupportContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(4, 1, 4),
                FixedQuaternion.Identity,
                (Fixed64)2,
                Fixed64.One,
                Vector3d.Down,
                Vector3d.Up,
                out contact);

        Assert.False(hit);
        Assert.Equal(default, contact);
    }

    [Fact]
    public void FiniteSurfaceContact_ClampsFullDomainPenetration()
    {
        Fixed64 plane = Fixed64.MaxValue;
        var triangle = new FixedTriangle(
            new Vector3d(plane, -Fixed64.One, -Fixed64.One),
            new Vector3d(plane, Fixed64.One, -Fixed64.One),
            new Vector3d(plane, Fixed64.Zero, Fixed64.One));

        Assert.True(triangle.TryGetCenteredFiniteCylinderSupportContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.MinIncrement,
            Fixed64.Zero,
            -Vector3d.Right,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void FiniteSurfaceContact_ReturnsExactRepresentablePenetration()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, -2, -2),
            new Vector3d(0, 2, -2),
            new Vector3d(0, 0, 2));

        Assert.True(triangle.TryGetCenteredFiniteCylinderSupportContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            (Fixed64)2,
            Fixed64.One,
            -Vector3d.Right,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void FiniteSurfaceContact_RejectsPositivePlaneSeparation()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, -2, -2),
            new Vector3d(0, 2, -2),
            new Vector3d(0, 0, 2));

        Assert.False(triangle.TryGetCenteredFiniteCylinderSupportContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, 0, 0),
            FixedQuaternion.Identity,
            (Fixed64)2,
            Fixed64.One,
            -Vector3d.Right,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void FiniteSurfaceContact_WhenSupportHasNoTriangleFrameChart_ShouldReturnFalse()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Up,
            Vector3d.Forward);

        Assert.False(triangle.TryGetCenteredFiniteCylinderSupportContact(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.One,
            Fixed64.One,
            Vector3d.Right,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void FiniteSurfaceContact_ValidatesDimensionsAndNormal()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Forward);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetCenteredFiniteCylinderSupportContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Right,
                Vector3d.Right,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetCenteredFiniteCylinderSupportContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                -Fixed64.MinIncrement,
                Vector3d.Right,
                Vector3d.Right,
                out _));
        Assert.Throws<ArgumentException>(() =>
            triangle.TryGetCenteredFiniteCylinderSupportContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Right,
                Vector3d.Zero,
                out _));
    }

    [Fact]
    public void CapsuleContact_WithUnrepresentableWorldWitnesses_ShouldRetainBothRigidFrames()
    {
        var triangle = new FixedTriangle(
            new Vector3d(2, -1, -1),
            new Vector3d(2, 1, -1),
            new Vector3d(2, 0, 1));
        Vector3d triangleOrigin = new(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(triangle.TryGetCenteredCapsuleContact(
            triangleOrigin,
            FixedQuaternion.Identity,
            capsuleCenter,
            FixedQuaternion.Identity,
            (Fixed64)2,
            Fixed64.One,
            -Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(Vector3d.Zero, separation);
        Assert.Equal(Fixed64.Zero, contact.SecondAnchor.LocalPoint.X);
        Assert.Equal(Fixed64.Zero, contact.SecondAnchor.LocalPoint.Z);
        Assert.True(contact.SecondAnchor.LocalPoint.Y >= -Fixed64.One);
        Assert.True(contact.SecondAnchor.LocalPoint.Y <= Fixed64.One);
        Assert.Equal(Vector3d.Right, contact.SecondAnchor.LocalDisplacement);
    }

    [Fact]
    public void CapsuleContact_WithUnrepresentableLocalCenter_ShouldUseWorldChart()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.MaxValue, -Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.One));
        Vector3d triangleOrigin = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = Vector3d.Right;

        Assert.True(triangle.TryGetCenteredCapsuleContact(
            triangleOrigin,
            FixedQuaternion.Identity,
            capsuleCenter,
            FixedQuaternion.Identity,
            (Fixed64)2,
            (Fixed64)2,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d trianglePoint));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d capsulePoint));
        Assert.True(trianglePoint.X > capsulePoint.X);
        Assert.Equal(
            contact.Depth,
            trianglePoint.X - capsulePoint.X);
    }

    [Fact]
    public void CapsuleContact_WhenSeparatedOrNoChartExists_ShouldReturnFalse()
    {
        var ordinary = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Forward);
        Assert.False(ordinary.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up * 4,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                -Fixed64.Pi / Fixed64.Two),
            Fixed64.One,
            Fixed64.One,
            Vector3d.Up,
            out FixedContactAnchors separated));
        Assert.Equal(default, separated);

        var unmaterializable = new FixedTriangle(
            new Vector3d(Fixed64.MaxValue, -Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.One));
        Assert.False(unmaterializable.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.One,
            Fixed64.One,
            Vector3d.Right,
            out FixedContactAnchors noChart));
        Assert.Equal(default, noChart);
    }

    [Fact]
    public void CapsuleContact_WithCoincidentAxisAndTriangle_ShouldUseFallbackNormal()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-2, 0, -2),
            new Vector3d(2, 0, -2),
            new Vector3d(0, 0, 4));

        Assert.True(triangle.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
    }

    [Fact]
    public void CapsuleContact_ShouldValidateTriangleFrame()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up);

        Assert.Throws<ArgumentException>(() =>
            triangle.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                default,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Right,
                out _));
    }

    [Fact]
    public void SphereContact_WithUnrepresentableWorldTriangle_ShouldRetainLocalAnchor()
    {
        var triangle = new FixedTriangle(
            new Vector3d(2, -1, -1),
            new Vector3d(2, 1, -1),
            new Vector3d(2, 0, 1));
        Vector3d origin = new(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(triangle.TryGetSphereContact(
            origin,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(Vector3d.Zero, separation);
    }

    [Fact]
    public void SphereContact_WithUnrepresentableLocalCenter_ShouldUseWorldChart()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.MaxValue, -Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.One));
        Vector3d origin = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d sphereCenter = new(
            Fixed64.MaxValue - Fixed64.MinIncrement,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(triangle.TryGetSphereContact(
            origin,
            FixedQuaternion.Identity,
            sphereCenter,
            FixedQuaternion.Identity,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(
            out Vector3d trianglePoint));
        Assert.Equal(
            new Vector3d(
                -Fixed64.MinIncrement,
                Fixed64.Zero,
                Fixed64.Zero),
            trianglePoint);
        Assert.True(contact.SecondAnchor.TryGetPoint(
            out Vector3d spherePoint));
        Assert.Equal(trianglePoint, spherePoint);
    }

    [Fact]
    public void SphereContact_WhenNoInteractingChartExists_ShouldReturnFalse()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.MaxValue, -Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.One));

        Assert.False(triangle.TryGetSphereContact(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void SphereContact_WithUnrepresentableLocalDistance_ShouldRemainSeparated()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.MaxValue, -Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.One));

        Assert.False(triangle.TryGetSphereContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Fixed64.MaxValue,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void SphereContact_WithinPlaneTolerance_ShouldOrientTheWindingTowardTheSphere()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, -2, -2),
            new Vector3d(0, 2, -2),
            new Vector3d(0, 0, 4));
        Vector3d center = new(
            -Fixed64.MinIncrement,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(triangle.TryGetSphereContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            center,
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(-Vector3d.Right, contact.Normal);
    }

    [Fact]
    public void SphereContact_WithinPlaneTolerance_ShouldRetainWindingTowardTheSphere()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, -2, -2),
            new Vector3d(0, 2, -2),
            new Vector3d(0, 0, 4));
        Vector3d center = new(
            Fixed64.MinIncrement,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(triangle.TryGetSphereContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            center,
            FixedQuaternion.Identity,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
    }

    [Fact]
    public void SphereContact_ShouldValidateFrameAndRadius()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up);

        Assert.Throws<ArgumentException>(() =>
            triangle.TryGetSphereContact(
                Vector3d.Zero,
                default,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            triangle.TryGetSphereContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                default,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetSphereContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                -Fixed64.One,
                out _));
    }
}
