using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleRigidContactsTests
{
    [Theory]
    [InlineData(1, 0, 1)]
    [InlineData(2, 2, 2)]
    [InlineData(0, 1, 4)]
    [InlineData(0, 0, 5)]
    [InlineData(4, 0, 3)]
    [InlineData(0, 4, 6)]
    public void CapsuleFeature_CoreEndpointOnFaceBoundary_ShouldRetainExactEdgeAndVertexIdentity(
        int xHalves, int zHalves, int expectedMask)
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right * 2, Vector3d.Forward * 2);
        Vector3d witness = new(Fixed64.FromFraction(xHalves, 2), Fixed64.Zero,
            Fixed64.FromFraction(zHalves, 2));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            witness + Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity,
            Fixed64.Half, Fixed64.Half, Vector3d.Up, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out bool localChart));
        Assert.True(localChart);
        Assert.Equal(expectedMask, feature.EdgeMask);
        Assert.True(feature.IsZeroSeparation);
        Assert.Equal(witness, contact.FirstAnchor.LocalPoint);
        Assert.Equal(-Fixed64.Quarter, contact.SecondAnchor.LocalPoint.Y);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
    }

    [Theory]
    [InlineData(0, 0, 0, true)]
    [InlineData(1, 0, 0, false)]
    [InlineData(0, 1, 0, false)]
    [InlineData(0, 0, 1, false)]
    public void CapsuleFeature_ZeroSeparation_ShouldRequireEveryExactCoordinateToCoincide(
        int x, int y, int z, bool coincident)
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(x, y, z) * Fixed64.Half, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, Vector3d.Up, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out _));
        Assert.Equal(coincident, feature.IsZeroSeparation);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
        if (coincident) Assert.Equal(Fixed64.One, contact.Depth);
        else Assert.Equal(Fixed64.Half, contact.Depth);
    }

    [Fact]
    public void CapsuleContact_PointCoreAgainstFullDomainFace_ShouldKeepExactInteriorWitness()
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.MinValue),
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.MinValue),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right * Fixed64.Quarter, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half,
            Vector3d.Right, out FixedContactAnchors contact, out AxisTriangleContactFeature feature, out _));
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.Equal(0, feature.EdgeMask);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleContact_PointCoreAgainstSubEpsilonTriangle_ShouldRetainExactEdgeFeature(bool parallelFace)
    {
        var triangle = new FixedTriangle(Vector3d.Zero,
            (parallelFace ? Vector3d.Up : Vector3d.Right) * Fixed64.MinIncrement,
            Vector3d.Forward * Fixed64.MinIncrement);
        Vector3d center = parallelFace
            ? new Vector3d(Fixed64.Quarter, Fixed64.Half, Fixed64.Half)
            : new Vector3d(Fixed64.Half, Fixed64.Quarter, Fixed64.Half);
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, Vector3d.Up, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out _));
        Assert.Equal(2, feature.EdgeMask);
        // The exact edge midpoint is half a raw unit on each vertex axis and rounds to even zero.
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
        Assert.True(contact.Normal.X > Fixed64.Zero);
        Assert.True(contact.Normal.Z > Fixed64.Zero);
        Assert.True(contact.Depth > Fixed64.Zero);
    }

    [Fact]
    public void CapsuleContact_ParallelPointOutsideFace_ShouldRetainTheExposedVertex()
    {
        var triangle = new FixedTriangle(new Vector3d(0, -2, -2),
            new Vector3d(0, 2, -2), new Vector3d(0, 0, 2));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Quarter, Fixed64.Zero, (Fixed64)3), FixedQuaternion.Identity,
            Fixed64.Zero, (Fixed64)2, Vector3d.Right, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out _));
        Assert.Equal(triangle.C, contact.FirstAnchor.LocalPoint);
        Assert.Equal(6, feature.EdgeMask);
        Assert.True(contact.Normal.X > Fixed64.Zero);
        Assert.True(contact.Normal.Z > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
    }

    [Fact]
    public void CapsuleContact_ParallelEdgeWithInteriorCoreWitness_ShouldRetainTheVertexContact()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Forward * 2, Vector3d.Up * 2);
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Quarter, -Fixed64.Quarter, -Fixed64.Quarter), FixedQuaternion.Identity,
            (Fixed64)2, Fixed64.One, Vector3d.Right, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out _));
        Assert.Equal(triangle.A, contact.FirstAnchor.LocalPoint);
        Assert.Equal(5, feature.EdgeMask);
        Assert.Equal(Fixed64.Quarter, contact.SecondAnchor.LocalPoint.Y);
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.True(contact.Normal.X > Fixed64.Zero);
        Assert.True(contact.Normal.Z < Fixed64.Zero);
        Assert.True(contact.Depth > Fixed64.Half);
    }

    [Fact]
    public void CapsuleFeature_ShouldPreserveRaySignsLostByFinalNormalRounding()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right * 2, Vector3d.Forward * 2);
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, (Fixed64)2, -Fixed64.MinIncrement), FixedQuaternion.Identity,
            Fixed64.Zero, (Fixed64)3, Vector3d.Up, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out _));
        Assert.Equal(1, feature.EdgeMask);
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
        Assert.Equal(-1, feature.CompareSeparationProjection(Vector3d.Zero, Vector3d.Forward));
        Assert.Equal(1, feature.CompareSeparationProjection(Vector3d.Forward, Vector3d.Zero));
    }

    [Theory]
    [InlineData(-2, 3)]
    [InlineData(2, 3)]
    [InlineData(2, 8)]
    public void CapsuleContact_ParallelCoreWithCenterOutsideFace_ShouldRetainEndpointOrCrossingContact(int centerY, int length)
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)), new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Quarter, (Fixed64)centerY, Fixed64.Zero), FixedQuaternion.Identity, (Fixed64)length,
            Fixed64.Half, Vector3d.Right, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.Equal(Fixed64.Zero, contact.FirstAnchor.LocalPoint.X);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleContact_ParallelSphereLimit_ShouldDistinguishExactTouchFromPositiveGap(bool positiveGap)
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)), new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2));
        Fixed64 x = Fixed64.Half + (positiveGap ? Fixed64.MinIncrement : Fixed64.Zero);
        bool hit = triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(x, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Fixed64.Zero,
            Fixed64.Half, Vector3d.Right, out FixedContactAnchors contact);
        Assert.Equal(!positiveGap, hit);
        if (hit) Assert.Equal(Fixed64.Zero, contact.Depth);
    }

    [Fact]
    public void CapsuleContact_ParallelCoreWithUnrepresentableEndpoint_ShouldKeepTheRationalAdmittedEndpoint()
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Zero, Fixed64.MinValue, (Fixed64)(-2)),
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, (Fixed64)(-2)), new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Quarter, Fixed64.MaxValue - Fixed64.One, Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.MaxValue, Fixed64.Half,
            Vector3d.Right, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.Equal(Fixed64.Zero, contact.FirstAnchor.LocalPoint.X);
        Assert.True(contact.SecondAnchor.LocalPoint.Y < Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleContact_ParallelCoreInsideFaceProjection_ShouldRetainFaceContact(bool sphereLimit)
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Zero, (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)(-2)), new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2));
        Fixed64 length = sphereLimit ? Fixed64.Zero : Fixed64.Half;
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right * Fixed64.Quarter, FixedQuaternion.Identity, length, Fixed64.Half,
            Vector3d.Right, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
    }

    [Theory]
    [InlineData(2, 2, 0)]
    [InlineData(2, -1, 1)]
    [InlineData(5, 5, 2)]
    [InlineData(-1, 2, 4)]
    [InlineData(-1, -1, 5)]
    [InlineData(9, -1, 3)]
    [InlineData(-1, 9, 6)]
    public void CapsuleFeature_ShouldRetainExactTriangleFaceEdgeAndVertexIdentity(int xQuarters, int zQuarters, int mask)
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right * 2, Vector3d.Forward * 2);
        Vector3d center = new(Fixed64.FromFraction(xQuarters, 4), Fixed64.Quarter, Fixed64.FromFraction(zQuarters, 4));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Vector3d.Up,
            out FixedContactAnchors contact, out AxisTriangleContactFeature feature, out bool localChart));
        Assert.True(localChart);
        Assert.Equal(mask, feature.EdgeMask);
        Assert.Equal(1, feature.CompareSeparationProjection(Vector3d.Zero, Vector3d.Up));
        Assert.Equal(-1, feature.CompareSeparationProjection(Vector3d.Up, Vector3d.Zero));
        Assert.Equal(0, feature.CompareSeparationProjection(Vector3d.Zero, Vector3d.Zero));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Vector3d.Up,
            out FixedContactAnchors ordinary));
        Assert.Equal(ordinary, contact);
    }

    [Fact]
    public void CapsuleFeature_CoincidentAxis_ShouldRetainZeroSeparationAndFallbackNormal()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right * 2, Vector3d.Forward * 2);
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity,
            Fixed64.One, Fixed64.One, Vector3d.Up, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out bool localChart));
        Assert.True(localChart);
        Assert.Equal(0, feature.EdgeMask);
        Assert.Equal(0, feature.CompareSeparationProjection(Vector3d.Zero, Vector3d.One));
        Assert.Equal(Vector3d.Up, contact.Normal);
    }

    [Fact]
    public void CapsuleFeature_DegenerateTriangle_ShouldDeclineTopologicalClassification()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One,
            Vector3d.Up, out _, out AxisTriangleContactFeature feature, out _));
        Assert.Equal(7, feature.EdgeMask);
        Assert.Equal(1, feature.CompareSeparationProjection(Vector3d.Zero, Vector3d.Up));
    }

    [Fact]
    public void CapsuleFeature_WorldFallback_ShouldCompareRaysInTheSelectedWorldChart()
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.MaxValue, -Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.MaxValue, Fixed64.One, -Fixed64.One), new Vector3d(Fixed64.MaxValue, Fixed64.Zero, (Fixed64)1));
        Assert.True(triangle.TryGetCenteredCapsuleContact(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Right, FixedQuaternion.Identity, (Fixed64)2,
            (Fixed64)2, Vector3d.Right, out _, out AxisTriangleContactFeature feature, out bool localChart));
        Assert.False(localChart);
        Assert.Equal(1, feature.CompareSeparationProjection(Vector3d.Zero, Vector3d.Right));
        Assert.Equal(1, feature.CompareSeparationProjection(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleFeature_SeparatedOrUnrepresentableChart_ShouldReturnAnEmptyCertificate(bool noChart)
    {
        var triangle = noChart
            ? new FixedTriangle(new Vector3d(Fixed64.MaxValue, -Fixed64.One, -Fixed64.One),
                new Vector3d(Fixed64.MaxValue, Fixed64.One, -Fixed64.One), new Vector3d(Fixed64.MaxValue, Fixed64.Zero, (Fixed64)1))
            : new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Forward);
        Vector3d origin = noChart ? new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero) : Vector3d.Zero;
        Vector3d center = noChart ? new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero) : Vector3d.Up * 4;
        Assert.False(triangle.TryGetCenteredCapsuleContact(origin, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Vector3d.Up,
            out FixedContactAnchors contact, out AxisTriangleContactFeature feature, out _));
        Assert.Equal(default, contact);
        Assert.Equal(0, feature.EdgeMask);
        Assert.Equal(0, feature.CompareSeparationProjection(Vector3d.Zero, Vector3d.One));
    }

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
