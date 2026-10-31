using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void CollinearTriangleMiddleVertex_ShouldRetainItsAuthoredIncidence()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Right * 2);
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right + Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.Half, Vector3d.Up, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out _));
        Assert.Equal(7, feature.EdgeMask);
        Assert.Equal(1, feature.TriangleVertex);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Vector3d.Right, point);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
    }

    [Theory]
    [InlineData(1, 4, 1)]
    [InlineData(2, 2, 1.5)]
    public void ParallelCapsuleEdgeFamily_ShouldRetainAnExactInteriorWitness(int centerZ, int length, double expectedZ)
    {
        var segment = new FixedTriangle(Vector3d.Zero, Vector3d.Forward * 2, Vector3d.Forward * 2);
        FixedQuaternion rotation = FixedQuaternion.LookRotation(Vector3d.Up, Vector3d.Forward);
        Assert.True(segment.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(1, 0, centerZ), rotation, (Fixed64)length, Fixed64.Two, Vector3d.Right,
            out FixedContactAnchors contact, out AxisTriangleContactFeature feature, out _));
        Assert.Equal(-1, feature.TriangleVertex);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)expectedZ), point);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void CenteredSphereTriangleFeature_ShouldRetainExactPlaneHemisphere(int side)
    {
        var triangle = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up * (Fixed64.Quarter * side), FixedQuaternion.Identity, Fixed64.Zero,
            Fixed64.Half, Vector3d.Up, out _, out AxisTriangleContactFeature feature, out bool local));
        Assert.True(local);
        Assert.Equal(side, feature.CompareSeparationProjection(default, Signed192.Signed(1), default));
        Assert.Equal(-side, feature.CompareSeparationProjection(default, Signed192.Signed(-1), default));
        Assert.Equal(0, feature.CompareSeparationProjection(Signed192.Signed(1), default, default));
        var normal = new WideAxis3(default, Signed320.ExtendValue(Signed192.Signed(1)), default);
        Assert.Equal(side, feature.CompareSeparationProjection(normal));
        Assert.Equal(-side, feature.CompareSeparationProjection(-normal));
    }

    [Theory]
    [InlineData(0, 3, 0, 2, 2)]
    [InlineData(0, 0.5, 0.25, 0.5, -1)]
    [InlineData(1, 0.5, 0.25, 0.5, -1)]
    [InlineData(2, 0.5, 0.25, 0.5, -1)]
    public void CollinearTriangle_ShouldDistinguishItsLastVertexFromFiniteInteriorWitnesses(
        int axis, double coordinate, double gap, double radius, int expectedVertex)
    {
        Vector3d direction = axis == 0 ? new Vector3d(1, 1, 0) : axis == 1 ? Vector3d.Up : Vector3d.Forward;
        Vector3d perpendicular = axis == 0 ? Vector3d.Forward : Vector3d.Right;
        var triangle = new FixedTriangle(Vector3d.Zero, direction, direction * 2);
        Vector3d center = direction * (Fixed64)coordinate + perpendicular * (Fixed64)gap;
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)radius, perpendicular,
            out FixedContactAnchors contact, out AxisTriangleContactFeature feature, out _));
        Assert.Equal(7, feature.EdgeMask);
        Assert.Equal(expectedVertex, feature.TriangleVertex);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(direction * (expectedVertex == 2 ? Fixed64.Two : (Fixed64)coordinate), point);
        Fixed64 expectedGap = expectedVertex == 2 ? FixedMath.Sqrt(Fixed64.Two) : (Fixed64)gap;
        Assert.InRange(FixedMath.Abs(contact.Depth - ((Fixed64)radius - expectedGap)).m_rawValue, 0L, 1L);
    }

    [Theory]
    [InlineData(5, 2, 4, false)]
    [InlineData(3, 2, 2, false)]
    [InlineData(-1, 0, 0, false)]
    [InlineData(-3, 0, -2, false)]
    [InlineData(5, 2, 4, true)]
    [InlineData(-3, 0, -2, true)]
    public void ParallelFiniteCore_WithDisjointOrTouchingIntervals_ShouldKeepTheActualEndpointPair(
        int centerZ, int triangleZ, int coreZ, bool reverse)
    {
        Vector3d first = reverse ? Vector3d.Forward * 2 : Vector3d.Zero;
        Vector3d second = reverse ? Vector3d.Zero : Vector3d.Forward * 2;
        var triangle = new FixedTriangle(first, second, second);
        Assert.True(triangle.TryGetClosestPointsToCenteredAxis(new Vector3d(1, 0, centerZ),
            Vector3d.Forward, Fixed64.Two, out Vector3d trianglePoint, out Vector3d corePoint));
        Assert.Equal(Vector3d.Forward * (Fixed64)triangleZ, trianglePoint);
        Assert.Equal(new Vector3d(1, 0, coreZ), corePoint);
    }

    [Fact]
    public void CenteredAxisTriangleClosestPoints_SelectFaceEdgeAndIntersection()
    {
        FixedTriangle triangle = new(
            new Vector3d(-2, 0, -2),
            new Vector3d(2, 0, -2),
            new Vector3d(0, 0, 2));

        Assert.True(triangle.TryGetClosestPointsToCenteredAxis(
            new Vector3d(0, 2, 0),
            Vector3d.Up,
            Fixed64.Two,
            out Vector3d facePoint,
            out Vector3d faceAxisPoint));
        Assert.Equal(Vector3d.Zero, facePoint);
        Assert.Equal(Vector3d.Up, faceAxisPoint);

        Assert.True(triangle.TryGetClosestPointsToCenteredAxis(
            new Vector3d(3, 2, -2),
            Vector3d.Up,
            Fixed64.Two,
            out Vector3d edgePoint,
            out Vector3d edgeAxisPoint));
        Assert.Equal(new Vector3d(2, 0, -2), edgePoint);
        Assert.Equal(new Vector3d(3, 1, -2), edgeAxisPoint);

        Assert.True(triangle.TryGetClosestPointsToCenteredAxis(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            out Vector3d intersectionPoint,
            out Vector3d intersectionAxisPoint));
        Assert.Equal(Vector3d.Zero, intersectionPoint);
        Assert.Equal(intersectionPoint, intersectionAxisPoint);
    }

    [Fact]
    public void CenteredAxisTriangleClosestPoints_DoNotMaterializeRejectedEndpoint()
    {
        Fixed64 plane = Fixed64.MaxValue - Fixed64.One;
        FixedTriangle triangle = new(
            new Vector3d(plane, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(plane, Fixed64.Two, Fixed64.Zero),
            new Vector3d(plane, Fixed64.Zero, Fixed64.Two));

        Assert.False(FixedSegment.TryGetCenteredAxisEndpoint(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Left,
            Fixed64.Two,
            positive: false,
            out _));
        Assert.True(triangle.TryGetClosestPointsToCenteredAxis(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Left,
            Fixed64.Two,
            out Vector3d trianglePoint,
            out Vector3d axisPoint));
        Assert.Equal(
            new Vector3d(plane, Fixed64.Zero, Fixed64.Zero),
            trianglePoint);
        Assert.Equal(trianglePoint, axisPoint);
    }

    [Fact]
    public void CenteredCapsuleTriangleContact_DecidesBeforeFinalWitnesses()
    {
        FixedTriangle triangle = new(
            new Vector3d(-2, 0, -2),
            new Vector3d(2, 0, -2),
            new Vector3d(0, 0, 2));

        Assert.True(triangle.TryGetCenteredCapsuleContact(
            new Vector3d(0, 2, 0),
            Vector3d.Up,
            Fixed64.Two,
            (Fixed64)2,
            Vector3d.Up,
            out Vector3d pointOnTriangle,
            out Vector3d pointOnCapsule,
            out Vector3d normal,
            out Fixed64 depth));
        Assert.Equal(Vector3d.Zero, pointOnTriangle);
        Assert.Equal(new Vector3d(0, -1, 0), pointOnCapsule);
        Assert.Equal(Vector3d.Up, normal);
        Assert.Equal(Fixed64.One, depth);

        Assert.False(triangle.TryGetCenteredCapsuleContact(
            new Vector3d(0, 2, 0),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.FromRaw(Fixed64.One.m_rawValue - 1L),
            Vector3d.Up,
            out _,
            out _,
            out _,
            out _));

        Vector3d scalarFace = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);
        FixedTriangle faceTriangle = new(
            scalarFace,
            scalarFace,
            scalarFace);
        Assert.True(faceTriangle.DoesCenteredCapsuleOverlap(
            scalarFace,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One));
        Assert.False(faceTriangle.TryGetCenteredCapsuleContact(
            scalarFace,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Vector3d.Right,
            out _,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void CenteredCapsuleTriangleRelations_RejectInvalidAuthoredGeometry()
    {
        FixedTriangle triangle = new(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up);

        Assert.Throws<ArgumentException>(() =>
            triangle.DoesCenteredCapsuleOverlap(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.DoesCenteredCapsuleOverlap(
                Vector3d.Zero,
                Vector3d.Up,
                -Fixed64.One,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.DoesCenteredCapsuleOverlap(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One));
        Assert.Throws<ArgumentException>(() =>
            triangle.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                out _,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Up,
                out _,
                out _,
                out _,
                out _));
    }
}
