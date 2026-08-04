//=======================================================================
// FiniteAxisTriangle.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FiniteAxisIntersectionTests
{
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
