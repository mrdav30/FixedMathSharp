//=======================================================================
// FixedTriangle.RelativeClosestPoint.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedTriangleRelativeClosestPointTests
{
    [Fact]
    public void ClosestPointAnchor_OrdinaryInputMatchesLocalTriangleContract()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-2, -1, 0),
            new Vector3d(3, -1, 0),
            new Vector3d(-2, 4, 0));
        Vector3d localPoint = new(
            Fixed64.Half,
            Fixed64.Half,
            (Fixed64)3);
        Vector3d origin = new(7, -5, 11);
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)(-23),
                (Fixed64)31);
        var point = new FixedPointAnchor(
            origin,
            rotation,
            localPoint);

        FixedPointAnchor closest =
            triangle.GetClosestPointAnchor(origin, rotation, point);

        Assert.Equal(
            triangle.ClosestPoint(localPoint),
            closest.LocalPoint);
        Assert.Equal(origin, closest.Origin);
        Assert.Equal(rotation, closest.Rotation);
    }

    [Theory]
    [InlineData(3, -1, 2, 0)]
    [InlineData(1, -1, 1, 0)]
    [InlineData(-1, 3, 0, 2)]
    [InlineData(-1, 1, 0, 1)]
    public void ClosestPointAnchor_SelectsEveryOuterVertexAndEdgeRegion(
        int queryX,
        int queryY,
        int expectedX,
        int expectedY)
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(2, 0, 0),
            new Vector3d(0, 2, 0));
        var point = new FixedPointAnchor(
            new Vector3d(queryX, queryY, 0),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        FixedPointAnchor closest = triangle.GetClosestPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            point);

        Assert.Equal(
            new Vector3d(expectedX, expectedY, 0),
            closest.LocalPoint);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ClosestPointAnchor_CancelsMirroredScalarFaceOrigins(
        bool positive)
    {
        Fixed64 edge = positive
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Vector3d origin = new(edge, edge, edge);
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)11,
                (Fixed64)(-19),
                (Fixed64)7);
        var triangle = new FixedTriangle(
            new Vector3d(-1, -1, 0),
            new Vector3d(1, -1, 0),
            new Vector3d(-1, 1, 0));
        var point = new FixedPointAnchor(
            origin,
            rotation,
            new Vector3d(
                -Fixed64.Quarter,
                -Fixed64.Quarter,
                (Fixed64)2));

        FixedPointAnchor closest =
            triangle.GetClosestPointAnchor(origin, rotation, point);

        Assert.Equal(
            new Vector3d(
                -Fixed64.Quarter,
                -Fixed64.Quarter,
                Fixed64.Zero),
            closest.LocalPoint);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ClosestPointAnchor_RetainsQueryBeyondLocalScalarDomain(
        bool positive)
    {
        var triangle = new FixedTriangle(
            new Vector3d(-Fixed64.Half, -Fixed64.Half, Fixed64.Zero),
            new Vector3d(Fixed64.Half, -Fixed64.Half, Fixed64.Zero),
            new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Zero));
        Vector3d query = new(
            positive ? Fixed64.MaxValue : Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);
        var point = new FixedPointAnchor(
            query,
            FixedQuaternion.Identity,
            Vector3d.Zero);

        FixedPointAnchor closest =
            triangle.GetClosestPointAnchor(
                new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero),
                FixedQuaternion.Identity,
                point);

        Assert.Equal(
            positive
                ? new Vector3d(
                    Fixed64.Half,
                    -Fixed64.Half,
                    Fixed64.Zero)
                : new Vector3d(
                    -Fixed64.Half,
                    -Fixed64.Half,
                    Fixed64.Zero),
            closest.LocalPoint);
    }

    [Fact]
    public void ClosestPointAnchor_DegenerateTriangleMatchesEdgeOrder()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-2, 0, 0),
            Vector3d.Zero,
            new Vector3d(2, 0, 0));
        Vector3d pointValue = new(
            Fixed64.Half,
            (Fixed64)3,
            Fixed64.Zero);
        var point = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            pointValue);

        FixedPointAnchor closest =
            triangle.GetClosestPointAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                point);

        Assert.Equal(triangle.ClosestPoint(pointValue), closest.LocalPoint);
    }

    [Fact]
    public void ClosestPointAnchor_DegenerateTriangleHandlesPointAndBeforeStartRegions()
    {
        var pointTriangle = new FixedTriangle(
            Vector3d.One,
            Vector3d.One,
            Vector3d.One);
        var point = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(-1, 3, 0));

        FixedPointAnchor pointClosest = pointTriangle.GetClosestPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            point);

        Assert.Equal(Vector3d.One, pointClosest.LocalPoint);

        var lineTriangle = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(2, 0, 0),
            new Vector3d(4, 0, 0));
        FixedPointAnchor beforeStart = lineTriangle.GetClosestPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new FixedPointAnchor(
                new Vector3d(-1, 0, 0),
                FixedQuaternion.Identity,
                Vector3d.Zero));

        Assert.Equal(Vector3d.Zero, beforeStart.LocalPoint);
    }

    [Fact]
    public void ClosestPointAnchor_ValidatesBothRigidFrames()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up);
        var point = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.Throws<ArgumentException>(() =>
            triangle.GetClosestPointAnchor(
                Vector3d.Zero,
                default,
                point));
        Assert.Throws<ArgumentException>(() =>
            triangle.GetClosestPointAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                default));
    }

    [Fact]
    public void ClosestPointAnchor_WarmedPathDoesNotAllocate()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-1, -1, 0),
            new Vector3d(1, -1, 0),
            new Vector3d(-1, 1, 0));
        var point = new FixedPointAnchor(
            new Vector3d(7, -5, 3),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.Quarter,
                Fixed64.Quarter,
                (Fixed64)2));
        _ = triangle.GetClosestPointAnchor(
            new Vector3d(7, -5, 3),
            FixedQuaternion.Identity,
            point);
        long before = GC.GetAllocatedBytesForCurrentThread();

        for (int iteration = 0; iteration < 64; iteration++)
        {
            _ = triangle.GetClosestPointAnchor(
                new Vector3d(7, -5, 3),
                FixedQuaternion.Identity,
                point);
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }
}
