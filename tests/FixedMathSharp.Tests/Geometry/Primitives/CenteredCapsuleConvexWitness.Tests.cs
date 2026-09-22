//=======================================================================
// CenteredCapsuleConvexWitness.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCapsuleConvexWitnessTests
{
    [Theory]
    [InlineData(-1, 4)]
    [InlineData(1, 4)]
    [InlineData(-1, 3)]
    [InlineData(1, 3)]
    [InlineData(-2, 4)]
    [InlineData(2, 4)]
    public void Contacts_SideVertexSharesTangentialPosition(int x, int quarterHeight)
    {
        Fixed64 height = Fixed64.FromFraction(quarterHeight, 4);
        Vector2d[] triangle = { new((Fixed64)x, height), new(x + 1, 3), new(x - 1, 3) };
        Span<FixedPointAnchor2d> capsule = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> polygon = stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One,
            Vector2d.Zero, Fixed64.Zero, triangle, capsule, polygon,
            out int count, out Vector2d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(1, count);
        Assert.Equal(Vector2d.Forward, normal);
        Assert.Equal(Fixed64.One - height, depth);
        Assert.False(clamped);
        Assert.True(capsule[0].TryGetPoint(out Vector2d capsulePoint));
        Assert.True(polygon[0].TryGetPoint(out Vector2d polygonPoint));
        Assert.Equal(new Vector2d((Fixed64)x, Fixed64.One), capsulePoint);
        Assert.Equal(new Vector2d((Fixed64)x, height), polygonPoint);
    }

    [Theory]
    [InlineData(-30)]
    [InlineData(30)]
    [InlineData(90)]
    public void Contacts_SideVertexRetainsLocalWitnessUnderRotation(int degrees)
    {
        Fixed64 rotation = FixedMath.DegToRad((Fixed64)degrees);
        Vector2d[] triangle = { new(Fixed64.One, Fixed64.FromFraction(3, 4)), new(2, 3), new(0, 3) };
        Span<FixedPointAnchor2d> capsule = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> polygon = stackalloc FixedPointAnchor2d[2];
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            new Vector2d(20, -30), rotation, Vector2d.Right, (Fixed64)4, Fixed64.One,
            new Vector2d(20, -30), rotation, triangle, capsule, polygon,
            out int count, out _, out _, out _));
        Assert.Equal(1, count);
        // Public normal rotation and axial parameter each round once; allow
        // four raw units for their composed local-coordinate rounding.
        FixedMathTestHelper.AssertWithinRange(capsule[0].LocalPoint.X,
            Fixed64.One - Fixed64.FromRaw(4), Fixed64.One + Fixed64.FromRaw(4));
        Assert.Equal(Fixed64.Zero, capsule[0].LocalPoint.Y);
        Assert.Equal(triangle[0], polygon[0].LocalPoint);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Contacts_SideVertexDoesNotMaterializeUnrepresentableWorldPoints(bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        int sign = maximumFace ? 1 : -1;
        Vector2d origin = new(face, face);
        Vector2d[] triangle = { new(sign, sign), new(2 * sign, 3 * sign), new(0, 3 * sign) };
        Span<FixedPointAnchor2d> capsule = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> polygon = stackalloc FixedPointAnchor2d[2];
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            origin, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One,
            origin, Fixed64.Zero, triangle, capsule, polygon,
            out int count, out _, out Fixed64 depth, out bool clamped));
        Assert.Equal(1, count);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(clamped);
        Assert.False(capsule[0].TryGetPoint(out _));
        Assert.False(polygon[0].TryGetPoint(out _));
        Assert.Equal(0, capsule[0].CompareSquaredDistance(capsule[0], polygon[0]));
    }

    [Fact]
    public void Contacts_SideVertexResolvesAxesShorterThanEpsilon()
    {
        Fixed64 step = Fixed64.MinIncrement;
        Vector2d[] triangle = { new(step, Fixed64.One), new(1, 3), new(-1, 3) };
        Span<FixedPointAnchor2d> capsule = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> polygon = stackalloc FixedPointAnchor2d[2];
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.FromRaw(3), Fixed64.One,
            Vector2d.Zero, Fixed64.Zero, triangle, capsule, polygon,
            out int count, out _, out _, out _));
        Assert.Equal(1, count);
        Assert.Equal(0, capsule[0].CompareSquaredDistance(capsule[0], polygon[0]));
    }

    [Fact]
    public void Contacts_SideVertexProjectsBetweenDifferentOriginsAndFrames()
    {
        Vector2d[] triangle = { new(8, 4), new(10, 3), new(10, 5) };
        Span<FixedPointAnchor2d> capsule = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> polygon = stackalloc FixedPointAnchor2d[2];
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            new Vector2d(10, 20), Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One,
            new Vector2d(15, 13), Fixed64.HalfPi, triangle, capsule, polygon,
            out int count, out Vector2d normal, out Fixed64 depth, out _));
        Assert.Equal(1, count);
        Assert.Equal(Vector2d.Forward, normal);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.True(capsule[0].TryGetPoint(out Vector2d capsulePoint));
        Assert.True(polygon[0].TryGetPoint(out Vector2d polygonPoint));
        Assert.Equal(new Vector2d(11, 21), capsulePoint);
        Assert.Equal(capsulePoint, polygonPoint);
    }

    [Fact]
    public void Contacts_SideWitnessDoesNotAllocateAfterWarmup()
    {
        Vector2d[] triangle = { new(1, 1), new(2, 3), new(0, 3) };
        FixedPointAnchor2d[] capsule = new FixedPointAnchor2d[2];
        FixedPointAnchor2d[] polygon = new FixedPointAnchor2d[2];
        bool correct = true;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            correct &= FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                Vector2d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One,
                Vector2d.Zero, Fixed64.Zero, triangle, capsule, polygon,
                out _, out _, out _, out _)
                && capsule[0].LocalPoint == Vector2d.Right;
        });
        Assert.True(correct);
        Assert.Equal(0, allocated);
    }
}
