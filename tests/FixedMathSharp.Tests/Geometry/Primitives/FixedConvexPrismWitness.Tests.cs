using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismWitnessTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void PointFootprint_SelectsTheRadialExitForAnOverlappingVerticalCore(bool capsule, bool scalarLimit)
    {
        Vector2d[] offsets = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Fixed64 radius = scalarLimit ? Fixed64.MaxValue : Fixed64.Quarter;
        Fixed64 length = capsule ? Fixed64.One : Fixed64.Zero;
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up, length, radius, Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out var contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(radius, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Right * radius, first);
        Assert.Equal(Vector3d.Zero, second);
        Vector3d boundary = Vector3d.Left * radius;
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(boundary, FixedQuaternion.Identity,
            Vector3d.Up, length, radius, Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out var touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        boundary.X = Fixed64.FromRaw(boundary.X.m_rawValue - 1);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(boundary, FixedQuaternion.Identity,
            Vector3d.Up, length, radius, Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out _));
    }

    [Fact]
    public void PointFootprint_RejectsAnObliqueCoreAtItsFinitePerpendicularBound()
    {
        Vector2d[] offsets = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Vector3d axis = new Vector3d(1, 0, 1).Normalized;
        Fixed64 radius = Fixed64.FromFraction(129, 100);
        Assert.True((Fixed64)(-2) + axis.X + radius < Fixed64.Zero);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(new Vector3d(-2, 0, -1),
            FixedQuaternion.Identity, axis, Fixed64.Two, radius, Vector3d.Zero,
            Fixed64.Zero, offsets, (Fixed64)100, out _));
    }

    [Fact]
    public void LargeRadiusDiagonalSupport_ScalesItsExactDirectionBeforeRounding()
    {
        Vector2d[] offsets = { new(1, 1), new(1, 1), new(1, 1) };
        const long coordinate = 6_521_908_912_666_391_105;
        BigInteger twiceSquare = 2 * (BigInteger)long.MaxValue * long.MaxValue;
        BigInteger twiceCoordinate = 2 * (BigInteger)coordinate;
        Assert.True((twiceCoordinate - 1) * (twiceCoordinate - 1) < twiceSquare);
        Assert.True(twiceSquare < (twiceCoordinate + 1) * (twiceCoordinate + 1));
        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(Vector3d.Zero, Fixed64.MaxValue,
            Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out var contact));
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(coordinate), Fixed64.Zero, Fixed64.FromRaw(coordinate)), first);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(1, 0, 1), second);
    }

    [Fact]
    public void ObliqueCoreAtFiniteVertex_RanksInteriorAndEndpointEdgeFeetExactly()
    {
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Vector3d center = new(-1, -1, -2), origin = new(1, 2, 1);
        Vector3d axis = new Vector3d(1, 1, -1).Normalized;
        // n=(1,1,2) is perpendicular to the core and nonnegative on the cube.
        // Its bound sqrt(6) is attained at core midpoint / prism vertex zero.
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, FixedQuaternion.Identity,
            axis, (Fixed64)10, (Fixed64)3, origin, Fixed64.Zero, offsets, Fixed64.Two, out var contact));
        Assert.Equal((Fixed64)3 - FixedMath.Sqrt((Fixed64)6), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Zero, second);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        // Independent nearest-even half-cell for sqrt(3/2), rather than a
        // scalar square-root approximation as the exact witness oracle.
        const long root = 5_260_239_169;
        BigInteger scaled = 6 * (BigInteger)Fixed64.One.m_rawValue * Fixed64.One.m_rawValue;
        Assert.True((2 * (BigInteger)root - 1) * (2 * (BigInteger)root - 1) < scaled);
        Assert.True(scaled < (2 * (BigInteger)root + 1) * (2 * (BigInteger)root + 1));
        Fixed64 xy = Fixed64.FromRaw(root) - Fixed64.One;
        Assert.Equal(new Vector3d(xy, xy, FixedMath.Sqrt((Fixed64)6) - Fixed64.Two), first);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            center - contact.Normal * (contact.Depth + Fixed64.FromRaw(64)), FixedQuaternion.Identity,
            axis, (Fixed64)10, (Fixed64)3, origin, Fixed64.Zero, offsets, Fixed64.Two, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NativeYawRim_RetainsTranslationAndZeroCoreParityAtScalarFaces(bool rotatedZeroCore)
    {
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Fixed64 yaw = Fixed64.FromFraction(1, 3);
        Vector3d center = new(-Fixed64.Two * FixedMath.Cos(yaw), -Fixed64.Two, -Fixed64.Two * FixedMath.Sin(yaw));
        Vector3d shift = new(Fixed64.MaxValue - (Fixed64)8, Fixed64.MaxValue - (Fixed64)8, Fixed64.MaxValue - (Fixed64)8);
        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(center, Fixed64.Two, Vector3d.Zero,
            yaw, offsets, Fixed64.One, out var expected));
        FixedQuaternion rotation = rotatedZeroCore
            ? FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)22, (Fixed64)17, (Fixed64)9)
            : FixedQuaternion.Identity;
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center + shift, rotation,
            Vector3d.Up, Fixed64.Zero, Fixed64.Two, shift, yaw, offsets, Fixed64.One, out var actual));
        Assert.Equal(expected.Normal, actual.Normal);
        Assert.Equal(expected.Depth, actual.Depth);
        Assert.False(actual.DepthIsClamped);
        Assert.Equal(center + shift, actual.FirstAnchor.Origin);
        Assert.Equal(rotation, actual.FirstAnchor.Rotation);
        Assert.Equal(shift, actual.SecondAnchor.Origin);
        Assert.Equal(expected.SecondAnchor.Rotation, actual.SecondAnchor.Rotation);
        Assert.Equal(expected.SecondAnchor.LocalPoint, actual.SecondAnchor.LocalPoint);
        Assert.Equal(expected.SecondAnchor.LocalDisplacement, actual.SecondAnchor.LocalDisplacement);
        Assert.True(actual.FirstAnchor.TryGetOffsetFrom(actual.SecondAnchor, out Vector3d difference));
        Vector3d ranked = actual.Normal * actual.Depth;
        Fixed64 bound = Fixed64.FromRaw(8);
        for (int axis = 0; axis < 3; axis++)
            FixedMathTestHelper.AssertWithinRange(difference[axis], ranked[axis] - bound, ranked[axis] + bound);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            center + shift - actual.Normal * (actual.Depth + Fixed64.FromRaw(64)), rotation, Vector3d.Up,
            Fixed64.Zero, Fixed64.Two, shift, yaw, offsets, Fixed64.One, out _));
    }

    [Fact]
    public void LargeFiniteFace_RejectsAnObliqueCoreWhoseCompleteRadiusMissesIt()
    {
        Vector2d[] offsets = { new(-1, -100), new(1, -100), new(1, 100), new(-1, 100) };
        Vector3d axis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero);
        // Every core point plus its radius remains at negative X; the large
        // tangential face extent must not hide that exact separating plane.
        Assert.True(-Fixed64.One + axis.X + Fixed64.Quarter < Fixed64.Zero);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(Vector3d.Left, FixedQuaternion.Identity,
            axis, Fixed64.Two, Fixed64.Quarter, Vector3d.Right, Fixed64.Zero, offsets, (Fixed64)100, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapProjectionAtAnOddCoreEndpoint_RetainsTheHalfRawFeature(bool reverseCore)
    {
        Fixed64 rawFive = Fixed64.FromRaw(5);
        Vector2d[] offsets = { new(Fixed64.Zero, rawFive), new(rawFive, -rawFive), new(1, 1) };
        // The quaternion's exact 2:1 ratio maps local +X to (3/5,0,4/5).
        // The odd five-raw core ends at (1.5,0,2) raw, on 2X+Z=5 raw.
        Fixed64 q = Fixed64.FromRaw(1_920_767_767);
        FixedQuaternion rotation = new(Fixed64.Zero, -q, Fixed64.Zero, q + q);
        Vector3d center = Vector3d.Up * Fixed64.FromFraction(9, 8);
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, rotation,
            reverseCore ? Vector3d.Left : Vector3d.Right, rawFive, Fixed64.Quarter,
            Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out var contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 8), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(2), Fixed64.FromFraction(7, 8), Fixed64.FromRaw(2)), first);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(2), Fixed64.One, Fixed64.FromRaw(2)), second);
        Assert.Equal(contact.Normal * contact.Depth, first - second);
    }

    [Fact]
    public void SphereAtHorizontalRim_UsesTheFiniteEdgeRatherThanAnUnpairedVertexAxis()
    {
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Vector3d center = new(-1, -1, 1), origin = new(1, 1, 1);
        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(center, Fixed64.Two,
            origin, Fixed64.Zero, offsets, Fixed64.One, out var contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(0, 0, 1), second);
        Fixed64 invRootTwo = FixedMath.Sqrt(Fixed64.Half);
        FixedMathTestHelper.AssertWithinRelativeTolerance(invRootTwo, contact.Normal.X, Fixed64.FromRaw(4));
        FixedMathTestHelper.AssertWithinRelativeTolerance(invRootTwo, contact.Normal.Y, Fixed64.FromRaw(4));
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.Two - FixedMath.Sqrt(Fixed64.Two),
            contact.Depth, Fixed64.FromRaw(4));
        Assert.False(FixedConvexPrismRelations.TryGetSphereContact(center, Fixed64.FromFraction(5, 4),
            origin, Fixed64.Zero, offsets, Fixed64.One, out _));
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    public void HorizontalRim_ThreeFourFiveSupportPreservesOneRawTangency(bool capsule, int radiusRawOffset)
    {
        Vector2d[] offsets = { new(-1, -1), new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Vector3d center = new(-3, -4, 1), origin = new(1, 1, 1);
        Fixed64 radius = (Fixed64)5 + Fixed64.FromRaw(radiusRawOffset);
        bool hit = FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, FixedQuaternion.Identity,
            Vector3d.Forward, capsule ? Fixed64.One : Fixed64.Zero, radius, origin,
            Fixed64.Zero, offsets, Fixed64.One, out var contact);
        Assert.Equal(radiusRawOffset >= 0, hit);
        if (!hit) return;
        Assert.Equal(Fixed64.FromRaw(radiusRawOffset), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(0, 0, 1), second);
        if (radiusRawOffset == 0)
        {
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.Equal(second, first);
        }
    }

    [Fact]
    public void MatchedSupportsAndFiniteRimQueries_DoNotAllocateAfterWarmup()
    {
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            Assert.True(FixedConvexPrismRelations.TryGetSphereContact(Vector3d.Zero, Fixed64.FromFraction(3, 4),
                Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.Two, out _));
            Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(Vector3d.Up * Fixed64.Half,
                FixedQuaternion.Identity, Vector3d.Right, (Fixed64)10, Fixed64.Quarter,
                Vector3d.Right * Fixed64.Two, Fixed64.Zero, offsets, Fixed64.One, out _));
            Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(new Vector3d(-3, -4, 1),
                FixedQuaternion.Identity, Vector3d.Forward, Fixed64.One, (Fixed64)5,
                new Vector3d(1, 1, 1), Fixed64.Zero, offsets, Fixed64.One, out _));
        });
        Assert.Equal(0, allocated);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CrossingCore_RetainsNativeYawAndFramesBeyondTheScalarDomain(bool reverseCore)
    {
        Vector2d[] offsets = { new(-Fixed64.Half, -Fixed64.One), new(Fixed64.Half, -Fixed64.One),
            new(Fixed64.Half, Fixed64.One), new(-Fixed64.Half, Fixed64.One) };
        Fixed64 yaw = Fixed64.FromFraction(1, 3);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw);
        Vector3d origin = new(Fixed64.MaxValue - (Fixed64)4, Fixed64.MaxValue - Fixed64.Half, Fixed64.MaxValue);
        Vector3d center = origin + Vector3d.Up * Fixed64.Half;
        Vector3d shiftedOrigin = origin - new Vector3d(FixedMath.Cos(yaw), Fixed64.Zero, FixedMath.Sin(yaw))
            * Fixed64.FromFraction(5, 2);
        // Place the finite face to the left of this long core. Local coordinates
        // stay small even though the retained world supports may exceed MaxValue.
        Vector3d axis = reverseCore ? Vector3d.Left : Vector3d.Right;
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, rotation, axis,
            (Fixed64)10, Fixed64.Quarter, shiftedOrigin, yaw, offsets, Fixed64.One, out var contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.Equal(center, contact.FirstAnchor.Origin);
        Assert.Equal(rotation, contact.FirstAnchor.Rotation);
        Assert.Equal(shiftedOrigin, contact.SecondAnchor.Origin);
        Assert.Equal(rotation, contact.SecondAnchor.Rotation);
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d delta));
        Fixed64 bound = Fixed64.FromRaw(8);
        FixedMathTestHelper.AssertWithinRange(delta.X, -bound, bound);
        FixedMathTestHelper.AssertWithinRange(delta.Y, -contact.Depth - bound, -contact.Depth + bound);
        FixedMathTestHelper.AssertWithinRange(delta.Z, -bound, bound);
        Assert.True(contact.SecondAnchor.TryGetLocalPointIn(shiftedOrigin, rotation, out Vector3d local));
        Assert.Equal(Fixed64.One, local.Y);
        FixedMathTestHelper.AssertWithinRange(local.X, -Fixed64.Half - bound, Fixed64.Half + bound);
        FixedMathTestHelper.AssertWithinRange(local.Z, -Fixed64.One - bound, Fixed64.One + bound);
    }

    [Fact]
    public void ObliqueCapsule_PrismAndBoxAgreeOnFiniteContactsAndWholeShapeClearance()
    {
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)22, (Fixed64)17, (Fixed64)9);
        Fixed64 bound = Fixed64.FromRaw(8);
        for (int x = -2; x <= 2; x++)
        for (int y = -2; y <= 2; y++)
        for (int z = -2; z <= 2; z++)
        {
            Vector3d center = new Vector3d(x, y, z) * Fixed64.FromFraction(3, 4);
            bool hit = FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, rotation, Vector3d.Up,
                Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out var actual);
            bool expectedHit = box.TryGetCenteredCapsuleContact(center, rotation, Vector3d.Up,
                Fixed64.Two, Fixed64.Half, out var expected);
            Assert.True(hit == expectedHit, $"Classification differs at {center}");
            if (!hit) continue;
            Assert.Equal(expected.Depth, actual.Depth);
            Assert.False(actual.DepthIsClamped);
            Assert.True(actual.FirstAnchor.TryGetOffsetFrom(actual.SecondAnchor, out Vector3d delta));
            Vector3d ranked = actual.Normal * actual.Depth;
            // Two rounded anchors and the rounded direction/depth contribute at
            // most eight raw units per component for this bounded unit-box fixture.
            for (int axis = 0; axis < 3; axis++)
                FixedMathTestHelper.AssertWithinRange(delta[axis], ranked[axis] - bound, ranked[axis] + bound);
            Vector3d boundary = center - actual.Normal * actual.Depth;
            bool touching = FixedConvexPrismRelations.TryGetCenteredCapsuleContact(boundary, rotation, Vector3d.Up,
                Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out var boundaryContact);
            Assert.True(!touching || boundaryContact.Depth <= bound);
            Assert.False(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
                center - actual.Normal * (actual.Depth + Fixed64.FromRaw(64)), rotation, Vector3d.Up,
                Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out _));
        }
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(-1, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    public void CrossingCapsule_MatchesTheOpposingFiniteCap(int sign, bool reverseCore)
    {
        Vector2d[] offsets = { new(-Fixed64.Half, -Fixed64.One), new(Fixed64.Half, -Fixed64.One),
            new(Fixed64.Half, Fixed64.One), new(-Fixed64.Half, Fixed64.One) };
        Vector3d center = Vector3d.Up * Fixed64.Half;
        Vector3d origin = Vector3d.Right * Fixed64.FromFraction(5 * sign, 2);
        Vector3d axis = reverseCore ? Vector3d.Left : Vector3d.Right;
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, FixedQuaternion.Identity,
            axis, (Fixed64)10, Fixed64.Quarter, origin, Fixed64.Zero, offsets, Fixed64.One, out var contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d((Fixed64)(2 * sign), Fixed64.Quarter, Fixed64.Zero), first);
        Assert.Equal(new Vector3d((Fixed64)(2 * sign), Fixed64.One, Fixed64.Zero), second);
        Assert.Equal(contact.Normal * contact.Depth, first - second);
        // The core crosses X=[2,3] (or its mirror); Y attains the global 3/4 bound.
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center + Vector3d.Up * contact.Depth,
            FixedQuaternion.Identity, axis, (Fixed64)10, Fixed64.Quarter, origin, Fixed64.Zero,
            offsets, Fixed64.One, out var touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            center + Vector3d.Up * (contact.Depth + Fixed64.MinIncrement), FixedQuaternion.Identity,
            axis, (Fixed64)10, Fixed64.Quarter, origin, Fixed64.Zero, offsets, Fixed64.One, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedSphere_MatchesTheSelectedOpposingFace(bool reverseWinding)
    {
        Vector2d[] offsets = { new(-1, -3), new(1, -3), new(1, 3), new(-1, 3) };
        if (reverseWinding) Array.Reverse(offsets);
        Fixed64 radius = Fixed64.FromFraction(3, 4);
        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(Vector3d.Zero, radius,
            Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.Two, out var contact));
        Assert.Equal(Fixed64.FromFraction(7, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.One, contact.Normal.X.Abs());
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(contact.Normal * radius, first);
        Assert.Equal(-contact.Normal, second);
        Assert.Equal(contact.Normal * contact.Depth, first - second);
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, radius, Vector3d.Zero,
            Fixed64.Zero, offsets, Fixed64.Two, out var zeroCore));
        Assert.Equal(contact, zeroCore);
        // The prism contains a centered unit ball; its X support attains 1+3/4.
        Assert.True(FixedConvexPrismRelations.TryGetSphereContact(-contact.Normal * contact.Depth, radius,
            Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.Two, out var touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Assert.False(FixedConvexPrismRelations.TryGetSphereContact(
            -contact.Normal * (contact.Depth + Fixed64.MinIncrement), radius,
            Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.Two, out _));
    }

    [Fact]
    public void ScalarLimitCapsule_RetainsDistinctOpposingSupportsBeforeDepthClamping()
    {
        Fixed64 m = Fixed64.MaxValue;
        Vector2d[] offsets = { new(-m, -m), new(m, -m), new(m, m), new(-m, m) };
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Up, m, m, Vector3d.Zero, Fixed64.Zero,
            offsets, m, out var contact));
        Assert.True(contact.DepthIsClamped);
        Assert.Equal(m, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(contact.Normal * m, first);
        Assert.Equal(-contact.Normal * m, second);
        Assert.False(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out _));
    }
}
