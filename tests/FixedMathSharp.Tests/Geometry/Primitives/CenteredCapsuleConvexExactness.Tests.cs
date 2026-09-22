//=======================================================================
// CenteredCapsuleConvexExactness.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCapsuleConvexExactnessTests
{
    private static readonly Vector2d[] Corner =
    {
        new(-1, -1), new(0, -1), new(0, 0), new(-1, 0),
    };

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void Contacts_CornerClassificationDistinguishesOneRawRadiusStep(long radiusStep)
    {
        foreach (int scale in new[] { 1, 10, 1000, 100000000 })
        foreach (int degrees in new[] { 0, 30 })
        {
            Fixed64 rotation = FixedMath.DegToRad((Fixed64)degrees);
            Fixed64 radius = Fixed64.FromRaw(((Fixed64)(5 * scale)).m_rawValue + radiusStep);
            bool hit = FixedConvex2dRelations.TryGetCircleContact(
                new Vector2d(3 * scale, 4 * scale), rotation, radius,
                Vector2d.Zero, rotation, Corner,
                out _, out FixedPointAnchor2d convexContact,
                out Vector2d normal, out Fixed64 depth, out bool clamped);

            Assert.Equal(radiusStep >= 0, hit);
            Assert.False(clamped);
            Assert.Equal(Fixed64.FromRaw(Math.Max(radiusStep, 0)), depth);
            Assert.Equal(radiusStep >= 0
                ? new Vector2d(Fixed64.FromFraction(-3, 5), Fixed64.FromFraction(-4, 5))
                : Vector2d.Zero, normal);
            Assert.Equal(Vector2d.Zero, convexContact.LocalPoint);
        }
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void MinimumTranslation_ObliqueFaceClassifiesBeforeNormalRounding(long radiusStep)
    {
        // The supporting face lies on 3*x + 4*y = 0; its nearest point is (0,0).
        Vector2d[] triangle = { new(4, -3), new(-4, 3), new(-4, -3) };
        bool hit = FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            new Vector2d(3, 4), Vector2d.Forward, Fixed64.Zero,
            Fixed64.FromRaw(((Fixed64)5).m_rawValue + radiusStep),
            Vector2d.Zero, triangle, out Vector2d normal, out Fixed64 depth);

        Assert.Equal(radiusStep >= 0, hit);
        Assert.Equal(Fixed64.FromRaw(Math.Max(radiusStep, 0)), depth);
        Assert.Equal(radiusStep >= 0
            ? new Vector2d(Fixed64.FromFraction(-3, 5), Fixed64.FromFraction(-4, 5))
            : Vector2d.Zero, normal);
    }

    [Theory]
    [InlineData(-30)]
    [InlineData(0)]
    [InlineData(30)]
    public void Contacts_RotatedCapsuleRetainsExactEndpointTangency(int degrees)
    {
        Fixed64 rotation = FixedMath.DegToRad((Fixed64)degrees);
        Vector2d direction = new(-FixedMath.Sin(rotation), FixedMath.Cos(rotation));
        Vector2d center = new Vector2d(3, 4) + direction;
        Span<FixedPointAnchor2d> capsuleContacts = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts = stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            center, rotation, Vector2d.Forward, Fixed64.Two, (Fixed64)5,
            Vector2d.Zero, Fixed64.Zero, Corner, capsuleContacts, convexContacts,
            out int count, out Vector2d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(1, count);
        Assert.Equal(Vector2d.Zero, convexContacts[0].LocalPoint);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(clamped);
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            center, rotation, Fixed64.Two, (Fixed64)5, Vector2d.Zero, Corner,
            out Vector2d rotationNormal, out Fixed64 rotationDepth));
        Assert.Equal(normal, rotationNormal);
        Assert.Equal(depth, rotationDepth);
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            center, direction, Fixed64.Two, (Fixed64)5, Vector2d.Zero, Corner,
            out Vector2d directionNormal, out Fixed64 directionDepth));
        Assert.Equal(normal, directionNormal);
        Assert.Equal(depth, directionDepth);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Contacts_TangencySurvivesUnrepresentablePolygonVertices(bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        int sign = maximumFace ? 1 : -1;
        Vector2d origin = new(face, face);
        Vector2d center = origin - new Vector2d(sign * 3, sign * 4);
        Vector2d[] polygon = { new(0, 0), new(sign, 0), new(sign, sign), new(0, sign) };

        Assert.True(FixedConvex2dRelations.TryGetCircleContact(
            center, Fixed64.Zero, (Fixed64)5, origin, Fixed64.Zero, polygon,
            out _, out FixedPointAnchor2d contact, out Vector2d normal,
            out Fixed64 depth, out bool clamped));
        Assert.Equal(Vector2d.Zero, contact.LocalPoint);
        Assert.Equal(new Vector2d(Fixed64.FromFraction(sign * 3, 5), Fixed64.FromFraction(sign * 4, 5)), normal);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(clamped);
    }

    [Fact]
    public void Sweep_InitialCornerTangencyReturnsZeroDistanceEvenWhenMovingAway()
    {
        Assert.True(FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(
            new Vector2d(3, 4), Vector2d.Forward, Fixed64.Zero, (Fixed64)5,
            Vector2d.Right, Fixed64.One, Vector2d.Zero, Corner,
            out Fixed64 distance, out Vector2d normal, out Vector2d contact));
        Assert.Equal(Fixed64.Zero, distance);
        Assert.Equal(Vector2d.Zero, contact);
        Assert.True(FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(
            new Vector2d(3, 4), Vector2d.Forward, Fixed64.Zero, (Fixed64)5,
            Vector2d.Right, Fixed64.One, Vector2d.Zero, Fixed64.Pi / (Fixed64)6, Corner,
            out Fixed64 rotatedDistance, out Vector2d rotatedNormal, out FixedPointAnchor2d rotatedContact));
        Assert.Equal(Fixed64.Zero, rotatedDistance);
        Assert.Equal(normal, rotatedNormal);
        Assert.Equal(Vector2d.Zero, rotatedContact.LocalPoint);
    }

    [Theory]
    [InlineData(1, 2)]
    [InlineData(2, 2)]
    [InlineData(long.MaxValue - 1, long.MaxValue - 1)]
    [InlineData(long.MaxValue, long.MaxValue)]
    public void Contacts_DepthRoundsOnceWithRadiusIncluded(long radiusRaw, long expectedDepthRaw)
    {
        Vector2d[] polygon = { new(0, -1), new(1, -1), new(1, 1), new(0, 1) };
        Span<FixedPointAnchor2d> capsule = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convex = stackalloc FixedPointAnchor2d[2];

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.MinIncrement,
            Fixed64.FromRaw(radiusRaw), Vector2d.Zero, Fixed64.Zero, polygon,
            capsule, convex, out _, out Vector2d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Vector2d.Right, normal);
        Assert.Equal(expectedDepthRaw, depth.m_rawValue);
        Assert.Equal(radiusRaw == long.MaxValue, clamped);
    }

    [Fact]
    public void Contacts_ExactlyMaximumDepthIsNotClamped()
    {
        Assert.True(FixedConvex2dRelations.TryGetCircleContact(
            Vector2d.Zero, Fixed64.Zero, Fixed64.MaxValue,
            Vector2d.Zero, Fixed64.Zero, Corner, out _, out _, out _,
            out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.False(clamped);
    }

    [Fact]
    public void Contacts_NegativeAxialOverlapRoundsRadiusInclusiveMidpointToEven()
    {
        Span<FixedPointAnchor2d> capsule = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convex = stackalloc FixedPointAnchor2d[2];
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            new Vector2d(Fixed64.MinIncrement, -Fixed64.Half), Fixed64.Zero,
            Vector2d.Right, Fixed64.MinIncrement, Fixed64.FromRaw(3),
            Vector2d.Zero, Fixed64.Zero, Corner, capsule, convex,
            out _, out _, out Fixed64 depth, out bool clamped));
        // Radius 3 raw minus an axial gap of 1/2 raw is 2.5 raw, which rounds to 2.
        Assert.Equal(2L, depth.m_rawValue);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(4294967296000000004L, 3149371036902676416L)]
    [InlineData(4294967296000000011L, 3149371036902676407L)]
    public void MinimumTranslation_TinyObliqueFaceRefinesIrrationalDepth(long centerRaw, long expectedRaw)
    {
        Fixed64 step = Fixed64.MinIncrement;
        Vector2d[] triangle = { new(-step, step), new(step, -step), new(-step, -step) };
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            new Vector2d(Fixed64.FromRaw(centerRaw), Fixed64.FromRaw(centerRaw)),
            Vector2d.Forward, Fixed64.Zero, Fixed64.MaxValue,
            Vector2d.Zero, triangle, out _, out Fixed64 depth));
        // Nearest-even rounding of long.MaxValue - centerRaw*sqrt(2).
        // These straddle opposite sides of the midpoint between the root bounds.
        Assert.Equal(expectedRaw, depth.m_rawValue);
    }

    [Fact]
    public void MinimumTranslation_IncludesTheCapsuleSideAxis()
    {
        Vector2d[] polygon =
        {
            new(0, -1), new(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(1, 4)),
            new(0, 1), new(Fixed64.FromFraction(-3, 2), Fixed64.FromFraction(-1, 4)),
        };
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            Vector2d.Zero, Vector2d.Right, Fixed64.Two, Fixed64.Half,
            Vector2d.Zero, polygon, out Vector2d normal, out Fixed64 depth));
        Assert.Equal(Fixed64.FromFraction(3, 2), depth);
        Assert.Equal(Vector2d.Forward, normal);
    }

    [Fact]
    public void Contacts_EdgeAxisCanSelectAnOppositeVertexSupport()
    {
        Vector2d[] triangle = { new(-1, -1), new(1, -1), new(0, 1) };
        Assert.True(FixedConvex2dRelations.TryGetCircleContact(
            new Vector2d(0, 2), Fixed64.Zero, Fixed64.Two,
            Vector2d.Zero, Fixed64.Zero, triangle,
            out _, out FixedPointAnchor2d convexContact,
            out Vector2d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Vector2d.Forward, convexContact.LocalPoint);
        Assert.Equal(Vector2d.Backward, normal);
        Assert.Equal(Fixed64.One, depth);
        Assert.False(clamped);
    }

    [Fact]
    public void MinimumTranslation_IrrationalDepthRoundsToNearestRawValue()
    {
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
            new Vector2d(1, 1), Vector2d.Right, Fixed64.Zero, Fixed64.Two,
            Vector2d.Zero, Corner, out _, out Fixed64 depth));
        // round((2 - sqrt(2)) * 2^32), derived independently of FixedMath.Sqrt.
        Assert.Equal(2515933592L, depth.m_rawValue);
    }

    [Fact]
    public void MinimumTranslation_MatchesExactSegmentDistanceOracle()
    {
        Vector2d[] axes =
        {
            Vector2d.Right, Vector2d.Forward,
            new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)),
            new(Fixed64.FromFraction(-4, 5), Fixed64.FromFraction(3, 5)),
        };
        System.Random random = new(20092026);
        for (int i = 0; i < 1200; i++)
        {
            Vector2d axis = axes[i % axes.Length];
            Fixed64 rotation = FixedMath.DegToRad((Fixed64)((i % 7) * 30));
            Fixed64 length = Fixed64.FromFraction(random.Next(0, 33), 4);
            Fixed64 radius = Fixed64.FromFraction(random.Next(0, 21), 4);
            Vector2d center = new(Fixed64.FromFraction(random.Next(-40, 41), 4), Fixed64.FromFraction(random.Next(-40, 41), 4));
            Vector2d origin = new(Fixed64.FromFraction(random.Next(-8, 9), 4), Fixed64.FromFraction(random.Next(-8, 9), 4));
            bool expected = IntersectsBySegmentDistances(center, axis, length, radius, origin, rotation, Corner);
            bool actual = FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                center, axis, length, radius, origin, rotation, Corner, out Vector2d normal, out Fixed64 depth);
            Assert.True(expected == actual, $"Fixture {i}: center={center}, axis={axis}, length={length}, radius={radius}, origin={origin}, rotation={rotation}");
            if (actual)
            {
                Assert.True(normal.IsNormalized());
                Assert.True(depth >= Fixed64.Zero);
            }
            else
            {
                Assert.Equal(Vector2d.Zero, normal);
                Assert.Equal(Fixed64.Zero, depth);
            }
        }
    }

    [Fact]
    public void MinimumTranslation_FullDomainRotatedInputsMatchExactOracle()
    {
        Fixed64 maximum = Fixed64.MaxValue;
        Fixed64 minimum = Fixed64.MinValue;
        Vector2d[][] polygons =
        {
            Corner,
            new Vector2d[]
            {
                new(minimum, Fixed64.Zero), new(Fixed64.Zero, minimum),
                new(maximum, Fixed64.Zero), new(Fixed64.Zero, maximum),
            },
        };
        Vector2d[] centers = { Vector2d.Zero, new(maximum, maximum), new(minimum, minimum), new(maximum, minimum) };
        Vector2d[] origins = { Vector2d.Zero, new(minimum, maximum), new(maximum, minimum) };
        Vector2d diagonal = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        foreach (Vector2d[] polygon in polygons)
        foreach (Vector2d center in centers)
        foreach (Vector2d origin in origins)
        foreach (int degrees in new[] { 0, 30, 60 })
        {
            Fixed64 rotation = FixedMath.DegToRad((Fixed64)degrees);
            bool expected = IntersectsBySegmentDistances(
                center, diagonal, maximum, maximum, origin, rotation, polygon);
            bool actual = FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
                center, diagonal, maximum, maximum, origin, rotation, polygon, out _, out _);
            Assert.Equal(expected, actual);
        }
    }

    [Fact]
    public void Contacts_ExactClassificationRemainsAllocationFreeAfterWarmup()
    {
        FixedPointAnchor2d[] capsule = new FixedPointAnchor2d[2];
        FixedPointAnchor2d[] convex = new FixedPointAnchor2d[2];
        bool correct = true;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int i = 0; i < 32; i++)
            {
                correct &= FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                    new Vector2d(3, 4), Fixed64.Zero, Vector2d.Forward, Fixed64.Zero,
                    (Fixed64)5, Vector2d.Zero, Fixed64.Zero, Corner, capsule, convex,
                    out int count, out _, out Fixed64 depth, out bool clamped)
                    && count == 1 && depth == Fixed64.Zero && !clamped;
            }
        });
        Assert.True(correct);
        Assert.Equal(0, allocated);
    }

    // Independent oracle: segment/edge intersection and point-to-segment distances.
    // All coordinates share denominator 2*S^2, retaining half-raw endpoints and
    // exact products of the authored fixed-point rotation coefficients.
    private static bool IntersectsBySegmentDistances(
        Vector2d center, Vector2d axis, Fixed64 length, Fixed64 radius,
        Vector2d origin, Fixed64 rotation, Vector2d[] offsets)
    {
        BigInteger scale = BigInteger.One << 32;
        (BigInteger X, BigInteger Y) start = (
            2 * scale * center.X.m_rawValue - (BigInteger)axis.X.m_rawValue * length.m_rawValue,
            2 * scale * center.Y.m_rawValue - (BigInteger)axis.Y.m_rawValue * length.m_rawValue);
        (BigInteger X, BigInteger Y) end = (
            2 * scale * center.X.m_rawValue + (BigInteger)axis.X.m_rawValue * length.m_rawValue,
            2 * scale * center.Y.m_rawValue + (BigInteger)axis.Y.m_rawValue * length.m_rawValue);
        BigInteger cosine = FixedMath.Cos(rotation).m_rawValue;
        BigInteger sine = FixedMath.Sin(rotation).m_rawValue;
        var polygon = new (BigInteger X, BigInteger Y)[offsets.Length];
        for (int i = 0; i < offsets.Length; i++)
        {
            polygon[i] = (
                2 * (scale * origin.X.m_rawValue + cosine * offsets[i].X.m_rawValue - sine * offsets[i].Y.m_rawValue),
                2 * (scale * origin.Y.m_rawValue + sine * offsets[i].X.m_rawValue + cosine * offsets[i].Y.m_rawValue));
        }
        BigInteger radiusSquared = BigInteger.Pow(2 * scale * radius.m_rawValue, 2);
        bool startInside = true;
        for (int i = 0; i < polygon.Length; i++)
        {
            var a = polygon[i];
            var b = polygon[(i + 1) % polygon.Length];
            BigInteger startSide = Cross(a, b, start);
            BigInteger endSide = Cross(a, b, end);
            startInside &= startSide.Sign >= 0;
            if ((startSide.Sign * endSide.Sign <= 0
                    && Cross(start, end, a).Sign * Cross(start, end, b).Sign <= 0
                    && BigInteger.Max(BigInteger.Min(start.X, end.X), BigInteger.Min(a.X, b.X)) <= BigInteger.Min(BigInteger.Max(start.X, end.X), BigInteger.Max(a.X, b.X))
                    && BigInteger.Max(BigInteger.Min(start.Y, end.Y), BigInteger.Min(a.Y, b.Y)) <= BigInteger.Min(BigInteger.Max(start.Y, end.Y), BigInteger.Max(a.Y, b.Y)))
                || PointWithinRadius(start, a, b, radiusSquared)
                || PointWithinRadius(end, a, b, radiusSquared)
                || PointWithinRadius(a, start, end, radiusSquared))
            {
                return true;
            }
        }
        return startInside;
    }

    private static BigInteger Cross(
        (BigInteger X, BigInteger Y) a, (BigInteger X, BigInteger Y) b, (BigInteger X, BigInteger Y) p) =>
        (b.X - a.X) * (p.Y - a.Y) - (b.Y - a.Y) * (p.X - a.X);

    private static bool PointWithinRadius(
        (BigInteger X, BigInteger Y) point, (BigInteger X, BigInteger Y) a,
        (BigInteger X, BigInteger Y) b, BigInteger radiusSquared)
    {
        BigInteger x = b.X - a.X;
        BigInteger y = b.Y - a.Y;
        BigInteger dx = point.X - a.X;
        BigInteger dy = point.Y - a.Y;
        BigInteger lengthSquared = x * x + y * y;
        BigInteger projection = dx * x + dy * y;
        if (projection <= 0)
            return dx * dx + dy * dy <= radiusSquared;
        if (projection >= lengthSquared)
        {
            dx = point.X - b.X;
            dy = point.Y - b.Y;
            return dx * dx + dy * dy <= radiusSquared;
        }
        return BigInteger.Pow(dx * y - dy * x, 2) <= radiusSquared * lengthSquared;
    }
}
