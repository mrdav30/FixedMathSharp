using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedConvex2dSweepContractsTests
{
    private static readonly Vector2d[] UnitBox =
    {
        new(-Fixed64.One, -Fixed64.One),
        new(Fixed64.One, -Fixed64.One),
        new(Fixed64.One, Fixed64.One),
        new(-Fixed64.One, Fixed64.One),
    };

    [Theory]
    [InlineData(false, false, false, false)]
    [InlineData(false, false, false, true)]
    [InlineData(false, false, true, false)]
    [InlineData(false, false, true, true)]
    [InlineData(false, true, false, false)]
    [InlineData(false, true, false, true)]
    [InlineData(false, true, true, false)]
    [InlineData(false, true, true, true)]
    [InlineData(true, false, false, false)]
    [InlineData(true, false, false, true)]
    [InlineData(true, false, true, false)]
    [InlineData(true, false, true, true)]
    [InlineData(true, true, false, false)]
    [InlineData(true, true, false, true)]
    [InlineData(true, true, true, false)]
    [InlineData(true, true, true, true)]
    public void CircleAndCapsuleSweeps_KeepEntryNormalWhenPolygonOriginIsRebased(
        bool capsule, bool reverse, bool rotated, bool reversedWinding)
    {
        Vector2d Rotate(Vector2d p) => rotated ? new(-p.Y, p.X) : p;
        Vector2d origin = new(7, -13);
        Vector2d center = origin + Rotate(new(reverse ? 6 : -2, 0));
        Vector2d direction = Rotate(reverse ? Vector2d.Left : Vector2d.Right);
        Vector2d expectedNormal = -direction;
        Vector2d expectedPoint = new(Fixed64.FromFraction(reverse ? 5 : 3, 2), Fixed64.Zero);
        Fixed64 expectedDistance = capsule ? Fixed64.FromFraction(5, 2) : (Fixed64)3;
        for (int frame = 0; frame < 2; frame++)
        {
            Vector2d shift = frame == 0 ? Vector2d.Zero : new(2, 1);
            Vector2d targetOrigin = origin + Rotate(shift);
            Vector2d[] polygon =
            {
                new(Fixed64.FromFraction(3, 2), Fixed64.Zero), new(Fixed64.FromFraction(5, 2), Fixed64.Zero),
                new(Fixed64.FromFraction(5, 2), Fixed64.FromFraction(3, 2)), new(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(5, 2)),
            };
            for (int i = 0; i < polygon.Length; i++) polygon[i] -= shift;
            if (reversedWinding) Array.Reverse(polygon);
            Fixed64 distance;
            Vector2d normal;
            Vector2d offset;
            bool hit;
            if (rotated)
            {
                FixedPointAnchor2d contact;
                hit = capsule
                    ? FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(center, Rotate(Vector2d.Right),
                        Fixed64.One, Fixed64.Half, direction, (Fixed64)8, targetOrigin, Fixed64.HalfPi, polygon,
                        out distance, out normal, out contact)
                    : FixedConvex2dRelations.TryGetSweptCircleFirstDistance(center, Fixed64.Half, direction,
                        (Fixed64)8, targetOrigin, Fixed64.HalfPi, polygon, out distance, out normal, out contact);
                Assert.Equal(targetOrigin, contact.Origin);
                Assert.Equal(Fixed64.HalfPi, contact.Rotation);
                Assert.True(contact.TryGetPoint(out Vector2d world));
                Assert.Equal(origin + Rotate(expectedPoint), world);
                offset = contact.LocalPoint;
            }
            else
            {
                hit = capsule
                    ? FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(center, Vector2d.Right,
                        Fixed64.One, Fixed64.Half, direction, (Fixed64)8, targetOrigin, polygon,
                        out distance, out normal, out offset)
                    : FixedConvex2dRelations.TryGetSweptCircleFirstDistance(center, Fixed64.Half, direction,
                        (Fixed64)8, targetOrigin, polygon, out distance, out normal, out offset);
            }
            Assert.True(hit);
            Assert.Equal(expectedDistance, distance);
            Assert.Equal(expectedNormal, normal);
            Assert.Equal(expectedPoint - shift, offset);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ConvexSweep_KeepsEntryNormalWhenEitherOriginIsRebased(bool reverse, bool rotated)
    {
        Vector2d Rotate(Vector2d p) => rotated ? new(-p.Y, p.X) : p;
        Vector2d origin = new(7, -13);
        Vector2d center = origin + Rotate(new(reverse ? 6 : -2, 1));
        Vector2d direction = Rotate(reverse ? Vector2d.Left : Vector2d.Right);
        Fixed64 rotation = rotated ? Fixed64.HalfPi : Fixed64.Zero;
        Vector2d firstWorldContact = default;
        for (int targetFrame = 0; targetFrame < 2; targetFrame++)
        for (int sourceFrame = 0; sourceFrame < 2; sourceFrame++)
        {
            Vector2d targetShift = targetFrame == 0 ? Vector2d.Zero : new(2, 1);
            Vector2d sourceShift = sourceFrame == 0 ? Vector2d.Zero : new(5, 3);
            Vector2d[] target = { new(1, 0), new(3, 0), new(3, 2), new(1, 2) };
            Vector2d[] source = { new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
                new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half) };
            for (int i = 0; i < target.Length; i++) target[i] -= targetShift;
            for (int i = 0; i < source.Length; i++) source[i] -= sourceShift;
            Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
                center + Rotate(sourceShift), rotation, source, direction, (Fixed64)8,
                origin + Rotate(targetShift), rotation, target,
                out Fixed64 distance, out Vector2d normal, out FixedPointAnchor2d contact));
            Assert.Equal(Fixed64.FromFraction(5, 2), distance);
            Assert.Equal(-direction, normal);
            Assert.Equal((Fixed64)(reverse ? 3 : 1), contact.LocalPoint.X + targetShift.X);
            FixedMathTestHelper.AssertWithinRange(contact.LocalPoint.Y + targetShift.Y, Fixed64.Half, Fixed64.FromFraction(3, 2));
            Assert.True(contact.TryGetPoint(out Vector2d world));
            Assert.Equal(origin + Rotate(contact.LocalPoint + targetShift), world);
            if (targetFrame == 0 && sourceFrame == 0) firstWorldContact = world;
            else Assert.Equal(firstWorldContact, world);
        }
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void TangentSweep_NormalFacesSourceWhenMotionCannotChooseItsSign(bool capsule, bool reverse, bool rotated)
    {
        Vector2d Rotate(Vector2d p) => rotated ? new(-p.Y, p.X) : p;
        Vector2d center = Rotate(new(reverse ? 2 : -2, 1));
        Vector2d direction = Rotate(reverse ? Vector2d.Left : Vector2d.Right);
        for (int frame = 0; frame < 2; frame++)
        {
            Vector2d shift = frame == 0 ? Vector2d.Zero : new(0, 2);
            Vector2d[] polygon = { new Vector2d(0, 0) - shift, new Vector2d(1, -1) - shift, new Vector2d(-1, -1) - shift };
            bool hit = capsule
                ? FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(center, Rotate(Vector2d.Right),
                    Fixed64.One, Fixed64.One, direction, (Fixed64)4, Rotate(shift),
                    rotated ? Fixed64.HalfPi : Fixed64.Zero, polygon,
                    out Fixed64 distance, out Vector2d normal, out FixedPointAnchor2d contact)
                : FixedConvex2dRelations.TryGetSweptCircleFirstDistance(center, Fixed64.One, direction,
                    (Fixed64)4, Rotate(shift), rotated ? Fixed64.HalfPi : Fixed64.Zero, polygon,
                    out distance, out normal, out contact);
            Assert.True(hit);
            Assert.Equal(capsule ? Fixed64.FromFraction(3, 2) : Fixed64.Two, distance);
            Assert.Equal(Rotate(Vector2d.Forward), normal);
            Assert.Equal(-shift, contact.LocalPoint);
            Assert.True(contact.TryGetPoint(out Vector2d world));
            Assert.Equal(Vector2d.Zero, world);
        }
    }

    [Fact]
    public void SegmentEntry_RetainsNormalAndUnmaterializableAnchorAcrossRebasing()
    {
        for (int frame = 0; frame < 2; frame++)
        {
            Fixed64 shift = frame == 0 ? Fixed64.Zero : -Fixed64.Two;
            Vector2d origin = new(Fixed64.MaxValue + shift, Fixed64.Zero);
            Vector2d[] polygon = { new(Fixed64.One - shift, Fixed64.Zero), new((Fixed64)3 - shift, Fixed64.Zero),
                new((Fixed64)3 - shift, Fixed64.Two), new(Fixed64.One - shift, Fixed64.Two) };
            Assert.True(FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(
                new(Fixed64.MaxValue - Fixed64.Two, Fixed64.One), Vector2d.Right, (Fixed64)8,
                origin, Fixed64.Zero, polygon, out Fixed64 distance, out Vector2d normal, out FixedPointAnchor2d contact));
            Assert.Equal((Fixed64)3, distance);
            Assert.Equal(Vector2d.Left, normal);
            Assert.Equal(new Vector2d(Fixed64.One - shift, Fixed64.One), contact.LocalPoint);
            Assert.False(contact.TryGetPoint(out _));
        }
    }

    [Theory]
    [InlineData(false, 7158278826L)]
    [InlineData(true, 7158278824L)]
    public void SweepEntry_NormalOpposesMotionNearRoundedFaceCrossing(bool minimumRadius, long expectedRawDistance)
    {
        Vector2d[] polygon = { new(1, -100), new(3, -100), new(3, 100), new(1, 100) };
        Vector2d direction = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        bool hit = minimumRadius
            ? FixedConvex2dRelations.TryGetSweptCircleFirstDistance(Vector2d.Zero, Fixed64.MinIncrement,
                direction, (Fixed64)3, Vector2d.Zero, polygon, out Fixed64 distance, out Vector2d normal, out Vector2d contact)
            : FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(Vector2d.Zero, direction,
                (Fixed64)3, Vector2d.Zero, polygon, out distance, out normal, out contact);
        Assert.True(hit);
        Assert.Equal(Fixed64.FromRaw(expectedRawDistance), distance);
        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(Fixed64.One, contact.X);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void ZeroRadiusVertexGraze_RetainsSupportingNormalAndApexWitness(int shape, bool reverse)
    {
        Vector2d[] vertices = { new(0, 0), new(1, -1), new(-1, -1) };
        Vector2d direction = reverse ? Vector2d.Left : Vector2d.Right;
        Vector2d center = new((Fixed64)(reverse ? 2 : -2), shape == 2 ? Fixed64.Half : Fixed64.Zero);
        for (int order = 0; order < 6; order++)
        for (int frame = 0; frame < 3; frame++)
        {
            Vector2d shift = frame == 0 ? Vector2d.Zero : frame == 1 ? new(Fixed64.Zero, -Fixed64.Half) : new(4, 2);
            var polygon = new Vector2d[3];
            for (int i = 0; i < polygon.Length; i++)
                polygon[i] = vertices[(order / 2 + (order % 2 == 0 ? i : 3 - i)) % 3] - shift;
            Fixed64 distance;
            Vector2d normal;
            Vector2d contact;
            bool hit;
            if (shape == 2)
            {
                Vector2d[] source = { new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
                    new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half) };
                hit = FixedConvex2dRelations.TryGetSweptConvexFirstDistance(center, source, direction,
                    (Fixed64)4, shift, polygon, out distance, out normal, out contact);
            }
            else
            {
                hit = shape == 0
                    ? FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(center, direction,
                        (Fixed64)4, shift, polygon, out distance, out normal, out contact)
                    : FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(center, Vector2d.Right,
                        Fixed64.One, Fixed64.Zero, direction, (Fixed64)4, shift, polygon,
                        out distance, out normal, out contact);
            }
            Assert.True(hit);
            Assert.Equal(shape == 0 ? Fixed64.Two : Fixed64.FromFraction(3, 2), distance);
            Assert.Equal(-shift, contact);
            Assert.True(normal.IsNormalized());
            // A valid supporting normal cannot put another target vertex beyond the contacted apex.
            foreach (Vector2d vertex in vertices) Assert.True(Vector2d.Dot(vertex, normal) <= Fixed64.Zero);
        }
    }

    [Fact]
    public void SegmentEntry_RoundsTheWitnessFromTheExactEventRatherThanRoundedDistance()
    {
        Fixed64 left = Fixed64.FromRaw(1), right = Fixed64.FromRaw(3);
        Vector2d[] polygon = { new(left, (Fixed64)(-100)), new(right, (Fixed64)(-100)),
            new(right, (Fixed64)100), new(left, (Fixed64)100) };
        Assert.True(FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(Vector2d.Zero,
            new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)), (Fixed64)3,
            Vector2d.Zero, polygon, out Fixed64 distance, out Vector2d normal, out Vector2d contact));
        Assert.Equal(Fixed64.FromRaw(2), distance);
        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(new Vector2d(left, Fixed64.FromRaw(1)), contact);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(false, 2)]
    [InlineData(false, 3)]
    [InlineData(false, 4)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    [InlineData(true, 3)]
    [InlineData(true, 4)]
    public void PublicZeroRadiusSweep_CollapsedTargetsRetainFiniteEndAndOddHalfAxis(bool pointTarget, int shape)
    {
        Vector2d[] polygon = { Vector2d.Zero, Vector2d.Zero, pointTarget ? Vector2d.Zero : new Vector2d(2, 0) };
        Vector2d center = new(-2, 0);
        Fixed64 distance;
        Vector2d normal, contact;
        bool hit;
        if (shape >= 3)
        {
            Vector2d[] source = { shape == 3 ? Vector2d.Zero : new(-Fixed64.Half, Fixed64.Zero),
                shape == 3 ? Vector2d.Zero : new(Fixed64.Half, Fixed64.Zero),
                shape == 3 ? Vector2d.Zero : new(Fixed64.Half, Fixed64.Zero) };
            hit = FixedConvex2dRelations.TryGetSweptConvexFirstDistance(center, Fixed64.Zero, source,
                Vector2d.Right, (Fixed64)4, Vector2d.Zero, Fixed64.Zero, polygon,
                out distance, out normal, out FixedPointAnchor2d anchor);
            contact = anchor.LocalPoint;
        }
        else
        {
            hit = shape == 0
                ? FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(center, Vector2d.Right,
                    (Fixed64)4, Vector2d.Zero, polygon, out distance, out normal, out contact)
                : FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(center, Vector2d.Right,
                    shape == 1 ? Fixed64.One : Fixed64.FromRaw(3), Fixed64.Zero, Vector2d.Right,
                    (Fixed64)4, Vector2d.Zero, polygon, out distance, out normal, out contact);
        }
        Assert.True(hit);
        Assert.Equal(shape == 2 ? Fixed64.FromRaw(8589934590L)
            : shape == 1 || shape == 4 ? Fixed64.FromFraction(3, 2) : Fixed64.Two, distance);
        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(Vector2d.Zero, contact);
    }

    [Theory]
    [InlineData(false, -3, 0)]
    [InlineData(false, 2, 1)]
    [InlineData(true, -3, 0)]
    [InlineData(true, 2, 1)]
    public void ZeroRadiusCore_SideWitnessClampsToTheFiniteTargetEdge(bool reverse, int y, int contactY)
    {
        Vector2d[] polygon = { new(0, 0), new(1, 0), new(1, 1), new(0, 1) };
        Assert.True(FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(new Vector2d(reverse ? 3 : -2, y),
            Vector2d.Forward, (Fixed64)8, Fixed64.Zero, reverse ? Vector2d.Left : Vector2d.Right, (Fixed64)4,
            Vector2d.Zero, polygon, out Fixed64 distance, out Vector2d normal, out Vector2d contact));
        Assert.Equal(Fixed64.Two, distance);
        Assert.Equal(reverse ? Vector2d.Right : Vector2d.Left, normal);
        Assert.Equal(new Vector2d(reverse ? 1 : 0, contactY), contact);
    }

    [Fact]
    public void SegmentEntry_ObliqueFaceWitnessUsesTheExactSupportingAxis()
    {
        Vector2d[] polygon = { new(0, 0), new(5, 12), new(5, 0) };
        for (int order = 0; order < 2; order++)
        {
            if (order != 0) Array.Reverse(polygon);
            Assert.True(FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(new Vector2d(-2, 6),
                Vector2d.Right, (Fixed64)10, Vector2d.Zero, polygon,
                out Fixed64 distance, out Vector2d normal, out Vector2d contact));
            Assert.Equal(Fixed64.FromFraction(9, 2), distance);
            Assert.Equal(new Vector2d(Fixed64.FromFraction(-12, 13), Fixed64.FromFraction(5, 13)), normal);
            Assert.Equal(new Vector2d(Fixed64.FromFraction(5, 2), (Fixed64)6), contact);
        }
    }

    [Fact]
    public void ZeroRadiusSweeps_CoincidentCollapsedShapesRetainInitialContact()
    {
        Vector2d[] point = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(Vector2d.Zero, Fixed64.Zero,
            Vector2d.Right, Fixed64.One, Vector2d.Zero, point,
            out Fixed64 circleDistance, out _, out Vector2d circleContact));
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(Vector2d.Zero, point,
            Vector2d.Right, Fixed64.One, Vector2d.Zero, point,
            out Fixed64 convexDistance, out _, out Vector2d convexContact));
        Assert.Equal(Fixed64.Zero, circleDistance);
        Assert.Equal(Fixed64.Zero, convexDistance);
        Assert.Equal(Vector2d.Zero, circleContact);
        Assert.Equal(Vector2d.Zero, convexContact);
    }

    [Fact]
    public void SweepContactRecovery_DoesNotAllocateForWarmedQueries()
    {
        Vector2d[] target = { new(0, 0), new(5, 12), new(5, 0) };
        Vector2d[] source = { new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half) };
        long bytes = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int i = 0; i < 32; i++)
            {
                if (!FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(new Vector2d(-2, 6), Vector2d.Right,
                        (Fixed64)10, Vector2d.Zero, target, out _, out _, out _)
                    || !FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistance(new Vector2d(-2, 6), Vector2d.Forward,
                        Fixed64.One, Fixed64.Zero, Vector2d.Right, (Fixed64)10, Vector2d.Zero, target, out _, out _, out _)
                    || !FixedConvex2dRelations.TryGetSweptCircleFirstDistance(new Vector2d(-2, 6), Fixed64.Half,
                        Vector2d.Right, (Fixed64)10, Vector2d.Zero, target, out _, out _, out _)
                    || !FixedConvex2dRelations.TryGetSweptConvexFirstDistance(new Vector2d(-2, 6), source,
                        Vector2d.Right, (Fixed64)10, Vector2d.Zero, target, out _, out _, out _))
                    throw new InvalidOperationException("Warmed sweep fixtures must hit.");
            }
        });
        Assert.Equal(0, bytes);
    }

    [Fact]
    public void SweepFamilies_ReportInitialOverlapAtZeroDistance()
    {
        Assert.True(FixedConvex2dRelations
            .TryGetSegmentFirstIntersectionDistance(
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                UnitBox,
                out Fixed64 segmentDistance,
                out Vector2d segmentNormal,
                out _));
        Assert.True(FixedConvex2dRelations
            .TryGetSegmentFirstIntersectionDistance(
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.PiOver4,
                UnitBox,
                out Fixed64 rotatedSegmentDistance,
                out Vector2d rotatedSegmentNormal,
                out _));
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            Vector2d.Zero,
            Fixed64.Half,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Zero,
            UnitBox,
            out Fixed64 circleDistance,
            out _,
            out _));
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            Vector2d.Zero,
            Fixed64.Half,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.PiOver4,
            UnitBox,
            out Fixed64 rotatedCircleDistance,
            out _,
            out _));
        Assert.True(FixedConvex2dRelations
            .TryGetSweptCenteredCapsuleFirstDistance(
                Vector2d.Zero,
                Vector2d.Forward,
                Fixed64.One,
                Fixed64.Half,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                UnitBox,
                out Fixed64 capsuleDistance,
                out _,
                out _));
        Assert.True(FixedConvex2dRelations
            .TryGetSweptCenteredCapsuleFirstDistance(
                Vector2d.Zero,
                Vector2d.Forward,
                Fixed64.One,
                Fixed64.Half,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.PiOver4,
                UnitBox,
                out Fixed64 rotatedCapsuleDistance,
                out _,
                out _));
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            Vector2d.Zero,
            UnitBox,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Right * Fixed64.Half,
            UnitBox,
            out Fixed64 convexDistance,
            out _,
            out _));
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            Vector2d.Zero,
            Fixed64.PiOver4,
            UnitBox,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Right * Fixed64.Half,
            Fixed64.PiOver4,
            UnitBox,
            out Fixed64 rotatedConvexDistance,
            out _,
            out _));

        Assert.Equal(Fixed64.Zero, segmentDistance);
        Assert.Equal(-Vector2d.Right, segmentNormal);
        Assert.Equal(Fixed64.Zero, rotatedSegmentDistance);
        Assert.Equal(-Vector2d.Right, rotatedSegmentNormal);
        Assert.Equal(Fixed64.Zero, circleDistance);
        Assert.Equal(Fixed64.Zero, rotatedCircleDistance);
        Assert.Equal(Fixed64.Zero, capsuleDistance);
        Assert.Equal(Fixed64.Zero, rotatedCapsuleDistance);
        Assert.Equal(Fixed64.Zero, convexDistance);
        Assert.Equal(Fixed64.Zero, rotatedConvexDistance);
    }

    [Fact]
    public void RotatedSweepFamilies_ReturnFalseWhenMovingAway()
    {
        Vector2d farLeft = new((Fixed64)(-5), Fixed64.Zero);

        Assert.False(FixedConvex2dRelations
            .TryGetSegmentFirstIntersectionDistance(
                farLeft,
                Vector2d.Left,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.PiOver4,
                UnitBox,
                out _,
                out _,
                out _));
        Assert.False(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            farLeft,
            Fixed64.Half,
            Vector2d.Left,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.PiOver4,
            UnitBox,
            out _,
            out _,
            out _));
        Assert.False(FixedConvex2dRelations
            .TryGetSweptCenteredCapsuleFirstDistance(
                farLeft,
                Vector2d.Forward,
                Fixed64.One,
                Fixed64.Half,
                Vector2d.Left,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.PiOver4,
                UnitBox,
                out _,
                out _,
                out _));
        Assert.False(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            farLeft,
            Fixed64.PiOver4,
            UnitBox,
            Vector2d.Left,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.PiOver4,
            UnitBox,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void SweepFamilies_RejectInvalidGeometryAndTravelContracts()
    {
        Assert.Equal(
            "circleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
                    Vector2d.Zero,
                    -Fixed64.One,
                    Vector2d.Right,
                    Fixed64.One,
                    Vector2d.Zero,
                    UnitBox,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "circleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
                    Vector2d.Zero,
                    -Fixed64.One,
                    Vector2d.Right,
                    Fixed64.One,
                    Vector2d.Zero,
                    Fixed64.PiOver4,
                    UnitBox,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "capsuleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations
                    .TryGetSweptCenteredCapsuleFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        Fixed64.One,
                        -Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        Vector2d.Zero,
                        UnitBox,
                        out _,
                        out _,
                        out _)).ParamName);
        Assert.Equal(
            "capsuleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations
                    .TryGetSweptCenteredCapsuleFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        Fixed64.One,
                        -Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        Vector2d.Zero,
                        Fixed64.PiOver4,
                        UnitBox,
                        out _,
                        out _,
                        out _)).ParamName);
        Assert.Equal(
            "direction",
            Assert.Throws<ArgumentException>(() =>
                FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
                    Vector2d.Zero,
                    UnitBox,
                    Vector2d.One,
                    Fixed64.One,
                    Vector2d.Right * (Fixed64)4,
                    UnitBox,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "maximumDistance",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
                    Vector2d.Zero,
                    UnitBox,
                    Vector2d.Right,
                    -Fixed64.One,
                    Vector2d.Right * (Fixed64)4,
                    UnitBox,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "axisDirection",
            Assert.Throws<ArgumentException>(() =>
                FixedConvex2dRelations
                    .TryGetSweptCenteredCapsuleFirstDistance(
                        Vector2d.Zero,
                        Vector2d.One,
                        Fixed64.One,
                        Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        Vector2d.Right * (Fixed64)4,
                        UnitBox,
                        out _,
                        out _,
                        out _)).ParamName);
        Assert.Equal(
            "axisLength",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations
                    .TryGetSweptCenteredCapsuleFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        -Fixed64.One,
                        Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        Vector2d.Right * (Fixed64)4,
                        UnitBox,
                        out _,
                        out _,
                        out _)).ParamName);
    }
}
