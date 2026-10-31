using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedConvex2dDegenerateRelationsTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void ContainsPoint_CollapsedBoundaryHasFiniteClosedExtent(int rotationCase, bool point)
    {
        Fixed64 rotation = Rotation(rotationCase);
        Vector2d origin = new(7, -13);
        Vector2d[] offsets = point
            ? new[] { Vector2d.Right, Vector2d.Right, Vector2d.Right }
            : new[] { Vector2d.Zero, Vector2d.Zero, new Vector2d(2, 0), Vector2d.Zero };
        for (int order = 0; order < offsets.Length * 2; order++)
        {
            Assert.True(FixedConvex2dRelations.ContainsPoint(origin + Rotate(Vector2d.Right, rotation), origin, rotation, offsets));
            Assert.Equal(!point, FixedConvex2dRelations.ContainsPoint(origin, origin, rotation, offsets));
            Assert.Equal(!point, FixedConvex2dRelations.ContainsPoint(origin + Rotate(new(2, 0), rotation), origin, rotation, offsets));
            Assert.False(FixedConvex2dRelations.ContainsPoint(origin + Rotate(new(-1, 0), rotation), origin, rotation, offsets));
            Assert.False(FixedConvex2dRelations.ContainsPoint(origin + Rotate(new(3, 0), rotation), origin, rotation, offsets));
            Assert.False(FixedConvex2dRelations.ContainsPoint(origin + Rotate(Vector2d.Right, rotation)
                + new Vector2d(Fixed64.MinIncrement, Fixed64.MinIncrement), origin, rotation, offsets));
            if (point)
            {
                Vector2d world = origin + Rotate(Vector2d.Right, rotation);
                foreach (Vector2d delta in new[] { new Vector2d(Fixed64.MinIncrement, Fixed64.Zero),
                    new Vector2d(-Fixed64.MinIncrement, Fixed64.Zero), new Vector2d(Fixed64.Zero, Fixed64.MinIncrement),
                    new Vector2d(Fixed64.Zero, -Fixed64.MinIncrement) })
                    Assert.False(FixedConvex2dRelations.ContainsPoint(world + delta, origin, rotation, offsets));
            }
            Cycle(offsets);
            if (order == offsets.Length - 1) Array.Reverse(offsets);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PublicSweep_CollapsedTargetKeepsTheFirstFiniteEntry(bool point)
    {
        Vector2d[] target = { Vector2d.Zero, Vector2d.Zero, point ? Vector2d.Zero : new(2, 0) };
        Assert.True(FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(
            new(-2, 0), Vector2d.Right, (Fixed64)4, Vector2d.Zero, target,
            out Fixed64 distance, out Vector2d normal, out Vector2d contact));
        Assert.Equal(Fixed64.Two, distance);
        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(Vector2d.Zero, contact);

        Vector2d[] source = { new(-Fixed64.Half, Fixed64.Zero), Vector2d.Zero, new(Fixed64.Half, Fixed64.Zero) };
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            new(-2, 0), Fixed64.Zero, source, Vector2d.Right, (Fixed64)4,
            Vector2d.Zero, Fixed64.Zero, target, out distance, out normal, out FixedPointAnchor2d anchor));
        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(Vector2d.Zero, anchor.LocalPoint);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void StaticCollapsedContacts_RejectFiniteGapsAndKeepEndpointTouch(bool point, bool rotated)
    {
        Fixed64 rotation = rotated ? Fixed64.HalfPi : Fixed64.Zero;
        Vector2d[] first = point
            ? new[] { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero }
            : new[] { new Vector2d(-1, 0), Vector2d.Zero, new Vector2d(1, 0), Vector2d.Zero };
        Vector2d[] second = { new(-1, 0), Vector2d.Zero, new(1, 0), Vector2d.Zero };
        Fixed64 touching = point ? Fixed64.One : Fixed64.Two;
        for (int reverse = 0; reverse < 2; reverse++)
        {
            Vector2d axis = Rotate(Vector2d.Right, rotation);
            Vector2d perpendicularGap = Rotate(new(Fixed64.Zero, Fixed64.MinIncrement), rotation);
            Assert.False(Contacts(perpendicularGap, rotation, first, Vector2d.Zero, rotation,
                second, reverse != 0, out _, out _, out _));
            Assert.False(Contacts(Vector2d.Zero, rotation, first, axis * (touching + Fixed64.MinIncrement), rotation,
                second, reverse != 0, out _, out _, out _));
            Assert.True(Contacts(Vector2d.Zero, rotation, first, axis * touching, rotation,
                second, reverse != 0, out Fixed64 depth, out _, out _));
            Assert.Equal(Fixed64.Zero, depth);
            Assert.True(Contacts(Vector2d.Zero, rotation, first, axis * (touching - Fixed64.Half), rotation,
                second, reverse != 0, out depth, out _, out _));
            Assert.Equal(Fixed64.Zero, depth);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoincidentPointPolygons_HaveAUnitZeroDepthContact(bool reverse)
    {
        Vector2d[] point = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Assert.False(Contacts(Vector2d.Zero, Fixed64.Zero, point,
            new(Fixed64.MinIncrement, Fixed64.Zero), Fixed64.Zero, point, reverse, out _, out _, out _));
        Assert.False(Contacts(Vector2d.Zero, Fixed64.Zero, point,
            new(Fixed64.Zero, Fixed64.MinIncrement), Fixed64.Zero, point, reverse, out _, out _, out _));
        Assert.True(Contacts(Vector2d.Zero, Fixed64.Zero, point,
            Vector2d.Zero, Fixed64.Zero, point, reverse, out Fixed64 depth, out Vector2d normal, out _));
        Assert.Equal(Fixed64.Zero, depth);
        Assert.Equal(Fixed64.One, Vector2d.Dot(normal, normal));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void PointOnRepeatedSegment_PreservesBothAuthoredFrames(int rotationCase, bool reverse)
    {
        Fixed64 rotation = Rotation(rotationCase);
        Vector2d[] point = { Vector2d.Right, Vector2d.Right, Vector2d.Right };
        Vector2d[] line = { Vector2d.Right, Vector2d.Right, new(2, 0), Vector2d.Zero, Vector2d.Right };
        for (int order = 0; order < line.Length * 2; order++)
        {
            Assert.True(Contacts(Vector2d.Zero, rotation, point, Vector2d.Zero, rotation, line, reverse,
                out Fixed64 depth, out _, out Vector2d linePoint));
            Assert.Equal(Fixed64.Zero, depth);
            Assert.Equal(Rotate(Vector2d.Right, rotation), linePoint);
            Cycle(line);
            if (order == line.Length - 1) Array.Reverse(line);
        }
    }

    [Fact]
    public void CrossingSegments_RetainPositiveMinimumExitDepth()
    {
        Vector2d[] line = { new(-1, 0), Vector2d.Zero, new(1, 0), Vector2d.Zero };
        Assert.True(Contacts(Vector2d.Zero, Fixed64.Zero, line, Vector2d.Zero, Fixed64.HalfPi, line, false,
            out Fixed64 depth, out Vector2d normal, out _));
        Assert.Equal(Fixed64.One, depth);
        Assert.True(Contacts(-normal * depth, Fixed64.Zero, line, Vector2d.Zero, Fixed64.HalfPi, line, false,
            out Fixed64 touchingDepth, out _, out _));
        Assert.Equal(Fixed64.Zero, touchingDepth);
        Assert.False(Contacts(-normal * (depth + Fixed64.MinIncrement), Fixed64.Zero, line,
            Vector2d.Zero, Fixed64.HalfPi, line, false, out _, out _, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SubdividedSupportFace_KeepsItsCompleteExtent(bool reverse)
    {
        Vector2d[] point = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Vector2d[] box = { new(1, 0), new(2, 0), new(2, 2), new(0, 2), new(0, 0) };
        Vector2d contact = new(Fixed64.FromFraction(3, 2), Fixed64.Zero);
        Assert.True(Contacts(contact, Fixed64.Zero, point, Vector2d.Zero, Fixed64.Zero, box, reverse,
            out Fixed64 depth, out _, out Vector2d boxPoint));
        Assert.Equal(Fixed64.Zero, depth);
        Assert.Equal(contact, boxPoint);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void CircleAgainstCollapsedPoint_RetainsInitialOverlapAndFiniteGap(int rotationCase)
    {
        Fixed64 rotation = Rotation(rotationCase);
        Vector2d[] point = { Vector2d.Right, Vector2d.Right, Vector2d.Right };
        Vector2d world = Rotate(Vector2d.Right, rotation);
        Assert.True(FixedConvex2dRelations.TryGetCircleContact(world, Fixed64.Zero, Fixed64.Half,
            Vector2d.Zero, rotation, point, out FixedPointAnchor2d circle, out FixedPointAnchor2d target,
            out Vector2d normal, out Fixed64 depth, out bool clamped));
        Assert.False(clamped);
        Assert.Equal(Fixed64.Half, depth);
        Assert.Equal(Fixed64.One, Vector2d.Dot(normal, normal));
        Assert.True(target.TryGetPoint(out Vector2d targetPoint));
        Assert.Equal(world, targetPoint);
        Assert.True(circle.TryGetPoint(out Vector2d circlePoint));
        Assert.Equal(Fixed64.Half, (circlePoint - targetPoint).Magnitude);
        Assert.False(FixedConvex2dRelations.TryGetCircleContact(world + Vector2d.Right, Fixed64.Zero, Fixed64.Half,
            Vector2d.Zero, rotation, point, out _, out _, out _, out _, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ClosedUprightRelation_AcceptsCollapsedHullWhileStrictSweepHasNoInterior(bool point, bool positiveRadius)
    {
        Vector2d[] target = { Vector2d.Zero, Vector2d.Zero, point ? Vector2d.Zero : new(2, 0) };
        Fixed64 radius = positiveRadius ? Fixed64.Half : Fixed64.Zero;
        Vector2d center = new(point ? Fixed64.Zero : Fixed64.One, Fixed64.Zero);
        Assert.True(FixedConvex2dRelations.IntersectsUprightCapsule(center, Fixed64.One, radius,
            Vector2d.Zero, target));
        Assert.False(FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(center - Vector2d.Right,
            center + Vector2d.Right, Fixed64.One, radius, Vector2d.Zero, target));
        Assert.False(FixedConvex2dRelations.IntersectsUprightCapsule(new(3, 0), Fixed64.One, radius,
            Vector2d.Zero, target));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CapsuleAgainstPoint_PreservesClosedTouchAndPositiveTranslation(bool finiteCore, bool positiveRadius)
    {
        Vector2d[] point = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Fixed64 length = finiteCore ? Fixed64.One : Fixed64.Zero;
        Fixed64 radius = positiveRadius ? Fixed64.Half : Fixed64.Zero;
        for (int step = -1; step <= 1; step++)
        {
            Vector2d center = new(Fixed64.Zero, length / Fixed64.Two + radius + Fixed64.FromRaw(step));
            bool closed = step <= 0 && (length > Fixed64.Zero || radius > Fixed64.Zero || step == 0);
            bool strict = step < 0 && radius > Fixed64.Zero;
            Assert.Equal(closed, FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(center,
                Vector2d.Forward, length, radius, Vector2d.Zero, Fixed64.Zero, point,
                out _, out Fixed64 depth));
            Assert.Equal(closed, FixedConvex2dRelations.IntersectsUprightCapsule(center, length, radius,
                Vector2d.Zero, point));
            Assert.Equal(strict, FixedSegment2d.DoesCenteredCapsulePenetrateConvex(center,
                Vector2d.Forward, length, radius, Vector2d.Zero, Fixed64.Zero, point));
            Assert.Equal(closed && step < 0 && radius > Fixed64.Zero ? Fixed64.MinIncrement : Fixed64.Zero, depth);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CollapsedRelations_PreserveFiniteExtentBeyondScalarCoordinates(bool rotated)
    {
        Fixed64 rotation = rotated ? Fixed64.HalfPi : Fixed64.Zero;
        Vector2d origin = new(Fixed64.MaxValue, Fixed64.MaxValue);
        Vector2d[] line = { new(Fixed64.MinValue, Fixed64.Zero), new(Fixed64.MinValue, Fixed64.Zero),
            new(Fixed64.MaxValue, Fixed64.Zero) };
        Vector2d World(Fixed64 coordinate) => rotated ? new(Fixed64.MaxValue, coordinate) : new(coordinate, Fixed64.MaxValue);
        Vector2d boundary = World(-Fixed64.MinIncrement);
        Assert.True(FixedConvex2dRelations.ContainsPoint(boundary, origin, rotation, line));
        Assert.True(FixedConvex2dRelations.ContainsPoint(World(Fixed64.Zero), origin, rotation, line));
        Assert.False(FixedConvex2dRelations.ContainsPoint(World(Fixed64.FromRaw(-2)), origin, rotation, line));
        Assert.False(FixedConvex2dRelations.ContainsPoint(World(Fixed64.MinValue), origin, rotation, line));
        Vector2d[] point = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Assert.True(Contacts(boundary, Fixed64.Zero, point, origin, rotation, line, false,
            out Fixed64 depth, out _, out Vector2d contact));
        Assert.Equal(Fixed64.Zero, depth);
        Assert.Equal(boundary, contact);

        Vector2d[] beyond = { new Vector2d(1, 0), new Vector2d(1, 0), new Vector2d(3, 0) };
        Vector2d[] midpoint = { new Vector2d(2, 0), new Vector2d(2, 0), new Vector2d(2, 0) };
        Assert.False(FixedConvex2dRelations.ContainsPoint(origin, origin, rotation, beyond));
        Span<FixedPointAnchor2d> first = stackalloc FixedPointAnchor2d[2], second = stackalloc FixedPointAnchor2d[2];
        Assert.True(FixedConvex2dRelations.TryGetConvexContacts(origin, rotation, midpoint,
            origin, rotation, beyond, first, second, out int count, out _, out depth, out bool clamped));
        Assert.Equal(1, count);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(clamped);
        Assert.Equal(new Vector2d(2, 0), first[0].LocalPoint);
        Assert.Equal(new Vector2d(2, 0), second[0].LocalPoint);
        Assert.False(first[0].TryGetPoint(out _));
        Assert.False(second[0].TryGetPoint(out _));
    }

    [Fact]
    public void CollapsedRelations_DoNotAllocateForWarmedQueries()
    {
        Vector2d[] point = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Vector2d[] line = { Vector2d.Zero, Vector2d.Zero, new(2, 0), Vector2d.Zero };
        FixedPointAnchor2d[] a = new FixedPointAnchor2d[2], b = new FixedPointAnchor2d[2];
        long bytes = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int i = 0; i < 32; i++)
            {
                if (!FixedConvex2dRelations.ContainsPoint(Vector2d.Right, Vector2d.Zero, line)
                    || !FixedConvex2dRelations.TryGetConvexContacts(Vector2d.Right, Fixed64.PiOver6, point,
                        Vector2d.Right, Fixed64.PiOver6, line, a, b, out _, out _, out _, out _)
                    || !FixedConvex2dRelations.IntersectsUprightCapsule(Vector2d.Right, Fixed64.One, Fixed64.Zero,
                        Vector2d.Zero, line)
                    || !FixedConvex2dRelations.TryGetCircleContact(Vector2d.Zero, Fixed64.Zero, Fixed64.One,
                        Vector2d.Zero, Fixed64.Zero, point, out _, out _, out _, out _, out _)
                    || !FixedConvex2dRelations.TryGetSegmentFirstIntersectionDistance(new(-2, 0), Vector2d.Right,
                        (Fixed64)4, Vector2d.Zero, line, out _, out _, out _))
                    throw new InvalidOperationException("The allocation fixture must exercise successful collapsed queries.");
            }
        });
        Assert.Equal(0L, bytes);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void InternalSweepCore_RetainsBoundaryStartForDownstreamQueries(bool rotated, bool authoredChord)
    {
        Fixed64 rotation = rotated ? Fixed64.HalfPi : Fixed64.Zero;
        Vector2d[] box = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Vector2d direction = Rotate(Vector2d.Right, rotation);
        // Gravitas calls the shared core directly. It must also accept initial
        // boundary contact without the public wrapper's static-contact precheck.
        Assert.True(FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistanceCore(
            Rotate(new(-1, 0), rotation), Vector2d.Right, Fixed64.Zero, Fixed64.Zero,
            authoredChord ? direction * (Fixed64)4 : direction, (Fixed64)4,
            Vector2d.Zero, rotation, box, out Fixed64 distance, out Vector2d normal, out Vector2d contact,
            authoredDisplacement: authoredChord));
        Assert.Equal(Fixed64.Zero, distance);
        Assert.Equal(-direction, normal);
        Assert.Equal(new Vector2d(-1, 0), contact);
    }

    private static bool Contacts(Vector2d firstOrigin, Fixed64 firstRotation, Vector2d[] first,
        Vector2d secondOrigin, Fixed64 secondRotation, Vector2d[] second, bool reverse,
        out Fixed64 depth, out Vector2d normal, out Vector2d secondPoint)
    {
        Span<FixedPointAnchor2d> a = stackalloc FixedPointAnchor2d[2], b = stackalloc FixedPointAnchor2d[2];
        bool hit = reverse
            ? FixedConvex2dRelations.TryGetConvexContacts(secondOrigin, secondRotation, second,
                firstOrigin, firstRotation, first, b, a, out int count, out normal, out depth, out bool clamped)
            : FixedConvex2dRelations.TryGetConvexContacts(firstOrigin, firstRotation, first,
                secondOrigin, secondRotation, second, a, b, out count, out normal, out depth, out clamped);
        secondPoint = default;
        if (!hit) { Assert.Equal(0, count); return false; }
        Assert.False(clamped);
        Assert.InRange(count, 1, 2);
        for (int i = 0; i < count; i++)
        {
            Assert.True(a[i].TryGetPoint(out Vector2d firstPoint));
            Assert.True(b[i].TryGetPoint(out secondPoint));
            Vector2d difference = reverse ? secondPoint - firstPoint : firstPoint - secondPoint;
            Vector2d expected = normal * depth;
            FixedMathTestHelper.AssertWithinRange(difference.X, expected.X - Fixed64.MinIncrement, expected.X + Fixed64.MinIncrement);
            FixedMathTestHelper.AssertWithinRange(difference.Y, expected.Y - Fixed64.MinIncrement, expected.Y + Fixed64.MinIncrement);
        }
        return true;
    }

    private static Fixed64 Rotation(int value) => value == 0 ? Fixed64.Zero : value == 1 ? Fixed64.HalfPi : Fixed64.PiOver6;
    private static Vector2d Rotate(Vector2d value, Fixed64 rotation) => new(
        value.X * FixedMath.Cos(rotation) - value.Y * FixedMath.Sin(rotation),
        value.X * FixedMath.Sin(rotation) + value.Y * FixedMath.Cos(rotation));
    private static void Cycle(Vector2d[] values)
    {
        Vector2d first = values[0];
        for (int i = 0; i < values.Length - 1; i++) values[i] = values[i + 1];
        values[values.Length - 1] = first;
    }
}
