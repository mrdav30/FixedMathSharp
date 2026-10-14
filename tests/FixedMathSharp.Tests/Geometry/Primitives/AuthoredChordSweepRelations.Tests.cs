using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class AuthoredChordSweepRelationsTests
{
    [Theory]
    [InlineData(3, 4, 5, false)]
    [InlineData(3, 4, 5, true)]
    [InlineData(5, 12, 13, false)]
    [InlineData(5, 12, 13, true)]
    [InlineData(7, 24, 25, false)]
    [InlineData(7, 24, 25, true)]
    public void CirclePolygonChord_ExactTangencyRetainsPhysicalDistance(
        int a, int b, int radius, bool reverse)
    {
        Vector2d tangent = new(-b, a), travel = new(a, b);
        Vector2d start = tangent - travel, displacement = travel * Fixed64.Two;
        if (reverse) { start = tangent + travel; displacement = -displacement; }
        for (int frame = 0; frame < 2; frame++)
        {
            Vector2d shift = frame == 0 ? Vector2d.Zero : new(0, 200);
            Vector2d[] vertices = { new Vector2d(0, 0) - shift, new Vector2d(100, 0) - shift,
                new Vector2d(100, -100) - shift, new Vector2d(0, -100) - shift };
            Assert.True(FixedConvex2dRelations.TryGetSweptCenteredCapsuleFirstDistanceCore(
                start, Vector2d.Right, Fixed64.Zero, (Fixed64)radius, displacement,
                (Fixed64)(radius * 2), shift, Fixed64.Zero, vertices,
                out Fixed64 distance, out Vector2d normal, out Vector2d contact, authoredDisplacement: true));
            Assert.Equal((Fixed64)radius, distance);
            Assert.Equal(new Vector2d(Fixed64.FromFraction(-b, radius), Fixed64.FromFraction(a, radius)), normal);
            Assert.Equal(-shift, contact);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CapsuleChord_ExactTangencyMatchesSegmentAndCenteredTargets(bool centeredTarget, bool nonzeroCore)
    {
        Vector2d start = new(-7, nonzeroCore ? 0 : -1), displacement = new(6, 8);
        Fixed64 axisLength = nonzeroCore ? Fixed64.Two : Fixed64.Zero;
        bool found = centeredTarget
            ? WideFiniteAxisIntersection.TryGetSweptCenteredCapsulesFirstDistance(
                start, Vector2d.Forward, axisLength, (Fixed64)5, displacement, (Fixed64)10,
                Vector2d.Zero, Vector2d.Forward, Fixed64.Zero, Fixed64.Zero,
                out Fixed64 distance, out Vector2d normal, authoredDisplacement: true)
            : WideFiniteAxisIntersection.TryGetSweptCenteredCapsuleSegmentFirstDistance(
                start, Vector2d.Forward, axisLength, (Fixed64)5, displacement, (Fixed64)10,
                new FixedSegment2d(Vector2d.Zero, Vector2d.Zero), Fixed64.Zero,
                out distance, out normal, authoredDisplacement: true);

        Assert.True(found);
        Assert.Equal((Fixed64)5, distance);
        Assert.Equal(new Vector2d(Fixed64.FromFraction(4, 5), -Fixed64.FromFraction(3, 5)), normal);
    }

    [Fact]
    public void ConvexChordSweep_MaterializesDistanceAndTargetContactOnAuthoredPath()
    {
        Vector2d[] source =
        {
            new(-Fixed64.Quarter, -Fixed64.Quarter), new(Fixed64.Quarter, -Fixed64.Quarter),
            new(Fixed64.Quarter, Fixed64.Quarter), new(-Fixed64.Quarter, Fixed64.Quarter)
        };
        Vector2d[] target = { new(0, -100), new(100, -100), new(100, 100), new(0, 100) };

        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistanceCore(
            new Vector2d(-2, 0), Fixed64.Zero, source, new Vector2d(5, 12), (Fixed64)13,
            Vector2d.Zero, Fixed64.Zero, target, out Fixed64 distance, out Vector2d normal,
            out Vector2d contact, authoredDisplacement: true));

        Assert.Equal(Fixed64.FromFraction(91, 20), distance);
        Assert.Equal(Vector2d.Left, normal);
        Assert.Equal(Fixed64.Zero, contact.X);
    }
}
