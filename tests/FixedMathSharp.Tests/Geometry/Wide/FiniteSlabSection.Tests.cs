using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteSlabSectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineSection_ShouldRetainOddRawHeightAdmission(bool fullHeight)
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var frame = new FiniteSlabSectionFrame(source, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, Fixed64.FromRaw(1), fullHeight);
        long boundary = fullHeight ? 1 : 2;
        Assert.True(frame.ContainsPoint(new(Fixed64.Zero, Fixed64.FromRaw(boundary), Fixed64.Zero)));
        Assert.True(frame.ContainsPoint(new(Fixed64.Zero, Fixed64.FromRaw(-boundary), Fixed64.Zero)));
        Assert.False(frame.ContainsPoint(new(Fixed64.Zero, Fixed64.FromRaw(boundary + 1), Fixed64.Zero)));
        Assert.False(frame.ContainsPoint(new(Fixed64.Zero, Fixed64.FromRaw(-boundary - 1), Fixed64.Zero)));
        Assert.Throws<ArgumentOutOfRangeException>(() => new FiniteSlabSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.Zero, fullHeight));
        Assert.Throws<ArgumentOutOfRangeException>(() => new FiniteSlabSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, (Fixed64)(fullHeight ? 8193 : 4097), fullHeight));
        var rigid = new FiniteSlabSectionFrame(ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity),
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One, fullHeight);
        Assert.True(rigid.ContainsPoint(Vector3d.Zero));
    }

    [Fact]
    public void AffineTriangleAdmission_ShouldRetainAContainedCurvedOrPrismSection()
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var frame = new FiniteSlabSectionFrame(source, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, Fixed64.Half);
        FixedTriangle horizontal = new(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        Assert.True(frame.IntersectsTriangle(horizontal));
        Assert.False(frame.IntersectsTriangle(new(new Vector3d(-8, 2, -8), new Vector3d(8, 2, -8), new Vector3d(0, 2, 8))));
        FixedTriangle vertical = new(new Vector3d(Fixed64.FromFraction(3, 2), (Fixed64)(-8), (Fixed64)(-8)),
            new Vector3d(Fixed64.FromFraction(3, 2), (Fixed64)8, (Fixed64)(-8)),
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, (Fixed64)8));
        Assert.True(frame.IntersectsTriangle(vertical));
        Span<Vector2d> offsets = stackalloc Vector2d[4] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Assert.True(frame.IntersectsTriangle(horizontal, offsets));
        Assert.True(frame.IntersectsTriangle(vertical, offsets));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    public void TriangleAdmission_ShouldUseAContainedSectionWithoutContactMaterialization(int length)
    {
        var frame = new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)length, Fixed64.One, Fixed64.One);
        FixedTriangle horizontal = new(new Vector3d(-5, 0, -5), new Vector3d(5, 0, -5), new Vector3d(0, 0, 5));
        Assert.True(frame.IntersectsTriangle(horizontal));
        Assert.True(frame.IntersectsTriangle(new(horizontal.C, horizontal.B, horizontal.A)));
        Assert.False(frame.IntersectsTriangle(new(new Vector3d(-5, 2, -5), new Vector3d(5, 2, -5), new Vector3d(0, 2, 5))));
        FixedTriangle vertical = new(new Vector3d((Fixed64).8, (Fixed64)(-3), (Fixed64)(-3)),
            new Vector3d((Fixed64).8, (Fixed64)3, (Fixed64)(-3)), new Vector3d((Fixed64).8, Fixed64.Zero, (Fixed64)3));
        Assert.True(frame.IntersectsTriangle(vertical));
        Assert.False(frame.IntersectsTriangle(new(new Vector3d(2, -3, -3), new Vector3d(2, 3, -3), new Vector3d(2, 0, 3))));
    }

    [Fact]
    public void TriangleAdmission_ShouldRetainPrismAndCollapsedCoreSections()
    {
        FixedTriangle horizontal = new(new Vector3d(-5, 0, -5), new Vector3d(5, 0, -5), new Vector3d(0, 0, 5));
        var frame = new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Span<Vector2d> offsets = stackalloc Vector2d[4]
        { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Assert.True(frame.IntersectsTriangle(horizontal, offsets));
        Assert.False(frame.IntersectsTriangle(new(new Vector3d(-5, 2, -5), new Vector3d(5, 2, -5), new Vector3d(0, 2, 5)), offsets));
        FixedTriangle vertical = new(new Vector3d(0, -3, -3), new Vector3d(0, 3, -3), new Vector3d(0, 0, 3));
        Assert.True(frame.IntersectsTriangle(vertical));
        Assert.False(frame.IntersectsTriangle(new(new Vector3d(1, -3, -3), new Vector3d(1, 3, -3), new Vector3d(1, 0, 3))));
        var core = new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, Fixed64.One);
        Assert.True(core.IntersectsTriangle(horizontal));
        Assert.False(core.IntersectsTriangle(new(new Vector3d(-5, 0, 2), new Vector3d(5, 0, 2), new Vector3d(0, 5, 2))));
    }

    private static Vector3d Point(Fixed64 x, Fixed64 y, Fixed64 z) => new(x, y, z);
    private static Fixed64 Parameter(ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        Span<ulong> values = stackalloc ulong[2 * FiniteSectionPoint.Words]; Span<int> signs = stackalloc int[2];
        var scaled = new ContactQuadratic(values, signs);
        ContactQuadratic.Scale(numerator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(Fixed64.One))), scaled);
        return ContactQuadratic.RoundRatio(scaled, denominator, root);
    }
    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    public void Section_ShouldRetainFlatCapsAndRoundedPlanarBoundary(int length)
    {
        var frame = new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)length, Fixed64.One, Fixed64.Half);
        Assert.True(frame.ContainsPoint(Point(Fixed64.One, Fixed64.Half, Fixed64.Zero)));
        Assert.False(frame.ContainsPoint(Point(Fixed64.One, Fixed64.Half + Fixed64.Epsilon, Fixed64.Zero)));
        Assert.False(frame.ContainsPoint(Point(Fixed64.One + Fixed64.Epsilon, Fixed64.Zero, Fixed64.Zero)));
        Assert.True(frame.IntersectsSegment(new FixedSegment(Point(-Fixed64.Two, Fixed64.Zero, Fixed64.Zero), Point(Fixed64.Two, Fixed64.Zero, Fixed64.Zero))));
        Assert.False(frame.IntersectsSegment(new FixedSegment(Point(-Fixed64.Two, Fixed64.One, Fixed64.Zero), Point(Fixed64.Two, Fixed64.One, Fixed64.Zero))));
        Assert.True(frame.IntersectsSegment(new FixedSegment(Point(Fixed64.Zero, -Fixed64.Two, Fixed64.Zero), Point(Fixed64.Zero, Fixed64.Two, Fixed64.Zero))));
        Assert.False(frame.IntersectsSegment(new FixedSegment(Point(Fixed64.Two, -Fixed64.Two, Fixed64.Zero), Point(Fixed64.Two, Fixed64.Two, Fixed64.Zero))));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    public void FirstParameter_ShouldIntersectAxialAndRadialIntervalsWithoutRounding(int length)
    {
        var frame = new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)length, Fixed64.One, Fixed64.Half);
        Span<ulong> values = stackalloc ulong[4 * FiniteSectionPoint.Words];
        Span<int> signs = stackalloc int[4];
        Span<ulong> root = stackalloc ulong[FiniteSectionPoint.RootWords];
        var numerator = ContactQuadratic.At(values, signs, 0, FiniteSectionPoint.Words);
        var denominator = ContactQuadratic.At(values, signs, 1, FiniteSectionPoint.Words);
        Assert.True(frame.TryGetFirstSegmentParameter(new FixedSegment(Point(-Fixed64.Two, -Fixed64.One, Fixed64.Zero), Point(Fixed64.Two, Fixed64.One, Fixed64.Zero)), numerator, denominator, root));
        Assert.Equal((Fixed64)0.25, Parameter(numerator, denominator, root));
        Assert.True(frame.TryGetFirstSegmentParameter(new FixedSegment(Point(Fixed64.Two, Fixed64.Zero, Fixed64.Zero), Point(-Fixed64.Two, Fixed64.Zero, Fixed64.Zero)), numerator, denominator, root));
        Assert.Equal((Fixed64)0.25, Parameter(numerator, denominator, root));
        Assert.True(frame.TryGetFirstSegmentParameter(new FixedSegment(Point(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero), Point(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)), numerator, denominator, root));
        Assert.Equal(Fixed64.Zero, Parameter(numerator, denominator, root));
        Assert.False(frame.TryGetFirstSegmentParameter(new FixedSegment(Point(Fixed64.Two, Fixed64.Zero, Fixed64.Zero), Point(Fixed64.Two, Fixed64.Zero, Fixed64.Zero)), numerator, denominator, root));
    }

    [Fact]
    public void DegenerateRadius_ShouldRetainFiniteCoreAndAxialTouch()
    {
        var frame = new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, Fixed64.Half);
        Assert.True(frame.ContainsPoint(Point(Fixed64.Zero, Fixed64.Half, Fixed64.One)));
        Assert.False(frame.ContainsPoint(Point(Fixed64.Epsilon, Fixed64.Zero, Fixed64.Zero)));
        Assert.True(frame.IntersectsSegment(new FixedSegment(Point(-Fixed64.One, Fixed64.Zero, Fixed64.Zero), Point(Fixed64.One, Fixed64.Zero, Fixed64.Zero))));
        Assert.False(frame.IntersectsSegment(new FixedSegment(Point(-Fixed64.One, Fixed64.One, Fixed64.Zero), Point(Fixed64.One, Fixed64.One, Fixed64.Zero))));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void NonpositiveThickness_ShouldRejectInvalidExtrusion(int thickness)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, (Fixed64)thickness));
    }

    [Fact]
    public void CanonicalSectionPoint_ShouldIgnoreSegmentOrderAndRetainClosedRimTouch()
    {
        var frame = new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, Fixed64.Half);
        Span<ulong> firstValues = stackalloc ulong[FiniteSectionPoint.StorageWords]; Span<int> firstSigns = stackalloc int[FiniteSectionPoint.SignCount];
        Span<ulong> secondValues = stackalloc ulong[FiniteSectionPoint.StorageWords]; Span<int> secondSigns = stackalloc int[FiniteSectionPoint.SignCount];
        var first = new FiniteSectionPoint(firstValues, firstSigns); var second = new FiniteSectionPoint(secondValues, secondSigns);
        var segment = new FixedSegment(Point(-Fixed64.Two, -Fixed64.One, Fixed64.Zero), Point(Fixed64.Two, Fixed64.One, Fixed64.Zero));
        Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(segment, frame, first));
        Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(segment.End, segment.Start), frame, second));
        Assert.Equal(0, FiniteSectionPoint.CompareLexicographic(first, second));
        Assert.Equal(-Fixed64.One, ContactQuadratic.RoundRatio(first.Coordinate(0), first.Denominator, first.Root));
        Assert.Equal(-Fixed64.Half, ContactQuadratic.RoundRatio(first.Coordinate(1), first.Denominator, first.Root));
        Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(Point(-Fixed64.One, Fixed64.Half, Fixed64.Two), Point(Fixed64.One, Fixed64.Half, Fixed64.Two)), frame, first));
        Assert.Equal(Fixed64.Zero, ContactQuadratic.RoundRatio(first.Coordinate(0), first.Denominator, first.Root));
        Assert.False(FiniteSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(Point(-Fixed64.One, Fixed64.One, Fixed64.Two), Point(Fixed64.One, Fixed64.One, Fixed64.Two)), frame, first));
    }
}
