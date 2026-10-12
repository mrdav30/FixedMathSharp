using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteSlabSectionBoundaryTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PrismPointAdmission_ShouldRespectEveryClosedWallAndBothCaps(bool reverse)
    {
        Vector2d[] offsets = Square(reverse);
        var frame = Frame();
        Assert.True(frame.ContainsPoint(Vector3d.Zero, Array.Empty<Vector2d>()));
        Assert.True(frame.ContainsPoint(Vector3d.Zero, offsets));
        for (int axis = 0; axis < 3; axis++)
            for (int sign = -1; sign <= 1; sign += 2)
            {
                Vector3d boundary = Vector3d.Zero; boundary[axis] = (Fixed64)sign;
                Assert.True(frame.ContainsPoint(boundary, offsets));
                boundary[axis] = (Fixed64)sign * (Fixed64.One + Fixed64.FromRaw(1));
                Assert.False(frame.ContainsPoint(boundary, offsets));
            }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ProjectedPrismInterval_ShouldRetainTangencyAndRejectParallelOutsideWalls(bool reverse)
    {
        Vector2d[] offsets = Square(reverse);
        var frame = Frame();
        Assert.Equal(Fixed64.One / 4, FirstParameter(frame, new(new(-2, 2, 0), new(2, 2, 0)), offsets, true));
        Assert.Equal(Fixed64.One / 4, FirstParameter(frame, new(new(2, 2, 0), new(-2, 2, 0)), offsets, true));
        Assert.Null(FirstParameter(frame, new(new(-2, 2, 0), new(2, 2, 0)), offsets, false));
        Assert.Equal(Fixed64.Half, FirstParameter(frame, new(new(0, 0, 2), new(2, 0, 0)), offsets, false));
        Assert.Null(FirstParameter(frame, new(new(-2, 0, 2), new(2, 0, 2)), offsets, true));
        Assert.Null(FirstParameter(frame, new(new(-2, 0, 5), new(5, 0, -2)), offsets, true));
        Assert.Null(FirstParameter(frame, new(new(2, -2, 0), new(2, 2, 0)), offsets, false));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void InvalidPrism_ShouldRejectInsufficientOrCollinearOffsets(int count)
    {
        var offsets = new Vector2d[count];
        Assert.Throws<ArgumentException>(() => FiniteSlabSectionFrame.GetPrismWinding(offsets));
        Assert.Throws<ArgumentException>(() => FiniteSlabSectionFrame.GetPrismWinding(
            new[] { new Vector2d(-1, 0), Vector2d.Zero, new Vector2d(1, 0) }));
    }

    [Fact]
    public void Winding_ShouldSkipAnInitialCollinearFanWithoutLosingAdmission()
    {
        Vector2d[] offsets = { new(-1, -1), new(0, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Assert.Equal(1, FiniteSlabSectionFrame.GetPrismWinding(offsets));
        Array.Reverse(offsets);
        Assert.Equal(-1, FiniteSlabSectionFrame.GetPrismWinding(offsets));
        Assert.True(Frame().ContainsPoint(Vector3d.Zero, offsets));
    }

    [Theory]
    [InlineData(-3, false)]
    [InlineData(-2, true)]
    [InlineData(0, true)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    public void ContainedPrismPlaneSection_ShouldRespectBothSupportExtrema(int constant, bool intersects)
    {
        FixedTriangle triangle = new(new Vector3d(constant - 10, 10, -10),
            new Vector3d(constant + 10, -10, -10), new Vector3d(constant, 0, 10));
        var frame = Frame();
        Assert.Equal(intersects, frame.IntersectsTriangle(triangle, Square(false)));
        Assert.Equal(intersects, frame.IntersectsTriangle(new(triangle.C, triangle.B, triangle.A), Square(true)));
    }

    [Theory]
    [InlineData(-3, false)]
    [InlineData(-2, false)]
    [InlineData(-1, true)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(3, false)]
    public void RadialPlaneSection_ShouldRespectSignedSupportAndZeroAxialNormal(int constant, bool intersects)
    {
        // x+z=constant has support sqrt(2), with no Y component.
        FixedTriangle triangle = new(new Vector3d(constant - 10, -10, 10),
            new Vector3d(constant + 10, -10, -10), new Vector3d(constant, 10, 0));
        var frame = Frame();
        Assert.Equal(intersects, frame.IntersectsTriangle(triangle));
        Assert.Equal(intersects, frame.IntersectsTriangle(new(triangle.C, triangle.B, triangle.A)));
    }

    [Fact]
    public void DegenerateOutsideTriangle_ShouldHaveNoPlaneSection()
    {
        Vector3d point = new(2, 2, 2);
        Assert.False(Frame().IntersectsTriangle(new(point, point, point)));
        Assert.False(Frame().IntersectsTriangle(new(point, point, point), Square(false)));
        Assert.True(Frame().IntersectsTriangle(new(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero)));
    }

    [Theory]
    [InlineData(-3, false)]
    [InlineData(0, true)]
    [InlineData(3, false)]
    public void NegativeTiltedPlaneNormals_ShouldRetainSignedSlabSupports(int constant, bool intersects)
    {
        FixedTriangle xy = new(new Vector3d(constant + 10, 10, -10),
            new Vector3d(constant - 10, -10, -10), new Vector3d(constant, 0, 10));
        FixedTriangle xz = new(new Vector3d(constant + 10, -10, 10),
            new Vector3d(constant - 10, -10, -10), new Vector3d(constant, 10, 0));
        var frame = Frame();
        Assert.Equal(intersects, frame.IntersectsTriangle(xy));
        Assert.Equal(intersects, frame.IntersectsTriangle(xz));
        Assert.Equal(intersects, frame.IntersectsTriangle(xy, Square(false)));
    }

    [Fact]
    public void FullRangePrism_ShouldRetainClosedExtremePointsAndFiniteIntervals()
    {
        Fixed64 low = Fixed64.MinValue, high = Fixed64.MaxValue;
        var frame = new FiniteSlabSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, high, high);
        Vector2d[] offsets = { new(low, low), new(high, low), new(high, high), new(low, high) };
        Assert.True(frame.ContainsPoint(new(high, high, low), offsets));
        Assert.False(frame.ContainsPoint(new(high, low, low), offsets));
        Assert.Equal(Fixed64.Zero, FirstParameter(frame, new(new(low, Fixed64.Zero, low), new(high, Fixed64.Zero, high)), offsets, false));
        Assert.True(frame.ContainsPoint(Vector3d.Right * high));
    }

    private static FiniteSlabSectionFrame Frame() => new(Vector3d.Zero, FixedQuaternion.Identity,
        Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);

    private static Vector2d[] Square(bool reverse)
    {
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        if (reverse) Array.Reverse(offsets);
        return offsets;
    }

    private static Fixed64? FirstParameter(FiniteSlabSectionFrame frame, FixedSegment segment,
        ReadOnlySpan<Vector2d> offsets, bool projected)
    {
        Span<ulong> values = stackalloc ulong[6 * FiniteSectionPoint.Words]; Span<int> signs = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[FiniteSectionPoint.RootWords];
        ContactQuadratic numerator = ContactQuadratic.At(values, signs, 0, FiniteSectionPoint.Words),
            denominator = ContactQuadratic.At(values, signs, 1, FiniteSectionPoint.Words);
        bool admitted = projected ? frame.TryGetFirstProjectedSegmentParameter(segment, offsets, numerator, denominator, root)
            : frame.TryGetFirstSegmentParameter(segment, offsets, numerator, denominator, root);
        if (!admitted) return null;
        var scaled = ContactQuadratic.At(values, signs, 2, FiniteSectionPoint.Words);
        ContactQuadratic.Scale(numerator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(Fixed64.One))), scaled);
        return ContactQuadratic.RoundRatio(scaled, denominator, root);
    }
}
