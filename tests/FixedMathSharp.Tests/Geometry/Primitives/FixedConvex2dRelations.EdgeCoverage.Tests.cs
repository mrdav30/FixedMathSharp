using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvex2dEdgeCoverageTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PartialOppositeFill_ShouldPreserveExactOriginalParametersInEitherWinding(bool reverse)
    {
        Vector2d[] polygon = { new(1, 0), new(3, 0), new(1, -2) };
        if (reverse) System.Array.Reverse(polygon);
        Assert.True(Cover(polygon, out var interval, out int sides));
        Assert.Equal(2, sides);
        EqualRatio(interval.LowerNumerator, interval.LowerDenominator, 1, 4);
        EqualRatio(interval.UpperNumerator, interval.UpperDenominator, 3, 4);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SameSideOrFilledInterior_ShouldRetainItsActualHalfNeighborhoods(bool interior)
    {
        Vector2d[] polygon = interior
            ? new[] { new Vector2d(-1, -1), new Vector2d(5, -1), new Vector2d(5, 1), new Vector2d(-1, 1) }
            : new[] { new Vector2d(1, 0), new Vector2d(3, 0), new Vector2d(1, 2) };
        Assert.True(Cover(polygon, out _, out int sides));
        Assert.Equal(interior ? 3 : 1, sides);
    }

    [Fact]
    public void SharedCorner_ShouldRemainAClosedSingletonWithoutCoveringAnOpenSpan()
    {
        Assert.True(Cover(new[] { new Vector2d(4, 0), new Vector2d(6, 0), new Vector2d(4, -2) }, out var interval, out _));
        EqualRatio(interval.LowerNumerator, interval.LowerDenominator, 1, 1);
        EqualRatio(interval.UpperNumerator, interval.UpperDenominator, 1, 1);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactFixedSinCosFrames_ShouldSupportIndependentHalfTurnsAndWorldLimitOrigins(bool atLimit)
    {
        Vector2d origin = atLimit ? new(Fixed64.MaxValue - (Fixed64)16, Fixed64.MinValue + (Fixed64)16) : Vector2d.Zero;
        Vector2d targetOrigin = new(origin.X + Fixed64.One, origin.Y);
        Assert.True(WideConvex2dRelations.TryGetEdgeCoverage(origin, Fixed64.Zero,
            new FixedSegment2d(new(0, 0), new(4, 0)), new(0, 2), targetOrigin, Fixed64.Pi,
            new[] { new Vector2d(0, 0), new Vector2d(-2, 0), new Vector2d(0, 2) }, out var interval, out int sides));
        Assert.Equal(2, sides);
        EqualRatio(interval.LowerNumerator, interval.LowerDenominator, 1, 4);
        EqualRatio(interval.UpperNumerator, interval.UpperDenominator, 3, 4);
    }

    [Fact]
    public void ObliqueFixedSinCosFrame_ShouldKeepExactPartialCoverageWithoutQuaternionConversion()
    {
        Fixed64 angle = FixedMath.DegToRad((Fixed64)37);
        Assert.True(WideConvex2dRelations.TryGetEdgeCoverage(new(7, -3), angle,
            new FixedSegment2d(new(0, 0), new(4, 0)), new(0, 2), new(7, -3), angle,
            new[] { new Vector2d(1, 0), new Vector2d(3, 0), new Vector2d(1, -2) }, out var interval, out int sides));
        Assert.Equal(2, sides);
        EqualRatio(interval.LowerNumerator, interval.LowerDenominator, 1, 4);
        EqualRatio(interval.UpperNumerator, interval.UpperDenominator, 3, 4);
    }

    [Fact]
    public void IndependentlyObliqueFrames_ShouldKeepExactlyCommutingAuthoredEdges()
    {
        Fixed64 firstAngle = FixedMath.DegToRad((Fixed64)37), secondAngle = FixedMath.DegToRad((Fixed64)(-23));
        Vector2d Rotate(Vector2d point, Fixed64 angle)
        {
            Assert.True(Vector2d.TryRotate(point, angle, out Vector2d rotated));
            return rotated;
        }
        // Integer-coordinate first rotations are exactly representable.
        // The two independent shared-S maps then commute without rounding.
        Assert.True(WideConvex2dRelations.TryGetEdgeCoverage(new(7, -3), firstAngle,
            new FixedSegment2d(Vector2d.Zero, Rotate(new(4, 0), secondAngle)), Rotate(new(0, 2), secondAngle),
            new(7, -3), secondAngle,
            new[] { Rotate(new(1, 0), firstAngle), Rotate(new(3, 0), firstAngle), Rotate(new(1, -2), firstAngle) },
            out var interval, out int sides));
        Assert.Equal(2, sides);
        EqualRatio(interval.LowerNumerator, interval.LowerDenominator, 1, 4);
        EqualRatio(interval.UpperNumerator, interval.UpperDenominator, 3, 4);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void GapsAndDegenerateInputs_ShouldNotAuthorizeCoverage(int variant)
    {
        Vector2d[] polygon = variant == 0
            ? new[] { new Vector2d(1, -1), new Vector2d(3, -1), new Vector2d(1, -2) }
            : variant == 1 ? new[] { new Vector2d(8, 0), new Vector2d(10, 0), new Vector2d(8, -2) }
            : new[] { new Vector2d(0, 0), new Vector2d(1, 0), new Vector2d(2, 0) };
        Assert.False(Cover(polygon, out _, out _));
    }

    [Fact]
    public void OneRawTransverseGap_ShouldRemainOpen()
    {
        Assert.False(WideConvex2dRelations.TryGetEdgeCoverage(Vector2d.Zero, Fixed64.Zero,
            new FixedSegment2d(new(0, 0), new(4, 0)), new(0, 2),
            new Vector2d(Fixed64.Zero, -Fixed64.FromRaw(1)), Fixed64.Zero,
            new[] { new Vector2d(1, 0), new Vector2d(3, 0), new Vector2d(1, -2) }, out _, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SourceEndpointReversal_ShouldReverseTheIntervalWithoutChangingFilledSide(bool reverse)
    {
        Assert.True(WideConvex2dRelations.TryGetEdgeCoverage(Vector2d.Zero, Fixed64.Zero,
            reverse ? new FixedSegment2d(new(4, 0), new(0, 0)) : new FixedSegment2d(new(0, 0), new(4, 0)), new(0, 2),
            Vector2d.Zero, Fixed64.Zero, new[] { new Vector2d(0, 0), new Vector2d(1, 0), new Vector2d(0, -2) }, out var interval, out int sides));
        Assert.Equal(2, sides);
        EqualRatio(interval.LowerNumerator, interval.LowerDenominator, reverse ? 3 : 0, 4);
        EqualRatio(interval.UpperNumerator, interval.UpperDenominator, reverse ? 4 : 1, 4);
    }

    [Fact]
    public void FullRawTargetOffsets_ShouldPreserveCompleteCoverage()
    {
        Assert.True(Cover(new[] { Vector2d.Zero, new Vector2d(Fixed64.MaxValue, Fixed64.Zero), new Vector2d(Fixed64.Zero, Fixed64.MinValue) }, out var interval, out int sides));
        Assert.False(interval.IsClipped);
        Assert.Equal(2, sides);
    }

    [Fact]
    public void MissingAreaOrSourceInward_ShouldNotAuthorizeCoverage()
    {
        Assert.False(Cover(System.Array.Empty<Vector2d>(), out _, out _));
        Assert.False(WideConvex2dRelations.TryGetEdgeCoverage(Vector2d.Zero, Fixed64.Zero,
            new FixedSegment2d(new(0, 0), new(4, 0)), new(2, 0), Vector2d.Zero, Fixed64.Zero,
            new[] { new Vector2d(0, 0), new Vector2d(4, 0), new Vector2d(0, -2) }, out _, out _));
    }

    private static bool Cover(Vector2d[] target, out RationalSegmentInterval interval, out int sides) =>
        WideConvex2dRelations.TryGetEdgeCoverage(Vector2d.Zero, Fixed64.Zero,
            new FixedSegment2d(new(0, 0), new(4, 0)), new(0, 2), Vector2d.Zero, Fixed64.Zero, target, out interval, out sides);

    private static void EqualRatio(Signed320 n, Signed320 d, long expectedN, long expectedD) =>
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(n, d,
            Signed320.ExtendValue(Signed192.Signed(expectedN)), Signed320.ExtendValue(Signed192.Signed(expectedD))));
}
