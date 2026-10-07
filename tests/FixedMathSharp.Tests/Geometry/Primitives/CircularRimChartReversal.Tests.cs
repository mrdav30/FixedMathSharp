//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CircularRimChartReversalTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ReversedChart_EqualsIndependentSwappedBasisBuildIncludingZeroCoefficients(bool wide)
    {
        var fixtures = new[]
        {
            (Offset: (3L, -7L, 11L), First: (2L, 5L, 0L), Second: (-3L, 0L, 5L), Radius: 5L),
            (Offset: (-3L, 7L, -11L), First: (-2L, -5L, 0L), Second: (3L, 0L, -5L), Radius: 5L),
            (Offset: (0L, 0L, 0L), First: (2L, 5L, 0L), Second: (-3L, 0L, 5L), Radius: 5L),
            (Offset: (3L, -7L, 11L), First: (0L, 0L, 0L), Second: (2L, 5L, -3L), Radius: 5L),
            (Offset: (3L, -7L, 11L), First: (2L, 5L, -3L), Second: (0L, 0L, 0L), Radius: 5L),
            (Offset: (3L, -7L, 11L), First: (2L, 5L, -3L), Second: (2L, 5L, -3L), Radius: 0L)
        };
        foreach (var fixture in fixtures)
        {
            // Dirty trailing admission slots belong to the caller. Reversal
            // must touch only the 26 shared slots, including nominal zero ends.
            var actual = new ulong[30 * CylinderContactAlgebra.Words];
            var actualSigns = new sbyte[30];
            Array.Fill(actual, 0xA5A5A5A5A5A5A5A5UL);
            Array.Fill(actualSigns, (sbyte)-1);
            var expected = (ulong[])actual.Clone();
            var expectedSigns = (sbyte[])actualSigns.Clone();
            WideAxis3 offset = Axis(fixture.Offset, wide), first = Axis(fixture.First, wide), second = Axis(fixture.Second, wide);
            Signed320 radius = Scalar(fixture.Radius, wide);
            Signed192 scale = Signed192.Signed(1L << 32);
            CircularRimContactAlgebra.BuildParameter(offset, radius, scale, first, second, actual, actualSigns);
            CircularRimContactAlgebra.BuildParameter(offset, radius, scale, second, first, expected, expectedSigns);
            var original = (ulong[])actual.Clone();
            var originalSigns = (sbyte[])actualSigns.Clone();

            CircularRimContactAlgebra.ReverseParameterChart(actual, actualSigns);
            Assert.Equal(expected, actual);
            Assert.Equal(expectedSigns, actualSigns);
            CircularRimContactAlgebra.ReverseParameterChart(actual, actualSigns);
            Assert.Equal(original, actual);
            Assert.Equal(originalSigns, actualSigns);
        }
    }

    private static WideAxis3 Axis((long X, long Y, long Z) value, bool wide) =>
        new(Scalar(value.X, wide), Scalar(value.Y, wide), Scalar(value.Z, wide));

    private static Signed320 Scalar(long value, bool wide)
    {
        if (!wide || value == 0) return Signed320.ExtendValue(Signed192.Signed(value));
        // Up to 196 significant bits with nonzero lower limbs exercise carries
        // within the production chart-direction bound (<2^198).
        ulong magnitude = (ulong)Math.Abs(value);
        var positive = new Signed320(0, magnitude, ulong.MaxValue - 17, ulong.MaxValue - 5, magnitude);
        return value < 0 ? WideArithmetic.Negate(positive) : positive;
    }
}
