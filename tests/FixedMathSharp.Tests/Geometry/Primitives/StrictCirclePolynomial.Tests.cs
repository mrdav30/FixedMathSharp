using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class StrictCirclePolynomialTests
{
    [Fact]
    public void CommonSigns_AcceptConstantNegativeRadialOnNonemptyStrip() =>
        Assert.True(HasPoint(new[] { -1, 0, 0, 0, 0 }, new[] { 1, 0, 0 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_RejectConstantPositiveRadialOnNonemptyStrip() =>
        Assert.False(HasPoint(new[] { 1, 0, 0, 0, 0 }, new[] { 1, 0, 0 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_RejectConstantNegativeRadialOnEmptyStrip() =>
        Assert.False(HasPoint(new[] { -1, 0, 0, 0, 0 }, new[] { 0, 1, 0 }, new[] { 0, -1, 0 }));

    [Fact]
    public void CommonSigns_RejectConstantPositiveRadialOnEmptyStrip() =>
        Assert.False(HasPoint(new[] { 1, 0, 0, 0, 0 }, new[] { 0, 1, 0 }, new[] { 0, -1, 0 }));

    [Fact]
    public void CommonSigns_RejectIdenticallyZeroRadial() =>
        Assert.False(HasPoint(new[] { 0, 0, 0, 0, 0 }, new[] { 1, 0, 0 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_RejectFourthPowerTangency() =>
        Assert.False(HasPoint(new[] { 1, -4, 6, -4, 1 }, new[] { 1, 0, 0 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_CountThirdPowerSignChange() =>
        Assert.True(HasPoint(new[] { 1, -3, 3, -1, 0 }, new[] { 0, 1, 0 }, new[] { 2, -1, 0 }));

    [Fact]
    public void CommonSigns_AllowEitherSideOfDoubleStripRoot() =>
        Assert.True(HasPoint(new[] { -1, 0, 1, 0, 0 }, new[] { 0, 0, 1 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_AllowRootlessPositiveQuadraticStrip() =>
        Assert.True(HasPoint(new[] { -1, 0, 1, 0, 0 }, new[] { 1, 0, 1 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_PreserveUpperBoundaryWhenInsertingDuplicateLowerRoot() =>
        Assert.False(HasPoint(new[] { 4, -1, 0, 0, 0 }, new[] { 0, -2, 1 }, new[] { 0, 3, -1 }));

    [Fact]
    public void CommonSigns_CountTripleRootAtZero() =>
        Assert.True(HasPoint(new[] { 0, 0, 0, -1, 1 }, new[] { 1, 0, 0 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_RejectDisjointRadialAndAxialIntervals() =>
        Assert.False(HasPoint(new[] { -1, 0, 1, 0, 0 }, new[] { -2, 1, 0 }, new[] { 3, -1, 0 }));

    [Fact]
    public void CommonSigns_AcceptCoincidentOpenIntervals() =>
        Assert.True(HasPoint(new[] { -1, 0, 1, 0, 0 }, new[] { 1, 1, 0 }, new[] { 1, -1, 0 }));

    [Fact]
    public void CommonSigns_RejectRadialTangency() =>
        Assert.False(HasPoint(new[] { 1, -2, 1, 0, 0 }, new[] { 1, 0, 0 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_RejectOnlySharedBoundaryPoint() =>
        Assert.False(HasPoint(new[] { -1, 0, 1, 0, 0 }, new[] { -1, 1, 0 }, new[] { 2, -1, 0 }));

    [Fact]
    public void CommonSigns_AcceptBetweenIrrationalSlabRoots() =>
        Assert.True(HasPoint(new[] { 1, 0, -1, 0, 0 }, new[] { 2, 0, -1 }, new[] { 0, 1, 0 }));

    [Fact]
    public void CommonSigns_RejectBetweenIrrationalSlabRoots() =>
        Assert.False(HasPoint(new[] { 2, 0, -1, 0, 0 }, new[] { 2, 0, -1 }, new[] { 0, 1, 0 }));

    [Fact]
    public void CommonSigns_CountOddRootsAfterRepeatedEndpoint() =>
        Assert.True(HasPoint(new[] { 4, 8, 3, -2, -1 }, new[] { 1, 1, 0 }, new[] { 3, -1, 0 }));

    [Fact]
    public void CommonSigns_DoNotCountTwoDoubleRootsAsSignChanges() =>
        Assert.False(HasPoint(new[] { 4, 0, -4, 0, 1 }, new[] { 1, 0, 0 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_CountTripleAndSimpleRoots() =>
        Assert.True(HasPoint(new[] { -2, 5, -3, -1, 1 }, new[] { 3, 1, 0 }, new[] { 0, -1, 0 }));

    [Fact]
    public void CommonSigns_RejectIdenticallyZeroStrip() =>
        Assert.False(HasPoint(new[] { -1, 0, 0, 0, 0 }, new[] { 0, 0, 0 }, new[] { 1, 0, 0 }));

    [Fact]
    public void CommonSigns_MatchIndependentFactorizedIntervalOracle()
    {
        // Every boundary is an integer, so the half-integer grid contains a
        // witness in every nonempty open sign cell. Expectations use the
        // original factors, not the expanded polynomial supplied to the kernel.
        int[][] factorizations =
        {
            System.Array.Empty<int>(), new[] { 0 }, new[] { -1 }, new[] { 2 },
            new[] { 0, 0 }, new[] { 1, 1 }, new[] { -1, 2 },
            new[] { 0, 0, 0 }, new[] { 1, 1, 1 }, new[] { 0, 0, 2 },
            new[] { 0, 2, 2 }, new[] { -2, 0, 2 },
            new[] { 0, 0, 0, 0 }, new[] { 1, 1, 1, 1 },
            new[] { 0, 0, 0, 1 }, new[] { 0, 1, 1, 1 },
            new[] { 0, 0, 1, 1 }, new[] { -1, -1, 1, 1 },
            new[] { -2, -1, 0, 1 }, new[] { 0, 0, 1, 2 }, new[] { -2, 0, 0, 1 },
        };
        int[][] lowerStrips =
        {
            new[] { 1, 0, 0 }, new[] { 0, -2, 1 }, new[] { 4, 0, -1 }, new[] { 0, -2, 1 },
            new[] { 1, 0, -1 }, new[] { 1, -2, 1 }, new[] { 0, 0, -1 }, new[] { 1, 0, 1 },
        };
        int[][] upperStrips =
        {
            new[] { 1, 0, 0 }, new[] { 0, 3, -1 }, new[] { 0, 3, -1 }, new[] { 0, -3, 1 },
            new[] { 1, 0, -1 }, new[] { -2, 3, -1 }, new[] { 1, 0, 0 }, new[] { 1, 0, 0 },
        };
        int sample = 0;
        foreach (int[] roots in factorizations)
        foreach (int leading in new[] { -1, 1 })
        foreach (int strip in new[] { 0, 1, 2, 3, 4, 5, 6, 7 })
        {
            var polynomial = new int[5];
            polynomial[0] = leading;
            for (int index = 0; index < roots.Length; index++)
            {
                for (int coefficient = index + 1; coefficient > 0; coefficient--)
                    polynomial[coefficient] = polynomial[coefficient - 1] - roots[index] * polynomial[coefficient];
                polynomial[0] *= -roots[index];
            }
            bool expected = false;
            // All real roots lie in [-3,3]; +/-4.5 also sample both unbounded cells.
            for (int twiceParameter = -9; twiceParameter <= 9; twiceParameter++)
            {
                long value = leading;
                for (int index = 0; index < roots.Length; index++)
                    value *= twiceParameter - 2 * roots[index];
                expected |= value < 0 && EvaluateQuadraticAtHalfParameter(lowerStrips[strip], twiceParameter) > 0
                    && EvaluateQuadraticAtHalfParameter(upperStrips[strip], twiceParameter) > 0;
            }
            bool actual = HasPoint(polynomial, lowerStrips[strip], upperStrips[strip]);
            Assert.True(actual == expected, $"Factorized interval sample {sample}: expected {expected}, actual {actual}.");
            sample++;
        }
    }

    private static long EvaluateQuadraticAtHalfParameter(int[] coefficients, int twiceParameter) =>
        4L * coefficients[0] + 2L * coefficients[1] * twiceParameter
        + (long)coefficients[2] * twiceParameter * twiceParameter;

    private static bool HasPoint(int[] radial, int[] lower, int[] upper)
    {
        var wideRadial = new Signed832[5];
        var wideLower = new Signed320[3];
        var wideUpper = new Signed320[3];
        for (int index = 0; index < 5; index++)
            wideRadial[index] = Signed832.ExtendValue(Signed192.Signed(radial[index]));
        for (int index = 0; index < 3; index++)
        {
            wideLower[index] = Signed320.ExtendValue(Signed192.Signed(lower[index]));
            wideUpper[index] = Signed320.ExtendValue(Signed192.Signed(upper[index]));
        }
        return WideFiniteAxisIntersection.HasStrictCirclePolynomialPoint(wideRadial, wideLower, wideUpper);
    }
}
