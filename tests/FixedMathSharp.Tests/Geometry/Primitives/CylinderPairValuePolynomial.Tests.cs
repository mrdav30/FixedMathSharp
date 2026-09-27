using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairValuePolynomialTests
{
    [Fact]
    public void Build_MatchesIndependentConicDeterminantAndDiscriminant()
    {
        // a=(1,2,2), u=(2,-1,0), w=(2,4,-5), b=(2,-3,4),
        // c=(3,-2,5), radii 7 and 11. A sign error in any mixed term
        // changes these independently expanded integer coefficients.
        BigInteger[] invariants = { 5, 45, 29, 49, 121, 38, 8, -27, 7, -28, 32 };
        AssertConstruction(invariants, 16);
    }

    [Fact]
    public void Build_PreservesCoplanarSquaredFactorAndRawCoefficientScale()
    {
        // a=Z,u=X,w=Y,b=(3,0,4),c=(6,0,2),radii 2 and 3.
        // c.u*b.w-c.w*b.u=0: det(T) vanishes identically. Removing
        // repeated factors or primitive-normalizing would fail this oracle.
        BigInteger[] invariants = { 1, 1, 25, 4, 9, 40, 6, 0, 3, 0, 26 };
        BigInteger[][] pencil = IndependentPencil(invariants);
        Assert.All(pencil[0], coefficient => Assert.Equal(BigInteger.Zero, coefficient));
        BigInteger[] expected = Scale(Multiply(Multiply(pencil[1], pencil[1]),
            Subtract(Multiply(pencil[2], pencil[2]),
                Scale(Multiply(pencil[3], pencil[1]), 4))), 3);
        Assert.Equal(Pad(expected), Construct(invariants, 16, out int degree));
        Assert.Equal(8, degree);
    }

    [Fact]
    public void Build_PropagatesHighWordsFromExactScaledGeometry()
    {
        BigInteger[] invariants = { 5, 45, 29, 49, 121, 38, 8, -27, 7, -28, 32 };
        // Scale both plane basis vectors by 2^70, b by 2^80, and all
        // distances by 2^90. Every invariant still describes exact geometry.
        int[] shifts = { 140, 140, 160, 180, 180, 180, 160, 160, 150, 150, 170 };
        for (int index = 0; index < invariants.Length; index++)
            invariants[index] <<= shifts[index];
        AssertConstruction(invariants, 128);
    }

    [Fact]
    public void Build_TrimsNonzeroParallelPencilToDegreeSix()
    {
        BigInteger[] invariants = { 1, 1, 1, 4, 9, 50, 3, 4, 0, 0, 5 };
        BigInteger[] actual = Construct(invariants, 16, out int degree);
        Assert.Equal(6, degree);
        Assert.Equal(Pad(IndependentDiscriminant(invariants)), actual);
        Assert.True(actual[6] > 0);
    }

    [Fact]
    public void Build_ClearsDirtySlotsAndReportsIdenticallyZeroCoaxialPencil()
    {
        BigInteger[] invariants = { 1, 1, 1, 4, 9, 25, 0, 0, 0, 0, 5 };
        BigInteger[] actual = Construct(invariants, 16, out int degree);
        Assert.Equal(-1, degree);
        Assert.All(actual, coefficient => Assert.Equal(BigInteger.Zero, coefficient));
        Assert.Equal(Pad(IndependentDiscriminant(invariants)), actual);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void Build_DirectionalDerivativeMatchesExactParameterCoefficient(int component)
    {
        BigInteger[] invariants = { 5, 45, 29, 49, 121, 38, 8, -27, 7, -28, 32 };
        var direction = new BigInteger[CylinderPairValueDerivative.InvariantCount];
        direction[component] = 1;
        AssertDirectionalConstruction(invariants, direction);
    }

    [Fact]
    public void Build_MixedDirectionAccumulatesBothProductRuleTerms()
    {
        BigInteger[] invariants = { 5, 45, 29, 49, 121, 38, 8, -27, 7, -28, 32 };
        // c' = (-2,3,1), together with nonzero squared-radius derivatives.
        BigInteger[] direction = { 1, -2, -14, -7, 3, -9 };
        AssertDirectionalConstruction(invariants, direction);
    }

    [Fact]
    public void Build_ZeroDirectionClearsDirtyDerivativeWithoutChangingValue()
    {
        BigInteger[] invariants = { 5, 45, 29, 49, 121, 38, 8, -27, 7, -28, 32 };
        AssertDirectionalConstruction(invariants, new BigInteger[CylinderPairValueDerivative.InvariantCount]);
    }

    [Fact]
    public void Build_ZeroValuedInputStillContributesItsNonzeroDerivative()
    {
        // c=(1,2,5) is perpendicular to u, but its c.u derivative is not
        // zero. Skipping dual products on the value sign loses this term.
        BigInteger[] invariants = { 5, 45, 29, 49, 121, 30, 0, -15, 7, -28, 16 };
        BigInteger[] direction = { 0, 0, 0, 1, 0, 0 };
        AssertDirectionalConstruction(invariants, direction);
    }

    [Fact]
    public void Build_DirectionalDerivativePreservesHighWordProductRuleCarries()
    {
        BigInteger[] invariants = { 5, 45, 29, 49, 121, 38, 8, -27, 7, -28, 32 };
        int[] shifts = { 140, 140, 160, 180, 180, 180, 160, 160, 150, 150, 170 };
        BigInteger[] direction = { 1, -2, -14, -7, 3, -9 };
        int[] derivativeShifts = { 180, 180, 180, 160, 160, 170 };
        for (int index = 0; index < invariants.Length; index++)
            invariants[index] <<= shifts[index];
        for (int index = 0; index < direction.Length; index++)
            direction[index] <<= derivativeShifts[index];
        AssertDirectionalConstruction(invariants, direction, 128);
    }

    private static void AssertDirectionalConstruction(BigInteger[] invariants, BigInteger[] direction,
        int words = 16)
    {
        var actualDerivative = new BigInteger[CylinderPairValuePolynomial.CoefficientCount];
        BigInteger[] actual = Construct(invariants, words, out int degree, direction, actualDerivative);
        Assert.Equal(8, degree);
        Assert.Equal(Pad(IndependentDiscriminant(invariants)), actual);
        Assert.Equal(Pad(IndependentDirectionalDerivative(invariants, direction)), actualDerivative);
    }

    private static void AssertConstruction(BigInteger[] invariants, int words)
    {
        BigInteger[] expected = Pad(IndependentDiscriminant(invariants));
        BigInteger[] actual = Construct(invariants, words, out int degree);
        Assert.Equal(expected, actual);
        Assert.Equal(8, degree);
        Assert.True(actual[8] > 0);
    }

    private static BigInteger[] Construct(BigInteger[] invariants, int words, out int degree,
        BigInteger[]? direction = null, BigInteger[]? actualDerivative = null)
    {
        var input = new ulong[words * CylinderPairValuePolynomial.InvariantCount];
        var inputSigns = new sbyte[CylinderPairValuePolynomial.InvariantCount];
        Encode(invariants, words, input, inputSigns);
        var output = new ulong[words * CylinderPairValuePolynomial.CoefficientCount];
        var signs = new sbyte[CylinderPairValuePolynomial.CoefficientCount];
        var scratch = new ulong[words * CylinderPairValuePolynomial.ScratchCoefficientCount];
        var scratchSigns = new sbyte[CylinderPairValuePolynomial.ScratchCoefficientCount];
        Array.Fill(output, ulong.MaxValue);
        Array.Fill(signs, (sbyte)-1);
        Array.Fill(scratch, ulong.MaxValue);
        Array.Fill(scratchSigns, (sbyte)-1);
        CylinderPairValueDerivative derivative = default;
        if (direction is not null)
        {
            var derivativeInput = new ulong[words * CylinderPairValueDerivative.InvariantCount];
            var derivativeInputSigns = new sbyte[CylinderPairValueDerivative.InvariantCount];
            Encode(direction, words, derivativeInput, derivativeInputSigns);
            var derivativeOutput = new ulong[words * CylinderPairValuePolynomial.CoefficientCount];
            var derivativeSigns = new sbyte[CylinderPairValuePolynomial.CoefficientCount];
            var derivativeScratch = new ulong[words * CylinderPairValueDerivative.ScratchCoefficientCount];
            var derivativeScratchSigns = new sbyte[CylinderPairValueDerivative.ScratchCoefficientCount];
            Array.Fill(derivativeOutput, ulong.MaxValue);
            Array.Fill(derivativeSigns, (sbyte)-1);
            Array.Fill(derivativeScratch, ulong.MaxValue);
            Array.Fill(derivativeScratchSigns, (sbyte)-1);
            derivative = new CylinderPairValueDerivative(derivativeInput, derivativeInputSigns,
                derivativeOutput, derivativeSigns, derivativeScratch, derivativeScratchSigns);
        }
        degree = CylinderPairValuePolynomial.Build(input, inputSigns, words,
            output, signs, scratch, scratchSigns, derivative);
        var actual = new BigInteger[signs.Length];
        Decode(output, signs, words, actual);
        if (direction is not null)
            Decode(derivative.Coefficients, derivative.Signs, words, actualDerivative!);
        return actual;
    }

    private static void Encode(BigInteger[] values, int words, ulong[] magnitudes, sbyte[] signs)
    {
        for (int index = 0; index < values.Length; index++)
        {
            signs[index] = (sbyte)values[index].Sign;
            BigInteger remaining = BigInteger.Abs(values[index]);
            for (int word = 0; word < words; word++)
            {
                magnitudes[index * words + word] = (ulong)(remaining & ulong.MaxValue);
                remaining >>= 64;
            }
            Assert.Equal(BigInteger.Zero, remaining);
        }
    }

    private static void Decode(ReadOnlySpan<ulong> magnitudes, ReadOnlySpan<sbyte> signs,
        int words, BigInteger[] actual)
    {
        for (int index = 0; index < actual.Length; index++)
        {
            BigInteger magnitude = BigInteger.Zero;
            for (int word = words - 1; word >= 0; word--)
                magnitude = (magnitude << 64) + magnitudes[index * words + word];
            Assert.Equal(magnitude.IsZero, signs[index] == 0);
            actual[index] = magnitude * signs[index];
        }
    }

    private static BigInteger[] IndependentDirectionalDerivative(BigInteger[] invariants,
        BigInteger[] direction)
    {
        // The classical discriminant is degree at most sixteen in a linear
        // perturbation of these six scalar invariants (A,B,C1,D have degrees
        // at most 1,3,5,4). Extract its exact linear coefficient by rational
        // Lagrange interpolation at 0..16, not a numerical finite difference.
        const int order = 16;
        const int commonDenominator = 720720; // lcm(1,...,16)
        int[] indices = { 3, 4, 5, 6, 7, 10 };
        int harmonicNumerator = 0;
        for (int index = 1; index <= order; index++)
            harmonicNumerator += commonDenominator / index;
        BigInteger[] result = Scale(IndependentDiscriminant(invariants), -harmonicNumerator);
        BigInteger binomial = BigInteger.One;
        for (int sample = 1; sample <= order; sample++)
        {
            binomial = binomial * (order - sample + 1) / sample;
            var perturbed = (BigInteger[])invariants.Clone();
            for (int index = 0; index < indices.Length; index++)
                perturbed[indices[index]] += sample * direction[index];
            BigInteger weight = binomial * (commonDenominator / sample);
            if ((sample & 1) == 0)
                weight = -weight;
            result = Add(result, Scale(IndependentDiscriminant(perturbed), weight));
        }
        return Divide(result, commonDenominator);
    }

    private static BigInteger[] IndependentDiscriminant(BigInteger[] invariants)
    {
        BigInteger[][] f = IndependentPencil(invariants);
        BigInteger[] d = f[0], c = f[1], b = f[2], a = f[3];
        // Classical five-term discriminant, deliberately not 4PR-N².
        BigInteger[] result = Multiply(Multiply(b, b), Multiply(c, c));
        result = Subtract(result, Scale(Multiply(a, Multiply(c, Multiply(c, c))), 4));
        result = Subtract(result, Scale(Multiply(d, Multiply(b, Multiply(b, b))), 4));
        result = Subtract(result, Scale(Multiply(Multiply(a, a), Multiply(d, d)), 27));
        result = Add(result, Scale(Multiply(Multiply(a, b), Multiply(c, d)), 18));
        return Scale(result, 3);
    }

    private static BigInteger[][] IndependentPencil(BigInteger[] v)
    {
        // Interpolate the cubic from four exact matrix determinants. This
        // does not use the production invariant formulas for A,B,C1,D.
        BigInteger[] f0 = MatrixDeterminant(v, 0);
        BigInteger[] fp = MatrixDeterminant(v, 1);
        BigInteger[] fm = MatrixDeterminant(v, -1);
        BigInteger[] f2 = MatrixDeterminant(v, 2);
        BigInteger[] b = Divide(Subtract(Add(fp, fm), Scale(f0, 2)), 2);
        BigInteger[] aPlusC = Divide(Subtract(fp, fm), 2);
        BigInteger[] a = Divide(Subtract(Subtract(Subtract(f2, f0), Scale(b, 4)),
            Scale(aPlusC, 2)), 6);
        return new[] { f0, Subtract(aPlusC, a), b, a };
    }

    private static BigInteger[] MatrixDeterminant(BigInteger[] v, int lambda)
    {
        BigInteger g=v[0], h=v[1], delta=v[2], rho=v[3], tau=v[4];
        BigInteger cSquared=v[5], c0=v[6], c1=v[7], b0=v[8], b1=v[9], j=v[10];
        BigInteger z0 = cSquared + rho;
        BigInteger[] hPolynomial = { z0 + tau, -1 };
        BigInteger[] xx = { 4 * delta * c0 * c0 + 4 * tau * b0 * b0 - lambda * g };
        BigInteger[] xy = { 4 * delta * c0 * c1 + 4 * tau * b0 * b1 };
        BigInteger[] yy = { 4 * delta * c1 * c1 + 4 * tau * b1 * b1 - lambda * h };
        BigInteger[] xz = Add(Scale(hPolynomial, 2 * delta * c0),
            new[] { -4 * tau * delta * c0 + 4 * tau * j * b0 });
        BigInteger[] yz = Add(Scale(hPolynomial, 2 * delta * c1),
            new[] { -4 * tau * delta * c1 + 4 * tau * j * b1 });
        BigInteger[] zz = Add(Scale(Multiply(hPolynomial, hPolynomial), delta),
            new[] { -4 * tau * (delta * z0 - j * j) + lambda * rho });
        return Add(Subtract(Subtract(Multiply(xx, Multiply(yy, zz)),
            Multiply(xx, Multiply(yz, yz))), Multiply(zz, Multiply(xy, xy))),
            Subtract(Scale(Multiply(xy, Multiply(xz, yz)), 2),
                Multiply(yy, Multiply(xz, xz))));
    }

    private static BigInteger[] Add(BigInteger[] left, BigInteger[] right)
    {
        var result = new BigInteger[Math.Max(left.Length, right.Length)];
        for (int index = 0; index < result.Length; index++)
            result[index] = (index < left.Length ? left[index] : 0)
                + (index < right.Length ? right[index] : 0);
        return result;
    }

    private static BigInteger[] Subtract(BigInteger[] left, BigInteger[] right) =>
        Add(left, Scale(right, -1));

    private static BigInteger[] Multiply(BigInteger[] left, BigInteger[] right)
    {
        var result = new BigInteger[left.Length + right.Length - 1];
        for (int first = 0; first < left.Length; first++)
        for (int second = 0; second < right.Length; second++)
            result[first + second] += left[first] * right[second];
        return result;
    }

    private static BigInteger[] Scale(BigInteger[] value, BigInteger scale)
    {
        var result = new BigInteger[value.Length];
        for (int index = 0; index < value.Length; index++)
            result[index] = value[index] * scale;
        return result;
    }

    private static BigInteger[] Divide(BigInteger[] value, int divisor)
    {
        var result = new BigInteger[value.Length];
        for (int index = 0; index < value.Length; index++)
        {
            result[index] = BigInteger.DivRem(value[index], divisor, out BigInteger remainder);
            Assert.Equal(BigInteger.Zero, remainder);
        }
        return result;
    }

    private static BigInteger[] Pad(BigInteger[] value)
    {
        var result = new BigInteger[CylinderPairValuePolynomial.CoefficientCount];
        for (int index = 0; index < value.Length; index++)
        {
            if (index < result.Length)
                result[index] = value[index];
            else
                Assert.Equal(BigInteger.Zero, value[index]);
        }
        return result;
    }
}
