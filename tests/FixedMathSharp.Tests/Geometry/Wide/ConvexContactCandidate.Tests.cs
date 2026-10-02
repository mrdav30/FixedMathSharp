using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class ConvexContactCandidateTests
{
    [Theory]
    [InlineData(0, -1, false)]
    [InlineData(0, 0, false)]
    [InlineData(0, 1, false)]
    [InlineData(1, -1, false)]
    [InlineData(1, 0, false)]
    [InlineData(1, 1, false)]
    [InlineData(0, -1, true)]
    [InlineData(0, 0, true)]
    [InlineData(0, 1, true)]
    [InlineData(1, -1, true)]
    [InlineData(1, 0, true)]
    [InlineData(1, 1, true)]
    [InlineData(long.MaxValue, -1, false)]
    [InlineData(long.MaxValue, 0, false)]
    [InlineData(long.MaxValue, 1, false)]
    [InlineData(long.MaxValue, -1, true)]
    [InlineData(long.MaxValue, 0, true)]
    [InlineData(long.MaxValue, 1, true)]
    public void SquaredGapMidpointComparison_PreservesRawNeighborsAndBothParities(
        long roundedRaw, int offset, bool irrationalParameter)
    {
        ulong[] coefficients = irrationalParameter ? new ulong[] { 1, 0, 2 } : new ulong[] { 1, 2 };
        sbyte[] signs = irrationalParameter ? new sbyte[] { -1, 0, 1 } : new sbyte[] { -1, 1 };
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, 0, cell,
            out FiniteAxisValueRoot parameter));
        ulong[] numerator = new ulong[5 * ConvexContactCandidate.Words];
        ulong[] denominator = new ulong[numerator.Length];
        sbyte[] numeratorSigns = { (sbyte)Math.Sign(offset), 0, 1, 0, 0 };
        sbyte[] denominatorSigns = { 1, 0, 0, 0, 0 };
        ulong twiceRaw = ((ulong)roundedRaw << 1) | 1UL;
        // The parameter squared is 1/4 or 1/2. N/D is therefore exactly
        // (d+1/2)^2 + offset/2^65. A lower neighbor must remain eligible
        // even when the public rounded depths coincide.
        numerator[0] = (ulong)Math.Abs(offset);
        BigInteger coefficient = (BigInteger)twiceRaw * twiceRaw * (irrationalParameter ? 1 : 2);
        for (int word = 1; coefficient != 0; word++)
        {
            numerator[2 * ConvexContactCandidate.Words + word] = (ulong)(coefficient & ulong.MaxValue);
            coefficient >>= 64;
        }
        denominator[1] = 2;
        Assert.Equal(offset, ConvexContactValueRoot.CompareSquaredGapToTwiceRaw(ref parameter,
            numerator, numeratorSigns, denominator, denominatorSigns, twiceRaw));
        Assert.Equal(roundedRaw == long.MaxValue ? offset : -1, ConvexContactValueRoot.CompareSquaredGapToTwiceRaw(ref parameter,
            numerator, numeratorSigns, denominator, denominatorSigns, ulong.MaxValue));
    }

    [Theory]
    [InlineData(0, 0)]
    [InlineData(0, 1)]
    [InlineData(0, 50)]
    [InlineData(1, 0)]
    [InlineData(1, 1)]
    [InlineData(1, 50)]
    public void BoxCylinderAnalyticGap_MatchesUnreducedIntegerSupportIdentity(int radius, int centerY)
    {
        var geometry = new BoxCylinderGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(2, 1, 3), new Vector3d(0, centerY, 0),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4),
            Vector3d.Up, Fixed64.Two, (Fixed64)radius);
        ulong[] values = new ulong[ConvexContactCandidate.Slots * ConvexContactCandidate.Words];
        int[] signs = new int[ConvexContactCandidate.Slots];
        BoxCylinderAnalyticFeatures.TryGetBest(geometry, values, signs, out int gapSign, out _);
        var candidate = new ConvexContactCandidate(values, signs, gapSign);
        BigInteger u = 0, n2 = 0, axial = 0, half = 0, center = 0, t = 0;
        for (int component = 0; component < 3; component++)
        {
            // The identity box frame makes the retained world normal the
            // authored support direction, without a rounded inverse rotation.
            BigInteger n = Magnitude(candidate.NormalRational(component)) * signs[component];
            BigInteger a = Signed(BoxCylinderGeometry.GetComponent(geometry.Axis, component));
            u += a * a; n2 += n * n; axial += a * n;
            half += Signed(BoxCylinderGeometry.GetComponent(geometry.Half, component)) * n;
            center += Signed(BoxCylinderGeometry.GetComponent(geometry.CenterDifference, component)) * n;
            t += Signed(BoxCylinderGeometry.GetComponent(geometry.HalfExtents, component)) * BigInteger.Abs(n);
        }
        t += BigInteger.Abs(half) - BigInteger.Abs(center);
        BigInteger r = Signed(geometry.Radius);
        BigInteger s = Signed(Signed320.ExtendValue(geometry.RawScale));
        // Independent unreduced identity: g=(U*T+sqrt(R²*U*V))/(U*S*|n|).
        // Compare rational terms and squared radical terms separately, so
        // moving a nonnegative factor outside the square root remains valid.
        BigInteger p = u * t, q = r * r * u * (u * n2 - axial * axial);
        BigInteger d = u * u * s * s * n2;
        BigInteger actualD = Magnitude(candidate.GapDenominator);
        Assert.Equal((p * p + q) * actualD, Magnitude(candidate.GapRational) * d);
        BigInteger b = Magnitude(candidate.GapRadical);
        BigInteger c = Magnitude(candidate.GapRadicand);
        Assert.Equal(4 * p * p * q * actualD * actualD, b * b * c * d * d);
        Assert.Equal(q.IsZero || p.IsZero ? 0 : p.Sign, c.IsZero ? 0 : candidate.GapRadicalSign);
        Assert.Equal(p.Sign < 0 ? (q - p * p).Sign : p.Sign > 0 || !q.IsZero ? 1 : 0, gapSign);
    }

    [Theory]
    [InlineData(0, 2, 3, -1)]
    [InlineData(1, 2, 2, 1)]
    [InlineData(1, 2, 3, 1)]
    [InlineData(1, 2, 5, 1)]
    [InlineData(1, 2, 6, -1)]
    [InlineData(1, 4, 9, 0)]
    [InlineData(-1, 3, 2, -1)]
    [InlineData(-1, 2, 2, -1)]
    public void RadicalGapRanking_PreservesOpposingTermsAndExactTies(
        int rationalDifference, int leftRadicand, int rightRadicand, int expected)
    {
        // Squared gaps differ by r + sqrt(c) - sqrt(d). These cases cover
        // agreeing and opposing differences after squaring the radical sums,
        // equal sums of radicands, and the exact tie 1 + sqrt(4) = sqrt(9).
        foreach (int word in new[] { 0, ConvexContactCandidate.Words - 1 })
        {
            foreach (var (gapSign, radicalSign) in new[] { (1, 1), (-1, 1), (1, -1), (-1, -1) })
            {
                ulong[] leftValues = Encode(10 + radicalSign * rationalDifference, leftRadicand, word);
                ulong[] rightValues = Encode(10, rightRadicand, word);
                int[] signs = new int[ConvexContactCandidate.Slots];
                signs[0] = signs[7] = 1;
                signs[8] = radicalSign;
                var left = new ConvexContactCandidate(leftValues, signs, gapSign);
                var right = new ConvexContactCandidate(rightValues, signs, gapSign);
                Assert.Equal(gapSign * radicalSign * expected,
                    WideConvexPrismRelations.CompareConvexContactCandidates(left, right));
                Assert.Equal(-gapSign * radicalSign * expected,
                    WideConvexPrismRelations.CompareConvexContactCandidates(right, left));
            }
        }
    }

    private static ulong[] Encode(int rational, int radicand, int word)
    {
        var values = new ulong[ConvexContactCandidate.Slots * ConvexContactCandidate.Words];
        values[0] = 1;
        // Scaling A, B and D together preserves (A+B*sqrt(C))/D, including
        // at the largest supported coefficient stride.
        values[7 * ConvexContactCandidate.Words + word] = (ulong)rational;
        values[8 * ConvexContactCandidate.Words + word] = 1;
        values[9 * ConvexContactCandidate.Words] = (ulong)radicand;
        values[10 * ConvexContactCandidate.Words + word] = 1;
        return values;
    }

    private static BigInteger Signed(Signed320 value)
    {
        Span<ulong> words = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(value, out words[4], out words[3], out words[2], out words[1], out words[0]);
        return value.Sign * Magnitude(words);
    }

    private static BigInteger Magnitude(ReadOnlySpan<ulong> words)
    {
        BigInteger value = 0;
        for (int index = words.Length - 1; index >= 0; index--)
            value = (value << 64) + words[index];
        return value;
    }
}
