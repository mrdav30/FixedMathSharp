//=======================================================================
// ContactQuadratic.Ratios.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class ContactQuadraticRatioTests
{
    [Theory]
    [InlineData(0, 0)]
    [InlineData(5, 7)]
    [InlineData(-5, 7)]
    public void ZeroRadicandProduct_ShouldIgnoreRadicalCoefficientsAndKeepCanonicalSigns(int first, int second)
    {
        const int words = 9;
        Span<ulong> values = stackalloc ulong[6 * words], root = stackalloc ulong[3];
        Span<int> signs = stackalloc int[6];
        root.Clear();
        ContactQuadratic a = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic b = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic product = ContactQuadratic.At(values, signs, 2, words);
        SetIntegerField(a, first, 3); SetIntegerField(b, second, -2);
        product.Values.Fill(ulong.MaxValue); product.Signs.Fill(7);
        ContactQuadratic.Multiply(a, b, root, product);
        Assert.Equal(first * second, product.Sign(root) * (int)product.Rational[0]);
        Assert.Equal(Math.Sign(first * second), product.Signs[0]);
        Assert.Equal(0, product.Signs[1]);
        Assert.Equal(0, WideArithmetic.GetActiveMagnitudeLength(product.Radical));
        Assert.Equal(0, WideArithmetic.GetActiveMagnitudeLength(product.Rational[1..]));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(-1)]
    public void Set_ShouldReplaceBothFieldsOfReusedScratch(int sign)
    {
        const int words = 64;
        Span<ulong> values = stackalloc ulong[2 * words];
        Span<int> signs = stackalloc int[2];
        values.Fill(ulong.MaxValue); signs.Fill(-1);
        var value = new ContactQuadratic(values, signs);
        value.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(sign))));
        Assert.Equal(sign, signs[0]); Assert.Equal(0, signs[1]);
        Assert.Equal((ulong)Math.Abs(sign), value.Rational[0]);
        Assert.Equal(0, WideArithmetic.GetActiveMagnitudeLength(value.Rational[1..]));
        Assert.Equal(0, WideArithmetic.GetActiveMagnitudeLength(value.Radical));
    }

    [Theory]
    [InlineData(446, 300, 3, false)]
    [InlineData(445, 300, 3, false)]
    [InlineData(446, 5, 0, false)]
    [InlineData(0, 12, 3, false)]
    [InlineData(14, 0, 0, false)]
    [InlineData(0, 12, 3, true)]
    [InlineData(446, 300, 3, true)]
    [InlineData(-1, 12, 3, true)]
    [InlineData(446, -1, 3, false)]
    public void PointNormalization_ShouldPreserveExactHomogeneousRatiosAndEnclosurePrecision(
        int rootPower, int rationalPower, int radicalPower, bool rationalOnly)
    {
        const int words = 8;
        Span<ulong> values = stackalloc ulong[8 * words], root = stackalloc ulong[words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic x = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic y = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic z = ContactQuadratic.At(values, signs, 2, words);
        ContactQuadratic d = ContactQuadratic.At(values, signs, 3, words);
        BigInteger aScale = rationalPower < 0 ? BigInteger.Zero : BigInteger.One << rationalPower;
        BigInteger bScale = rationalOnly ? BigInteger.Zero : BigInteger.One << radicalPower;
        BigInteger[] original = { 5*aScale, -3*bScale, 0, 2*bScale, -14*aScale, 0, 9*aScale, bScale };
        SetIntegerField(x, original[0], original[1]); SetIntegerField(y, original[2], original[3]);
        SetIntegerField(z, original[4], original[5]); SetIntegerField(d, original[6], original[7]);
        // These banks have no mathematical contribution, despite stale storage.
        for (int index = 0; index < signs.Length; index++)
            if (signs[index] == 0) values.Slice(index * words, words).Fill(ulong.MaxValue);
        BigInteger oldRoot = rootPower < 0 ? BigInteger.Zero : (BigInteger)3 << rootPower;
        WriteMagnitude(oldRoot, root);
        WideArithmetic.GetMagnitudeSquareRootBounds(root, out Signed192 oldLower, out Signed192 oldUpper, out int oldShift);
        int[] originalSigns = signs.ToArray();
        WideFiniteAxisIntersection.NormalizeQuadraticPointPowerOfTwo(x, y, z, d, root);
        BigInteger newRoot = ReadMagnitude(root);
        BigInteger factorSquared = oldRoot.IsZero ? BigInteger.One : oldRoot / newRoot, factor = BigInteger.One;
        while (factorSquared > 1) { Assert.Equal(BigInteger.Zero, factorSquared % 4); factorSquared /= 4; factor *= 2; }
        Assert.Equal(oldRoot, newRoot * factor * factor);
        WideArithmetic.GetMagnitudeSquareRootBounds(root, out Signed192 newLower, out Signed192 newUpper, out int newShift);
        Assert.Equal(oldLower, newLower); Assert.Equal(oldUpper, newUpper);
        Assert.Equal(BigInteger.One << (oldShift - newShift), factor);
        Assert.True(signs.SequenceEqual(originalSigns));
        BigInteger[] normalized = new BigInteger[8];
        for (int index = 0; index < 8; index++)
        {
            normalized[index] = signs[index] == 0 ? BigInteger.Zero
                : signs[index] * ReadMagnitude(values.Slice(index * words, words));
            Assert.True(BigInteger.Abs(normalized[index]) <= BigInteger.Abs(original[index]));
        }
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            BigInteger a = original[2 * coordinate], b = original[2 * coordinate + 1] * factor;
            BigInteger na = normalized[2 * coordinate], nb = normalized[2 * coordinate + 1];
            BigInteger da = original[6], db = original[7] * factor, nda = normalized[6], ndb = normalized[7];
            Assert.Equal(BigInteger.Zero, a*nda + b*ndb*newRoot - na*da - nb*db*newRoot);
            Assert.Equal(BigInteger.Zero, a*ndb + b*nda - na*db - nb*da);
        }

        static BigInteger ReadMagnitude(ReadOnlySpan<ulong> field)
        {
            BigInteger result = BigInteger.Zero;
            for (int index = field.Length - 1; index >= 0; index--) result = (result << 64) + field[index];
            return result;
        }
    }

    [Theory]
    [InlineData(1, -1, -1, 2)]
    [InlineData(-1, 1, 1, 2)]
    [InlineData(0, 1, -1, 2)]
    [InlineData(-1, 0, 1, 5)]
    [InlineData(0, 0, -1, 2)]
    [InlineData(1, 1, 1, 5)]
    public void Copy_ShouldUseCompleteActiveFieldsAndClearReusedDestinationTails(
        int rationalSign, int radicalSign, int multiplier, int destinationWords)
    {
        Span<ulong> sourceValues = stackalloc ulong[8]; Span<int> sourceSigns = stackalloc int[2];
        Span<ulong> destinationValues = stackalloc ulong[2 * destinationWords];
        Span<int> destinationSigns = stackalloc int[2]; Span<ulong> expected = stackalloc ulong[destinationWords];
        var source = new ContactQuadratic(sourceValues, sourceSigns);
        var destination = new ContactQuadratic(destinationValues, destinationSigns);
        BigInteger rational = rationalSign * ((BigInteger.One << 80) + 3);
        BigInteger radical = radicalSign * ((BigInteger.One << 65) + 5);
        SetIntegerField(source, rational, radical);
        // Zero signs are authoritative even when a borrowed bank still contains
        // old bytes that do not fit the smaller destination.
        if (rationalSign == 0) source.Rational.Fill(ulong.MaxValue);
        if (radicalSign == 0) source.Radical.Fill(ulong.MaxValue);
        ulong[] original = sourceValues.ToArray(); int[] originalSigns = sourceSigns.ToArray();
        destinationValues.Fill(ulong.MaxValue); destinationSigns.Fill(-1);
        source.CopyTo(destination, multiplier);
        WriteMagnitude(rational, expected); Assert.True(expected.SequenceEqual(destination.Rational));
        WriteMagnitude(radical, expected); Assert.True(expected.SequenceEqual(destination.Radical));
        Assert.Equal(rationalSign * multiplier, destination.Signs[0]);
        Assert.Equal(radicalSign * multiplier, destination.Signs[1]);
        Assert.True(sourceValues.SequenceEqual(original)); Assert.True(sourceSigns.SequenceEqual(originalSigns));
    }

    [Fact]
    public void Copy_IntoItsOwnBorrowedBanks_ShouldPreserveMagnitudeAndApplyTheRequestedSign()
    {
        Span<ulong> values = stackalloc ulong[8]; Span<int> signs = stackalloc int[2];
        var value = new ContactQuadratic(values, signs);
        SetIntegerField(value, (BigInteger.One << 80) + 3, -((BigInteger.One << 65) + 5));
        ulong[] original = values.ToArray();
        value.CopyTo(value, -1);
        Assert.True(values.SequenceEqual(original));
        Assert.Equal(-1, value.Signs[0]); Assert.Equal(1, value.Signs[1]);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RetainedPoint_WithDistinctOriginsAndRotatedTarget_ShouldRoundOnlyInTheFinalLattice(bool quadratic)
    {
        const int words = 4;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic y = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic z = ContactQuadratic.At(values, signs, 2, words);
        ContactQuadratic denominator = ContactQuadratic.At(values, signs, 3, words);
        SetIntegerField(x, 1, quadratic ? 1 : 0);
        SetIntegerField(y, 3, quadratic ? -1 : 0);
        SetIntegerField(z, -1, quadratic ? 1 : 0);
        SetIntegerField(denominator, 2, 0);
        Span<ulong> root = stackalloc ulong[1] { quadratic ? 2UL : 0UL };
        var sourceOrigin = new Vector3d(Fixed64.FromRaw(1), Fixed64.FromRaw(-1), Fixed64.FromRaw(1));
        Vector3d targetOrigin = Vector3d.Up * Fixed64.FromRaw(1);
        FixedQuaternion targetRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        Assert.True(QuadraticPointMaterialization.TryGetPointInRigidFrame(sourceOrigin, FixedQuaternion.Identity,
            targetOrigin, targetRotation, x, y, z, denominator, root, out Vector3d point));
        // Inverse quarter-turn maps world Y to local X and -world X to local Y.
        // The half-raw rational start must remain exact until origin addition
        // and that transform; rounding it first gives the wrong local Y.
        Assert.Equal(new Vector3d(Fixed64.FromRaw(quadratic ? -1 : 0), Fixed64.FromRaw(-2),
            Fixed64.FromRaw(quadratic ? 1 : 0)), point);
    }

    [Fact]
    public void QuadraticParameter_ShouldRetainDiscriminantsWiderThanItsCoefficientBanks()
    {
        const int words = 10;
        Span<ulong> values = stackalloc ulong[10 * words], root = stackalloc ulong[17];
        Span<ulong> expected = stackalloc ulong[17];
        Span<int> signs = stackalloc int[10];
        ContactQuadratic a = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic b = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic c = ContactQuadratic.At(values, signs, 2, words);
        ContactQuadratic numerator = ContactQuadratic.At(values, signs, 3, words);
        ContactQuadratic denominator = ContactQuadratic.At(values, signs, 4, words);
        BigInteger aa = (BigInteger.One << 528) + 3;
        BigInteger bb = -((BigInteger.One << 527) + 5);
        BigInteger cc = -((BigInteger.One << 528) + 7);
        SetIntegerField(a, aa, 0); SetIntegerField(b, bb, 0); SetIntegerField(c, cc, 0);
        WriteMagnitude(bb * bb - aa * cc, expected);
        Assert.True(ContactQuadratic.TryGetParameter(a, b, c, 1, root, numerator, denominator));
        Assert.True(root.SequenceEqual(expected));
        Assert.True(numerator.Sign(root) > 0);
        Assert.True(denominator.Sign(root) > 0);
        Assert.True(ContactQuadratic.TryGetParameter(a, b, c, -1, root, numerator, denominator));
        Assert.True(root.SequenceEqual(expected));
        Assert.True(numerator.Sign(root) < 0);
    }

    [Fact]
    public void RootMultiplication_PreservesMixedWidthCoefficientsAndClearsReusedDestinationTails()
    {
        Span<ulong> input = stackalloc ulong[4], output = stackalloc ulong[16], expected = stackalloc ulong[8];
        Span<int> signs = stackalloc int[4];
        var value = new ContactQuadratic(input, signs[..2]); var result = new ContactQuadratic(output, signs[2..]);
        foreach (var item in new (BigInteger A, BigInteger B)[] { (3, -7), (-9, 4), (0, 0), ((BigInteger.One << 80) + 3, -((BigInteger.One << 65) + 5)) })
        {
            SetIntegerField(value, item.A, item.B);
            ulong[] original = input.ToArray(); int[] originalSigns = value.Signs.ToArray();
            output.Fill(ulong.MaxValue); result.Signs.Fill(-1);
            ContactQuadratic.MultiplyRoot(value, new ulong[] { 5 }, result);
            WriteMagnitude(5 * item.B, expected); Assert.True(expected.SequenceEqual(result.Rational));
            WriteMagnitude(item.A, expected); Assert.True(expected.SequenceEqual(result.Radical));
            Assert.Equal(item.B.Sign, result.Signs[0]); Assert.Equal(item.A.Sign, result.Signs[1]);
            Assert.True(input.SequenceEqual(original)); Assert.True(value.Signs.SequenceEqual(originalSigns));
        }
    }

    [Fact]
    public void RationalComparison_PreservesSignedDenominatorsAndBorrowedInputsAgainstBigIntegerOracle()
    {
        const int words = 16;
        Span<ulong> fields = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic a = ContactQuadratic.At(fields, signs, 0, words), b = ContactQuadratic.At(fields, signs, 1, words);
        ContactQuadratic c = ContactQuadratic.At(fields, signs, 2, words), d = ContactQuadratic.At(fields, signs, 3, words);
        BigInteger wide = (BigInteger.One << 320) + 3;
        BigInteger[] numerators = { -wide, -7, 0, 5, wide }, denominators = { -wide, -3, 2, wide };
        foreach (BigInteger firstN in numerators)
        foreach (BigInteger firstD in denominators)
        foreach (BigInteger secondN in numerators)
        foreach (BigInteger secondD in denominators)
        {
            SetIntegerField(a, firstN, 0); SetIntegerField(b, firstD, 0);
            SetIntegerField(c, secondN, 0); SetIntegerField(d, secondD, 0);
            // A retained zero sign can legitimately outlive old borrowed bytes.
            // Rational comparison must ignore every radical bank in that case.
            a.Radical.Fill(ulong.MaxValue); b.Radical.Fill(ulong.MaxValue);
            c.Radical.Fill(ulong.MaxValue); d.Radical.Fill(ulong.MaxValue);
            ulong[] original = fields.ToArray(); int[] originalSigns = signs.ToArray();
            int expected = (firstN * secondD - secondN * firstD).Sign * firstD.Sign * secondD.Sign;
            Assert.Equal(expected, ContactQuadratic.CompareRatios(a, b, new ulong[] { 2 }, c, d, new ulong[] { 3 }));
            Assert.Equal(0, ContactQuadratic.CompareRatios(a, b, new ulong[] { 2 }, a, b, new ulong[] { 7 }));
            Assert.True(fields.SequenceEqual(original)); Assert.True(signs.SequenceEqual(originalSigns));
        }
    }

    [Theory]
    [InlineData(-7)]
    [InlineData(0)]
    [InlineData(1)]
    public void Scale_PreservesExactSignedCoefficientProductsAndClearsZeroSignBanks(int scalar)
    {
        const int words = 16;
        Span<ulong> fields = stackalloc ulong[4 * words], expected = stackalloc ulong[words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic value = ContactQuadratic.At(fields, signs, 0, words), result = ContactQuadratic.At(fields, signs, 1, words);
        var cases = new (BigInteger A, BigInteger B)[]
        {
            (0, 0), (3, 0), (0, -5), (-7, 11), ((BigInteger.One << 320) + 3, -((BigInteger.One << 128) + 5))
        };
        foreach (var item in cases)
        {
            SetIntegerField(value, item.A, item.B);
            if (item.A == 0) value.Rational.Fill(ulong.MaxValue);
            if (item.B == 0) value.Radical.Fill(ulong.MaxValue);
            ulong[] original = value.Values.ToArray(); int[] originalSigns = value.Signs.ToArray();
            result.Values.Fill(ulong.MaxValue); result.Signs.Fill(-1);
            ContactQuadratic.Scale(value, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(scalar))), result);
            WriteMagnitude(item.A * scalar, expected); Assert.True(expected.SequenceEqual(result.Rational));
            WriteMagnitude(item.B * scalar, expected); Assert.True(expected.SequenceEqual(result.Radical));
            Assert.Equal((item.A * scalar).Sign, result.Signs[0]); Assert.Equal((item.B * scalar).Sign, result.Signs[1]);
            Assert.True(value.Values.SequenceEqual(original)); Assert.True(value.Signs.SequenceEqual(originalSigns));
        }
        // The span overload also treats an explicit scalar sign zero as zero,
        // regardless of bytes left in its borrowed magnitude storage.
        expected.Fill(ulong.MaxValue); result.Values.Fill(ulong.MaxValue); result.Signs.Fill(1);
        ContactQuadratic.Scale(value, expected, 0, result);
        foreach (ulong word in result.Values) Assert.Equal(0UL, word);
        Assert.Equal(0, result.Signs[0]); Assert.Equal(0, result.Signs[1]);
        // Unsigned magnitude callers use +1 even when the magnitude is zero.
        expected.Clear(); result.Values.Fill(ulong.MaxValue); result.Signs.Fill(1);
        ContactQuadratic.Scale(value, expected, 1, result);
        foreach (ulong word in result.Values) Assert.Equal(0UL, word);
        Assert.Equal(0, result.Signs[0]); Assert.Equal(0, result.Signs[1]);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void RationalMultiplication_PreservesBigIntegerProductsAndAliasedStorage(int alias)
    {
        const int words = 16;
        Span<ulong> fields = stackalloc ulong[6 * words], expectedMagnitude = stackalloc ulong[words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic first = ContactQuadratic.At(fields, signs, 0, words), second = ContactQuadratic.At(fields, signs, 1, words);
        ContactQuadratic separate = ContactQuadratic.At(fields, signs, 2, words);
        var cases = new (BigInteger A, BigInteger B)[] { (0, 7), (3, -5), (-7, -11), ((BigInteger.One << 320) + 3, -((BigInteger.One << 128) + 5)) };
        foreach (var item in cases)
        {
            SetIntegerField(first, item.A, 0); SetIntegerField(second, item.B, 0);
            separate.Values.Fill(ulong.MaxValue); separate.Signs.Fill(1);
            ContactQuadratic other = alias == 3 ? first : second;
            ContactQuadratic destination = alias == 0 ? separate : alias == 2 ? second : first;
            BigInteger expected = item.A * (alias == 3 ? item.A : item.B);
            ContactQuadratic.Multiply(first, other, new ulong[] { 7 }, destination);
            WriteMagnitude(expected, expectedMagnitude);
            Assert.True(expectedMagnitude.SequenceEqual(destination.Rational));
            Assert.Equal(expected.Sign, destination.Signs[0]); Assert.Equal(0, destination.Signs[1]);
            foreach (ulong word in destination.Radical) Assert.Equal(0UL, word);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(5)]
    public void UnitMetricRootRatio_PreservesNonunitScaleAndWideRationalOracle(int commonWords)
    {
        const int words = 16;
        Span<ulong> fields = stackalloc ulong[4 * words], metric = stackalloc ulong[words], scale = stackalloc ulong[words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, signs, 0, words), d = ContactQuadratic.At(fields, signs, 1, words);
        metric.Clear(); metric[0] = 1; scale.Clear(); scale[0] = 3;
        var cases = new (BigInteger N, BigInteger D)[] { (0, 2), (15, 2), (21, 2), (5, 2), (3 * (BigInteger)long.MaxValue, 1), (3 * (2 * (BigInteger)long.MaxValue + 1), 2), (3 * (4 * (BigInteger)long.MaxValue + 1), 4) };
        foreach (var item in cases)
        {
            SetIntegerField(n, item.N << (64 * commonWords), 0); SetIntegerField(d, item.D << (64 * commonWords), 0);
            BigInteger expected = BigInteger.DivRem(item.N, 3 * item.D, out BigInteger remainder);
            int half = (2 * remainder).CompareTo(3 * item.D);
            if (half > 0 || half == 0 && !expected.IsEven) expected++;
            bool represented = expected <= long.MaxValue;
            Assert.Equal(represented, ContactQuadratic.TryRoundRootRatio(n, d, ReadOnlySpan<ulong>.Empty, metric, scale, out Fixed64 actual));
            Assert.Equal(represented ? (long)expected : 0, actual.m_rawValue);
        }
    }

    [Theory]
    [InlineData(3, 0, 1, 1)]
    [InlineData(1, 1, 1, 0)]
    [InlineData(1, 0, 2, -1)]
    public void UnitMetricRootRatio_RetainsExactRadicalNumeratorOrDenominator(int a, int b, int c, int d)
    {
        Span<ulong> fields = stackalloc ulong[4 * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic numerator = ContactQuadratic.At(fields, signs, 0), denominator = ContactQuadratic.At(fields, signs, 1);
        Set(numerator, a, b); Set(denominator, c, d);
        long expected = RoundQuadraticOracle(a, b, c, d, 2, 0, long.MaxValue, 0);
        Assert.True(ContactQuadratic.TryRoundRootRatio(numerator, denominator, new ulong[] { 2 },
            new ulong[] { 1 }, new ulong[] { 1 }, out Fixed64 actual));
        Assert.Equal(expected, actual.m_rawValue);
    }

    [Fact]
    public void SignedRawQuadraticRounding_AgreesWithIndependentBigIntegerOracleAcrossClippedDomains()
    {
        var cases = new (BigInteger A, BigInteger B, BigInteger C, BigInteger D, BigInteger Root)[]
        {
            (1, 1, 1, 0, 2), (-1, -1, 1, 0, 2), (-1, 1, 1, 0, 2), (1, -1, 1, 0, 2),
            (7, -2, 3, 1, 2), (-7, 2, 3, 1, 2), (3, 1, -1, 1, 2), (-3, -1, 2, -1, 2),
            (0, 1, 1, 0, 0), (1, 1, 2, 0, 1), (2, 1, 2, 0, 1), (-2, -1, 2, 0, 1),
            (long.MinValue + 2, -1, 1, 0, 4), (long.MinValue + 1, -1, 1, 0, 2), (long.MaxValue - 1, 1, 1, 0, 2),
            (0, 1, BigInteger.One << 168, 0, (BigInteger.One << 400) + 1),
            (0, -1, BigInteger.One << 168, 0, (BigInteger.One << 400) + 1),
            (-(BigInteger.One << 200), 1, 1, 0, (BigInteger.One << 400) + 1)
        };
        var domains = new (long Low, long High)[] { (long.MinValue, long.MaxValue), (-4, 3), (0, 0), (1, 1) };
        const int words = 16;
        Span<ulong> fields = stackalloc ulong[4 * words], root = stackalloc ulong[words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, signs, 0, words), d = ContactQuadratic.At(fields, signs, 1, words);
        foreach (var item in cases)
        {
            SetIntegerField(n, item.A, item.B); SetIntegerField(d, item.C, item.D); WriteMagnitude(item.Root, root);
            foreach (var domain in domains)
            for (long parity = -1; parity <= 2; parity++)
            {
                long expected = RoundQuadraticOracle(item.A, item.B, item.C, item.D, item.Root, domain.Low, domain.High, parity);
                Assert.Equal(expected, ContactQuadratic.RoundRatio(n, d, root, domain.Low, domain.High, parity).m_rawValue);
            }
        }
    }

    [Fact]
    public void PaddedQuadraticRounding_RetainsEveryActiveHighLimbWithoutReadingStaleScratch()
    {
        const int words = 160;
        Span<ulong> fields = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, signs, 0, words), d = ContactQuadratic.At(fields, signs, 1, words);
        BigInteger common = BigInteger.One << (64 * 130);
        SetIntegerField(n, 3 * common, common); SetIntegerField(d, 2 * common, 0);
        Assert.Equal(2, ContactQuadratic.RoundRatio(n, d, new ulong[] { 2 }).m_rawValue);
        // The same retained banks now have only one active word. Both exact
        // signs and the enclosure must ignore the cleared former high tail.
        SetIntegerField(n, -3, -1); SetIntegerField(d, 2, 0);
        Assert.Equal(-2, ContactQuadratic.RoundRatio(n, d, new ulong[] { 2 }).m_rawValue);
    }

    private static long RoundQuadraticOracle(BigInteger a, BigInteger b, BigInteger c, BigInteger d,
        BigInteger root, long low, long high, long parity)
    {
        // BigInteger is an independent test oracle. Each comparison retains
        // signed coefficients, then uses squaring only for opposite signs.
        while (low < high)
        {
            long midpoint = (long)((BigInteger)low + (((BigInteger)high - low + 1) >> 1));
            if (QuadraticSign(a - midpoint * c, b - midpoint * d, root) >= 0) low = midpoint;
            else high = midpoint - 1;
        }
        BigInteger half = 2 * (BigInteger)low + 1;
        int comparison = QuadraticSign(2 * a - half * c, 2 * b - half * d, root);
        return unchecked(low + (comparison > 0 || comparison == 0 && ((low ^ parity) & 1) != 0 ? 1 : 0));
    }

    private static int QuadraticSign(BigInteger rational, BigInteger radical, BigInteger root)
    {
        if (root.IsZero || radical.IsZero) return rational.Sign;
        if (rational.IsZero) return radical.Sign;
        if (rational.Sign == radical.Sign) return rational.Sign;
        return rational.Sign * (rational * rational).CompareTo(radical * radical * root);
    }

    private static void SetIntegerField(ContactQuadratic value, BigInteger rational, BigInteger radical)
    {
        value.Clear(); WriteMagnitude(rational, value.Rational); WriteMagnitude(radical, value.Radical);
        value.Signs[0] = rational.Sign; value.Signs[1] = radical.Sign;
    }

    private static void WriteMagnitude(BigInteger value, Span<ulong> destination)
    {
        destination.Clear(); value = BigInteger.Abs(value);
        for (int index = 0; !value.IsZero; index++, value >>= 64)
            destination[index] = (ulong)(value & ulong.MaxValue);
    }

    [Theory]
    [InlineData(1, 1, 3, 2, 2, -2, 1, 2, 0, 8, 0)]
    [InlineData(2, 1, 3, 1, 2, 2, 1, 3, 1, 3, -1)]
    [InlineData(-2, -1, 3, 1, 2, -2, -1, 3, 1, 3, 1)]
    [InlineData(2, 1, -3, -1, 2, -2, -1, 3, 1, 3, 1)]
    [InlineData(1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0)]
    [InlineData(0, 1, 1, 0, 2, 0, 1, 1, 0, 3, -1)]
    public void IndependentQuadraticRatios_PreserveSignsAndExactEquality(
        int a, int b, int c, int d, int k, int e, int f, int g, int h, int l, int expected)
    {
        Assert.Equal(expected, Compare(a, b, c, d, k, e, f, g, h, l));
        Assert.Equal(-expected, Compare(e, f, g, h, l, a, b, c, d, k));
    }

    [Fact]
    public void RationalizedFields_AgreeWithIndependentIntegerRatioOracle()
    {
        // Perfect-square roots make all four signed coefficients independent,
        // while the expected result needs only ordinary integer arithmetic.
        for (int a = -2; a <= 2; a++)
        for (int b = -2; b <= 2; b++)
        for (int c = -2; c <= 2; c++)
        for (int d = -2; d <= 2; d++)
        {
            int left = a + 2 * b, right = c + 3 * d;
            int firstDenominator = 7 + 2 * d, secondDenominator = -8 + 3 * b;
            int expected = Math.Sign((left * secondDenominator - right * firstDenominator)
                * Math.Sign(firstDenominator * secondDenominator));
            Assert.Equal(expected, Compare(a, b, 7, d, 4, c, d, -8, b, 9));
        }
    }

    [Fact]
    public void DifferentRadicands_CompareOneIntegerNeighborOfAnExactTie()
    {
        // sqrt(8)/2 = sqrt(2), independent of the root representation.
        Assert.Equal(0, Compare(0, 1, 1, 0, 2, 0, 1, 2, 0, 8));
        Assert.Equal(-1, Compare(0, 1, 1, 0, 2, 1, 1, 2, 0, 8));
        Assert.Equal(1, Compare(0, 1, 1, 0, 2, -1, 1, 2, 0, 8));
    }

    [Theory]
    [InlineData(0, 1, 1, 1, 0)]
    [InlineData(1, 2, 1, 1, 0)]
    [InlineData(3, 2, 1, 1, 2)]
    [InlineData(5, 2, 1, 1, 2)]
    [InlineData(7, 2, 1, 1, 4)]
    [InlineData(1, 1, 2, 1, 1)]
    [InlineData(3, 1, 2, 1, 4)]
    [InlineData(1, 1, 0, 1, 0)]
    public void RootRatioMaterialization_RoundsRawUnitsOnlyAtFinalNearestEven(
        int numerator, int denominator, int metric, int scale, long expectedRaw)
    {
        Span<ulong> values = stackalloc ulong[4 * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0), d = ContactQuadratic.At(values, signs, 1);
        Set(n, numerator, 0); Set(d, denominator, 0);
        Assert.True(ContactQuadratic.TryRoundRootRatio(n, d, new ulong[] { 0 },
            new ulong[] { (ulong)metric }, new ulong[] { (ulong)scale }, out Fixed64 result));
        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void WideBorrowedFields_PreserveHighLimbsAndClearCopiedTails()
    {
        const int words = 48;
        Span<ulong> values = stackalloc ulong[8 * words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0, words), d = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic otherN = ContactQuadratic.At(values, signs, 2, words), otherD = ContactQuadratic.At(values, signs, 3, words);
        Set(n, 0, 0); Set(d, 0, 0); Set(otherN, 0, 0); Set(otherD, 0, 0);
        n.Rational[43] = 3; d.Rational[43] = 2; n.Signs[0] = d.Signs[0] = 1;
        Set(otherN, 1, 0); Set(otherD, 1, 0);
        Assert.Equal(1, ContactQuadratic.CompareRatios(n, d, new ulong[] { 2 }, otherN, otherD, new ulong[] { 3 }));
        Assert.Equal(2, ContactQuadratic.RoundRatio(n, d, new ulong[] { 2 }).m_rawValue);
        Assert.True(ContactQuadratic.TryRoundRootRatio(n, d, new ulong[] { 2 }, new ulong[] { 1 }, new ulong[] { 1 }, out Fixed64 result));
        Assert.Equal(2, result.m_rawValue);
        otherN.Values.Fill(ulong.MaxValue);
        Span<ulong> smallValues = stackalloc ulong[2 * ConvexContactCandidate.Words];
        Span<int> smallSigns = stackalloc int[2];
        var small = new ContactQuadratic(smallValues, smallSigns); Set(small, 2, -3);
        small.CopyTo(otherN);
        Assert.Equal(2UL, otherN.Rational[0]); Assert.Equal(3UL, otherN.Radical[0]);
        Assert.Equal(0UL, otherN.Rational[47]); Assert.Equal(0UL, otherN.Radical[47]);
        Assert.Equal(-1, otherN.Signs[1]);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void RootRatioMaterialization_HandlesUpperHalfWithoutOverflow(int boundary)
    {
        Span<ulong> values = stackalloc ulong[4 * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0), d = ContactQuadratic.At(values, signs, 1);
        Set(n, 0, 0); Set(d, 1, 0); n.Signs[0] = 1;
        if (boundary == 0) n.Rational[0] = (ulong)long.MaxValue;
        else if (boundary == 1) n.Rational[0] = (ulong)long.MaxValue + 1;
        else if (boundary == 2) { n.Rational[0] = ulong.MaxValue; d.Rational[0] = 2; }
        else { n.Rational[0] = ulong.MaxValue - 2; n.Rational[1] = 1; d.Rational[0] = 4; }
        bool represented = ContactQuadratic.TryRoundRootRatio(n, d, new ulong[] { 0 },
            new ulong[] { 1 }, new ulong[] { 1 }, out Fixed64 result);
        Assert.Equal(boundary == 0 || boundary == 3, represented);
        Assert.Equal(represented ? long.MaxValue : 0, result.m_rawValue);
    }

    [Theory]
    [InlineData(0, 0, 1, 0, 2, 0)]
    [InlineData(1, 0, 4, 0, 2, 0)]
    [InlineData(9, 0, 4, 0, 2, 2)]
    [InlineData(25, 0, 4, 0, 2, 2)]
    [InlineData(49, 0, 4, 0, 2, 4)]
    [InlineData(2, 0, 1, 0, 2, 1)]
    [InlineData(3, 2, 1, 0, 2, 2)]
    [InlineData(1, 1, 3, 2, 2, 1)]
    [InlineData(-1, 1, 1, 0, 2, 1)]
    [InlineData(3, -2, 1, 0, 2, 0)]
    [InlineData(1, 0, -1, 1, 2, 2)]
    public void SquareRootRatioMaterialization_PreservesFieldAndNearestEven(int a, int b, int c, int d, int k, long expectedRaw)
    {
        Span<ulong> values = stackalloc ulong[4 * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0), denominator = ContactQuadratic.At(values, signs, 1);
        Set(n, a, b); Set(denominator, c, d);
        Assert.True(ContactQuadratic.TryRoundSquareRootRatio(n, denominator, new ulong[] { (ulong)k }, out Fixed64 result));
        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void RationalSquareRootRatios_AgreeWithIndependentSmallIntegerOracle()
    {
        Span<ulong> values = stackalloc ulong[4 * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0), d = ContactQuadratic.At(values, signs, 1);
        for (int numerator = 0; numerator < 100; numerator++)
        for (int denominator = 1; denominator < 18; denominator++)
        {
            int floor = 0;
            while ((floor + 1) * (floor + 1) * denominator <= numerator) floor++;
            int comparison = (4 * numerator).CompareTo(denominator * (2 * floor + 1) * (2 * floor + 1));
            long expected = floor + (comparison > 0 || comparison == 0 && (floor & 1) != 0 ? 1 : 0);
            Set(n, numerator, 0); Set(d, denominator, 0);
            Assert.True(ContactQuadratic.TryRoundSquareRootRatio(n, d, ReadOnlySpan<ulong>.Empty, out Fixed64 result));
            Assert.Equal(expected, result.m_rawValue);
        }
    }

    private static int Compare(int a, int b, int c, int d, int k,
        int e, int f, int g, int h, int l)
    {
        const int words = ConvexContactCandidate.Words;
        Span<ulong> values = stackalloc ulong[8 * words];
        Span<int> signs = stackalloc int[8];
        Span<ulong> firstRoot = stackalloc ulong[words];
        Span<ulong> secondRoot = stackalloc ulong[words];
        firstRoot.Clear(); secondRoot.Clear(); firstRoot[0] = (ulong)k; secondRoot[0] = (ulong)l;
        ContactQuadratic firstNumerator = ContactQuadratic.At(values, signs, 0);
        ContactQuadratic firstDenominator = ContactQuadratic.At(values, signs, 1);
        ContactQuadratic secondNumerator = ContactQuadratic.At(values, signs, 2);
        ContactQuadratic secondDenominator = ContactQuadratic.At(values, signs, 3);
        Set(firstNumerator, a, b); Set(firstDenominator, c, d);
        Set(secondNumerator, e, f); Set(secondDenominator, g, h);
        return ContactQuadratic.CompareRatios(firstNumerator, firstDenominator, firstRoot,
            secondNumerator, secondDenominator, secondRoot);
    }

    private static void Set(ContactQuadratic value, int rational, int radical)
    {
        value.Clear(); value.Rational[0] = (ulong)Math.Abs(rational);
        value.Radical[0] = (ulong)Math.Abs(radical);
        value.Signs[0] = Math.Sign(rational); value.Signs[1] = Math.Sign(radical);
    }
}
