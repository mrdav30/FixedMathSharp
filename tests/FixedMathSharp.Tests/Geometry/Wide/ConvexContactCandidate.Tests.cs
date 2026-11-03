using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class ConvexContactCandidateTests
{
    [Theory]
    [InlineData(5, 25, 1)]
    [InlineData(-5, 25, 0)]
    [InlineData(-5, 24, -1)]
    [InlineData(-5, 26, 1)]
    [InlineData(0, 0, 0)]
    [InlineData(0, 26, 1)]
    public void RadialGapEncoding_CanReuseItsOwnInvariantSlots(int p, int q, int expectedSign)
    {
        foreach (int words in new[] { 24, ConvexContactCandidate.Words })
            foreach (int word in new[] { 0, (words - 2) / 2 })
            {
                var numerator = new ulong[words]; numerator[word] = (ulong)Math.Abs(p);
                var radicand = new ulong[words]; radicand[2 * word] = (ulong)q;
                var denominator = new ulong[words]; denominator[word] = 7;
                var actual = new ulong[ConvexContactCandidate.Slots * words];
                var actualSigns = new int[ConvexContactCandidate.Slots];
                var expected = new ulong[actual.Length];
                var expectedSigns = new int[actualSigns.Length];
                CylinderContactAlgebra.BuildRadialCandidate(numerator, -Math.Sign(p), radicand, denominator, actual, actualSigns);
                int sign = CylinderContactAlgebra.BuildRadialCandidate(numerator, Math.Sign(p),
                    actual.AsSpan(9 * words, words), actual.AsSpan(10 * words, words), actual, actualSigns);
                Assert.Equal(expectedSign, sign);
                Assert.Equal(sign, CylinderContactAlgebra.BuildRadialCandidate(numerator, Math.Sign(p),
                    radicand, denominator, expected, expectedSigns));
                Assert.Equal(expected, actual);
                Assert.Equal(expectedSigns, actualSigns);
                var candidate = new ConvexContactCandidate(actual, actualSigns, sign, words);
                BigInteger scale = BigInteger.One << (64 * word);
                Assert.Equal((p * p + q) * scale * scale, Magnitude(candidate.GapRational));
                Assert.Equal(2 * p * scale, candidate.GapRadicalSign * Magnitude(candidate.GapRadical));
                Assert.Equal(q * scale * scale, Magnitude(candidate.GapRadicand));
                Assert.Equal(7 * scale, Magnitude(candidate.GapDenominator));
            }
    }

    [Theory]
    [InlineData(0, 0, 3, 0)]
    [InlineData(0, 2, 3, 1)]
    [InlineData(0, -2, 3, -1)]
    [InlineData(0, -2, 0, 0)]
    [InlineData(3, 0, 2, 1)]
    [InlineData(-3, 0, 2, -1)]
    [InlineData(3, -2, 2, 1)]
    [InlineData(-3, 2, 2, -1)]
    [InlineData(2, -1, 4, 0)]
    [InlineData(-2, 1, 4, 0)]
    [InlineData(1, -2, 3, -1)]
    [InlineData(-1, 2, 3, 1)]
    public void QuadraticGapEncoding_PreservesExactSquareAndOriginalSign(int a, int b, int k, int expectedSign)
    {
        foreach (int word in new[] { 0, 12 })
        {
            const int words = ConvexContactCandidate.Words;
            var numeratorValues = new ulong[2 * words];
            var numeratorSigns = new[] { Math.Sign(a), Math.Sign(b) };
            numeratorValues[word] = (ulong)Math.Abs(a);
            numeratorValues[words + word] = (ulong)Math.Abs(b);
            var root = new ulong[words]; root[0] = (ulong)k;
            var denominator = new ulong[words]; denominator[0] = 7;
            var values = new ulong[ConvexContactCandidate.Slots * words];
            var signs = new int[ConvexContactCandidate.Slots];
            int gapSign = CylinderContactAlgebra.BuildQuadraticCandidate(
                new ContactQuadratic(numeratorValues, numeratorSigns), root, denominator, values, signs);
            var candidate = new ConvexContactCandidate(values, signs, gapSign);
            // Independent integer coefficients of (A+B*sqrt(K))², including
            // opposite signs, exact cancellation and coefficients beyond 64 bits.
            BigInteger scale = BigInteger.One << (128 * word);
            BigInteger rational = (a * a + b * b * k) * scale;
            BigInteger radical = (2 * a * b) * scale;
            Assert.Equal(rational, Magnitude(candidate.GapRational));
            Assert.Equal(radical, candidate.GapRadicalSign * Magnitude(candidate.GapRadical));
            Assert.Equal(rational.Sign, signs[7]);
            Assert.Equal(radical.Sign, signs[8]);
            Assert.Equal((BigInteger)k, Magnitude(candidate.GapRadicand));
            Assert.Equal((BigInteger)7, Magnitude(candidate.GapDenominator));
            Assert.Equal(expectedSign, gapSign);
        }
    }

    [Theory]
    [InlineData(1L << 32)]
    [InlineData(17L << 32)]
    [InlineData(long.MaxValue / 4)]
    public void BoxCylinderAnalyticRanking_PreservesExactDepthAcrossWideCoordinateScales(long radiusRaw)
    {
        Fixed64 radius = Fixed64.FromRaw(radiusRaw);
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.FromRaw(2 * radiusRaw + 1), radius, radius));
        const long scale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-scale), Fixed64.FromRaw(2 * scale));
        // Both shapes contain a radius-r ball. Forward is perpendicular to
        // the cylinder axis and attains exactly 2r, proving the minimum.
        // The extra X raw unit prevents trivial common-coordinate reduction.
        Assert.True(box.TryGetCenteredCylinderContact(Vector3d.Zero, rotation, Vector3d.Up,
            Fixed64.FromRaw(2 * radiusRaw), radius, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(2 * radiusRaw), contact.Depth);
        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void BoxCylinderAnalyticRanking_PreservesRawDenominatorOnVertexRimSeparation()
    {
        const long scale = 784_150_157L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(2 * scale), Fixed64.Zero,
            Fixed64.FromRaw(-scale), Fixed64.FromRaw(5 * scale));
        Fixed64 delta = Fixed64.FromFraction(1, 64);
        Vector3d center = new(-(Fixed64)2 - delta, (Fixed64)2 + delta, (Fixed64)5 + (Fixed64)4 * delta);
        var geometry = new BoxCylinderGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One,
            center, rotation, Vector3d.Up, (Fixed64)6, (Fixed64)3);
        ulong[] values = new ulong[ConvexContactCandidate.Slots * ConvexContactCandidate.Words];
        int[] signs = new int[ConvexContactCandidate.Slots];
        // The existing vertex/rim fixture separates despite positive rational
        // analytic directions. Its early miss must still return raw gap units.
        Assert.False(BoxCylinderAnalyticFeatures.TryGetBest(geometry, values, signs, out int gapSign, out _));
        Assert.Equal(-1, gapSign);
        var candidate = new ConvexContactCandidate(values, signs, gapSign);
        BigInteger u = 0, n2 = 0;
        for (int component = 0; component < 3; component++)
        {
            BigInteger a = Signed(BoxCylinderGeometry.GetComponent(geometry.Axis, component));
            BigInteger n = Magnitude(candidate.NormalRational(component));
            u += a * a;
            n2 += n * n;
        }
        BigInteger s = Signed(Signed320.ExtendValue(geometry.RawScale));
        // Stored world normals include the rational basis denominator;
        // account for its square when recovering the local direction length.
        BigInteger basis = Signed(Signed320.ExtendValue(geometry.WorldBasis.Denominator));
        Assert.Equal(u * s * s * n2, Magnitude(candidate.GapDenominator) * basis * basis);
    }

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
    [InlineData(1, 4, 8, 1)]
    [InlineData(1, 4, 10, -1)]
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

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RadicalGapRanking_PreservesWideNearCancellationAndScaledRootTies(bool negative)
    {
        BigInteger t = (BigInteger.One << 160) + 17;
        foreach (var (rationalDifference, leftRoot, leftCoefficient, rightRoot, rightCoefficient, expected) in new[]
        {
            (0, t * t + 1, BigInteger.One, t * t + 2, BigInteger.One, -1),
            (0, 4 * (t * t + 1), BigInteger.One, t * t + 1, new BigInteger(2), 0),
            (0, (BigInteger.One << 192) - 1, BigInteger.One, BigInteger.One << 192, BigInteger.One, -1),
            (1, t * t + 1, BigInteger.One, (t + 1) * (t + 1) + 1, BigInteger.One, 1),
            (1, t * t, BigInteger.One, (t + 1) * (t + 1), BigInteger.One, 0)
        })
        {
            // sqrt(T²+1)>T proves 1+sqrt(T²+1)>sqrt((T+1)²+1).
            BigInteger rational = BigInteger.One << 200;
            var left = EncodeWideRoot(leftRoot, leftCoefficient, rational + (negative ? -rationalDifference : rationalDifference));
            var right = EncodeWideRoot(rightRoot, rightCoefficient, rational);
            int[] signs = new int[ConvexContactCandidate.Slots];
            signs[7] = 1; signs[8] = negative ? -1 : 1;
            var a = new ConvexContactCandidate(left, signs, 1);
            var b = new ConvexContactCandidate(right, signs, 1);
            int sign = negative ? -expected : expected;
            Assert.Equal(sign, WideConvexPrismRelations.CompareConvexContactCandidates(a, b));
            Assert.Equal(-sign, WideConvexPrismRelations.CompareConvexContactCandidates(b, a));
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RationalPrismCandidate_MatchesUnboundedSupportOracleAtBothFieldWidths(bool cone, bool reverse)
    {
        // Full raw origin/footprint differences and a 504-bit normal stress the
        // complete width proof, rather than only comparing two bounded builders.
        Vector2d[] footprint = {
            new(Fixed64.FromRaw(long.MinValue + 7), Fixed64.FromRaw(long.MinValue + 9)),
            new(Fixed64.FromRaw(long.MaxValue - 11), Fixed64.FromRaw(long.MinValue + 9)),
            new(Fixed64.FromRaw(long.MaxValue - 11), Fixed64.FromRaw(long.MaxValue - 13)),
            new(Fixed64.FromRaw(long.MinValue + 7), Fixed64.FromRaw(long.MaxValue - 13)) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(
            new Vector3d(Fixed64.FromRaw(long.MaxValue - 16), Fixed64.FromRaw(long.MinValue + 32), Fixed64.FromRaw(long.MaxValue - 64)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)22, (Fixed64)17, (Fixed64)9),
            new Vector3d(Fixed64.FromRaw((1L << 32) - 1), Fixed64.FromRaw(1), Fixed64.FromRaw(1)),
            Fixed64.FromRaw(long.MaxValue - 3), Fixed64.FromRaw((long.MaxValue >> 3) - 5),
            new Vector3d(Fixed64.FromRaw(long.MinValue + 64), Fixed64.FromRaw(long.MaxValue - 32), Fixed64.FromRaw(long.MinValue + 16)),
            Fixed64.FromFraction(1, 3), footprint, Fixed64.FromRaw(long.MaxValue - 15), cone, vertices);
        int orientation = reverse ? -1 : 1;
        BigInteger x = orientation * ((BigInteger.One << 503) + 7);
        BigInteger y = -orientation * ((BigInteger.One << 502) + 3);
        BigInteger z = orientation * ((BigInteger.One << 501) + 11);
        var normal = new WideDirection3d(Integer576(x), Integer576(y), Integer576(z));
        BigInteger minimum = 0;
        int minimumVertex = -1;
        for (int index = 0; index < geometry.VertexCount; index++)
        {
            WideDirection3d vertex = geometry.Vertex(index);
            BigInteger projection = x * Signed(vertex.X) + y * Signed(vertex.Y) + z * Signed(vertex.Z);
            if (minimumVertex >= 0 && projection >= minimum) continue;
            minimum = projection; minimumVertex = index;
        }
        BigInteger metric = x * x + y * y + z * z;
        BigInteger half = x * Signed(geometry.Half.X) + y * Signed(geometry.Half.Y) + z * Signed(geometry.Half.Z);
        BigInteger q = Signed(geometry.SquaredAxis);
        BigInteger axial = x * Signed(geometry.Axis.X) + y * Signed(geometry.Axis.Y) + z * Signed(geometry.Axis.Z);
        BigInteger radial = Signed(geometry.Radius) * Signed(geometry.Radius) * (q * metric - axial * axial);
        int comparison = half.Sign < 0 ? -1 : (4 * q * half * half).CompareTo(radial);
        bool apex = cone && comparison >= 0;
        int expectedFeature = cone ? comparison == 0 ? 0 : apex ? 1 : -1 : half.Sign;
        BigInteger p = (cone ? apex ? half : -half : BigInteger.Abs(half)) - minimum;
        if (!apex) p *= q;
        BigInteger root = apex ? BigInteger.Zero : q * radial;
        BigInteger scale = Signed(Signed320.ExtendValue(geometry.RawScale));
        BigInteger[] oracle = { x, y, z, 0, 0, 0, 0, p * p + root, 2 * p, root, scale * scale * metric * (apex ? BigInteger.One : q * q) };
        int expectedGap = p.Sign < 0 ? root.CompareTo(p * p) : p.Sign > 0 || root.Sign > 0 ? 1 : 0;
        foreach (int words in new[] { 24, ConvexContactCandidate.Words })
        {
            var direction = new ulong[3 * words];
            var directionSigns = new int[3];
            var values = new ulong[ConvexContactCandidate.Slots * words];
            var signs = new int[ConvexContactCandidate.Slots];
            Assert.True(geometry.TryBuildCandidate(normal, direction, directionSigns, values, signs,
                -1, out int gap, out int vertex, out int feature));
            Assert.Equal(expectedGap, gap);
            Assert.Equal(minimumVertex, vertex);
            Assert.Equal(expectedFeature, feature);
            for (int slot = 0; slot < oracle.Length; slot++)
            {
                Assert.Equal(BigInteger.Abs(oracle[slot]), Magnitude(values.AsSpan(slot * words, words)));
                if (slot < 6 || slot == 7 || slot == 8) Assert.Equal(oracle[slot].Sign, signs[slot]);
            }
        }
    }

    [Fact]
    public void CompactPrismSelection_ClearsUninitializedPaddingAndRetainsTheFirstExactTie()
    {
        Vector2d[] footprint = { new(-Fixed64.One, -Fixed64.One), new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One), new(-Fixed64.One, Fixed64.One) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.One, false, vertices);
        const int words = 24;
        var compact = new ulong[ConvexContactCandidate.Slots * words];
        var signs = new int[ConvexContactCandidate.Slots];
        var direction = new ulong[3 * words];
        var directionSigns = new int[3];
        // HasValue=false permits arbitrary retained storage. Every padding limb
        // must be overwritten on the first successful compact admission.
        var retained = new ulong[ConvexContactCandidate.Slots * ConvexContactCandidate.Words];
        Array.Fill(retained, ulong.MaxValue);
        var retainedSigns = new int[ConvexContactCandidate.Slots];
        Array.Fill(retainedSigns, -1);
        scoped var selection = new CurvedPrismContactSelection(retained, retainedSigns);
        var right = new WideDirection3d(Signed576.One, default, default);
        Assert.True(geometry.TryBuildCandidate(right, direction, directionSigns, compact, signs,
            -1, out int gap, out int vertex, out int feature));
        selection.Keep(geometry, compact, signs, gap, vertex, feature, words);
        Assert.True(selection.HasValue);
        Assert.Equal(gap, selection.GapSign);
        Assert.Equal(vertex, selection.Vertex);
        Assert.Equal(feature, selection.Feature);
        Assert.Equal(signs, retainedSigns);
        for (int slot = 0; slot < ConvexContactCandidate.Slots; slot++)
        {
            Assert.Equal(compact.AsSpan(slot * words, words).ToArray(),
                retained.AsSpan(slot * ConvexContactCandidate.Words, words).ToArray());
            foreach (ulong limb in retained.AsSpan(slot * ConvexContactCandidate.Words + words,
                ConvexContactCandidate.Words - words)) Assert.Equal(0UL, limb);
        }
        var snapshot = (ulong[])retained.Clone();
        var signSnapshot = (int[])retainedSigns.Clone();
        int firstVertex = vertex, firstFeature = feature;
        // Both sides of the centered cube/cylinder have depth two. The later
        // opposite direction must keep the earlier normal and support witness.
        Assert.True(geometry.TryBuildCandidate(-right, direction, directionSigns, compact, signs,
            -1, out gap, out vertex, out feature));
        selection.Keep(geometry, compact, signs, gap, vertex, feature, words);
        Assert.Equal(snapshot, retained);
        Assert.Equal(signSnapshot, retainedSigns);
        Assert.Equal(firstVertex, selection.Vertex);
        Assert.Equal(firstFeature, selection.Feature);
    }

    private static Signed576 Integer576(BigInteger value)
    {
        BigInteger magnitude = BigInteger.Abs(value);
        var words = new ulong[9];
        for (int word = 0; word < words.Length; word++, magnitude >>= 64)
            words[word] = (ulong)(magnitude & ulong.MaxValue);
        Assert.Equal(BigInteger.Zero, magnitude);
        var result = new Signed576(words[8], words[7], words[6], words[5], words[4], words[3], words[2], words[1], words[0]);
        return value.Sign < 0 ? WideArithmetic.Negate(result) : result;
    }

    private static BigInteger Signed(Signed576 value)
    {
        Span<ulong> words = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(value, words);
        return value.Sign * Magnitude(words);
    }

    private static ulong[] EncodeWideRoot(BigInteger radicand, BigInteger coefficient, BigInteger rational)
    {
        var values = new ulong[ConvexContactCandidate.Slots * ConvexContactCandidate.Words];
        Write(7, rational);
        Write(8, coefficient); Write(9, radicand); Write(10, BigInteger.One);
        return values;

        void Write(int slot, BigInteger value)
        {
            for (int word = 0; value != 0; word++, value >>= 64)
                values[slot * ConvexContactCandidate.Words + word] = (ulong)(value & ulong.MaxValue);
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
