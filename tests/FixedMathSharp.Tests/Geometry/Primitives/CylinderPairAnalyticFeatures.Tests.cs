using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairAnalyticFeaturesTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Build_MatchesIndependentIntegerSupportFormula(int axisIndex)
    {
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(new Vector3d(-2, 3, 1),
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw(Fixed64.FromRaw(13)), Fixed64.FromRaw(7), new Vector3d(4, -1, 8),
            new FixedQuaternion(Fixed64.Zero, s, Fixed64.Zero, 2 * s), Vector3d.Left,
            Signed192.Raw(Fixed64.FromRaw(19)), Fixed64.FromRaw(11));
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        Array.Fill(values, ulong.MaxValue);
        Array.Fill(signs, -1);
        CylinderPairAnalyticFeatures.Build(geometry, axisIndex, values, signs, out int gapSign);
        BigInteger[] a = Axis(geometry.FirstAxis), b = Axis(geometry.SecondAxis);
        BigInteger[] n = axisIndex == 0 ? a : axisIndex == 1 ? b : Cross(a, b);
        BigInteger c = BigInteger.Abs(Dot(Axis(geometry.FirstHalf), n))
            + BigInteger.Abs(Dot(Axis(geometry.SecondHalf), n))
            - BigInteger.Abs(Dot(Axis(geometry.CenterDifference), n));
        BigInteger delta = axisIndex == 2 ? BigInteger.One : Dot(axisIndex == 0 ? b : a, axisIndex == 0 ? b : a);
        BigInteger nSquared = Dot(n, n);
        BigInteger projection = axisIndex == 2 ? nSquared
            : delta * nSquared - BigInteger.Pow(Dot(axisIndex == 0 ? b : a, n), 2);
        BigInteger radius = axisIndex == 0 ? Integer(geometry.SecondRadius)
            : axisIndex == 1 ? Integer(geometry.FirstRadius)
            : Integer(geometry.FirstRadius) + Integer(geometry.SecondRadius);
        BigInteger scale = Integer(geometry.RawScale);
        BigInteger[] expected = { c * c * delta + radius * radius * projection,
            2 * c * radius, projection * delta, delta * nSquared * scale * scale };
        for (int index = 0; index < expected.Length; index++)
            Assert.Equal(expected[index], Slot(values, signs, index + 7));
        Assert.Equal(c.Sign < 0 ? (radius * radius * projection).CompareTo(c * c * delta)
            : c.Sign > 0 || projection > 0 ? 1 : 0, gapSign);
        for (int index = 3; index <= 6; index++)
            Assert.Equal(BigInteger.Zero, Slot(values, signs, index));
    }

    [Theory]
    [InlineData(0, 0, 3)]
    [InlineData(0, 2, 1)]
    [InlineData(0, 3, 0)]
    [InlineData(0, 4, -1)]
    [InlineData(1, 2, 1)]
    [InlineData(2, 3, 1)]
    public void AxisCandidates_PreserveSignedGapAndExactNormal(int axisIndex, int separation, int gap)
    {
        Vector3d direction = axisIndex == 0 ? Vector3d.Up : axisIndex == 1 ? Vector3d.Right : Vector3d.Forward;
        var geometry = Geometry(direction * (Fixed64)separation);
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        CylinderPairAnalyticFeatures.Build(geometry, axisIndex, values, signs, out int gapSign);
        Assert.Equal(Math.Sign(gap), gapSign);
        var candidate = new ConvexContactCandidate(values, signs, gapSign);
        Assert.Equal(direction, WideConvexPrismRelations.GetConvexContactCandidateNormal(candidate));
        if (gap >= 0)
        {
            WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(candidate, Fixed64.Zero,
                out Fixed64 depth, out bool clamped);
            Assert.Equal((Fixed64)gap, depth);
            Assert.False(clamped);
        }
    }

    [Theory]
    [InlineData(0, 4, -1)]
    [InlineData(0, 5, -1)]
    [InlineData(0, 9, 0)]
    [InlineData(0, 16, 1)]
    [InlineData(2, 1, 0)]
    [InlineData(2, 4, 1)]
    [InlineData(2, 5, 1)]
    [InlineData(1, 1, -1)]
    [InlineData(1, 4, 0)]
    [InlineData(1, 9, 1)]
    public void RootComparison_PreservesUnsquaredRadicalBranch(int separation, int squaredDepth, int comparison)
    {
        var geometry = Geometry(Vector3d.Up * (Fixed64)separation);
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        CylinderPairAnalyticFeatures.Build(geometry, 0, values, signs, out int gapSign);
        BigInteger scale = Integer(geometry.RawScale);
        ulong[] polynomial = new ulong[16];
        sbyte[] polynomialSigns = { -1, 1 };
        Write((BigInteger)squaredDepth * (BigInteger.One << 64) * scale * scale, polynomial.AsSpan(0, 8));
        Write(BigInteger.One << geometry.ValueShift, polynomial.AsSpan(8, 8));
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, polynomialSigns)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, polynomialSigns, 0, cell, out FiniteAxisValueRoot root));
        Assert.Equal(comparison, ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, ref root,
            new ConvexContactCandidate(values, signs, gapSign)));
    }

    [Theory]
    [InlineData(575, -1)]
    [InlineData(576, 0)]
    [InlineData(577, 1)]
    public void RootComparison_PreservesNontrivialRawScale(int numerator, int comparison)
    {
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw((Fixed64)10), Fixed64.One, new Vector3d(0, 1, 8),
            new FixedQuaternion(Fixed64.Zero, s, Fixed64.Zero, 2 * s), Vector3d.Left,
            Signed192.Raw((Fixed64)10), Fixed64.One);
        Assert.Equal((BigInteger)5, Integer(geometry.RawScale));
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        CylinderPairAnalyticFeatures.Build(geometry, 0, values, signs, out int gapSign);
        // c=0, b=(7,0,24), radius=5Q: S=576Q²/25.
        ulong[] polynomial = new ulong[16];
        sbyte[] polynomialSigns = { -1, 1 };
        Write((BigInteger)numerator << 64, polynomial.AsSpan(0, 8));
        Write((BigInteger)25 << geometry.ValueShift, polynomial.AsSpan(8, 8));
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, polynomialSigns)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, polynomialSigns, 0, cell, out FiniteAxisValueRoot root));
        Assert.Equal(comparison, ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, ref root,
            new ConvexContactCandidate(values, signs, gapSign)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RootComparison_ShouldRetainDiscoveredRationalCellAndMetadata(bool radicalCandidate)
    {
        ulong[] polynomial = { 3, 8 };
        sbyte[] polynomialSigns = { -1, 1 };
        ulong[] cell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, polynomialSigns,
            0, cell, out FiniteAxisValueRoot root));
        Assert.False(root.IsRational);
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        values[0] = 1;
        signs[0] = 1;
        // The analytic squared gap is 3/8 in both cases. The radical case
        // discovers equality in the squared query after the linear sign guard.
        int numeratorSlot = radicalCandidate ? 8 : 7;
        values[numeratorSlot * ConvexContactCandidate.Words] = 3;
        signs[numeratorSlot] = 1;
        if (radicalCandidate)
            values[9 * ConvexContactCandidate.Words] = 1;
        values[10 * ConvexContactCandidate.Words] = 8;

        Assert.Equal(0, ConvexContactValueRoot.CompareRootSquared(Signed192.Signed(1), 0, ref root,
            new ConvexContactCandidate(values, signs, 1)));

        Assert.True(root.IsRational);
        Assert.Equal(3, root.DenominatorShift);
        Assert.Equal(3UL, cell[0]);
        for (int index = 1; index < cell.Length; index++)
            Assert.Equal(0UL, cell[index]);
        Assert.Equal(0, root.Ordinal);
        Assert.True(root.Coefficients.SequenceEqual(polynomial));
        Assert.True(root.Signs.SequenceEqual(polynomialSigns));
        Assert.True(ConvexContactValueRoot.GetRoundedDepth(Signed192.Signed(1), 0, ref root,
            out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.MinIncrement, depth);
        Assert.False(clamped);
    }

    [Fact]
    public void RootComparison_ShouldRetainRefinedIrrationalCellWithoutChangingRootIdentity()
    {
        ulong[] polynomial = { 1, 0, 2 };
        sbyte[] polynomialSigns = { -1, 0, 1 };
        ulong[] cell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, polynomialSigns,
            0, cell, out FiniteAxisValueRoot root));
        int originalShift = root.DenominatorShift;
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        values[0] = 1;
        signs[0] = 1;
        values[7 * ConvexContactCandidate.Words] = 7;
        signs[7] = 1;
        values[10 * ConvexContactCandidate.Words] = 10;
        var candidate = new ConvexContactCandidate(values, signs, 1);

        // sqrt(1/2) > 7/10; the original coarse cell straddles 7/10.
        Assert.Equal(1, ConvexContactValueRoot.CompareRootSquared(Signed192.Signed(1), 0, ref root, candidate));

        Assert.True(root.DenominatorShift > originalShift);
        Assert.False(root.IsRational);
        Assert.Equal(0, root.Ordinal);
        Assert.True(root.Coefficients.SequenceEqual(polynomial));
        Assert.True(root.Signs.SequenceEqual(polynomialSigns));
        int refinedShift = root.DenominatorShift;
        Assert.Equal(1, ConvexContactValueRoot.CompareRootSquared(Signed192.Signed(1), 0, ref root, candidate));
        Assert.True(root.DenominatorShift >= refinedShift);
        BigInteger lowerNumerator = BigInteger.Zero;
        for (int index = cell.Length - 1; index >= 0; index--)
            lowerNumerator = (lowerNumerator << 64) | cell[index];
        BigInteger denominatorSquared = BigInteger.One << (2 * root.DenominatorShift);
        Assert.True(2 * lowerNumerator * lowerNumerator < denominatorSquared);
        Assert.True(2 * (lowerNumerator + 1) * (lowerNumerator + 1) > denominatorSquared);
    }

    [Theory]
    [InlineData(1, 2)]
    [InlineData(3, 2)]
    public void Depth_RoundsHalfRawTiesOnceToEven(int lengthRaw, int expectedRaw)
    {
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw(Fixed64.FromRaw(lengthRaw)), Fixed64.FromRaw(1), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw(Fixed64.FromRaw(1)), Fixed64.FromRaw(1));
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        CylinderPairAnalyticFeatures.Build(geometry, 0, values, signs, out int gapSign);
        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(new ConvexContactCandidate(values, signs, gapSign),
            Fixed64.Zero, out Fixed64 depth, out bool clamped);
        Assert.Equal((long)expectedRaw, depth.m_rawValue);
        Assert.False(clamped);
    }

    [Fact]
    public void Normal_UsesExactWorldRotationBeforeNearestEvenRounding()
    {
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, s, 2 * s), Vector3d.Up,
            Signed192.Raw(Fixed64.Two), Fixed64.One, new Vector3d(-4, 3, 0), FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw(Fixed64.Two), Fixed64.One);
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        CylinderPairAnalyticFeatures.Build(geometry, 0, values, signs, out int gapSign);
        Vector3d normal = WideConvexPrismRelations.GetConvexContactCandidateNormal(new ConvexContactCandidate(values, signs, gapSign));
        Assert.Equal(-3435973837L, normal.X.m_rawValue);
        Assert.Equal(2576980378L, normal.Y.m_rawValue);
        Assert.Equal(0L, normal.Z.m_rawValue);
    }

    [Theory]
    [InlineData(1L, 1UL, 1, 0L, 1L)]
    [InlineData(2L, 3UL, 1, 2L, 1L)]
    [InlineData(2L, 3UL, -1, -2L, 1L)]
    [InlineData(4_294_967_296L, 1UL, 1, 0L, 4_294_967_296L)]
    [InlineData(long.MaxValue, 3UL, -1, -2L, long.MaxValue)]
    public void ScaledAnalyticNormal_PreservesExactHalfRawTies(
        long scaleRaw, ulong odd, int orientation, long expectedRaw, long expectedY)
    {
        // n=(odd,sqrt((2S)^2-odd^2),0) has norm exactly 2S. Scaling
        // its X component by S therefore produces an exact odd/2 raw tie.
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        values[0] = odd; signs[0] = orientation;
        values[4 * ConvexContactCandidate.Words] = 1; signs[4] = 1;
        Write(4 * (BigInteger)scaleRaw * scaleRaw - odd * odd,
            values.AsSpan(6 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        ulong[] original = (ulong[])values.Clone();
        Vector3d normal = WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(
            new ConvexContactCandidate(values, signs, 0), Fixed64.FromRaw(scaleRaw));
        Assert.Equal(expectedRaw, normal.X.m_rawValue);
        Assert.Equal(expectedY, normal.Y.m_rawValue);
        Assert.Equal(0L, normal.Z.m_rawValue);
        Assert.Equal(original, values);
    }

    [Theory]
    [InlineData(64, 0L, false)]
    [InlineData(64, 1L, false)]
    [InlineData(64, 4_294_967_296L, false)]
    [InlineData(2500, long.MaxValue, false)]
    [InlineData(2500, long.MaxValue, true)]
    public void ScaledAnalyticNormal_PreservesWideSharedRadicalCancellation(
        int coefficientBits, long scaleRaw, bool wideRadicand)
    {
        // A common positive factor (2^bits-1)-sqrt(2) cancels exactly.
        // The resulting direction is (-3,4,0)/5 at every authored scale.
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        BigInteger common = (BigInteger.One << coefficientBits) - 1;
        Write(3 * common, values.AsSpan(0, ConvexContactCandidate.Words)); signs[0] = -1;
        Write(4 * common, values.AsSpan(ConvexContactCandidate.Words, ConvexContactCandidate.Words)); signs[1] = 1;
        values[3 * ConvexContactCandidate.Words] = 3; signs[3] = 1;
        values[4 * ConvexContactCandidate.Words] = 4; signs[4] = -1;
        values[6 * ConvexContactCandidate.Words] = 2;
        if (wideRadicand)
        {
            // H*(1+sqrt(H)) fills all three field widths. Its exact common
            // factor cancels too, while exercising the 122-word norm slots.
            Write(3 * common, values.AsSpan(3 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
            Write(4 * common, values.AsSpan(4 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
            Write(common, values.AsSpan(6 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
            signs[3] = -1; signs[4] = 1;
        }
        ulong[] original = (ulong[])values.Clone();
        Vector3d normal = WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(
            new ConvexContactCandidate(values, signs, 0), Fixed64.FromRaw(scaleRaw));
        Assert.Equal(-RoundFifths(3 * (BigInteger)scaleRaw), normal.X.m_rawValue);
        Assert.Equal(RoundFifths(4 * (BigInteger)scaleRaw), normal.Y.m_rawValue);
        Assert.Equal(0L, normal.Z.m_rawValue);
        Assert.Equal(original, values);
    }

    [Theory]
    [InlineData(1, 3, 1L)]
    [InlineData(1, 3, 4_294_967_296L)]
    [InlineData(1, 3, long.MaxValue)]
    [InlineData(2, 3, 1L)]
    [InlineData(2, 3, 4_294_967_296L)]
    [InlineData(2, 3, long.MaxValue)]
    [InlineData(4, 2, 1L)]
    [InlineData(4, 2, 4_294_967_296L)]
    [InlineData(4, 2, long.MaxValue)]
    [InlineData(1, 0, 1L)]
    [InlineData(1, 0, long.MaxValue)]
    public void ScaledAnalyticNormal_PreservesCanceledDirectionsAcrossUncertainDenominatorBounds(
        int rationalFactor, int smallRadicand, long scaleRaw)
    {
        // n=(-3,4,0)*(H-sqrt(C)). The common factor cancels exactly,
        // including its sign. With the integer sqrt(C) enclosure [1,2],
        // (H,C)=(1,3) gives norm lower bound 0 and (2,3) gives -25,
        // although both exact norms are positive; (4,2) has a positive bound.
        // The forty-word radicand also exercises an upper-root carry beyond
        // the twenty words sufficient for its floor root.
        BigInteger radicand = smallRadicand == 0
            ? (BigInteger.One << (64 * ConvexContactCandidate.Words)) - 1
            : smallRadicand;
        int factorSign = (rationalFactor * rationalFactor - radicand).Sign;
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        values[0] = (ulong)(3 * rationalFactor); signs[0] = -1;
        values[ConvexContactCandidate.Words] = (ulong)(4 * rationalFactor); signs[1] = 1;
        values[3 * ConvexContactCandidate.Words] = 3; signs[3] = 1;
        values[4 * ConvexContactCandidate.Words] = 4; signs[4] = -1;
        Write(radicand, values.AsSpan(6 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        ulong[] originalValues = (ulong[])values.Clone();
        int[] originalSigns = (int[])signs.Clone();

        Vector3d normal = WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(
            new ConvexContactCandidate(values, signs, 0), Fixed64.FromRaw(scaleRaw));

        Assert.Equal(-factorSign * RoundFifths(3 * (BigInteger)scaleRaw), normal.X.m_rawValue);
        Assert.Equal(factorSign * RoundFifths(4 * (BigInteger)scaleRaw), normal.Y.m_rawValue);
        Assert.Equal(0L, normal.Z.m_rawValue);
        Assert.Equal(originalValues, values);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(1, 0L, 1, false, -1)]
    [InlineData(1, 0L, 1, false, 0)]
    [InlineData(1, 0L, 1, false, 1)]
    [InlineData(1, 1L, 1, true, -1)]
    [InlineData(1, 1L, 1, true, 0)]
    [InlineData(1, 1L, 1, true, 1)]
    [InlineData(-1, 2L, 3, false, -1)]
    [InlineData(-1, 2L, 3, false, 0)]
    [InlineData(-1, 2L, 3, false, 1)]
    [InlineData(-1, 3L, 3, true, -1)]
    [InlineData(-1, 3L, 3, true, 0)]
    [InlineData(-1, 3L, 3, true, 1)]
    public void AnalyticDepth_PreservesHalfRawNeighborsAndTotalDepthParity(
        int gapSign, long radiusRaw, int odd, bool negativeRational, int offset)
    {
        BigInteger common = (BigInteger.One << 200) + 1;
        BigInteger numerator = odd * odd * common + offset;
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        // Both A=T+2,B=-1 and A=-T,B=T give A+B*sqrt(4)=T.
        // The exact squared gap is odd^2/4 + offset/(4*common).
        Write(negativeRational ? numerator : numerator + 2,
            values.AsSpan(7 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        signs[7] = negativeRational ? -1 : 1;
        Write(negativeRational ? numerator : BigInteger.One,
            values.AsSpan(8 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        signs[8] = negativeRational ? 1 : -1;
        values[9 * ConvexContactCandidate.Words] = 4;
        Write(4 * common, values.AsSpan(10 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        ulong[] originalValues = (ulong[])values.Clone();
        int[] originalSigns = (int[])signs.Clone();
        long midpointFloor = radiusRaw + (gapSign > 0 ? (odd - 1) / 2 : -(odd + 1) / 2);
        long increment = offset == 0 ? midpointFloor & 1L : gapSign * offset > 0 ? 1L : 0L;

        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(
            new ConvexContactCandidate(values, signs, gapSign), Fixed64.FromRaw(radiusRaw),
            out Fixed64 depth, out bool clamped);

        Assert.Equal(midpointFloor + increment, depth.m_rawValue);
        Assert.False(clamped);
        Assert.Equal(originalValues, values);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void AnalyticDepth_ClassifiesExactMaximumNeighborsBeforeRoundedClamping(int offset)
    {
        BigInteger common = (BigInteger.One << 200) + 1;
        BigInteger twiceMaximum = 2 * (BigInteger)long.MaxValue;
        BigInteger numerator = twiceMaximum * twiceMaximum * common + offset;
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        // A=-T,B=T,C=4 again leaves exact positive numerator T.
        // The +/-1 neighbors round to MaxValue but only +1 exceeds it.
        Write(numerator, values.AsSpan(7 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        Write(numerator, values.AsSpan(8 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        signs[7] = -1; signs[8] = 1;
        values[9 * ConvexContactCandidate.Words] = 4;
        Write(4 * common, values.AsSpan(10 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        ulong[] originalValues = (ulong[])values.Clone();
        int[] originalSigns = (int[])signs.Clone();

        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(
            new ConvexContactCandidate(values, signs, 1), Fixed64.Zero,
            out Fixed64 depth, out bool clamped);

        Assert.Equal(long.MaxValue, depth.m_rawValue);
        Assert.Equal(offset > 0, clamped);
        Assert.Equal(originalValues, values);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(-1, 1L)]
    [InlineData(0, 2L)]
    [InlineData(1, 2L)]
    public void AnalyticDepth_PreservesHalfRawNeighborsWithFortyWordRadicandCancellation(
        int offset, long expectedRaw)
    {
        BigInteger common = (BigInteger.One << 200) + 1;
        BigInteger numerator = common + offset;
        BigInteger radicalRoot = (BigInteger.One << 1279) + (BigInteger.One << 1100) + 3;
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        // The forty-word perfect square has nonzero discarded bits in the
        // prefix enclosure. Its sqrt is still exactly radicalRoot, so the
        // signed cancellation -(radicalRoot-T)+sqrt(C) leaves T without
        // any approximation, even though the enclosure's numerator minimum
        // is negative. Radius one adds before final nearest-even rounding.
        Write(radicalRoot - numerator,
            values.AsSpan(7 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        signs[7] = -1;
        values[8 * ConvexContactCandidate.Words] = 1; signs[8] = 1;
        Write(radicalRoot * radicalRoot,
            values.AsSpan(9 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        Write(4 * common, values.AsSpan(10 * ConvexContactCandidate.Words, ConvexContactCandidate.Words));
        ulong[] originalValues = (ulong[])values.Clone();
        int[] originalSigns = (int[])signs.Clone();

        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(
            new ConvexContactCandidate(values, signs, 1), Fixed64.FromRaw(1),
            out Fixed64 depth, out bool clamped);

        Assert.Equal(expectedRaw, depth.m_rawValue);
        Assert.False(clamped);
        Assert.Equal(originalValues, values);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(1L)]
    [InlineData(long.MaxValue)]
    public void ScaledAnalyticNormal_ZeroDirectionPreservesBorrowedInputs(long scaleRaw)
    {
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        ulong[] originalValues = (ulong[])values.Clone();
        int[] originalSigns = (int[])signs.Clone();

        Vector3d normal = WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(
            new ConvexContactCandidate(values, signs, 0), Fixed64.FromRaw(scaleRaw));

        Assert.Equal(Vector3d.Zero, normal);
        Assert.Equal(originalValues, values);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(1, 1L, 3L)]
    [InlineData(-1, 3L, 1L)]
    [InlineData(-1, 2L, 0L)]
    public void AnalyticDepth_PureRadicalSquaredGapPreservesSignedRadiusOffset(
        int gapSign, long radiusRaw, long expectedRaw)
    {
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        // A=0,B=1,C=16,D=1 gives squared gap sqrt(16)=4, hence
        // magnitude 2 raw units before adding or subtracting the radius.
        values[8 * ConvexContactCandidate.Words] = 1; signs[8] = 1;
        values[9 * ConvexContactCandidate.Words] = 16;
        values[10 * ConvexContactCandidate.Words] = 1;
        ulong[] originalValues = (ulong[])values.Clone();
        int[] originalSigns = (int[])signs.Clone();

        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(
            new ConvexContactCandidate(values, signs, gapSign), Fixed64.FromRaw(radiusRaw),
            out Fixed64 depth, out bool clamped);

        Assert.Equal(expectedRaw, depth.m_rawValue);
        Assert.False(clamped);
        Assert.Equal(originalValues, values);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(1L, 0L, 1L)]
    [InlineData(5L, 1L, 5L)]
    public void ScaledAnalyticNormal_NegativeComponentSquareEnclosureKeepsExactRounding(
        long scaleRaw, long expectedX, long expectedY)
    {
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        // n=(sqrt(2)-1,4,0), norm^2=19-2sqrt(2). The sqrt enclosure
        // [1,2] gives X^2 minimum -1 but norm^2 minimum 15, so only
        // the numerator lower bound is uncertain. At scale 1, 0<X<1/2
        // and 1/2<Y<1. At scale 5, 1/2<X<3/2 and 9/2<Y<5;
        // the strict X>1/2 test follows from 281^2>2*198^2.
        values[0] = 1; signs[0] = -1;
        values[ConvexContactCandidate.Words] = 4; signs[1] = 1;
        values[3 * ConvexContactCandidate.Words] = 1; signs[3] = 1;
        values[6 * ConvexContactCandidate.Words] = 2;
        ulong[] originalValues = (ulong[])values.Clone();
        int[] originalSigns = (int[])signs.Clone();

        Vector3d normal = WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(
            new ConvexContactCandidate(values, signs, 0), Fixed64.FromRaw(scaleRaw));

        Assert.Equal(expectedX, normal.X.m_rawValue);
        Assert.Equal(expectedY, normal.Y.m_rawValue);
        Assert.Equal(0L, normal.Z.m_rawValue);
        Assert.Equal(originalValues, values);
        Assert.Equal(originalSigns, signs);
    }

    private static long RoundFifths(BigInteger numerator)
    {
        BigInteger floor = BigInteger.DivRem(numerator, 5, out BigInteger remainder);
        return (long)(floor + (remainder > 2 ? 1 : 0));
    }

    [Fact]
    public void Depth_ClassifiesClampingFromExactUnroundedValue()
    {
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw(Fixed64.MaxValue), Fixed64.MaxValue, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw(Fixed64.MaxValue), Fixed64.MaxValue);
        ulong[] values = new ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        int[] signs = new int[ConvexContactCandidate.Slots];
        CylinderPairAnalyticFeatures.Build(geometry, 0, values, signs, out int gapSign);
        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(new ConvexContactCandidate(values, signs, gapSign),
            Fixed64.Zero, out Fixed64 depth, out bool clamped);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.True(clamped);
    }

    private static CylinderPairGeometry Geometry(Vector3d center) => new(Vector3d.Zero, FixedQuaternion.Identity,
        Vector3d.Up, Signed192.Raw(Fixed64.Two), Fixed64.Two, center, FixedQuaternion.Identity,
        Vector3d.Right, Signed192.Raw(Fixed64.Two), Fixed64.Two);

    private static BigInteger[] Axis(WideAxis3 axis) => new[] { Integer(axis.X), Integer(axis.Y), Integer(axis.Z) };
    private static BigInteger Dot(BigInteger[] a, BigInteger[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    private static BigInteger[] Cross(BigInteger[] a, BigInteger[] b) => new[]
        { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
    private static BigInteger Integer(Signed192 value) =>
        ((BigInteger)(long)value.High << 128) | ((BigInteger)value.Middle << 64) | value.Low;
    private static BigInteger Integer(Signed320 value) =>
        ((BigInteger)(long)value.Word4 << 256) | ((BigInteger)value.Word3 << 192)
        | ((BigInteger)value.Word2 << 128) | ((BigInteger)value.Word1 << 64) | value.Word0;
    private static BigInteger Slot(ulong[] values, int[] signs, int slot)
    {
        BigInteger result = 0;
        for (int word = ConvexContactCandidate.Words - 1; word >= 0; word--)
            result = (result << 64) | values[slot * ConvexContactCandidate.Words + word];
        Assert.Equal(result.IsZero, signs[slot] == 0);
        return result * signs[slot];
    }
    private static void Write(BigInteger value, Span<ulong> words)
    {
        for (int index = 0; index < words.Length; index++)
        {
            words[index] = (ulong)(value & ulong.MaxValue);
            value >>= 64;
        }
        Assert.Equal(BigInteger.Zero, value);
    }
}
