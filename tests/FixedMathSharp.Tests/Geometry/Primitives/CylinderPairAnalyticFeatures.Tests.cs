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
