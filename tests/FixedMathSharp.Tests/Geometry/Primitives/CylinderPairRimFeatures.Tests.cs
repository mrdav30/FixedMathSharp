using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairRimFeaturesTests
{
    [Theory]
    [InlineData(1UL, 0L, false)]
    [InlineData(3UL, 2L, false)]
    [InlineData(5UL, 2L, false)]
    [InlineData(18446744073709551613UL, long.MaxValue - 1, false)]
    [InlineData(18446744073709551614UL, long.MaxValue, false)]
    [InlineData(18446744073709551615UL, long.MaxValue, true)]
    public void DepthConversion_PreservesHalfRawTiesAndConceptualMaximum(
        ulong twiceRaw, long expectedRaw, bool expectedClamp)
    {
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up, Signed192.Raw(Fixed64.MaxValue), Fixed64.MaxValue,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw(Fixed64.MaxValue), Fixed64.MaxValue);
        BigInteger scale = ((BigInteger)(long)geometry.RawScale.High << 128)
            | ((BigInteger)geometry.RawScale.Middle << 64) | geometry.RawScale.Low;
        // S=(RawScale*twiceRaw/2)^2. This linear polynomial identifies the
        // exact value without first rounding either the depth or its square.
        BigInteger[] terms = { -BigInteger.Pow(scale * twiceRaw, 2),
            BigInteger.One << (geometry.ValueShift + 2) };
        ulong[] values = new ulong[6];
        sbyte[] signs = { -1, 1 };
        for (int index = 0; index < terms.Length; index++)
        {
            BigInteger magnitude = BigInteger.Abs(terms[index]);
            for (int word = 0; word < 3; word++)
            {
                values[index * 3 + word] = (ulong)(magnitude & ulong.MaxValue);
                magnitude >>= 64;
            }
            Assert.Equal(BigInteger.Zero, magnitude);
        }
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(values, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(values, signs, 0, cell,
            out FiniteAxisValueRoot root));
        ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift, ref root, out Fixed64 depth, out bool clamped);
        Assert.Equal(expectedRaw, depth.m_rawValue);
        Assert.Equal(expectedClamp, clamped);
    }

    [Theory]
    [InlineData(11, 8, -1)]
    [InlineData(5, 4, 1)]
    public void SimpleRim_AdmitsTheIrrationalMinimumOnEitherSideOfSeparation(int z, int divisor, int expectedSign)
    {
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up, Signed192.Raw(Fixed64.Two), Fixed64.One,
            new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, (Fixed64)z / divisor),
            FixedQuaternion.Identity, Vector3d.Right, Signed192.Raw(Fixed64.Two), Fixed64.One);
        ulong[] values = new ulong[9 * CylinderPairRimFeatures.CoefficientWords];
        sbyte[] signs = new sbyte[9];
        CylinderPairRimFeatures.BuildValues(geometry, 1, 1, values, signs);
        ulong[] repeated = new ulong[5 * CylinderPairRimFeatures.CoefficientWords];
        sbyte[] repeatedSigns = new sbyte[5];
        WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(values, signs, repeated, repeatedSigns);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(values, signs)];
        ulong[] derivatives = new ulong[CylinderPairRimFeatures.AdmissionCoefficientCount * CylinderPairRimFeatures.CoefficientWords];
        sbyte[] derivativeSigns = new sbyte[CylinderPairRimFeatures.AdmissionCoefficientCount];
        int readyMask = 0;
        bool admitted = false;
        for (int ordinal = 0; WideFiniteAxisIntersection.TryGetFiniteValueRoot(values, signs,
            ordinal, cell, out FiniteAxisValueRoot root); ordinal++)
        {
            if (IsRepeatedRoot(root, repeated, repeatedSigns))
                continue;
            int slope = CylinderPairRimFeatures.GetSimpleValueSlopeSign(root);
            if (!CylinderPairRimFeatures.TryAdmitSimple(geometry, 1, 1, ref root, slope,
                    derivatives, derivativeSigns, ref readyMask, out int gapSign))
                continue;
            Assert.Equal(expectedSign, gapSign);
            admitted = true;
            Vector3d normal = CylinderPairRimFeatures.GetSimpleNormal(geometry, 1, 1, ref root, slope, gapSign);
            Assert.Equal(normal.X, normal.Y);
            Assert.True(normal.X > Fixed64.Zero);
            Assert.True(normal.Z > Fixed64.Zero);
            if (gapSign > 0)
            {
                ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift, ref root, out Fixed64 depth, out bool clamped);
                Assert.InRange(depth.m_rawValue, 1L, Fixed64.FromFraction(1, 20).m_rawValue);
                Assert.False(clamped);
            }
            break;
        }
        Assert.True(admitted);
    }

    [Theory]
    [InlineData(11, 8)]
    [InlineData(5, 4)]
    public void SimpleAdmission_ReusesQueriesAcrossDistinctRootsAndResetsForCapSigns(int z, int divisor)
    {
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up, Signed192.Raw(Fixed64.Two), Fixed64.One,
            new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, (Fixed64)z / divisor),
            FixedQuaternion.Identity, Vector3d.Right, Signed192.Raw(Fixed64.Two), Fixed64.One);
        const int words = CylinderPairRimFeatures.CoefficientWords;
        ulong[] values = new ulong[9 * words];
        sbyte[] signs = new sbyte[9];
        ulong[] repeated = new ulong[5 * words];
        sbyte[] repeatedSigns = new sbyte[5];
        ulong[] derivatives = new ulong[CylinderPairRimFeatures.AdmissionCoefficientCount * words];
        sbyte[] derivativeSigns = new sbyte[CylinderPairRimFeatures.AdmissionCoefficientCount];
        ulong[] freshDerivatives = new ulong[derivatives.Length];
        sbyte[] freshDerivativeSigns = new sbyte[derivativeSigns.Length];
        Array.Fill(derivatives, ulong.MaxValue);
        Array.Fill(derivativeSigns, (sbyte)-1);
        int admittedCount = 0;
        for (int firstSign = -1; firstSign <= 1; firstSign += 2)
        {
            for (int secondSign = -1; secondSign <= 1; secondSign += 2)
            {
                CylinderPairRimFeatures.BuildValues(geometry, firstSign, secondSign, values, signs);
                WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(values, signs, repeated, repeatedSigns);
                ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(values, signs)];
                ulong[] freshCell = new ulong[cell.Length];
                // The same dirty storage crosses cap pairs; only the ready
                // mask is reset. A stale polynomial must not survive that reset.
                int readyMask = 0;
                int simpleCount = 0;
                for (int ordinal = 0; WideFiniteAxisIntersection.TryGetFiniteValueRoot(values, signs,
                    ordinal, cell, out FiniteAxisValueRoot root); ordinal++)
                {
                    if (IsRepeatedRoot(root, repeated, repeatedSigns))
                        continue;
                    simpleCount++;
                    cell.CopyTo(freshCell, 0);
                    var fresh = new FiniteAxisValueRoot
                    {
                        Coefficients = root.Coefficients, Signs = root.Signs, LowerNumerator = freshCell,
                        DenominatorShift = root.DenominatorShift, Ordinal = root.Ordinal, IsRational = root.IsRational
                    };
                    int slope = CylinderPairRimFeatures.GetSimpleValueSlopeSign(root);
                    int freshMask = 0;
                    bool expected = CylinderPairRimFeatures.TryAdmitSimple(geometry, firstSign, secondSign,
                        ref fresh, slope, freshDerivatives, freshDerivativeSigns, ref freshMask, out int expectedGap);
                    int previousMask = readyMask;
                    ulong[] previousDerivatives = (ulong[])derivatives.Clone();
                    sbyte[] previousSigns = (sbyte[])derivativeSigns.Clone();
                    bool actual = CylinderPairRimFeatures.TryAdmitSimple(geometry, firstSign, secondSign,
                        ref root, slope, derivatives, derivativeSigns, ref readyMask, out int actualGap);
                    Assert.Equal(expected, actual);
                    Assert.Equal(expectedGap, actualGap);
                    Assert.Equal(fresh.IsRational, root.IsRational);
                    Assert.Equal(fresh.DenominatorShift, root.DenominatorShift);
                    Assert.Equal(freshCell, cell);
                    Assert.Equal(previousMask | freshMask, readyMask);
                    for (int query = 0; query < 4; query++)
                    {
                        if ((previousMask & (1 << query)) == 0)
                            continue;
                        Assert.True(previousDerivatives.AsSpan(query * 9 * words, 9 * words)
                            .SequenceEqual(derivatives.AsSpan(query * 9 * words, 9 * words)));
                        Assert.True(previousSigns.AsSpan(query * 9, 9)
                            .SequenceEqual(derivativeSigns.AsSpan(query * 9, 9)));
                    }
                    if (actual)
                        admittedCount++;
                }
                Assert.True(simpleCount >= 2, "Each cap pair must exercise distinct simple roots.");
            }
        }
        Assert.True(admittedCount > 0);
    }

    [Fact]
    public void SimpleRim_AdmitsAndRoundsAnExactStationaryContact()
    {
        // n=(3,3,4)/sqrt(34), radial points (3,0,4) and (0,3,4).
        // With c=(-9/4,-9/4,-7), the residual is (3,3,4)/4.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up, Signed192.Raw(Fixed64.Two), (Fixed64)5,
            new Vector3d((Fixed64)13 / 4, (Fixed64)13 / 4, (Fixed64)7),
            FixedQuaternion.Identity, Vector3d.Right, Signed192.Raw(Fixed64.Two), (Fixed64)5);
        const int words = CylinderPairRimFeatures.CoefficientWords;
        ulong[] values = new ulong[9 * words];
        sbyte[] signs = new sbyte[9];
        CylinderPairRimFeatures.BuildValues(geometry, 1, 1, values, signs);
        ulong[] repeated = new ulong[5 * words];
        sbyte[] repeatedSigns = new sbyte[5];
        WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(values, signs, repeated, repeatedSigns);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(values, signs)];
        ulong[] exactValue = new ulong[2] { 0, 34 };
        ulong[] derivatives = new ulong[CylinderPairRimFeatures.AdmissionCoefficientCount * words];
        sbyte[] derivativeSigns = new sbyte[CylinderPairRimFeatures.AdmissionCoefficientCount];
        int readyMask = 0;
        bool found = false;
        for (int ordinal = 0; WideFiniteAxisIntersection.TryGetFiniteValueRoot(values, signs,
            ordinal, cell, out FiniteAxisValueRoot root); ordinal++)
        {
            if (WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root,
                exactValue, geometry.ValueShift + 4) != 0)
                continue;
            found = true;
            Assert.False(IsRepeatedRoot(root, repeated, repeatedSigns));
            int slope = CylinderPairRimFeatures.GetSimpleValueSlopeSign(root);
            Assert.NotEqual(0, slope);
            Assert.True(CylinderPairRimFeatures.TryAdmitSimple(geometry, 1, 1, ref root, slope,
                derivatives, derivativeSigns, ref readyMask, out int gapSign));
            Assert.Equal(1, gapSign);
            Vector3d normal = CylinderPairRimFeatures.GetSimpleNormal(geometry, 1, 1, ref root, slope, gapSign);
            AssertNormalComponent(normal.X.m_rawValue, 9, 34);
            AssertNormalComponent(normal.Y.m_rawValue, 9, 34);
            AssertNormalComponent(normal.Z.m_rawValue, 16, 34);
            ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift, ref root, out Fixed64 depth, out bool clamped);
            AssertNormalComponent(depth.m_rawValue, 34, 16);
            Assert.False(clamped);
        }
        Assert.True(found);
    }

    [Fact]
    public void SimpleRim_RejectsAZeroFirstRadialSupportBoundary()
    {
        // a=Z, b=X, p=(3,4,0), q=(0,0,5), c=(-3,-4,-4).
        // The residual v=(0,0,1) is stationary for both rims, but p.v=0
        // and q.v=5. The unit-tangent Hessian is [[1,3/5],[3/5,4/5]],
        // whose determinant 11/25 is nonzero: this is a simple stationary
        // value, not a repeated-root surrogate for a missing boundary case.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Forward, Signed192.Raw(Fixed64.Two), (Fixed64)5,
            new Vector3d(4, 4, 5), FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw(Fixed64.Two), (Fixed64)5);
        const int words = CylinderPairRimFeatures.CoefficientWords;
        ulong[] values = new ulong[9 * words];
        sbyte[] signs = new sbyte[9];
        CylinderPairRimFeatures.BuildValues(geometry, 1, 1, values, signs);
        ulong[] repeated = new ulong[5 * words];
        sbyte[] repeatedSigns = new sbyte[5];
        WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(values, signs, repeated, repeatedSigns);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(values, signs)];
        ulong[] exactValue = { 0, 1 }; // S=1 in world units, or 2^64 in raw units.
        ulong[] derivatives = new ulong[CylinderPairRimFeatures.AdmissionCoefficientCount * words];
        sbyte[] derivativeSigns = new sbyte[CylinderPairRimFeatures.AdmissionCoefficientCount];
        int readyMask = 0;
        bool found = false;
        for (int ordinal = 0; WideFiniteAxisIntersection.TryGetFiniteValueRoot(values, signs,
            ordinal, cell, out FiniteAxisValueRoot root); ordinal++)
        {
            if (WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root,
                    exactValue, geometry.ValueShift) != 0)
                continue;
            found = true;
            Assert.False(IsRepeatedRoot(root, repeated, repeatedSigns));
            int slope = CylinderPairRimFeatures.GetSimpleValueSlopeSign(root);
            Assert.NotEqual(0, slope);
            Assert.False(CylinderPairRimFeatures.TryAdmitSimple(geometry, 1, 1,
                ref root, slope, derivatives, derivativeSigns, ref readyMask, out int gapSign));
            Assert.Equal(0, gapSign);
            Assert.Equal(1, readyMask); // A zero first radial needs no other admission polynomial.
        }
        Assert.True(found);
    }

    [Fact]
    public void RepeatedFactor_RecognizesAndRoundsTheKnownRimMinimum()
    {
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw((Fixed64)10), Fixed64.One, new Vector3d(0, 1, 8),
            new FixedQuaternion(Fixed64.Zero, s, Fixed64.Zero, 2 * s), Vector3d.Left,
            Signed192.Raw((Fixed64)10), Fixed64.One);
        ulong[] values = new ulong[9 * CylinderPairRimFeatures.CoefficientWords];
        sbyte[] signs = new sbyte[9];
        CylinderPairRimFeatures.BuildValues(geometry, 1, 1, values, signs);
        ulong[] repeated = new ulong[5 * CylinderPairRimFeatures.CoefficientWords];
        sbyte[] repeatedSigns = new sbyte[5];
        Assert.True(WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(values, signs, repeated, repeatedSigns) > 0);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(repeated, repeatedSigns)];
        // RawScale=5: S=25*351/400*2^64=351*2^60.
        ulong[] exactValue = new ulong[2] { 0, 351 };
        bool found = false;
        for (int ordinal = 0; WideFiniteAxisIntersection.TryGetFiniteValueRoot(repeated, repeatedSigns,
            ordinal, cell, out FiniteAxisValueRoot root); ordinal++)
        {
            if (WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root,
                exactValue, geometry.ValueShift + 4) != 0)
                continue;
            found = true;
            ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift, ref root, out Fixed64 depth, out bool clamped);
            Assert.Equal(4023309325, depth.m_rawValue);
            Assert.False(clamped);
        }
        Assert.True(found);
    }

    private static bool IsRepeatedRoot(FiniteAxisValueRoot source, ulong[] factor, sbyte[] signs)
    {
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(factor, signs)];
        ulong[] upper = source.LowerNumerator.ToArray();
        WideArithmetic.AddWord(upper, 0, 1);
        for (int ordinal = 0; WideFiniteAxisIntersection.TryGetFiniteValueRoot(factor, signs,
            ordinal, cell, out FiniteAxisValueRoot repeated); ordinal++)
        {
            int lowerComparison = WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(
                repeated, source.LowerNumerator, source.DenominatorShift);
            if (source.IsRational ? lowerComparison == 0 : lowerComparison > 0
                && WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(repeated, upper, source.DenominatorShift) < 0)
                return true;
        }
        return false;
    }

    private static void AssertNormalComponent(long raw, int numerator, int denominator)
    {
        BigInteger square = (BigInteger)numerator << 66;
        BigInteger twice = (BigInteger)raw * 2;
        Assert.True((BigInteger)denominator * (twice - 1) * (twice - 1) < square);
        Assert.True(square < (BigInteger)denominator * (twice + 1) * (twice + 1));
    }
}
