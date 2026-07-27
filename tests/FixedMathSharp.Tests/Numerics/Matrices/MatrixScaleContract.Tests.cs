using FixedMathSharp.Geometry;
using System;
using System.Collections.Generic;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public class MatrixScaleContractTests
{
    public static IEnumerable<object[]> SignedScaleCases()
    {
        yield return new object[] { 1, 1, 1 };
        yield return new object[] { 2, 3, 4 };
        yield return new object[] { -2, 3, 4 };
        yield return new object[] { 2, -3, 4 };
        yield return new object[] { 2, 3, -4 };
        yield return new object[] { -2, -3, 4 };
        yield return new object[] { -2, 3, -4 };
        yield return new object[] { 2, -3, -4 };
        yield return new object[] { -2, -3, -4 };
        yield return new object[] { 0, -3, 4 };
        yield return new object[] { -2, 0, 4 };
        yield return new object[] { -2, 3, 0 };
    }

    [Theory]
    [MemberData(nameof(SignedScaleCases))]
    public void ScaleExtraction_ReturnsRowMagnitudesAndCanonicalReflection(
        int x,
        int y,
        int z)
    {
        Vector3d authoredScale = new(x, y, z);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        Fixed4x4 matrix4x4 = Fixed4x4.CreateTransform(Vector3d.Zero, rotation, authoredScale);
        Fixed3x3 matrix3x3 = GetBasis(matrix4x4);
        Vector3d magnitudes = new(Math.Abs(x), Math.Abs(y), Math.Abs(z));
        bool reflected = x != 0 && y != 0 && z != 0 && Math.Sign(x) * Math.Sign(y) * Math.Sign(z) < 0;
        Vector3d lossyScale = new(reflected ? -magnitudes.X : magnitudes.X, magnitudes.Y, magnitudes.Z);

        Assert.Equal(magnitudes, Fixed3x3.ExtractScaleMagnitudes(matrix3x3));
        Assert.Equal(lossyScale, Fixed3x3.ExtractLossyScale(matrix3x3));
        Assert.Equal(magnitudes, Fixed4x4.ExtractScaleMagnitudes(matrix4x4));
        Assert.Equal(lossyScale, Fixed4x4.ExtractLossyScale(matrix4x4));
        Assert.Equal(lossyScale, matrix4x4.LossyScale);
    }

    [Fact]
    public void ScaleExtraction_UsesBasisRowsInsteadOfDiagonalEntries()
    {
        Fixed3x3 rotation3x3 = Fixed3x3.CreateRotationY(Fixed64.HalfPi);
        Fixed4x4 rotation4x4 = Fixed4x4.CreateRotation(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi));

        Assert.NotEqual(Vector3d.One, new Vector3d(rotation3x3.M11, rotation3x3.M22, rotation3x3.M33));
        Assert.NotEqual(Vector3d.One, new Vector3d(rotation4x4.M11, rotation4x4.M22, rotation4x4.M33));
        Assert.Equal(Vector3d.One, rotation3x3.ExtractScaleMagnitudes());
        Assert.Equal(Vector3d.One, rotation3x3.ExtractLossyScale());
        Assert.Equal(Vector3d.One, rotation4x4.ExtractScaleMagnitudes());
        Assert.Equal(Vector3d.One, rotation4x4.ExtractLossyScale());
    }

    [Fact]
    public void ScaleExtraction_CoversShearAndRepresentableRawLimits()
    {
        Fixed3x3 shear = new(
            Fixed64.One, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Vector3d shearMagnitudes = new(FixedMath.Sqrt(Fixed64.Two), Fixed64.One, Fixed64.One);
        Fixed64 negativeLimit = Fixed64.FromRaw(-long.MaxValue);
        Fixed3x3 limits = Fixed3x3.CreateScale(new Vector3d(Fixed64.MaxValue, negativeLimit, Fixed64.One));

        Assert.Equal(shearMagnitudes, shear.ExtractScaleMagnitudes());
        Assert.Equal(shearMagnitudes, shear.ExtractLossyScale());
        Assert.Equal(new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.One), limits.ExtractScaleMagnitudes());
        Assert.Equal(new Vector3d(-Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.One), limits.ExtractLossyScale());
        Assert.Equal(shearMagnitudes, To4x4(shear).ExtractScaleMagnitudes());
        Assert.Equal(new Vector3d(-Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.One), To4x4(limits).ExtractLossyScale());
    }

    [Fact]
    public void TripleProductSign_MatchesBigIntegerAcrossFullRawDomain()
    {
        long[] extremes =
        {
            long.MinValue,
            long.MinValue + 1,
            -FixedMath.ONE_L,
            -1,
            0,
            1,
            FixedMath.ONE_L,
            long.MaxValue - 1,
            long.MaxValue,
        };

        ulong state = 0xD1B5_4A32_D192_ED03UL;
        for (int sample = 0; sample < 512; sample++)
        {
            long[] raw = new long[9];
            for (int component = 0; component < raw.Length; component++)
            {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                raw[component] = sample < extremes.Length
                    ? extremes[(sample + component) % extremes.Length]
                    : unchecked((long)state);
            }

            int expected = (
                (BigInteger)raw[0] * ((BigInteger)raw[4] * raw[8] - (BigInteger)raw[5] * raw[7])
                - (BigInteger)raw[1] * ((BigInteger)raw[3] * raw[8] - (BigInteger)raw[5] * raw[6])
                + (BigInteger)raw[2] * ((BigInteger)raw[3] * raw[7] - (BigInteger)raw[4] * raw[6])).Sign;

            int actual = WideGeometry.GetTripleProductSign(
                Fixed64.FromRaw(raw[0]), Fixed64.FromRaw(raw[1]), Fixed64.FromRaw(raw[2]),
                Fixed64.FromRaw(raw[3]), Fixed64.FromRaw(raw[4]), Fixed64.FromRaw(raw[5]),
                Fixed64.FromRaw(raw[6]), Fixed64.FromRaw(raw[7]), Fixed64.FromRaw(raw[8]));

            Assert.Equal(expected, actual);
        }
    }

    public static IEnumerable<object[]> ValidTrsCases()
    {
        yield return new object[] { Fixed4x4.Identity };
        yield return new object[]
        {
            Fixed4x4.CreateTransform(
                new Vector3d(7, -8, 9),
                FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(30), new Fixed64(45), new Fixed64(60)),
                new Vector3d(2, 3, 4))
        };
        yield return new object[]
        {
            Fixed4x4.CreateTransform(
                new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.FromRaw(long.MaxValue - 1)),
                FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi),
                Vector3d.One)
        };
        yield return new object[]
        {
            Fixed4x4.CreateScale(new Vector3d(Fixed64.FromRaw(1), Fixed64.MaxValue, Fixed64.One))
        };
    }

    [Theory]
    [MemberData(nameof(ValidTrsCases))]
    public void Decompose_AcceptsAffineTrsAndReconstructsNormalizedBasis(Fixed4x4 matrix)
    {
        Assert.True(Fixed4x4.Decompose(matrix, out Vector3d translation, out FixedQuaternion rotation, out Vector3d scale));
        Assert.Equal(matrix.Translation, translation);
        AssertNormalizedBasisReconstructed(matrix, rotation, scale);
    }

    [Fact]
    public void CreateTransform_ArbitraryAxisQuaternion_ProducesStrictlyDecomposableTrs()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            new Vector3d(1, 2, 3),
            Fixed64.PiOver4);
        Fixed4x4 matrix = Fixed4x4.CreateTransform(
            new Vector3d(4, 5, 6),
            rotation,
            new Vector3d(2, 3, 4));

        Assert.NotEqual(Fixed64.One.m_rawValue, rotation.Magnitude.m_rawValue);
        Assert.True(Fixed4x4.Decompose(matrix, out _, out FixedQuaternion decomposedRotation, out Vector3d scale));
        AssertNormalizedBasisReconstructed(matrix, decomposedRotation, scale);
    }

    public static IEnumerable<object[]> NonUnitQuaternionMatrixCases()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            new Vector3d(1, 2, 3),
            Fixed64.PiOver4) * Fixed64.Two;
        yield return new object[]
        {
            "CreateRotation",
            rotation,
            Fixed4x4.CreateRotation(rotation),
        };
        yield return new object[]
        {
            "CreateTransform",
            rotation,
            Fixed4x4.CreateTransform(new Vector3d(4, 5, 6), rotation, new Vector3d(2, 3, 4)),
        };
        yield return new object[]
        {
            "TranslateRotateScale",
            rotation,
            Fixed4x4.TranslateRotateScale(new Vector3d(4, 5, 6), rotation, Vector3d.One),
        };
        yield return new object[]
        {
            "SetRotation",
            rotation,
            Fixed4x4.SetRotation(Fixed4x4.CreateScale(new Vector3d(2, 3, 4)), rotation),
        };
    }

    [Theory]
    [MemberData(nameof(NonUnitQuaternionMatrixCases))]
    public void QuaternionMatrixBoundaries_NormalizeNonUnitInput(
        string apiName,
        FixedQuaternion input,
        Fixed4x4 matrix)
    {
        Assert.False(input.IsNormalized());
        Assert.True(
            Fixed4x4.Decompose(matrix, out _, out FixedQuaternion rotation, out Vector3d scale),
            apiName);
        AssertNormalizedBasisReconstructed(matrix, rotation, scale);
    }

    public static IEnumerable<object[]> NearUnitQuaternionMatrixCases()
    {
        FixedQuaternion rotation = CreateTolerantNearUnitQuaternion();
        yield return new object[]
        {
            "CreateRotation",
            rotation,
            Fixed4x4.CreateRotation(rotation),
        };
        yield return new object[]
        {
            "CreateTransform",
            rotation,
            Fixed4x4.CreateTransform(new Vector3d(4, 5, 6), rotation, new Vector3d(2, 3, 4)),
        };
        yield return new object[]
        {
            "TranslateRotateScale",
            rotation,
            Fixed4x4.TranslateRotateScale(new Vector3d(4, 5, 6), rotation, Vector3d.One),
        };
        yield return new object[]
        {
            "SetRotation",
            rotation,
            Fixed4x4.SetRotation(Fixed4x4.CreateScale(new Vector3d(2, 3, 4)), rotation),
        };
    }

    [Fact]
    public void PublicQuaternionNormalization_RepairsNearUnitInputOutsideSquaredTolerance()
    {
        FixedQuaternion input = CreateTolerantNearUnitQuaternion();
        FixedQuaternion normalized = input.Normalized;
        Fixed3x3 directMatrix = normalized.ToMatrix3x3();

        Assert.False(input.IsNormalized());
        Assert.True(normalized.IsNormalized());
        Assert.NotEqual(input, normalized);
        AssertProperRotationMatrix(directMatrix);
    }

    public static IEnumerable<object[]> DirectQuaternionMatrixCases()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            new Vector3d(1, 2, 3),
            Fixed64.PiOver4);
        yield return new object[] { "near-unit tolerant", CreateTolerantNearUnitQuaternion() };
        yield return new object[] { "ordinary nonunit", rotation * new Fixed64(3) };
        yield return new object[]
        {
            "tiny raw",
            new FixedQuaternion(
                Fixed64.FromRaw(1),
                Fixed64.FromRaw(-1),
                Fixed64.FromRaw(2),
                Fixed64.FromRaw(-2)),
        };
        yield return new object[]
        {
            "mixed extrema",
            new FixedQuaternion(
                Fixed64.MaxValue,
                Fixed64.MinValue,
                Fixed64.MaxValue,
                Fixed64.MinValue),
        };
    }

    [Theory]
    [MemberData(nameof(DirectQuaternionMatrixCases))]
    public void ToMatrix3x3_NonzeroFullDomainInputsProduceProperRotation(
        string caseName,
        FixedQuaternion input)
    {
        Fixed3x3 matrix = input.ToMatrix3x3();

        Assert.NotEqual(FixedQuaternion.Zero, input);
        AssertProperRotationMatrix(matrix, caseName);
    }

    public static IEnumerable<object[]> RepresentableQuaternionScaleCases()
    {
        yield return new object[] { "half", Fixed64.Half };
        yield return new object[] { "two", Fixed64.Two };
        yield return new object[] { "sixteen", new Fixed64(16) };
        yield return new object[] { "one thousand twenty-four", new Fixed64(1024) };
    }

    [Theory]
    [MemberData(nameof(RepresentableQuaternionScaleCases))]
    public void ToMatrix3x3_RepresentableCommonScalesPreserveRotation(
        string caseName,
        Fixed64 factor)
    {
        FixedQuaternion input = FixedQuaternion.FromAxisAngle(
            new Vector3d(1, 2, 3),
            Fixed64.PiOver4);
        Fixed3x3 expected = input.ToMatrix3x3();
        Fixed3x3 actual = (input * factor).ToMatrix3x3();

        AssertMatrixWithin(expected, actual, caseName);
        AssertProperRotationMatrix(actual, caseName);
    }

    [Fact]
    public void ToMatrix3x3_QuaternionAndNegationProduceSameRotation()
    {
        FixedQuaternion input = FixedQuaternion.FromAxisAngle(
            new Vector3d(1, 2, 3),
            Fixed64.PiOver4) * new Fixed64(3);

        Assert.Equal(input.ToMatrix3x3(), (-input).ToMatrix3x3());
    }

    [Fact]
    public void ToMatrix3x3_TinyAndMixedExtremeRatiosMatchRepresentableInputs()
    {
        FixedQuaternion tiny = new(
            Fixed64.FromRaw(1),
            Fixed64.FromRaw(-1),
            Fixed64.FromRaw(2),
            Fixed64.FromRaw(-2));
        FixedQuaternion tinyRatio = new(
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.One,
            -Fixed64.One);
        FixedQuaternion extreme = new(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.MaxValue,
            Fixed64.MinValue);
        FixedQuaternion extremeRatio = new(
            Fixed64.One,
            -Fixed64.One,
            Fixed64.One,
            -Fixed64.One);

        AssertMatrixWithin(tinyRatio.ToMatrix3x3(), tiny.ToMatrix3x3(), "tiny raw ratio");
        AssertMatrixWithin(extremeRatio.ToMatrix3x3(), extreme.ToMatrix3x3(), "mixed extreme ratio");
    }

    [Fact]
    public void ToMatrix3x3_ZeroReturnsIdentity()
    {
        Assert.Equal(Fixed3x3.Identity, FixedQuaternion.Zero.ToMatrix3x3());
    }

    [Fact]
    public void ToMatrix3x3_LowerOrdinaryGateCounterexampleProducesProperDecomposableRotation()
    {
        FixedQuaternion input = new(
            Fixed64.FromRaw(268435457),
            Fixed64.FromRaw(11865206),
            Fixed64.FromRaw(44287645),
            Fixed64.FromRaw(-90409710));
        Fixed3x3 oracle = ToScaleRelativeMatrixOracle(input);
        Fixed3x3 actual = input.ToMatrix3x3();

        Assert.Equal(
            FixedMath.ScaleSafeMagnitudeThreshold.m_rawValue + 1,
            input.X.m_rawValue);
        AssertProperRotationMatrix(oracle, "scale-relative oracle");
        AssertMatrixWithin(oracle, actual, "lower ordinary gate counterexample");
        AssertProperRotationMatrix(actual, "lower ordinary gate counterexample");
        Assert.True(Fixed4x4.Decompose(To4x4(actual), out _, out _, out _));
    }

    [Fact]
    public void ToMatrix3x3_OrdinaryFastPathMatchesScaleRelativeOracleAcrossComponentGrid()
    {
        Fixed64[] components =
        {
            -Fixed64.Two,
            -Fixed64.One,
            -Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Two,
        };

        foreach (Fixed64 x in components)
            foreach (Fixed64 y in components)
                foreach (Fixed64 z in components)
                    foreach (Fixed64 w in components)
                    {
                        FixedQuaternion input = new(x, y, z, w);
                        if (input == FixedQuaternion.Zero)
                            continue;

                        Fixed3x3 actual = input.ToMatrix3x3();
                        AssertMatrixWithin(ToScaleRelativeMatrixOracle(input), actual, input.ToString());
                        AssertProperRotationMatrix(actual, input.ToString());
                    }
    }

    [Fact]
    public void ToMatrix3x3_OrdinaryFastPathMatchesScaleRelativeOracleAcrossFullBandSamples()
    {
        ulong state = 0xE703_7ED1_A0B4_28DBUL;
        for (int sample = 0; sample < 2048; sample++)
        {
            FixedQuaternion input = NextOrdinaryBandQuaternion(ref state, sample);
            Fixed64 componentScale = FixedMath.Max(
                FixedMath.Max(input.X.Abs(), input.Y.Abs()),
                FixedMath.Max(input.Z.Abs(), input.W.Abs()));

            Fixed3x3 actual = input.ToMatrix3x3();
            Assert.InRange(
                componentScale.m_rawValue,
                Fixed64.Half.m_rawValue,
                Fixed64.Two.m_rawValue);
            AssertMatrixWithin(ToScaleRelativeMatrixOracle(input), actual, $"raw sample {sample}");
            AssertProperRotationMatrix(actual, $"raw sample {sample}");
        }
    }

    [Fact]
    public void ToMatrix3x3_OrdinaryFastPathEdgesPreserveScaleEquivalentRotation()
    {
        Fixed64 belowLower = Fixed64.FromRaw(Fixed64.Half.m_rawValue - 1);
        Fixed64 aboveLower = Fixed64.FromRaw(Fixed64.Half.m_rawValue + 1);
        Fixed64 aboveUpper = Fixed64.FromRaw(Fixed64.Two.m_rawValue + 1);
        FixedQuaternion[] inputs =
        {
            new(belowLower, Fixed64.Quarter, -Fixed64.Eighth, Fixed64.FromRaw(715827883)),
            new(Fixed64.Half, Fixed64.Quarter, -Fixed64.Eighth, Fixed64.FromRaw(715827883)),
            new(aboveLower, Fixed64.Quarter, -Fixed64.Eighth, Fixed64.FromRaw(715827883)),
            new(Fixed64.Two, Fixed64.One, -Fixed64.Half, Fixed64.FromRaw(2863311531)),
            new(aboveUpper, Fixed64.One, -Fixed64.Half, Fixed64.FromRaw(2863311531)),
        };

        foreach (FixedQuaternion input in inputs)
        {
            Fixed3x3 actual = input.ToMatrix3x3();
            AssertMatrixWithin(ToScaleRelativeMatrixOracle(input), actual, input.ToString());
            AssertProperRotationMatrix(actual, input.ToString());
        }
    }

    [Theory]
    [MemberData(nameof(NearUnitQuaternionMatrixCases))]
    public void QuaternionMatrixBoundaries_StrictlyNormalizeNearUnitInputOutsideSquaredTolerance(
        string apiName,
        FixedQuaternion input,
        Fixed4x4 matrix)
    {
        Assert.False(input.IsNormalized());
        Assert.True(input.Normalized.IsNormalized());
        Assert.NotEqual(input, input.Normalized);
        Assert.Equal(
            -Fixed64.Epsilon.m_rawValue,
            input.Magnitude.m_rawValue - Fixed64.One.m_rawValue);
        Assert.True(
            Fixed4x4.Decompose(matrix, out _, out FixedQuaternion rotation, out Vector3d scale),
            apiName);
        AssertNormalizedBasisReconstructed(matrix, rotation, scale);
    }

    public static IEnumerable<object[]> StrictQuaternionNormalizationSafetyCases()
    {
        yield return new object[] { "zero", FixedQuaternion.Zero };
        yield return new object[]
        {
            "tiny",
            new FixedQuaternion(
                Fixed64.FromRaw(1),
                Fixed64.FromRaw(-1),
                Fixed64.FromRaw(2),
                Fixed64.FromRaw(-2)),
        };
        yield return new object[]
        {
            "extreme",
            new FixedQuaternion(
                Fixed64.MaxValue,
                Fixed64.MinValue,
                Fixed64.MaxValue,
                Fixed64.MinValue),
        };
    }

    [Theory]
    [MemberData(nameof(StrictQuaternionNormalizationSafetyCases))]
    public void QuaternionMatrixStrictNormalization_HandlesFullDomainSafetyCases(
        string caseName,
        FixedQuaternion input)
    {
        Fixed4x4 matrix = Fixed4x4.CreateRotation(input);

        Assert.True(
            Fixed4x4.Decompose(matrix, out _, out FixedQuaternion rotation, out Vector3d scale),
            caseName);
        AssertNormalizedBasisReconstructed(matrix, rotation, scale);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Decompose_CanonicalizesSingleReflectionToNegativeX(int negativeAxis)
    {
        Vector3d authoredScale = new(2, 3, 4);
        authoredScale[negativeAxis] = -authoredScale[negativeAxis];
        Fixed4x4 matrix = Fixed4x4.CreateTransform(
            new Vector3d(1, 2, 3),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            authoredScale);

        Assert.True(Fixed4x4.Decompose(matrix, out _, out FixedQuaternion rotation, out Vector3d scale));
        Assert.Equal(matrix.ExtractLossyScale(), scale);
        Assert.True(scale.X < Fixed64.Zero);
        Assert.True(scale.Y > Fixed64.Zero);
        Assert.True(scale.Z > Fixed64.Zero);
        AssertNormalizedBasisReconstructed(matrix, rotation, scale);
    }

    public static IEnumerable<object[]> InvalidMatrices()
    {
        for (int component = 0; component < 3; component++)
        {
            Fixed4x4 perspective = Fixed4x4.Identity;
            perspective[12 + component] = Fixed64.One;
            yield return new object[] { perspective };
        }

        Fixed4x4 homogeneous = Fixed4x4.Identity;
        homogeneous.M44 = Fixed64.Two;
        yield return new object[] { homogeneous };

        int[] shearIndices = { 4, 8, 1, 9, 2, 6 };
        foreach (int index in shearIndices)
        {
            Fixed4x4 shear = Fixed4x4.Identity;
            shear[index] = Fixed64.One;
            yield return new object[] { shear };
        }

        for (int row = 0; row < 3; row++)
        {
            Fixed4x4 zeroAxis = Fixed4x4.Identity;
            zeroAxis[row] = Fixed64.Zero;
            zeroAxis[row + 4] = Fixed64.Zero;
            zeroAxis[row + 8] = Fixed64.Zero;
            yield return new object[] { zeroAxis };
        }

        yield return new object[]
        {
            new Fixed4x4(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One)
        };
        for (int row = 0; row < 3; row++)
        {
            Fixed4x4 unrepresentableMagnitude = Fixed4x4.Identity;
            unrepresentableMagnitude[row] = Fixed64.MaxValue;
            unrepresentableMagnitude[row + 4] = Fixed64.MaxValue;
            yield return new object[] { unrepresentableMagnitude };
        }

        yield return new object[]
        {
            new Fixed4x4(
                Fixed64.FromRaw(938_673_717), Fixed64.FromRaw(4_105_859_133), Fixed64.FromRaw(841_164_157), Fixed64.Zero,
                Fixed64.FromRaw(19_783_822), Fixed64.FromRaw(857_652_179), Fixed64.FromRaw(-4_208_418_345), Fixed64.Zero,
                Fixed64.FromRaw(-4_191_091_066), Fixed64.FromRaw(923_632_751), Fixed64.FromRaw(168_528_758), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One)
        };

        Fixed4x4 outsideTolerance = Fixed4x4.Identity;
        outsideTolerance.M21 = Fixed64.FromRaw(Fixed64.Epsilon.m_rawValue + 1);
        yield return new object[] { outsideTolerance };
    }

    [Theory]
    [MemberData(nameof(InvalidMatrices))]
    public void Decompose_RejectsNonTrsAndWritesNeutralOutputs(Fixed4x4 matrix)
    {
        Assert.False(Fixed4x4.Decompose(matrix, out Vector3d translation, out FixedQuaternion rotation, out Vector3d scale));
        Assert.Equal(Vector3d.Zero, translation);
        Assert.Equal(FixedQuaternion.Identity, rotation);
        Assert.Equal(Vector3d.One, scale);
    }

    [Fact]
    public void Decompose_RejectsScaleAmplifiedReconstructionError()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            new Vector3d(Fixed64.One, Fixed64.Two, (Fixed64)3),
            Fixed64.PiOver6);
        Fixed4x4 matrix = Fixed4x4.CreateTransform(
            Vector3d.Zero,
            rotation,
            new Vector3d((Fixed64)1000, (Fixed64)1000, (Fixed64)1000));

        Assert.False(Fixed4x4.Decompose(
            matrix,
            out Vector3d translation,
            out FixedQuaternion decomposedRotation,
            out Vector3d scale));
        Assert.Equal(Vector3d.Zero, translation);
        Assert.Equal(FixedQuaternion.Identity, decomposedRotation);
        Assert.Equal(Vector3d.One, scale);
        Assert.False(FixedTransform.TryCreateFromLocalMatrix(matrix, out FixedTransform? transform));
        Assert.Null(transform);
    }

    private static void AssertNormalizedBasisReconstructed(
        Fixed4x4 matrix,
        FixedQuaternion rotation,
        Vector3d scale)
    {
        Fixed3x3 reconstructed = rotation.ToMatrix3x3();
        Vector3d expectedX = new Vector3d(matrix.M11, matrix.M12, matrix.M13).Normalized;
        Vector3d expectedY = new Vector3d(matrix.M21, matrix.M22, matrix.M23).Normalized;
        Vector3d expectedZ = new Vector3d(matrix.M31, matrix.M32, matrix.M33).Normalized;
        if (scale.X < Fixed64.Zero)
            expectedX = -expectedX;

        AssertVectorWithin(expectedX, new Vector3d(reconstructed.M11, reconstructed.M12, reconstructed.M13));
        AssertVectorWithin(expectedY, new Vector3d(reconstructed.M21, reconstructed.M22, reconstructed.M23));
        AssertVectorWithin(expectedZ, new Vector3d(reconstructed.M31, reconstructed.M32, reconstructed.M33));
    }

    private static void AssertVectorWithin(Vector3d expected, Vector3d actual)
    {
        Assert.True((expected.X - actual.X).Abs() <= Fixed64.Epsilon);
        Assert.True((expected.Y - actual.Y).Abs() <= Fixed64.Epsilon);
        Assert.True((expected.Z - actual.Z).Abs() <= Fixed64.Epsilon);
    }

    private static void AssertMatrixWithin(Fixed3x3 expected, Fixed3x3 actual, string caseName)
    {
        int[] indices = { 0, 1, 2, 4, 5, 6, 8, 9, 10 };
        foreach (int index in indices)
        {
            Assert.True(
                (expected[index] - actual[index]).Abs() <= Fixed64.Epsilon,
                $"{caseName}: component {index} expected {expected[index]} but was {actual[index]}.");
        }
    }

    private static void AssertProperRotationMatrix(Fixed3x3 matrix, string caseName = "rotation")
    {
        Vector3d rowX = new(matrix.M11, matrix.M12, matrix.M13);
        Vector3d rowY = new(matrix.M21, matrix.M22, matrix.M23);
        Vector3d rowZ = new(matrix.M31, matrix.M32, matrix.M33);

        Assert.True((rowX.Magnitude - Fixed64.One).Abs() <= Fixed64.Epsilon, $"{caseName}: X row is not unit length.");
        Assert.True((rowY.Magnitude - Fixed64.One).Abs() <= Fixed64.Epsilon, $"{caseName}: Y row is not unit length.");
        Assert.True((rowZ.Magnitude - Fixed64.One).Abs() <= Fixed64.Epsilon, $"{caseName}: Z row is not unit length.");
        Assert.True(Vector3d.Dot(rowX, rowY).Abs() <= Fixed64.Epsilon, $"{caseName}: X/Y rows are not orthogonal.");
        Assert.True(Vector3d.Dot(rowX, rowZ).Abs() <= Fixed64.Epsilon, $"{caseName}: X/Z rows are not orthogonal.");
        Assert.True(Vector3d.Dot(rowY, rowZ).Abs() <= Fixed64.Epsilon, $"{caseName}: Y/Z rows are not orthogonal.");
        Assert.True(Vector3d.Dot(Vector3d.Cross(rowX, rowY), rowZ) > Fixed64.Zero, $"{caseName}: basis is not right-handed.");
    }

    private static Fixed3x3 ToScaleRelativeMatrixOracle(FixedQuaternion input)
    {
        Fixed64 componentScale = FixedMath.Max(
            FixedMath.Max(input.X.Abs(), input.Y.Abs()),
            FixedMath.Max(input.Z.Abs(), input.W.Abs()));
        if (componentScale == Fixed64.Zero)
            return Fixed3x3.Identity;

        Fixed64 x = input.X / componentScale;
        Fixed64 y = input.Y / componentScale;
        Fixed64 z = input.Z / componentScale;
        Fixed64 w = input.W / componentScale;
        Fixed64 x2 = x * x;
        Fixed64 y2 = y * y;
        Fixed64 z2 = z * z;
        Fixed64 factor = Fixed64.Two / (x2 + y2 + z2 + (w * w));

        return new Fixed3x3(
            Fixed64.One - factor * (y2 + z2), factor * ((x * y) + (z * w)), factor * ((x * z) - (y * w)),
            factor * ((x * y) - (z * w)), Fixed64.One - factor * (x2 + z2), factor * ((y * z) + (x * w)),
            factor * ((x * z) + (y * w)), factor * ((y * z) - (x * w)), Fixed64.One - factor * (x2 + y2));
    }

    private static FixedQuaternion NextOrdinaryBandQuaternion(ref ulong state, int sample)
    {
        long minRaw = Fixed64.Half.m_rawValue;
        long maxRaw = Fixed64.Two.m_rawValue;
        long componentScaleRaw = sample switch
        {
            0 => minRaw,
            1 => minRaw + 1,
            2 => maxRaw - 1,
            3 => maxRaw,
            _ => minRaw + (long)(NextRandom(ref state) % (ulong)(maxRaw - minRaw + 1)),
        };
        long componentSpan = (componentScaleRaw * 2) + 1;
        Fixed64 x = Fixed64.FromRaw((long)(NextRandom(ref state) % (ulong)componentSpan) - componentScaleRaw);
        Fixed64 y = Fixed64.FromRaw((long)(NextRandom(ref state) % (ulong)componentSpan) - componentScaleRaw);
        Fixed64 z = Fixed64.FromRaw((long)(NextRandom(ref state) % (ulong)componentSpan) - componentScaleRaw);
        Fixed64 w = Fixed64.FromRaw((long)(NextRandom(ref state) % (ulong)componentSpan) - componentScaleRaw);
        Fixed64 signedScale = (NextRandom(ref state) & 1UL) == 0
            ? Fixed64.FromRaw(componentScaleRaw)
            : Fixed64.FromRaw(-componentScaleRaw);

        return (sample & 3) switch
        {
            0 => new FixedQuaternion(signedScale, y, z, w),
            1 => new FixedQuaternion(x, signedScale, z, w),
            2 => new FixedQuaternion(x, y, signedScale, w),
            _ => new FixedQuaternion(x, y, z, signedScale),
        };
    }

    private static ulong NextRandom(ref ulong state)
    {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    }

    private static Fixed3x3 GetBasis(Fixed4x4 matrix) => new(
        matrix.M11, matrix.M12, matrix.M13,
        matrix.M21, matrix.M22, matrix.M23,
        matrix.M31, matrix.M32, matrix.M33);

    private static FixedQuaternion CreateTolerantNearUnitQuaternion()
    {
        Fixed64 nearUnitScale = Fixed64.FromRaw(
            Fixed64.One.m_rawValue - Fixed64.Epsilon.m_rawValue);
        return FixedQuaternion.FromAxisAngle(
            new Vector3d(1, 1, 1),
            Fixed64.HalfPi) * nearUnitScale;
    }

    private static Fixed4x4 To4x4(Fixed3x3 matrix) => new(
        matrix.M11, matrix.M12, matrix.M13, Fixed64.Zero,
        matrix.M21, matrix.M22, matrix.M23, Fixed64.Zero,
        matrix.M31, matrix.M32, matrix.M33, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
}
