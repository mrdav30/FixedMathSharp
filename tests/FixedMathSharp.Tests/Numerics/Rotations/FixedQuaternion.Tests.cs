using System;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;
using MemoryPack;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedQuaternionTests
{
    private static void AssertRepresentsSameRotation(FixedQuaternion actual, FixedQuaternion expected, Fixed64? tolerance = null)
    {
        Fixed64 limit = tolerance ?? Fixed64.FromDouble(0.0001);
        Assert.True(
            actual.FuzzyEqual(expected, limit) || actual.FuzzyEqual(expected * -Fixed64.One, limit),
            $"Expected {actual} to represent the same rotation as {expected}.");
    }

    #region Test: Initialization and Identity

    [Fact]
    public void FixedQuaternion_InitializesCorrectly()
    {
        var quaternion = new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Assert.Equal(Fixed64.One, quaternion.X);
        Assert.Equal(Fixed64.Zero, quaternion.Y);
        Assert.Equal(Fixed64.Zero, quaternion.Z);
        Assert.Equal(Fixed64.One, quaternion.W);
    }

    [Fact]
    public void FixedQuaternion_Identity_IsCorrect()
    {
        var identity = FixedQuaternion.Identity;
        Assert.Equal(Fixed64.Zero, identity.X);
        Assert.Equal(Fixed64.Zero, identity.Y);
        Assert.Equal(Fixed64.Zero, identity.Z);
        Assert.Equal(Fixed64.One, identity.W);
    }

    [Fact]
    public void FixedQuaternion_Indexer_GetSetAndInvalidIndexBehaveCorrectly()
    {
        var quaternion = new FixedQuaternion(Fixed64.One, Fixed64.Zero, new Fixed64(2), new Fixed64(3));

        Assert.Equal(Fixed64.One, quaternion[0]);
        Assert.Equal(Fixed64.Zero, quaternion[1]);
        Assert.Equal(new Fixed64(2), quaternion[2]);
        Assert.Equal(new Fixed64(3), quaternion[3]);

        quaternion[0] = new Fixed64(7);
        quaternion[1] = new Fixed64(8);
        quaternion[2] = new Fixed64(9);
        quaternion[3] = new Fixed64(10);

        Assert.Equal(new Fixed64(7), quaternion.X);
        Assert.Equal(new Fixed64(8), quaternion.Y);
        Assert.Equal(new Fixed64(9), quaternion.Z);
        Assert.Equal(new Fixed64(10), quaternion.W);
        Assert.Throws<IndexOutOfRangeException>(() => _ = quaternion[4]);
        Assert.Throws<IndexOutOfRangeException>(() => quaternion[-1] = Fixed64.Zero);
    }

    [Fact]
    public void FixedQuaternion_Set_UpdatesAllComponents()
    {
        var quaternion = FixedQuaternion.Zero;

        quaternion.Set(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));

        Assert.Equal(new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4)), quaternion);
    }

    #endregion

    #region Test: Normalization

    [Fact]
    public void FixedQuaternion_Normalize_WorksCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(1), new Fixed64(1), new Fixed64(1));
        quaternion.NormalizeInPlace();

        var magnitudeSqr = quaternion.X * quaternion.X + quaternion.Y * quaternion.Y + quaternion.Z * quaternion.Z + quaternion.W * quaternion.W;
        Assert.Equal(Fixed64.One, magnitudeSqr, Fixed64.FromDouble(0.0001)); // Ensure magnitude squared is 1
    }

    [Fact]
    public void FixedQuaternion_NormalizeStatic_WorksCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(2), new Fixed64(2), new Fixed64(2), new Fixed64(2));
        var normalized = FixedQuaternion.GetNormalized(quaternion);

        var magnitudeSqr = normalized.X * normalized.X + normalized.Y * normalized.Y + normalized.Z * normalized.Z + normalized.W * normalized.W;
        Assert.Equal(Fixed64.One, magnitudeSqr, Fixed64.FromDouble(0.0001));
    }

    [Fact]
    public void FixedQuaternion_NormalProperty_ReturnsNormalizedCopyWithoutMutatingSource()
    {
        var quaternion = new FixedQuaternion(new Fixed64(2), Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        var normal = quaternion.Normalized;

        Assert.True(normal.IsNormalized());
        Assert.Equal(new Fixed64(2), quaternion.X);
        Assert.Equal(Fixed64.Zero, quaternion.Y);
        Assert.Equal(Fixed64.Zero, quaternion.Z);
        Assert.Equal(Fixed64.Zero, quaternion.W);
    }

    [Fact]
    public void FixedQuaternion_IsNormalized_DetectsUnitAndNonUnitQuaternions()
    {
        Assert.True(FixedQuaternion.Identity.IsNormalized());
        Assert.False(new FixedQuaternion(new Fixed64(2), Fixed64.Zero, Fixed64.Zero, Fixed64.Zero).IsNormalized());
    }

    [Fact]
    public void FixedQuaternion_GetMagnitude_And_GetNormalized_HandleZeroQuaternion()
    {
        Assert.Equal(Fixed64.Zero, FixedQuaternion.GetMagnitude(FixedQuaternion.Zero));
        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.GetNormalized(FixedQuaternion.Zero));
        Assert.Equal(
            Fixed64.FromRaw(5),
            FixedQuaternion.GetMagnitude(new FixedQuaternion(
                Fixed64.FromRaw(3),
                Fixed64.FromRaw(4),
                Fixed64.Zero,
                Fixed64.Zero)));

        var slightlyAboveUnit = new FixedQuaternion(Fixed64.One, Fixed64.FromRaw(65536), Fixed64.Zero, Fixed64.Zero);
        Assert.Equal(Fixed64.One, FixedQuaternion.GetMagnitude(slightlyAboveUnit));
    }

    [Fact]
    public void FixedQuaternion_MagnitudeAndNormalization_HandleFullFiniteComponentDomain()
    {
        Fixed64 quarterMaximum = Fixed64.FromRaw(long.MaxValue / 4L);
        FixedQuaternion[] quaternions =
        {
            new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue),
            new(quarterMaximum, quarterMaximum, quarterMaximum, quarterMaximum),
        };

        foreach (FixedQuaternion quaternion in quaternions)
        {
            Assert.Equal(GetMagnitudeOracle(quaternion), FixedQuaternion.GetMagnitude(quaternion));
            Assert.True(FixedQuaternion.GetNormalized(quaternion).IsNormalized(), $"Normalization failed for {quaternion}.");
        }

        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.GetNormalized(FixedQuaternion.Zero));
    }

    [Fact]
    public void FixedQuaternion_GetMagnitude_PreservesRepresentableNearMaximumBoundary()
    {
        var quaternion = new FixedQuaternion(
            Fixed64.FromRaw(7_441_271_719_093_614_805L),
            Fixed64.FromRaw(4_814_940_524_119_397_815L),
            Fixed64.FromRaw(2_188_609_329_145_180_825L),
            Fixed64.FromRaw(1_313_165_597_487_108_495L));
        Fixed64 expected = Fixed64.FromRaw(long.MaxValue - 3L);

        Assert.Equal(expected, GetMagnitudeOracle(quaternion));
        Assert.Equal(expected, FixedQuaternion.GetMagnitude(quaternion));
    }

    [Fact]
    public void FixedQuaternion_GetMagnitude_SaturatesOnlyWhenRoundedRootExceedsMaximum()
    {
        var roundsOutside = new FixedQuaternion(
            Fixed64.MaxValue,
            Fixed64.FromRaw(3_037_000_500L),
            Fixed64.Zero,
            Fixed64.Zero);
        var minimumComponent = new FixedQuaternion(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(GetRoundedMagnitudeRaw(roundsOutside) > long.MaxValue);
        Assert.Equal(BigInteger.One << 63, GetRoundedMagnitudeRaw(minimumComponent));
        Assert.Equal(Fixed64.MaxValue, FixedQuaternion.GetMagnitude(roundsOutside));
        Assert.Equal(Fixed64.MaxValue, FixedQuaternion.GetMagnitude(minimumComponent));
    }

    [Fact]
    public void FixedQuaternion_GetMagnitude_MatchesOracleForAsymmetricOrdinaryComponents()
    {
        var quaternion = new FixedQuaternion(
            Fixed64.FromRaw(29_624_622_239L),
            Fixed64.FromRaw(-6_665_689_937L),
            Fixed64.FromRaw(-7_791_209_407L),
            Fixed64.FromRaw(-31_327_321_054L));
        Fixed64 expected = Fixed64.FromRaw(44_318_773_151L);

        Assert.Equal(expected, GetMagnitudeOracle(quaternion));
        Assert.Equal(expected, FixedQuaternion.GetMagnitude(quaternion));
    }

    [Fact]
    public void FixedQuaternion_GetMagnitude_MatchesOracleAcrossDeterministicBroadSample()
    {
        const int sampleCountPerDomain = 10_000;
        ulong state = 0x8A5C_D789_635D_2DFFUL;
        int representableCount = 0;
        int saturatedCount = 0;

        for (int i = 0; i < sampleCountPerDomain * 2; i++)
        {
            bool fullRawDomain = i >= sampleCountPerDomain;
            FixedQuaternion quaternion = CreateDeterministicQuaternion(ref state, fullRawDomain);
            BigInteger expectedRaw = GetRoundedMagnitudeRaw(quaternion);
            Fixed64 expected = expectedRaw > long.MaxValue
                ? Fixed64.MaxValue
                : Fixed64.FromRaw((long)expectedRaw);

            if (expectedRaw > long.MaxValue)
                saturatedCount++;
            else
                representableCount++;

            Fixed64 actual = FixedQuaternion.GetMagnitude(quaternion);
            Assert.True(
                actual == expected,
                $"Sample {i} ({(fullRawDomain ? "full" : "ordinary")}) expected raw {expected.m_rawValue} " +
                $"but got {actual.m_rawValue} for ({quaternion.X.m_rawValue}, {quaternion.Y.m_rawValue}, " +
                $"{quaternion.Z.m_rawValue}, {quaternion.W.m_rawValue}).");
        }

        Assert.True(representableCount > 0);
        Assert.True(saturatedCount > 0);
    }

    [Fact]
    public void FixedQuaternion_Normalization_IsInvariantUnderRepresentablePositiveScaling()
    {
        var source = new FixedQuaternion(Fixed64.One, -Fixed64.Two, Fixed64.Three, new Fixed64(4));
        var scaled = source * new Fixed64(17);

        Assert.True(source.Normalized.FuzzyEqual(scaled.Normalized, Fixed64.Epsilon));
        Assert.True(source.Normalized.IsNormalized());
        Assert.True(scaled.Normalized.IsNormalized());
    }

    [Theory]
    [InlineData(1, 1, 1, 1)]
    [InlineData(2, 2, 2, 2)]
    [InlineData(-3, 4, 12, -5)]
    [InlineData(123, 456, 789, 321)]
    public void FixedQuaternion_Normalize_MatchesComponentDivisionByMagnitude(int x, int y, int z, int w)
    {
        var source = new FixedQuaternion(new Fixed64(x), new Fixed64(y), new Fixed64(z), new Fixed64(w));

        AssertNormalizeMatchesComponentDivision(source);
    }

    [Fact]
    public void FixedQuaternion_Normalize_MatchesComponentDivisionByMagnitude_ForFractionalAndHugeValues()
    {
        AssertNormalizeMatchesComponentDivision(new FixedQuaternion(
            Fixed64.FromDouble(1.5),
            Fixed64.FromDouble(-2.25),
            Fixed64.FromDouble(3.75),
            Fixed64.FromDouble(-4.5)));

        AssertNormalizeMatchesComponentDivision(new FixedQuaternion(
            new Fixed64(10000),
            new Fixed64(-20000),
            new Fixed64(30000),
            new Fixed64(-10000)));
    }

    [Fact]
    public void FixedQuaternion_Normalize_TinyRawComponentsPreservesRatiosAndReturnsUnitQuaternion()
    {
        var source = new FixedQuaternion(
            Fixed64.FromRaw(1),
            Fixed64.FromRaw(-1),
            Fixed64.FromRaw(2),
            Fixed64.FromRaw(-2));

        FixedQuaternion normalized = source.Normalized;
        Assert.True(normalized.IsNormalized());
        Assert.Equal(normalized.X, -normalized.Y);
        Assert.Equal(normalized.Z, -normalized.W);
        Assert.True(FixedMath.Abs(normalized.Z - (normalized.X * Fixed64.Two)) <= Fixed64.Epsilon);

        var inPlace = source;
        Assert.Equal(normalized, inPlace.NormalizeInPlace());
    }

    [Fact]
    public void FixedQuaternion_MagnitudeProperties_ReturnExpectedValues()
    {
        var quaternion = new FixedQuaternion(Fixed64.One, new Fixed64(2), new Fixed64(2), new Fixed64(4));

        Assert.Equal(new Fixed64(25), quaternion.MagnitudeSquared);
        Assert.Equal(new Fixed64(5), quaternion.Magnitude);
        Assert.Equal(FixedQuaternion.GetMagnitude(quaternion), quaternion.Magnitude);
    }

    private static void AssertNormalizeMatchesComponentDivision(FixedQuaternion source)
    {
        Fixed64 magnitude = source.Magnitude;
        if (magnitude == Fixed64.Zero)
        {
            Assert.Equal(FixedQuaternion.Identity, source.Normalized);

            var zeroLength = source;
            Assert.Equal(FixedQuaternion.Identity, zeroLength.NormalizeInPlace());
            return;
        }

        var expected = new FixedQuaternion(
            source.X / magnitude,
            source.Y / magnitude,
            source.Z / magnitude,
            source.W / magnitude);

        Assert.Equal(expected, source.Normalized);

        var inPlace = source;
        Assert.Equal(expected, inPlace.NormalizeInPlace());
    }

    #endregion

    #region Test: Conjugate and Inverse

    [Fact]
    public void FixedQuaternion_Conjugate_WorksCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var conjugate = quaternion.Conjugate();

        Assert.Equal(new Fixed64(-1), conjugate.X);
        Assert.Equal(new Fixed64(-2), conjugate.Y);
        Assert.Equal(new Fixed64(-3), conjugate.Z);
        Assert.Equal(new Fixed64(4), conjugate.W);
    }

    [Fact]
    public void FixedQuaternion_Inverse_WorksCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var inverse = quaternion.Inverse();

        var multiplied = quaternion * inverse;
        Assert.True(multiplied.FuzzyEqual(FixedQuaternion.Identity)); // quaternion * inverse = identity
    }

    [Fact]
    public void FixedQuaternion_Inverse_HandlesIdentityAndZero()
    {
        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.Identity.Inverse());
        Assert.Equal(FixedQuaternion.Zero, FixedQuaternion.Zero.Inverse());
    }

    [Fact]
    public void FixedQuaternion_Divide_MultipliesByInverseDivisor()
    {
        var dividend = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        var divisor = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);

        var result = FixedQuaternion.Divide(dividend, divisor);

        AssertRepresentsSameRotation(result, FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4));
    }

    [Fact]
    public void FixedQuaternion_Divide_ZeroDivisorThrows()
    {
        Assert.Throws<InvalidOperationException>(() => FixedQuaternion.Divide(FixedQuaternion.Identity, FixedQuaternion.Zero));
    }

    #endregion

    #region Test: Conversion

    [Fact]
    public void FixedQuaternion_FromEulerAnglesInDegrees_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));
        var eulerAngles = quaternion.EulerAngles;

        Assert.True(eulerAngles.FuzzyEqual(new Vector3d(90, 0, 0), Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_FromEulerAnglesInDegrees_MatchesExpected()
    {
        var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(90));
        var expected = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, FixedMath.Sqrt(Fixed64.Half), FixedMath.Sqrt(Fixed64.Half));

        Assert.True(quaternion.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)),
                    $"Expected: {expected}, Actual: {quaternion}");
    }

    [Fact]
    public void FixedQuaternion_ToEulerAngles_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));
        var eulerAngles = quaternion.ToEulerAngles();

        Assert.True(eulerAngles.FuzzyEqual(new Vector3d(90, 0, 0), Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_EulerAngles_SetterUpdatesQuaternion()
    {
        var quaternion = FixedQuaternion.Identity;

        quaternion.EulerAngles = new Vector3d(0, 90, 0);

        Assert.True(quaternion.FuzzyEqual(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, new Fixed64(90), Fixed64.Zero), Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_FromMatrix_WorksCorrectly()
    {
        var matrix = new Fixed3x3(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        var result = FixedQuaternion.FromMatrix(matrix);
        Assert.True(result.FuzzyEqual(FixedQuaternion.Identity), $"FromMatrix returned {result}, expected Identity.");
    }

    [Fact]
    public void FixedQuaternion_FromMatrix4x4_WorksCorrectly()
    {
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        var matrix = Fixed4x4.CreateTransform(Vector3d.Zero, rotation, Vector3d.One);

        var result = FixedQuaternion.FromMatrix(matrix);

        Assert.True(result.FuzzyEqual(rotation, Fixed64.FromDouble(0.0001)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void FixedQuaternion_FromMatrix_HandlesNegativeTraceBranches(int branch)
    {
        FixedQuaternion expected = branch switch
        {
            0 => FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.Pi),
            1 => FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi),
            _ => FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.Pi),
        };

        FixedQuaternion result = FixedQuaternion.FromMatrix(expected.ToMatrix3x3());

        AssertRepresentsSameRotation(result, expected);
    }

    [Fact]
    public void FixedQuaternion_FromDirection_WorksCorrectly()
    {
        var direction = Vector3d.Right; // X-axis direction
        var result = FixedQuaternion.FromDirection(direction);

        // Expect a 90-degree rotation around the Y-axis
        var expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);

        Assert.True(result.FuzzyEqual(expected), $"FromDirection returned {result}, expected {expected}.");
    }

    [Fact]
    public void FixedQuaternion_FromDirection_ForwardReturnsIdentity()
    {
        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.FromDirection(Vector3d.Forward));
        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.FromDirection(Vector3d.Zero));
    }

    [Fact]
    public void FixedQuaternion_CanonicalForwardDirection_ReturnsIdentityRotation()
    {
        AssertRepresentsSameRotation(FixedQuaternion.FromDirection(Vector3d.Forward), FixedQuaternion.Identity);
        AssertRepresentsSameRotation(FixedQuaternion.LookRotation(Vector3d.Forward), FixedQuaternion.Identity);
    }

    [Fact]
    public void FixedQuaternion_FromDirection_NormalizesDirectionInput()
    {
        FixedQuaternion scaled = FixedQuaternion.FromDirection(Vector3d.Right * new Fixed64(3));
        FixedQuaternion unit = FixedQuaternion.FromDirection(Vector3d.Right);

        AssertRepresentsSameRotation(scaled, unit);
    }

    [Fact]
    public void FixedQuaternion_FromDirection_NormalizesExtremeDirectionWithoutSaturatedSquareSum()
    {
        var extreme = new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);

        FixedQuaternion actual = FixedQuaternion.FromDirection(extreme);
        FixedQuaternion expected = FixedQuaternion.FromDirection(extreme.Normalized);

        AssertRepresentsSameRotation(actual, expected);
    }

    [Fact]
    public void FixedQuaternion_FromDirection_BackwardRotatesForwardToBackward()
    {
        FixedQuaternion result = FixedQuaternion.FromDirection(Vector3d.Backward);

        Vector3d rotatedForward = result.Rotate(Vector3d.Forward);

        Assert.True(
            rotatedForward.FuzzyEqual(Vector3d.Backward, Fixed64.FromDouble(0.0001)),
            $"FromDirection returned {result}, which rotated forward to {rotatedForward} instead of {Vector3d.Backward}.");
    }

    [Fact]
    public void FixedQuaternion_FromDirection_NearForwardRotatesForwardToDirection()
    {
        Vector3d direction = new Vector3d(Fixed64.FromDouble(0.001), Fixed64.Zero, Fixed64.One).Normalized;

        FixedQuaternion result = FixedQuaternion.FromDirection(direction);

        Vector3d rotatedForward = result.Rotate(Vector3d.Forward);
        Assert.True(
            rotatedForward.FuzzyEqual(direction, Fixed64.FromDouble(0.0001)),
            $"FromDirection returned {result}, which rotated forward to {rotatedForward} instead of {direction}.");
    }

    [Fact]
    public void FixedQuaternion_FromDirection_NearBackwardRotatesForwardToDirection()
    {
        Vector3d direction = new Vector3d(Fixed64.FromDouble(0.001), Fixed64.Zero, -Fixed64.One).Normalized;

        FixedQuaternion result = FixedQuaternion.FromDirection(direction);

        Vector3d rotatedForward = result.Rotate(Vector3d.Forward);
        Assert.True(
            rotatedForward.FuzzyEqual(direction, Fixed64.FromDouble(0.0002)),
            $"FromDirection returned {result}, which rotated forward to {rotatedForward} instead of {direction}.");
    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_WorksCorrectly()
    {
        var axis = Vector3d.Up;
        var angle = Fixed64.HalfPi;  // 90 degrees

        var result = FixedQuaternion.FromAxisAngle(axis, angle);
        var expected = new FixedQuaternion(Fixed64.Zero, FixedMath.Sin(Fixed64.PiOver4), Fixed64.Zero, FixedMath.Cos(Fixed64.PiOver4));

        Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

        axis = Vector3d.Forward;

        result = FixedQuaternion.FromAxisAngle(axis, angle);
        expected = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, FixedMath.Sin(Fixed64.PiOver4), FixedMath.Cos(Fixed64.PiOver4));

        Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

        axis = Vector3d.Right;

        result = FixedQuaternion.FromAxisAngle(axis, angle);
        expected = new FixedQuaternion(FixedMath.Sin(Fixed64.PiOver4), Fixed64.Zero, Fixed64.Zero, FixedMath.Cos(Fixed64.PiOver4));

        Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_NormalizesAxisInput()
    {
        var result = FixedQuaternion.FromAxisAngle(new Vector3d(0, 2, 0), Fixed64.HalfPi);
        var expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);

        Assert.True(result.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)));
        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.FromAxisAngle(Vector3d.Zero, Fixed64.HalfPi));
    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_NormalizesExtremeAxisWithoutSaturatedSquareSum()
    {
        var extreme = new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);

        FixedQuaternion actual = FixedQuaternion.FromAxisAngle(extreme, Fixed64.PiOver4);
        FixedQuaternion expected = FixedQuaternion.FromAxisAngle(extreme.Normalized, Fixed64.PiOver4);

        AssertRepresentsSameRotation(actual, expected);
    }

    [Fact]
    public void FixedQuaternion_FromEulerAngles_ValidInput_DoesNotAllocate()
    {
        Fixed64 pitch = Fixed64.PiOver4;
        Fixed64 yaw = Fixed64.Zero;
        Fixed64 roll = Fixed64.Zero;

        _ = FixedQuaternion.FromEulerAngles(pitch, yaw, roll);

        long before = GC.GetAllocatedBytesForCurrentThread();
        _ = FixedQuaternion.FromEulerAngles(pitch, yaw, roll);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);
    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_AcceptsPeriodicMultiTurnAngles()
    {
        Fixed64 angle = Fixed64.PiOver4;
        FixedQuaternion expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, angle);
        Fixed64[] periodicAngles =
        {
            angle + Fixed64.TwoPi,
            angle - Fixed64.TwoPi,
            angle + (Fixed64.TwoPi * new Fixed64(7)),
            angle - (Fixed64.TwoPi * new Fixed64(6)),
        };

        foreach (Fixed64 periodicAngle in periodicAngles)
            AssertRepresentsSameRotation(FixedQuaternion.FromAxisAngle(Vector3d.Up, periodicAngle), expected);
    }

    [Fact]
    public void FixedQuaternion_FromEulerAngles_AcceptsPeriodicMultiTurnComponents()
    {
        Fixed64 pitch = Fixed64.PiOver6;
        Fixed64 yaw = -Fixed64.PiOver4;
        Fixed64 roll = Fixed64.PiOver3;
        FixedQuaternion expected = FixedQuaternion.FromEulerAngles(pitch, yaw, roll);

        AssertRepresentsSameRotation(
            FixedQuaternion.FromEulerAngles(pitch + Fixed64.TwoPi, yaw, roll),
            expected);
        AssertRepresentsSameRotation(
            FixedQuaternion.FromEulerAngles(pitch, yaw - (Fixed64.TwoPi * new Fixed64(5)), roll),
            expected);
        AssertRepresentsSameRotation(
            FixedQuaternion.FromEulerAngles(pitch, yaw, roll + (Fixed64.TwoPi * new Fixed64(4))),
            expected);
    }

    [Fact]
    public void FixedQuaternion_ToDirection_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.Identity;
        var result = quaternion.ToDirection();

        var expected = new Vector3d(0, 0, 1);  // Default forward direction

        Assert.True(result.FuzzyEqual(expected), $"ToDirection returned {result}, expected {expected}.");
    }

    [Fact]
    public void FixedQuaternion_ToMatrix_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.Identity;
        var result = quaternion.ToMatrix3x3();

        var expected = new Fixed3x3(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        Assert.True(result == expected, $"ToMatrix returned {result}, expected Identity matrix.");
    }

    [Fact]
    public void FixedQuaternion_ToEulerAngles_HandlesGimbalLockPitch()
    {
        var quaternion = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        var eulerAngles = quaternion.ToEulerAngles();

        Assert.True(eulerAngles.FuzzyEqual(new Vector3d(0, 90, 0)));
    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_Equals_FromEulerAngles()
    {
        var quaternion = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        var expectedQuaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(90), new Fixed64(0));

        Assert.Equal(Fixed64.HalfPi, FixedMath.DegToRad(new Fixed64(90)));

        Assert.True(quaternion.Equals(expectedQuaternion));
    }

    [Fact]
    public void FixedQuaternion_ToAngularVelocity_WorksCorrectly()
    {
        var prevRotation = FixedQuaternion.Identity;
        var currentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4); // Rotated 45 degrees around Y-axis
        var deltaTime = new Fixed64(2); // Assume 2 seconds elapsed

        var angularVelocity = FixedQuaternion.ToAngularVelocity(currentRotation, prevRotation, deltaTime);

        var expected = new Vector3d(Fixed64.Zero, Fixed64.PiOver4 / deltaTime, Fixed64.Zero); // Expect ω = θ / dt
        Assert.True(angularVelocity.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)),
            $"ToAngularVelocity returned {angularVelocity}, expected {expected}");
    }

    [Fact]
    public void FixedQuaternion_ToAngularVelocity_ZeroForNoRotation()
    {
        var prevRotation = FixedQuaternion.Identity;
        var currentRotation = FixedQuaternion.Identity;
        var deltaTime = Fixed64.One;

        var angularVelocity = FixedQuaternion.ToAngularVelocity(currentRotation, prevRotation, deltaTime);

        Assert.True(angularVelocity.FuzzyEqual(Vector3d.Zero),
            $"ToAngularVelocity should return zero for no rotation, but got {angularVelocity}");
    }

    #endregion

    #region Test: Lerp and Slerp

    [Fact]
    public void FixedQuaternion_Lerp_WorksCorrectly()
    {
        var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var result = FixedQuaternion.Lerp(q1, q2, Fixed64.FromDouble(0.5)); // Halfway between q1 and q2
        var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(45), new Fixed64(0), new Fixed64(0));

        Assert.True(result.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_Lerp_ClampsInterpolationFactor()
    {
        var q1 = FixedQuaternion.Identity;
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var result = FixedQuaternion.Lerp(q1, q2, new Fixed64(2));

        Assert.True(result.FuzzyEqual(q2, Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_Lerp_UsesShortestPathForNegatedQuaternion()
    {
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.HalfPi);
        var negatedRotation = rotation * -Fixed64.One;

        var result = FixedQuaternion.Lerp(rotation, negatedRotation, Fixed64.Half);

        AssertRepresentsSameRotation(result, rotation);
    }

    [Fact]
    public void FixedQuaternion_Slerp_WorksCorrectly()
    {
        var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var result = FixedQuaternion.Slerp(q1, q2, Fixed64.FromDouble(0.5)); // Halfway between q1 and q2
        var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(45), new Fixed64(0), new Fixed64(0));

        Assert.True(result.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_Slerp_UsesShortestPathForNegatedQuaternion()
    {
        var result = FixedQuaternion.Slerp(FixedQuaternion.Identity, FixedQuaternion.Identity * -Fixed64.One, Fixed64.Half);

        Assert.True(result.FuzzyEqual(FixedQuaternion.Identity, Fixed64.FromDouble(0.0001)));
    }


    #endregion

    #region Test: Dot Product and Angle

    [Fact]
    public void FixedQuaternion_Dot_WorksCorrectly()
    {
        var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var dotProduct = FixedQuaternion.Dot(q1, q2);
        Assert.True(dotProduct > Fixed64.Zero); // Should be positive for non-opposite quaternions
    }

    [Fact]
    public void FixedQuaternion_AngleBetween_WorksCorrectly()
    {
        var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var angle = FixedQuaternion.Angle(q1, q2);
        FixedMathTestHelper.AssertWithinRelativeTolerance(new Fixed64(90), angle);
    }

    [Fact]
    public void FixedQuaternion_AngleBetween_NearIdentityUsesRelativeVectorMagnitude()
    {
        Fixed64 expectedAngle = Fixed64.FromFraction(21, 1000);
        FixedQuaternion target = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            FixedMath.DegToRad(expectedAngle)).Normalized;

        Fixed64 actualAngle = FixedQuaternion.Angle(FixedQuaternion.Identity, target);

        Assert.True(
            (actualAngle - expectedAngle).Abs() <= Fixed64.Epsilon * (Fixed64)16,
            $"Expected {expectedAngle} degrees, got {actualAngle} degrees.");
        Assert.Equal(Fixed64.Zero, FixedQuaternion.Angle(target, -target));
    }

    [Fact]
    public void FixedQuaternion_AngleAxis_WorksCorrectly()
    {
        var angle = new Fixed64(90);  // 90 degrees
        var axis = Vector3d.Up;

        var result = FixedQuaternion.AngleAxis(angle, axis);
        var expected = FixedQuaternion.FromAxisAngle(axis, Fixed64.HalfPi);

        Assert.True(result.FuzzyEqual(expected), $"AngleAxis returned {result}, expected {expected}.");
    }

    [Fact]
    public void FixedQuaternion_DegreeConstructors_AcceptPeriodicMultiTurnAngles()
    {
        FixedQuaternion expectedEuler = FixedQuaternion.FromEulerAnglesInDegrees(
            new Fixed64(30),
            new Fixed64(-45),
            new Fixed64(60));

        AssertRepresentsSameRotation(
            FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(390), new Fixed64(-45), new Fixed64(60)),
            expectedEuler);
        AssertRepresentsSameRotation(
            FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(30), new Fixed64(-765), new Fixed64(60)),
            expectedEuler);
        AssertRepresentsSameRotation(
            FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(30), new Fixed64(-45), new Fixed64(1_500)),
            expectedEuler);

        Fixed64[] angles =
        {
            new Fixed64(90),
            new Fixed64(450),
            new Fixed64(-270),
            new Fixed64(2_250),
            new Fixed64(-2_070),
        };
        foreach (Fixed64 angle in angles)
        {
            AssertRepresentsSameRotation(
                FixedQuaternion.AngleAxis(angle, Vector3d.Up),
                FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.DegToRad(angle)));
        }

        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.AngleAxis(new Fixed64(90), Vector3d.Zero));
    }

    [Theory]
    [InlineData(long.MinValue)]
    [InlineData(long.MaxValue)]
    public void FixedQuaternion_ExtremeRadianConstructors_MatchModuloTwoPiReduction(long angleRaw)
    {
        Fixed64 angle = Fixed64.FromRaw(angleRaw);
        Fixed64 reduced = angle % Fixed64.TwoPi;

        AssertRepresentsSameRotation(
            FixedQuaternion.FromAxisAngle(Vector3d.Up, angle),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, reduced));
        AssertRepresentsSameRotation(
            FixedQuaternion.FromEulerAngles(angle, Fixed64.PiOver6, -Fixed64.PiOver4),
            FixedQuaternion.FromEulerAngles(reduced, Fixed64.PiOver6, -Fixed64.PiOver4));
    }

    [Theory]
    [InlineData(long.MinValue)]
    [InlineData(long.MaxValue)]
    public void FixedQuaternion_ExtremeDegreeConstructors_MatchModuloFullTurnReduction(long angleRaw)
    {
        Fixed64 angle = Fixed64.FromRaw(angleRaw);
        Fixed64 reduced = angle % new Fixed64(360);

        AssertRepresentsSameRotation(
            FixedQuaternion.AngleAxis(angle, Vector3d.Up),
            FixedQuaternion.AngleAxis(reduced, Vector3d.Up));
        AssertRepresentsSameRotation(
            FixedQuaternion.FromEulerAnglesInDegrees(angle, new Fixed64(30), new Fixed64(-45)),
            FixedQuaternion.FromEulerAnglesInDegrees(reduced, new Fixed64(30), new Fixed64(-45)));
    }

    #endregion

    #region Test: Rotation

    [Fact]
    public void FixedQuaternion_RotateVector_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(90)); // 90° around Z-axis
        var vector = new Vector3d(1, 0, 0);

        var result = quaternion.Rotate(vector);
        Assert.True(result.FuzzyEqual(new Vector3d(0, 1, 0))); // Expect (0, 1, 0) after rotation
    }

    [Fact]
    public void FixedQuaternion_TryRotate_UsesExactScaleInvariantBasis()
    {
        var quaternion = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);

        Assert.True(quaternion.TryRotate(Vector3d.Right, out Vector3d rotated));
        Assert.True(rotated.FuzzyEqual(Vector3d.Up, Fixed64.FromRaw(2)));

        Assert.True(FixedQuaternion.Zero.TryRotate(
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue),
            out Vector3d zero));
        Assert.Equal(Vector3d.Zero, zero);

        FixedQuaternion diagonal = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.PiOver4);
        Assert.False(diagonal.TryRotate(
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero),
            out Vector3d overflow));
        Assert.Equal(default, overflow);
    }

    [Fact]
    public void FixedQuaternion_ExactPointOperations_DeferNarrowingAndFailAtomically()
    {
        var rotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Two);
        var localPoint = new Vector3d(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);
        Assert.False(rotation.TryRotate(localPoint, out _));
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            Fixed64.One,
            (Fixed64)5,
            out Fixed64 oneFifth));
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            (Fixed64)2,
            (Fixed64)5,
            out Fixed64 twoFifths));
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            (Fixed64)3,
            (Fixed64)5,
            out Fixed64 threeFifths));

        Assert.True(rotation.TryTransformPoint(
            new Vector3d(
                Fixed64.Zero,
                -Fixed64.MaxValue,
                Fixed64.Zero),
            localPoint,
            out Vector3d transformed));
        Assert.Equal(new Vector3d(
            -oneFifth,
            twoFifths,
            Fixed64.Zero), transformed);

        Assert.True(rotation.TryGetRelativeOffset(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.Zero),
            new Vector3d(
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.Zero),
            Vector3d.Zero,
            localPoint,
            out Vector3d relative));
        Assert.Equal(new Vector3d(
            oneFifth,
            threeFifths,
            Fixed64.Zero), relative);

        Assert.True(FixedQuaternion.Identity.TryTransformPoint(
            Vector3d.One,
            Vector3d.One,
            out Vector3d identityPoint));
        Assert.Equal(Vector3d.One * Fixed64.Two, identityPoint);
        Assert.True(FixedQuaternion.Identity.TryGetRelativeOffset(
            Vector3d.One,
            Vector3d.One,
            Vector3d.One,
            Vector3d.One,
            out Vector3d identityRelative));
        Assert.Equal(Vector3d.Zero, identityRelative);

        Assert.False(FixedQuaternion.Identity.TryTransformPoint(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            out Vector3d transformOverflow));
        Assert.Equal(default, transformOverflow);
        Assert.False(FixedQuaternion.Identity.TryGetRelativeOffset(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(-Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            out Vector3d relativeOverflow));
        Assert.Equal(default, relativeOverflow);
        Assert.False(rotation.TryGetRelativeOffset(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero,
            out Vector3d rotatedRelativeOverflow));
        Assert.Equal(default, rotatedRelativeOverflow);
    }

    [Fact]
    public void FixedQuaternion_ExactPointOperations_PreserveZeroQuaternionContract()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.One);

        Assert.True(FixedQuaternion.Zero.TryTransformPoint(
            origin,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue),
            out Vector3d transformed));
        Assert.Equal(origin, transformed);

        Assert.True(FixedQuaternion.Zero.TryGetRelativeOffset(
            origin,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.MaxValue,
                Fixed64.Zero),
            Vector3d.Zero,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue),
            out Vector3d relative));
        Assert.Equal(new Vector3d(
            -Fixed64.MinIncrement,
            -Fixed64.MinIncrement,
            Fixed64.One), relative);
    }

    [Fact]
    public void FixedQuaternion_ThreeTermPointTransform_DefersAllNarrowingAndFailsAtomically()
    {
        var rotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Two);
        var localPoint = new Vector3d(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            Fixed64.One,
            (Fixed64)5,
            out Fixed64 oneFifth));
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            (Fixed64)3,
            (Fixed64)5,
            out Fixed64 threeFifths));

        Assert.True(rotation.TryTransformPoint(
            new Vector3d(Fixed64.Zero, -Fixed64.MaxValue, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, -Fixed64.MaxValue, Fixed64.Zero),
            localPoint,
            out Vector3d rotatedCancellation));
        Assert.Equal(
            new Vector3d(-oneFifth, -threeFifths, Fixed64.Zero),
            rotatedCancellation);

        Assert.True(FixedQuaternion.Identity.TryTransformPoint(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(-Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            out Vector3d identityCancellation));
        Assert.Equal(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            identityCancellation);

        Assert.True(FixedQuaternion.Zero.TryTransformPoint(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            localPoint,
            out Vector3d zeroRotation));
        Assert.Equal(
            new Vector3d(-Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero),
            zeroRotation);

        Assert.False(FixedQuaternion.Identity.TryTransformPoint(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            out Vector3d overflow));
        Assert.Equal(default, overflow);
    }

    [Fact]
    public void FixedQuaternion_ExactPointOperations_DoNotAllocateAfterWarmup()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            Fixed64.PiOver4);
        Vector3d origin = new(Fixed64.One, Fixed64.Two, (Fixed64)3);
        Vector3d localPoint = new(
            Fixed64.FromFraction(1, 4),
            Fixed64.FromFraction(1, 2),
            Fixed64.FromFraction(3, 4));
        Assert.True(rotation.TryTransformPoint(
            origin,
            localPoint,
            out Vector3d transformed));
        Assert.True(rotation.TryTransformPoint(
            origin,
            Vector3d.One,
            localPoint,
            out transformed));
        Assert.True(rotation.TryGetRelativeOffset(
            origin,
            localPoint,
            Vector3d.One,
            localPoint,
            out Vector3d relative));
        Assert.True(FixedQuaternion.Identity.TryTransformPoint(
            origin,
            localPoint,
            out transformed));
        Assert.True(FixedQuaternion.Identity.TryTransformPoint(
            origin,
            Vector3d.One,
            localPoint,
            out transformed));
        Assert.True(FixedQuaternion.Zero.TryTransformPoint(
            origin,
            Vector3d.One,
            localPoint,
            out transformed));
        Assert.True(FixedQuaternion.Identity.TryGetRelativeOffset(
            origin,
            localPoint,
            Vector3d.One,
            localPoint,
            out relative));

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 32; iteration++)
        {
            _ = rotation.TryTransformPoint(
                origin,
                localPoint,
                out transformed);
            _ = rotation.TryTransformPoint(
                origin,
                Vector3d.One,
                localPoint,
                out transformed);
            _ = rotation.TryGetRelativeOffset(
                origin,
                localPoint,
                Vector3d.One,
                localPoint,
                out relative);
            _ = FixedQuaternion.Identity.TryTransformPoint(
                origin,
                localPoint,
                out transformed);
            _ = FixedQuaternion.Identity.TryTransformPoint(
                origin,
                Vector3d.One,
                localPoint,
                out transformed);
            _ = FixedQuaternion.Zero.TryTransformPoint(
                origin,
                Vector3d.One,
                localPoint,
                out transformed);
            _ = FixedQuaternion.Identity.TryGetRelativeOffset(
                origin,
                localPoint,
                Vector3d.One,
                localPoint,
                out relative);
        }

        Assert.Equal(
            0L,
            GC.GetAllocatedBytesForCurrentThread() - before);
    }

    [Fact]
    public void FixedQuaternion_Rotated_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.Identity;
        var sin = FixedMath.Sin(Fixed64.PiOver4);  // 45° rotation
        var cos = FixedMath.Cos(Fixed64.PiOver4);

        var result = quaternion.Rotated(sin, cos);
        var expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);

        Assert.True(result.FuzzyEqual(expected), $"Rotated quaternion was {result}, expected {expected}.");
    }

    [Fact]
    public void FixedQuaternion_Rotated_UsesCustomAxis()
    {
        var quaternion = FixedQuaternion.Identity;
        var sin = FixedMath.Sin(Fixed64.PiOver4);
        var cos = FixedMath.Cos(Fixed64.PiOver4);

        var result = quaternion.Rotated(sin, cos, Vector3d.Right);

        Assert.True(result.FuzzyEqual(FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4), Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_LookRotation_WorksCorrectly()
    {
        var forward = new Vector3d(0, 0, 1);
        var result = FixedQuaternion.LookRotation(forward);

        var expected = FixedQuaternion.Identity;  // No rotation needed along Z-axis

        Assert.True(result.FuzzyEqual(expected), $"Look rotation returned {result}, expected {expected}.");
    }

    [Fact]
    public void FixedQuaternion_LookRotation_WithCustomUpVector_WorksCorrectly()
    {
        var result = FixedQuaternion.LookRotation(Vector3d.Right, Vector3d.Up);

        Assert.True(result.FuzzyEqual(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi), Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_QuaternionLog_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4); // 45-degree rotation around Y-axis
        var logResult = FixedQuaternion.QuaternionLog(quaternion);

        var expected = new Vector3d(Fixed64.Zero, Fixed64.PiOver4, Fixed64.Zero); // Expect log(q) = θ * axis
        Assert.True(logResult.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)),
            $"QuaternionLog returned {logResult}, expected {expected}");
    }

    [Fact]
    public void FixedQuaternion_QuaternionLog_ReturnsZeroForIdentity()
    {
        var identity = FixedQuaternion.Identity;
        var logResult = FixedQuaternion.QuaternionLog(identity);

        Assert.True(logResult.FuzzyEqual(Vector3d.Zero),
            $"QuaternionLog of Identity should be (0,0,0), but got {logResult}");
    }

    [Fact]
    public void FixedQuaternion_QuaternionLog_ReturnsZeroForVerySmallRotation()
    {
        var tinyRotation = new FixedQuaternion(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero, Fixed64.One).NormalizeInPlace();

        Assert.True(FixedQuaternion.QuaternionLog(tinyRotation).FuzzyEqual(Vector3d.Zero));
    }

    [Fact]
    public void FixedQuaternion_QuaternionLog_NormalizesAndClampsEndpointDriftAcrossFiniteDomain()
    {
        Fixed64 endpointVectorThreshold = Fixed64.FromRaw(4_096);
        FixedQuaternion[] quaternions =
        {
            new(endpointVectorThreshold, Fixed64.Zero, Fixed64.Zero, Fixed64.One + Fixed64.MinIncrement),
            new(endpointVectorThreshold, Fixed64.Zero, Fixed64.Zero, -Fixed64.One - Fixed64.MinIncrement),
            new(endpointVectorThreshold, Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new(endpointVectorThreshold, Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue),
        };

        foreach (FixedQuaternion quaternion in quaternions)
            AssertQuaternionLogMatchesIndependentNormalization(quaternion);
    }

    [Fact]
    public void FixedQuaternion_QuaternionLog_ZeroAndTinyVectorPartReturnZero()
    {
        Assert.Equal(Vector3d.Zero, FixedQuaternion.QuaternionLog(FixedQuaternion.Zero));

        var tinyVectorPart = new FixedQuaternion(
            Fixed64.FromRaw(4_095),
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One + Fixed64.MinIncrement);
        Assert.Equal(Vector3d.Zero, FixedQuaternion.QuaternionLog(tinyVectorPart));
    }

    #endregion

    #region Test: Operators

    [Fact]
    public void FixedQuaternion_Multiplication_WorksCorrectly()
    {
        var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(90));
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var result = q2 * q1; // quaternion multiplication is non-commutative
        var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(90));

        Assert.True(result.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_ScalarArithmeticEqualityAndFormatting_WorkCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var same = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));

        Assert.Equal(new FixedQuaternion(new Fixed64(2), new Fixed64(4), new Fixed64(6), new Fixed64(8)), quaternion * new Fixed64(2));
        Assert.Equal(new FixedQuaternion(new Fixed64(2), new Fixed64(4), new Fixed64(6), new Fixed64(8)), new Fixed64(2) * quaternion);
        Assert.Equal(new FixedQuaternion(Fixed64.FromDouble(0.5), Fixed64.One, Fixed64.FromDouble(1.5), new Fixed64(2)), quaternion / new Fixed64(2));
        Assert.Equal(new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(5)), quaternion + FixedQuaternion.Identity);
        Assert.True(quaternion == same);
        Assert.False(quaternion != same);
        Assert.False(quaternion.Equals("not-a-quaternion"));
        Assert.Equal(quaternion.GetHashCode(), same.GetHashCode());
        Assert.Equal("(1, 2, 3, 4)", quaternion.ToString());
    }

    [Fact]
    public void FixedQuaternion_SubtractionNegationAndDeconstruction_WorkCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var other = new FixedQuaternion(Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One);

        Assert.Equal(new FixedQuaternion(Fixed64.Zero, Fixed64.One, new Fixed64(2), new Fixed64(3)), quaternion - other);
        Assert.Equal(new FixedQuaternion(new Fixed64(-1), new Fixed64(-2), new Fixed64(-3), new Fixed64(-4)), -quaternion);

        quaternion.Deconstruct(out Fixed64 x, out Fixed64 y, out Fixed64 z, out Fixed64 w);
        quaternion.Deconstruct(out int ix, out int iy, out int iz, out int iw);
        quaternion.Deconstruct(out long lx, out long ly, out long lz, out long lw);
        quaternion.Deconstruct(out double dx, out double dy, out double dz, out double dw);

        Assert.Equal(new Fixed64(1), x);
        Assert.Equal(new Fixed64(2), y);
        Assert.Equal(new Fixed64(3), z);
        Assert.Equal(new Fixed64(4), w);
        Assert.Equal(1, ix);
        Assert.Equal(2, iy);
        Assert.Equal(3, iz);
        Assert.Equal(4, iw);
        Assert.Equal(quaternion.X.m_rawValue, lx);
        Assert.Equal(quaternion.Y.m_rawValue, ly);
        Assert.Equal(quaternion.Z.m_rawValue, lz);
        Assert.Equal(quaternion.W.m_rawValue, lw);
        Assert.Equal(1d, dx);
        Assert.Equal(2d, dy);
        Assert.Equal(3d, dz);
        Assert.Equal(4d, dw);
    }

    [Fact]
    public void FixedQuaternion_ExtensionMethods_DelegateAndCompareCorrectly()
    {
        var currentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);
        var previousRotation = FixedQuaternion.Identity;
        var nearRotation = new FixedQuaternion(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        var angularVelocity = currentRotation.ToAngularVelocity(previousRotation, new Fixed64(2));

        Assert.True(angularVelocity.FuzzyEqual(FixedQuaternion.ToAngularVelocity(currentRotation, previousRotation, new Fixed64(2)), Fixed64.FromDouble(0.0001)));
        Assert.True(FixedQuaternion.Identity.FuzzyEqualAbsolute(nearRotation, Fixed64.FromRaw(1)));
        Assert.True(FixedQuaternion.Identity.FuzzyEqual(nearRotation, Fixed64.FromDouble(0.01)));
        Assert.False(FixedQuaternion.Identity.FuzzyEqualAbsolute(FixedQuaternion.Zero, Fixed64.FromDouble(0.1)));
        Assert.False(FixedQuaternion.Identity.FuzzyEqual(FixedQuaternion.Zero, Fixed64.FromDouble(0.1)));
    }

    [Fact]
    public void FixedQuaternion_ReceiverShapedExtensions_MatchStaticImplementations()
    {
        var start = FixedQuaternion.Identity;
        var end = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);

        Assert.True(start.Lerp(end, Fixed64.Half).FuzzyEqual(FixedQuaternion.Lerp(start, end, Fixed64.Half), Fixed64.FromDouble(0.0001)));
        Assert.True(start.Slerp(end, Fixed64.Half).FuzzyEqual(FixedQuaternion.Slerp(start, end, Fixed64.Half), Fixed64.FromDouble(0.0001)));
        Assert.Equal(FixedQuaternion.Angle(start, end), start.Angle(end));
        Assert.Equal(FixedQuaternion.Dot(start, end), start.Dot(end));
        Assert.Equal(FixedQuaternion.QuaternionLog(end), end.QuaternionLog());
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void FixedQuaternion_FuzzyEqualAbsolute_ReturnsFalse_WhenAnyComponentExceedsTolerance(int componentIndex)
    {
        var baseline = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var changed = OffsetQuaternionComponent(baseline, componentIndex, Fixed64.FromDouble(0.2));

        Assert.False(baseline.FuzzyEqualAbsolute(changed, Fixed64.FromDouble(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void FixedQuaternion_FuzzyEqual_ReturnsFalse_WhenAnyComponentExceedsPercentage(int componentIndex)
    {
        var baseline = new FixedQuaternion(new Fixed64(100), new Fixed64(100), new Fixed64(100), new Fixed64(100));
        var changed = OffsetQuaternionComponent(baseline, componentIndex, new Fixed64(50));

        Assert.False(baseline.FuzzyEqual(changed, Fixed64.FromDouble(0.01)));
    }

    #endregion

    #region Test: Serialization


    [Fact]
    public void FixedQuanternion_NetSerialization_RoundTripMaintainsData()
    {
        var quaternion = FixedQuaternion.Identity;
        var sin = FixedMath.Sin(Fixed64.PiOver4);  // 45° rotation
        var cos = FixedMath.Cos(Fixed64.PiOver4);

        var originalRotation = quaternion.Rotated(sin, cos);

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalRotation, jsonOptions);
        var deserializedRotation = JsonSerializer.Deserialize<FixedQuaternion>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalRotation, deserializedRotation);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void FixedQuanternion_MemoryPackSerialization_RoundTripMaintainsData()
    {
        var quaternion = FixedQuaternion.Identity;
        var sin = FixedMath.Sin(Fixed64.PiOver4);  // 45° rotation
        var cos = FixedMath.Cos(Fixed64.PiOver4);

        FixedQuaternion originalValue = quaternion.Rotated(sin, cos);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedQuaternion deserializedValue = MemoryPackSerializer.Deserialize<FixedQuaternion>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion

    private static void AssertQuaternionLogMatchesIndependentNormalization(FixedQuaternion quaternion)
    {
        Vector3d actual = default;
        Exception? exception = Record.Exception(() => actual = FixedQuaternion.QuaternionLog(quaternion));

        Assert.Null(exception);
        Vector3d expected = FixedQuaternion.QuaternionLog(NormalizeQuaternionOracle(quaternion));
        Assert.True(
            actual.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)),
            $"QuaternionLog returned {actual} for {quaternion}; expected {expected}.");
    }

    private static Fixed64 GetMagnitudeOracle(FixedQuaternion quaternion)
    {
        BigInteger magnitudeRaw = GetRoundedMagnitudeRaw(quaternion);
        return magnitudeRaw > long.MaxValue
            ? Fixed64.MaxValue
            : Fixed64.FromRaw((long)magnitudeRaw);
    }

    private static FixedQuaternion CreateDeterministicQuaternion(ref ulong state, bool fullRawDomain)
    {
        return new FixedQuaternion(
            Fixed64.FromRaw(NextDeterministicRaw(ref state, fullRawDomain)),
            Fixed64.FromRaw(NextDeterministicRaw(ref state, fullRawDomain)),
            Fixed64.FromRaw(NextDeterministicRaw(ref state, fullRawDomain)),
            Fixed64.FromRaw(NextDeterministicRaw(ref state, fullRawDomain)));
    }

    private static long NextDeterministicRaw(ref ulong state, bool fullRawDomain)
    {
        ulong bits = NextSplitMix64(ref state);
        if (fullRawDomain)
            return unchecked((long)bits);

        const long tenRaw = 10L << 32;
        const ulong ordinarySpan = (20UL << 32) + 1UL;
        return (long)(bits % ordinarySpan) - tenRaw;
    }

    private static ulong NextSplitMix64(ref ulong state)
    {
        unchecked
        {
            state += 0x9E37_79B9_7F4A_7C15UL;
            ulong value = state;
            value = (value ^ (value >> 30)) * 0xBF58_476D_1CE4_E5B9UL;
            value = (value ^ (value >> 27)) * 0x94D0_49BB_1331_11EBUL;
            return value ^ (value >> 31);
        }
    }

    private static FixedQuaternion NormalizeQuaternionOracle(FixedQuaternion quaternion)
    {
        BigInteger magnitudeRaw = GetRoundedMagnitudeRaw(quaternion);
        if (magnitudeRaw.IsZero)
            return FixedQuaternion.Identity;

        return new FixedQuaternion(
            Fixed64.FromRaw((long)RoundDivideToEven((BigInteger)quaternion.X.m_rawValue << 32, magnitudeRaw)),
            Fixed64.FromRaw((long)RoundDivideToEven((BigInteger)quaternion.Y.m_rawValue << 32, magnitudeRaw)),
            Fixed64.FromRaw((long)RoundDivideToEven((BigInteger)quaternion.Z.m_rawValue << 32, magnitudeRaw)),
            Fixed64.FromRaw((long)RoundDivideToEven((BigInteger)quaternion.W.m_rawValue << 32, magnitudeRaw)));
    }

    private static BigInteger GetRoundedMagnitudeRaw(FixedQuaternion quaternion)
    {
        BigInteger x = quaternion.X.m_rawValue;
        BigInteger y = quaternion.Y.m_rawValue;
        BigInteger z = quaternion.Z.m_rawValue;
        BigInteger w = quaternion.W.m_rawValue;
        BigInteger squareSum = (x * x) + (y * y) + (z * z) + (w * w);

        BigInteger low = BigInteger.Zero;
        BigInteger high = BigInteger.One << 65;
        while (low < high)
        {
            BigInteger midpoint = (low + high + BigInteger.One) >> 1;
            if (midpoint * midpoint <= squareSum)
                low = midpoint;
            else
                high = midpoint - BigInteger.One;
        }

        BigInteger lowerDistance = squareSum - (low * low);
        BigInteger upper = low + BigInteger.One;
        BigInteger upperDistance = (upper * upper) - squareSum;
        return upperDistance < lowerDistance || (upperDistance == lowerDistance && !low.IsEven)
            ? upper
            : low;
    }

    private static BigInteger RoundDivideToEven(BigInteger numerator, BigInteger denominator)
    {
        bool negative = numerator.Sign < 0;
        numerator = BigInteger.Abs(numerator);
        BigInteger quotient = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        BigInteger twiceRemainder = remainder << 1;
        if (twiceRemainder > denominator || (twiceRemainder == denominator && !quotient.IsEven))
            quotient++;

        return negative ? -quotient : quotient;
    }

    private static FixedQuaternion OffsetQuaternionComponent(FixedQuaternion quaternion, int componentIndex, Fixed64 offset)
    {
        switch (componentIndex)
        {
            case 0:
                quaternion.X += offset;
                break;
            case 1:
                quaternion.Y += offset;
                break;
            case 2:
                quaternion.Z += offset;
                break;
            case 3:
                quaternion.W += offset;
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(componentIndex));
        }

        return quaternion;
    }
}
