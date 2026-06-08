using MemoryPack;
using System;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

public class Vector4dTests
{
    [Fact]
    public void Constructor_Default_InitializesToZero()
    {
        var vector = new Vector4d();

        Assert.Equal(Fixed64.Zero, vector.X);
        Assert.Equal(Fixed64.Zero, vector.Y);
        Assert.Equal(Fixed64.Zero, vector.Z);
        Assert.Equal(Fixed64.Zero, vector.W);
    }

    [Fact]
    public void Constructor_Overloads_InitializeComponents()
    {
        var fromComponents = new Vector4d(Fixed64.One, new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var fromScalar = new Vector4d(new Fixed64(5));
        var fromVector2 = new Vector4d(new Vector2d(1, 2), new Fixed64(3), new Fixed64(4));
        var fromVector3 = new Vector4d(new Vector3d(1, 2, 3), new Fixed64(4));

        Assert.Equal(new Vector4d(1, 2, 3, 4), fromComponents);
        Assert.Equal(new Vector4d(5, 5, 5, 5), fromScalar);
        Assert.Equal(new Vector4d(1, 2, 3, 4), fromVector2);
        Assert.Equal(new Vector4d(1, 2, 3, 4), fromVector3);
    }

    [Fact]
    public void FromDouble_UsesCheckedFixed64Conversion()
    {
        var vector = Vector4d.FromDouble(1.25, -2.5, 3.75, -4.5);

        Assert.Equal(Fixed64.FromDouble(1.25), vector.X);
        Assert.Equal(Fixed64.FromDouble(-2.5), vector.Y);
        Assert.Equal(Fixed64.FromDouble(3.75), vector.Z);
        Assert.Equal(Fixed64.FromDouble(-4.5), vector.W);

        Assert.Throws<ArgumentOutOfRangeException>(() => Vector4d.FromDouble(0, 0, 0, double.NegativeInfinity));
        Assert.Throws<OverflowException>(() => Vector4d.FromDouble(2147483648d, 0, 0, 0));
    }

    [Fact]
    public void StaticFields_ReturnExpectedValues()
    {
        Assert.Equal(new Vector4d(0, 0, 0, 0), Vector4d.Zero);
        Assert.Equal(new Vector4d(1, 1, 1, 1), Vector4d.One);
        Assert.Equal(new Vector4d(-1, -1, -1, -1), Vector4d.Negative);
        Assert.Equal(new Vector4d(1, 0, 0, 0), Vector4d.UnitX);
        Assert.Equal(new Vector4d(0, 1, 0, 0), Vector4d.UnitY);
        Assert.Equal(new Vector4d(0, 0, 1, 0), Vector4d.UnitZ);
        Assert.Equal(new Vector4d(0, 0, 0, 1), Vector4d.UnitW);
    }

    [Fact]
    public void Indexer_GetAndSet_RoundTripsComponents()
    {
        var vector = new Vector4d(1, 2, 3, 4);

        Assert.Equal(Fixed64.One, vector[0]);
        Assert.Equal(new Fixed64(2), vector[1]);
        Assert.Equal(new Fixed64(3), vector[2]);
        Assert.Equal(new Fixed64(4), vector[3]);

        vector[0] = new Fixed64(5);
        vector[1] = new Fixed64(6);
        vector[2] = new Fixed64(7);
        vector[3] = new Fixed64(8);

        Assert.Equal(new Vector4d(5, 6, 7, 8), vector);
    }

    [Fact]
    public void Indexer_InvalidIndex_Throws()
    {
        var vector = new Vector4d(1, 2, 3, 4);

        Assert.Throws<IndexOutOfRangeException>(() => _ = vector[-1]);
        Assert.Throws<IndexOutOfRangeException>(() => _ = vector[4]);
        Assert.Throws<IndexOutOfRangeException>(() => { vector[-1] = Fixed64.One; });
        Assert.Throws<IndexOutOfRangeException>(() => { vector[4] = Fixed64.One; });
    }

    [Fact]
    public void MagnitudeAndNormalization_WorkCorrectly()
    {
        var vector = new Vector4d(1, 2, 2, 4);

        Assert.Equal(new Fixed64(5), vector.Magnitude);
        Assert.Equal(new Fixed64(25), vector.MagnitudeSquared);

        var normalized = vector.Normalized;

        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, normalized.Magnitude);
    }

    [Fact]
    public void Normalize_ZeroVector_ReturnsZero()
    {
        var vector = Vector4d.Zero;

        Assert.Equal(Vector4d.Zero, vector.NormalizeInPlace());
        Assert.Equal(Vector4d.Zero, Vector4d.GetNormalized(Vector4d.Zero));
    }

    [Fact]
    public void InPlaceHelpers_ModifyVectorCorrectly()
    {
        var vector = new Vector4d(1, 2, 3, 4);

        Assert.Equal(new Vector4d(5, 6, 7, 8), vector.AddInPlace(new Fixed64(4)));
        Assert.Equal(new Vector4d(6, 8, 10, 12), vector.AddInPlace(Fixed64.One, new Fixed64(2), new Fixed64(3), new Fixed64(4)));
        Assert.Equal(new Vector4d(7, 9, 11, 13), vector.AddInPlace(Vector4d.One));
        Assert.Equal(new Vector4d(6, 8, 10, 12), vector.SubtractInPlace(Fixed64.One));
        Assert.Equal(new Vector4d(4, 5, 6, 7), vector.SubtractInPlace(new Fixed64(2), new Fixed64(3), new Fixed64(4), new Fixed64(5)));
        Assert.Equal(Vector4d.One, vector.SubtractInPlace(new Vector4d(3, 4, 5, 6)));
        Assert.Equal(new Vector4d(3, 3, 3, 3), vector.MultiplyInPlace(new Fixed64(3)));
        Assert.Equal(new Vector4d(6, 9, 12, 15), vector.MultiplyInPlace(new Vector4d(2, 3, 4, 5)));

        Assert.Equal(new Vector4d(9, 8, 7, 6), vector.Set(new Fixed64(9), new Fixed64(8), new Fixed64(7), new Fixed64(6)));
    }

    [Fact]
    public void StaticArithmeticHelpers_ReturnExpectedValues()
    {
        var left = new Vector4d(12, 18, 24, 30);
        var right = new Vector4d(3, 6, 8, 10);

        Assert.Equal(new Vector4d(15, 24, 32, 40), Vector4d.Add(left, right));
        Assert.Equal(new Vector4d(9, 12, 16, 20), Vector4d.Subtract(left, right));
        Assert.Equal(new Vector4d(36, 108, 192, 300), Vector4d.Multiply(left, right));
        Assert.Equal(new Vector4d(24, 36, 48, 60), Vector4d.Multiply(left, new Fixed64(2)));
        Assert.Equal(new Vector4d(24, 36, 48, 60), new Fixed64(2) * left);
        Assert.Equal(new Vector4d(4, 3, 3, 3), Vector4d.Divide(left, right));
        Assert.Equal(new Vector4d(6, 9, 12, 15), Vector4d.Divide(left, new Fixed64(2)));
    }

    [Fact]
    public void MultiplyInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector4d(2, 3, 4, 5);

        Assert.Equal(new Vector4d(4, 6, 8, 10), vector.MultiplyInPlace(new Fixed64(2)));
        Assert.Equal(new Vector4d(12, 24, 40, 60), vector.MultiplyInPlace(new Fixed64(3), new Fixed64(4), new Fixed64(5), new Fixed64(6)));
        Assert.Equal(new Vector4d(24, 72, 160, 360), vector.MultiplyInPlace(new Vector4d(2, 3, 4, 6)));
    }

    [Fact]
    public void DivideInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector4d(24, 72, 160, 360);

        Assert.Equal(new Vector4d(12, 36, 80, 180), vector.DivideInPlace(new Fixed64(2)));
        Assert.Equal(new Vector4d(4, 9, 16, 30), vector.DivideInPlace(new Fixed64(3), new Fixed64(4), new Fixed64(5), new Fixed64(6)));
        Assert.Equal(new Vector4d(2, 3, 4, 5), vector.DivideInPlace(new Vector4d(2, 3, 4, 6)));
    }

    [Fact]
    public void InPlaceHelpers_CanChainWhenAssigned()
    {
        var vector = new Vector4d(2, 4, 6, 8);

        vector = vector.AddInPlace(new Vector4d(2, 2, 2, 2))
            .MultiplyInPlace(new Fixed64(3))
            .DivideInPlace(new Vector4d(2, 3, 4, 5))
            .SubtractInPlace(Fixed64.One);

        Assert.Equal(new Vector4d(5, 5, 5, 5), vector);
    }

    [Fact]
    public void StaticHelpers_ReturnExpectedValues()
    {
        var a = new Vector4d(1, 2, 3, 4);
        var b = new Vector4d(5, 6, 7, 8);

        Assert.Equal(new Vector4d(3, 4, 5, 6), Vector4d.Lerp(a, b, Fixed64.Half));
        Assert.Equal(new Vector4d(-3, -2, -1, 0), Vector4d.UnclampedLerp(a, b, -Fixed64.One));
        Assert.Equal(new Fixed64(70), Vector4d.Dot(a, b));
        Assert.Equal(new Fixed64(8), Vector4d.DistanceSquared(a, new Vector4d(3, 4, 3, 4)));
        Assert.Equal(new Fixed64(4), Vector4d.Distance(a, new Vector4d(3, 4, 5, 6)));
        Assert.Equal(new Fixed64(4), a.Distance(new Vector4d(3, 4, 5, 6)));
        Assert.Equal(new Fixed64(4), a.Distance(new Fixed64(3), new Fixed64(4), new Fixed64(5), new Fixed64(6)));
        Assert.Equal(new Fixed64(8), a.DistanceSquared(new Vector4d(3, 4, 3, 4)));
        Assert.Equal(new Fixed64(70), a.Dot(b));
        Assert.Equal(new Vector4d(5, 12, 21, 32), Vector4d.Multiply(a, b));
        Assert.Equal(new Vector4d(1, 2, 3, 4), Vector4d.Min(a, b));
        Assert.Equal(new Vector4d(5, 6, 7, 8), Vector4d.Max(a, b));
        Assert.Equal(new Vector4d(3, 4, 5, 6), Vector4d.Midpoint(a, b));
        Assert.Equal(new Vector4d(1, 2, 3, 4), Vector4d.Abs(new Vector4d(-1, -2, -3, -4)));
        Assert.Equal(new Vector4d(1, -1, 1, -1), Vector4d.Sign(new Vector4d(2, -3, 4, -5)));
        Assert.Equal(new Vector4d(1, 2, 7, 4), Vector4d.Clamp(new Vector4d(-1, 2, 9, 4), a, b));
    }

    [Fact]
    public void Operators_PerformComponentMath()
    {
        var a = new Vector4d(1, 2, 3, 4);
        var b = new Vector4d(5, 6, 7, 8);

        Assert.Equal(new Vector4d(6, 8, 10, 12), a + b);
        Assert.Equal(new Vector4d(4, 4, 4, 4), b - a);
        Assert.Equal(new Vector4d(-1, -2, -3, -4), -a);
        Assert.Equal(new Vector4d(2, 4, 6, 8), a * new Fixed64(2));
        Assert.Equal(new Vector4d(2, 4, 6, 8), new Fixed64(2) * a);
        Assert.Equal(new Vector4d(3, 6, 9, 12), a * 3);
        Assert.Equal(new Vector4d(3, 6, 9, 12), 3 * a);
        Assert.Equal(new Vector4d(5, 12, 21, 32), a * b);
        Assert.Equal(new Vector4d(Fixed64.Half, Fixed64.One, Fixed64.FromDouble(1.5), new Fixed64(2)), a / new Fixed64(2));
        Assert.Equal(new Vector4d(Fixed64.Half, Fixed64.One, Fixed64.FromDouble(1.5), new Fixed64(2)), a / 2);
        Assert.Equal(new Vector4d(new Fixed64(5), new Fixed64(3), new Fixed64(7) / new Fixed64(3), new Fixed64(2)), b / a);
        Assert.Equal(new Vector4d(2, 3, 4, 5), a + Fixed64.One);
        Assert.Equal(new Vector4d(2, 3, 4, 5), Fixed64.One + a);
        Assert.Equal(new Vector4d(4, 6, 8, 10), a + (3, 4, 5, 6));
        Assert.Equal(new Vector4d(4, 6, 8, 10), (3, 4, 5, 6) + a);
        Assert.Equal(new Vector4d(0, 1, 2, 3), a - Fixed64.One);
        Assert.Equal(new Vector4d(9, 8, 7, 6), new Fixed64(10) - a);
        Assert.Equal(new Vector4d(-2, -2, -2, -2), a - (3, 4, 5, 6));
        Assert.Equal(new Vector4d(2, 2, 2, 2), (3, 4, 5, 6) - a);
        Assert.True(a == new Vector4d(1, 2, 3, 4));
        Assert.True(a != b);
    }

    [Fact]
    public void MatrixOperator_FromVectorSide_DelegatesToMatrixTransform()
    {
        var matrix = Fixed4x4.CreateScale(new Vector3d(2, 3, 4));
        var vector = new Vector4d(1, 2, 3, 1);

        Assert.Equal(matrix * vector, vector * matrix);
    }

    [Fact]
    public void PropertiesFormattingEqualityHashingAndComparison_WorkCorrectly()
    {
        var vector = new Vector4d(1, 2, 3, 4);
        var same = new Vector4d(1, 2, 3, 4);
        var larger = new Vector4d(2, 3, 4, 5);
        Vector4d comparer = vector;

        Assert.False(vector.IsZero);
        Assert.True(Vector4d.Zero.IsZero);
        Assert.Equal(vector.LongStateHash, same.LongStateHash);
        Assert.Equal(vector.StateHash, same.StateHash);
        Assert.Equal(vector.GetHashCode(), comparer.GetHashCode(same));
        Assert.True(comparer.Equals(vector, same));
        Assert.True(vector.Equals((object)same));
        Assert.False(vector.Equals((object)new object()));
        Assert.True(vector.CompareTo(larger) < 0);
        Assert.StartsWith("(", vector.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void NormalizeOut_CoversZeroUnitAndNonUnitVectors()
    {
        var vector = new Vector4d(1, 2, 2, 4);

        var normalized = vector.NormalizeInPlace(out var magnitude);

        Assert.Equal(new Fixed64(5), magnitude);
        Assert.Equal(vector, normalized);
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, vector.Magnitude);

        var zero = Vector4d.Zero;
        Assert.Equal(Vector4d.Zero, zero.NormalizeInPlace(out magnitude));
        Assert.Equal(Fixed64.Zero, magnitude);

        var unit = Vector4d.UnitX;
        Assert.Equal(Vector4d.UnitX, unit.NormalizeInPlace(out magnitude));
        Assert.Equal(Fixed64.One, magnitude);
        Assert.Equal(Vector4d.UnitX, Vector4d.GetNormalized(Vector4d.UnitX));
    }

    [Theory]
    [InlineData(1, 2, 2, 4)]
    [InlineData(3, 4, 12, 0)]
    [InlineData(-3, 4, 12, -5)]
    [InlineData(123, 456, 789, 321)]
    public void Normalize_MatchesComponentDivisionByMagnitude(int x, int y, int z, int w)
    {
        var source = new Vector4d(x, y, z, w);

        AssertNormalizeMatchesComponentDivision(source);
    }

    [Fact]
    public void Normalize_MatchesComponentDivisionByMagnitude_ForFractionalHugeAndTinyRawValues()
    {
        AssertNormalizeMatchesComponentDivision(Vector4d.FromDouble(1.5, -2.25, 3.75, -4.5));
        AssertNormalizeMatchesComponentDivision(new Vector4d(10000, -20000, 30000, -10000));
        AssertNormalizeMatchesComponentDivision(new Vector4d(Fixed64.FromRaw(1), Fixed64.FromRaw(-1), Fixed64.FromRaw(2), Fixed64.FromRaw(-2)));
    }

    [Fact]
    public void IsNormalized_ReturnsExpectedValues()
    {
        Assert.True(Vector4d.UnitX.IsNormalized());
        Assert.False(Vector4d.Zero.IsNormalized());
        Assert.False(new Vector4d(2, 0, 0, 0).IsNormalized());
    }

    private static void AssertNormalizeMatchesComponentDivision(Vector4d source)
    {
        Fixed64 magnitude = source.Magnitude;
        if (magnitude == Fixed64.Zero)
        {
            Assert.Equal(Vector4d.Zero, source.Normalized);

            var zeroLength = source;
            Assert.Equal(Vector4d.Zero, zeroLength.NormalizeInPlace());
            return;
        }

        var expected = new Vector4d(source.X / magnitude, source.Y / magnitude, source.Z / magnitude, source.W / magnitude);

        Assert.Equal(expected, source.Normalized);

        var inPlace = source;
        Assert.Equal(expected, inPlace.NormalizeInPlace());
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void AllComponentsGreaterThanEpsilon_ReturnsFalse_WhenAnyComponentIsTooSmall(int componentIndex)
    {
        var vector = new Vector4d(
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon + Fixed64.One);
        vector[componentIndex] = Fixed64.Epsilon;

        Assert.False(vector.AllComponentsGreaterThanEpsilon());
    }

    [Fact]
    public void AllComponentsGreaterThanEpsilon_ReturnsTrue_WhenAllComponentsExceedEpsilon()
    {
        var vector = new Vector4d(
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon + Fixed64.One);

        Assert.True(vector.AllComponentsGreaterThanEpsilon());
    }

    [Fact]
    public void SnapSmallComponentsToZero_CoversDefaultCustomBoundaryAndEachComponentDecision()
    {
        var halfEpsilon = Fixed64.FromRaw(Fixed64.Epsilon.m_rawValue / 2);
        var defaultResult = new Vector4d(halfEpsilon, -halfEpsilon, Fixed64.Epsilon, -Fixed64.Epsilon)
            .SnapSmallComponentsToZero();

        Assert.Equal(Vector4d.Zero, new Vector4d(defaultResult.X, defaultResult.Y, Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(Fixed64.Epsilon, defaultResult.Z);
        Assert.Equal(-Fixed64.Epsilon, defaultResult.W);

        var threshold = Fixed64.FromDouble(0.1);
        var customResult = new Vector4d(Fixed64.FromDouble(0.2), Fixed64.FromDouble(-0.05), Fixed64.FromDouble(-0.1), Fixed64.FromDouble(0.15))
            .SnapSmallComponentsToZero(threshold);

        Assert.Equal(Fixed64.FromDouble(0.2), customResult.X);
        Assert.Equal(Fixed64.Zero, customResult.Y);
        Assert.Equal(Fixed64.FromDouble(-0.1), customResult.Z);
        Assert.Equal(Fixed64.FromDouble(0.15), customResult.W);

        var complement = new Vector4d(
            Fixed64.FromDouble(-0.05),
            Fixed64.FromDouble(0.2),
            Fixed64.FromDouble(0.05),
            Fixed64.FromDouble(-0.05)).SnapSmallComponentsToZero(threshold);

        Assert.Equal(Fixed64.Zero, complement.X);
        Assert.Equal(Fixed64.FromDouble(0.2), complement.Y);
        Assert.Equal(Fixed64.Zero, complement.Z);
        Assert.Equal(Fixed64.Zero, complement.W);
    }

    [Fact]
    public void ExtensionHelpers_ReturnExpectedValues()
    {
        var vector = Vector4d.FromDouble(2, -3, 0.5, -0.25);

        Assert.Equal(Vector4d.FromDouble(1, -1, 0.5, -0.25), vector.ClampOne());
        Assert.Equal(Vector4d.FromDouble(1, -1, 0.5, -0.25), vector.Clamp(Vector4d.Negative, Vector4d.One));
        Assert.Equal(Vector4d.FromDouble(2, 3, 0.5, 0.25), vector.Abs());
        Assert.Equal(new Vector4d(1, -1, 1, -1), vector.Sign());
    }

    [Fact]
    public void ReceiverShapedExtensions_MatchStaticImplementations()
    {
        var start = new Vector4d(1, 2, 3, 4);
        var end = new Vector4d(5, 6, 7, 8);
        var matrix = Fixed4x4.CreateTranslation(new Vector3d(10, 20, 30));

        Assert.Equal(Vector4d.Lerp(start, end, Fixed64.Half), start.Lerp(end, Fixed64.Half));
        Assert.Equal(Vector4d.UnclampedLerp(start, end, new Fixed64(2)), start.UnclampedLerp(end, new Fixed64(2)));
        Assert.Equal(Vector4d.Midpoint(start, end), start.Midpoint(end));
        Assert.Equal(Vector4d.Max(start, end), start.Max(end));
        Assert.Equal(Vector4d.Min(start, end), start.Min(end));
        Assert.Equal(Vector4d.Transform(matrix, start), start.Transform(matrix));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void FuzzyEqualAbsolute_ReturnsFalse_WhenAnyComponentExceedsTolerance(int componentIndex)
    {
        var actual = new Vector4d(1, 2, 3, 4);
        var expected = actual;
        expected[componentIndex] += Fixed64.FromDouble(0.2);

        Assert.False(actual.FuzzyEqualAbsolute(expected, Fixed64.FromDouble(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void FuzzyEqual_ReturnsFalse_WhenAnyComponentExceedsPercentage(int componentIndex)
    {
        var actual = new Vector4d(100, 100, 100, 100);
        var expected = actual;
        expected[componentIndex] = new Fixed64(150);

        Assert.False(actual.FuzzyEqual(expected, Fixed64.FromDouble(0.01)));
    }

    [Fact]
    public void FuzzyEqual_WithExplicitPercentage_ReturnsTrueForCloseComponents()
    {
        var actual = new Vector4d(100, 100, 100, 100);
        var expected = Vector4d.FromDouble(101, 99, 100.5, 99.5);

        Assert.True(actual.FuzzyEqualAbsolute(expected, new Fixed64(1)));
        Assert.True(actual.FuzzyEqual(expected, Fixed64.FromDouble(0.02)));
    }

    [Fact]
    public void ComparisonOperators_ReturnTrueWhenAllComponentsSatisfyComparison()
    {
        var lower = new Vector4d(1, 2, 3, 4);
        var higher = new Vector4d(5, 6, 7, 8);
        var sameAsLower = new Vector4d(1, 2, 3, 4);

        Assert.True(higher > lower);
        Assert.True(lower < higher);
        Assert.True(higher >= lower);
        Assert.True(lower <= higher);
        Assert.True(lower >= sameAsLower);
        Assert.True(lower <= sameAsLower);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void GreaterThanOperators_ReturnFalse_WhenAnyComponentFails(int componentIndex)
    {
        var left = new Vector4d(5, 5, 5, 5);
        var strictRight = new Vector4d(1, 1, 1, 1);
        strictRight[componentIndex] = new Fixed64(5);
        var inclusiveRight = new Vector4d(1, 1, 1, 1);
        inclusiveRight[componentIndex] = new Fixed64(6);

        Assert.False(left > strictRight);
        Assert.False(left >= inclusiveRight);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void LessThanOperators_ReturnFalse_WhenAnyComponentFails(int componentIndex)
    {
        var left = new Vector4d(1, 1, 1, 1);
        var strictRight = new Vector4d(5, 5, 5, 5);
        strictRight[componentIndex] = Fixed64.One;
        var inclusiveRight = new Vector4d(5, 5, 5, 5);
        inclusiveRight[componentIndex] = Fixed64.Zero;

        Assert.False(left < strictRight);
        Assert.False(left <= inclusiveRight);
    }

    [Fact]
    public void Transform_WithTranslationMatrix_UsesHomogeneousW()
    {
        var matrix = Fixed4x4.CreateTranslation(new Fixed64(10), new Fixed64(20), new Fixed64(30));

        Assert.Equal(new Vector4d(11, 22, 33, 1), matrix * new Vector4d(1, 2, 3, 1));
        Assert.Equal(new Vector4d(1, 2, 3, 0), matrix * new Vector4d(1, 2, 3, 0));
    }

    [Fact]
    public void Transform_WithProjectionTerms_ComputesW()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        Assert.Equal(new Vector4d(2, 4, 6, 6), matrix * new Vector4d(2, 4, 6, 1));
    }

    [Fact]
    public void ConversionHelpers_ReturnExpectedValues()
    {
        var vector = new Vector4d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(2.5), Fixed64.FromDouble(3.75), Fixed64.FromDouble(4.5));
        var vector2 = new Vector2d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(2.5));
        var vector3 = new Vector3d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(2.5), Fixed64.FromDouble(3.75));

        vector.Deconstruct(out double fx, out double fy, out double fz, out double fw);

        vector.Deconstruct(out int ix, out int iy, out int iz, out int iw);

        Assert.Equal(new Vector3d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(2.5), Fixed64.FromDouble(3.75)), vector.ToVector3d());
        Assert.Equal(vector, vector2.ToVector4d(Fixed64.FromDouble(3.75), Fixed64.FromDouble(4.5)));
        Assert.Equal(vector, vector3.ToVector4d(Fixed64.FromDouble(4.5)));
        Assert.Equal(1.25f, fx);
        Assert.Equal(2.5f, fy);
        Assert.Equal(3.75f, fz);
        Assert.Equal(4.5f, fw);
        Assert.Equal(1, ix);
        Assert.Equal(2, iy);
        Assert.Equal(4, iz);
        Assert.Equal(4, iw);
    }

    [Fact]
    public void FuzzyEqual_ComparesComponents()
    {
        var vector = new Vector4d(100, 100, 100, 100);
        var near = Vector4d.FromDouble(100.0000008537, 100.0000008537, 100.0000008537, 100.0000008537);
        var far = Vector4d.FromDouble(100.0001, 100.0001, 100.0001, 100.0001);

        Assert.True(vector.FuzzyEqual(near));
        Assert.False(vector.FuzzyEqual(far));
    }

    [Fact]
    public void Vector4d_NetSerialization_RoundTripMaintainsData()
    {
        var originalValue = new Vector4d(Fixed64.Pi, Fixed64.HalfPi, Fixed64.TwoPi, Fixed64.Half);

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<Vector4d>(json, jsonOptions);

        Assert.Equal(originalValue, deserializedValue);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void Vector4d_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Vector4d originalValue = new(Fixed64.Pi, Fixed64.HalfPi, Fixed64.TwoPi, Fixed64.Half);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Vector4d deserializedValue = MemoryPackSerializer.Deserialize<Vector4d>(bytes);

        Assert.Equal(originalValue, deserializedValue);
    }
#endif
}
