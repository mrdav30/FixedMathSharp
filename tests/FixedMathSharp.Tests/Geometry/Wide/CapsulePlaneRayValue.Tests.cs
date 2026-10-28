using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CapsulePlaneRayValueTests
{
    [Theory]
    [InlineData(-1, 0)]
    [InlineData(0, 0)]
    [InlineData(1, 0)]
    [InlineData(-1, 7)]
    [InlineData(0, 7)]
    [InlineData(1, 7)]
    public void SquaredQuarticValueComparesAnIndependentFieldWithoutRounding(int delta, int shift)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic a = Field(values, signs, 0), b = Field(values, signs, 1), c = Field(values, signs, 2);
        Set(a, 4); Set(b, 0); Set(c, -1, -2);
        Span<ulong> pointRoot = stackalloc ulong[1] { 2 }, thresholdRoot = stackalloc ulong[1] { 8 };
        Assert.True(value.TrySetUpperRoot(a, b, c, pointRoot, shift));
        // t²=(1+2sqrt(2))/4. The independently represented threshold is
        // (256+256sqrt(8)+delta)/1024, including the exact algebraic tie.
        Set(a, 256 + delta, 256); Set(b, 1024);
        Assert.Equal(-delta, value.CompareSquaredQuadratic(a, b, thresholdRoot));
    }

    private const int Words = 10;
    private static ContactQuadratic Field(Span<ulong> values, Span<int> signs, int index) => ContactQuadratic.At(values, signs, index, Words);
    private static Signed832 Wide(long value) => Signed832.ExtendValue(Signed320.ExtendValue(Signed192.Signed(value)));

    private static void Set(ContactQuadratic value, long rational, long radical = 0)
    {
        value.Set(Wide(rational));
        value.Radical[0] = radical < 0 ? unchecked((ulong)-radical) : (ulong)radical;
        value.Signs[1] = Math.Sign(radical);
    }

    private static void SetLinear(ref CapsulePlaneRayValue value, int twiceValue, int shift)
    {
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic a = Field(values, signs, 0), b = Field(values, signs, 1), c = Field(values, signs, 2);
        Set(a, 0); Set(b, 1); Set(c, -twiceValue);
        Assert.True(value.TrySetUpperRoot(a, b, c, ReadOnlySpan<ulong>.Empty, shift));
    }

    [Theory]
    [InlineData(0, 2, 3, 4, 6, 0)]
    [InlineData(2, 1, 7, 3, 21, 1)]
    [InlineData(0, 3, 3, 2, 1, -1)]
    public void DividedValues_ShouldComparePhysicalValuesAcrossBinaryAndNonbinaryScales(
        int firstShift, int secondShift, int firstDivisor, int secondTwiceValue, int secondDivisor, int expected)
    {
        Span<ulong> firstStorage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> firstSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        Span<ulong> secondStorage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> secondSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var first = new CapsulePlaneRayValue(firstStorage, firstSigns);
        var second = new CapsulePlaneRayValue(secondStorage, secondSigns);
        SetLinear(ref first, 2, firstShift);
        SetLinear(ref second, secondTwiceValue, secondShift);
        Assert.Equal(expected, first.Compare(second, Signed320.ExtendValue(Signed192.Signed(firstDivisor)), Signed320.ExtendValue(Signed192.Signed(secondDivisor))));
        Assert.Equal(-expected, second.Compare(first, Signed320.ExtendValue(Signed192.Signed(secondDivisor)), Signed320.ExtendValue(Signed192.Signed(firstDivisor))));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void UpperRoot_ShouldRejectConjugateAndInwardIntersections(int radicalSign)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic a = Field(values, signs, 0), b = Field(values, signs, 1), c = Field(values, signs, 2);
        Set(a, 1); Set(b, 0, radicalSign); Set(c, radicalSign > 0 ? -1 : 1);
        Span<ulong> root = stackalloc ulong[1] { 2 };
        Assert.True(value.TrySetUpperRoot(a, b, c, root, 2));
        Assert.True(value.HasValue);
        Signed576 metric = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(Fixed64.One), Signed192.Raw(Fixed64.One)));
        Assert.True(value.TryGetDepth(Signed832.ExtendValue(metric), Signed320.ExtendValue(Signed192.Signed(1)), out Fixed64 depth, out bool overflow));
        Assert.False(overflow);
        Fixed64 expected = radicalSign > 0 ? FixedMath.Sqrt((Fixed64)3) - FixedMath.Sqrt(Fixed64.Two)
            : FixedMath.Sqrt(Fixed64.Two) + Fixed64.One;
        Assert.InRange(FixedMath.Abs(depth - expected).m_rawValue, 0L, 2L);
        // Compare independent known depths instead of materializing nested roots:
        // positive b gives sqrt(3)-sqrt(2), negative b gives sqrt(2)+1.
        Set(a, radicalSign > 0 ? 0 : 2); Set(b, 1);
        Assert.Equal(1, value.CompareQuadratic(a, b, ReadOnlySpan<ulong>.Empty));
        Set(a, radicalSign > 0 ? 1 : 3);
        Assert.Equal(-1, value.CompareQuadratic(a, b, ReadOnlySpan<ulong>.Empty));
    }

    [Fact]
    public void RationalRay_WithIndependentQuadraticPoint_ShouldKeepBothBoundaryFieldsExact()
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic a = Field(values, signs, 0), b = Field(values, signs, 1), c = Field(values, signs, 2);
        Span<ulong> pointRoot = stackalloc ulong[1] { 2 };
        // The point field contains sqrt(2), but its rational ray equation
        // t^2-3=0 has the independent positive boundary depth sqrt(3).
        Set(a, 1); Set(b, 0); Set(c, -3);
        Assert.True(value.TrySetUpperRoot(a, b, c, pointRoot, 1));
        Set(a, 0, 1); Set(b, 1);
        Assert.Equal(1, value.CompareQuadratic(a, b, pointRoot));
        Signed192 scale = Signed192.Raw(Fixed64.One);
        Signed576 metric = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(scale, scale));
        Assert.True(value.TryGetDepth(Signed832.ExtendValue(metric), Signed320.ExtendValue(Signed192.Signed(1)), out Fixed64 depth, out bool overflow));
        Assert.False(overflow);
        Assert.InRange(FixedMath.Abs(depth - FixedMath.Sqrt((Fixed64)3)).m_rawValue, 0L, 1L);

        ContactQuadratic x = a, y = b, z = c, d = Field(values, signs, 3);
        Set(x, 0, Fixed64.One.m_rawValue); Set(y, 0); Set(z, 0); Set(d, 1);
        var direction = new WideDirection3d(new WideAxis3(default, Signed320.ExtendValue(scale), default));
        Assert.True(value.TryGetPoint(x, y, z, d, pointRoot, direction, Signed320.ExtendValue(Signed192.Signed(1)), out Vector3d point));
        Assert.InRange(FixedMath.Abs(point.X - FixedMath.Sqrt(Fixed64.Two)).m_rawValue, 0L, 1L);
        Assert.Equal(depth, point.Y);
        Assert.Equal(Fixed64.Zero, point.Z);
    }

    [Fact]
    public void UpperRoot_WhenBothFieldPolynomialsVanish_ShouldKeepIndependentFieldSigns()
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic a = Field(values, signs, 0), b = Field(values, signs, 1), c = Field(values, signs, 2);
        // Twice this polynomial is 2(t-1)^2+2sqrt(2)(t-1).
        Set(a, 2); Set(b, -2, 1); Set(c, 2, -2);
        Span<ulong> root = stackalloc ulong[1] { 2 };
        Assert.True(value.TrySetUpperRoot(a, b, c, root, 2));
        Set(a, -1); Set(b, 1);
        Assert.Equal(0, value.SignLinear(a, b, ReadOnlySpan<ulong>.Empty));
        Set(a, 0, -1); root[0] = 3;
        Assert.Equal(-1, value.SignLinear(a, b, root));
        Set(a, 0, 1);
        Assert.Equal(1, value.SignLinear(a, b, root));
    }

    [Theory]
    [InlineData(-2, 3, 3)]
    [InlineData(-1, 1, 1)]
    public void UpperRoot_WithAZeroPointRadicand_ShouldAdmitTheActualRationalExit(
        int halfLinear, int constant, int expected)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic a = Field(values, signs, 0), b = Field(values, signs, 1), c = Field(values, signs, 2);
        Set(a, 1, 2); Set(b, halfLinear, -7); Set(c, constant, 5);
        Span<ulong> pointRoot = stackalloc ulong[1] { 0 };
        // Radical coefficients remain nonzero, but sqrt(0) contributes nothing.
        // The actual equations are (t-1)(t-3) and (t-1)^2, respectively:
        // retain the outward exit at 3 or the exact tangent at 1.
        Assert.True(value.TrySetUpperRoot(a, b, c, pointRoot, 2));
        Set(a, expected); Set(b, 1);
        Assert.Equal(0, value.CompareQuadratic(a, b, ReadOnlySpan<ulong>.Empty));
        Signed192 scale = Signed192.Raw(Fixed64.One);
        Signed576 metric = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(scale, scale));
        Assert.True(value.TryGetDepth(Signed832.ExtendValue(metric), Signed320.ExtendValue(Signed192.Signed(1)), out Fixed64 depth, out bool overflow));
        Assert.False(overflow);
        Assert.Equal((Fixed64)expected, depth);
    }

    [Fact]
    public void UpperRoot_WithNoRealPositiveExit_ShouldReturnNoValue()
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic a = Field(values, signs, 0), b = Field(values, signs, 1), c = Field(values, signs, 2);
        Set(a, 1); Set(b, 0); Set(c, 1);
        Assert.False(value.TrySetUpperRoot(a, b, c, ReadOnlySpan<ulong>.Empty, 0));
        Assert.False(value.HasValue);
    }

    [Fact]
    public void UpperRoot_ShouldAdmitAnExactTangentBoundary()
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic a = Field(values, signs, 0), b = Field(values, signs, 1), c = Field(values, signs, 2);
        Set(a, 1); Set(b, -1); Set(c, 1);
        Assert.True(value.TrySetUpperRoot(a, b, c, ReadOnlySpan<ulong>.Empty, 0));
        Set(a, 1); Set(b, 1);
        Assert.Equal(0, value.CompareQuadratic(a, b, ReadOnlySpan<ulong>.Empty));
    }

    [Fact]
    public void CloselySeparatedExitRoots_ShouldRestoreTheExactOrdinalBeyondCompactStorage()
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        const int words = 20;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic a = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic b = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic c = ContactQuadratic.At(values, signs, 2, words);
        a.Clear(); b.Clear(); c.Clear();
        a.Rational[18] = 1UL << 48; a.Signs[0] = 1;
        b.Rational[18] = 1UL << 47; b.Signs[0] = -1;
        c.Rational[..18].Fill(ulong.MaxValue); c.Rational[18] = (1UL << 46) - 1; c.Signs[0] = 1;
        // The upper outward exit is 1/2+2^-600. A rounded root would lose
        // its side of the exact half-raw depth threshold.
        Assert.True(value.TrySetUpperRoot(a, b, c, ReadOnlySpan<ulong>.Empty, 0));
        Assert.False(value.Metadata.HasCompactCell);
        Set(a, 1); Set(b, 2);
        Assert.Equal(1, value.CompareQuadratic(a, b, ReadOnlySpan<ulong>.Empty));
        Assert.True(value.TryGetDepth(Wide(1), Signed320.ExtendValue(Signed192.Signed(1)), out Fixed64 depth, out bool overflow));
        Assert.False(overflow); Assert.Equal(1L, depth.m_rawValue);
        Set(a, 0); Set(b, 1); Set(c, 0);
        Assert.True(value.TryGetPoint(a, a, c, b, ReadOnlySpan<ulong>.Empty,
            new WideDirection3d(new WideAxis3(default, Signed320.ExtendValue(Signed192.Signed(1)), default)),
            Signed320.ExtendValue(Signed192.Signed(1)), out Vector3d point));
        Assert.Equal(Vector3d.Up * Fixed64.FromRaw(1), point);

        // The same exact exit approaches both final coordinate limits from
        // inside: Max+1-t=Max+1/2-2^-600 and Min-1+t=Min-1/2+2^-600.
        // Range admission must retain the deep cell before coordinate rounding;
        // losing that tiny side of either threshold would reject a valid point.
        Signed192 upperStart = WideArithmetic.AddSigned192(Signed192.Signed(long.MaxValue), Signed192.Signed(1));
        Signed192 lowerStart = WideArithmetic.SubtractSigned192(Signed192.Signed(long.MinValue), Signed192.Signed(1));
        a.Set(Signed576.ExtendValue(Signed320.ExtendValue(upperStart)));
        b.Set(Signed576.ExtendValue(Signed320.ExtendValue(lowerStart)));
        ContactQuadratic d = ContactQuadratic.At(values, signs, 3, words); Set(d, 1);
        Assert.True(value.TryGetPoint(a, b, c, d, ReadOnlySpan<ulong>.Empty,
            new WideDirection3d(new WideAxis3(Signed320.ExtendValue(Signed192.Signed(-1)), Signed320.ExtendValue(Signed192.Signed(1)), default)),
            Signed320.ExtendValue(Signed192.Signed(1)), out point));
        Assert.Equal(new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.Zero), point);
    }

    [Fact]
    public void RootComparison_ShouldPreserveEqualityAcrossStorageAndDifferentNormalizations()
    {
        Span<ulong> firstStorage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<ulong> secondStorage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> firstSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount], secondSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var first = new CapsulePlaneRayValue(firstStorage, firstSigns);
        var second = new CapsulePlaneRayValue(secondStorage, secondSigns);
        SetLinear(ref first, 2, 0); SetLinear(ref second, 2, 2);
        Assert.Equal(0, first.Compare(second));
        SetLinear(ref second, 3, 2);
        Assert.Equal(-1, first.Compare(second)); Assert.Equal(1, second.Compare(first));
        second.CopyFrom(first);
        CapsulePlaneRayValueMetadata saved = second.Metadata;
        second.Restore(default); Assert.False(second.HasValue);
        second.Restore(saved); Assert.Equal(0, first.Compare(second));
    }

    [Theory]
    [InlineData(1, 0)]
    [InlineData(3, 2)]
    public void DepthRounding_ShouldUseNearestEven(int twiceValue, long expectedRaw)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        SetLinear(ref value, twiceValue, 2);
        Assert.True(value.TryGetDepth(Wide(1), Signed320.ExtendValue(Signed192.Signed(1)), out Fixed64 depth, out bool overflow));
        Assert.False(overflow); Assert.Equal(expectedRaw, depth.m_rawValue);
    }

    [Theory]
    [InlineData(2, false)]
    [InlineData(4, true)]
    public void DepthAtTheScalarLimit_ShouldDistinguishEqualityFromOverflow(int twiceValue, bool expectedOverflow)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns); SetLinear(ref value, twiceValue, 2);
        Signed576 metric = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(Fixed64.MaxValue), Signed192.Raw(Fixed64.MaxValue)));
        Assert.True(value.TryGetDepth(Signed832.ExtendValue(metric), Signed320.ExtendValue(Signed192.Signed(1)), out Fixed64 depth, out bool overflow));
        Assert.Equal(Fixed64.MaxValue, depth); Assert.Equal(expectedOverflow, overflow);
    }

    [Fact]
    public void PointMaterialization_ShouldRetainTheAffineStartUntilFinalRounding()
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns); SetLinear(ref value, 3, 2);
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        Set(x, 0); Set(y, 2); Set(z, 0); Set(d, 1);
        var direction = new WideDirection3d(new WideAxis3(default, Signed320.ExtendValue(Signed192.Raw(Fixed64.One)), default));
        Assert.True(value.TryGetPoint(x, y, z, d, ReadOnlySpan<ulong>.Empty, direction, Signed320.ExtendValue(Signed192.Signed(1)), out Vector3d point));
        Assert.Equal(Vector3d.Up * (Fixed64.FromFraction(3, 2) + Fixed64.FromRaw(2)), point);
    }

    [Theory]
    [InlineData(1, 0, 0)]
    [InlineData(3, 0, 2)]
    [InlineData(-1, 0, 0)]
    [InlineData(-3, 0, -2)]
    [InlineData(1, 1, 2)]
    [InlineData(1, -1, 0)]
    public void PointMaterialization_HalfRawTies_ShouldUseTheFinalLocalLattice(
        int numerator, int addedRaw, long expectedRaw)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns); SetLinear(ref value, 2, 0);
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        Set(x, 0); Set(y, numerator + 2 * addedRaw); Set(z, 0); Set(d, 1);
        Assert.True(value.TryGetPoint(x, y, z, d, ReadOnlySpan<ulong>.Empty, default, Signed320.ExtendValue(Signed192.Signed(2)), out Vector3d point));
        Assert.Equal(Vector3d.Up * Fixed64.FromRaw(expectedRaw), point);
    }

    [Theory]
    [InlineData(1, 1, 0)]
    [InlineData(3, 1, 2)]
    [InlineData(1, -1, 0)]
    [InlineData(3, -1, -2)]
    public void PointMaterialization_WithAnActiveRayAtHalfRaw_ShouldUseNearestEven(
        int twiceValue, int directionSign, long expectedRaw)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns); SetLinear(ref value, twiceValue, 2);
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        Set(x, 0); Set(y, 0); Set(z, 0); Set(d, 1);
        var direction = new WideDirection3d(new WideAxis3(default, Signed320.ExtendValue(Signed192.Signed(directionSign)), default));
        Assert.True(value.TryGetPoint(x, y, z, d, ReadOnlySpan<ulong>.Empty, direction,
            Signed320.ExtendValue(Signed192.Signed(1)), out Vector3d point));
        Assert.Equal(Vector3d.Up * Fixed64.FromRaw(expectedRaw), point);
    }

    [Theory]
    [InlineData(1, 4)]
    [InlineData(-1, 3)]
    public void PointMaterialization_WithACancelledPositiveDenominator_ShouldKeepExactRounding(
        int directionSign, long expectedRaw)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns); SetLinear(ref value, 1, 2);
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        Set(x, 0); Set(y, 1); Set(z, 0); Set(d, 2, -1);
        Span<ulong> root = stackalloc ulong[1] { 3 };
        // D=2-sqrt(3)>0 although its coarse interval includes zero.
        // 1/D=2+sqrt(3), so adding +/-1/2 rounds to raw 4 or 3.
        var direction = new WideDirection3d(new WideAxis3(default, Signed320.ExtendValue(Signed192.Signed(directionSign)), default));
        Assert.True(value.TryGetPoint(x, y, z, d, root, direction, Signed320.ExtendValue(Signed192.Signed(1)), out Vector3d point));
        Assert.Equal(Vector3d.Up * Fixed64.FromRaw(expectedRaw), point);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void PointMaterialization_WithIndependentRadicals_ShouldRetainTheAffineExit(int directionSign)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns);
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        Set(x, 1); Set(y, 0); Set(z, -2);
        Assert.True(value.TrySetUpperRoot(x, y, z, ReadOnlySpan<ulong>.Empty, 1));
        Set(x, 0); Set(y, 0, Fixed64.One.m_rawValue); Set(z, 0); Set(d, 1);
        Span<ulong> root = stackalloc ulong[1] { 3 };
        var direction = new WideDirection3d(new WideAxis3(default,
            Signed320.ExtendValue(Signed192.Raw(Fixed64.One * directionSign)), default));
        Assert.True(value.TryGetPoint(x, y, z, d, root, direction, Signed320.ExtendValue(Signed192.Signed(1)), out Vector3d point));
        Fixed64 expected = FixedMath.Sqrt((Fixed64)3) + FixedMath.Sqrt(Fixed64.Two) * directionSign;
        Assert.InRange(FixedMath.Abs(point.Y - expected).m_rawValue, 0L, 1L);
        Assert.Equal(Fixed64.Zero, point.X); Assert.Equal(Fixed64.Zero, point.Z);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 1)]
    [InlineData(2, 1)]
    [InlineData(0, -1)]
    [InlineData(1, -1)]
    [InlineData(2, -1)]
    public void PointMaterialization_OutsideAnyCoordinateDomain_ShouldFail(int axis, int sign)
    {
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var value = new CapsulePlaneRayValue(storage, valueSigns); SetLinear(ref value, 4, 2);
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        Set(x, 0); Set(y, 0); Set(z, 0); Set(d, 1);
        Set(axis == 0 ? x : axis == 1 ? y : z, sign > 0 ? long.MaxValue : long.MinValue);
        Signed320 raw = Signed320.ExtendValue(Signed192.Raw(Fixed64.One * sign));
        var direction = new WideDirection3d(new WideAxis3(axis == 0 ? raw : default, axis == 1 ? raw : default, axis == 2 ? raw : default));
        Assert.False(value.TryGetPoint(x, y, z, d, ReadOnlySpan<ulong>.Empty, direction, Signed320.ExtendValue(Signed192.Signed(1)), out Vector3d point));
        Assert.Equal(Vector3d.Zero, point);
    }
}
