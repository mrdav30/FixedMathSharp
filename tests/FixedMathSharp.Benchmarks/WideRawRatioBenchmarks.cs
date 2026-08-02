using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class WideRawRatioBenchmarks
{
    private readonly Signed576 _oneWordNumerator = Positive(0UL, 123_456_789UL);
    private readonly Signed576 _oneWordDenominator = Positive(0UL, 97UL);

    private readonly Signed576 _twoWordNumeratorFor32Bit =
        Positive(1UL, 0x1234_5678_9ABC_DEF0UL);
    private readonly Signed576 _denominator32Bit = Positive(0UL, uint.MaxValue);

    private readonly Signed576 _twoWordNumeratorFor64Bit =
        Positive(0x1000_0000_0000_0000UL, 0x1234_5678_9ABC_DEF0UL);
    private readonly Signed576 _denominator64Bit =
        Positive(0UL, 0xF000_0000_0000_0001UL);

    private readonly Signed576 _unrepresentableNumerator = Positive(1UL, 0UL);
    private readonly Signed576 _unitDenominator = Positive(0UL, 1UL);

    private readonly Signed576 _multiWordNumerator =
        Positive(3UL, 0x1234_5678_9ABC_DEF0UL);
    private readonly Signed576 _multiWordDenominator =
        Positive(1UL, 0xFEDC_BA98_7654_3210UL);

    [Benchmark]
    public bool OneWordNumeratorAndDenominator() =>
        Fixed64.TryGetSignedRawRatio(
            _oneWordNumerator,
            _oneWordDenominator,
            out _);

    [Benchmark]
    public bool TwoWordNumeratorAnd32BitDenominator() =>
        Fixed64.TryGetSignedRawRatio(
            _twoWordNumeratorFor32Bit,
            _denominator32Bit,
            out _);

    [Benchmark]
    public bool TwoWordNumeratorAnd64BitDenominator() =>
        Fixed64.TryGetSignedRawRatio(
            _twoWordNumeratorFor64Bit,
            _denominator64Bit,
            out _);

    [Benchmark]
    public bool UnrepresentableQuotient() =>
        Fixed64.TryGetSignedRawRatio(
            _unrepresentableNumerator,
            _unitDenominator,
            out _);

    [Benchmark(Baseline = true)]
    public bool MultiWordDenominatorControl() =>
        Fixed64.TryGetSignedRawRatio(
            _multiWordNumerator,
            _multiWordDenominator,
            out _);

    private static Signed576 Positive(ulong high, ulong low) =>
        new(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, high, low);
}
