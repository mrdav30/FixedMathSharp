using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Fixed64ArithmeticBenchmarks
{
    private readonly Fixed64[] _left = BenchmarkFixtures.ScalarsA;
    private readonly Fixed64[] _right = BenchmarkFixtures.ScalarsB;
    private readonly Fixed64[] _positive = BenchmarkFixtures.PositiveScalars;
    private readonly Fixed64[] _angles = BenchmarkFixtures.Angles;

    [Benchmark]
    public Fixed64 AddSubtractMultiply()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] + _right[i]) * Fixed64.Half - _right[i];

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Divide()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] / _positive[i];

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Sqrt()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _positive.Length; i++)
            accumulator += FixedMath.Sqrt(_positive[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 SinCos()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _angles.Length; i++)
            accumulator += FixedMath.Sin(_angles[i]) + FixedMath.Cos(_angles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Atan2()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += FixedMath.Atan2(_left[i], _right[i]);

        return accumulator;
    }
}
