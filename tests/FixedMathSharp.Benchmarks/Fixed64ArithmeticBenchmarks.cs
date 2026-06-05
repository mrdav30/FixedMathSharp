using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Fixed64ArithmeticBenchmarks
{
    private readonly Fixed64[] _left = BenchmarkFixtures.ScalarsA;
    private readonly Fixed64[] _right = BenchmarkFixtures.ScalarsB;
    private readonly Fixed64[] _positive = BenchmarkFixtures.PositiveScalars;
    private readonly Fixed64[] _unit = BenchmarkFixtures.UnitScalars;
    private readonly Fixed64[] _angles = BenchmarkFixtures.Angles;
    private readonly Fixed64[] _tangentAngles = BenchmarkFixtures.TangentAngles;

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
    public Fixed64 Sin()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _angles.Length; i++)
            accumulator += FixedMath.Sin(_angles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Cos()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _angles.Length; i++)
            accumulator += FixedMath.Cos(_angles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Tan()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _tangentAngles.Length; i++)
            accumulator += FixedMath.Tan(_tangentAngles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Acos()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _unit.Length; i++)
            accumulator += FixedMath.Acos(_unit[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Asin()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _unit.Length; i++)
            accumulator += FixedMath.Asin(_unit[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Atan()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += FixedMath.Atan(_left[i]);

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

    [Benchmark]
    public Fixed64 Pow()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _positive.Length; i++)
            accumulator += FixedMath.Pow(_positive[i], _unit[i] * Fixed64.Half);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Pow2()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _unit.Length; i++)
            accumulator += FixedMath.Pow2(_unit[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Log2()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _positive.Length; i++)
            accumulator += FixedMath.Log2(_positive[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Ln()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _positive.Length; i++)
            accumulator += FixedMath.Ln(_positive[i]);

        return accumulator;
    }
}
