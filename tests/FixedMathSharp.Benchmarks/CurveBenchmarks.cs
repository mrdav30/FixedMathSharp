using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class CurveBenchmarks
{
    private readonly Fixed64[] _times = BenchmarkFixtures.CurveTimes;
    private readonly FixedCurve[] _linearCurves = BenchmarkFixtures.LinearCurves;
    private readonly FixedCurve[] _stepCurves = BenchmarkFixtures.StepCurves;
    private readonly FixedCurve[] _smoothCurves = BenchmarkFixtures.SmoothCurves;
    private readonly FixedCurve[] _cubicCurves = BenchmarkFixtures.CubicCurves;
    private readonly FixedCurve[] _linearTwoKeyCurves = CreateCurves(FixedCurveMode.Linear, 2);
    private readonly FixedCurve[] _linearThirtyTwoKeyCurves = CreateCurves(FixedCurveMode.Linear, 32);

    [Benchmark]
    public Fixed64 LinearEvaluate()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _linearCurves.Length; i++)
            accumulator += _linearCurves[i].Evaluate(_times[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 StepEvaluate()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _stepCurves.Length; i++)
            accumulator += _stepCurves[i].Evaluate(_times[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 SmoothEvaluate()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _smoothCurves.Length; i++)
            accumulator += _smoothCurves[i].Evaluate(_times[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 CubicEvaluate()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _cubicCurves.Length; i++)
            accumulator += _cubicCurves[i].Evaluate(_times[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 LinearEvaluateTwoKeys()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _linearTwoKeyCurves.Length; i++)
            accumulator += _linearTwoKeyCurves[i].Evaluate(Fixed64.Half);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 LinearEvaluateThirtyTwoKeys()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _linearThirtyTwoKeyCurves.Length; i++)
            accumulator += _linearThirtyTwoKeyCurves[i].Evaluate(_times[i]);

        return accumulator;
    }

    private static FixedCurve[] CreateCurves(FixedCurveMode mode, int keyCount)
    {
        var curves = new FixedCurve[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < curves.Length; i++)
            curves[i] = new FixedCurve(mode, CreateKeys(i, keyCount));

        return curves;
    }

    private static FixedCurveKey[] CreateKeys(int seed, int keyCount)
    {
        var keys = new FixedCurveKey[keyCount];
        for (int i = 0; i < keys.Length; i++)
        {
            Fixed64 time = new Fixed64(i);
            Fixed64 value = new Fixed64((seed + i) % 17) + Fixed64.FromFraction(i, keyCount);
            keys[i] = new FixedCurveKey(time, value, Fixed64.Half, Fixed64.Half);
        }

        return keys;
    }
}
