using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Vector4dBenchmarks
{
    private readonly Vector4d[] _left = BenchmarkFixtures.Vector4sA;
    private readonly Vector4d[] _right = BenchmarkFixtures.Vector4sB;

    [Benchmark]
    public Vector4d Add()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] + _right[i];

        return accumulator;
    }

    [Benchmark]
    public Vector4d AddOut()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d.Add(_left[i], _right[i], out Vector4d result);
            accumulator += result;
        }

        return accumulator;
    }

    [Benchmark]
    public Vector4d AddScale()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] + _right[i]) * Fixed64.Half;

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Dot()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector4d.Dot(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Magnitude()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i].Magnitude;

        return accumulator;
    }

    [Benchmark]
    public Vector4d Normal()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i].Normal;

        return accumulator;
    }

    [Benchmark]
    public Vector4d NormalizeInPlace()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d value = _left[i];
            accumulator += value.Normalize();
        }

        return accumulator;
    }

    [Benchmark]
    public Vector4d GetNormalized()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector4d.GetNormalized(_left[i]);

        return accumulator;
    }

    [Benchmark]
    public int IsNormalized()
    {
        int count = 0;
        for (int i = 0; i < _left.Length; i++)
        {
            if (_left[i].IsNormalized())
                count++;
        }

        return count;
    }

    [Benchmark]
    public Fixed64 Distance()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector4d.Distance(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector4d Lerp()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector4d.Lerp(_left[i], _right[i], Fixed64.Half);

        return accumulator;
    }
}
