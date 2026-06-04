using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Vector2dBenchmarks
{
    private readonly Vector2d[] _left = BenchmarkFixtures.Vector2sA;
    private readonly Vector2d[] _right = BenchmarkFixtures.Vector2sB;

    [Benchmark]
    public Vector2d Add()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] + _right[i];

        return accumulator;
    }

    [Benchmark]
    public Vector2d AddOut()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d.Add(_left[i], _right[i], out Vector2d result);
            accumulator += result;
        }

        return accumulator;
    }

    [Benchmark]
    public Vector2d AddScale()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] + _right[i]) * Fixed64.Half;

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Dot()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.Dot(_left[i], _right[i]);

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
    public Vector2d Normal()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i].Normal;

        return accumulator;
    }

    [Benchmark]
    public Vector2d NormalizeInPlace()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d value = _left[i];
            accumulator += value.Normalize();
        }

        return accumulator;
    }

    [Benchmark]
    public Vector2d GetNormalized()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.GetNormalized(_left[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Distance()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.Distance(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector2d Lerp()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.Lerp(_left[i], _right[i], Fixed64.Half);

        return accumulator;
    }

    [Benchmark]
    public Vector2d LerpOut()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d.Lerp(_left[i], _right[i], Fixed64.Half, out Vector2d result);
            accumulator += result;
        }

        return accumulator;
    }
}
