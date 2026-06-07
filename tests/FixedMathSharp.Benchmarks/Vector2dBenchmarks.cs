using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Vector2dBenchmarks
{
    private readonly Vector2d[] _left = BenchmarkFixtures.Vector2sA;
    private readonly Vector2d[] _right = BenchmarkFixtures.Vector2sB;
    private static readonly Fixed64 s_checkDistanceThreshold = new Fixed64(16);
    private static readonly Fixed64 s_two = new Fixed64(2);

    [Benchmark]
    public Vector2d Add()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] + _right[i];

        return accumulator;
    }

    [Benchmark]
    public Vector2d AddStatic()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.Add(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector2d AddInPlace()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d value = _left[i];
            accumulator += value.AddInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector2d SubtractStatic()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.Subtract(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector2d SubtractInPlace()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d value = _left[i];
            accumulator += value.SubtractInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector2d AddMultiply()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] + _right[i]) * Fixed64.Half;

        return accumulator;
    }

    [Benchmark]
    public Vector2d MultiplyStatic()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.Multiply(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector2d MultiplyInPlace()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d value = _left[i];
            accumulator += value.MultiplyInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector2d DivideStatic()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.Divide(_left[i], s_two);

        return accumulator;
    }

    [Benchmark]
    public Vector2d DivideInPlace()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d value = _left[i];
            accumulator += value.DivideInPlace(s_two);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector2d ChainedReturnByValue()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d scaledRight = Vector2d.Multiply(_right[i], Fixed64.Quarter);
            Vector2d value = Vector2d.Add(_left[i], _right[i]);
            value = Vector2d.Multiply(value, Fixed64.Half);
            value = Vector2d.Divide(value, s_two);
            accumulator += Vector2d.Subtract(value, scaledRight);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector2d ChainedInPlaceAssignment()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d scaledRight = Vector2d.Multiply(_right[i], Fixed64.Quarter);
            Vector2d value = _left[i];
            value = value.AddInPlace(_right[i])
                .MultiplyInPlace(Fixed64.Half)
                .DivideInPlace(s_two)
                .SubtractInPlace(scaledRight);
            accumulator += value;
        }

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
    public Vector2d Normalized()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i].Normalized;

        return accumulator;
    }

    [Benchmark]
    public Vector2d NormalizeInPlace()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d value = _left[i];
            accumulator += value.NormalizeInPlace();
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
    public int DistanceThreshold()
    {
        int count = 0;
        for (int i = 0; i < _left.Length; i++)
        {
            if (Vector2d.Distance(_left[i], _right[i]) <= s_checkDistanceThreshold)
                count++;
        }

        return count;
    }

    [Benchmark]
    public int CheckDistance()
    {
        int count = 0;
        for (int i = 0; i < _left.Length; i++)
        {
            if (_left[i].CheckDistance(_right[i], s_checkDistanceThreshold))
                count++;
        }

        return count;
    }

    [Benchmark]
    public Vector2d Lerp()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector2d.Lerp(_left[i], _right[i], Fixed64.Half);

        return accumulator;
    }
}
