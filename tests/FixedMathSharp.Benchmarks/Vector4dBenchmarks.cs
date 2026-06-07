using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Vector4dBenchmarks
{
    private readonly Vector4d[] _left = BenchmarkFixtures.Vector4sA;
    private readonly Vector4d[] _right = BenchmarkFixtures.Vector4sB;
    private static readonly Fixed64 s_two = new Fixed64(2);

    [Benchmark]
    public Vector4d Add()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] + _right[i];

        return accumulator;
    }

    [Benchmark]
    public Vector4d AddStatic()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector4d.Add(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector4d AddInPlace()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d value = _left[i];
            accumulator += value.AddInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector4d SubtractStatic()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector4d.Subtract(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector4d SubtractInPlace()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d value = _left[i];
            accumulator += value.SubtractInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector4d AddMultiply()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] + _right[i]) * Fixed64.Half;

        return accumulator;
    }

    [Benchmark]
    public Vector4d MultiplyStatic()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector4d.Multiply(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector4d MultiplyInPlace()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d value = _left[i];
            accumulator += value.MultiplyInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector4d DivideStatic()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector4d.Divide(_left[i], s_two);

        return accumulator;
    }

    [Benchmark]
    public Vector4d DivideInPlace()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d value = _left[i];
            accumulator += value.DivideInPlace(s_two);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector4d ChainedReturnByValue()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d scaledRight = Vector4d.Multiply(_right[i], Fixed64.Quarter);
            Vector4d value = Vector4d.Add(_left[i], _right[i]);
            value = Vector4d.Multiply(value, Fixed64.Half);
            value = Vector4d.Divide(value, s_two);
            accumulator += Vector4d.Subtract(value, scaledRight);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector4d ChainedInPlaceAssignment()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d scaledRight = Vector4d.Multiply(_right[i], Fixed64.Quarter);
            Vector4d value = _left[i];
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
    public Vector4d Normalized()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i].Normalized;

        return accumulator;
    }

    [Benchmark]
    public Vector4d NormalizeInPlace()
    {
        Vector4d accumulator = Vector4d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector4d value = _left[i];
            accumulator += value.NormalizeInPlace();
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
