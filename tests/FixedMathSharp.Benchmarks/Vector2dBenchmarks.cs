using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Vector2dBenchmarks
{
    private readonly Vector2d[] _left = BenchmarkFixtures.Vector2sA;
    private readonly Vector2d[] _right = BenchmarkFixtures.Vector2sB;
    private static readonly Fixed64 s_checkDistanceThreshold = new Fixed64(16);
    private static readonly Fixed64 s_two = new Fixed64(2);
    private static readonly Vector2d s_extremeCandidate = new(Fixed64.MaxValue, Fixed64.MaxValue);
    private static readonly Vector2d s_extremeCurrent = new(Fixed64.MinValue, Fixed64.MinValue);
    private static readonly Vector2d s_extremeDirection = new(Fixed64.MaxValue, Fixed64.MaxValue);

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
    public Vector2d TryAddSubtractExact()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            if (Vector2d.TryAdd(_left[i], _right[i], out Vector2d sum)
                && Vector2d.TrySubtract(_left[i], _right[i], out Vector2d difference))
            {
                accumulator += sum + difference;
            }
        }

        return accumulator;
    }

    [Benchmark]
    public Vector2d AddSubtractInverseChecked()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d sum = _left[i] + _right[i];
            Vector2d difference = _left[i] - _right[i];
            if (sum - _right[i] == _left[i] && difference + _right[i] == _left[i])
                accumulator += sum + difference;
        }

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
    public int CompareProjectionSaturatingOrdinary()
    {
        int accumulator = 0;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d direction = _left[(i + 37) & (BenchmarkFixtures.SampleCount - 1)];
            accumulator += Vector2d.Dot(_left[i] - _right[i], direction).CompareTo(Fixed64.Zero);
        }

        return accumulator;
    }

    [Benchmark]
    public int CompareProjectionSaturatingExtreme()
    {
        int accumulator = 0;
        for (int i = 0; i < BenchmarkFixtures.SampleCount; i++)
        {
            accumulator += Vector2d.Dot(
                s_extremeCandidate - s_extremeCurrent,
                s_extremeDirection).CompareTo(Fixed64.Zero);
        }

        return accumulator;
    }

    [Benchmark]
    public int CompareProjectionExactOrdinary()
    {
        int accumulator = 0;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector2d direction = _left[(i + 37) & (BenchmarkFixtures.SampleCount - 1)];
            accumulator += Vector2d.CompareProjection(_left[i], _right[i], direction);
        }

        return accumulator;
    }

    [Benchmark]
    public int CompareProjectionExactExtreme()
    {
        int accumulator = 0;
        for (int i = 0; i < BenchmarkFixtures.SampleCount; i++)
        {
            accumulator += Vector2d.CompareProjection(
                s_extremeCandidate,
                s_extremeCurrent,
                s_extremeDirection);
        }

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
