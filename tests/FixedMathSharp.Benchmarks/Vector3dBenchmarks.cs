using BenchmarkDotNet.Attributes;
using FixedMathSharp.Bounds;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Vector3dBenchmarks
{
    private readonly Vector3d[] _left = BenchmarkFixtures.VectorsA;
    private readonly Vector3d[] _right = BenchmarkFixtures.VectorsB;
    private static readonly Fixed64 s_two = new Fixed64(2);

    [Benchmark]
    public Vector3d Add()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] + _right[i];

        return accumulator;
    }

    [Benchmark]
    public Vector3d AddStatic()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.Add(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d AddInPlace()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d value = _left[i];
            accumulator += value.AddInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d SubtractStatic()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.Subtract(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d SubtractInPlace()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d value = _left[i];
            accumulator += value.SubtractInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d AddMultiply()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] + _right[i]) * Fixed64.Half;

        return accumulator;
    }

    [Benchmark]
    public Vector3d MultiplyStatic()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.Multiply(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d MultiplyInPlace()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d value = _left[i];
            accumulator += value.MultiplyInPlace(_right[i]);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d DivideStatic()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.Divide(_left[i], s_two);

        return accumulator;
    }

    [Benchmark]
    public Vector3d DivideInPlace()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d value = _left[i];
            accumulator += value.DivideInPlace(s_two);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d ChainedReturnByValue()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d scaledRight = Vector3d.Multiply(_right[i], Fixed64.Quarter);
            Vector3d value = Vector3d.Add(_left[i], _right[i]);
            value = Vector3d.Multiply(value, Fixed64.Half);
            value = Vector3d.Divide(value, s_two);
            accumulator += Vector3d.Subtract(value, scaledRight);
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d ChainedInPlaceAssignment()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d scaledRight = Vector3d.Multiply(_right[i], Fixed64.Quarter);
            Vector3d value = _left[i];
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
            accumulator += Vector3d.Dot(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d Cross()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.Cross(_left[i], _right[i]);

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
    public Vector3d Normal()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i].Normal;

        return accumulator;
    }

    [Benchmark]
    public Vector3d NormalizeInPlace()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d value = _left[i];
            accumulator += value.Normalize();
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d GetNormalized()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.GetNormalized(_left[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d Project()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.Project(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d ProjectOnPlane()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.ProjectOnPlane(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d ProjectPointOnPlane()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            var plane = new FixedPlane(_right[i], Vector3d.Dot(_right[i], _left[i]) * Fixed64.Quarter);
            accumulator += Vector3d.ProjectOnPlane(_left[i], plane);
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Angle()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.Angle(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d ClosestPointOnLineSegment()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.ClosestPointOnLineSegment(_right[i], _left[i], _left[i] + _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d ClosestPointsOnTwoLines()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            int next = (i + 17) & (BenchmarkFixtures.SampleCount - 1);
            (Vector3d pointOnLine1, Vector3d pointOnLine2) = Vector3d.ClosestPointsOnTwoLines(
                _left[i],
                _left[i] + _right[i],
                _right[i],
                _right[i] + _left[next]);
            accumulator += pointOnLine1 + pointOnLine2;
        }

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
            accumulator += Vector3d.Distance(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d Lerp()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += Vector3d.Lerp(_left[i], _right[i], Fixed64.Half);

        return accumulator;
    }
}
