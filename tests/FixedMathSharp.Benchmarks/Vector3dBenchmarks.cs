using BenchmarkDotNet.Attributes;
using FixedMathSharp.Bounds;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Vector3dBenchmarks
{
    private readonly Vector3d[] _left = BenchmarkFixtures.VectorsA;
    private readonly Vector3d[] _right = BenchmarkFixtures.VectorsB;

    [Benchmark]
    public Vector3d Add()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] + _right[i];

        return accumulator;
    }

    [Benchmark]
    public Vector3d AddOut()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d.Add(_left[i], _right[i], out Vector3d result);
            accumulator += result;
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d AddScale()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] + _right[i]) * Fixed64.Half;

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

    [Benchmark]
    public Vector3d LerpOut()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Vector3d.Lerp(_left[i], _right[i], Fixed64.Half, out Vector3d result);
            accumulator += result;
        }

        return accumulator;
    }
}
