using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Vector3dBenchmarks
{
    private readonly Vector3d[] _left = BenchmarkFixtures.VectorsA;
    private readonly Vector3d[] _right = BenchmarkFixtures.VectorsB;

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
    public Vector3d Normalize()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i].Normal;

        return accumulator;
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
