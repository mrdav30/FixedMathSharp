using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Matrix4x4Benchmarks
{
    private readonly Fixed4x4[] _matrices = BenchmarkFixtures.Matrices;
    private readonly FixedQuaternion[] _rotations = BenchmarkFixtures.RotationsA;
    private readonly Vector3d[] _points = BenchmarkFixtures.VectorsA;

    [Benchmark]
    public Fixed4x4 CreateTransform()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _points.Length; i++)
        {
            Vector3d scale = new(
                Fixed64.One + Fixed64.FromFraction((i % 7) + 1, 16),
                Fixed64.One + Fixed64.FromFraction((i % 5) + 1, 16),
                Fixed64.One + Fixed64.FromFraction((i % 3) + 1, 16));

            accumulator = accumulator * Fixed4x4.CreateTransform(_points[i] * Fixed64.Quarter, _rotations[i], scale);
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed4x4 Multiply()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _matrices.Length; i++)
            accumulator = accumulator * _matrices[i];

        return accumulator;
    }

    [Benchmark]
    public Vector3d TransformPoint()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _matrices.Length; i++)
            accumulator += Fixed4x4.TransformPoint(_matrices[i], _points[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed4x4 InvertAffine()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _matrices.Length; i++)
        {
            if (Fixed4x4.Invert(_matrices[i], out Fixed4x4 inverse))
                accumulator = accumulator * inverse;
        }

        return accumulator;
    }
}
