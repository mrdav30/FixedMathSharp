using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Matrix3x3Benchmarks
{
    private readonly Fixed64[] _angles = BenchmarkFixtures.Angles;
    private readonly Vector3d[] _directions = BenchmarkFixtures.NormalizedAxes;
    private readonly Fixed3x3[] _matrices = BenchmarkFixtures.Matrix3s;

    [Benchmark]
    public Fixed3x3 CreateRotation()
    {
        Fixed3x3 accumulator = Fixed3x3.Identity;
        for (int i = 0; i < _angles.Length; i++)
        {
            Fixed64 pitch = _angles[(i + 7) & (BenchmarkFixtures.SampleCount - 1)] * Fixed64.Quarter;
            Fixed64 yaw = _angles[(i + 13) & (BenchmarkFixtures.SampleCount - 1)] * Fixed64.Quarter;
            Fixed64 roll = _angles[(i + 29) & (BenchmarkFixtures.SampleCount - 1)] * Fixed64.Quarter;
            accumulator =
                accumulator *
                Fixed3x3.CreateRotationY(yaw) *
                Fixed3x3.CreateRotationX(pitch) *
                Fixed3x3.CreateRotationZ(roll);
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed3x3 Multiply()
    {
        Fixed3x3 accumulator = Fixed3x3.Identity;
        for (int i = 0; i < _matrices.Length; i++)
            accumulator = accumulator * _matrices[i];

        return accumulator;
    }

    [Benchmark]
    public Vector3d TransformDirection()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _matrices.Length; i++)
            accumulator += Fixed3x3.TransformDirection(_matrices[i], _directions[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Determinant()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _matrices.Length; i++)
            accumulator += _matrices[i].GetDeterminant();

        return accumulator;
    }

    [Benchmark]
    public Fixed3x3 Invert()
    {
        Fixed3x3 accumulator = Fixed3x3.Identity;
        for (int i = 0; i < _matrices.Length; i++)
        {
            if (Fixed3x3.Invert(_matrices[i], out Fixed3x3? inverse) && inverse.HasValue)
                accumulator = accumulator * inverse.Value;
        }

        return accumulator;
    }
}
