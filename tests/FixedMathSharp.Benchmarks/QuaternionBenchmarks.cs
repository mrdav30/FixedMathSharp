using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class QuaternionBenchmarks
{
    private readonly Fixed64[] _angles = BenchmarkFixtures.Angles;
    private readonly Fixed64[] _degreeAngles = BenchmarkFixtures.DegreeAngles;
    private readonly Vector3d[] _axes = BenchmarkFixtures.NormalizedAxes;
    private readonly Vector3d[] _vectors = BenchmarkFixtures.VectorsA;
    private readonly FixedQuaternion[] _left = BenchmarkFixtures.RotationsA;
    private readonly FixedQuaternion[] _right = BenchmarkFixtures.RotationsB;

    [Benchmark]
    public FixedQuaternion FromAxisAngle()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _angles.Length; i++)
            accumulator = accumulator * FixedQuaternion.FromAxisAngle(_axes[i], _angles[i]);

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion AngleAxis()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _degreeAngles.Length; i++)
            accumulator = accumulator * FixedQuaternion.AngleAxis(_degreeAngles[i], _axes[i]);

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion FromEulerAngles()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _angles.Length; i++)
        {
            Fixed64 pitch = _angles[i] * Fixed64.Quarter;
            Fixed64 yaw = _angles[(i + 17) & (BenchmarkFixtures.SampleCount - 1)] * Fixed64.Quarter;
            Fixed64 roll = _angles[(i + 31) & (BenchmarkFixtures.SampleCount - 1)] * Fixed64.Quarter;
            accumulator = accumulator * FixedQuaternion.FromEulerAngles(pitch, yaw, roll);
        }

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion Multiply()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _left.Length; i++)
            accumulator = accumulator * (_left[i] * _right[i]);

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion Lerp()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _left.Length; i++)
            accumulator = accumulator * FixedQuaternion.Lerp(_left[i], _right[i], Fixed64.Half);

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion Slerp()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _left.Length; i++)
            accumulator = accumulator * FixedQuaternion.Slerp(_left[i], _right[i], Fixed64.Half);

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion Normalize()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _left.Length; i++)
        {
            FixedQuaternion value = _left[i] * _right[i];
            accumulator = accumulator * value.Normalize();
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d ToEulerAngles()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i].ToEulerAngles();

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion FromDirection()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _axes.Length; i++)
            accumulator = accumulator * FixedQuaternion.FromDirection(_axes[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d RotateVector()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] * _vectors[i];

        return accumulator;
    }
}
