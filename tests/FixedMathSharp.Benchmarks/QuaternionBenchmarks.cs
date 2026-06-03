using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class QuaternionBenchmarks
{
    private readonly Fixed64[] _angles = BenchmarkFixtures.Angles;
    private readonly Vector3d[] _vectors = BenchmarkFixtures.VectorsA;
    private readonly FixedQuaternion[] _left = BenchmarkFixtures.RotationsA;
    private readonly FixedQuaternion[] _right = BenchmarkFixtures.RotationsB;

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
    public FixedQuaternion Slerp()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _left.Length; i++)
            accumulator = accumulator * FixedQuaternion.Slerp(_left[i], _right[i], Fixed64.Half);

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
