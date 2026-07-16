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
    private readonly FixedQuaternion[] _fullDomainMatrixInputs = CreateFullDomainMatrixInputs();

    [Benchmark]
    public FixedQuaternion FromAxisAngle()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _angles.Length; i++)
            accumulator = accumulator * FixedQuaternion.FromAxisAngle(_axes[i], _angles[i]);

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion FromAxisAngleCardinal()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _angles.Length; i++)
            accumulator = accumulator * FixedQuaternion.FromAxisAngle(Vector3d.Up, _angles[i]);

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
    public FixedQuaternion NormalizeInPlace()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _left.Length; i++)
        {
            FixedQuaternion value = _left[i] * _right[i];
            accumulator = accumulator * value.NormalizeInPlace();
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

    [Benchmark(OperationsPerInvoke = BenchmarkFixtures.SampleCount)]
    public Fixed64 ToMatrix3x3Ordinary()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Fixed3x3 matrix = _left[i].ToMatrix3x3();
            accumulator += matrix.M11 + matrix.M12 + matrix.M13
                + matrix.M21 + matrix.M22 + matrix.M23
                + matrix.M31 + matrix.M32 + matrix.M33;
        }

        return accumulator;
    }

    [Benchmark(OperationsPerInvoke = BenchmarkFixtures.SampleCount)]
    public Fixed64 ToMatrix3x3FullDomain()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _fullDomainMatrixInputs.Length; i++)
        {
            Fixed3x3 matrix = _fullDomainMatrixInputs[i].ToMatrix3x3();
            accumulator += matrix.M11 + matrix.M12 + matrix.M13
                + matrix.M21 + matrix.M22 + matrix.M23
                + matrix.M31 + matrix.M32 + matrix.M33;
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
    public FixedQuaternion FromDirectionCardinal()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        accumulator = accumulator * FixedQuaternion.FromDirection(Vector3d.Forward);
        accumulator = accumulator * FixedQuaternion.FromDirection(Vector3d.Right);
        accumulator = accumulator * FixedQuaternion.FromDirection(Vector3d.Backward);
        accumulator = accumulator * FixedQuaternion.FromDirection(Vector3d.Left);
        accumulator = accumulator * FixedQuaternion.FromDirection(Vector3d.Up);
        accumulator = accumulator * FixedQuaternion.FromDirection(Vector3d.Down);

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion FromDirectionNearParallel()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _axes.Length; i++)
        {
            Vector3d direction = new(
                Fixed64.FromRaw((i + 1) << 10),
                Fixed64.FromRaw((i & 1) == 0 ? 1L << 9 : -(1L << 9)),
                Fixed64.One);

            accumulator = accumulator * FixedQuaternion.FromDirection(direction);
        }

        return accumulator;
    }

    [Benchmark]
    public FixedQuaternion FromDirectionNearAntiParallel()
    {
        FixedQuaternion accumulator = FixedQuaternion.Identity;
        for (int i = 0; i < _axes.Length; i++)
        {
            Vector3d direction = new(
                Fixed64.FromRaw((i + 1) << 10),
                Fixed64.FromRaw((i & 1) == 0 ? 1L << 9 : -(1L << 9)),
                -Fixed64.One);

            accumulator = accumulator * FixedQuaternion.FromDirection(direction);
        }

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

    private static FixedQuaternion[] CreateFullDomainMatrixInputs()
    {
        var inputs = new FixedQuaternion[BenchmarkFixtures.SampleCount];
        ulong state = 0xA076_1D64_78BD_642FUL;
        for (int i = 0; i < inputs.Length; i++)
        {
            inputs[i] = new FixedQuaternion(
                Fixed64.FromRaw(unchecked((long)NextSplitMix64(ref state))),
                Fixed64.FromRaw(unchecked((long)NextSplitMix64(ref state))),
                Fixed64.FromRaw(unchecked((long)NextSplitMix64(ref state))),
                Fixed64.FromRaw(unchecked((long)NextSplitMix64(ref state))));
        }

        inputs[0] = FixedQuaternion.Zero;
        inputs[1] = new FixedQuaternion(
            Fixed64.FromRaw(1),
            Fixed64.FromRaw(-1),
            Fixed64.FromRaw(2),
            Fixed64.FromRaw(-2));
        inputs[2] = new FixedQuaternion(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.MaxValue,
            Fixed64.MinValue);
        return inputs;
    }

    private static ulong NextSplitMix64(ref ulong state)
    {
        unchecked
        {
            state += 0x9E37_79B9_7F4A_7C15UL;
            ulong value = state;
            value = (value ^ (value >> 30)) * 0xBF58_476D_1CE4_E5B9UL;
            value = (value ^ (value >> 27)) * 0x94D0_49BB_1331_11EBUL;
            return value ^ (value >> 31);
        }
    }
}
