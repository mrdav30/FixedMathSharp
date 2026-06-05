using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Matrix4x4Benchmarks
{
    private readonly Fixed4x4[] _matrices = BenchmarkFixtures.Matrices;
    private readonly Fixed4x4[] _perspectiveMatrices = BenchmarkFixtures.PerspectiveMatrices;
    private readonly FixedQuaternion[] _rotations = BenchmarkFixtures.RotationsA;
    private readonly Vector3d[] _points = BenchmarkFixtures.VectorsA;
    private readonly Vector3d[] _scales = BenchmarkFixtures.Scales;
    private readonly Vector3d[] _translations = BenchmarkFixtures.Translations;

    [Benchmark]
    public Fixed4x4 CreateTranslation()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _points.Length; i++)
            accumulator = accumulator * Fixed4x4.CreateTranslation(_translations[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed4x4 CreateRotation()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _rotations.Length; i++)
            accumulator = accumulator * Fixed4x4.CreateRotation(_rotations[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed4x4 CreateScale()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _scales.Length; i++)
            accumulator = accumulator * Fixed4x4.CreateScale(_scales[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed4x4 CreateTransform()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _points.Length; i++)
            accumulator = accumulator * Fixed4x4.CreateTransform(_translations[i], _rotations[i], _scales[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed4x4 ScaleRotateTranslate()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _points.Length; i++)
            accumulator = accumulator * Fixed4x4.ScaleRotateTranslate(_translations[i], _rotations[i], _scales[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed4x4 TranslateRotateScale()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _points.Length; i++)
            accumulator = accumulator * Fixed4x4.TranslateRotateScale(_translations[i], _rotations[i], _scales[i]);

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
    public Fixed64 ExtractRotation()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _matrices.Length; i++)
        {
            FixedQuaternion rotation = Fixed4x4.ExtractRotation(_matrices[i]);
            accumulator += rotation.X + rotation.Y + rotation.Z + rotation.W;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Decompose()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _matrices.Length; i++)
        {
            if (Fixed4x4.Decompose(_matrices[i], out Vector3d scale, out FixedQuaternion rotation, out Vector3d translation))
            {
                accumulator += scale.X + scale.Y + scale.Z;
                accumulator += rotation.X + rotation.Y + rotation.Z + rotation.W;
                accumulator += translation.X + translation.Y + translation.Z;
            }
        }

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

    [Benchmark]
    public Fixed4x4 InvertFull()
    {
        Fixed4x4 accumulator = Fixed4x4.Identity;
        for (int i = 0; i < _perspectiveMatrices.Length; i++)
        {
            if (Fixed4x4.Invert(_perspectiveMatrices[i], out Fixed4x4 inverse))
                accumulator = accumulator * inverse;
        }

        return accumulator;
    }
}
