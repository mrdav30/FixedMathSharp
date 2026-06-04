namespace FixedMathSharp.Benchmarks;

internal static class BenchmarkFixtures
{
    internal const int SampleCount = 256;

    internal static readonly Fixed64[] ScalarsA = CreateScalars(17, 5);
    internal static readonly Fixed64[] ScalarsB = CreateScalars(29, 7);
    internal static readonly Fixed64[] PositiveScalars = CreatePositiveScalars();
    internal static readonly Fixed64[] UnitScalars = CreateUnitScalars();
    internal static readonly Fixed64[] Angles = CreateAngles();
    internal static readonly Fixed64[] TangentAngles = CreateTangentAngles();
    internal static readonly Fixed64[] DegreeAngles = CreateDegreeAngles();
    internal static readonly Vector2d[] Vector2sA = CreateVector2s(3);
    internal static readonly Vector2d[] Vector2sB = CreateVector2s(19);
    internal static readonly Vector3d[] VectorsA = CreateVectors(3);
    internal static readonly Vector3d[] VectorsB = CreateVectors(19);
    internal static readonly Vector3d[] NormalizedAxes = CreateNormalizedAxes();
    internal static readonly Vector3d[] Translations = CreateTranslations();
    internal static readonly Vector3d[] Scales = CreateScales();
    internal static readonly Vector4d[] Vector4sA = CreateVector4s(3);
    internal static readonly Vector4d[] Vector4sB = CreateVector4s(19);
    internal static readonly FixedQuaternion[] RotationsA = CreateRotations(5);
    internal static readonly FixedQuaternion[] RotationsB = CreateRotations(23);
    internal static readonly Fixed3x3[] Matrix3s = CreateMatrix3s();
    internal static readonly Fixed4x4[] Matrices = CreateMatrices();
    internal static readonly Fixed4x4[] PerspectiveMatrices = CreatePerspectiveMatrices();

    private static Fixed64[] CreateScalars(int seed, int step)
    {
        var values = new Fixed64[SampleCount];
        for (int i = 0; i < values.Length; i++)
        {
            int integer = ((seed + (i * step)) % 41) - 20;
            long fraction = ((long)((seed * 13) + (i * 97)) & 0xFFFFL) << 12;
            values[i] = new Fixed64(integer) + Fixed64.FromRaw(fraction);
        }

        return values;
    }

    private static Fixed64[] CreatePositiveScalars()
    {
        var values = new Fixed64[SampleCount];
        for (int i = 0; i < values.Length; i++)
        {
            int integer = 1 + ((i * 7) % 31);
            long fraction = ((long)((i * 43) + 11) & 0xFFFFL) << 12;
            values[i] = new Fixed64(integer) + Fixed64.FromRaw(fraction);
        }

        return values;
    }

    private static Fixed64[] CreateUnitScalars()
    {
        var values = new Fixed64[SampleCount];
        for (int i = 0; i < values.Length; i++)
            values[i] = Fixed64.FromFraction(i - (SampleCount / 2), SampleCount / 2);

        return values;
    }

    private static Fixed64[] CreateAngles()
    {
        var values = new Fixed64[SampleCount];
        for (int i = 0; i < values.Length; i++)
        {
            Fixed64 centered = Fixed64.FromFraction(i - (SampleCount / 2), SampleCount / 2);
            values[i] = centered * Fixed64.Pi;
        }

        return values;
    }

    private static Fixed64[] CreateTangentAngles()
    {
        var values = new Fixed64[SampleCount];
        for (int i = 0; i < values.Length; i++)
        {
            Fixed64 centered = Fixed64.FromFraction(i - (SampleCount / 2), SampleCount / 2);
            values[i] = centered * Fixed64.PiOver3;
        }

        return values;
    }

    private static Fixed64[] CreateDegreeAngles()
    {
        var values = new Fixed64[SampleCount];
        for (int i = 0; i < values.Length; i++)
            values[i] = Fixed64.FromFraction((i % 181) - 90, 1);

        return values;
    }

    private static Vector2d[] CreateVector2s(int seed)
    {
        var vectors = new Vector2d[SampleCount];
        for (int i = 0; i < vectors.Length; i++)
        {
            Fixed64 x = new Fixed64(((seed + (i * 3)) % 29) - 14) + Fixed64.FromFraction(i + 1, SampleCount);
            Fixed64 y = new Fixed64(((seed + (i * 5)) % 31) - 15) + Fixed64.FromFraction(i + 3, SampleCount);
            vectors[i] = new Vector2d(x, y);
        }

        return vectors;
    }

    private static Vector3d[] CreateVectors(int seed)
    {
        var vectors = new Vector3d[SampleCount];
        for (int i = 0; i < vectors.Length; i++)
        {
            Fixed64 x = new Fixed64(((seed + (i * 3)) % 29) - 14) + Fixed64.FromFraction(i + 1, SampleCount);
            Fixed64 y = new Fixed64(((seed + (i * 5)) % 31) - 15) + Fixed64.FromFraction(i + 3, SampleCount);
            Fixed64 z = new Fixed64(((seed + (i * 7)) % 37) - 18) + Fixed64.FromFraction(i + 5, SampleCount);
            vectors[i] = new Vector3d(x, y, z);
        }

        return vectors;
    }

    private static Vector3d[] CreateNormalizedAxes()
    {
        var axes = new Vector3d[SampleCount];
        for (int i = 0; i < axes.Length; i++)
            axes[i] = VectorsA[i].Normal;

        return axes;
    }

    private static Vector3d[] CreateTranslations()
    {
        var translations = new Vector3d[SampleCount];
        for (int i = 0; i < translations.Length; i++)
            translations[i] = VectorsA[i] * Fixed64.Quarter;

        return translations;
    }

    private static Vector3d[] CreateScales()
    {
        var scales = new Vector3d[SampleCount];
        for (int i = 0; i < scales.Length; i++)
        {
            scales[i] = new Vector3d(
                Fixed64.One + Fixed64.FromFraction((i % 7) + 1, 16),
                Fixed64.One + Fixed64.FromFraction((i % 5) + 1, 16),
                Fixed64.One + Fixed64.FromFraction((i % 3) + 1, 16));
        }

        return scales;
    }

    private static Vector4d[] CreateVector4s(int seed)
    {
        var vectors = new Vector4d[SampleCount];
        for (int i = 0; i < vectors.Length; i++)
        {
            Fixed64 x = new Fixed64(((seed + (i * 3)) % 29) - 14) + Fixed64.FromFraction(i + 1, SampleCount);
            Fixed64 y = new Fixed64(((seed + (i * 5)) % 31) - 15) + Fixed64.FromFraction(i + 3, SampleCount);
            Fixed64 z = new Fixed64(((seed + (i * 7)) % 37) - 18) + Fixed64.FromFraction(i + 5, SampleCount);
            Fixed64 w = new Fixed64(((seed + (i * 11)) % 43) - 21) + Fixed64.FromFraction(i + 7, SampleCount);
            vectors[i] = new Vector4d(x, y, z, w);
        }

        return vectors;
    }

    private static FixedQuaternion[] CreateRotations(int seed)
    {
        var rotations = new FixedQuaternion[SampleCount];
        for (int i = 0; i < rotations.Length; i++)
        {
            Fixed64 pitch = Angles[(i + seed) & (SampleCount - 1)] * Fixed64.Quarter;
            Fixed64 yaw = Angles[(i + seed + 17) & (SampleCount - 1)] * Fixed64.Quarter;
            Fixed64 roll = Angles[(i + seed + 31) & (SampleCount - 1)] * Fixed64.Quarter;
            rotations[i] = FixedQuaternion.FromEulerAngles(pitch, yaw, roll);
        }

        return rotations;
    }

    private static Fixed3x3[] CreateMatrix3s()
    {
        var matrices = new Fixed3x3[SampleCount];
        for (int i = 0; i < matrices.Length; i++)
        {
            Fixed64 pitch = Angles[(i + 7) & (SampleCount - 1)] * Fixed64.Quarter;
            Fixed64 yaw = Angles[(i + 13) & (SampleCount - 1)] * Fixed64.Quarter;
            Fixed64 roll = Angles[(i + 29) & (SampleCount - 1)] * Fixed64.Quarter;
            Fixed3x3 rotation =
                Fixed3x3.CreateRotationY(yaw) *
                Fixed3x3.CreateRotationX(pitch) *
                Fixed3x3.CreateRotationZ(roll);

            matrices[i] = rotation * Fixed3x3.CreateScale(Scales[i]);
        }

        return matrices;
    }

    private static Fixed4x4[] CreateMatrices()
    {
        var matrices = new Fixed4x4[SampleCount];
        for (int i = 0; i < matrices.Length; i++)
            matrices[i] = Fixed4x4.CreateTransform(Translations[i], RotationsA[i], Scales[i]);

        return matrices;
    }

    private static Fixed4x4[] CreatePerspectiveMatrices()
    {
        var matrices = new Fixed4x4[SampleCount];
        for (int i = 0; i < matrices.Length; i++)
        {
            Fixed64 aspect = Fixed64.FromFraction(16 + (i % 3), 9);
            Fixed64 farPlane = new Fixed64(64 + (i % 17));
            matrices[i] = Fixed4x4.CreatePerspectiveFieldOfView(Fixed64.PiOver3, aspect, Fixed64.One, farPlane);
        }

        return matrices;
    }
}
