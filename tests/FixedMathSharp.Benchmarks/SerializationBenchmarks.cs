using BenchmarkDotNet.Attributes;
using FixedMathSharp.Bounds;
#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
using MemoryPack;
#endif
using System.Text.Json;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class SerializationBenchmarks
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        ReferenceHandler = ReferenceHandler.IgnoreCycles,
    };

    private readonly Fixed64[] _scalars = BenchmarkFixtures.ScalarsA;
    private readonly Vector2d[] _vector2s = BenchmarkFixtures.Vector2sA;
    private readonly Vector3d[] _vector3s = BenchmarkFixtures.VectorsA;
    private readonly Vector4d[] _vector4s = BenchmarkFixtures.Vector4sA;
    private readonly FixedQuaternion[] _quaternions = BenchmarkFixtures.RotationsA;
    private readonly Fixed3x3[] _matrix3s = BenchmarkFixtures.Matrix3s;
    private readonly Fixed4x4[] _matrix4s = BenchmarkFixtures.Matrices;
    private readonly FixedBoundArea[] _areas = CreateAreas();
    private readonly FixedBoundBox[] _boxes = CreateBoxes();
    private readonly FixedBoundSphere[] _spheres = CreateSpheres();
    private readonly FixedCurve[] _curves = BenchmarkFixtures.CubicCurves;
    private readonly byte[][] _jsonScalarBytes;
    private readonly byte[][] _jsonVector3Bytes;
    private readonly byte[][] _jsonCurveBytes;
#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    private readonly byte[][] _memoryPackScalarBytes;
    private readonly byte[][] _memoryPackVector3Bytes;
    private readonly byte[][] _memoryPackCurveBytes;
#endif

    public SerializationBenchmarks()
    {
        _jsonScalarBytes = CreateJsonBytes(_scalars);
        _jsonVector3Bytes = CreateJsonBytes(_vector3s);
        _jsonCurveBytes = CreateJsonBytes(_curves);
#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
        _memoryPackScalarBytes = CreateMemoryPackBytes(_scalars);
        _memoryPackVector3Bytes = CreateMemoryPackBytes(_vector3s);
        _memoryPackCurveBytes = CreateMemoryPackBytes(_curves);
#endif
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Benchmark]
    public int MemoryPackSerializeMathTypes()
    {
        int bytes = 0;
        for (int i = 0; i < _scalars.Length; i++)
        {
            bytes += MemoryPackSerializer.Serialize(_scalars[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_vector2s[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_vector3s[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_vector4s[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_quaternions[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_matrix3s[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_matrix4s[i]).Length;
        }

        return bytes;
    }

    [Benchmark]
    public int MemoryPackSerializeBoundsAndCurves()
    {
        int bytes = 0;
        for (int i = 0; i < _boxes.Length; i++)
        {
            bytes += MemoryPackSerializer.Serialize(_areas[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_boxes[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_spheres[i]).Length;
            bytes += MemoryPackSerializer.Serialize(_curves[i]).Length;
        }

        return bytes;
    }

    [Benchmark]
    public Fixed64 MemoryPackDeserializeScalar()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _memoryPackScalarBytes.Length; i++)
            accumulator += MemoryPackSerializer.Deserialize<Fixed64>(_memoryPackScalarBytes[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d MemoryPackDeserializeVector3d()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _memoryPackVector3Bytes.Length; i++)
            accumulator += MemoryPackSerializer.Deserialize<Vector3d>(_memoryPackVector3Bytes[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 MemoryPackRoundTripCurve()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _curves.Length; i++)
        {
            byte[] bytes = MemoryPackSerializer.Serialize(_curves[i]);
            accumulator += MemoryPackSerializer.Deserialize<FixedCurve>(bytes).Evaluate(BenchmarkFixtures.CurveTimes[i]);
        }

        return accumulator;
    }
#endif

    [Benchmark]
    public int JsonSerializeMathTypes()
    {
        int bytes = 0;
        for (int i = 0; i < _scalars.Length; i++)
        {
            bytes += JsonSerializer.SerializeToUtf8Bytes(_scalars[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_vector2s[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_vector3s[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_vector4s[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_quaternions[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_matrix3s[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_matrix4s[i], JsonOptions).Length;
        }

        return bytes;
    }

    [Benchmark]
    public int JsonSerializeBoundsAndCurves()
    {
        int bytes = 0;
        for (int i = 0; i < _boxes.Length; i++)
        {
            bytes += JsonSerializer.SerializeToUtf8Bytes(_areas[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_boxes[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_spheres[i], JsonOptions).Length;
            bytes += JsonSerializer.SerializeToUtf8Bytes(_curves[i], JsonOptions).Length;
        }

        return bytes;
    }

    [Benchmark]
    public Fixed64 JsonDeserializeScalar()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _jsonScalarBytes.Length; i++)
            accumulator += JsonSerializer.Deserialize<Fixed64>(_jsonScalarBytes[i], JsonOptions);

        return accumulator;
    }

    [Benchmark]
    public Vector3d JsonDeserializeVector3d()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _jsonVector3Bytes.Length; i++)
            accumulator += JsonSerializer.Deserialize<Vector3d>(_jsonVector3Bytes[i], JsonOptions);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 JsonRoundTripCurve()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _curves.Length; i++)
        {
            byte[] bytes = JsonSerializer.SerializeToUtf8Bytes(_curves[i], JsonOptions);
            accumulator += JsonSerializer.Deserialize<FixedCurve>(bytes, JsonOptions).Evaluate(BenchmarkFixtures.CurveTimes[i]);
        }

        return accumulator;
    }

    private static byte[][] CreateJsonBytes<T>(T[] values)
    {
        var bytes = new byte[values.Length][];
        for (int i = 0; i < values.Length; i++)
            bytes[i] = JsonSerializer.SerializeToUtf8Bytes(values[i], JsonOptions);

        return bytes;
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    private static byte[][] CreateMemoryPackBytes<T>(T[] values)
    {
        var bytes = new byte[values.Length][];
        for (int i = 0; i < values.Length; i++)
            bytes[i] = MemoryPackSerializer.Serialize(values[i]);

        return bytes;
    }
#endif

    private static FixedBoundArea[] CreateAreas()
    {
        var areas = new FixedBoundArea[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < areas.Length; i++)
            areas[i] = FixedBoundArea.FromCenterAndSize(BenchmarkFixtures.Vector2sA[i], new Vector2d(
                Fixed64.One + Fixed64.FromFraction((i % 7) + 1, 16),
                Fixed64.One + Fixed64.FromFraction((i % 5) + 1, 16)));

        return areas;
    }

    private static FixedBoundBox[] CreateBoxes()
    {
        var boxes = new FixedBoundBox[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < boxes.Length; i++)
            boxes[i] = FixedBoundBox.FromCenterAndSize(BenchmarkFixtures.VectorsA[i], BenchmarkFixtures.Scales[i]);

        return boxes;
    }

    private static FixedBoundSphere[] CreateSpheres()
    {
        var spheres = new FixedBoundSphere[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < spheres.Length; i++)
            spheres[i] = new FixedBoundSphere(BenchmarkFixtures.VectorsB[i] * Fixed64.Quarter, Fixed64.One + Fixed64.FromFraction(i % 9, 4));

        return spheres;
    }
}
