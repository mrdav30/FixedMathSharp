using BenchmarkDotNet.Attributes;
using FixedMathSharp.Bounds;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class BoundsBenchmarks
{
    private readonly FixedBoundBox[] _boxes = CreateBoxes();
    private readonly Vector3d[] _points = BenchmarkFixtures.VectorsA;
    private readonly FixedBoundSphere[] _spheres = CreateSpheres();

    [Benchmark]
    public int BoxContainsPoint()
    {
        int count = 0;
        for (int i = 0; i < _boxes.Length; i++)
        {
            if (_boxes[i].Contains(_points[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int BoxIntersectsBox()
    {
        int count = 0;
        for (int i = 0; i < _boxes.Length; i++)
        {
            if (_boxes[i].Intersects(_boxes[(i + 1) & (BenchmarkFixtures.SampleCount - 1)]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int BoxIntersectsSphere()
    {
        int count = 0;
        for (int i = 0; i < _boxes.Length; i++)
        {
            if (_boxes[i].Intersects(_spheres[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public Vector3d BoxClampPoint()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _boxes.Length; i++)
            accumulator += _boxes[i].ClampPoint(_points[(i + 37) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    private static FixedBoundBox[] CreateBoxes()
    {
        var boxes = new FixedBoundBox[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < boxes.Length; i++)
        {
            Vector3d center = BenchmarkFixtures.VectorsA[i] * Fixed64.Quarter;
            Vector3d half = new(
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8),
                Fixed64.One + Fixed64.FromFraction(i % 3, 8));

            boxes[i] = new FixedBoundBox(center - half, center + half);
        }

        return boxes;
    }

    private static FixedBoundSphere[] CreateSpheres()
    {
        var spheres = new FixedBoundSphere[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < spheres.Length; i++)
        {
            Vector3d center = BenchmarkFixtures.VectorsB[i] * Fixed64.Quarter;
            Fixed64 radius = Fixed64.One + Fixed64.FromFraction((i % 9) + 1, 4);
            spheres[i] = new FixedBoundSphere(center, radius);
        }

        return spheres;
    }
}
