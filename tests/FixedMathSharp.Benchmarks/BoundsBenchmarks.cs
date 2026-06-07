using BenchmarkDotNet.Attributes;
using FixedMathSharp.Bounds;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class BoundsBenchmarks
{
    private readonly FixedBoundArea[] _areas = CreateAreas();
    private readonly FixedBoundBox[] _boxes = CreateBoxes();
    private readonly Vector3d[] _cornerBuffer = new Vector3d[FixedBoundFrustum.CornerCount];
    private readonly FixedBoundFrustum[] _frustums = CreateFrustums();
    private readonly Fixed4x4[] _matrices = BenchmarkFixtures.PerspectiveMatrices;
    private readonly FixedPlane[] _planeBuffer = new FixedPlane[FixedBoundFrustum.PlaneCount];
    private readonly FixedPlane[] _planes = CreatePlanes();
    private readonly Vector3d[] _points = BenchmarkFixtures.VectorsA;
    private readonly FixedRay[] _rays = CreateRays();
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
    public int BoxIntersectsArea()
    {
        int count = 0;
        for (int i = 0; i < _boxes.Length; i++)
        {
            if (_boxes[i].Intersects(_areas[i]))
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

    [Benchmark]
    public int SphereContainsPoint()
    {
        int count = 0;
        for (int i = 0; i < _spheres.Length; i++)
        {
            if (_spheres[i].Contains(_points[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int SphereIntersectsSphere()
    {
        int count = 0;
        for (int i = 0; i < _spheres.Length; i++)
        {
            if (_spheres[i].Intersects(_spheres[(i + 1) & (BenchmarkFixtures.SampleCount - 1)]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int SphereIntersectsBox()
    {
        int count = 0;
        for (int i = 0; i < _spheres.Length; i++)
        {
            if (_spheres[i].Intersects(_boxes[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public Vector3d SphereClampPoint()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _spheres.Length; i++)
            accumulator += _spheres[i].ClampPoint(_points[(i + 53) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public FixedBoundSphere SphereCreateFromBoundingBox()
    {
        FixedBoundSphere accumulator = _spheres[0];
        for (int i = 0; i < _boxes.Length; i++)
            accumulator = FixedBoundSphere.CreateMerged(accumulator, FixedBoundSphere.CreateFromBoundingBox(_boxes[i]));

        return accumulator;
    }

    [Benchmark]
    public FixedBoundSphere SphereTransform()
    {
        FixedBoundSphere accumulator = _spheres[0];
        for (int i = 0; i < _spheres.Length; i++)
            accumulator = FixedBoundSphere.CreateMerged(accumulator, _spheres[i].Transform(BenchmarkFixtures.Matrices[i]));

        return accumulator;
    }

    [Benchmark]
    public int AreaContainsPoint()
    {
        int count = 0;
        for (int i = 0; i < _areas.Length; i++)
        {
            if (_areas[i].Contains(_points[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int AreaIntersectsArea()
    {
        int count = 0;
        for (int i = 0; i < _areas.Length; i++)
        {
            if (_areas[i].Intersects(_areas[(i + 1) & (BenchmarkFixtures.SampleCount - 1)]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int AreaIntersectsSphere()
    {
        int count = 0;
        for (int i = 0; i < _areas.Length; i++)
        {
            if (_areas[i].Intersects(_spheres[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public Vector3d AreaClampPoint()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _areas.Length; i++)
            accumulator += _areas[i].ClampPoint(_points[(i + 71) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public int FrustumCreateFromMatrix()
    {
        int count = 0;
        for (int i = 0; i < _matrices.Length; i++)
        {
            var frustum = new FixedBoundFrustum(_matrices[i]);
            if (frustum.Contains(_points[i]) != FixedEnclosureType.Disjoint)
                count++;
        }

        return count;
    }

    [Benchmark]
    public long FrustumConstructOnly()
    {
        long accumulator = 0;
        for (int i = 0; i < _matrices.Length; i++)
        {
            var frustum = new FixedBoundFrustum(_matrices[i]);
            accumulator += frustum.Min.X.m_rawValue;
        }

        return accumulator;
    }

    [Benchmark]
    public long FrustumSetMatrix()
    {
        long accumulator = 0;
        var frustum = _frustums[0];

        for (int i = 0; i < _matrices.Length; i++)
        {
            frustum.Matrix = _matrices[i];
            accumulator += frustum.Min.X.m_rawValue;
        }

        return accumulator;
    }

    [Benchmark]
    public int FrustumContainsPoint()
    {
        int count = 0;
        for (int i = 0; i < _frustums.Length; i++)
        {
            if (_frustums[i].Contains(_points[i]) != FixedEnclosureType.Disjoint)
                count++;
        }

        return count;
    }

    [Benchmark]
    public int FrustumIntersectsBox()
    {
        int count = 0;
        for (int i = 0; i < _frustums.Length; i++)
        {
            if (_frustums[i].Intersects(_boxes[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int FrustumIntersectsSphere()
    {
        int count = 0;
        for (int i = 0; i < _frustums.Length; i++)
        {
            if (_frustums[i].Intersects(_spheres[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public Fixed64 FrustumIntersectsRay()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _frustums.Length; i++)
        {
            Fixed64? hit = _frustums[i].Intersects(_rays[i]);
            if (hit.HasValue)
                accumulator += hit.Value;
        }

        return accumulator;
    }

    [Benchmark]
    public Vector3d FrustumGetCornersIntoArray()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _frustums.Length; i++)
        {
            _frustums[i].GetCorners(_cornerBuffer);
            accumulator += _cornerBuffer[i & (FixedBoundFrustum.CornerCount - 1)];
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 FrustumGetPlanesIntoArray()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _frustums.Length; i++)
        {
            _frustums[i].GetPlanes(_planeBuffer);
            accumulator += _planeBuffer[i % FixedBoundFrustum.PlaneCount].D;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 RayIntersectsBox()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _rays.Length; i++)
        {
            Fixed64? hit = _rays[i].Intersects(_boxes[i]);
            if (hit.HasValue)
                accumulator += hit.Value;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 RayIntersectsSphere()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _rays.Length; i++)
        {
            Fixed64? hit = _rays[i].Intersects(_spheres[i]);
            if (hit.HasValue)
                accumulator += hit.Value;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 RayIntersectsPlane()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _rays.Length; i++)
        {
            Fixed64? hit = _rays[i].Intersects(_planes[i]);
            if (hit.HasValue)
                accumulator += hit.Value;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 PlaneDotCoordinate()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _planes.Length; i++)
            accumulator += _planes[i].DotCoordinate(_points[i]);

        return accumulator;
    }

    [Benchmark]
    public int PlaneIntersectsBox()
    {
        int count = 0;
        for (int i = 0; i < _planes.Length; i++)
            count += (int)_planes[i].Intersects(_boxes[i]);

        return count;
    }

    [Benchmark]
    public int PlaneIntersectsSphere()
    {
        int count = 0;
        for (int i = 0; i < _planes.Length; i++)
            count += (int)_planes[i].Intersects(_spheres[i]);

        return count;
    }

    private static FixedBoundArea[] CreateAreas()
    {
        var areas = new FixedBoundArea[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < areas.Length; i++)
        {
            Vector3d center = BenchmarkFixtures.VectorsB[i] * Fixed64.Quarter;
            Vector3d half = new(
                Fixed64.One + Fixed64.FromFraction(i % 3, 8),
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8));

            areas[i] = new FixedBoundArea(center - half, center + half);
        }

        return areas;
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

    private static FixedBoundFrustum[] CreateFrustums()
    {
        var frustums = new FixedBoundFrustum[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < frustums.Length; i++)
            frustums[i] = new FixedBoundFrustum(BenchmarkFixtures.PerspectiveMatrices[i]);

        return frustums;
    }

    private static FixedPlane[] CreatePlanes()
    {
        var planes = new FixedPlane[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < planes.Length; i++)
            planes[i] = new FixedPlane(BenchmarkFixtures.NormalizedAxes[i], Fixed64.FromFraction((i % 7) - 3, 2));

        return planes;
    }

    private static FixedRay[] CreateRays()
    {
        var rays = new FixedRay[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < rays.Length; i++)
            rays[i] = new FixedRay(BenchmarkFixtures.VectorsB[i], BenchmarkFixtures.NormalizedAxes[(i + 37) & (BenchmarkFixtures.SampleCount - 1)]);

        return rays;
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
