using BenchmarkDotNet.Attributes;
using FixedMathSharp.Bounds;
using System;
using System.Collections.Generic;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class BoundsBenchmarks
{
    private readonly FixedBoundArea[] _areas = CreateAreas();
    private readonly FixedBoundBox[] _boxes = CreateBoxes();
    private readonly FixedBoundCircle[] _circles = CreateCircles();
    private readonly FixedSegment2d[] _segments2d = CreateSegments2d();
    private readonly FixedSegment3d[] _segments3d = CreateSegments3d();
    private readonly Vector3d[] _boxCornerBuffer = new Vector3d[FixedBoundBox.CornerCount];
    private readonly Vector3d[] _cornerBuffer = new Vector3d[FixedBoundFrustum.CornerCount];
    private readonly FixedBoundFrustum[] _frustums = CreateFrustums();
    private readonly Fixed4x4[] _matrices = BenchmarkFixtures.PerspectiveMatrices;
    private readonly FixedPlane[] _planeBuffer = new FixedPlane[FixedBoundFrustum.PlaneCount];
    private readonly FixedPlane[] _planes = CreatePlanes();
    private readonly Vector3d[] _points = BenchmarkFixtures.VectorsA;
    private readonly Vector2d[] _points2d = BenchmarkFixtures.Vector2sA;
    private readonly FixedRay2d[] _rays2d = CreateRays2d();
    private readonly Vector3d[] _spherePointCloud = CreateSpherePointCloud();
    private readonly FixedRay[] _rays = CreateRays();
    private readonly FixedBoundSphere[] _spheres = CreateSpheres();

    [Benchmark]
    public Fixed64 AreaConstructCenterSize()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _points2d.Length; i++)
        {
            Vector2d size = new(
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8));
            var area = FixedBoundArea.FromCenterAndSize(_points2d[i] * Fixed64.Quarter, size);
            accumulator += area.Min.X + area.Max.Y;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 AreaConstructMinMax()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _points2d.Length; i++)
        {
            Vector2d min = _points2d[i] * Fixed64.Quarter;
            Vector2d max = min + new Vector2d(
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8));
            var area = FixedBoundArea.FromMinMax(max, min);
            accumulator += area.Min.X + area.Max.Y;
        }

        return accumulator;
    }

    [Benchmark]
    public int AreaContainsPoint()
    {
        int count = 0;
        for (int i = 0; i < _areas.Length; i++)
        {
            if (_areas[i].Contains(_points2d[i]))
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
    public int AreaIntersectsAreaStrict()
    {
        int count = 0;
        for (int i = 0; i < _areas.Length; i++)
        {
            if (_areas[i].IntersectsStrict(_areas[(i + 1) & (BenchmarkFixtures.SampleCount - 1)]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public Vector2d AreaClampPoint()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _areas.Length; i++)
            accumulator += _areas[i].ClampPoint(_points2d[(i + 37) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public FixedBoundArea AreaUnion()
    {
        FixedBoundArea accumulator = _areas[0];
        for (int i = 1; i < _areas.Length; i++)
            accumulator = FixedBoundArea.Union(accumulator, _areas[i]);

        return accumulator;
    }

    [Benchmark]
    public int CircleContainsPoint()
    {
        int count = 0;
        for (int i = 0; i < _circles.Length; i++)
        {
            if (_circles[i].Contains(_points2d[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int CircleIntersectsCircle()
    {
        int count = 0;
        for (int i = 0; i < _circles.Length; i++)
        {
            if (_circles[i].Intersects(_circles[(i + 1) & (BenchmarkFixtures.SampleCount - 1)]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int CircleIntersectsArea()
    {
        int count = 0;
        for (int i = 0; i < _circles.Length; i++)
        {
            if (_circles[i].Intersects(_areas[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public Vector2d CircleClampPoint()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _circles.Length; i++)
            accumulator += _circles[i].ClampPoint(_points2d[(i + 37) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public Vector2d CircleProjectPoint()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _circles.Length; i++)
            accumulator += _circles[i].ProjectPoint(_points2d[(i + 53) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public Vector2d Segment2dClosestPoint()
    {
        Vector2d accumulator = Vector2d.Zero;
        for (int i = 0; i < _segments2d.Length; i++)
            accumulator += _segments2d[i].ClosestPoint(_points2d[(i + 37) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Segment2dDistanceSquared()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _segments2d.Length; i++)
            accumulator += _segments2d[i].DistanceSquared(_points2d[(i + 53) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d Segment3dClosestPoint()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _segments3d.Length; i++)
            accumulator += _segments3d[i].ClosestPoint(_points[(i + 37) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Segment3dDistanceSquared()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _segments3d.Length; i++)
            accumulator += _segments3d[i].DistanceSquared(_points[(i + 53) & (BenchmarkFixtures.SampleCount - 1)]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Ray2dIntersectsArea()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _rays2d.Length; i++)
        {
            Fixed64? hit = _rays2d[i].Intersects(_areas[i]);
            if (hit.HasValue)
                accumulator += hit.Value;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Ray2dIntersectsCircle()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _rays2d.Length; i++)
        {
            Fixed64? hit = _rays2d[i].Intersects(_circles[i]);
            if (hit.HasValue)
                accumulator += hit.Value;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 BoxConstructCenterSize()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _points.Length; i++)
        {
            Vector3d size = new(
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8),
                Fixed64.One + Fixed64.FromFraction(i % 3, 8));
            var box = FixedBoundBox.FromCenterAndSize(_points[i] * Fixed64.Quarter, size);
            accumulator += box.Min.X + box.Max.Z;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 BoxConstructMinMax()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _points.Length; i++)
        {
            Vector3d min = _points[i] * Fixed64.Quarter;
            Vector3d max = min + new Vector3d(
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8),
                Fixed64.One + Fixed64.FromFraction(i % 3, 8));
            var box = FixedBoundBox.FromMinMax(max, min);
            accumulator += box.Min.X + box.Max.Z;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 BoxSetMinMax()
    {
        Fixed64 accumulator = Fixed64.Zero;
        var box = _boxes[0];
        for (int i = 0; i < _boxes.Length; i++)
        {
            box.SetMinMax(_boxes[i].Min, _boxes[i].Max);
            accumulator += box.Center.X;
        }

        return accumulator;
    }

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

    [Benchmark]
    public Vector3d BoxGetCorner()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _boxes.Length; i++)
            accumulator += _boxes[i].GetCorner(i & (FixedBoundBox.CornerCount - 1));

        return accumulator;
    }

    [Benchmark]
    public Vector3d BoxCopyCorners()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _boxes.Length; i++)
        {
            _boxes[i].CopyCorners(_boxCornerBuffer);
            accumulator += _boxCornerBuffer[i & (FixedBoundBox.CornerCount - 1)];
        }

        return accumulator;
    }

    [Benchmark]
    public FixedBoundBox BoxUnion()
    {
        FixedBoundBox accumulator = _boxes[0];
        for (int i = 1; i < _boxes.Length; i++)
            accumulator = FixedBoundBox.Union(accumulator, _boxes[i]);

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
    public FixedBoundSphere SphereCreateFromPointsArray()
    {
        return FixedBoundSphere.CreateFromPoints(_spherePointCloud);
    }

    [Benchmark]
    public FixedBoundSphere SphereCreateFromPointsSpan()
    {
        return FixedBoundSphere.CreateFromPoints(_spherePointCloud.AsSpan());
    }

    [Benchmark]
    public FixedBoundSphere SphereCreateFromPointsEnumerable()
    {
        return FixedBoundSphere.CreateFromPoints(EnumerateSpherePointCloud());
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
            Vector2d center = BenchmarkFixtures.Vector2sA[i] * Fixed64.Quarter;
            Vector2d half = new(
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8));

            areas[i] = FixedBoundArea.FromMinMax(center - half, center + half);
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

            boxes[i] = FixedBoundBox.FromMinMax(center - half, center + half);
        }

        return boxes;
    }

    private static FixedBoundCircle[] CreateCircles()
    {
        var circles = new FixedBoundCircle[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < circles.Length; i++)
        {
            Vector2d center = BenchmarkFixtures.Vector2sB[i] * Fixed64.Quarter;
            Fixed64 radius = Fixed64.One + Fixed64.FromFraction((i % 9) + 1, 4);

            circles[i] = new FixedBoundCircle(center, radius);
        }

        return circles;
    }

    private static FixedSegment2d[] CreateSegments2d()
    {
        var segments = new FixedSegment2d[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < segments.Length; i++)
        {
            Vector2d start = BenchmarkFixtures.Vector2sA[i] * Fixed64.Quarter;
            Vector2d delta = new(
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8));

            if ((i & 1) != 0)
                delta.X = -delta.X;

            segments[i] = new FixedSegment2d(start, start + delta);
        }

        return segments;
    }

    private static FixedSegment3d[] CreateSegments3d()
    {
        var segments = new FixedSegment3d[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < segments.Length; i++)
        {
            Vector3d start = BenchmarkFixtures.VectorsA[i] * Fixed64.Quarter;
            Vector3d delta = new(
                Fixed64.One + Fixed64.FromFraction(i % 5, 8),
                Fixed64.One + Fixed64.FromFraction(i % 7, 8),
                Fixed64.One + Fixed64.FromFraction(i % 3, 8));

            if ((i & 1) != 0)
                delta.X = -delta.X;
            if ((i & 2) != 0)
                delta.Z = -delta.Z;

            segments[i] = new FixedSegment3d(start, start + delta);
        }

        return segments;
    }

    private static FixedRay2d[] CreateRays2d()
    {
        var rays = new FixedRay2d[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < rays.Length; i++)
        {
            Vector2d direction = BenchmarkFixtures.Vector2sA[(i + 37) & (BenchmarkFixtures.SampleCount - 1)].Normalized;
            if (direction.EqualsZero())
                direction = Vector2d.Right;

            rays[i] = new FixedRay2d(BenchmarkFixtures.Vector2sB[i], direction);
        }

        return rays;
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

    private static Vector3d[] CreateSpherePointCloud()
    {
        var points = new Vector3d[BenchmarkFixtures.SampleCount];
        for (int i = 0; i < points.Length; i++)
        {
            points[i] = BenchmarkFixtures.VectorsA[i] + (BenchmarkFixtures.NormalizedAxes[i] * Fixed64.FromFraction((i % 9) + 1, 4));
        }

        return points;
    }

    private IEnumerable<Vector3d> EnumerateSpherePointCloud()
    {
        for (int i = 0; i < _spherePointCloud.Length; i++)
            yield return _spherePointCloud[i];
    }
}
