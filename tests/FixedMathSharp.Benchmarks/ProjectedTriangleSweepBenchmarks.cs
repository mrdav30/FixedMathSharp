using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class ProjectedTriangleSweepBenchmarks
{
    private FixedTriangle _triangle;
    private Vector3d _ordinaryOrigin;
    private Vector2d _ordinaryCircleStart;
    private Vector3d _scalarFaceOrigin;
    private Vector2d _scalarFaceCircleStart;
    private Vector3d _contactSlabCenter;
    private FixedTriangle[][] _verticalTriangles;

    [GlobalSetup]
    public void Setup()
    {
        _triangle = new FixedTriangle(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero));
        _ordinaryOrigin = Vector3d.Zero;
        _ordinaryCircleStart = new Vector2d((Fixed64)(-2), Fixed64.Zero);
        _scalarFaceOrigin =
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        _scalarFaceCircleStart = new Vector2d(
            Fixed64.MaxValue - Fixed64.Two,
            Fixed64.Zero);
        _contactSlabCenter =
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        _verticalTriangles = new FixedTriangle[3][];
        for (int grid = 0; grid < _verticalTriangles.Length; grid++)
        {
            int subdivisions = 8 << grid;
            var triangles = new FixedTriangle[2 * subdivisions * subdivisions];
            for (int y = 0; y < subdivisions; y++)
                for (int z = 0; z < subdivisions; z++)
                {
                    Fixed64 lowerY = Fixed64.FromFraction(y, subdivisions) - Fixed64.Half;
                    Fixed64 upperY = Fixed64.FromFraction(y + 1, subdivisions) - Fixed64.Half;
                    Fixed64 lowerZ = Fixed64.FromFraction(z, subdivisions) - Fixed64.Half;
                    Fixed64 upperZ = Fixed64.FromFraction(z + 1, subdivisions) - Fixed64.Half;
                    int index = 2 * (y * subdivisions + z);
                    triangles[index] = new FixedTriangle(new(Fixed64.Zero, lowerY, lowerZ), new(Fixed64.Zero, upperY, lowerZ), new(Fixed64.Zero, lowerY, upperZ));
                    triangles[index + 1] = new FixedTriangle(new(Fixed64.Zero, lowerY, upperZ), new(Fixed64.Zero, upperY, lowerZ), new(Fixed64.Zero, upperY, upperZ));
                }
            _verticalTriangles[grid] = triangles;
            if (VerticalSubdivisionSweep(subdivisions) != triangles.Length)
                throw new System.InvalidOperationException("Every vertical triangle must meet the sweep.");
        }
        // The central two rows meet [-1/8,1/8], plus one touching row on each side.
        if (ClippedVerticalSubdivisionSweep() != 64)
            throw new System.InvalidOperationException("Clipped vertical rows must retain inclusive slab boundaries.");

        if (OrdinaryProjectedTriangleSweep() != Fixed64.FromFraction(3, 2)
            || ScalarFaceProjectedTriangleSweep()
                != Fixed64.FromFraction(3, 2)
            || !TriangleCircleSlabContact()
            || !TriangleCapsuleSlabContact())
        {
            throw new System.InvalidOperationException(
                "Projected-triangle benchmark scenarios must intersect.");
        }
    }

    [Benchmark(Baseline = true)]
    public Fixed64 OrdinaryProjectedTriangleSweep()
    {
        _ = _triangle.TryGetFiniteSlabProjectedCircleSweep(
            _ordinaryOrigin,
            FixedQuaternion.Identity,
            _ordinaryCircleStart,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out _);
        return distance;
    }

    [Benchmark]
    public Fixed64 ScalarFaceProjectedTriangleSweep()
    {
        _ = _triangle.TryGetFiniteSlabProjectedCircleSweep(
            _scalarFaceOrigin,
            FixedQuaternion.Identity,
            _scalarFaceCircleStart,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out _);
        return distance;
    }

    [Benchmark]
    [Arguments(8)]
    [Arguments(16)]
    [Arguments(32)]
    public int VerticalSubdivisionSweep(int subdivisions) =>
        SweepVerticalTriangles(_verticalTriangles[subdivisions == 8 ? 0 : subdivisions == 16 ? 1 : 2], Fixed64.Half);

    [Benchmark]
    public int ClippedVerticalSubdivisionSweep() => SweepVerticalTriangles(_verticalTriangles[0], Fixed64.FromFraction(1, 8));

    private static int SweepVerticalTriangles(FixedTriangle[] triangles, Fixed64 halfThickness)
    {
        int hits = 0;
        foreach (FixedTriangle triangle in triangles)
            if (triangle.TryGetFiniteSlabProjectedCircleSweep(Vector3d.Zero, FixedQuaternion.Identity,
                    new Vector2d(-3, 0), Vector2d.Right, (Fixed64)6, Fixed64.Half,
                    Fixed64.Zero, halfThickness, out _, out _))
                hits++;
        return hits;
    }

    [Benchmark]
    public bool TriangleCircleSlabContact() =>
        _triangle.TryGetCircleSlabContact(
            _ordinaryOrigin,
            FixedQuaternion.Identity,
            _contactSlabCenter,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _);

    [Benchmark]
    public bool TriangleCapsuleSlabContact() =>
        _triangle.TryGetCenteredCapsuleSlabContact(
            _ordinaryOrigin,
            FixedQuaternion.Identity,
            _contactSlabCenter,
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.One,
            out _);
}
