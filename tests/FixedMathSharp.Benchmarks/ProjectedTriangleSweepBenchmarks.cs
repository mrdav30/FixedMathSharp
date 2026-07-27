using BenchmarkDotNet.Attributes;
using FixedMathSharp.Bounds;

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
