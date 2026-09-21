using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

/// <summary>Full-body strict planar sweeps, including closed-contact near misses.</summary>
[MemoryDiagnoser]
public class PlanarCapsuleSweepBenchmarks
{
    private readonly Vector2d[] _square = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };

    [GlobalSetup]
    public void Setup()
    {
        if (!MiddleCrossing() || SideTangency() || RoundedCornerMiss() || !StationaryOverlap())
            throw new InvalidOperationException("Planar sweeps must retain exact strict classifications.");
    }

    [Benchmark]
    public bool MiddleCrossing() => FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
        new(-3, 0), new(3, 0), (Fixed64)4, Fixed64.Half, Vector2d.Zero, _square);

    [Benchmark]
    public bool SideTangency() => FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
        new(2, -3), new(2, 3), Fixed64.One, Fixed64.One, Vector2d.Zero, _square);

    [Benchmark]
    public bool RoundedCornerMiss() => FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
        new(2, 2), new(3, 3), Fixed64.Zero, Fixed64.One, Vector2d.Zero, _square);

    [Benchmark]
    public bool StationaryOverlap() => FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
        Vector2d.Zero, Vector2d.Zero, Fixed64.One, Fixed64.Half, Vector2d.Zero, _square);
}
