using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

/// <summary>Closed capsule/polygon contacts before public normal and depth rounding.</summary>
[MemoryDiagnoser]
public class CenteredCapsuleConvexBenchmarks
{
    private readonly Vector2d[] _square = { new(-1, -1), new(0, -1), new(0, 0), new(-1, 0) };
    private readonly Vector2d[] _triangle = { new(1, 1), new(2, 3), new(0, 3) };

    [GlobalSetup]
    public void Setup()
    {
        if (!SideOverlap() || !RotatedOverlap() || !CornerTangency() || CornerMiss())
            throw new InvalidOperationException("Capsule contacts must retain exact closed classifications.");
        if (!SideVertexContact().TryGetPoint(out Vector2d witness) || witness != Vector2d.One)
            throw new InvalidOperationException("Capsule side contact must match the opposing vertex.");
    }

    [Benchmark]
    public bool SideOverlap() => FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
        new(Fixed64.Half, -Fixed64.Half), Vector2d.Forward, Fixed64.One, Fixed64.One,
        Vector2d.Zero, _square, out _, out _);

    [Benchmark]
    public bool RotatedOverlap() => FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
        Vector2d.Zero, Vector2d.One.Normalized, Fixed64.One, Fixed64.One,
        Vector2d.Zero, Fixed64.Pi / (Fixed64)6, _square, out _, out _);

    [Benchmark]
    public bool CornerTangency() => FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
        new(3, 4), Vector2d.Forward, Fixed64.Zero, (Fixed64)5,
        Vector2d.Zero, _square, out _, out _);

    [Benchmark]
    public bool CornerMiss() => FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation(
        new(3, 4), Vector2d.Forward, Fixed64.Zero, (Fixed64)5 - Fixed64.MinIncrement,
        Vector2d.Zero, _square, out _, out _);

    [Benchmark]
    public FixedPointAnchor2d SideVertexContact()
    {
        Span<FixedPointAnchor2d> capsule = stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> polygon = stackalloc FixedPointAnchor2d[2];
        _ = FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            Vector2d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One,
            Vector2d.Zero, Fixed64.Zero, _triangle, capsule, polygon,
            out _, out _, out _, out _);
        return capsule[0];
    }
}
