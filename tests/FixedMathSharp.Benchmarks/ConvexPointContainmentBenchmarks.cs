using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

/// <summary>Exact point containment for small translated and rotated convex footprints.</summary>
[MemoryDiagnoser]
public class ConvexPointContainmentBenchmarks
{
    private Vector2d[] _vertices;
    private readonly Vector2d _origin = new(40, 50);
    private readonly Vector2d _outside = new(50, 50);
    private Fixed64 _rotation;

    [Params(4, 6)]
    public int VertexCount { get; set; }

    [Params(0, 30)]
    public int RotationDegrees { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _vertices = VertexCount == 4
            ? new Vector2d[] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) }
            : new Vector2d[]
            {
                new(1, 0), new(Fixed64.Half, Fixed64.One),
                new(-Fixed64.Half, Fixed64.One), new(-1, 0),
                new(-Fixed64.Half, -Fixed64.One), new(Fixed64.Half, -Fixed64.One)
            };
        _rotation = RotationDegrees == 0 ? Fixed64.Zero : Fixed64.PiOver6;
        if (!Inside() || Outside())
            throw new InvalidOperationException("Containment benchmarks must retain their inside/outside classifications.");
    }

    [Benchmark]
    public bool Inside() => FixedConvex2dRelations.ContainsPoint(_origin, _origin, _rotation, _vertices);

    [Benchmark]
    public bool Outside() => FixedConvex2dRelations.ContainsPoint(_outside, _origin, _rotation, _vertices);
}
