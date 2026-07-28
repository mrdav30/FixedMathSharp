using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;
using System;

namespace FixedMathSharp.Benchmarks;

/// <summary>
/// Measures exact point-anchor materialization and frame re-expression used by
/// collision and query loops.
/// </summary>
[MemoryDiagnoser]
public class PointAnchorBenchmarks
{
    private readonly FixedPointAnchor _anchor = new(
        new Vector3d(5, -3, 7),
        FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
        new Vector3d(2, 1, -4),
        new Vector3d(Fixed64.Half, -Fixed64.Half, Fixed64.One));
    private readonly Vector3d _frameOrigin = new(1, 2, 3);
    private readonly FixedQuaternion _frameRotation =
        FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4);
    private readonly FixedPointAnchor _other = new(
        new Vector3d(1, 2, 3),
        FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4),
        new Vector3d(-2, 4, 1),
        new Vector3d(-Fixed64.Half, Fixed64.One, Fixed64.Half));
    private readonly FixedPointAnchor2d _anchor2d = new(
        new Vector2d(5, -3),
        Fixed64.PiOver3,
        new Vector2d(2, 1),
        new Vector2d(-Fixed64.Half, Fixed64.FromFraction(1, 4)));
    private readonly FixedPointAnchor2d _other2d = new(
        new Vector2d(1, 2),
        -Fixed64.PiOver4,
        new Vector2d(-2, 4),
        new Vector2d(Fixed64.Half, -Fixed64.One));
    private readonly FixedPointAnchor _scalarFaceAnchor = new(
        new Vector3d(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero),
        FixedQuaternion.Identity,
        new Vector3d(
            Fixed64.MinIncrement,
            Fixed64.Zero,
            Fixed64.Zero));
    private readonly FixedPointAnchor _worldOrigin = new(
        Vector3d.Zero,
        FixedQuaternion.Identity,
        Vector3d.Zero);
    private readonly Vector3d _representableLeverVector;
    private readonly FixedLever _representableLever;
    private readonly FixedLever _scalarFaceLever;
    private readonly FixedLever2d _scalarFaceLever2d =
        new FixedPointAnchor2d(
            new Vector2d(
                Fixed64.MaxValue,
                Fixed64.Zero),
            Fixed64.Zero,
            new Vector2d(
                Fixed64.MinIncrement,
                Fixed64.Zero))
        .GetLeverFrom(default);
    private readonly Fixed3x3 _tinyForwardScale = new(
        Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, Fixed64.MinIncrement);

    public PointAnchorBenchmarks()
    {
        if (!_anchor.TryGetOffsetFrom(
                _other,
                out _representableLeverVector)
            || !_anchor.TryGetLeverFrom(
                _other,
                out _representableLever))
        {
            throw new InvalidOperationException(
                "The representable point-anchor fixture is invalid.");
        }
        if (!_scalarFaceAnchor.TryGetLeverFrom(
                _worldOrigin,
                out _scalarFaceLever))
        {
            throw new InvalidOperationException(
                "The full-domain point-anchor fixture is invalid.");
        }
    }

    [Benchmark(Baseline = true)]
    public bool MaterializeWorldPoint() =>
        _anchor.TryGetPoint(out _);

    [Benchmark]
    public bool ReexpressInRigidFrame() =>
        _anchor.TryGetLocalPointIn(
            _frameOrigin,
            _frameRotation,
            out _);

    [Benchmark]
    public bool ScaledRelativeOffset() =>
        _anchor.TryGetScaledOffsetFrom(
            _other,
            Fixed64.Half,
            out _);

    [Benchmark]
    public bool ProjectedRelativeOffset() =>
        _anchor.TryGetProjectedOffsetFrom(
            _other,
            Vector3d.Right,
            out _);

    [Benchmark]
    public bool RelativeOffset2d() =>
        _anchor2d.TryGetOffsetFrom(_other2d, out _);

    [Benchmark]
    public bool ReexpressInRotatedFrame2d() =>
        _anchor2d.TryGetLocalPointIn(
            _other2d.Origin,
            _other2d.Rotation,
            out _);

    [Benchmark]
    public Fixed64 CompactCrossProjection() =>
        Vector3d.Dot(
            Vector3d.Cross(_representableLeverVector, Vector3d.Up),
            Vector3d.Forward);

    [Benchmark]
    public bool ExactRepresentableCrossProjection() =>
        _representableLever.TryGetCrossProductProjection(
            Vector3d.Up,
            Vector3d.Forward,
            out _);

    [Benchmark]
    public bool CreateFullDomainLever() =>
        _scalarFaceAnchor.TryGetLeverFrom(
            _worldOrigin,
            out _);

    [Benchmark]
    public bool FullDomainCrossProjection() =>
        _scalarFaceLever.TryGetCrossProductProjection(
            Vector3d.Up,
            Vector3d.Forward * Fixed64.Half,
            out _);

    [Benchmark]
    public bool FullDomainQuadraticForm() =>
        _scalarFaceLever.TryGetCrossProductQuadraticForm(
            Vector3d.Up,
            _tinyForwardScale,
            out _);

    [Benchmark]
    public bool FullDomainTransformedCross() =>
        _scalarFaceLever.TryGetTransformedScaledCrossProduct(
            Vector3d.Up,
            Fixed3x3.Identity,
            Fixed64.Half,
            Fixed64.One,
            out _);

    [Benchmark]
    public bool FullDomainSquaredCross2d() =>
        _scalarFaceLever2d.TryGetScaledSquaredCrossProduct(
            Vector2d.Forward,
            Fixed64.MinIncrement,
            out _);
}
