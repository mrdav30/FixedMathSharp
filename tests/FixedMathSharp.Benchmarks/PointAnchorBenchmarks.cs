using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

/// <summary>
/// Measures exact point-anchor materialization and frame re-expression used by
/// collision and query loops.
/// </summary>
[MemoryDiagnoser]
public class PointAnchorBenchmarks
{
    private readonly FixedPointAnchor _sameFrameIdentity = new(
        new Vector3d(5, -3, 7),
        FixedQuaternion.Identity,
        new Vector3d(2, 1, -4),
        new Vector3d(Fixed64.Half, -Fixed64.Half, Fixed64.One));
    private readonly FixedPointAnchor _sameFrameIdentityOther = new(
        new Vector3d(5, -3, 7),
        FixedQuaternion.Identity,
        new Vector3d(-2, 4, 1));
    private readonly FixedPointAnchor _identityFrameOtherOrigin = new(
        new Vector3d(-1, 4, 3),
        FixedQuaternion.Identity,
        new Vector3d(-2, 4, 1));
    private readonly FixedPointAnchor _sameFrameRotated = new(
        new Vector3d(5, -3, 7),
        FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
        new Vector3d(2, 1, -4),
        new Vector3d(Fixed64.Half, -Fixed64.Half, Fixed64.One));
    private readonly FixedPointAnchor _sameFrameRotatedOther = new(
        new Vector3d(5, -3, 7),
        FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
        new Vector3d(-2, 4, 1));
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
    public bool SameFrameIdentityOffset() =>
        _sameFrameIdentity.TryGetOffsetFrom(
            _sameFrameIdentityOther,
            out _);

    [Benchmark]
    public bool IdentityFrameOffset() =>
        _sameFrameIdentity.TryGetOffsetFrom(
            _identityFrameOtherOrigin,
            out _);

    [Benchmark]
    public bool SameFrameRotatedOffset() =>
        _sameFrameRotated.TryGetOffsetFrom(
            _sameFrameRotatedOther,
            out _);

    [Benchmark]
    public bool ReexpressInRotatedFrame2d() =>
        _anchor2d.TryGetLocalPointIn(
            _other2d.Origin,
            _other2d.Rotation,
            out _);

}
