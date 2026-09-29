using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class TriangleCapsuleSlabContactBenchmarks
{
    private FixedTriangle _cap, _side, _end, _rim, _gap, _scalarFace;
    private Vector3d _above, _beside, _ahead, _endShift, _scalarOrigin;
    private Fixed64 _oddCore;

    [GlobalSetup]
    public void Setup()
    {
        _cap = new FixedTriangle(new Vector3d(-4, 0, -4),
            new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        _side = new FixedTriangle(new Vector3d(0, -2, -4),
            new Vector3d(0, 2, -4), new Vector3d(0, 0, 4));
        _end = new FixedTriangle(new Vector3d(-4, -4, 0),
            new Vector3d(4, -4, 0), new Vector3d(0, 4, 0));
        _rim = new FixedTriangle(
            new Vector3d(Fixed64.FromFraction(93, 16), (Fixed64)4, Fixed64.FromFraction(5, 4)),
            new Vector3d(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(11, 2), Fixed64.FromFraction(-5, 4)),
            new Vector3d(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(35, 4), Fixed64.Zero));
        _gap = new FixedTriangle(
            new Vector3d(Fixed64.FromFraction(99, 16), Fixed64.FromFraction(9, 2), Fixed64.FromFraction(5, 4)),
            new Vector3d(Fixed64.FromFraction(67, 16), (Fixed64)6, Fixed64.FromFraction(-5, 4)),
            new Vector3d(Fixed64.FromFraction(131, 16), Fixed64.FromFraction(37, 4), Fixed64.Zero));
        _scalarFace = new FixedTriangle(new Vector3d(1, -2, -4),
            new Vector3d(1, 2, -4), new Vector3d(1, 0, 4));
        _above = new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero);
        _beside = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        _ahead = new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half);
        _endShift = _beside;
        _scalarOrigin = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        _oddCore = Fixed64.Two + Fixed64.MinIncrement;

        if (!CapFace() || !StraightSideSeam() || !RoundedEndOddCore()
            || !ObliqueRimOverlap() || !UnmaterializedScalarFace())
            throw new InvalidOperationException("Positive-core slab overlap fixtures must contact.");
        // n=(3/5,4/5,0) leaves a strictly positive 1/80 separating gap.
        if (CertifiedRimGap())
            throw new InvalidOperationException("The certified rim gap must remain separated.");
    }

    [Benchmark]
    public bool CapFace() => _cap.TryGetCenteredCapsuleSlabContact(
        Vector3d.Zero, FixedQuaternion.Identity, _above, Fixed64.Zero,
        Vector2d.Forward, Fixed64.Two, Fixed64.One, Fixed64.One, out _);

    // n is perpendicular to the core: both endpoint regions meet here,
    // and the paired witness uses an interior point of the straight side.
    [Benchmark]
    public bool StraightSideSeam() => _side.TryGetCenteredCapsuleSlabContact(
        Vector3d.Zero, FixedQuaternion.Identity, _beside, Fixed64.Zero,
        Vector2d.Forward, Fixed64.Two, Fixed64.One, Fixed64.One, out _);

    [Benchmark]
    public bool RoundedEndOddCore() => _end.TryGetCenteredCapsuleSlabContact(
        Vector3d.Zero, FixedQuaternion.Identity, _ahead, Fixed64.Zero,
        Vector2d.Forward, _oddCore, Fixed64.One, Fixed64.Two, out _);

    // Shift the cylinder rim fixture onto the positive endpoint of a unit
    // core. Its oblique edge/rim minimum remains 5/16 with inward n=(-3,-4,0)/5.
    [Benchmark]
    public bool ObliqueRimOverlap() => _rim.TryGetCenteredCapsuleSlabContact(
        _endShift, FixedQuaternion.Identity, Vector3d.Zero, Fixed64.Zero,
        Vector2d.Right, Fixed64.One, (Fixed64)5, (Fixed64)5, out _);

    [Benchmark]
    public bool CertifiedRimGap() => _gap.TryGetCenteredCapsuleSlabContact(
        Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, Fixed64.Zero,
        Vector2d.Right, Fixed64.One, (Fixed64)5, (Fixed64)5, out _);

    [Benchmark]
    public bool UnmaterializedScalarFace() => _scalarFace.TryGetCenteredCapsuleSlabContact(
        _scalarOrigin, FixedQuaternion.Identity, _scalarOrigin, Fixed64.Zero,
        Vector2d.Forward, _oddCore, Fixed64.Two, Fixed64.Two, out _);
}
