using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;
using System;

namespace FixedMathSharp.Benchmarks;

/// <summary>
/// Measures semantic-anchor contact reduction with and without compact
/// manifold reconstruction.
/// </summary>
[MemoryDiagnoser]
public class OrientedBoxAnchorBenchmarks
{
    private static readonly Vector3d[] HullPoints =
    {
        new(-1, -1, -1),
        new(1, -1, -1),
        new(1, 1, -1),
        new(-1, 1, -1),
        new(-1, -1, 1),
        new(1, -1, 1),
        new(1, 1, 1),
        new(-1, 1, 1)
    };

    private static readonly int[] HullTriangles =
    {
        0, 2, 1, 0, 3, 2,
        4, 5, 6, 4, 6, 7,
        0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2,
        0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5
    };

    private static readonly int[] HullEdges =
    {
        0, 1, 1, 2, 2, 3, 3, 0,
        4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7
    };

    private readonly FixedOrientedBox _box = new(
        Vector3d.Zero,
        FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)7,
            (Fixed64)19,
            (Fixed64)(-11)),
        new Vector3d(2, 1, 3));
    private readonly FixedTriangle _triangle = new(
        new Vector3d(-3, 0, -3),
        new Vector3d(3, 0, -3),
        new Vector3d(0, 0, 3));
    private readonly FixedQuaternion _triangleRotation =
        FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4);
    private readonly Signed576 _leftRadialRational = ToSigned576(-2L);
    private readonly Signed832 _leftRadialNumerator = ToSigned832(19L);
    private readonly Signed576 _leftRadialDenominator = ToSigned576(2L);
    private readonly Signed576 _leftRadialAxisSquared = ToSigned576(7L);
    private readonly Signed576 _rightRadialRational = ToSigned576(4L);
    private readonly Signed832 _rightRadialNumerator = ToSigned832(7L);
    private readonly Signed576 _rightRadialDenominator = ToSigned576(11L);
    private readonly Signed576 _rightRadialAxisSquared = ToSigned576(3L);

    [Benchmark(Baseline = true)]
    public bool TrianglePrimary() =>
        _box.TryGetTriangleContact(
            new Vector3d(0, 1, 0),
            _triangleRotation,
            _triangle,
            out _);

    [Benchmark]
    public bool TriangleManifold()
    {
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];
        return _box.TryGetTriangleContact(
            new Vector3d(0, 1, 0),
            _triangleRotation,
            _triangle,
            contacts,
            out _,
            out _);
    }

    [Benchmark]
    public bool CylinderManifold()
    {
        Span<FixedContactLocalPoints> contacts =
            stackalloc FixedContactLocalPoints[4];
        return _box.TryGetCenteredCylinderContact(
            new Vector3d(0, 1, 0),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            contacts,
            out _,
            out _);
    }

    [Benchmark]
    public bool SpherePrimary() =>
        _box.TryGetSphereContact(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.FromFraction(3, 2)),
            _triangleRotation,
            Fixed64.One,
            out _);

    [Benchmark]
    public bool CircleSlabPrimary() =>
        _box.TryGetCircleSlabContact(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.FromFraction(3, 2)),
            Fixed64.PiOver4,
            Fixed64.One,
            Fixed64.One,
            out _);

    [Benchmark]
    public bool CapsuleSlabPrimary() =>
        _box.TryGetCenteredCapsuleSlabContact(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            Fixed64.PiOver4,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.One,
            out _);

    [Benchmark]
    public int RadialProjectionWorstCaseComparison() =>
        WideArithmetic.CompareRadialProjectionDepths(
            _leftRadialRational,
            _leftRadialNumerator,
            _leftRadialDenominator,
            _leftRadialAxisSquared,
            _rightRadialRational,
            _rightRadialNumerator,
            _rightRadialDenominator,
            _rightRadialAxisSquared,
            Signed192.Signed(7L));

    [Benchmark]
    public bool ConvexHullPrimary() =>
        _box.TryGetConvexHullContact(
            new Vector3d(
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            HullPoints,
            HullTriangles,
            HullEdges,
            out _);

    private static Signed576 ToSigned576(long value) =>
        Signed576.ExtendValue(
            Signed320.ExtendValue(
                Signed192.Signed(value)));

    private static Signed832 ToSigned832(long value) =>
        Signed832.ExtendValue(ToSigned576(value));
}
