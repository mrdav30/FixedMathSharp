using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;
using System;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class RigidFiniteShapeRelationBenchmarks
{
    private static readonly Vector3d[] CubePoints =
    {
        new(-1, -1, -1),
        new(1, -1, -1),
        new(1, 1, -1),
        new(-1, 1, -1),
        new(-1, -1, 1),
        new(1, -1, 1),
        new(1, 1, 1),
        new(-1, 1, 1),
    };

    private static readonly int[] CubeTriangles =
    {
        0, 2, 1, 0, 3, 2,
        4, 5, 6, 4, 6, 7,
        0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5,
        0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2,
    };

    private static readonly int[] CubeEdges =
    {
        0, 1, 1, 2, 2, 3, 3, 0,
        4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7,
    };

    private readonly FixedQuaternion _tiltedCylinder =
        FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.PiOver6);
    private readonly FixedQuaternion _tiltedCapsule =
        FixedQuaternion.FromAxisAngle(
            Vector3d.Right,
            Fixed64.PiOver6);
    private readonly FixedQuaternion _irreducibleWideRotation =
        FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            Fixed64.Pi / (Fixed64)4);
    private readonly FixedQuaternion _edgeCapsule =
        FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)(-90));
    private readonly ulong[] _irreducibleRadicands =
    {
        2UL,
        3UL,
        5UL,
        7UL,
        11UL,
    };
    private readonly int[] _irreducibleSigns =
    {
        1,
        -1,
        1,
        -1,
        1,
    };

    [GlobalSetup]
    public void Setup()
    {
        if (!OrdinaryCylinderCapsule()
            || !FullDomainCancellationCylinderCapsule()
            || !IrreducibleWideCylinderCapsule()
            || !OrdinaryCylinderCylinder()
            || !FullDomainCancellationCylinderCylinder()
            || !MultiRadicalCylinderCylinder()
            || IrreducibleGenericRadicalSign() == 0
            || !OrdinaryHullCapsule()
            || !EdgeFeatureHullCapsule()
            || !OrdinaryCapsuleSlabProjection()
            || !OrdinaryCylinderSlabProjection()
            || !OrdinaryConeSlabProjection()
            || !ScalarFaceCapsuleSlabProjection())
        {
            throw new InvalidOperationException(
                "Rigid finite-shape benchmark scenarios must retain their intended contacts.");
        }
    }

    [Benchmark(Baseline = true)]
    public bool OrdinaryCylinderCapsule() =>
        FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(
                Fixed64.Half,
                Fixed64.One + Fixed64.Half,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out _);

    [Benchmark]
    public bool IrreducibleWideCylinderCapsule()
    {
        bool result = WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                new Vector3d(
                    Fixed64.FromRaw(-9_223_371_298_328_934_912L),
                    Fixed64.FromRaw(-9_223_371_221_263_323_628L),
                    Fixed64.FromRaw(-9_223_371_427_101_329_995L)),
                _irreducibleWideRotation,
                Vector3d.Right,
                Fixed64.FromRaw(9_223_371_717_157_954_270L),
                Fixed64.FromRaw(360_447_514_427L),
                new Vector3d(
                    Fixed64.FromRaw(-9_223_371_981_293_118_689L),
                    Fixed64.FromRaw(-9_223_371_361_618_948_553L),
                    Fixed64.FromRaw(-9_223_371_712_040_780_011L)),
                _irreducibleWideRotation,
                Vector3d.Forward,
                Fixed64.FromRaw(855_113_133_883L),
                Fixed64.FromRaw(9_223_371_872_063_158_705L),
                out _,
                out bool usedWideCandidate);
        return result && usedWideCandidate;
    }

    [Benchmark]
    public bool FullDomainCancellationCylinderCapsule() =>
        FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            new Vector3d(
                Fixed64.MinValue + Fixed64.FromRaw(2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.MaxValue,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.MaxValue,
            out _);

    [Benchmark]
    public bool OrdinaryCylinderCylinder()
    {
        bool result = WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                new Vector3d(
                    Fixed64.Half,
                    Fixed64.One + Fixed64.Half,
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                out _,
                out bool usedWideCandidate,
                out bool usedMultiRadicalRanking);
        return result
            && !usedWideCandidate
            && !usedMultiRadicalRanking;
    }

    [Benchmark]
    public bool MultiRadicalCylinderCylinder()
    {
        bool result = WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero,
                _tiltedCylinder,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                new Vector3d(-2, -2, -1),
                _tiltedCapsule,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                out _,
                out bool usedWideCandidate,
                out bool usedMultiRadicalRanking);
        return result
            && !usedWideCandidate
            && usedMultiRadicalRanking;
    }

    [Benchmark]
    public bool FullDomainCancellationCylinderCylinder() =>
        FixedSegment.TryGetCenteredFiniteCylindersContact(
            new Vector3d(
                Fixed64.MinValue + Fixed64.FromRaw(2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.MaxValue,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.MaxValue,
            out _);

    [Benchmark]
    public int IrreducibleGenericRadicalSign() =>
        WideArithmetic.GetLinearRadicalSumSign(
            _irreducibleRadicands,
            radicandWordCount: 1,
            _irreducibleSigns);

    [Benchmark]
    public bool OrdinaryHullCapsule() =>
        FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out _);

    [Benchmark]
    public bool EdgeFeatureHullCapsule() =>
        FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(27, 20),
                Fixed64.FromFraction(27, 20)),
            _edgeCapsule,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out _);

    [Benchmark]
    public bool OrdinaryCapsuleSlabProjection() =>
        FixedSegment.TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            Vector3d.Right * Fixed64.FromFraction(3, 4),
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _);

    [Benchmark]
    public bool OrdinaryCylinderSlabProjection() =>
        FixedSegment.TryGetCenteredFiniteCylinderCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            Vector3d.Right * Fixed64.FromFraction(3, 4),
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _);

    [Benchmark]
    public bool OrdinaryConeSlabProjection() =>
        FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            Vector3d.Right * Fixed64.FromFraction(3, 4),
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _);

    [Benchmark]
    public bool ScalarFaceCapsuleSlabProjection() =>
        FixedSegment.TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Right
                * (Fixed64.MaxValue - Fixed64.FromFraction(1, 4)),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            Vector3d.Right * Fixed64.MaxValue,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _);
}
