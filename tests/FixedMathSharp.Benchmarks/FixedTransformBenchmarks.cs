using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

/// <summary>
/// Measures allocation-free transform reads and mutations. Hierarchy reads are linear in depth.
/// </summary>
[MemoryDiagnoser]
public class FixedTransformBenchmarks
{
    private readonly FixedTransform _root;
    private readonly FixedTransform _depthEight;
    private readonly FixedTransform _reparentParentA;
    private readonly FixedTransform _reparentParentB;
    private readonly FixedTransform _successfulChild;
    private readonly FixedTransform _singularParent;
    private readonly FixedTransform _failedChild;

    public FixedTransformBenchmarks()
    {
        _root = new FixedTransform(
            new Vector3d((Fixed64)1, (Fixed64)2, (Fixed64)3),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));

        _depthEight = _root;
        for (int i = 0; i < 8; i++)
        {
            _depthEight = new FixedTransform(
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
                FixedQuaternion.Identity,
                Vector3d.One,
                _depthEight);
        }

        _reparentParentA = new FixedTransform(Vector3d.Right, FixedQuaternion.Identity, Vector3d.One);
        _reparentParentB = new FixedTransform(Vector3d.Right, FixedQuaternion.Identity, Vector3d.One);
        _successfulChild = new FixedTransform(
            Vector3d.Forward,
            FixedQuaternion.Identity,
            Vector3d.One,
            _reparentParentA);
        _singularParent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero);
        _failedChild = new FixedTransform(Vector3d.Forward, FixedQuaternion.Identity, Vector3d.One);
    }

    [Benchmark]
    public Fixed4x4 RootWorldMatrix() => _root.LocalToWorldMatrix;

    [Benchmark]
    public Vector3d RootWorldPosition() => _root.WorldPosition;

    [Benchmark]
    public FixedQuaternion RootWorldRotation() => _root.WorldRotation;

    [Benchmark]
    public Vector3d RootLossyScale() => _root.LossyScale;

    [Benchmark]
    public Fixed4x4 DepthEightWorldMatrix() => _depthEight.LocalToWorldMatrix;

    [Benchmark]
    public Vector3d DepthEightLossyScale() => _depthEight.LossyScale;

    [Benchmark]
    public bool SuccessfulReparent() =>
        _successfulChild.TrySetParentKeepingWorld(
            ReferenceEquals(_successfulChild.Parent, _reparentParentA)
                ? _reparentParentB
                : _reparentParentA);

    [Benchmark]
    public bool FailedReparent() => _failedChild.TrySetParentKeepingWorld(_singularParent);
}
