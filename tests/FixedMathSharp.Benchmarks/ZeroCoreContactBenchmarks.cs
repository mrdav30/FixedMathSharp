using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class ZeroCoreContactBenchmarks
{
    private Vector2d _center;
    private Vector3d _center3d;
    private Fixed64 _radius;
    private Fixed64 _length;
    private Fixed64 _rotation;
    private FixedQuaternion _rotation3d;
    private FixedQuaternion _inverseRotation3d;

    [Params("Axis", "Diagonal", "Rotated", "Coincident", "LargeAxis", "Separated", "CapsuleControl")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _center = Vector2d.Right;
        _radius = Fixed64.One;
        _length = Fixed64.Zero;
        _rotation = Fixed64.Zero;
        Fixed64 expectedDepth = Fixed64.One;
        Vector2d expectedNormal = Vector2d.Right;
        bool expectedHit = true;
        switch (Geometry)
        {
            case "Axis": break;
            case "Diagonal":
                _center = new Vector2d(3, 4);
                _radius = (Fixed64)5;
                expectedDepth = (Fixed64)5;
                expectedNormal = new Vector2d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
                break;
            case "Rotated":
                _rotation = Fixed64.One;
                break;
            case "Coincident":
                _center = Vector2d.Zero;
                expectedDepth = Fixed64.Two;
                break;
            case "LargeAxis":
                _center = new Vector2d(30000, 0);
                _radius = (Fixed64)25000;
                expectedDepth = (Fixed64)20000;
                break;
            case "Separated":
                _center = new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2));
                expectedHit = false;
                expectedDepth = Fixed64.Zero;
                expectedNormal = Vector2d.Zero;
                break;
            case "CapsuleControl":
                _length = Fixed64.Two;
                break;
            default: throw new InvalidOperationException("Unknown contact geometry.");
        }
        _center3d = new Vector3d(_center.X, _center.Y, Fixed64.Zero);
        _rotation3d = FixedQuaternion.FromAxisAngle(Vector3d.Forward, _rotation);
        _inverseRotation3d = _rotation3d.Inverse();
        bool circle = Circle(out FixedContactAnchors2d circleContact);
        bool sphere = Sphere(out FixedContactAnchors sphereContact);
        if (circle != expectedHit || sphere != expectedHit ||
            circleContact.Depth != expectedDepth || sphereContact.Depth != expectedDepth ||
            circleContact.Normal != expectedNormal ||
            sphereContact.Normal != new Vector3d(expectedNormal.X, expectedNormal.Y, Fixed64.Zero) ||
            circleContact.DepthIsClamped || sphereContact.DepthIsClamped)
            throw new InvalidOperationException($"Incorrect contact for {Geometry}.");
    }

    [Benchmark]
    public bool Circle() => Circle(out _);

    [Benchmark]
    public bool Sphere() => Sphere(out _);

    private bool Circle(out FixedContactAnchors2d contact) =>
        FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero, _rotation, Vector2d.Forward, _length, _radius,
            _center, -_rotation, Vector2d.Forward, _length, _radius,
            Vector2d.Right, out contact);

    private bool Sphere(out FixedContactAnchors contact) =>
        FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero, _rotation3d, Vector3d.Up, _length, _radius,
            _center3d, _inverseRotation3d, Vector3d.Up, _length, _radius,
            Vector3d.Right, out contact);
}
