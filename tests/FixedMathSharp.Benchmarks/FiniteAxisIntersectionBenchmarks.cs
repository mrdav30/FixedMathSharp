using BenchmarkDotNet.Attributes;
using FixedMathSharp.Bounds;
using System;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class FiniteAxisIntersectionBenchmarks
{
    private FixedSegment2d _query2D;
    private FixedSegment2d _capsuleAxis2D;
    private FixedSegment _query3D;
    private FixedSegment _capsuleAxis3D;
    private FixedSegment _cylinderAxis;
    private FixedSegment _expandedCylinderQuery;
    private Vector2d _capsuleCenter2D;
    private Vector2d _capsuleDirection2D;
    private Vector3d _capsuleCenter3D;
    private Vector3d _capsuleDirection3D;
    private Vector2d _distancePoint2D;
    private Vector2d _distanceDirection2D;
    private Vector3d _distancePoint3D;
    private Vector3d _distanceDirection3D;
    private Vector2d _wideCapsuleCenter2D;
    private Vector2d _wideCapsulePoint2D;
    private Vector2d _wideCapsuleDirection2D;
    private Fixed64 _wideAxisHalfLength;
    private Fixed64 _wideRadius;
    private Fixed64 _wideExpansion;
    private Vector3d _cylinderCenter;
    private Vector3d _cylinderDirection;
    private FixedSegment _arbitraryRawQuery;
    private FixedSegment _arbitraryRawAxis;
    private FixedRay2d _boundedRay2D;
    private FixedRay _boundedRay3D;
    private Fixed64 _boundedRayMaximum;
    private Fixed64 _pointParameter;
    private Fixed64 _radius;
    private Fixed64 _axisHalfLength;
    private Fixed64 _radialExpansion;
    private Fixed64 _axialExpansion;
    private Fixed64 _arbitraryRawRadius;

    [Params(1, 100_000)]
    public int Scale { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        Fixed64 scale = (Fixed64)Scale;
        Fixed64 doubleScale = scale * 2;
        _radius = scale * Fixed64.Half;
        _axisHalfLength = scale;
        _radialExpansion = scale * Fixed64.Quarter;
        _axialExpansion = scale * Fixed64.Half;

        _query2D = new FixedSegment2d(
            new Vector2d(-doubleScale, Fixed64.Zero),
            new Vector2d(doubleScale, Fixed64.Zero));
        _capsuleAxis2D = new FixedSegment2d(
            new Vector2d(Fixed64.Zero, -scale),
            new Vector2d(Fixed64.Zero, scale));
        _capsuleCenter2D = Vector2d.Zero;
        _capsuleDirection2D = Vector2d.Forward;
        _query3D = new FixedSegment(
            new Vector3d(-doubleScale, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(doubleScale, Fixed64.Zero, Fixed64.Zero));
        _capsuleAxis3D = new FixedSegment(
            new Vector3d(Fixed64.Zero, -scale, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, scale, Fixed64.Zero));
        _capsuleCenter3D = Vector3d.Zero;
        _capsuleDirection3D = Vector3d.Up;
        _distancePoint2D = new Vector2d(
            scale * Fixed64.FromFraction(3, 2),
            -scale * Fixed64.Half);
        _distanceDirection2D = new Vector2d(Fixed64.One, Fixed64.Two).Normalized;
        _distancePoint3D = new Vector3d(_distancePoint2D.X, _distancePoint2D.Y, Fixed64.Zero);
        _distanceDirection3D = new Vector3d(
            _distanceDirection2D.X,
            _distanceDirection2D.Y,
            Fixed64.Zero);
        _wideCapsuleCenter2D = new Vector2d(Fixed64.MaxValue - (Fixed64)5 * scale, Fixed64.Zero);
        _wideCapsulePoint2D = new Vector2d(Fixed64.MaxValue - scale, (Fixed64)8 * scale);
        _wideCapsuleDirection2D = new Vector2d(Fixed64.One, Fixed64.One).Normalized;
        _wideAxisHalfLength = (Fixed64)10 * scale;
        _wideRadius = (Fixed64)2 * scale;
        _wideExpansion = scale;
        _cylinderAxis = _capsuleAxis3D;
        _cylinderCenter = Vector3d.Zero;
        _cylinderDirection = Vector3d.Up;
        _expandedCylinderQuery = new FixedSegment(
            new Vector3d(-doubleScale, scale + _radialExpansion, Fixed64.Zero),
            new Vector3d(doubleScale, scale + _radialExpansion, Fixed64.Zero));

        Fixed64 oneRaw = Fixed64.FromRaw(1);
        _arbitraryRawRadius = scale * Fixed64.Half;
        _arbitraryRawQuery = new FixedSegment(
            new Vector3d(-doubleScale + oneRaw, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(doubleScale, Fixed64.Zero, Fixed64.Zero));
        _arbitraryRawAxis = new FixedSegment(
            new Vector3d(Fixed64.Zero, -scale, -scale),
            new Vector3d(Fixed64.Zero, scale, scale + oneRaw));
        _boundedRay2D = new FixedRay2d(_query2D.Start, Vector2d.Right);
        _boundedRay3D = new FixedRay(_query3D.Start, Vector3d.Right);
        _boundedRayMaximum = (Fixed64)4 * scale;
        _pointParameter = scale;

        if (!Capsule2DIntersectionInterval()
            || !Capsule3DIntersectionInterval()
            || !CenteredCapsule2DIntersectionInterval()
            || !CenteredCapsule3DIntersectionInterval()
            || CenteredCapsule2DDistance() == Fixed64.MaxValue
            || CenteredCapsule3DDistance() == Fixed64.MaxValue
            || CenteredCapsule2DWideDistance() == Fixed64.MaxValue
            || !CenteredCapsule2DContainsWidePoint()
            || CenteredCapsule2DNormalAndSurface() == Vector2d.Zero
            || !FiniteCylinderIntersectionInterval()
            || !AffineExpandedCylinderIntersectionInterval()
            || !CenteredExpandedCylinderIntersectionInterval()
            || !ArbitraryRawCylinderIntersectionInterval()
            || !BoundedRayCapsule2DIntersectionInterval()
            || !BoundedRayCapsule3DIntersectionInterval()
            || !BoundedRayCenteredCapsule2DIntersectionInterval()
            || !BoundedRayCenteredCapsule3DIntersectionInterval()
            || !BoundedRayFiniteCylinderIntersectionInterval()
            || !CenteredCapsule2DDistanceInterval()
            || !CenteredCapsule3DDistanceInterval()
            || !FiniteCylinderDistanceInterval())
        {
            throw new InvalidOperationException("Finite-axis benchmark scenarios must intersect their targets.");
        }
    }

    [Benchmark]
    public bool Capsule2DIntersectionInterval() =>
        _query2D.TryGetCapsuleIntersectionInterval(
            _capsuleAxis2D,
            _radius,
            Fixed64.Zero,
            out _,
            out _);

    [Benchmark]
    public bool Capsule3DIntersectionInterval() =>
        _query3D.TryGetCapsuleIntersectionInterval(
            _capsuleAxis3D,
            _radius,
            Fixed64.Zero,
            out _,
            out _);

    [Benchmark]
    public bool CenteredCapsule2DIntersectionInterval() =>
        _query2D.TryGetCapsuleIntersectionInterval(
            _capsuleCenter2D,
            _capsuleDirection2D,
            _axisHalfLength,
            _radius,
            Fixed64.Zero,
            out _,
            out _);

    [Benchmark]
    public bool CenteredCapsule3DIntersectionInterval() =>
        _query3D.TryGetCapsuleIntersectionInterval(
            _capsuleCenter3D,
            _capsuleDirection3D,
            _axisHalfLength,
            _radius,
            Fixed64.Zero,
            out _,
            out _);

    [Benchmark]
    public Fixed64 CenteredCapsule2DDistance() =>
        FixedSegment2d.GetDistanceToCenteredCapsule(
            _distancePoint2D,
            _capsuleCenter2D,
            _distanceDirection2D,
            _axisHalfLength,
            _radius);

    [Benchmark]
    public Fixed64 CenteredCapsule3DDistance() =>
        FixedSegment.GetDistanceToCenteredCapsule(
            _distancePoint3D,
            _capsuleCenter3D,
            _distanceDirection3D,
            _axisHalfLength,
            _radius);

    [Benchmark]
    public Fixed64 CenteredCapsule2DWideDistance() =>
        FixedSegment2d.GetDistanceToCenteredCapsule(
            _wideCapsulePoint2D,
            _wideCapsuleCenter2D,
            _wideCapsuleDirection2D,
            _wideAxisHalfLength,
            _wideRadius);

    [Benchmark]
    public bool CenteredCapsule2DContainsWidePoint() =>
        FixedSegment2d.ContainsPointInCenteredCapsule(
            _wideCapsulePoint2D,
            _wideCapsuleCenter2D,
            _wideCapsuleDirection2D,
            _wideAxisHalfLength,
            _wideRadius,
            _wideExpansion);

    [Benchmark]
    public Vector2d CenteredCapsule2DNormalAndSurface()
    {
        Vector2d direction = FixedSegment2d.GetDirectionFromCenteredAxis(
            _wideCapsulePoint2D,
            _wideCapsuleCenter2D,
            _wideCapsuleDirection2D,
            _wideAxisHalfLength);
        return FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            _wideCapsulePoint2D,
            _wideCapsuleCenter2D,
            _wideCapsuleDirection2D,
            _wideAxisHalfLength,
            _wideRadius,
            direction,
            out Vector2d surfacePoint)
            ? surfacePoint
            : Vector2d.Zero;
    }

    [Benchmark]
    public bool FiniteCylinderIntersectionInterval() =>
        _query3D.TryGetFiniteCylinderIntersectionInterval(
            _cylinderAxis,
            _radius,
            Fixed64.Zero,
            out _,
            out _);

    [Benchmark]
    public bool AffineExpandedCylinderIntersectionInterval() =>
        _expandedCylinderQuery.TryGetFiniteCylinderIntersectionInterval(
            _cylinderAxis,
            _axisHalfLength,
            _radius,
            _radialExpansion,
            _axialExpansion,
            out _,
            out _);

    [Benchmark]
    public bool CenteredExpandedCylinderIntersectionInterval() =>
        _expandedCylinderQuery.TryGetFiniteCylinderIntersectionInterval(
            _cylinderCenter,
            _cylinderDirection,
            _axisHalfLength,
            _radius,
            _radialExpansion,
            _axialExpansion,
            out _,
            out _);

    [Benchmark]
    public bool ArbitraryRawCylinderIntersectionInterval() =>
        _arbitraryRawQuery.TryGetFiniteCylinderIntersectionInterval(
            _arbitraryRawAxis,
            _arbitraryRawRadius,
            Fixed64.Zero,
            out _,
            out _);

    [Benchmark]
    public bool BoundedRayCapsule2DIntersectionInterval() =>
        _boundedRay2D.TryGetCapsuleIntersectionInterval(
            _capsuleAxis2D,
            _radius,
            _boundedRayMaximum,
            out _,
            out _);

    [Benchmark]
    public bool BoundedRayCapsule3DIntersectionInterval() =>
        _boundedRay3D.TryGetCapsuleIntersectionInterval(
            _capsuleAxis3D,
            _radius,
            _boundedRayMaximum,
            out _,
            out _);

    [Benchmark]
    public bool BoundedRayCenteredCapsule2DIntersectionInterval() =>
        _boundedRay2D.TryGetCapsuleIntersectionInterval(
            _capsuleCenter2D,
            _capsuleDirection2D,
            _axisHalfLength,
            _radius,
            _boundedRayMaximum,
            out _,
            out _);

    [Benchmark]
    public bool BoundedRayCenteredCapsule3DIntersectionInterval() =>
        _boundedRay3D.TryGetCapsuleIntersectionInterval(
            _capsuleCenter3D,
            _capsuleDirection3D,
            _axisHalfLength,
            _radius,
            _boundedRayMaximum,
            out _,
            out _);

    [Benchmark]
    public bool BoundedRayFiniteCylinderIntersectionInterval() =>
        _boundedRay3D.TryGetFiniteCylinderIntersectionInterval(
            _cylinderAxis,
            _radius,
            _boundedRayMaximum,
            out _,
            out _);

    [Benchmark]
    public Vector3d BoundedRayGetPoint() =>
        _boundedRay3D.GetPoint(_pointParameter);

    [Benchmark]
    public bool CenteredCapsule2DDistanceInterval() =>
        _query2D.TryGetCapsuleIntersectionDistanceInterval(
            _capsuleCenter2D,
            _capsuleDirection2D,
            _axisHalfLength,
            _radius,
            Fixed64.Zero,
            _boundedRayMaximum,
            out _,
            out _,
            out _,
            out _);

    [Benchmark]
    public bool CenteredCapsule3DDistanceInterval() =>
        _query3D.TryGetCapsuleIntersectionDistanceInterval(
            _capsuleCenter3D,
            _capsuleDirection3D,
            _axisHalfLength,
            _radius,
            Fixed64.Zero,
            _boundedRayMaximum,
            out _,
            out _,
            out _,
            out _);

    [Benchmark]
    public bool FiniteCylinderDistanceInterval() =>
        _query3D.TryGetFiniteCylinderIntersectionDistanceInterval(
            _cylinderAxis,
            _radius,
            Fixed64.Zero,
            _boundedRayMaximum,
            out _,
            out _,
            out _,
            out _);

    [Benchmark]
    public Vector3d SegmentGetPointAtDistance() =>
        _query3D.GetPointAtDistance(_pointParameter, _boundedRayMaximum);
}
