//=======================================================================
// CapsulePrismBoundaryContacts.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Reuses finite capsule events over one already admitted native boundary stratum.</summary>
internal sealed class CapsulePrismBoundaryContacts
{
    private readonly CapsulePlaneRayEventVisitor _visitor;
    private ulong[] _values = Array.Empty<ulong>();
    private readonly int[] _signs = new int[CapsulePlaneRaySelection.SignCount];
    private readonly sbyte[] _valueSigns = new sbyte[CapsulePlaneRaySelection.ValueSignCount];
    private CapsulePlaneRaySelectionMetadata _winner;
    private CapsulePlaneRayFrame _frame;
    private Vector3d _center, _origin;
    private FixedQuaternion _rotation;
    private Fixed64 _length, _radius, _lowerY, _upperY, _yawAngle;
    private Vector2d _start, _end;
    private WideRationalBasis3d _basis;
    private WideConvex2dRelations.RotationFrame2d _yaw;
    private Signed320 _baseD, _lowerN, _lowerD, _upperN, _upperD;
    private WideDirection3d _worldNormal;
    private RationalSegmentInterval _clip, _support;
    private bool _supportPrepared, _hasSupport;

    internal CapsulePrismBoundaryContacts() => _visitor = Keep;

    /// <summary>Restores one admitted source chart for connectivity without repeating finite event selection.</summary>
    internal bool PrepareSource(Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius,
        Vector3d origin, Fixed64 yaw, Vector2d start, Vector2d end, Vector2d inside,
        in RationalSegmentInterval clip, Fixed64 lowerY, Fixed64 upperY, int verticalNormal,
        out WideDirection3d localNormal)
    {
        localNormal = default; _supportPrepared = _hasSupport = false;
        if (lowerY > upperY || start == end) return false;
        _center = center; _origin = origin; _rotation = rotation; _length = length; _radius = radius;
        _start = start; _end = end; _lowerY = lowerY; _upperY = upperY; _clip = clip; _yawAngle = yaw;
        _basis = new WideRationalBasis3d(rotation); _yaw = new WideConvex2dRelations.RotationFrame2d(yaw);
        _baseD = WideArithmetic.MultiplySigned192(Signed192.One, _basis.Denominator);
        _lowerN = default; _lowerD = _upperN = _upperD = Signed320.ExtendValue(Signed192.Signed(1));
        if (clip.IsClipped)
        {
            ReduceParameter(clip.WideLowerNumerator, clip.WideLowerDenominator, out _lowerN, out _lowerD);
            ReduceParameter(clip.WideUpperNumerator, clip.WideUpperDenominator, out _upperN, out _upperD);
        }
        if (!GetNormal(inside, verticalNormal, out localNormal)) return false;
        if (lowerY != upperY && (!clip.IsClipped || RationalSegmentInterval.CompareParameters(_lowerN, _lowerD, _upperN, _upperD) != 0))
            throw new ArgumentException("A boundary stratum must be a line or point.", nameof(clip));
        return true;
    }

    /// <summary>Finds the deepest physical ray on one fully admitted closed line or point.</summary>
    /// <remarks>
    /// At least one of the height range and tangent interval must be degenerate. The caller proves
    /// continuity/incident admission over the complete stratum before numerical event selection.
    /// A collapsed height and tangent interval represent an isolated boundary point.
    /// </remarks>
    internal bool TryGetContact(Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius,
        Vector3d origin, Fixed64 yaw, Vector2d start, Vector2d end, Vector2d inside,
        in RationalSegmentInterval clip, Fixed64 lowerY, Fixed64 upperY, int verticalNormal,
        out FixedContactAnchors contact)
    {
        contact = default; _winner = default;
        if (!PrepareSource(center, rotation, length, radius, origin, yaw, start, end, inside, clip,
            lowerY, upperY, verticalNormal, out WideDirection3d localNormal)) return false;
        bool horizontal = lowerY == upperY;
        Signed320 zero = default, one = Signed320.ExtendValue(Signed192.Signed(1));
        WideAxis3 first = GetPoint(horizontal ? zero : _lowerN, horizontal ? one : _lowerD, lowerY, out Signed320 denominator);
        WideAxis3 last = GetPoint(horizontal ? one : _lowerN, horizontal ? one : _lowerD, upperY, out Signed320 lastDenominator);
        System.Diagnostics.Debug.Assert(lastDenominator.Equals(denominator));
        // Native/quaternion products retain removable common scale. Reduce the
        // complete line chart together, preserving both points over one exact D.
        Span<Signed320> coordinates = stackalloc Signed320[7]
            { first.X, first.Y, first.Z, last.X, last.Y, last.Z, denominator };
        WideArithmetic.ReduceCommonScale(coordinates);
        first = new(coordinates[0], coordinates[1], coordinates[2]);
        last = new(coordinates[3], coordinates[4], coordinates[5]); denominator = coordinates[6];
        _frame = CapsulePlaneRayFrame.ForCapsulePointRay(length, radius, denominator, localNormal, WideDirection3d.Dot(localNormal, first));
        Span<ulong> magnitude = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(Signed832.ExtendValue(denominator), magnitude);
        int pointBits = WideArithmetic.GetMagnitudeBitLength(magnitude) + 48, normalBits = 0;
        for (int axis = 0; axis < 3; axis++)
        {
            WideArithmetic.GetMagnitude(Signed832.ExtendValue(axis == 0 ? localNormal.X : axis == 1 ? localNormal.Y : localNormal.Z), magnitude);
            normalBits = Math.Max(normalBits, WideArithmetic.GetMagnitudeBitLength(magnitude));
        }
        // Stationary roots need <6P+2N bits and recovered point coefficients
        // <5P+2N. Their complete quadratic-field norm needs <16P+6N.
        if (16 * pointBits + 6 * normalBits + 32 > 64 * CapsulePlaneRaySelection.FieldWords) _frame = _frame.WithWideParameters();
        if (_values.Length < _frame.SelectionStorageWords) Array.Resize(ref _values, _frame.SelectionStorageWords);
        var source = new CapsulePlaneRayEventSource(new FixedSegment(new(start.X, lowerY, start.Y), new(end.X, upperY, end.Y)));
        CapsulePlaneRayEvents.VisitLocalSegmentEvents(source, _frame, first, last, 1, horizontal ? clip : default, _visitor);
        if (!_winner.HasValue) return false;
        var selection = BorrowWinner();
        bool movingRepresented = selection.TryGetExitAnchor(_frame, 1, center, rotation,
            out FixedPointAnchor moving, out Fixed64 depth, out bool clamped);
        bool targetRepresented = selection.TryGetPointInRigidFrame(_frame, origin, FixedQuaternion.Identity,
            center, rotation, out Vector3d target);
        System.Diagnostics.Debug.Assert(movingRepresented && targetRepresented);
        contact = new FixedContactAnchors(moving, new FixedPointAnchor(origin, FixedQuaternion.Identity, target),
            _worldNormal.GetNormal(false), depth, clamped);
        return true;
    }

    /// <summary>Tests whether this finite stratum meets an original support-core interval on its plane.</summary>
    internal bool ConnectsSupport(in RationalSegmentInterval coreInterval)
    {
        if (!_supportPrepared)
        {
            _supportPrepared = true;
            Span<Signed832> points = stackalloc Signed832[6];
            WideConvexPrismRelations.GetProjectedCapsuleSupport(_center, _rotation, _length, _origin,
                _yawAngle, new Vector3d(_start.X, _lowerY, _start.Y), _worldNormal, points, out Signed832 denominator);
            _hasSupport = WideConvexPrismRelations.TryClipProjectedPrismFace(points, denominator, _yawAngle,
                _start, _end, _clip, _lowerY, _upperY, default, out _support);
        }
        return _hasSupport && (!coreInterval.IsClipped
            || RationalSegmentInterval.CompareParameters(_support.WideLowerNumerator, _support.WideLowerDenominator,
                coreInterval.WideUpperNumerator, coreInterval.WideUpperDenominator) <= 0
            && RationalSegmentInterval.CompareParameters(coreInterval.WideLowerNumerator, coreInterval.WideLowerDenominator,
                _support.WideUpperNumerator, _support.WideUpperDenominator) <= 0);
    }

    /// <summary>Tests one exact native tangent/height intersection against the complete capsule.</summary>
    internal bool ContainsPoint(Signed320 tangentN, Signed320 tangentD, Fixed64 height)
    {
        WideAxis3 point = GetPoint(tangentN, tangentD, height, out Signed320 denominator);
        var finite = new CapsuleFiniteSectionFrame(_length, _radius, denominator);
        return finite.ContainsTransformedPoint(point);
    }

    private CapsulePlaneRaySelection BorrowWinner()
    {
        var selection = new CapsulePlaneRaySelection(_values, _signs, _valueSigns, _frame.FieldWords, _frame.RootWords, _frame.ValueWords);
        selection.Restore(_winner); return selection;
    }

    /// <summary>Compares this admitted line maximum with the borrowed native source depth before rounding.</summary>
    internal int CompareDepth(scoped in CapsulePrismContactDepth native)
    {
        System.Diagnostics.Debug.Assert(_winner.HasValue);
        return native.Compare(BorrowWinner(), _frame);
    }

    private void Keep(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        CapsulePlaneRayEvent descriptor, int orientation, scoped in CapsulePlaneRaySelection candidate)
    {
        var winner = BorrowWinner(); winner.KeepEvaluated(candidate, frame); _winner = winner.Metadata;
    }

    private static void ReduceParameter(Signed832 n, Signed832 d, out Signed320 reducedN, out Signed320 reducedD)
    {
        Span<ulong> numerator = stackalloc ulong[13], denominator = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(n, numerator); WideArithmetic.GetMagnitude(d, denominator);
        RationalSegmentInterval.PackReducedParameter(numerator, denominator, out n, out d);
        // Native side joins retain <162-bit coverage ratios before quaternion transformation.
        reducedN = new Signed320(n.Word4, n.Word3, n.Word2, n.Word1, n.Word0);
        reducedD = new Signed320(d.Word4, d.Word3, d.Word2, d.Word1, d.Word0);
    }

    private bool GetNormal(Vector2d inside, int vertical, out WideDirection3d local)
    {
        local = default;
        WideConvex2dRelations.GetRotatedOffset(_yaw, _start, out Signed192 ax, out Signed192 az);
        WideConvex2dRelations.GetRotatedOffset(_yaw, _end, out Signed192 bx, out Signed192 bz);
        Signed320 side = WideArithmetic.MultiplySubtract(WideArithmetic.Difference(_end.X, _start.X),
            WideArithmetic.Difference(inside.Y, _start.Y), WideArithmetic.Difference(_end.Y, _start.Y),
            WideArithmetic.Difference(inside.X, _start.X));
        if (side.IsZero) return false;
        Span<Signed576> normal = stackalloc Signed576[3]
        {
            Signed576.ExtendValue(WideArithmetic.SubtractSigned192(az, bz)), default,
            Signed576.ExtendValue(WideArithmetic.SubtractSigned192(bx, ax))
        };
        if (side.Sign < 0)
            for (int axis = 0; axis < 3; axis++) normal[axis] = WideArithmetic.SubtractSigned576(default, normal[axis]);
        if (vertical != 0) { normal[0] = normal[2] = default; normal[1] = Signed576.ExtendValue(Signed192.Signed(vertical)); }
        WideArithmetic.ReduceCommonScale(normal); _worldNormal = new WideDirection3d(normal[0], normal[1], normal[2]);
        WideRigidProjection.GetLocalAxisProjections(new WideAxis3(Signed320.NarrowValue(normal[0]),
            Signed320.NarrowValue(normal[1]), Signed320.NarrowValue(normal[2])), _basis, out normal[0], out normal[1], out normal[2]);
        WideArithmetic.ReduceCommonScale(normal); local = new WideDirection3d(normal[0], normal[1], normal[2]);
        return true;
    }

    private WideAxis3 GetPoint(Signed320 n, Signed320 d, Fixed64 height, out Signed320 denominator)
    {
        WideConvex2dRelations.GetRotatedOffset(_yaw, _start, out Signed192 ax, out Signed192 az);
        WideConvex2dRelations.GetRotatedOffset(_yaw, _end, out Signed192 bx, out Signed192 bz);
        Signed192 ex = WideArithmetic.SubtractSigned192(bx, ax), ez = WideArithmetic.SubtractSigned192(bz, az);
        ax = WideArithmetic.AddSigned192(ax, Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            WideArithmetic.Difference(_origin.X, _center.X), Signed192.One)));
        az = WideArithmetic.AddSigned192(az, Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            WideArithmetic.Difference(_origin.Z, _center.Z), Signed192.One)));
        Signed192 ay = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(WideArithmetic.AddSigned192(
            WideArithmetic.Difference(_origin.Y, _center.Y), Signed192.Raw(height)), Signed192.One));
        var point = new WideAxis3(Signed320.NarrowValue(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(d, Signed320.ExtendValue(ax)), WideArithmetic.MultiplySigned320(n, Signed320.ExtendValue(ex)))),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(d, Signed320.ExtendValue(ay))),
            Signed320.NarrowValue(WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(d, Signed320.ExtendValue(az)),
                WideArithmetic.MultiplySigned320(n, Signed320.ExtendValue(ez)))));
        WideRigidProjection.GetLocalAxisProjections(point, _basis, out Signed576 x, out Signed576 y, out Signed576 z);
        denominator = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(_baseD, d));
        // Native points <80 bits, compact cuts <162 and the inverse quaternion
        // <65 keep doubled coordinates <309 and their denominator <259 bits.
        return new WideAxis3(Signed320.NarrowValue(WideArithmetic.AddSigned576(x, x)),
            Signed320.NarrowValue(WideArithmetic.AddSigned576(y, y)), Signed320.NarrowValue(WideArithmetic.AddSigned576(z, z)));
    }
}
