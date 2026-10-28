// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Streams existing exact cone events on one retained native boundary line.</summary>
internal sealed class ConePrismBoundaryContacts
{
    private readonly ConePlaneRayEventVisitor _visit;
    private ulong[] _values = Array.Empty<ulong>();
    private readonly int[] _signs = new int[ConePlaneRaySelection.SignCount];
    private bool _hasValue;
    private ConePlaneRayFrame _frame;
    private ConePlaneRayEventSource _source;
    private ConePlaneRayEvent _event;

    internal ConePrismBoundaryContacts() { _visit = Visit; }

    // Side lines are horizontal or have a singleton tangent parameter.
    // A cap line identifies its original footprint edge explicitly.
    internal bool TryGetContact(in CurvedPrismContactGeometry geometry, int face,
        in RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper,
        Vector3d center, FixedQuaternion rotation, Vector3d prismOrigin,
        out FixedContactAnchors contact, int boundaryEdge = -1, scoped CurvedPrismContactValue native = default,
        PrismCurvedBoundaryContactVisitor? visitor = null, int ordinal = 0)
    {
        contact = default; _hasValue = false;
        CurvedPrismContact.GetFacePlane(geometry, face, out WideDirection3d normal, out _, out _);
        System.Diagnostics.Debug.Assert(!normal.IsZero);
        Span<Signed576> direction = stackalloc Signed576[3] { normal.X, normal.Y, normal.Z };
        WideArithmetic.ReduceCommonScale(direction); normal = new(direction[0], direction[1], direction[2]);
        if (face < 0) lower = upper = face == -1 ? -geometry.PrismHalfThickness : geometry.PrismHalfThickness;
        CurvedPrismContact.GetNativeBoundarySegment(geometry, face >= 0 ? face : boundaryEdge, clip, lower, upper,
            out WideAxis3 a, out WideAxis3 b, out Signed320 coordinateD, out var retained);
        var frame = new ConePlaneRayFrame(geometry.HeightValue, geometry.RadiusValue, coordinateD, normal, WideDirection3d.Dot(normal, a));
        a = new(a.X, WideArithmetic.SubtractSigned320(frame.Finite.ShapeFrame.Cap, a.Y), a.Z);
        b = new(b.X, WideArithmetic.SubtractSigned320(frame.Finite.ShapeFrame.Cap, b.Y), b.Z);
        // Same exact original segment/clip owns construction and admission.
        // Its complete domain is already on the admitted native surface;
        // a plane cohort or second prism-halfplane inventory adds no points.
        ConePlaneRayEvents.VisitNativeSegment(frame, a, b, retained, _visit);
        if (!_hasValue) return false;
        var maximum = Borrow(); var physicalFrame = _frame.WithConePose(center, rotation);
        if (!maximum.TryGetRoundedMaximumDepth(out Fixed64 depth)
            || !ConePlaneRayPointMaterialization.TryGetPointInFrame(physicalFrame, maximum.Point, maximum.Root, prismOrigin, out Vector3d sourcePoint)) return false;
        // An admitted finite exit has |X|,|Z|<=R and |Y|<=H/2 in this
        // centered local chart, so nearest rounding cannot exceed Fixed64.
        bool exitRepresented = ConePlaneRayPointMaterialization.TryGetExitConeLocalPoint(physicalFrame, maximum.Point, maximum.Root,
            ContactQuadratic.At(maximum.Values, maximum.Signs, 0, _frame.FieldWords),
            ContactQuadratic.At(maximum.Values, maximum.Signs, 1, _frame.FieldWords), 1, out Vector3d exitPoint);
        System.Diagnostics.Debug.Assert(exitRepresented);
        contact = new(new FixedPointAnchor(center, rotation, exitPoint), new FixedPointAnchor(prismOrigin, FixedQuaternion.Identity, sourcePoint),
            normal.Transform(geometry.WorldBasis).GetNormal(false), depth, false);
        if (visitor != null) visitor(ordinal, contact, native.CompareRootRatio(geometry,
            ContactQuadratic.At(maximum.Values, maximum.Signs, 0, _frame.FieldWords),
            ContactQuadratic.At(maximum.Values, maximum.Signs, 1, _frame.FieldWords), maximum.Root,
            _frame.NormalSquared, WideArithmetic.AddSigned320(_frame.Finite.ShapeFrame.CoordinateDenominator, _frame.Finite.ShapeFrame.CoordinateDenominator)));
        return true;
    }

    private ConePlaneRaySelection Borrow()
    {
        var result = new ConePlaneRaySelection(_values, _signs, _frame.FieldWords, _frame.RootWords);
        result.Restore(_frame, _source, _event); return result;
    }

    private void Visit(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEvent descriptor,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
        scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
    {
        if (!positive.HasValue) return;
        if (_values.Length < frame.SelectionStorageWords) Array.Resize(ref _values, frame.SelectionStorageWords);
        var maximum = new ConePlaneRaySelection(_values, _signs, frame.FieldWords, frame.RootWords);
        if (_hasValue) maximum.Restore(_frame, _source, _event);
        if (!maximum.KeepEvaluated(positive, frame)) return;
        _hasValue = true; _frame = frame; _source = maximum.MaximumSource; _event = maximum.MaximumEvent;
    }
}
