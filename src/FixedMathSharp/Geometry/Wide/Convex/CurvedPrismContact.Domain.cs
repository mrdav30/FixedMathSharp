//=======================================================================
// CurvedPrismContact.Domain.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Admits retained source support families before native curved minimum selection.</content>
internal static partial class CurvedPrismContact
{
    /// <remarks>The caller independently certifies physical source-prism overlap.
    /// A borrowed coplanar support supplier need not overlap; a far-boundary
    /// pass can also inspect a disjoint source. Restricting response supports
    /// never substitutes for the separate closed-solid overlap oracle.</remarks>
    internal static bool TryGetContact(in CurvedPrismContactGeometry geometry, PrismCurvedContactDomain domain,
        Vector3d center, FixedQuaternion rotation, Vector3d prismOrigin,
        out FixedContactAnchors contact, out int face, out bool physical, out int certificate,
        ReadOnlySpan<PrismCurvedBoundaryLine> boundaryLines = default,
        PrismCurvedBoundaryContactVisitor? boundaryVisitor = null, ConePrismBoundaryContacts? coneBoundary = null)
    {
        contact = default; face = -3; physical = false; certificate = -1;
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        var selection = new CurvedPrismContactSelection(values, signs, domain);
        Span<ulong> direction = stackalloc ulong[3 * Words]; Span<int> directionSigns = stackalloc int[3];
        // A same-plane full-support certificate supplies a cheap exact bound
        // before any large witness matching or quartic chart construction.
        foreach (var proof in domain.Certificates)
        {
            if (!proof.Physical && !domain.IncludeFarBoundary) continue;
            GetFacePlane(geometry, proof.Face, out WideDirection3d plane, out _, out _);
            KeepAxis(geometry, plane, direction, directionSigns, ref selection);
            System.Diagnostics.Debug.Assert(selection.HasValue);
            if (selection.GapSign == 0) break;
        }
        if (!domain.Edges.IsEmpty)
        {
            KeepAxis(geometry, new WideDirection3d(geometry.Axis), direction, directionSigns, ref selection);
            if (!selection.HasValue || selection.GapSign != 0) KeepAxis(geometry, geometry.Plane(1), direction, directionSigns, ref selection);
            if (!selection.HasValue || selection.GapSign != 0) KeepAnalytic(geometry, ref selection);
        }
        Fixed64 depth = Fixed64.Zero; bool clamped = false;
        if (selection.HasValue) WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(selection.Candidate,
            Fixed64.Zero, out depth, out clamped);
        CurvedPrismRimContacts.Selection rim = default;
        if (!domain.Edges.IsEmpty && (!selection.HasValue || selection.GapSign != 0))
            CurvedPrismRimContacts.GetContact(geometry, selection.Candidate, depth, clamped, out rim, out depth, out clamped,
                domain, selection.HasValue);
        var native = new CurvedPrismContactValue(selection.Candidate, selection.HasValue, rim);
        for (int index = 0; index < boundaryLines.Length; index++)
        {
            var line = boundaryLines[index];
            // The producer qualifies independent source domains. Numeric
            // line competition does not change this native minimum or infer
            // ownership from a coincident face or a preferred support point.
            if (geometry.IsCone)
            {
                System.Diagnostics.Debug.Assert(coneBoundary != null);
                coneBoundary!.TryGetContact(geometry, line.Face, line.Clip, line.Lower, line.Upper,
                    center, rotation, prismOrigin, out _, line.BoundaryEdge, native, boundaryVisitor, index);
            }
            else TryGetCylinderBoundaryContact(geometry, line.Face, line.Clip, line.Lower, line.Upper,
                center, rotation, prismOrigin, out _, line.BoundaryEdge, native, boundaryVisitor, index);
        }
        if (rim.HasValue)
        {
            CurvedPrismRimContacts.GetMaterials(geometry, rim, center, rotation, prismOrigin, out Vector3d normal,
                out FixedPointAnchor shapeAnchor, out FixedPointAnchor prismAnchor);
            contact = new(shapeAnchor, prismAnchor, normal, depth, clamped); physical = rim.Physical; return true;
        }
        if (!selection.HasValue) return false;
        if (selection.Certificate >= 0)
        {
            certificate = selection.Certificate; var proof = domain.Certificates[certificate];
            var supplied = proof.Contact; face = proof.Face; physical = proof.Physical;
            contact = new(supplied.FirstAnchor, supplied.SecondAnchor, supplied.Normal, depth, clamped); return true;
        }
        var window = GetDomainWindow(domain, selection.Window);
        // Keep admitted this unchanged candidate/window with the same exact
        // matcher. Only final normal/anchor materialization is added here.
        bool matched = TryGetAnalyticWitnesses(geometry, selection, center, rotation, prismOrigin,
            out Vector3d analyticNormal, out FixedPointAnchor shape, out FixedPointAnchor target, out physical, window);
        System.Diagnostics.Debug.Assert(matched);
        contact = new(shape, target, analyticNormal, depth, clamped); return true;
    }

    internal static bool TryAdmitAnalyticCandidate(in CurvedPrismContactGeometry geometry,
        PrismCurvedContactDomain domain, ReadOnlySpan<ulong> values, ReadOnlySpan<int> signs,
        int gap, int vertex, int feature, out int window, out int certificate)
    {
        window = 0; certificate = -1; bool physical;
        if (gap < 0) return false;
        for (int index = 0; index < domain.Certificates.Length; index++)
        {
            var proof = domain.Certificates[index];
            if (!proof.Physical && !domain.IncludeFarBoundary) continue;
            GetFacePlane(geometry, proof.Face, out WideDirection3d axis, out _, out _);
            if (IsFaceDirection(values, signs, axis))
            { certificate = index; return true; }
        }
        System.Diagnostics.Debug.Assert(!domain.Edges.IsEmpty);
        // Face directions belong exclusively to the complete full-support
        // certificate inventory. Do not rediscover an absent face proof by
        // matching the same closed family through its edge endpoints.
        for (int face = -2; face < geometry.VertexCount / 2; face++)
        {
            GetFacePlane(geometry, face, out WideDirection3d axis, out _, out _);
            if (IsFaceDirection(values, signs, axis)) return false;
        }
        Span<ulong> copy = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> copySigns = stackalloc int[ConvexContactCandidate.Slots];
        values.CopyTo(copy); signs.CopyTo(copySigns);
        var candidate = new CurvedPrismContactSelection(copy, copySigns)
            { HasValue = true, GapSign = gap, Vertex = vertex, Feature = feature };
        for (int index = 0; index < domain.Edges.Length; index++)
        {
            var edge = domain.Edges[index];
            if (edge.ReferenceVertex != vertex && DirectionDotSign(values, signs, Difference(geometry.Vertex(edge.ReferenceVertex), geometry.Vertex(vertex))) != 0) continue;
            // A non-endpoint support on a real edge requires the whole edge
            // to tie. Reject incompatible directions before matching witnesses.
            var interval = edge.Interval;
            bool endpoint = edge.Point && (!interval.IsClipped || interval.LowerNumerator.IsZero
                || interval.LowerNumerator.Equals(interval.LowerDenominator));
            if (!endpoint)
            {
                geometry.Edge(edge.Edge, out int first, out int last);
                if (DirectionDotSign(values, signs, Difference(geometry.Vertex(last), geometry.Vertex(first))) != 0) continue;
            }
            bool admits = true;
            foreach (var covector in domain.FanCovectors.Slice(edge.FanStart, edge.FanCount))
                if (DirectionDotSign(values, signs, covector) < 0) { admits = false; break; }
            if (!admits) continue;
            if (TryGetAnalyticWitnesses(geometry, candidate, default, FixedQuaternion.Identity, default,
                out _, out _, out _, out physical, new FaceWindow(edge), materialize: false)
                && (physical || domain.IncludeFarBoundary)) { window = -index - 1; return true; }
        }
        return false;
    }

    private static FaceWindow GetDomainWindow(PrismCurvedContactDomain domain, int window)
        => new(domain.Edges[-window - 1]);

    private static bool IsFaceDirection(ReadOnlySpan<ulong> values, ReadOnlySpan<int> signs, in WideDirection3d axis)
        => DirectionDotSign(values, signs, axis) > 0
        && DirectionDotSign(values, signs, new(default, axis.Z, WideArithmetic.Negate(axis.Y))) == 0
        && DirectionDotSign(values, signs, new(WideArithmetic.Negate(axis.Z), default, axis.X)) == 0
        && DirectionDotSign(values, signs, new(axis.Y, WideArithmetic.Negate(axis.X), default)) == 0;

    private static int DirectionDotSign(ReadOnlySpan<ulong> direction, ReadOnlySpan<int> directionSigns, in WideDirection3d delta)
    {
        if (delta.IsZero) return 0;
        if ((directionSigns[3] | directionSigns[4] | directionSigns[5]) == 0
            && WideArithmetic.GetMagnitudeBitLength(Slot(direction, 0)) < 576
            && WideArithmetic.GetMagnitudeBitLength(Slot(direction, 1)) < 576
            && WideArithmetic.GetMagnitudeBitLength(Slot(direction, 2)) < 576)
        {
            // Native rational directions and fan covectors need <576/<234
            // bits. Their complete dot fits Signed832 without CQ scratch.
            var normal = new WideDirection3d(
                directionSigns[0] == 0 ? default : CurvedPrismContactGeometry.ReadDirectionComponent(Slot(direction, 0), directionSigns[0]),
                directionSigns[1] == 0 ? default : CurvedPrismContactGeometry.ReadDirectionComponent(Slot(direction, 1), directionSigns[1]),
                directionSigns[2] == 0 ? default : CurvedPrismContactGeometry.ReadDirectionComponent(Slot(direction, 2), directionSigns[2]));
            return WideDirection3d.Dot(normal, delta).Sign;
        }
        return DirectionDotQuadraticSign(direction, directionSigns, delta);
    }

    // Keep the large CQ stack frame out of rational fan checks.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int DirectionDotQuadraticSign(ReadOnlySpan<ulong> direction, ReadOnlySpan<int> directionSigns, in WideDirection3d delta)
    {
        Span<ulong> fields = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        var value = At(fields, signs, 0); var component = At(fields, signs, 1); var term = At(fields, signs, 2);
        value.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            Slot(direction, axis).CopyTo(component.Rational); Slot(direction, axis + 3).CopyTo(component.Radical);
            component.Signs[0] = directionSigns[axis]; component.Signs[1] = directionSigns[axis + 3];
            Scale(component, axis == 0 ? delta.X : axis == 1 ? delta.Y : delta.Z, term); value.Add(term);
        }
        return value.Sign(Slot(direction, 6));
    }

    private static bool ClipRealEdgeWindow(in CurvedPrismContactGeometry geometry, in FaceWindow window,
        Span<ulong> start, Span<int> startSigns, Span<ulong> delta, Span<int> deltaSigns,
        ContactQuadratic denominator, ReadOnlySpan<ulong> root, ContactQuadratic lower,
        ContactQuadratic lowerDen, ContactQuadratic upper, ContactQuadratic upperDen)
    {
        geometry.Edge(window.SourceEdge, out int first, out int last);
        int vertex = window.SourceEdge / 3, kind = window.SourceEdge % 3;
        WideDirection3d a = geometry.Vertex(first), edge = Difference(geometry.Vertex(last), a);
        WideDirection3d firstPlane = kind == 2 ? geometry.Plane(3 + (vertex == 0 ? geometry.VertexCount / 2 - 1 : vertex - 1)) : geometry.Plane(1);
        WideDirection3d secondPlane = geometry.Plane(3 + vertex);
        Span<WideDirection3d> planes = stackalloc WideDirection3d[2] { firstPlane, secondPlane };
        foreach (var plane in planes)
        {
            Signed832 c = WideDirection3d.Dot(plane, a);
            if (!ClipFaceHalfplane(plane, c, start, startSigns, delta, deltaSigns, denominator, root, lower, lowerDen, upper, upperDen)
                || !ClipFaceHalfplane(-plane, WideArithmetic.SubtractSigned832(default, c), start, startSigns, delta, deltaSigns,
                    denominator, root, lower, lowerDen, upper, upperDen)) return false;
        }
        Signed832 dot = WideDirection3d.Dot(edge, a), squared = edge.SquaredLength;
        var clip = window.Tangent;
        Signed320 ln = clip.IsClipped ? clip.LowerNumerator : default, ld = clip.IsClipped ? clip.LowerDenominator : Signed320.One;
        Signed320 un = clip.IsClipped ? clip.UpperNumerator : Signed320.One, ud = clip.IsClipped ? clip.UpperDenominator : Signed320.One;
        return ClipFaceHalfplane(ScaleAxis(edge, ld), SumProducts(dot, ld, squared, ln), start, startSigns, delta, deltaSigns,
            denominator, root, lower, lowerDen, upper, upperDen)
            && ClipFaceHalfplane(-ScaleAxis(edge, ud), WideArithmetic.SubtractSigned832(default, SumProducts(dot, ud, squared, un)),
                start, startSigns, delta, deltaSigns, denominator, root, lower, lowerDen, upper, upperDen);
    }

    private static bool IsInsideRealEdgeWindow(in CurvedPrismContactGeometry geometry, in FaceWindow window,
        Span<ulong> point, Span<int> pointSigns, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        geometry.Edge(window.SourceEdge, out int first, out int last);
        var a = geometry.Vertex(first); var edge = Difference(geometry.Vertex(last), a);
        int words = denominator.FieldWords;
        Span<ulong> fields = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var n = WAt(fields, signs, 0); var d = WAt(fields, signs, 1); var work = WAt(fields, signs, 2);
        DotQuadratic(edge, point, pointSigns, n); Scale(denominator, WideDirection3d.Dot(edge, a), work); n.Add(work, -1);
        Scale(denominator, edge.SquaredLength, d);
        var clip = window.Tangent;
        Scale(n, Signed576.ExtendValue(clip.IsClipped ? clip.LowerDenominator : Signed320.One), work);
        Scale(d, Signed576.ExtendValue(clip.IsClipped ? clip.LowerNumerator : default), n); work.Add(n, -1);
        if (work.Sign(root) <= 0) return false;
        DotQuadratic(edge, point, pointSigns, n); Scale(denominator, WideDirection3d.Dot(edge, a), work); n.Add(work, -1);
        Scale(n, Signed576.ExtendValue(clip.IsClipped ? clip.UpperDenominator : Signed320.One), work);
        Scale(d, Signed576.ExtendValue(clip.IsClipped ? clip.UpperNumerator : Signed320.One), n); work.Add(n, -1);
        return work.Sign(root) < 0;
    }
}
