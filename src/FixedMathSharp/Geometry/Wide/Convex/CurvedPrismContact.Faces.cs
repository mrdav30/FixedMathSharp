//=======================================================================
// CurvedPrismContact.Faces.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Matches complete curved supports against retained native prism face windows.</content>
internal static partial class CurvedPrismContact
{
    // Side faces use their footprint edge index; -1/-2 identify bottom/top.
    // This is a full-support certificate, not an arbitrary finite-face maximum:
    // a cut that removes that support still needs the finite boundary events.
    internal static bool TryGetFaceSupportContact(in CurvedPrismContactGeometry geometry, int face,
        in RationalSegmentInterval tangentClip, Fixed64 lowerY, Fixed64 upperY,
        Vector3d center, FixedQuaternion rotation, Vector3d prismOrigin, bool includeRecovery,
        out FixedContactAnchors contact, out bool physical)
    {
        contact = default; physical = false;
        GetFacePlane(geometry, face, out WideDirection3d axis, out Signed832 minimum, out int vertex);
        System.Diagnostics.Debug.Assert(!axis.IsZero);
        Span<ulong> direction = stackalloc ulong[3 * Words]; Span<int> directionSigns = stackalloc int[3];
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        WriteDirection(-axis, direction, directionSigns);
        int other = geometry.BuildPlaneCandidate(direction, directionSigns, -axis,
            WideArithmetic.SubtractSigned832(default, minimum), values, signs, out _);
        // An outside source plane cannot supply an initial-section physical row.
        if (!includeRecovery && other < 0) return false;
        WriteDirection(axis, direction, directionSigns);
        int gap = geometry.BuildPlaneCandidate(direction, directionSigns, axis, minimum, values, signs, out int feature);
        if (gap < 0) return false;
        var selection = new CurvedPrismContactSelection(values, signs)
            { HasValue = true, GapSign = gap, Vertex = vertex, Feature = feature };
        if (!TryGetAnalyticWitnesses(geometry, selection, center, rotation, prismOrigin,
                out Vector3d normal, out FixedPointAnchor shape, out FixedPointAnchor target, out physical,
                new FaceWindow(face, tangentClip, lowerY, upperY)) || !includeRecovery && !physical) return false;
        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(selection.Candidate,
            Fixed64.Zero, out Fixed64 depth, out bool clamped);
        contact = new(shape, target, normal, depth, clamped); return true;
    }

    internal static void GetFacePlane(in CurvedPrismContactGeometry geometry, int face,
        out WideDirection3d axis, out Signed832 minimum, out int vertex)
    {
        axis = face < 0 ? geometry.Plane(1) : geometry.Plane(3 + face);
        if (face == -2) axis = -axis;
        vertex = face < 0 ? (face == -1 ? 0 : 1) : 2 * face;
        minimum = WideDirection3d.Dot(axis, geometry.Vertex(vertex));
        if (WideArithmetic.SubtractSigned832(geometry.ProjectionMinimum(axis), minimum).Sign != 0)
        { axis = -axis; minimum = WideArithmetic.SubtractSigned832(default, minimum); }
    }

    private readonly struct FaceWindow
    {
        internal readonly bool Enabled;
        internal readonly int Face, SourceEdge;
        internal readonly bool Open;
        internal readonly RationalSegmentInterval Tangent;
        internal readonly Fixed64 Lower, Upper;
        internal FaceWindow(int face, RationalSegmentInterval tangent, Fixed64 lower, Fixed64 upper)
        { Enabled = true; Face = face; SourceEdge = -1; Open = false; Tangent = tangent; Lower = lower; Upper = upper; }
        internal FaceWindow(in PrismCurvedEdgeWindow edge)
        { Enabled = true; Face = -1; SourceEdge = edge.Edge; Open = !edge.Point;
          Tangent = edge.Interval; Lower = Upper = default; }
    }

    private static bool ClipFaceWindow(in CurvedPrismContactGeometry geometry, in FaceWindow window,
        Span<ulong> start, Span<int> startSigns, Span<ulong> delta, Span<int> deltaSigns,
        ContactQuadratic denominator, ReadOnlySpan<ulong> root, ContactQuadratic lower,
        ContactQuadratic lowerDen, ContactQuadratic upper, ContactQuadratic upperDen)
    {
        if (window.SourceEdge >= 0) return ClipRealEdgeWindow(geometry, window, start, startSigns, delta, deltaSigns,
            denominator, root, lower, lowerDen, upper, upperDen);
        Span<WideDirection3d> normals = stackalloc WideDirection3d[4];
        Span<Signed832> constants = stackalloc Signed832[4];
        int count = GetFaceWindowHalfplanes(geometry, window.Face, window.Tangent, window.Lower, window.Upper, normals, constants);
        for (int index = 0; index < count; index++)
            if (!ClipFaceHalfplane(normals[index], constants[index], start, startSigns, delta, deltaSigns,
                denominator, root, lower, lowerDen, upper, upperDen)) return false;
        return true;
    }

    private static int GetFaceWindowHalfplanes(in CurvedPrismContactGeometry geometry, int face,
        in RationalSegmentInterval tangent, Fixed64 lowerY, Fixed64 upperY,
        Span<WideDirection3d> normals, Span<Signed832> constants)
    {
        if (face < 0) return 0; // The complete cap footprint is already clipped by the prism.
        geometry.Edge(3 * face, out int first, out int last);
        WideDirection3d a = geometry.Vertex(first), edge = Difference(geometry.Vertex(last), a);
        Signed832 dot = WideDirection3d.Dot(edge, a), squared = edge.SquaredLength;
        Signed320 ln = tangent.IsClipped ? tangent.LowerNumerator : default;
        Signed320 ld = tangent.IsClipped ? tangent.LowerDenominator : Signed320.One;
        Signed320 un = tangent.IsClipped ? tangent.UpperNumerator : Signed320.One;
        Signed320 ud = tangent.IsClipped ? tangent.UpperDenominator : Signed320.One;
        int count = 0;
        if (!ln.IsZero) { normals[count] = ScaleAxis(edge, ld); constants[count++] = SumProducts(dot, ld, squared, ln); }
        if (!un.Equals(ud))
        { normals[count] = -ScaleAxis(edge, ud); constants[count++] = WideArithmetic.SubtractSigned832(default, SumProducts(dot, ud, squared, un)); }
        Fixed64 half = geometry.PrismHalfThickness;
        if (lowerY > -half || upperY < half)
        {
            WideDirection3d up = geometry.Plane(1);
            Signed832 bottom = WideDirection3d.Dot(up, geometry.Vertex(0));
            Signed832 height = WideArithmetic.SubtractSigned832(WideDirection3d.Dot(up, geometry.Vertex(1)), bottom);
            Signed320 whole = Signed320.ExtendValue(WideArithmetic.AddSigned192(Signed192.Raw(half), Signed192.Raw(half)));
            if (lowerY > -half)
            { normals[count] = ScaleAxis(up, whole); constants[count++] = SumProducts(bottom, whole, height,
                Signed320.ExtendValue(WideArithmetic.AddSigned192(Signed192.Raw(lowerY), Signed192.Raw(half)))); }
            if (upperY < half)
            { normals[count] = -ScaleAxis(up, whole); constants[count++] = WideArithmetic.SubtractSigned832(default, SumProducts(bottom, whole, height,
                Signed320.ExtendValue(WideArithmetic.AddSigned192(Signed192.Raw(upperY), Signed192.Raw(half))))); }
        }
        return count;
    }

    // Native retained cut ratios fit Signed320. Their scaled edge normals stay
    // below 486 bits and plane constants below 652; the witness banks remain sufficient.
    private static WideDirection3d ScaleAxis(in WideDirection3d axis, Signed320 scale) => new(
        Signed576.NarrowValue(WideArithmetic.MultiplySigned576ToSigned832(axis.X, scale)),
        Signed576.NarrowValue(WideArithmetic.MultiplySigned576ToSigned832(axis.Y, scale)),
        Signed576.NarrowValue(WideArithmetic.MultiplySigned576ToSigned832(axis.Z, scale)));

    private static Signed832 SumProducts(Signed832 first, Signed320 firstScale, Signed832 second, Signed320 secondScale) =>
        WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned832(first, firstScale),
            WideArithmetic.MultiplySigned832(second, secondScale));

    private static bool ClipFaceHalfplane(in WideDirection3d normal, Signed832 constant,
        Span<ulong> start, Span<int> startSigns, Span<ulong> delta, Span<int> deltaSigns,
        ContactQuadratic denominator, ReadOnlySpan<ulong> root, ContactQuadratic lower,
        ContactQuadratic lowerDen, ContactQuadratic upper, ContactQuadratic upperDen)
    {
        int words = denominator.FieldWords;
        Span<ulong> values = stackalloc ulong[6 * words]; Span<int> signs = stackalloc int[6];
        var a = WAt(values, signs, 0); var b = WAt(values, signs, 1);
        var term = WAt(values, signs, 2);
        DotQuadratic(normal, start, startSigns, a); Scale(denominator, constant, term); a.Add(term, -1);
        DotQuadratic(normal, delta, deltaSigns, b);
        return ClipBound(a, b, root, lower, lowerDen, upper, upperDen);
    }

    private static bool ClipBound(ContactQuadratic a, ContactQuadratic b, ReadOnlySpan<ulong> root,
        ContactQuadratic lower, ContactQuadratic lowerDen, ContactQuadratic upper, ContactQuadratic upperDen)
    {
        int words = Math.Max(a.FieldWords, b.FieldWords);
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var comparison = WAt(values, signs, 0); var term = WAt(values, signs, 1);
        int slope = b.Sign(root);
        if (slope == 0) return a.Sign(root) >= 0;
        a.MultiplySign(-slope); b.MultiplySign(slope);
        var bound = slope > 0 ? lower : upper; var boundDen = slope > 0 ? lowerDen : upperDen;
        Multiply(a, boundDen, root, comparison); Multiply(bound, b, root, term); comparison.Add(term, -1);
        if (comparison.Sign(root) * slope > 0) { a.CopyTo(bound); b.CopyTo(boundDen); }
        return true;
    }

    private static void GetCurvedImplicit(in CurvedPrismContactGeometry geometry,
        Span<ulong> point, Span<int> pointSigns, ContactQuadratic denominator, ReadOnlySpan<ulong> root, ContactQuadratic result)
    {
        // This runs before support-parameter blending. The squared point and
        // root products add at most twenty limbs for all retained axis/radius
        // factors; size by active coefficients rather than padded witnesses.
        int active = ContactQuadratic.ActiveWords(denominator);
        for (int axis = 0; axis < 3; axis++) active = Math.Max(active, ContactQuadratic.ActiveWords(WAt(point, pointSigns, axis)));
        int words = 2 * active + WideArithmetic.GetActiveMagnitudeLength(root) + 20;
        Span<ulong> values = stackalloc ulong[16 * words]; Span<int> signs = stackalloc int[16];
        var axial = At(values, signs, 0, words); var maximum = At(values, signs, 1, words);
        var work = At(values, signs, 2, words); var product = At(values, signs, 3, words);
        var radial = At(values, signs, 4, words); var component = At(values, signs, 5, words);
        var d = At(values, signs, 6, words); var limit = At(values, signs, 7, words);
        denominator.CopyTo(d); axial.Clear(); radial.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            WAt(point, pointSigns, axis).CopyTo(component);
            Scale(component, Signed576.ExtendValue(axis == 0 ? geometry.Axis.X : axis == 1 ? geometry.Axis.Y : geometry.Axis.Z), work);
            axial.Add(work); Multiply(component, component, root, work); radial.Add(work);
        }
        Scale(d, WideArithmetic.MultiplySigned320(geometry.SquaredAxis, geometry.HalfFactor), maximum);
        Scale(radial, Signed576.ExtendValue(geometry.SquaredAxis), work);
        Multiply(axial, axial, root, product); work.Add(product, -1); work.CopyTo(radial);
        Signed576 radiusSquared = WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius);
        if (geometry.IsCone && !geometry.HalfFactor.IsZero)
        {
            Signed576 factor = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(geometry.HalfFactor, geometry.HalfFactor));
            factor = Signed576.NarrowValue(WideArithmetic.MultiplySigned576ToSigned832(factor, geometry.SquaredAxis));
            factor = WideArithmetic.AddSigned576(factor, factor); factor = WideArithmetic.AddSigned576(factor, factor);
            Scale(radial, factor, work); maximum.CopyTo(component); component.Add(axial, -1);
            Multiply(component, component, root, product); Scale(product, radiusSquared, limit);
        }
        else
        {
            radial.CopyTo(work); Multiply(d, d, root, product); Scale(product, radiusSquared, component);
            Scale(component, Signed576.ExtendValue(geometry.SquaredAxis), limit);
        }
        work.Add(limit, -1); work.CopyTo(result);
    }

}
