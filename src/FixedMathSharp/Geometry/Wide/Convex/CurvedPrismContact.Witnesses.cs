//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Matches the retained analytic feature before any physical point rounds.</content>
internal static partial class CurvedPrismContact
{
    // Generator normals have effective height <600 bits, their squared norm
    // <1202, and the common support denominator <832. Translated segment
    // numerators are <2200 bits; plane bounds <2440; cross-bound comparisons
    // <4880 and final coordinates (including frame scales) <4820. Pole
    // matching's squared disk test is <5700; its displacement is identically
    // zero, so clipping never multiplies two pole interval bounds. Keep all
    // products in 7168-bit banks; candidate ranking still uses forty.
    private const int WitnessWords = 112;

    private static void GetAnalyticWitnesses(in CurvedPrismContactGeometry geometry,
        scoped CurvedPrismContactSelection selection, Vector3d center, FixedQuaternion rotation,
        Vector3d prismOrigin, out Vector3d normal, out FixedPointAnchor shapeAnchor, out FixedPointAnchor prismAnchor)
    {
        bool matched = TryGetAnalyticWitnesses(geometry, selection, center, rotation, prismOrigin,
            out normal, out shapeAnchor, out prismAnchor, out _, default);
        System.Diagnostics.Debug.Assert(matched);
    }

    private static bool TryGetAnalyticWitnesses(in CurvedPrismContactGeometry geometry,
        scoped CurvedPrismContactSelection selection, Vector3d center, FixedQuaternion rotation,
        Vector3d prismOrigin, out Vector3d normal, out FixedPointAnchor shapeAnchor, out FixedPointAnchor prismAnchor,
        out bool physical, in FaceWindow window, bool materialize = true)
    {
        // Only a clipped algebraic cone-generator family can grow its physical
        // parameter beyond the ordinary bound. Rational native face normals,
        // pole disks and unique witnesses retain the existing smaller banks.
        int words = window.Enabled && window.SourceEdge >= 0 && geometry.IsCone
            && selection.Feature == CurvedPrismContactSelection.Free ? 192 : WitnessWords;
        shapeAnchor = prismAnchor = default; physical = false;
        normal = default;
        if (materialize)
        {
            Span<ulong> world = stackalloc ulong[7 * Words];
            Span<int> worldSigns = stackalloc int[7];
            world.Clear(); worldSigns.Clear();
            WriteWorldDirection(geometry.WorldBasis, selection.Values[..(3 * Words)], selection.Signs[..3],
                world[..(3 * Words)], worldSigns[..3]);
            WriteWorldDirection(geometry.WorldBasis, selection.Values.Slice(3 * Words, 3 * Words), selection.Signs.Slice(3, 3),
                world.Slice(3 * Words, 3 * Words), worldSigns.Slice(3, 3));
            Slot(selection.Values, 6).CopyTo(Slot(world, 6));
            normal = WideConvexPrismRelations.GetConvexContactCandidateNormal(new ConvexContactCandidate(world, worldSigns, 0));
        }

        Span<ulong> root = stackalloc ulong[words];
        root.Clear(); Slot(selection.Values, 6).CopyTo(root);
        Span<ulong> work = stackalloc ulong[24 * words];
        Span<int> signs = stackalloc int[24];
        ContactQuadratic metric = WAt(work, signs, 0), axial = WAt(work, signs, 1);
        ContactQuadratic minimum = WAt(work, signs, 2), supportDenominator = WAt(work, signs, 3);
        ContactQuadratic temporary = WAt(work, signs, 4), product = WAt(work, signs, 5);
        ContactQuadratic gap = WAt(work, signs, 6), q = WAt(work, signs, 7);
        Span<ulong> n = stackalloc ulong[6 * words];
        Span<int> nSigns = stackalloc int[6];
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic component = WAt(n, nSigns, axis);
            component.Clear();
            Slot(selection.Values, axis).CopyTo(component.Rational);
            Slot(selection.Values, axis + 3).CopyTo(component.Radical);
            component.Signs[0] = selection.Signs[axis]; component.Signs[1] = selection.Signs[axis + 3];
        }
        metric.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            Multiply(WAt(n, nSigns, axis), WAt(n, nSigns, axis), root, temporary);
            metric.Add(temporary);
        }
        DotQuadratic(new WideDirection3d(geometry.Axis), n, nSigns, axial);
        // Selection retained an exact minimum vertex for this same normal.
        DotQuadratic(geometry.Vertex(selection.Vertex), n, nSigns, minimum);
        q.Set(Signed576.ExtendValue(geometry.SquaredAxis));

        Span<ulong> support = stackalloc ulong[6 * words];
        Span<int> supportSigns = stackalloc int[6];
        Span<ulong> displacement = stackalloc ulong[6 * words];
        Span<int> displacementSigns = stackalloc int[6];
        displacement.Clear(); displacementSigns.Clear();
        Multiply(metric, q, root, temporary);
        Multiply(axial, axial, root, product); temporary.Add(product, -1);
        bool pole = temporary.Sign(root) == 0;
        if (pole)
        {
            // The pole's support disk is free in two dimensions. Select a
            // finite prism point inside that disk before translating it back.
            if (!GetPoleSupport(geometry, selection, n, nSigns, metric, minimum,
                support, supportSigns, supportDenominator, window)) return false;
        }
        else if (geometry.IsCone && selection.Feature == CurvedPrismContactSelection.Free)
        {
            // On the generator seam sqrt(q*(q*n²-h²))=2*q*H.n/R.
            // Rationalizing with that exact equality keeps one quadratic field.
            DotQuadratic(geometry.Half, n, nSigns, supportDenominator);
            Scale(supportDenominator, Signed576.ExtendValue(geometry.SquaredAxis), temporary);
            temporary.CopyTo(supportDenominator); supportDenominator.Add(temporary);
            Signed576 radiusSquared = WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius);
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic start = WAt(support, supportSigns, axis);
                ContactQuadratic delta = WAt(displacement, displacementSigns, axis);
                Scale(supportDenominator, Component(geometry.Half, axis), start);
                start.CopyTo(delta, -1); delta.Add(delta);
                Scale(WAt(n, nSigns, axis), Signed576.ExtendValue(geometry.SquaredAxis), temporary);
                Scale(axial, Signed576.ExtendValue(Component(geometry.Axis, axis)), product);
                temporary.Add(product, -1);
                Scale(temporary, radiusSquared, product); delta.Add(product);
            }
        }
        else if (!geometry.IsCone && selection.Feature == CurvedPrismContactSelection.Free)
        {
            // Side support is a finite axial segment. Its radial term uses
            // sqrt(n²); the exact local normal itself is rational here.
            metric.Rational.CopyTo(root);
            metric.CopyTo(supportDenominator);
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic start = WAt(support, supportSigns, axis);
                Scale(metric, Component(geometry.Half, axis), start);
                ContactQuadratic delta = WAt(displacement, displacementSigns, axis);
                start.CopyTo(delta); delta.Add(delta);
                start.MultiplySign(-1);
                Scale(WAt(n, nSigns, axis), Signed576.ExtendValue(geometry.Radius), temporary);
                MultiplyRoot(temporary, root, product); start.Add(product);
            }
        }
        else if (geometry.IsCone && selection.Feature == CurvedPrismContactSelection.Top)
        {
            supportDenominator.Set(Integer(1));
            for (int axis = 0; axis < 3; axis++)
                WAt(support, supportSigns, axis).Set(Component(geometry.Half, axis));
        }
        else
        {
            // Smooth rim point R*(q*n-h*A)/sqrt(q*(q*n²-h²)).
            // Store its rationalized quadratic coordinates, rather than feed
            // a rounded public normal back into support selection.
            Multiply(temporary, q, root, supportDenominator);
            supportDenominator.Rational.CopyTo(root);
            int cap = selection.Feature;
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic point = WAt(support, supportSigns, axis);
                Scale(supportDenominator, Component(geometry.Half, axis), point);
                point.MultiplySign(cap);
                Scale(WAt(n, nSigns, axis), Signed576.ExtendValue(geometry.SquaredAxis), temporary);
                Scale(axial, Signed576.ExtendValue(Component(geometry.Axis, axis)), product);
                temporary.Add(product, -1);
                Scale(temporary, Signed576.ExtendValue(geometry.Radius), product);
                MultiplyRoot(product, root, temporary); point.Add(temporary);
            }
        }

        DotQuadraticNormal(support, supportSigns, n, nSigns, root, gap);
        Multiply(minimum, supportDenominator, root, product); gap.Add(product, -1);
        Span<ulong> translated = stackalloc ulong[6 * words];
        Span<int> translatedSigns = stackalloc int[6];
        Span<ulong> translatedDelta = stackalloc ulong[6 * words];
        Span<int> translatedDeltaSigns = stackalloc int[6];
        ContactQuadratic translatedDenominator = WAt(work, signs, 8);
        Multiply(supportDenominator, metric, root, translatedDenominator);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic point = WAt(translated, translatedSigns, axis);
            Multiply(WAt(support, supportSigns, axis), metric, root, point);
            Multiply(gap, WAt(n, nSigns, axis), root, product); point.Add(product, -1);
            Multiply(WAt(displacement, displacementSigns, axis), metric, root,
                WAt(translatedDelta, translatedDeltaSigns, axis));
        }
        ContactQuadratic parameterNumerator = WAt(work, signs, 9), parameterDenominator = WAt(work, signs, 10);
        if (!ClipSegment(geometry, translated, translatedSigns, translatedDelta, translatedDeltaSigns,
            translatedDenominator, root, parameterNumerator, parameterDenominator, window, out physical)) return false;
        ContactQuadratic finalDenominator = WAt(work, signs, 11);
        Blend(support, supportSigns, displacement, displacementSigns,
            parameterNumerator, parameterDenominator, root);
        Multiply(supportDenominator, parameterDenominator, root, finalDenominator);
        if (materialize) shapeAnchor = CreateQuadraticAnchor(geometry, center, rotation, center,
            support, supportSigns, finalDenominator, root);
        Blend(translated, translatedSigns, translatedDelta, translatedDeltaSigns,
            parameterNumerator, parameterDenominator, root);
        Multiply(translatedDenominator, parameterDenominator, root, finalDenominator);
        if (window.SourceEdge >= 0 && window.Open && !IsInsideRealEdgeWindow(geometry, window, translated, translatedSigns, finalDenominator, root)) return false;
        if (materialize) prismAnchor = CreateQuadraticAnchor(geometry, center, rotation, prismOrigin,
            translated, translatedSigns, finalDenominator, root);
        return true;
    }

    private static bool GetPoleSupport(in CurvedPrismContactGeometry geometry,
        scoped CurvedPrismContactSelection selection, Span<ulong> normal, Span<int> normalSigns,
        ContactQuadratic metric, ContactQuadratic minimum, Span<ulong> support, Span<int> supportSigns,
        ContactQuadratic denominator, in FaceWindow window)
    {
        int words = denominator.FieldWords;
        Span<ulong> point = stackalloc ulong[3 * words], pointDenominator = stackalloc ulong[words];
        Span<int> pointSigns = stackalloc int[3];
        bool found;
        if (window.Enabled && (window.SourceEdge >= 0 || window.Face >= 0))
        {
            found = CurvedPrismCapWitness.TryGetWindowPoint(geometry, selection.Vertex, selection.Feature,
                window.Face, window.SourceEdge, window.Tangent, window.Open, window.Lower, window.Upper,
                point, pointSigns, pointDenominator, initialSection: true);
            if (!found) found = CurvedPrismCapWitness.TryGetWindowPoint(geometry, selection.Vertex, selection.Feature,
                window.Face, window.SourceEdge, window.Tangent, window.Open, window.Lower, window.Upper,
                point, pointSigns, pointDenominator);
        }
        else if (window.Enabled)
        {
            Span<ulong> pole = stackalloc ulong[3 * Words]; Span<int> poleSigns = stackalloc int[3];
            var axis = new WideDirection3d(geometry.Axis); if (selection.Feature < 0) axis = -axis;
            WriteDirection(axis, pole, poleSigns);
            found = CurvedPrismCapWitness.TryGetPoint(geometry, pole, poleSigns, selection.Vertex, selection.Feature,
                point, pointSigns, pointDenominator, initialSection: true);
            if (!found) found = CurvedPrismCapWitness.TryGetPoint(geometry, pole, poleSigns, selection.Vertex, selection.Feature,
                point, pointSigns, pointDenominator);
        }
        else found = CurvedPrismCapWitness.TryGetPoint(geometry, selection.Values[..(3 * Words)], selection.Signs[..3],
            selection.Vertex, selection.Feature, point, pointSigns, pointDenominator);
        if (!found) return false;
        Span<ulong> work = stackalloc ulong[6 * words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic pDen = WAt(work, signs, 0), component = WAt(work, signs, 1), temporary = WAt(work, signs, 2);
        pDen.Clear(); pointDenominator.CopyTo(pDen.Rational); pDen.Signs[0] = 1;
        Multiply(pDen, metric, default, denominator);
        for (int axis = 0; axis < 3; axis++)
        {
            component.Clear(); Slot(point, axis, words).CopyTo(component.Rational); component.Signs[0] = pointSigns[axis];
            ContactQuadratic result = WAt(support, supportSigns, axis);
            Multiply(component, metric, default, result);
            Scale(denominator, Component(geometry.Half, axis), component); component.MultiplySign(selection.Feature);
            result.Add(component);
            Multiply(minimum, WAt(normal, normalSigns, axis), default, component);
            Multiply(component, pDen, default, temporary); result.Add(temporary, -1);
        }
        return true;
    }

    private static bool ClipSegment(in CurvedPrismContactGeometry geometry, Span<ulong> start, Span<int> startSigns,
        Span<ulong> delta, Span<int> deltaSigns, ContactQuadratic denominator, ReadOnlySpan<ulong> root,
        ContactQuadratic parameterNumerator, ContactQuadratic parameterDenominator, in FaceWindow window, out bool physical)
    {
        int words = denominator.FieldWords; physical = false;
        Span<ulong> work = stackalloc ulong[16 * words];
        Span<int> signs = stackalloc int[16];
        ContactQuadratic lower = WAt(work, signs, 0), lowerDen = WAt(work, signs, 1);
        ContactQuadratic upper = WAt(work, signs, 2), upperDen = WAt(work, signs, 3);
        ContactQuadratic a = WAt(work, signs, 4), b = WAt(work, signs, 5);
        ContactQuadratic product = WAt(work, signs, 6), comparison = WAt(work, signs, 7);
        lower.Clear(); lowerDen.Set(Integer(1)); upper.Set(Integer(1)); upperDen.Set(Integer(1));
        for (int plane = 0; plane < geometry.PlaneCount; plane++)
        {
            WideDirection3d normal = geometry.Plane(plane);
            if (normal.IsZero) continue;
            for (int orientation = 0; orientation < 2; orientation++, normal = -normal)
            {
                DotQuadratic(normal, start, startSigns, a);
                Scale(denominator, geometry.ProjectionMinimum(normal), product); a.Add(product, -1);
                DotQuadratic(normal, delta, deltaSigns, b);
                int slope = b.Sign(root);
                if (slope == 0)
                {
                    if (window.Enabled)
                    {
                        if (a.Sign(root) < 0) return false;
                    }
#if DEBUG
                    else System.Diagnostics.Debug.Assert(a.Sign(root) >= 0);
#endif
                    continue;
                }
                // a+t*b>=0. Retain exact quadratic endpoints and compare
                // cross-products; no clipped parameter ever rounds.
                a.MultiplySign(-slope); b.MultiplySign(slope);
                ContactQuadratic bound = slope > 0 ? lower : upper;
                ContactQuadratic boundDen = slope > 0 ? lowerDen : upperDen;
                Multiply(a, boundDen, root, comparison);
                Multiply(bound, b, root, product); comparison.Add(product, -1);
                if (comparison.Sign(root) * slope > 0) { a.CopyTo(bound); b.CopyTo(boundDen); }
            }
        }
        if (window.Enabled && !ClipFaceWindow(geometry, window, start, startSigns, delta, deltaSigns,
            denominator, root, lower, lowerDen, upper, upperDen)) return false;
        if (window.Enabled)
        {
            Multiply(lower, upperDen, root, comparison);
            Multiply(upper, lowerDen, root, product); comparison.Add(product, -1);
            if (comparison.Sign(root) > 0) return false;
        }
#if DEBUG
        else
        {
            Multiply(lower, upperDen, root, comparison);
            Multiply(upper, lowerDen, root, product); comparison.Add(product, -1);
            System.Diagnostics.Debug.Assert(comparison.Sign(root) <= 0);
        }
#endif
        if (window.Enabled)
        {
            physical = ClipOriginalBody(geometry, start, startSigns, delta, deltaSigns, denominator, root, lower, lowerDen, upper, upperDen, window);
            Multiply(lower, upperDen, root, comparison); Multiply(upper, lowerDen, root, product); comparison.Add(product, -1);
        }
        // A fixed endpoint is deterministic and avoids multiplying two
        // independent interval denominators just to choose their midpoint.
        lower.CopyTo(parameterNumerator); lowerDen.CopyTo(parameterDenominator);
        if (window.Enabled && window.SourceEdge >= 0 && window.Open && comparison.Sign(root) < 0)
        {
            // The mediant lies strictly between ordered ratios with positive
            // denominators, including quadratic fields, without cross products.
            parameterNumerator.Add(upper); parameterDenominator.Add(upperDen);
        }
        return true;
    }

    private static bool ClipOriginalBody(in CurvedPrismContactGeometry geometry,
        Span<ulong> point, Span<int> pointSigns, Span<ulong> delta, Span<int> deltaSigns,
        ContactQuadratic denominator, ReadOnlySpan<ulong> root, ContactQuadratic lower,
        ContactQuadratic lowerDen, ContactQuadratic upper, ContactQuadratic upperDen, in FaceWindow window)
    {
        int words = denominator.FieldWords;
        Span<ulong> values = stackalloc ulong[16 * words]; Span<int> signs = stackalloc int[16];
        var l = WAt(values, signs, 0); var ld = WAt(values, signs, 1);
        var u = WAt(values, signs, 2); var ud = WAt(values, signs, 3);
        var a = WAt(values, signs, 4); var b = WAt(values, signs, 5);
        var comparison = WAt(values, signs, 6); var work = WAt(values, signs, 7);
        lower.CopyTo(l); lowerDen.CopyTo(ld); upper.CopyTo(u); upperDen.CopyTo(ud);
        var axis = new WideDirection3d(geometry.Axis);
        Signed832 bound = Signed832.ExtendValue(WideArithmetic.Negate(WideArithmetic.MultiplySigned320(geometry.SquaredAxis, geometry.HalfFactor)));
        if (!ClipFaceHalfplane(axis, bound, point, pointSigns, delta, deltaSigns, denominator, root, l, ld, u, ud)
            || !ClipFaceHalfplane(-axis, bound, point, pointSigns, delta, deltaSigns, denominator, root, l, ld, u, ud)) return false;
        GetCurvedImplicit(geometry, point, pointSigns, denominator, root, a);
        Span<ulong> end = stackalloc ulong[6 * words]; Span<int> endSigns = stackalloc int[6];
        for (int coordinate = 0; coordinate < 3; coordinate++)
        { WAt(point, pointSigns, coordinate).CopyTo(WAt(end, endSigns, coordinate)); WAt(end, endSigns, coordinate).Add(WAt(delta, deltaSigns, coordinate)); }
        GetCurvedImplicit(geometry, end, endSigns, denominator, root, b); b.Add(a, -1);
        // Every free analytic family is a cone generator or cylinder axis;
        // both are null directions of their radial quadratic form. Its
        // restriction is therefore LINEAR in t, with no new root field.
        a.MultiplySign(-1); b.MultiplySign(-1);
        if (!ClipBound(a, b, root, l, ld, u, ud)) return false;
        Multiply(l, ud, root, comparison); Multiply(u, ld, root, work); comparison.Add(work, -1);
        if (comparison.Sign(root) > 0) return false;
        if (window.SourceEdge >= 0 && window.Open)
        {
            var n = a; var d = b;
            l.CopyTo(n); ld.CopyTo(d);
            if (comparison.Sign(root) < 0) { n.Add(u); d.Add(ud); }
            Span<ulong> trial = stackalloc ulong[6 * words]; Span<int> trialSigns = stackalloc int[6];
            point.CopyTo(trial); pointSigns.CopyTo(trialSigns);
            Blend(trial, trialSigns, delta, deltaSigns, n, d, root); Multiply(denominator, d, root, a);
            // A body-only endpoint excluded by this open source stratum must
            // not suppress the separately admissible far-boundary family.
            if (!IsInsideRealEdgeWindow(geometry, window, trial, trialSigns, a, root)) return false;
        }
        l.CopyTo(lower); ld.CopyTo(lowerDen); u.CopyTo(upper); ud.CopyTo(upperDen); return true;
    }

    private static void Blend(Span<ulong> start, Span<int> startSigns, Span<ulong> delta, Span<int> deltaSigns,
        ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        int words = start.Length / startSigns.Length;
        Span<ulong> work = stackalloc ulong[4 * words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic first = WAt(work, signs, 0), second = WAt(work, signs, 1);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic point = WAt(start, startSigns, axis);
            Multiply(point, denominator, root, first);
            Multiply(WAt(delta, deltaSigns, axis), numerator, root, second);
            first.Add(second); first.CopyTo(point);
        }
    }

    internal static FixedPointAnchor CreateQuadraticAnchor(in CurvedPrismContactGeometry geometry,
        Vector3d center, FixedQuaternion rotation, Vector3d origin, Span<ulong> point, Span<int> pointSigns,
        ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        int words = point.Length / pointSigns.Length;
        WideOrientedBox.GetRelativeLocalPointNumerators(center, origin, geometry.WorldBasis,
            out Signed192 dx, out Signed192 dy, out Signed192 dz);
        Span<ulong> work = stackalloc ulong[8 * words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic rawDen = WAt(work, signs, 0), finalDen = WAt(work, signs, 1);
        ContactQuadratic numerator = WAt(work, signs, 2), product = WAt(work, signs, 3);
        Scale(denominator, Signed576.ExtendValue(geometry.RawScale), rawDen);
        Scale(rawDen, Signed576.ExtendValue(geometry.WorldBasis.Denominator), finalDen);
        int orientation = finalDen.Sign(root);
        finalDen.MultiplySign(orientation);
        Vector3d first = default, second = default;
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(WAt(point, pointSigns, axis), Signed576.ExtendValue(geometry.WorldBasis.Denominator), numerator);
            Scale(rawDen, Signed576.ExtendValue(axis == 0 ? dx : axis == 1 ? dy : dz), product);
            numerator.Add(product); numerator.MultiplySign(orientation);
            SplitRound(numerator, finalDen, root, out Fixed64 a, out Fixed64 b);
            first[axis] = a; second[axis] = b;
        }
        return new FixedPointAnchor(origin, rotation, first, second);
    }

    private static void DotQuadratic(in WideDirection3d vector, Span<ulong> values, Span<int> signs, ContactQuadratic result)
    {
        Span<ulong> work = stackalloc ulong[2 * result.FieldWords];
        Span<int> workSigns = stackalloc int[2];
        ContactQuadratic product = WAt(work, workSigns, 0);
        result.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(WAt(values, signs, axis), Component(vector, axis), product); result.Add(product);
        }
    }

    private static void DotQuadraticNormal(Span<ulong> first, Span<int> firstSigns, Span<ulong> second,
        Span<int> secondSigns, ReadOnlySpan<ulong> root, ContactQuadratic result)
    {
        Span<ulong> work = stackalloc ulong[2 * result.FieldWords];
        Span<int> signs = stackalloc int[2];
        ContactQuadratic product = WAt(work, signs, 0);
        result.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            Multiply(WAt(first, firstSigns, axis), WAt(second, secondSigns, axis), root, product); result.Add(product);
        }
    }

    private static ContactQuadratic WAt(Span<ulong> values, Span<int> signs, int index) => At(values, signs, index, values.Length / signs.Length);
    private static Signed576 Integer(long value) => Signed576.ExtendValue(Signed192.Signed(value));
}
