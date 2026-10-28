// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;

namespace FixedMathSharp.Geometry;

/// <content>Reuses finite-slab active sets on one exact native boundary line.</content>
internal static partial class CurvedPrismContact
{
    // Native callers admit positive-radius/height cylinders with local axis Up.
    // The line chart is A+lambda*E; mu=0 reuses the existing affine LP owner.
    internal static bool TryGetCylinderBoundaryContact(in CurvedPrismContactGeometry geometry, int face,
        in RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper,
        Vector3d center, FixedQuaternion rotation, Vector3d prismOrigin,
        out FixedContactAnchors contact, int boundaryEdge = -1, scoped CurvedPrismContactValue native = default,
        PrismCurvedBoundaryContactVisitor? visitor = null, int ordinal = 0)
    {
        contact = default;
        GetFacePlane(geometry, face, out WideDirection3d normal, out _, out _);
        System.Diagnostics.Debug.Assert(!normal.IsZero);
        Span<Signed576> direction = stackalloc Signed576[3] { normal.X, normal.Y, normal.Z };
        WideArithmetic.ReduceCommonScale(direction); normal = new(direction[0], direction[1], direction[2]);
        if (face < 0) lower = upper = face == -1 ? -geometry.PrismHalfThickness : geometry.PrismHalfThickness;
        GetNativeBoundarySegment(geometry, face >= 0 ? face : boundaryEdge, clip, lower, upper,
            out WideAxis3 a, out WideAxis3 b, out Signed320 coordinateD, out var retained);
        WideAxis3 e = new(WideArithmetic.SubtractSigned320(b.X, a.X), WideArithmetic.SubtractSigned320(b.Y, a.Y), WideArithmetic.SubtractSigned320(b.Z, a.Z));
        var frame = new FiniteSlabRayFrame(geometry.HeightValue, geometry.RadiusValue, coordinateD, normal, WideDirection3d.Dot(normal, a));
        if (!frame.Section.IntersectsTransformedSegment(a, b, retained)) return false;
        // mu=0 makes the affine LP one-dimensional: nonzero minors need
        // that row. Retained coefficients stay <2300 bits, discriminants
        // <2600 and replayed CQ points <2250. Rescaling and final frame
        // materialization stay <2450 bits, so 64-word banks suffice.
        Span<Signed832> rows = stackalloc Signed832[16];
        rows.Clear();
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
        rows[0] = retained.IsClipped ? retained.WideLowerDenominator : one;
        rows[3] = retained.IsClipped ? WideArithmetic.SubtractSigned832(default, retained.WideLowerNumerator) : default;
        rows[4] = retained.IsClipped ? WideArithmetic.SubtractSigned832(default, retained.WideUpperDenominator) : WideArithmetic.SubtractSigned832(default, one);
        rows[7] = retained.IsClipped ? retained.WideUpperNumerator : one;
        rows[9] = one; rows[13] = WideArithmetic.SubtractSigned832(default, one);
        Span<ulong> selectionValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> selectionSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var maximum = new FiniteSlabRaySelection(selectionValues, selectionSigns);
        // Initial line admission used the existing finite-section owner.
        // Clearance retains points that enter the cylinder during translation.
        FiniteSlabRayExtrema.AccumulateNativeLine(frame, a, e, rows, ref maximum);
        // The admitted finite line and positive cylinder have a compact,
        // nonempty ray envelope, so the complete active sets retain a maximum.
        maximum.GetDepth(frame, out Fixed64 depth, out bool clamped);
        const int words = FiniteSlabPolynomial.Words;
        Span<ulong> source = stackalloc ulong[6 * words], exit = stackalloc ulong[6 * words];
        Span<int> sourceSigns = stackalloc int[6], exitSigns = stackalloc int[6];
        Span<ulong> fields = stackalloc ulong[4 * words], root = stackalloc ulong[2 * words + 1];
        Span<int> fieldSigns = stackalloc int[4];
        var denominator = ContactQuadratic.At(fields, fieldSigns, 0, words);
        var term = ContactQuadratic.At(fields, fieldSigns, 1, words);
        maximum.GetNativeLinePointFields(frame, a, e, source, sourceSigns, exit, exitSigns, denominator, root);
        for (int axis = 0; axis < 3; axis++)
        {
            var point = WAt(source, sourceSigns, axis); ContactQuadratic.Scale(point, Signed576.ExtendValue(geometry.RawScale), term); term.CopyTo(point);
            point = WAt(exit, exitSigns, axis); ContactQuadratic.Scale(point, Signed576.ExtendValue(geometry.RawScale), term); term.CopyTo(point);
        }
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(WideArithmetic.AddSigned320(coordinateD, coordinateD)), term); term.CopyTo(denominator);
        FixedPointAnchor shapeAnchor = CreateQuadraticAnchor(geometry, center, rotation, center, exit, exitSigns, denominator, root);
        FixedPointAnchor sourceAnchor = CreateQuadraticAnchor(geometry, center, rotation, prismOrigin, source, sourceSigns, denominator, root);
        contact = new(shapeAnchor, sourceAnchor, normal.Transform(geometry.WorldBasis).GetNormal(false), depth, clamped);
        if (visitor != null) visitor(ordinal, contact, maximum.CompareNativeDepth(geometry, frame, native));
        return true;
    }
}
