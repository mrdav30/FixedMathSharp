//=======================================================================
// ConeSectionPoint.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Borrowed exact authored coordinates of a finite-cone section point.</summary>
internal readonly ref struct ConeSectionPoint
{
    internal const int Words = 10, RootWords = 17, StorageWords = 8 * Words + RootWords;
    internal const int SignCount = 8;
    internal readonly Span<ulong> Values;
    internal readonly Span<int> Signs;
    internal Span<ulong> Root => Values[(8 * Words)..];
    internal ContactQuadratic Coordinate(int axis) => ContactQuadratic.At(Values, Signs, axis, Words);
    internal ContactQuadratic Denominator => ContactQuadratic.At(Values, Signs, 3, Words);

    internal ConeSectionPoint(Span<ulong> values, Span<int> signs)
    {
        Values = values[..StorageWords]; Signs = signs[..SignCount];
    }

    /// <summary>Retains the lexicographically first point of a closed segment/cone section.</summary>
    internal static bool TryGetFirstSegmentPoint(FixedSegment segment,
        in ConeFiniteSectionFrame frame, ConeSectionPoint result)
    {
        result.Values.Clear(); result.Signs.Clear();
        if (ComparePositions(segment.Start, segment.End) > 0)
            segment = new FixedSegment(segment.End, segment.Start);
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic parameter = ContactQuadratic.At(work, signs, 0, Words);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 1, Words);
        ContactQuadratic denominator = result.Denominator;
        if (!TryGetSegmentParameter(segment, frame, false, parameter, denominator, result.Root))
            return false;
        // The shared frame bounds A/B/C below 528 bits, so a parameter
        // numerator/denominator fits 529 bits and its discriminant 1057 bits.
        // Authored raw endpoint differences are below 65 bits; the complete
        // point coefficients fit 595 bits (ten limbs), root fits seventeen.
        for (int axis = 0; axis < 3; axis++)
        {
            Signed192 first = Signed192.Raw(segment.Start[axis]);
            Signed192 difference = WideArithmetic.SubtractSigned192(Signed192.Raw(segment.End[axis]), first);
            ContactQuadratic coordinate = result.Coordinate(axis);
            ContactQuadratic.Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(first)), coordinate);
            ContactQuadratic.Scale(parameter, Signed576.ExtendValue(Signed320.ExtendValue(difference)), term);
            coordinate.Add(term);
        }
        return true;
    }

    /// <summary>Retains a closed cone-section endpoint in the input segment's exact parameter order.</summary>
    /// <remarks>
    /// Numerator/denominator coefficients need 529 bits and root needs 1057
    /// bits. A non-dyadic or irrational endpoint remains admitted without ever
    /// materializing t; callers may form paired coordinates in this same field.
    /// </remarks>
    internal static bool TryGetSegmentParameter(FixedSegment segment, in ConeFiniteSectionFrame frame,
        bool upper, ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root)
    {
        numerator.Clear(); denominator.Clear(); root.Clear();
        WideAxis3 start = frame.Transform(segment.Start), end = frame.Transform(segment.End);
        if (!frame.IntersectsTransformedSegment(start, end)) return false;
        WideAxis3 delta = new(WideArithmetic.SubtractSigned320(end.X, start.X),
            WideArithmetic.SubtractSigned320(end.Y, start.Y), WideArithmetic.SubtractSigned320(end.Z, start.Z));
        bool axial = frame.TryGetAxialInterval(start.Y, delta.Y, out Signed320 lowerN, out Signed320 lowerD,
            out Signed320 upperN, out Signed320 upperD);
        System.Diagnostics.Debug.Assert(axial);
        Signed320 clipN = upper ? upperN : lowerN, clipD = upper ? upperD : lowerD;
        Signed576 a = frame.QuadraticProduct(delta, delta), b = frame.QuadraticProduct(start, delta), c = frame.QuadraticProduct(start, start);
        if (WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(Signed832.ExtendValue(a),
                Signed832.ExtendValue(b), Signed832.ExtendValue(c), clipN, clipD) <= 0)
        {
            numerator.Set(Signed576.ExtendValue(clipN)); denominator.Set(Signed576.ExtendValue(clipD));
            return true;
        }
        Span<ulong> work = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic qa = ContactQuadratic.At(work, signs, 0, Words), qb = ContactQuadratic.At(work, signs, 1, Words);
        ContactQuadratic qc = ContactQuadratic.At(work, signs, 2, Words);
        qa.Set(a); qb.Set(b); qc.Set(c);
        // For A>0 the admitted interval is between the roots; for A<0 it
        // lies outside them. Finite axial clipping excludes the opposite
        // nappe. The linear and repeated-root chart has only branch +1.
        int branch = a.Sign == 0 ? 1 : (upper ? 1 : -1) * a.Sign;
        bool found = ConePlaneRayCharts.TryGetParameter(qa, qb, qc, branch, root, numerator, denominator);
        if (!found)
            found = ConePlaneRayCharts.TryGetParameter(qa, qb, qc, 1, root, numerator, denominator);
        System.Diagnostics.Debug.Assert(found);
        return true;
    }

    internal static int CompareLexicographic(ConeSectionPoint first, ConeSectionPoint second)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            int comparison = ContactQuadratic.CompareRatios(first.Coordinate(axis), first.Denominator, first.Root,
                second.Coordinate(axis), second.Denominator, second.Root);
            if (comparison != 0)
                return comparison;
        }
        return 0;
    }

    private static int ComparePositions(Vector3d first, Vector3d second)
    {
        int comparison = first.X.CompareTo(second.X);
        if (comparison != 0) return comparison;
        comparison = first.Y.CompareTo(second.Y);
        return comparison != 0 ? comparison : first.Z.CompareTo(second.Z);
    }

    internal void CopyTo(ConeSectionPoint destination)
    {
        Values.CopyTo(destination.Values); Signs.CopyTo(destination.Signs);
    }
}
