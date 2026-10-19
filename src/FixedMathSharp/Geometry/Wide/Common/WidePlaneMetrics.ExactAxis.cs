//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Borrowed projected metrics for exact scaled directions without narrowing the plane normal.</content>
internal readonly partial struct WidePlaneMetrics
{
    // Scaled collision directions have components below 400 bits. With full
    // scalar differences, Gram values need <932 bits and squared areas <1064.
    // Caller-owned 2048-bit fields retain complete metrics; ratio comparison
    // supplies its own complete cross-product scratch.
    internal static void GetSpanSquared(in WideDirection3d axis, Vector3d first, Vector3d second,
        scoped ContactQuadratic result) => GetWideGram(axis, Difference(second, first), Difference(second, first), result);

    internal static void GetAreaSquared(in WideDirection3d axis, Vector3d first, Vector3d second, Vector3d third,
        scoped ContactQuadratic result)
    {
        Signed832 triple = GetWideTriple(axis, Difference(second, first), Difference(third, first));
        int words = result.FieldWords;
        Span<ulong> values = stackalloc ulong[2 * words]; Span<int> signs = stackalloc int[2];
        var value = ContactQuadratic.At(values, signs, 0, words); value.Set(triple);
        ContactQuadratic.Multiply(value, value, ReadOnlySpan<ulong>.Empty, result);
    }

    internal static int GetAreaSign(in WideDirection3d axis, Vector3d first, Vector3d second, Vector3d third) =>
        GetWideTriple(axis, Difference(second, first), Difference(third, first)).Sign;

    internal static void GetTriangleDistanceSquared(in WideDirection3d axis, Vector3d point,
        Vector3d first, Vector3d second, Vector3d third, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator)
    {
        WideAxis3 edge = Difference(second, first), offset = Difference(point, first);
        int orientation = GetWideTriple(axis, edge, Difference(third, first)).Sign;
        if (orientation != 0 && orientation * GetWideTriple(axis, edge, offset).Sign >= 0
            && orientation * GetWideTriple(axis, Difference(third, second), Difference(point, second)).Sign >= 0
            && orientation * GetWideTriple(axis, Difference(first, third), Difference(point, third)).Sign >= 0)
        {
            numerator.Clear(); denominator.Set(axis.SquaredLength); return;
        }
        GetWideSegmentDistance(axis, point, first, second, numerator, denominator);
        int words = numerator.FieldWords;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var nextN = ContactQuadratic.At(values, signs, 0, words);
        var nextD = ContactQuadratic.At(values, signs, 1, words);
        GetWideSegmentDistance(axis, point, second, third, nextN, nextD);
        if (ContactQuadratic.CompareRatios(nextN, nextD, ReadOnlySpan<ulong>.Empty,
            numerator, denominator, ReadOnlySpan<ulong>.Empty) < 0) { nextN.CopyTo(numerator); nextD.CopyTo(denominator); }
        GetWideSegmentDistance(axis, point, third, first, nextN, nextD);
        if (ContactQuadratic.CompareRatios(nextN, nextD, ReadOnlySpan<ulong>.Empty,
            numerator, denominator, ReadOnlySpan<ulong>.Empty) < 0) { nextN.CopyTo(numerator); nextD.CopyTo(denominator); }
    }

    private static void GetWideSegmentDistance(in WideDirection3d axis, Vector3d point, Vector3d first, Vector3d second,
        scoped ContactQuadratic numerator, scoped ContactQuadratic denominator)
    {
        int words = numerator.FieldWords;
        Span<ulong> values = stackalloc ulong[6 * words]; Span<int> signs = stackalloc int[6];
        var length = ContactQuadratic.At(values, signs, 0, words);
        var projection = ContactQuadratic.At(values, signs, 1, words);
        var remainder = ContactQuadratic.At(values, signs, 2, words);
        WideAxis3 edge = Difference(second, first), offset = Difference(point, first);
        GetWideGram(axis, edge, edge, length); GetWideGram(axis, offset, edge, projection);
        projection.CopyTo(remainder); remainder.Add(length, -1);
        if (length.Signs[0] == 0 || projection.Signs[0] <= 0)
        { GetWideGram(axis, offset, offset, numerator); denominator.Set(axis.SquaredLength); }
        else if (remainder.Signs[0] >= 0)
        { offset = Difference(point, second); GetWideGram(axis, offset, offset, numerator); denominator.Set(axis.SquaredLength); }
        else
        {
            // Cancel the common normal metric before squaring the triple product.
            projection.Set(GetWideTriple(axis, offset, edge));
            ContactQuadratic.Multiply(projection, projection, ReadOnlySpan<ulong>.Empty, numerator);
            length.CopyTo(denominator);
        }
    }

    private static void GetWideGram(in WideDirection3d axis, in WideAxis3 first, in WideAxis3 second,
        scoped ContactQuadratic result)
    {
        int words = result.FieldWords;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var metric = ContactQuadratic.At(values, signs, 0, words);
        var projectionA = ContactQuadratic.At(values, signs, 1, words);
        var projectionB = ContactQuadratic.At(values, signs, 2, words);
        var product = ContactQuadratic.At(values, signs, 3, words);
        metric.Set(axis.SquaredLength);
        ContactQuadratic.Scale(metric, WideAxis3.Dot(first, second), result);
        projectionA.Set(WideDirection3d.Dot(axis, first)); projectionB.Set(WideDirection3d.Dot(axis, second));
        ContactQuadratic.Multiply(projectionA, projectionB, ReadOnlySpan<ulong>.Empty, product);
        result.Add(product, -1);
    }

    private static Signed832 GetWideTriple(in WideDirection3d axis, in WideAxis3 first, in WideAxis3 second) =>
        WideDirection3d.Dot(axis, WideAxis3.Cross(first, second));
}
