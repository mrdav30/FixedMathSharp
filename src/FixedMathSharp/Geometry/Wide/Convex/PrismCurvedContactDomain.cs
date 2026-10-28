// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;

namespace FixedMathSharp.Geometry;

/// <summary>An open real-edge interval or closed point with one exact incident support fan.</summary>
internal readonly struct PrismCurvedEdgeWindow
{
    internal readonly int Edge, ReferenceVertex, FanStart, FanCount;
    internal readonly RationalSegmentInterval Interval;
    internal readonly bool Point;
    internal PrismCurvedEdgeWindow(int edge, RationalSegmentInterval interval, bool point,
        int referenceVertex, int fanStart, int fanCount)
    { Edge = edge; Interval = interval; Point = point; ReferenceVertex = referenceVertex; FanStart = fanStart; FanCount = fanCount; }
}

/// <summary>A finite full-support witness certified on the same oriented source plane.</summary>
/// <remarks>The producer certifies exact plane equality; payload rounding never ranks candidates.</remarks>
internal readonly struct PrismCurvedFaceCertificate
{
    internal readonly int Face;
    internal readonly FixedContactAnchors Contact;
    internal readonly bool Physical;
    internal PrismCurvedFaceCertificate(int face, FixedContactAnchors contact, bool physical)
    { Face = face; Contact = contact; Physical = physical; }
}

/// <summary>A retained native boundary line competing with one convex source's complete exit.</summary>
internal readonly struct PrismCurvedBoundaryLine
{
    internal readonly int Face, BoundaryEdge;
    internal readonly RationalSegmentInterval Clip;
    internal readonly Fixed64 Lower, Upper;
    internal PrismCurvedBoundaryLine(int face, RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper, int boundaryEdge = -1)
    { Face = face; Clip = clip; Lower = lower; Upper = upper; BoundaryEdge = boundaryEdge; }
}

/// <summary>Streams a line's finite contact and exact depth order relative to the native exit.</summary>
internal delegate void PrismCurvedBoundaryContactVisitor(int ordinal, in FixedContactAnchors contact, int compareToNative);

/// <summary>Borrowed native minimum; comparisons never retain or round its wide value.</summary>
internal readonly ref struct CurvedPrismContactValue
{
    private readonly ConvexContactCandidate _analytic;
    private readonly CurvedPrismRimContacts.Selection _rim;
    private readonly bool _hasValue;
    internal bool HasValue => _hasValue;
    internal CurvedPrismContactValue(ConvexContactCandidate analytic, bool hasAnalytic, in CurvedPrismRimContacts.Selection rim)
    { _analytic = analytic; _rim = rim; _hasValue = hasAnalytic || rim.HasValue; }

    // Compares N*sqrt(metric)/(D*|scale|) with the native nonnegative gap.
    // Squaring preserves order. Keep the CQ denominator instead of multiplying
    // by its conjugate: the existing ratio owner already compares two fields.
    internal int CompareRootRatio(in CurvedPrismContactGeometry geometry, scoped ContactQuadratic numerator,
        scoped ContactQuadratic denominator, scoped ReadOnlySpan<ulong> root, Signed832 metric, Signed320 scale)
    {
        if (!HasValue) return -1;
        Span<ulong> metricWords = stackalloc ulong[13], scaleWords = stackalloc ulong[5], squaredScale = stackalloc ulong[10];
        WideArithmetic.GetMagnitude(metric, metricWords);
        WideArithmetic.GetMagnitude(scale, out scaleWords[4], out scaleWords[3], out scaleWords[2], out scaleWords[1], out scaleWords[0]);
        WideArithmetic.MultiplyMagnitudes(scaleWords, scaleWords, squaredScale);
        int words = 2 * Math.Max(ContactQuadratic.ActiveWords(numerator), ContactQuadratic.ActiveWords(denominator))
            + WideArithmetic.GetActiveMagnitudeLength(root)
            + Math.Max(WideArithmetic.GetActiveMagnitudeLength(metricWords), WideArithmetic.GetActiveMagnitudeLength(squaredScale)) + 2;
        Span<ulong> fields = stackalloc ulong[6 * words]; Span<int> signs = stackalloc int[6];
        var n = ContactQuadratic.At(fields, signs, 0, words); var d = ContactQuadratic.At(fields, signs, 1, words);
        var term = ContactQuadratic.At(fields, signs, 2, words);
        ContactQuadratic.Multiply(numerator, numerator, root, n); ContactQuadratic.Scale(n, metricWords, 1, term); term.CopyTo(n);
        ContactQuadratic.Multiply(denominator, denominator, root, d); ContactQuadratic.Scale(d, squaredScale, 1, term); term.CopyTo(d);
        if (_rim.HasValue) return -CurvedPrismRimContacts.CompareSquaredDepth(geometry, _rim, n, d, root);
        int analyticWords = Math.Max(1, Math.Max(WideArithmetic.GetActiveMagnitudeLength(_analytic.GapDenominator),
            Math.Max(WideArithmetic.GetActiveMagnitudeLength(_analytic.GapRational), WideArithmetic.GetActiveMagnitudeLength(_analytic.GapRadical))));
        Span<ulong> analyticValues = stackalloc ulong[4 * analyticWords]; Span<int> analyticSigns = stackalloc int[4];
        var a = ContactQuadratic.At(analyticValues, analyticSigns, 0, analyticWords); var b = ContactQuadratic.At(analyticValues, analyticSigns, 1, analyticWords);
        a.Clear(); b.Clear();
        _analytic.GapRational[..WideArithmetic.GetActiveMagnitudeLength(_analytic.GapRational)].CopyTo(a.Rational);
        _analytic.GapRadical[..WideArithmetic.GetActiveMagnitudeLength(_analytic.GapRadical)].CopyTo(a.Radical);
        a.Signs[0] = _analytic.GapRationalSign; a.Signs[1] = _analytic.GapRadicalSign;
        _analytic.GapDenominator[..WideArithmetic.GetActiveMagnitudeLength(_analytic.GapDenominator)].CopyTo(b.Rational); b.Signs[0] = 1;
        return ContactQuadratic.CompareRatios(n, d, root, a, b, _analytic.GapRadicand);
    }
}

/// <summary>Borrowed source windows and primitive-local support covectors; owns no topology or storage.</summary>
internal readonly ref struct PrismCurvedContactDomain
{
    internal readonly bool Enabled, IncludeFarBoundary;
    internal readonly ReadOnlySpan<PrismCurvedEdgeWindow> Edges;
    internal readonly ReadOnlySpan<WideDirection3d> FanCovectors;
    internal readonly ReadOnlySpan<PrismCurvedFaceCertificate> Certificates;
    internal PrismCurvedContactDomain(ReadOnlySpan<PrismCurvedEdgeWindow> edges, ReadOnlySpan<WideDirection3d> fanCovectors, bool includeFarBoundary = false, ReadOnlySpan<PrismCurvedFaceCertificate> certificates = default)
    { Enabled = true; IncludeFarBoundary = includeFarBoundary; Edges = edges; FanCovectors = fanCovectors; Certificates = certificates; }
}
