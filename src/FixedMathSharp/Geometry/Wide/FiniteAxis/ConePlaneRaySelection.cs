//=======================================================================
// ConePlaneRaySelection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Caller-owned storage for an admitted, unrounded plane-ray maximum.</summary>
internal ref struct ConePlaneRaySelection
{
    internal const int FieldWords = 64;
    internal const int RootWords = 40;
    internal const int StorageWords = 4 * FieldWords + RootWords;
    internal const int SignCount = 4;
    internal readonly Span<ulong> Values;
    internal readonly Span<int> Signs;
    internal bool HasValue;
    internal ConePlaneRayEvent CurrentEvent, MaximumEvent;
    internal FixedTriangle CurrentTriangle, MaximumTriangle;
    private Signed576 metric;
    private Signed192 scale;
    internal ReadOnlySpan<ulong> Root => Values[(4 * FieldWords)..];

    internal ConePlaneRaySelection(Span<ulong> values, Span<int> signs)
    {
        if (values.Length < StorageWords || signs.Length < SignCount)
            throw new ArgumentException("The plane-ray selection storage is too small.");
        Values = values[..StorageWords]; Signs = signs[..SignCount]; HasValue = false;
        CurrentEvent = MaximumEvent = default;
        CurrentTriangle = MaximumTriangle = default;
        metric = default; scale = default;
    }

    internal void Keep(scoped ContactQuadratic numerator, scoped ContactQuadratic denominator, scoped ReadOnlySpan<ulong> root,
        in ConePlaneRayFrame frame)
    {
        if (numerator.Sign(root) < 0 || denominator.Sign(root) <= 0)
            throw new ArgumentException("The admitted depth must be nonnegative with a positive denominator.");
        ContactQuadratic bestN = ContactQuadratic.At(Values, Signs, 0, FieldWords);
        ContactQuadratic bestD = ContactQuadratic.At(Values, Signs, 1, FieldWords);
        if (HasValue)
        {
            int comparison = ContactQuadratic.CompareRatios(numerator, denominator, root, bestN, bestD, Root);
            // Every event dispatcher supplies provenance before admission;
            // equal depths therefore always compare their exact anchors.
            if (comparison < 0 || comparison == 0 && ConePlaneRayEvents.CompareEventAnchors(
                    CurrentTriangle, CurrentEvent, MaximumTriangle, MaximumEvent, frame) >= 0)
                return;
        }
        numerator.CopyTo(bestN); denominator.CopyTo(bestD);
        Span<ulong> retainedRoot = Values.Slice(4 * FieldWords, RootWords);
        root.CopyTo(retainedRoot); retainedRoot[root.Length..].Clear();
        metric = frame.NormalSquared;
        scale = WideArithmetic.AddSigned192(frame.Finite.ShapeFrame.Denominator, frame.Finite.ShapeFrame.Denominator);
        HasValue = true;
        MaximumEvent = CurrentEvent;
        MaximumTriangle = CurrentTriangle;
    }

    internal bool TryGetRoundedMaximumDepth(out Fixed64 depth)
    {
        depth = Fixed64.Zero;
        if (!HasValue)
            return false;
        Span<ulong> metricWords = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(metric, metricWords);
        Span<ulong> scaleWords = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(scale, out scaleWords[2], out scaleWords[1], out scaleWords[0]);
        return ContactQuadratic.TryRoundRootRatio(ContactQuadratic.At(Values, Signs, 0, FieldWords),
            ContactQuadratic.At(Values, Signs, 1, FieldWords), Root, metricWords, scaleWords, out depth);
    }

    internal Fixed64 GetRoundedMaximumDepth()
    {
        if (!HasValue)
            throw new InvalidOperationException("The plane-ray selection has no admitted value.");
        if (!TryGetRoundedMaximumDepth(out Fixed64 depth))
            throw new OverflowException("The exact plane-ray maximum is outside the Fixed64 range.");
        return depth;
    }
}
