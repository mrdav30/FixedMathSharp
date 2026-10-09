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
    internal const int StorageWords = 4 * FieldWords + RootWords + ConePlaneRayPoint.StorageWords;
    internal const int SignCount = 4 + ConePlaneRayPoint.SignCount;
    internal readonly Span<ulong> Values;
    internal readonly Span<int> Signs;
    internal bool HasValue;
    internal ConePlaneRayEvent CurrentEvent, MaximumEvent;
    internal ConePlaneRayEventSource CurrentSource, MaximumSource;
    private Signed576 metric;
    private Signed192 scale;
    internal ReadOnlySpan<ulong> Root => Values.Slice(4 * FieldWords, RootWords);
    internal ConePlaneRayPoint Point => new(Values[(4 * FieldWords + RootWords)..], Signs[4..]);

    internal ConePlaneRaySelection(Span<ulong> values, Span<int> signs)
    {
        if (values.Length < StorageWords || signs.Length < SignCount)
            throw new ArgumentException("The plane-ray selection storage is too small.");
        Values = values[..StorageWords]; Signs = signs[..SignCount]; HasValue = false;
        CurrentEvent = MaximumEvent = default;
        CurrentSource = MaximumSource = default;
        metric = default; scale = default;
    }

    internal bool Keep(scoped ContactQuadratic numerator, scoped ContactQuadratic denominator, scoped ReadOnlySpan<ulong> root,
        in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point)
    {
        if (numerator.Sign(root) < 0 || denominator.Sign(root) <= 0)
            throw new ArgumentException("The admitted depth must be nonnegative with a positive denominator.");
        ContactQuadratic bestN = ContactQuadratic.At(Values, Signs, 0, FieldWords);
        ContactQuadratic bestD = ContactQuadratic.At(Values, Signs, 1, FieldWords);
        if (HasValue)
        {
            int comparison = ContactQuadratic.CompareRatios(numerator, denominator, root, bestN, bestD, Root);
            int pointOrder = comparison == 0 ? point.CompareTo(Point, root, Root) : 0;
            if (comparison < 0 || comparison == 0 && (pointOrder > 0 || pointOrder == 0
                    && ConePlaneRayEvents.CompareCoincidentProvenance(CurrentSource, CurrentEvent, MaximumSource, MaximumEvent) >= 0))
                return false;
        }
        numerator.CopyTo(bestN); denominator.CopyTo(bestD);
        Span<ulong> retainedRoot = Values.Slice(4 * FieldWords, RootWords);
        root.CopyTo(retainedRoot); retainedRoot[root.Length..].Clear();
        metric = frame.NormalSquared;
        scale = WideArithmetic.AddSigned192(frame.Finite.ShapeFrame.Denominator, frame.Finite.ShapeFrame.Denominator);
        HasValue = true;
        MaximumEvent = CurrentEvent;
        MaximumSource = CurrentSource;
        // Rational points have zero radical coefficients, so the exit's root
        // also represents them. Side and certified exits already share the
        // point field. Retain only the winning point, never a wide event pool.
        point.CopyTo(Point);
        return true;
    }

    internal bool KeepEvaluated(scoped in ConePlaneRaySelection other, in ConePlaneRayFrame frame)
    {
        if (!other.HasValue) return false;
        CurrentSource = other.MaximumSource; CurrentEvent = other.MaximumEvent;
        return Keep(ContactQuadratic.At(other.Values, other.Signs, 0, FieldWords),
            ContactQuadratic.At(other.Values, other.Signs, 1, FieldWords), other.Root, frame, other.Point);
    }

    /// <summary>Restores metadata after reborrowing previously populated winning coefficient/sign storage.</summary>
    internal void Restore(in ConePlaneRayFrame frame, in ConePlaneRayEventSource source, ConePlaneRayEvent descriptor)
    {
        metric = frame.NormalSquared;
        scale = WideArithmetic.AddSigned192(frame.Finite.ShapeFrame.Denominator, frame.Finite.ShapeFrame.Denominator);
        CurrentSource = MaximumSource = source; CurrentEvent = MaximumEvent = descriptor; HasValue = true;
    }

    internal bool TryMaterialize(in ConePlaneRayFrame frame, int orientation,
        out Vector3d lower, out Vector3d upper, out Fixed64 depth)
    {
        lower = upper = default; depth = default;
        if (orientation != 1 && orientation != -1) throw new ArgumentOutOfRangeException(nameof(orientation));
        return HasValue && ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, Point, Root, out lower)
            && TryGetRoundedMaximumDepth(out depth)
            && ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, Point, Root,
                ContactQuadratic.At(Values, Signs, 0, FieldWords), ContactQuadratic.At(Values, Signs, 1, FieldWords), orientation, out upper);
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
