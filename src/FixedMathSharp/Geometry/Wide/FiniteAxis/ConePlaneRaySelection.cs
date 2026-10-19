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
    private Signed832 metric;
    private Signed320 scale;
    private readonly int _fieldWords, _rootWords;
    internal ReadOnlySpan<ulong> Root => Values.Slice(4 * _fieldWords, _rootWords);
    internal ConePlaneRayPoint Point => new(Values[(4 * _fieldWords + _rootWords)..], Signs[4..], _fieldWords);

    internal ConePlaneRaySelection(Span<ulong> values, Span<int> signs,
        int fieldWords = FieldWords, int rootWords = RootWords)
    {
        int storageWords = 12 * fieldWords + rootWords;
        if (values.Length < storageWords || signs.Length < SignCount)
            throw new ArgumentException("The plane-ray selection storage is too small.");
        Values = values[..storageWords]; Signs = signs[..SignCount]; HasValue = false;
        _fieldWords = fieldWords; _rootWords = rootWords;
        CurrentEvent = MaximumEvent = default;
        CurrentSource = MaximumSource = default;
        metric = default; scale = default;
    }

    internal bool Keep(scoped ContactQuadratic numerator, scoped ContactQuadratic denominator, scoped ReadOnlySpan<ulong> root,
        in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point)
    {
        if (numerator.Sign(root) < 0 || denominator.Sign(root) <= 0)
            throw new ArgumentException("The admitted depth must be nonnegative with a positive denominator.");
        ContactQuadratic bestN = ContactQuadratic.At(Values, Signs, 0, _fieldWords);
        ContactQuadratic bestD = ContactQuadratic.At(Values, Signs, 1, _fieldWords);
        if (HasValue)
        {
            int comparison = ContactQuadratic.CompareRatios(numerator, denominator, root, bestN, bestD, Root);
            int pointOrder = comparison == 0 ? point.CompareTo(Point, root, Root) : 0;
            if (comparison < 0 || comparison == 0 && (pointOrder > 0 || pointOrder == 0
                    && ConePlaneRayEvents.CompareCoincidentProvenance(CurrentSource, CurrentEvent, MaximumSource, MaximumEvent) >= 0))
                return false;
        }
        numerator.CopyTo(bestN); denominator.CopyTo(bestD);
        Span<ulong> retainedRoot = Values.Slice(4 * _fieldWords, _rootWords);
        root.CopyTo(retainedRoot); retainedRoot[root.Length..].Clear();
        metric = frame.NormalSquared;
        scale = WideArithmetic.AddSigned320(frame.Finite.ShapeFrame.CoordinateDenominator, frame.Finite.ShapeFrame.CoordinateDenominator);
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
        return Keep(ContactQuadratic.At(other.Values, other.Signs, 0, other._fieldWords),
            ContactQuadratic.At(other.Values, other.Signs, 1, other._fieldWords), other.Root, frame, other.Point);
    }

    /// <summary>Restores metadata after reborrowing previously populated winning coefficient/sign storage.</summary>
    internal void Restore(in ConePlaneRayFrame frame, in ConePlaneRayEventSource source, ConePlaneRayEvent descriptor)
    {
        metric = frame.NormalSquared;
        scale = WideArithmetic.AddSigned320(frame.Finite.ShapeFrame.CoordinateDenominator, frame.Finite.ShapeFrame.CoordinateDenominator);
        CurrentSource = MaximumSource = source; CurrentEvent = MaximumEvent = descriptor; HasValue = true;
    }

    internal bool TryMaterialize(in ConePlaneRayFrame frame, int orientation,
        out Vector3d lower, out Vector3d upper, out Fixed64 depth)
        => TryMaterialize(frame, orientation, Vector3d.Zero, out lower, out upper, out depth);

    internal bool TryMaterialize(in ConePlaneRayFrame frame, int orientation, Vector3d samplingOrigin,
        out Vector3d lower, out Vector3d upper, out Fixed64 depth)
    {
        lower = upper = default; depth = default;
        if (orientation != 1 && orientation != -1) throw new ArgumentOutOfRangeException(nameof(orientation));
        return HasValue && ConePlaneRayPointMaterialization.TryGetPointInFrame(frame, Point, Root, samplingOrigin, out lower)
            && TryGetRoundedMaximumDepth(out depth)
            && ConePlaneRayPointMaterialization.TryGetExitPointInFrame(frame, Point, Root,
                ContactQuadratic.At(Values, Signs, 0, _fieldWords), ContactQuadratic.At(Values, Signs, 1, _fieldWords), orientation, samplingOrigin, out upper);
    }

    internal bool TryGetRoundedMaximumDepth(out Fixed64 depth)
    {
        depth = Fixed64.Zero;
        if (!HasValue)
            return false;
        Span<ulong> metricWords = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(metric, metricWords);
        Span<ulong> scaleWords = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(scale, out scaleWords[4], out scaleWords[3], out scaleWords[2], out scaleWords[1], out scaleWords[0]);
        return ContactQuadratic.TryRoundRootRatio(ContactQuadratic.At(Values, Signs, 0, _fieldWords),
            ContactQuadratic.At(Values, Signs, 1, _fieldWords), Root, metricWords, scaleWords, out depth);
    }

    internal Fixed64 GetRoundedMaximumDepth()
    {
        if (!HasValue)
            throw new InvalidOperationException("The plane-ray selection has no admitted value.");
        if (!TryGetRoundedMaximumDepth(out Fixed64 depth))
            throw new OverflowException("The exact plane-ray maximum is outside the Fixed64 range.");
        return depth;
    }

    /// <summary>Compares physical depths across certified coplanar members in the same cone chart.</summary>
    /// <remarks>Member coordinate denominators may differ; the reduced normal metric must agree.</remarks>
    internal int CompareDepth(scoped in ConePlaneRaySelection other)
    {
        if (!HasValue || !other.HasValue)
            throw new InvalidOperationException("Both plane-ray selections must have an admitted value.");
        if (metric.IsZero || !metric.Equals(other.metric))
            throw new ArgumentException("The plane-ray selections must share a nonzero reduced normal metric.");
        ContactQuadratic firstD = ContactQuadratic.At(Values, Signs, 1, _fieldWords);
        ContactQuadratic secondD = ContactQuadratic.At(other.Values, other.Signs, 1, other._fieldWords);
        // sqrt(metric) cancels. Each positive 2D scale adds at most three limbs;
        // the ratio owner's subsequent cross products reserve their full widths.
        int words = Math.Max(ContactQuadratic.ActiveWords(firstD), ContactQuadratic.ActiveWords(secondD)) + 5;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic firstScaled = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic secondScaled = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic.Scale(firstD, Signed576.ExtendValue(scale), firstScaled);
        ContactQuadratic.Scale(secondD, Signed576.ExtendValue(other.scale), secondScaled);
        return ContactQuadratic.CompareRatios(ContactQuadratic.At(Values, Signs, 0, _fieldWords), firstScaled, Root,
            ContactQuadratic.At(other.Values, other.Signs, 0, other._fieldWords), secondScaled, other.Root);
    }
}
