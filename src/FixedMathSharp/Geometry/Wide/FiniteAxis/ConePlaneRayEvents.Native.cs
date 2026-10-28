// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;

namespace FixedMathSharp.Geometry;

/// <content>Streams exact native boundary segments without rounded source descriptors.</content>
internal static partial class ConePlaneRayEvents
{
    internal static bool VisitNativeSegment(in ConePlaneRayFrame frame, in WideAxis3 start, in WideAxis3 end,
        in RationalSegmentInterval clip, ConePlaneRayEventVisitor visitor)
    {
        if (clip.HasWideBounds && !frame.UsesWideBanks)
            return VisitNativeSegment(frame.WithWideParameters(), start, end, clip, visitor);
        Span<ulong> pv = stackalloc ulong[frame.SelectionStorageWords], nv = stackalloc ulong[frame.SelectionStorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps, frame.FieldWords, frame.RootWords);
        var negative = new ConePlaneRaySelection(nv, ns, frame.FieldWords, frame.RootWords);
        Span<WideAxis3> segment = stackalloc WideAxis3[2] { start, end };
        Span<RationalSegmentInterval> clips = stackalloc RationalSegmentInterval[clip.IsClipped ? 1 : 0];
        if (!clips.IsEmpty) clips[0] = clip;
        var source = ConePlaneRayEventSource.NativeSegment;
        var sink = new ConePlaneRayEventSink(source, frame, visitor, clips, segment);
        Accumulate(source, frame, ref positive, ref negative, ref sink);
        return sink.HasPoint;
    }

    internal static bool ContainsNativeSegmentPoint(in WideAxis3 start, in WideAxis3 end,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root, in RationalSegmentInterval clip)
    {
        WideAxis3 delta = new(WideArithmetic.SubtractSigned320(end.X, start.X),
            WideArithmetic.SubtractSigned320(end.Y, start.Y), WideArithmetic.SubtractSigned320(end.Z, start.Z));
        int pivot = !delta.X.IsZero ? 0 : !delta.Y.IsZero ? 1 : 2;
        Signed576 step = Signed576.ExtendValue(pivot == 0 ? delta.X : pivot == 1 ? delta.Y : delta.Z);
        int words = point.BankWords + 6;
        Span<ulong> values = stackalloc ulong[10 * words]; Span<int> signs = stackalloc int[10];
        var numerator = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        var coordinate = ContactQuadratic.At(values, signs, 2, words);
        var first = ContactQuadratic.At(values, signs, 3, words);
        var second = ContactQuadratic.At(values, signs, 4, words);
        ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(pivot == 0 ? start.X : pivot == 1 ? start.Y : start.Z), numerator);
        numerator.MultiplySign(-1); numerator.Add(pivot == 0 ? point.X : pivot == 1 ? point.Y : point.Z);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(axis == 0 ? start.X : axis == 1 ? start.Y : start.Z), coordinate);
            coordinate.MultiplySign(-1); coordinate.Add(axis == 0 ? point.X : axis == 1 ? point.Y : point.Z);
            if (step.IsZero)
            {
                if (coordinate.Sign(root) != 0) return false;
            }
            else
            {
                ContactQuadratic.Scale(coordinate, step, first);
                ContactQuadratic.Scale(numerator, Signed576.ExtendValue(axis == 0 ? delta.X : axis == 1 ? delta.Y : delta.Z), second);
                first.Add(second, -1); if (first.Sign(root) != 0) return false;
            }
        }
        if (step.IsZero) return true;
        ContactQuadratic.Scale(point.Denominator, step, denominator);
        if (denominator.Sign(root) < 0) { numerator.MultiplySign(-1); denominator.MultiplySign(-1); }
        if (clip.IsClipped) return clip.Contains(numerator, denominator, root);
        numerator.CopyTo(first); first.Add(denominator, -1);
        return numerator.Sign(root) >= 0 && first.Sign(root) <= 0;
    }
}
