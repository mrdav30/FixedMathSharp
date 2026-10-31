//=======================================================================
// ConePlaneRayEventSink.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>One borrowed exact output point or synchronous visitor; point-only replay does not solve unused exits.</summary>
internal ref struct ConePlaneRayEventSink
{
    internal readonly ConePlaneRayEvent Target;
    internal readonly bool PointOnly;
    internal readonly int RequestedOrientation;
    private readonly ConePlaneRayPoint outputPoint;
    private readonly Span<ulong> outputRoot;
    private readonly ConePlaneRayEventVisitor? visitor;
    private readonly ConePlaneRayEventSource source;
    private readonly ConePlaneRayFrame frame;
    internal readonly ReadOnlySpan<RationalSegmentInterval> Clip;
    internal readonly ReadOnlySpan<WideAxis3> NativeSegment;
    internal bool IsStreaming => visitor != null;
    internal bool HasPoint;

    internal ConePlaneRayEventSink(ConePlaneRayEvent target, ConePlaneRayPoint point, Span<ulong> root,
        bool pointOnly = false, int requestedOrientation = 0, in ConePlaneRayEventSource source = default,
        in ConePlaneRayFrame frame = default, ReadOnlySpan<RationalSegmentInterval> clip = default)
    { Target = target; outputPoint = point; outputRoot = root; PointOnly = pointOnly; RequestedOrientation = requestedOrientation;
        HasPoint = false; visitor = null; this.source = source; this.frame = frame; Clip = clip; NativeSegment = default; }

    internal ConePlaneRayEventSink(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEventVisitor visitor,
        ReadOnlySpan<RationalSegmentInterval> clip = default, ReadOnlySpan<WideAxis3> nativeSegment = default)
    { this.source = source; this.frame = frame; this.visitor = visitor; Target = default; outputPoint = default; outputRoot = default;
        PointOnly = false; RequestedOrientation = 0; HasPoint = false; Clip = clip; NativeSegment = nativeSegment; }

    internal void GetSegment(in ConePlaneRayEventSource currentSource, in ConePlaneRayFrame currentFrame, out WideAxis3 start, out WideAxis3 end)
    {
        if (!NativeSegment.IsEmpty) { start = NativeSegment[0]; end = NativeSegment[1]; }
        else { start = currentFrame.Transform(currentSource.A); end = currentFrame.Transform(currentSource.B); }
    }

    internal bool Wants(ConePlaneRayEventKind kind)
    {
        if (!IsStreaming) return Target.Kind == kind;
        if (source.IsSegment)
            return kind == ConePlaneRayEventKind.EdgeEndpoint || kind == ConePlaneRayEventKind.EdgeSide
                || kind == ConePlaneRayEventKind.EdgeStationary || kind == ConePlaneRayEventKind.UpperRim;
        if (frame.Normal.X.IsZero && frame.Normal.Z.IsZero && !frame.Normal.Y.IsZero)
            return kind == ConePlaneRayEventKind.Axis || kind == ConePlaneRayEventKind.LowerCircle;
        // The dispatcher invokes edge construction only for segment sources;
        // every remaining general-plane stratum belongs to its inventory.
        return true;
    }
    internal bool Wants(in ConePlaneRayEvent descriptor) => IsStreaming ? Wants(descriptor.Kind) : Target.Matches(descriptor);
    internal void Keep(in ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
        scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
    {
        if (!Wants(descriptor)) return;
        if (!NativeSegment.IsEmpty)
        {
            if (!ConePlaneRayEvents.ContainsNativeSegmentPoint(NativeSegment[0], NativeSegment[1], point, root,
                    Clip.IsEmpty ? default : Clip[0])) return;
        }
        else if (!Clip.IsEmpty && !ConePlaneRayEvents.IsPointInClip(source, frame, point, root, Clip[0])) return;
        if (visitor != null)
        {
            visitor(source, frame, descriptor, point, root, positive, negative);
            HasPoint = true;
            return;
        }
        point.CopyTo(outputPoint);
        root.CopyTo(outputRoot); outputRoot[root.Length..].Clear();
        HasPoint = true;
    }
}
