//=======================================================================
// ConePlaneRayEvents.Descriptors.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

internal enum ConePlaneRayEventScope : byte { Plane, Segment, NativeSegment }

/// <summary>An unbounded shared plane or one authored finite boundary segment.</summary>
internal readonly struct ConePlaneRayEventSource
{
    internal readonly ConePlaneRayEventScope Scope;
    internal bool IsSegment => Scope != ConePlaneRayEventScope.Plane;
    internal readonly Vector3d A, B;
    internal static ConePlaneRayEventSource Plane => default;
    internal ConePlaneRayEventSource(FixedSegment segment)
    { Scope = ConePlaneRayEventScope.Segment; A = segment.Start; B = segment.End; }
    // Native rational cuts are borrowed by the synchronous sink, never encoded
    // as rounded authored endpoints or replayed from this provenance record.
    internal static ConePlaneRayEventSource NativeSegment => new(ConePlaneRayEventScope.NativeSegment);
    private ConePlaneRayEventSource(ConePlaneRayEventScope scope)
    { Scope = scope; A = B = default; }
}

internal enum ConePlaneRayEventKind : byte
{
    EdgeEndpoint, EdgeSide, EdgeStationary, LowerCircle, UpperRim, Generator,
    Axis, Apex, BaseStationary
}

/// <summary>A reconstructible finite feature in its shared plane or finite segment.</summary>
internal readonly struct ConePlaneRayEvent
{
    internal readonly ConePlaneRayEventKind Kind;
    internal readonly byte Feature, Quadrant;
    internal readonly sbyte Branch, Endpoint, Orientation;

    internal ConePlaneRayEvent(ConePlaneRayEventKind kind, int feature = 0, int quadrant = 0,
        int branch = 0, int endpoint = -1, int orientation = 0)
    {
        Kind = kind; Feature = (byte)feature; Quadrant = (byte)quadrant;
        Branch = (sbyte)branch; Endpoint = (sbyte)endpoint; Orientation = (sbyte)orientation;
    }

    internal bool Matches(in ConePlaneRayEvent other) => Kind == other.Kind && Feature == other.Feature
        && Quadrant == other.Quadrant && Branch == other.Branch && Endpoint == other.Endpoint;


}

/// <summary>Synchronous consumption of one admitted event; borrowed fields cannot escape the callback.</summary>
internal delegate void ConePlaneRayEventVisitor(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
    ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
    scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative);

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

/// <content>Compact candidate provenance and direct event reconstruction.</content>
internal static partial class ConePlaneRayEvents
{
    private static void SetEvent(in ConePlaneRayEventSource source, in ConePlaneRayEvent descriptor,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative)
    {
        positive.HasValue = negative.HasValue = false;
        positive.CurrentEvent = descriptor; negative.CurrentEvent = descriptor;
        positive.CurrentSource = source; negative.CurrentSource = source;
    }

    private static bool ReconstructPoint(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped Span<ulong> root)
    {
        // Point-only routes never read selection storage or solve exits.
        scoped ConePlaneRaySelection positive = default, negative = default;
        var sink = new ConePlaneRayEventSink(descriptor, point, root, pointOnly: true);
        Accumulate(source, frame, ref positive, ref negative, ref sink);
        return sink.HasPoint;
    }

    /// <summary>Reconstructs only a clipped event's admitted point, without solving unused exits.</summary>
    internal static bool ReconstructPoint(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        ConePlaneRayEvent descriptor, in RationalSegmentInterval clip, scoped ConePlaneRayPoint point, scoped Span<ulong> root)
    {
        if (!clip.IsClipped) return ReconstructPoint(source, frame, descriptor, point, root);
        if (source.Scope != ConePlaneRayEventScope.Segment)
            throw new ArgumentException("Only authored segment sources can be replayed with a clipped interval.", nameof(source));
        scoped ConePlaneRaySelection positive = default, negative = default;
        Span<RationalSegmentInterval> clips = stackalloc RationalSegmentInterval[1] { clip };
        var sink = new ConePlaneRayEventSink(descriptor, point, root, pointOnly: true,
            source: source, frame: frame, clip: clips);
        Accumulate(source, frame, ref positive, ref negative, ref sink);
        return sink.HasPoint;
    }

    internal static int CompareEventAnchors(in ConePlaneRayEventSource firstSource, ConePlaneRayEvent first,
        in ConePlaneRayEventSource secondSource, ConePlaneRayEvent second, in ConePlaneRayFrame frame,
        bool includeCoincidentProvenance = true)
    {
        if (first.Matches(second) && firstSource.Scope == secondSource.Scope && firstSource.A == secondSource.A
            && firstSource.B == secondSource.B)
            return 0;
        Span<ulong> av = stackalloc ulong[frame.PointStorageWords];
        Span<int> sa = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<ulong> ar = stackalloc ulong[frame.RootWords];
        var a = new ConePlaneRayPoint(av, sa, frame.FieldWords);
        if (!ReconstructPoint(firstSource, frame, first, a, ar))
            throw new InvalidOperationException("An admitted finite event could not be reconstructed.");
        int comparison = CompareEventAnchors(a, ar, secondSource, second, frame);
        if (comparison != 0) return comparison;
        return includeCoincidentProvenance
            ? CompareCoincidentProvenance(firstSource, first, secondSource, second) : 0;
    }

    /// <summary>Compares an admitted point with a reconstructed event by exact coordinates.</summary>
    /// <remarks>The borrowed first point and root already belong to the supplied frame; provenance does not participate.</remarks>
    internal static int CompareEventAnchors(scoped ConePlaneRayPoint first, scoped ReadOnlySpan<ulong> firstRoot,
        in ConePlaneRayEventSource secondSource, ConePlaneRayEvent second, in ConePlaneRayFrame frame)
    {
        Span<ulong> values = stackalloc ulong[frame.PointStorageWords], root = stackalloc ulong[frame.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs, frame.FieldWords);
        if (!ReconstructPoint(secondSource, frame, second, point, root))
            throw new InvalidOperationException("An admitted finite event could not be reconstructed.");
        return first.CompareTo(point, firstRoot, root);
    }

    internal static int CompareCoincidentProvenance(in ConePlaneRayEventSource firstSource, ConePlaneRayEvent first,
        in ConePlaneRayEventSource secondSource, ConePlaneRayEvent second)
    {
        int scopeOrder = ((byte)firstSource.Scope).CompareTo((byte)secondSource.Scope);
        if (scopeOrder != 0) return scopeOrder;
        Span<Vector3d> a = stackalloc Vector3d[2] { firstSource.A, firstSource.B };
        Span<Vector3d> b = stackalloc Vector3d[2] { secondSource.A, secondSource.B };
        SortVertices(a); SortVertices(b);
        for (int i = 0; i < 2; i++)
        {
            int comparison = CompareVertex(a[i], b[i]);
            if (comparison != 0) return comparison;
        }
        int order = ((int)first.Kind).CompareTo((int)second.Kind);
        if (order == 0) order = first.Feature.CompareTo(second.Feature);
        if (order == 0) order = first.Quadrant.CompareTo(second.Quadrant);
        if (order == 0) order = first.Branch.CompareTo(second.Branch);
        return order == 0 ? first.Endpoint.CompareTo(second.Endpoint) : order;
    }

    private static void SortVertices(Span<Vector3d> vertices)
    {
        for (int i = 1; i < 2; i++)
            for (int j = i; j > 0 && CompareVertex(vertices[j], vertices[j - 1]) < 0; j--)
                (vertices[j], vertices[j - 1]) = (vertices[j - 1], vertices[j]);
    }

    private static int CompareVertex(Vector3d a, Vector3d b)
    {
        int order = a.X.m_rawValue.CompareTo(b.X.m_rawValue);
        if (order == 0) order = a.Y.m_rawValue.CompareTo(b.Y.m_rawValue);
        return order == 0 ? a.Z.m_rawValue.CompareTo(b.Z.m_rawValue) : order;
    }
}
