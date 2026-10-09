//=======================================================================
// ConePlaneRayEvents.Descriptors.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

internal enum ConePlaneRayEventScope : byte { Plane, Segment }

/// <summary>An unbounded shared plane or one authored finite boundary segment.</summary>
internal readonly struct ConePlaneRayEventSource
{
    internal readonly ConePlaneRayEventScope Scope;
    internal readonly Vector3d A, B;
    internal static ConePlaneRayEventSource Plane => default;
    internal ConePlaneRayEventSource(FixedSegment segment)
    { Scope = ConePlaneRayEventScope.Segment; A = segment.Start; B = segment.End; }
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
    internal bool IsStreaming => visitor != null;
    internal bool HasPoint;

    internal ConePlaneRayEventSink(ConePlaneRayEvent target, ConePlaneRayPoint point, Span<ulong> root,
        bool pointOnly = false, int requestedOrientation = 0)
    { Target = target; outputPoint = point; outputRoot = root; PointOnly = pointOnly; RequestedOrientation = requestedOrientation;
        HasPoint = false; visitor = null; source = default; frame = default; }

    internal ConePlaneRayEventSink(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEventVisitor visitor)
    { this.source = source; this.frame = frame; this.visitor = visitor; Target = default; outputPoint = default; outputRoot = default;
        PointOnly = false; RequestedOrientation = 0; HasPoint = false; }

    internal bool Wants(ConePlaneRayEventKind kind)
    {
        if (!IsStreaming) return Target.Kind == kind;
        if (source.Scope == ConePlaneRayEventScope.Segment)
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

    internal static int CompareEventAnchors(in ConePlaneRayEventSource firstSource, ConePlaneRayEvent first,
        in ConePlaneRayEventSource secondSource, ConePlaneRayEvent second, in ConePlaneRayFrame frame,
        bool includeCoincidentProvenance = true)
    {
        if (first.Matches(second) && firstSource.Scope == secondSource.Scope && firstSource.A == secondSource.A
            && firstSource.B == secondSource.B)
            return 0;
        Span<ulong> av = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<ulong> bv = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> sa = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<int> sb = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<ulong> ar = stackalloc ulong[RootWords];
        Span<ulong> br = stackalloc ulong[RootWords];
        var a = new ConePlaneRayPoint(av, sa); var b = new ConePlaneRayPoint(bv, sb);
        if (!ReconstructPoint(firstSource, frame, first, a, ar)
            || !ReconstructPoint(secondSource, frame, second, b, br))
            throw new InvalidOperationException("An admitted finite event could not be reconstructed.");
        int comparison = a.CompareTo(b, ar, br);
        if (comparison != 0) return comparison;
        return includeCoincidentProvenance
            ? CompareCoincidentProvenance(firstSource, first, secondSource, second) : 0;
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
