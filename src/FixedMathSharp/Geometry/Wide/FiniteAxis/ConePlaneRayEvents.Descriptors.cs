//=======================================================================
// ConePlaneRayEvents.Descriptors.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;













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
