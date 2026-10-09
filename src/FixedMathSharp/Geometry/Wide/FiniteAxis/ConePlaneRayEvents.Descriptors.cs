//=======================================================================
// ConePlaneRayEvents.Descriptors.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

internal enum ConePlaneRayEventKind : byte
{
    EdgeEndpoint, EdgeSide, EdgeStationary, LowerCircle, UpperRim, Generator,
    Axis, Apex, BaseStationary
}

/// <summary>A reconstructible finite feature in its retained triangle and shared frame.</summary>
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

    internal int ReconstructionCount => Endpoint == -2 ? 5 : 1;
    internal ConePlaneRayEvent GetReconstruction(int index)
    {
        if ((uint)index >= ReconstructionCount) throw new ArgumentOutOfRangeException(nameof(index));
        return Endpoint == -2 ? new ConePlaneRayEvent(Kind, Feature, Quadrant, Branch, index, Orientation) : this;
    }
}

/// <summary>Caller-owned per-triangle event storage with optional single-event reconstruction.</summary>
internal ref struct ConePlaneRayEventSink
{
    internal const int Capacity = 64;
    private readonly Span<ConePlaneRayEvent> events;
    internal int Count;
    internal readonly bool IsReconstruction;
    internal readonly ConePlaneRayEvent Target;
    private readonly ConePlaneRayPoint outputPoint;
    private readonly Span<ulong> outputRoot;
    internal bool HasPoint;

    internal ConePlaneRayEventSink(Span<ConePlaneRayEvent> events)
    {
        if (events.Length < Capacity)
            throw new ArgumentException("The finite cone event sink needs 64 descriptor slots.", nameof(events));
        this.events = events; Count = 0; IsReconstruction = false; Target = default;
        outputPoint = default; outputRoot = default; HasPoint = false;
    }

    internal ConePlaneRayEventSink(ConePlaneRayEvent target, ConePlaneRayPoint point, Span<ulong> root)
    {
        events = default; Count = 0; IsReconstruction = true; Target = target;
        outputPoint = point; outputRoot = root; HasPoint = false;
    }

    internal bool Wants(ConePlaneRayEventKind kind) => !IsReconstruction || Target.Kind == kind;
    internal bool Wants(in ConePlaneRayEvent descriptor) => !IsReconstruction || Target.Matches(descriptor);

    internal void Keep(in ConePlaneRayEvent descriptor, in ConePlaneRayFrame frame,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root)
    {
        if (IsReconstruction)
        {
            if (Wants(descriptor))
            {
                point.X.CopyTo(outputPoint.X); point.Y.CopyTo(outputPoint.Y); point.Z.CopyTo(outputPoint.Z);
                point.Denominator.CopyTo(outputPoint.Denominator);
                root.CopyTo(outputRoot); outputRoot[root.Length..].Clear();
                HasPoint = true;
            }
            return;
        }
        if (events.IsEmpty)
            return;
        // A degenerate interval is one family, not one wide value per root.
        // Endpoint=-2 asks its consumer to expand the finite endpoint events.
        ConePlaneRayEvent retained = descriptor;
        if (descriptor.Kind == ConePlaneRayEventKind.Generator
            || descriptor.Kind == ConePlaneRayEventKind.Axis && descriptor.Endpoint >= 0)
            retained = new ConePlaneRayEvent(descriptor.Kind, descriptor.Feature, descriptor.Quadrant,
                descriptor.Branch, -2, descriptor.Orientation);
        for (int i = 0; i < Count; i++)
            if (events[i].Matches(retained))
                return;
        events[Count++] = retained;
    }
}

/// <content>Compact candidate provenance and direct event reconstruction.</content>
internal static partial class ConePlaneRayEvents
{
    private static void SetEvent(FixedTriangle triangle, in ConePlaneRayEvent descriptor,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative)
    {
        positive.CurrentEvent = descriptor; negative.CurrentEvent = descriptor;
        positive.CurrentTriangle = triangle; negative.CurrentTriangle = triangle;
    }

    internal static bool TryMaterializeEvent(FixedTriangle triangle, in ConePlaneRayFrame frame,
        ConePlaneRayEvent descriptor, int orientation, out Vector3d lowerAnchor,
        out Vector3d coneAnchor, out Fixed64 depth)
    {
        lowerAnchor = coneAnchor = default; depth = default;
        if (orientation != 1 && orientation != -1) throw new ArgumentOutOfRangeException(nameof(orientation));
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<ulong> pointRoot = stackalloc ulong[RootWords];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        var sink = new ConePlaneRayEventSink(descriptor, point, pointRoot);
        Accumulate(triangle, frame, ref positive, ref negative, ref sink);
        ConePlaneRaySelection selected = orientation > 0 ? positive : negative;
        if (!sink.HasPoint || !selected.HasValue
            || !ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, pointRoot, out lowerAnchor)
            || !selected.TryGetRoundedMaximumDepth(out depth))
            return false;
        return ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, point, selected.Root,
            ContactQuadratic.At(selected.Values, selected.Signs, 0, Words),
            ContactQuadratic.At(selected.Values, selected.Signs, 1, Words), orientation, out coneAnchor);
    }

    private static bool ReconstructPoint(FixedTriangle triangle, in ConePlaneRayFrame frame,
        ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped Span<ulong> root)
    {
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        var sink = new ConePlaneRayEventSink(descriptor, point, root);
        Accumulate(triangle, frame, ref positive, ref negative, ref sink);
        return sink.HasPoint;
    }

    internal static int CompareEventAnchors(FixedTriangle firstTriangle, ConePlaneRayEvent first,
        FixedTriangle secondTriangle, ConePlaneRayEvent second, in ConePlaneRayFrame frame)
    {
        if (first.Matches(second) && firstTriangle.A == secondTriangle.A
            && firstTriangle.B == secondTriangle.B && firstTriangle.C == secondTriangle.C)
            return 0;
        Span<ulong> av = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<ulong> bv = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> sa = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<int> sb = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<ulong> ar = stackalloc ulong[RootWords];
        Span<ulong> br = stackalloc ulong[RootWords];
        var a = new ConePlaneRayPoint(av, sa); var b = new ConePlaneRayPoint(bv, sb);
        if (!ReconstructPoint(firstTriangle, frame, first, a, ar)
            || !ReconstructPoint(secondTriangle, frame, second, b, br))
            throw new InvalidOperationException("An admitted finite event could not be reconstructed.");
        // All triangles in this selection share one exact cone frame. Its
        // lexicographic coordinate order is a geometric tie independent of
        // authored winding, traversal order and alternative triangulation.
        for (int axis = 0; axis < 3; axis++)
        {
            int comparison = ContactQuadratic.CompareRatios(axis == 0 ? a.X : axis == 1 ? a.Y : a.Z,
                a.Denominator, ar, axis == 0 ? b.X : axis == 1 ? b.Y : b.Z, b.Denominator, br);
            if (comparison != 0) return comparison;
        }
        return CompareCoincidentProvenance(firstTriangle, first, secondTriangle, second);
    }

    private static int CompareCoincidentProvenance(FixedTriangle firstTriangle, ConePlaneRayEvent first,
        FixedTriangle secondTriangle, ConePlaneRayEvent second)
    {
        Span<Vector3d> a = stackalloc Vector3d[3] { firstTriangle.A, firstTriangle.B, firstTriangle.C };
        Span<Vector3d> b = stackalloc Vector3d[3] { secondTriangle.A, secondTriangle.B, secondTriangle.C };
        SortVertices(a); SortVertices(b);
        for (int i = 0; i < 3; i++)
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
        for (int i = 1; i < 3; i++)
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
