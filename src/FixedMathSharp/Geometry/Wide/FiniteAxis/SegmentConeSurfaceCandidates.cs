//=======================================================================
// SegmentConeSurfaceCandidates.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact finite-segment witnesses paired with finite-cone support features.</summary>
/// <content>Finite support strata, exact halfspace-clipped families, retained certificates, and caller-owned selection.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    // Four quartic charts contribute at most 16 roots, with two horizontal
    // principal feet, four endpoint rim branches, six side branches, three
    // apex feet and one base descriptor. Other strata replace these charts.
    internal const int MaximumCandidates = 32;
    internal static bool AccumulateSegmentConeSurfaceCandidates(FixedSegment segment,
        Vector3d origin, FixedQuaternion rotation, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        scoped ref ConeSurfaceSelection selection)
    {
        var input = new SegmentConeSurfaceInput(segment, origin, rotation, coneCenter, coneRotation, height, radius);
        var geometry = new SegmentConeSurfaceGeometry(input);
        if (!geometry.Finite.IntersectsTransformedSegment(geometry.Finite.Transform(segment.Start),
                geometry.Finite.Transform(segment.End)))
            return false;
        int first = selection.Count;
        AccumulateApex(geometry, ref selection);
        AccumulateBase(geometry, ref selection);
        AccumulateSide(geometry, ref selection);
        AccumulateAxis(geometry, ref selection);
        AccumulateRimPoints(geometry, ref selection);
        AccumulateMeridionalRim(geometry, ref selection);
        AccumulateRimRoots(geometry, ref selection);
        return selection.Count != first;
    }
}

internal enum ConeSurfaceFeature { Apex, Base, Side, Rim }
internal enum ConeSurfaceFamily { None, SegmentInterval, RotationCircle, NormalCone }
internal enum ConeSurfacePointLocation { SegmentIntervalUnspecified, Start, Interior, End }

/// <summary>One exact chart branch or a retained degenerate family.</summary>
internal readonly struct SegmentConeSurfaceCandidate
{
    internal int GetMaximumFamilyEventCount(int halfspaceCount) =>
        SegmentConeSurfaceCandidates.GetMaximumFamilyEventCount(this, halfspaceCount);

    internal bool AccumulateFamilyEvents(scoped ReadOnlySpan<WideAxis3> authoredHalfspaces,
        ConeSurfacePointLocation location, scoped ref ConeSurfaceFamilySelection selection) =>
        SegmentConeSurfaceCandidates.AccumulateFamilyEvents(this, authoredHalfspaces, location, ref selection);

    internal ConeSurfaceFeature Feature { get; }
    internal ConeSurfaceFamily Family { get; }
    internal ConeSurfacePointLocation PointLocation { get; }
    /// <summary>Whether a unique retained segment point is strictly interior; interval families have no unique point.</summary>
    internal bool IsSegmentInterior => PointLocation == ConeSurfacePointLocation.Interior;
    internal readonly SegmentConeSurfaceInput Input;
    internal readonly int Chart, RootOrdinal;

    internal SegmentConeSurfaceCandidate(SegmentConeSurfaceInput input, int chart, int rootOrdinal, ConeSurfacePointLocation location)
    {
        Input = input; Chart = chart; RootOrdinal = rootOrdinal;
        Feature = ConeSurfaceFeature.Rim; Family = ConeSurfaceFamily.None; PointLocation = location;
    }

    internal SegmentConeSurfaceCandidate(SegmentConeSurfaceInput input, ConeSurfaceFeature feature, ConeSurfaceFamily family)
    {
        Input = input; Feature = feature; Family = family;
        PointLocation = ConeSurfacePointLocation.SegmentIntervalUnspecified; Chart = RootOrdinal = 0;
    }

    internal SegmentConeSurfaceCandidate(SegmentConeSurfaceInput input, ConeSurfaceFeature feature,
        ConeSurfaceFamily family, int chart, int ordinal, ConeSurfacePointLocation location)
    {
        Input = input; Feature = feature; Family = family; Chart = chart;
        RootOrdinal = ordinal; PointLocation = location;
    }

    internal FixedContactAnchors GetContact() => Family == ConeSurfaceFamily.None
        ? Feature == ConeSurfaceFeature.Apex ? SegmentConeSurfaceCandidates.GetApexContact(this)
            : Feature == ConeSurfaceFeature.Base ? SegmentConeSurfaceCandidates.GetBaseContact(this)
            : Feature == ConeSurfaceFeature.Side ? SegmentConeSurfaceCandidates.GetSideContact(this)
            : Chart >= 0 ? SegmentConeSurfaceCandidates.GetRimRootContact(this)
                : SegmentConeSurfaceCandidates.GetRimPointContact(this)
        : throw new InvalidOperationException("A family requires an admitted exact parameter before materialization.");

    internal bool TryGetNormalDotSign(WideAxis3 authoredCovector, out int sign) =>
        SegmentConeSurfaceCandidates.TryGetNormalDotSign(this, authoredCovector, out sign);

}

/// <summary>Caller-owned retained finite-support branches and degenerate families.</summary>
internal ref struct ConeSurfaceSelection
{
    private readonly Span<SegmentConeSurfaceCandidate> candidates;
    internal int Count { get; private set; }
    internal SegmentConeSurfaceCandidate this[int index] => candidates[..Count][index];

    internal ConeSurfaceSelection(Span<SegmentConeSurfaceCandidate> candidates)
    {
        this.candidates = candidates; Count = 0;
    }

    internal void Add(SegmentConeSurfaceCandidate candidate)
    {
        if (Count == candidates.Length)
            throw new InvalidOperationException("The caller-owned surface candidate storage is full.");
        candidates[Count++] = candidate;
    }
}

/// <summary>Neutral source provenance sufficient to reconstruct one exact chart.</summary>
internal readonly struct SegmentConeSurfaceInput
{
    internal readonly FixedSegment Segment;
    internal readonly Vector3d Origin, Center;
    internal readonly FixedQuaternion Rotation, ConeRotation;
    internal readonly Fixed64 Height, Radius;

    internal SegmentConeSurfaceInput(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        Segment = segment; Origin = origin; Rotation = rotation; Center = center;
        ConeRotation = coneRotation; Height = height; Radius = radius;
    }
}

/// <summary>One exact centered frame shared by the segment's support charts.</summary>
internal readonly struct SegmentConeSurfaceGeometry
{
    internal readonly SegmentConeSurfaceInput Input;
    internal readonly ConeFiniteSectionFrame Finite;
    internal readonly WideAxis3 A, B, Edge, BaseOffset;
    internal readonly Signed320 Radius;
    internal readonly Signed192 RawScale;
    internal readonly int ValueShift;

    internal SegmentConeSurfaceGeometry(SegmentConeSurfaceInput input)
    {
        Input = input;
        Finite = new ConeFiniteSectionFrame(input.Origin, input.Rotation, input.Center,
            input.ConeRotation, input.Height, input.Radius);
        A = Finite.ShapeFrame.Transform(input.Segment.Start);
        B = Finite.ShapeFrame.Transform(input.Segment.End);
        Edge = new WideAxis3(WideArithmetic.SubtractSigned320(B.X, A.X),
            WideArithmetic.SubtractSigned320(B.Y, A.Y), WideArithmetic.SubtractSigned320(B.Z, A.Z));
        BaseOffset = new WideAxis3(A.X, WideArithmetic.AddSigned320(A.Y, Finite.ShapeFrame.Cap), A.Z);
        Radius = WideArithmetic.MultiplySigned192(Finite.ShapeFrame.Radius, Finite.ShapeFrame.Denominator);
        RawScale = WideArithmetic.AddSigned192(Finite.ShapeFrame.Denominator, Finite.ShapeFrame.Denominator);
        Span<Signed320> coordinates = stackalloc Signed320[8]
        {
            A.X, A.Y, A.Z, B.X, B.Y, B.Z, Finite.ShapeFrame.Cap, Radius
        };
        ValueShift = TriangleCircularGeometry.GetValueShift(coordinates, 4);
    }
}
