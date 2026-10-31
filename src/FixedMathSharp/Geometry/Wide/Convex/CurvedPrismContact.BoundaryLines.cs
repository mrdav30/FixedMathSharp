//=======================================================================
// CurvedPrismContact.BoundaryLines.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Certifies actual finite-body intersection at retained native boundary junctions.</content>
internal static partial class CurvedPrismContact
{
    // Horizontal sources retain their original edge parameter. Vertical sources
    // have a singleton tangent cut and retain the original lower/upper heights.
    internal static bool IntersectsBoundaryLine(in CurvedPrismContactGeometry geometry, int edge,
        in RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper)
    {
        GetNativeBoundarySegment(geometry, edge, clip, lower, upper, out var a, out var b, out var coordinateD, out var retained);
        return IntersectsNativeSegment(geometry, a, b, coordinateD, retained);
    }

    /// <summary>Gets doubled centered primitive coordinates in one exact original line chart.</summary>
    internal static void GetNativeBoundarySegment(in CurvedPrismContactGeometry geometry, int edge,
        in RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper,
        out WideAxis3 start, out WideAxis3 end, out Signed320 coordinateD, out RationalSegmentInterval retained)
    {
        Vector2d first = geometry.FootprintVertex(edge), last = geometry.FootprintVertex((edge + 1) % (geometry.VertexCount / 2));
        WideDirection3d a = geometry.GetNativeSourcePoint(new(first.X, lower, first.Y));
        WideDirection3d b = geometry.GetNativeSourcePoint(new(last.X, lower, last.Y));
        Signed576 denominator = Signed576.ExtendValue(geometry.NativeRawScale);
        bool vertical = lower != upper;
        if (vertical)
        {
            var c = geometry.GetNativeSourcePoint(new(first.X, upper, first.Y));
            var d = geometry.GetNativeSourcePoint(new(last.X, upper, last.Y));
            Signed320 n = clip.LowerNumerator, div = clip.LowerDenominator;
            a = InterpolateNativeSource(a, b, n, div); b = InterpolateNativeSource(c, d, n, div);
            denominator = WideArithmetic.MultiplySigned320(div, geometry.NativeRawScale);
        }
        NormalizeNativeSegment(a, b, denominator, out start, out end, out coordinateD);
        retained = vertical ? default : clip;
    }

    // Source feasibility is independent of support direction. Discarding a
    // disjoint real-edge window avoids replaying large witness fields for it.
    internal static bool IntersectsRealEdge(in CurvedPrismContactGeometry geometry, int edge,
        in RationalSegmentInterval clip)
    {
        geometry.Edge(edge, out int first, out int last);
        NormalizeNativeSegment(geometry.Vertex(first), geometry.Vertex(last), Signed576.ExtendValue(geometry.RawScale),
            out var a, out var b, out var coordinateD);
        return IntersectsNativeSegment(geometry, a, b, coordinateD, clip);
    }

    private static void NormalizeNativeSegment(in WideDirection3d a, in WideDirection3d b, Signed576 denominator,
        out WideAxis3 start, out WideAxis3 end, out Signed320 coordinateD)
    {
        Span<Signed576> shared = stackalloc Signed576[7] { a.X, a.Y, a.Z, b.X, b.Y, b.Z, denominator };
        WideArithmetic.ReduceCommonScale(shared);
        coordinateD = Signed320.NarrowValue(shared[6]);
        // Native near-field coordinates need <147 bits, singleton cuts <162.
        // After one common scale, doubled points stay <311 bits. The existing
        // identity frame selects wide quadratics when D exceeds Signed192:
        // cone coefficients <716 bits, cylinder coefficients <624, within832.
        start = DoubledBoundaryPoint(shared[0], shared[1], shared[2]);
        end = DoubledBoundaryPoint(shared[3], shared[4], shared[5]);
    }

    private static bool IntersectsNativeSegment(in CurvedPrismContactGeometry geometry,
        WideAxis3 start, WideAxis3 end, Signed320 coordinateD, in RationalSegmentInterval retained)
    {
        if (!geometry.IsCone)
            return new FiniteSlabSectionFrame(geometry.HeightValue, geometry.RadiusValue, coordinateD)
                .IntersectsTransformedSegment(start, end, retained);
        var cone = new ConeFiniteSectionFrame(geometry.HeightValue, geometry.RadiusValue, coordinateD);
        // The cone owner measures height down from the apex; native P is centered.
        start = new(start.X, WideArithmetic.SubtractSigned320(cone.ShapeFrame.Cap, start.Y), start.Z);
        end = new(end.X, WideArithmetic.SubtractSigned320(cone.ShapeFrame.Cap, end.Y), end.Z);
        return cone.IntersectsTransformedSegment(start, end, retained);
    }

    private static WideDirection3d InterpolateNativeSource(in WideDirection3d a, in WideDirection3d b,
        Signed320 n, Signed320 d)
    {
        Signed320 other = WideArithmetic.SubtractSigned320(d, n);
        return new(Signed576.NarrowValue(WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(a.X, other), WideArithmetic.MultiplySigned576ToSigned832(b.X, n))),
            Signed576.NarrowValue(WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(a.Y, other), WideArithmetic.MultiplySigned576ToSigned832(b.Y, n))),
            Signed576.NarrowValue(WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(a.Z, other), WideArithmetic.MultiplySigned576ToSigned832(b.Z, n))));
    }

    private static WideAxis3 DoubledBoundaryPoint(Signed576 x, Signed576 y, Signed576 z) => new(
        Signed320.NarrowValue(WideArithmetic.AddSigned576(x, x)), Signed320.NarrowValue(WideArithmetic.AddSigned576(y, y)),
        Signed320.NarrowValue(WideArithmetic.AddSigned576(z, z)));
}
