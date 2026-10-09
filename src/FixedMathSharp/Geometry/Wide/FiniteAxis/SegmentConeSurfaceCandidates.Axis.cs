//=======================================================================
// SegmentConeSurfaceCandidates.Axis.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>The zero-radius cone's finite axis and exact normal domains.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    private static void AccumulateAxis(in SegmentConeSurfaceGeometry geometry, ref ConeSurfaceSelection selection)
    {
        if (!geometry.Radius.IsZero)
            return;
        if (geometry.Edge.X.IsZero && geometry.Edge.Z.IsZero)
        {
            // Accumulation's finite intersection precondition proves that
            // this coaxial interval has at least one admitted point.
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Side,
                ConeSurfaceFamily.SegmentInterval, 0, 0, ConeSurfacePointLocation.SegmentIntervalUnspecified));
            return;
        }
        GetAxisParameter(geometry, out Signed576 numerator, out Signed576 denominator);
        ConeSurfacePointLocation location = numerator.IsZero ? ConeSurfacePointLocation.Start
            : WideArithmetic.SubtractSigned576(denominator, numerator).IsZero ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Interior;
        selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Side,
            ConeSurfaceFamily.NormalCone, 0, 0, location));
    }

    private static bool TryGetAxisContact(SegmentConeSurfaceCandidate candidate, Signed576 nx, Signed576 ny, Signed576 nz,
        out FixedContactAnchors contact)
    {
        contact = default;
        if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.NormalCone)
            return false;
        System.Diagnostics.Debug.Assert(candidate.Input.Radius == Fixed64.Zero);
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        GetAxisParameter(geometry, out Signed576 numerator, out Signed576 denominator);
        return TryMaterializeAxis(geometry, numerator, denominator, nx, ny, nz, out contact);
    }

    private static void GetAxisParameter(in SegmentConeSurfaceGeometry geometry,
        out Signed576 numerator, out Signed576 denominator)
    {
        // A non-coaxial segment already certified to intersect a zero-radius
        // cone has exactly one finite radial-axis crossing. Reuse that proof;
        // its projection parameter does not require a second admission test.
        denominator = CircularRimContactAlgebra.RadialDot(geometry.Edge, geometry.Edge);
        numerator = WideArithmetic.SubtractSigned576(default, CircularRimContactAlgebra.RadialDot(geometry.A, geometry.Edge));
    }

    private static bool TryMaterializeAxis(in SegmentConeSurfaceGeometry geometry, Signed576 numerator,
        Signed576 denominator, Signed576 nx, Signed576 ny, Signed576 nz, out FixedContactAnchors contact)
    {
        contact = default;
        if (nx.IsZero && ny.IsZero && nz.IsZero)
            return false;
        Signed832 y = WeightedCoordinate(geometry.A.Y, geometry.Edge.Y, numerator, denominator);
        Signed832 cap = WideArithmetic.MultiplySigned576ToSigned832(denominator, geometry.Finite.ShapeFrame.Cap);
        bool apex = WideArithmetic.SubtractSigned832(cap, y).IsZero;
        bool bottom = WideArithmetic.AddSigned832(cap, y).IsZero;
        if (apex ? ny.Sign > 0 : bottom ? ny.Sign < 0 : !ny.IsZero)
            return false;
        int tangent = NormalEdgeDot(nx, ny, nz, geometry.Edge);
        bool start = numerator.IsZero, end = WideArithmetic.SubtractSigned576(denominator, numerator).IsZero;
        if (start ? tangent > 0 : end ? tangent < 0 : tangent != 0)
            return false;
        Span<ulong> yMagnitude = stackalloc ulong[13], divisorMagnitude = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(y, yMagnitude);
        Signed832 divisor = WideArithmetic.MultiplySigned576ToSigned832(denominator, Signed320.ExtendValue(geometry.RawScale));
        WideArithmetic.GetMagnitude(divisor, divisorMagnitude);
        bool represented = Fixed64.TryGetSignedRawRatio(yMagnitude, divisorMagnitude, y.Sign < 0, out Fixed64 coneY);
        System.Diagnostics.Debug.Assert(represented);
        Vector3d point = new(RoundSegmentCoordinate(geometry.Input.Segment.Start.X, geometry.Input.Segment.End.X, numerator, denominator),
            RoundSegmentCoordinate(geometry.Input.Segment.Start.Y, geometry.Input.Segment.End.Y, numerator, denominator),
            RoundSegmentCoordinate(geometry.Input.Segment.Start.Z, geometry.Input.Segment.End.Z, numerator, denominator));
        contact = new FixedContactAnchors(new FixedPointAnchor(geometry.Input.Origin, geometry.Input.Rotation, point),
            new FixedPointAnchor(geometry.Input.Center, geometry.Input.ConeRotation, new Vector3d(Fixed64.Zero, coneY, Fixed64.Zero)),
            MaterializeRationalNormal(Signed832.ExtendValue(nx), Signed832.ExtendValue(ny), Signed832.ExtendValue(nz),
                geometry.Input.ConeRotation), Fixed64.Zero, false);
        return true;
    }
}
