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

    private static void GetAxisParameter(in SegmentConeSurfaceGeometry geometry,
        out Signed576 numerator, out Signed576 denominator)
    {
        // A non-coaxial segment already certified to intersect a zero-radius
        // cone has exactly one finite radial-axis crossing. Reuse that proof;
        // its projection parameter does not require a second admission test.
        denominator = CircularRimContactAlgebra.RadialDot(geometry.Edge, geometry.Edge);
        numerator = WideArithmetic.SubtractSigned576(default, CircularRimContactAlgebra.RadialDot(geometry.A, geometry.Edge));
    }

}
