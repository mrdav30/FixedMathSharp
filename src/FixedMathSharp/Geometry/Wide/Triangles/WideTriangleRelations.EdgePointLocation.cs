//=======================================================================
// WideTriangleRelations.EdgePointLocation.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Bounded incidence of exact rational edge points across independent rigid frames.</content>
internal static partial class WideTriangleRelations
{
    /// <summary>
    /// Locates A + (B-A)*n/d on a target edge: -1 outside, 0 start,
    /// 1 strict interior, 2 end. A degenerate target has only a start.
    /// Requires normalized rigid rotations and a nonnegative unit parameter.
    /// </summary>
    internal static int LocateEdgePoint(FixedSegment source, Vector3d sourceOrigin, FixedQuaternion sourceRotation,
        Signed320 n, Signed320 d, FixedSegment target, Vector3d targetOrigin, FixedQuaternion targetRotation,
        in RationalSegmentInterval targetInterval = default)
    {
        Span<Signed320> points = stackalloc Signed320[6];
        GetCoveragePoint(source.Start, sourceOrigin, sourceRotation, targetOrigin, targetRotation, points, out Signed320 frameD);
        GetCoveragePoint(source.End, sourceOrigin, sourceRotation, targetOrigin, targetRotation, points[3..], out _);
        Span<Signed320> delta = stackalloc Signed320[3], first = stackalloc Signed320[3], second = stackalloc Signed320[3];
        int axis = -1;
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            delta[coordinate] = WideArithmetic.SubtractSigned320(Signed320.ExtendValue(Signed192.Raw(target.End[coordinate])),
                Signed320.ExtendValue(Signed192.Raw(target.Start[coordinate])));
            if (!delta[coordinate].IsZero) axis = coordinate;
            first[coordinate] = CoverageDifference(points[coordinate], target.Start[coordinate], frameD);
            second[coordinate] = CoverageDifference(points[3 + coordinate], target.Start[coordinate], frameD);
        }
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            int other = (coordinate + 1) % 3;
            Signed576 a = CoverageCross(delta[coordinate], delta[other], first[coordinate], first[other]);
            Signed576 b = CoverageCross(delta[coordinate], delta[other], second[coordinate], second[other]);
            if (EdgePointValue(a, b, n, d).Sign != 0) return -1;
        }
        if (axis < 0)
        {
            for (int coordinate = 0; coordinate < 3; coordinate++)
                if (EdgePointValue(Signed576.ExtendValue(first[coordinate]), Signed576.ExtendValue(second[coordinate]), n, d).Sign != 0) return -1;
            return 0;
        }
        Signed832 startValue = EdgePointValue(Signed576.ExtendValue(first[axis]), Signed576.ExtendValue(second[axis]), n, d);
        Signed832 endValue = EdgePointValue(Signed576.ExtendValue(CoverageDifference(points[axis], target.End[axis], frameD)),
            Signed576.ExtendValue(CoverageDifference(points[3 + axis], target.End[axis], frameD)), n, d);
        int start = startValue.Sign, end = endValue.Sign, direction = delta[axis].Sign;
        if (start != 0 && end != 0 && (start != direction || end != -direction)) return -1;
        if (targetInterval.IsClipped)
        {
            Signed832 edgeD = WideArithmetic.MultiplySigned576ToSigned832(
                WideArithmetic.MultiplySigned320(delta[axis], frameD), Signed576.ExtendValue(d));
            Signed832 edgeN = startValue;
            if (direction < 0)
            { edgeN = WideArithmetic.SubtractSigned832(default, edgeN); edgeD = WideArithmetic.SubtractSigned832(default, edgeD); }
            // Cross-products are compared at full width; two independent
            // 319-bit bounds must never be narrowed into the incidence bank.
            if (WideArithmetic.CompareNonNegativeProducts(edgeN,
                    Signed832.ExtendValue(Signed576.ExtendValue(targetInterval.LowerDenominator)),
                    Signed832.ExtendValue(Signed576.ExtendValue(targetInterval.LowerNumerator)), edgeD) < 0
                || WideArithmetic.CompareNonNegativeProducts(edgeN,
                    Signed832.ExtendValue(Signed576.ExtendValue(targetInterval.UpperDenominator)),
                    Signed832.ExtendValue(Signed576.ExtendValue(targetInterval.UpperNumerator)), edgeD) > 0) return -1;
        }
        return start == 0 ? 0 : end == 0 ? 2 : 1;
    }

    private static Signed832 EdgePointValue(Signed576 first, Signed576 second, Signed320 n, Signed320 d)
    {
        // Unclipped mapped offsets need <201 bits and edge crosses <268.
        // A full 319-bit unit bound therefore keeps interpolation <589 bits.
        Signed832 a = WideArithmetic.MultiplySigned576ToSigned832(first,
            Signed576.ExtendValue(WideArithmetic.SubtractSigned320(d, n)));
        Signed832 b = WideArithmetic.MultiplySigned576ToSigned832(second, Signed576.ExtendValue(n));
        return WideArithmetic.AddSigned832(a, b);
    }
}
