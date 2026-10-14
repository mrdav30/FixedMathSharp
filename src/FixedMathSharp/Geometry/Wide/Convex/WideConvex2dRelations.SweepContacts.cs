//=======================================================================
// WideConvex2dRelations.SweepContacts.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact contact-axis recovery after an admitted zero-radius sweep.</content>
internal static partial class WideConvex2dRelations
{
    // The caller excluded initial contact and admitted a later hit, so a strict entry axis exists.
    internal static void GetSweptConvexContact(
        Vector2d sourceOrigin, Fixed64 sourceRotation, ReadOnlySpan<Vector2d> sourceOffsets,
        Vector2d targetOrigin, Fixed64 targetRotation, ReadOnlySpan<Vector2d> targetOffsets,
        Vector2d direction, out Vector2d normal, out Vector2d targetOffset)
    {
        RotationFrame2d sourceFrame = new(sourceRotation), targetFrame = new(targetRotation);
        WideAxis2d best = default;
        Signed320 bestGap = default, bestVelocity = default;
        for (int owner = 0; owner < 2; owner++)
        {
            ReadOnlySpan<Vector2d> offsets = owner == 0 ? sourceOffsets : targetOffsets;
            RotationFrame2d frame = owner == 0 ? sourceFrame : targetFrame;
            Vector2d origin = owner == 0 ? sourceOrigin : targetOrigin;
            int winding = GetPlanarSweepWinding(offsets);
            for (int i = 0; i < offsets.Length; i++)
            {
                GetTransformedEdge(frame, offsets[i], offsets[(i + 1) % offsets.Length],
                    out Signed192 x, out Signed192 y);
                WideAxis2d axis = new(y, WideArithmetic.Negate(x));
                if (axis.IsZero) continue;
                if (winding < 0) axis = -axis;
                Signed320 plane = GetProjection(origin, frame, offsets[i], axis);
                if (owner == 0)
                {
                    GetProjectionRange(targetOrigin, targetFrame, targetOffsets, axis, out Signed320 minimum, out Signed320 maximum);
                    if (winding == 0)
                        KeepSweepContactAxis(axis, plane, plane, minimum, maximum, direction, ref best, ref bestGap, ref bestVelocity);
                    else
                        KeepSweepEntryAxis(-axis, WideArithmetic.SubtractSigned320(minimum, plane), direction,
                            ref best, ref bestGap, ref bestVelocity);
                }
                else
                {
                    GetProjectionRange(sourceOrigin, sourceFrame, sourceOffsets, axis, out Signed320 minimum, out Signed320 maximum);
                    if (winding == 0)
                        KeepSweepContactAxis(axis, minimum, maximum, plane, plane, direction, ref best, ref bestGap, ref bestVelocity);
                    else
                        KeepSweepEntryAxis(axis, WideArithmetic.SubtractSigned320(minimum, plane), direction,
                            ref best, ref bestGap, ref bestVelocity);
                }
                if (winding == 0) break;
            }
        }
        // After admission, motion also covers longitudinal collinear and point
        // contacts. It is redundant for full-dimensional convex boundaries.
        KeepConvexSweepContactAxis(new WideAxis2d(Signed192.Raw(direction.X), Signed192.Raw(direction.Y)),
            sourceOrigin, sourceFrame, sourceOffsets, targetOrigin, targetFrame, targetOffsets, direction,
            ref best, ref bestGap, ref bestVelocity);
        normal = WideNormalization.GetNormalized(best.X, best.Y);
        GetSupportFeature(targetOrigin, targetFrame, targetOffsets, best, true, out Feature target);
        GetSupportFeature(sourceOrigin, sourceFrame, sourceOffsets, -best, true, out Feature source);
        GetWorldPoint(sourceOrigin, sourceFrame, source.Start, Vector2d.Zero, out Signed192 pointX, out Signed192 pointY);
        targetOffset = ProjectSweepSupport(pointX, pointY, 1, targetOrigin, targetFrame, target,
            direction, bestGap, bestVelocity);
    }

    internal static bool TryGetSweptCenteredAxisContact(
        Vector2d center, Vector2d coreAxis, Fixed64 axisLength,
        Vector2d targetOrigin, Fixed64 targetRotation, ReadOnlySpan<Vector2d> targetOffsets,
        Vector2d direction, out Vector2d normal, out Vector2d targetOffset)
    {
        RotationFrame2d frame = new(targetRotation);
        WideAxis2d best = default;
        Signed320 bestGap = default, bestVelocity = default;
        int winding = GetPlanarSweepWinding(targetOffsets);
        for (int i = 0; i < targetOffsets.Length; i++)
        {
            GetTransformedEdge(frame, targetOffsets[i], targetOffsets[(i + 1) % targetOffsets.Length],
                out Signed192 x, out Signed192 y);
            WideAxis2d axis = new(y, WideArithmetic.Negate(x));
            if (axis.IsZero) continue;
            if (winding == 0)
            {
                KeepCoreSweepContactAxis(axis, center, coreAxis, axisLength, targetOrigin, frame, targetOffsets, direction,
                    ref best, ref bestGap, ref bestVelocity);
                break; // All nonzero edges of a collinear boundary share this axis.
            }
            if (winding < 0) axis = -axis;
            GetSweepCoreRange(center, coreAxis, axisLength, axis, out Signed320 minimum, out _);
            Signed320 plane = GetProjection(targetOrigin, frame, targetOffsets[i], axis);
            plane = WideArithmetic.AddSigned320(plane, plane);
            KeepSweepEntryAxis(axis, WideArithmetic.SubtractSigned320(minimum, plane), direction,
                ref best, ref bestGap, ref bestVelocity);
        }
        if (axisLength > Fixed64.Zero)
            KeepCoreSweepContactAxis(new WideAxis2d(Signed192.Raw(coreAxis.Y), WideArithmetic.Negate(Signed192.Raw(coreAxis.X))),
                center, coreAxis, axisLength, targetOrigin, frame, targetOffsets, direction, ref best, ref bestGap, ref bestVelocity);
        KeepCoreSweepContactAxis(new WideAxis2d(Signed192.Raw(direction.X), Signed192.Raw(direction.Y)),
            center, coreAxis, axisLength, targetOrigin, frame, targetOffsets, direction, ref best, ref bestGap, ref bestVelocity);
        normal = default;
        targetOffset = default;
        if (best.IsZero) return false;
        normal = WideNormalization.GetNormalized(best.X, best.Y);
        GetSupportFeature(targetOrigin, frame, targetOffsets, best, true, out Feature target);
        Signed320 alignment = WideArithmetic.AddProducts(Signed192.Raw(coreAxis.X), best.X, Signed192.Raw(coreAxis.Y), best.Y);
        Signed192 length = Signed192.Raw(axisLength);
        if (alignment.Sign > 0) length = WideArithmetic.Negate(length);
        if (alignment.IsZero) length = default;
        // Doubled product-scale coordinates retain odd-raw half-axis endpoints.
        Signed192 pointX = WideArithmetic.AddSigned192(
            WideArithmetic.Double(WideArithmetic.Scale(center.X)),
            Signed192.NarrowValue(WideArithmetic.MultiplySigned192(Signed192.Raw(coreAxis.X), length)));
        Signed192 pointY = WideArithmetic.AddSigned192(
            WideArithmetic.Double(WideArithmetic.Scale(center.Y)),
            Signed192.NarrowValue(WideArithmetic.MultiplySigned192(Signed192.Raw(coreAxis.Y), length)));
        targetOffset = ProjectSweepSupport(pointX, pointY, 2, targetOrigin, frame, target,
            direction, bestGap, bestVelocity);
        return true;
    }

    private static void KeepConvexSweepContactAxis(WideAxis2d axis,
        Vector2d sourceOrigin, RotationFrame2d sourceFrame, ReadOnlySpan<Vector2d> sourceOffsets,
        Vector2d targetOrigin, RotationFrame2d targetFrame, ReadOnlySpan<Vector2d> targetOffsets, Vector2d direction,
        ref WideAxis2d best, ref Signed320 bestGap, ref Signed320 bestVelocity)
    {
        GetProjectionRange(sourceOrigin, sourceFrame, sourceOffsets, axis, out Signed320 sourceMin, out Signed320 sourceMax);
        GetProjectionRange(targetOrigin, targetFrame, targetOffsets, axis, out Signed320 targetMin, out Signed320 targetMax);
        KeepSweepContactAxis(axis, sourceMin, sourceMax, targetMin, targetMax, direction, ref best, ref bestGap, ref bestVelocity);
    }

    private static void KeepCoreSweepContactAxis(WideAxis2d axis, Vector2d center, Vector2d coreAxis, Fixed64 length,
        Vector2d targetOrigin, RotationFrame2d frame, ReadOnlySpan<Vector2d> targetOffsets, Vector2d direction,
        ref WideAxis2d best, ref Signed320 bestGap, ref Signed320 bestVelocity)
    {
        GetSweepCoreRange(center, coreAxis, length, axis, out Signed320 sourceMin, out Signed320 sourceMax);
        GetProjectionRange(targetOrigin, frame, targetOffsets, axis, out Signed320 targetMin, out Signed320 targetMax);
        KeepSweepContactAxis(axis, sourceMin, sourceMax,
            WideArithmetic.AddSigned320(targetMin, targetMin), WideArithmetic.AddSigned320(targetMax, targetMax), direction,
            ref best, ref bestGap, ref bestVelocity);
    }

    private static void GetSweepCoreRange(Vector2d center, Vector2d coreAxis, Fixed64 length,
        WideAxis2d axis, out Signed320 minimum, out Signed320 maximum)
    {
        Signed320 midpoint = WideArithmetic.AddProducts(WideArithmetic.Double(WideArithmetic.Scale(center.X)), axis.X,
            WideArithmetic.Double(WideArithmetic.Scale(center.Y)), axis.Y);
        Signed192 alignment = Signed192.NarrowValue(WideArithmetic.AddProducts(
            Signed192.Raw(coreAxis.X), axis.X, Signed192.Raw(coreAxis.Y), axis.Y));
        if (alignment.Sign < 0) alignment = WideArithmetic.Negate(alignment);
        Signed320 extent = WideArithmetic.MultiplySigned192(alignment, Signed192.Raw(length));
        minimum = WideArithmetic.SubtractSigned320(midpoint, extent);
        maximum = WideArithmetic.AddSigned320(midpoint, extent);
    }

    private static void KeepSweepContactAxis(WideAxis2d axis, Signed320 sourceMin, Signed320 sourceMax,
        Signed320 targetMin, Signed320 targetMax, Vector2d direction,
        ref WideAxis2d best, ref Signed320 bestGap, ref Signed320 bestVelocity)
    {
        Signed320 gap = WideArithmetic.SubtractSigned320(sourceMin, targetMax);
        if (gap.Sign <= 0)
        {
            axis = -axis;
            gap = WideArithmetic.SubtractSigned320(targetMin, sourceMax);
        }
        KeepSweepEntryAxis(axis, gap, direction, ref best, ref bestGap, ref bestVelocity);
    }

    private static void KeepSweepEntryAxis(WideAxis2d axis, Signed320 gap, Vector2d direction,
        ref WideAxis2d best, ref Signed320 bestGap, ref Signed320 bestVelocity)
    {
        if (gap.Sign <= 0) return;
        // Admission proves that every strict initial gap closes, so this
        // oriented axis has a strictly positive closing velocity.
        Signed320 velocity = WideArithmetic.SubtractSigned320(default,
            WideArithmetic.AddProducts(axis.X, Signed192.Raw(direction.X), axis.Y, Signed192.Raw(direction.Y)));
        // An admitted translational contact enters at the greatest axis-entry
        // gap/closing-speed ratio. Common distance scales cancel, including
        // authored chords. Product-scale frame coordinates and edge axes use
        // fewer than 100 bits each, so gaps use fewer than 202 bits and motion
        // projections fewer than 165. Their cross-products fit Signed576.
        // No rounded impact point participates in axis selection.
        if (!best.IsZero && WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(gap, bestVelocity),
                WideArithmetic.MultiplySigned320(bestGap, velocity)).Sign <= 0) return;
        best = axis;
        bestGap = gap;
        bestVelocity = velocity;
    }

    private static Vector2d ProjectSweepSupport(Signed192 pointX, Signed192 pointY, int pointScale,
        Vector2d targetOrigin, RotationFrame2d frame, Feature target,
        Vector2d direction, Signed320 gap, Signed320 velocity)
    {
        GetTransformedEdge(frame, target.Start, target.End, out Signed192 edgeX, out Signed192 edgeY);
        Signed320 edgeSquared = WideArithmetic.AddProducts(edgeX, edgeX, edgeY, edgeY);
        if (edgeSquared.IsZero) return target.Start;
        GetWorldPoint(targetOrigin, frame, target.Start, Vector2d.Zero, out Signed192 startX, out Signed192 startY);
        Signed576 denominator = WideArithmetic.MultiplySigned320(edgeSquared, velocity);
        if (pointScale == 2)
        {
            startX = WideArithmetic.Double(startX);
            startY = WideArithmetic.Double(startY);
            denominator = WideArithmetic.AddSigned576(denominator, denominator);
        }
        // Evaluate the support at the exact gap/velocity event, not at its
        // rounded public distance. Relative coordinate numerators use fewer
        // than 268 bits; edge projections fit 370 bits and final coordinates
        // fit 435 bits, within Signed320, Signed576 and Signed832 respectively.
        Signed320 relativeX = Signed320.NarrowValue(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(WideArithmetic.SubtractSigned192(pointX, startX)), velocity),
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(Signed192.Raw(direction.X)), gap)));
        Signed320 relativeY = Signed320.NarrowValue(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(WideArithmetic.SubtractSigned192(pointY, startY)), velocity),
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(Signed192.Raw(direction.Y)), gap)));
        Signed576 numerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(relativeX, Signed320.ExtendValue(edgeX)),
            WideArithmetic.MultiplySigned320(relativeY, Signed320.ExtendValue(edgeY)));
        if (numerator.Sign < 0) numerator = default;
        else if (WideArithmetic.SubtractSigned576(numerator, denominator).Sign > 0) numerator = denominator;
        return new Vector2d(
            GetSweepProjectedCoordinate(target.Start.X, target.End.X, numerator, denominator),
            GetSweepProjectedCoordinate(target.Start.Y, target.End.Y, numerator, denominator));
    }

    private static Fixed64 GetSweepProjectedCoordinate(Fixed64 start, Fixed64 end, Signed576 parameter, Signed576 denominator)
    {
        Signed832 numerator = WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(denominator, Signed320.ExtendValue(Signed192.Raw(start))),
            WideArithmetic.MultiplySigned576ToSigned832(parameter, Signed320.ExtendValue(WideArithmetic.Difference(end, start))));
        // Clamped interpolation between representable local endpoints fits.
        _ = Fixed64.TryGetSignedRawRatio(numerator, Signed832.ExtendValue(denominator), 0, out Fixed64 coordinate);
        return coordinate;
    }
}
