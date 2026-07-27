//=======================================================================
// WideFiniteAxisIntersection.RoundedBox.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides segment-vs-rounded-box intersection tests by computing the first distance
/// at which a spherically expanded query segment reaches an axis-aligned bounding box,
/// using per-axis breakpoints to evaluate closest-feature distance across intervals.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private const int BoxAxisCount = 3;
    private const int MaximumBoxBreakpointCount = BoxAxisCount * 2;

    #region Nested Types

    private readonly struct BoxBreakpoint
    {
        internal readonly RationalBound320 Parameter;
        internal readonly byte Axis;
        internal readonly sbyte StateAfter;

        internal BoxBreakpoint(RationalBound320 parameter, byte axis, sbyte stateAfter)
        {
            Parameter = parameter;
            Axis = axis;
            StateAfter = stateAfter;
        }
    }

    #endregion

    internal static bool TryGetSphericallyExpandedBoxFirstDistance(
        FixedSegment query,
        FixedBoundBox box,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        out Fixed64 distance)
    {
        distance = default;
        if (box.Intersects(new FixedBoundSphere(query.Start, sphericalExpansion)))
            return true;
        if (query.Start == query.End)
            return false;

        // The closest box feature is fixed between coordinate-boundary
        // crossings, so squared distance is one quadratic per interval.
        Span<BoxBreakpoint> breakpoints = stackalloc BoxBreakpoint[MaximumBoxBreakpointCount];
        int breakpointCount = 0;
        Span<sbyte> axisStates = stackalloc sbyte[BoxAxisCount];
        AddAxisBreakpoints(
            query.Start.X, query.End.X, box.Min.X, box.Max.X, 0,
            breakpoints, ref breakpointCount, out axisStates[0]);
        AddAxisBreakpoints(
            query.Start.Y, query.End.Y, box.Min.Y, box.Max.Y, 1,
            breakpoints, ref breakpointCount, out axisStates[1]);
        AddAxisBreakpoints(
            query.Start.Z, query.End.Z, box.Min.Z, box.Max.Z, 2,
            breakpoints, ref breakpointCount, out axisStates[2]);
        SortBreakpoints(breakpoints, breakpointCount);

        Signed320 unit = Scale320;
        RationalBound320 lower = new(default, unit);
        RationalBound320 maximum = new(unit, unit);
        int breakpointIndex = 0;
        while (true)
        {
            RationalBound320 upper = breakpointIndex < breakpointCount
                ? breakpoints[breakpointIndex].Parameter
                : maximum;
            GetBoxDistancePolynomial(
                query,
                box,
                axisStates,
                sphericalExpansion,
                out Signed320 coefficient,
                out Signed320 projection,
                out Signed320 constant);
            if (TrySolveBoundedQuadraticAtDistance(
                    coefficient,
                    projection,
                    constant,
                    lower,
                    upper,
                    segmentLength,
                    out distance,
                    out _))
            {
                return true;
            }

            if (breakpointIndex == breakpointCount)
                return false;

            do
            {
                BoxBreakpoint breakpoint = breakpoints[breakpointIndex++];
                axisStates[breakpoint.Axis] = breakpoint.StateAfter;
            }
            while (breakpointIndex < breakpointCount
                && Compare(breakpoints[breakpointIndex].Parameter, upper) == 0);
            lower = upper;
        }
    }

    internal static bool TryGetSphericallyExpandedProjectedBoxFirstDistance(
        Signed192 startX,
        Signed192 startY,
        Signed192 startZ,
        Signed192 endX,
        Signed192 endY,
        Signed192 endZ,
        Signed192 extentX,
        Signed192 extentY,
        Signed192 extentZ,
        Signed192 projectionDenominator,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        out Fixed64 distance)
    {
        distance = default;
        // A nonnegative Fixed64 radius scaled by a normalized quaternion-basis
        // denominator uses at most 127 signed bits.
        Signed192 radiusNumerator = Signed192.NarrowValue(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(sphericalExpansion.m_rawValue),
                projectionDenominator));
        if (IsWithinProjectedRoundedBox(
                startX,
                startY,
                startZ,
                extentX,
                extentY,
                extentZ,
                radiusNumerator))
        {
            return true;
        }

        if (WideArithmetic.SubtractSigned192(endX, startX).IsZero
            && WideArithmetic.SubtractSigned192(endY, startY).IsZero
            && WideArithmetic.SubtractSigned192(endZ, startZ).IsZero)
            return false;

        Span<BoxBreakpoint> breakpoints = stackalloc BoxBreakpoint[MaximumBoxBreakpointCount];
        int breakpointCount = 0;
        Span<sbyte> axisStates = stackalloc sbyte[BoxAxisCount];
        AddProjectedAxisBreakpoints(
            startX, endX, extentX, 0,
            breakpoints, ref breakpointCount, out axisStates[0]);
        AddProjectedAxisBreakpoints(
            startY, endY, extentY, 1,
            breakpoints, ref breakpointCount, out axisStates[1]);
        AddProjectedAxisBreakpoints(
            startZ, endZ, extentZ, 2,
            breakpoints, ref breakpointCount, out axisStates[2]);
        SortBreakpoints(breakpoints, breakpointCount);

        Signed320 unit = Scale320;
        RationalBound320 lower = new(default, unit);
        RationalBound320 maximum = new(unit, unit);
        int breakpointIndex = 0;
        while (true)
        {
            RationalBound320 upper = breakpointIndex < breakpointCount
                ? breakpoints[breakpointIndex].Parameter
                : maximum;
            GetProjectedBoxDistancePolynomial(
                startX,
                startY,
                startZ,
                endX,
                endY,
                endZ,
                extentX,
                extentY,
                extentZ,
                axisStates,
                radiusNumerator,
                out Signed320 coefficient,
                out Signed320 projection,
                out Signed320 constant);
            if (TrySolveBoundedQuadraticAtDistance(
                    coefficient,
                    projection,
                    constant,
                    lower,
                    upper,
                    segmentLength,
                    out distance,
                    out _))
            {
                return true;
            }

            if (breakpointIndex == breakpointCount)
                return false;

            do
            {
                BoxBreakpoint breakpoint = breakpoints[breakpointIndex++];
                axisStates[breakpoint.Axis] = breakpoint.StateAfter;
            }
            while (breakpointIndex < breakpointCount
                && Compare(breakpoints[breakpointIndex].Parameter, upper) == 0);
            lower = upper;
        }
    }

    private static void AddAxisBreakpoints(
        Fixed64 start,
        Fixed64 end,
        Fixed64 minimum,
        Fixed64 maximum,
        byte axis,
        Span<BoxBreakpoint> breakpoints,
        ref int count,
        out sbyte initialState)
    {
        Signed192 direction = WideArithmetic.Difference(end, start);
        initialState = start < minimum || (start == minimum && direction.Sign < 0)
            ? (sbyte)-1
            : start > maximum || (start == maximum && direction.Sign > 0)
                ? (sbyte)1
                : (sbyte)0;
        if (direction.Sign > 0)
        {
            AddBreakpoint(start, minimum, direction, axis, 0, breakpoints, ref count);
            AddBreakpoint(start, maximum, direction, axis, 1, breakpoints, ref count);
        }
        else if (direction.Sign < 0)
        {
            AddBreakpoint(start, maximum, direction, axis, 0, breakpoints, ref count);
            AddBreakpoint(start, minimum, direction, axis, -1, breakpoints, ref count);
        }
    }

    private static void AddProjectedAxisBreakpoints(
        Signed192 start,
        Signed192 end,
        Signed192 extent,
        byte axis,
        Span<BoxBreakpoint> breakpoints,
        ref int count,
        out sbyte initialState)
    {
        Signed192 minimum = WideArithmetic.SubtractSigned192(default, extent);
        Signed192 direction = WideArithmetic.SubtractSigned192(end, start);
        int minimumComparison = WideArithmetic.SubtractSigned192(start, minimum).Sign;
        int maximumComparison = WideArithmetic.SubtractSigned192(start, extent).Sign;
        initialState = minimumComparison < 0
            || (minimumComparison == 0 && direction.Sign < 0)
                ? (sbyte)-1
                : maximumComparison > 0
                    || (maximumComparison == 0 && direction.Sign > 0)
                        ? (sbyte)1
                        : (sbyte)0;
        if (direction.Sign > 0)
        {
            AddProjectedBreakpoint(
                start, minimum, direction, axis, 0, breakpoints, ref count);
            AddProjectedBreakpoint(
                start, extent, direction, axis, 1, breakpoints, ref count);
        }
        else if (direction.Sign < 0)
        {
            AddProjectedBreakpoint(
                start, extent, direction, axis, 0, breakpoints, ref count);
            AddProjectedBreakpoint(
                start, minimum, direction, axis, -1, breakpoints, ref count);
        }
    }

    private static void AddBreakpoint(
        Fixed64 start,
        Fixed64 boundary,
        Signed192 direction,
        byte axis,
        sbyte stateAfter,
        Span<BoxBreakpoint> breakpoints,
        ref int count)
    {
        RationalBound320 parameter = Normalize(
            Signed320.ExtendValue(WideArithmetic.Difference(boundary, start)),
            Signed320.ExtendValue(direction));
        Signed320 unit = Scale320;
        if (parameter.Numerator.Sign <= 0
            || Compare(parameter, new RationalBound320(unit, unit)) >= 0)
        {
            return;
        }

        breakpoints[count++] = new BoxBreakpoint(parameter, axis, stateAfter);
    }

    private static void AddProjectedBreakpoint(
        Signed192 start,
        Signed192 boundary,
        Signed192 direction,
        byte axis,
        sbyte stateAfter,
        Span<BoxBreakpoint> breakpoints,
        ref int count)
    {
        RationalBound320 parameter = Normalize(
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(boundary, start)),
            Signed320.ExtendValue(direction));
        Signed320 unit = Scale320;
        if (parameter.Numerator.Sign <= 0
            || Compare(parameter, new RationalBound320(unit, unit)) >= 0)
        {
            return;
        }

        breakpoints[count++] = new BoxBreakpoint(parameter, axis, stateAfter);
    }

    private static void SortBreakpoints(Span<BoxBreakpoint> breakpoints, int count)
    {
        for (int index = 1; index < count; index++)
        {
            BoxBreakpoint current = breakpoints[index];
            int insertion = index;
            while (insertion > 0
                && Compare(breakpoints[insertion - 1].Parameter, current.Parameter) > 0)
            {
                breakpoints[insertion] = breakpoints[insertion - 1];
                insertion--;
            }

            breakpoints[insertion] = current;
        }
    }

    private static void GetBoxDistancePolynomial(
        FixedSegment query,
        FixedBoundBox box,
        Span<sbyte> axisStates,
        Fixed64 sphericalExpansion,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        coefficient = default;
        projection = default;
        Signed192 radius = Signed192.Signed(sphericalExpansion.m_rawValue);
        constant = WideArithmetic.SubtractSigned320(
            default,
            WideArithmetic.MultiplySigned192(radius, radius));

        AddBoxAxisPolynomial(
            query.Start.X, query.End.X, box.Min.X, box.Max.X, axisStates[0],
            ref coefficient, ref projection, ref constant);
        AddBoxAxisPolynomial(
            query.Start.Y, query.End.Y, box.Min.Y, box.Max.Y, axisStates[1],
            ref coefficient, ref projection, ref constant);
        AddBoxAxisPolynomial(
            query.Start.Z, query.End.Z, box.Min.Z, box.Max.Z, axisStates[2],
            ref coefficient, ref projection, ref constant);
    }

    private static void GetProjectedBoxDistancePolynomial(
        Signed192 startX,
        Signed192 startY,
        Signed192 startZ,
        Signed192 endX,
        Signed192 endY,
        Signed192 endZ,
        Signed192 extentX,
        Signed192 extentY,
        Signed192 extentZ,
        Span<sbyte> axisStates,
        Signed192 radiusNumerator,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        coefficient = default;
        projection = default;
        constant = WideArithmetic.SubtractSigned320(
            default,
            WideArithmetic.MultiplySigned192(radiusNumerator, radiusNumerator));

        AddProjectedBoxAxisPolynomial(
            startX, endX, extentX, axisStates[0],
            ref coefficient, ref projection, ref constant);
        AddProjectedBoxAxisPolynomial(
            startY, endY, extentY, axisStates[1],
            ref coefficient, ref projection, ref constant);
        AddProjectedBoxAxisPolynomial(
            startZ, endZ, extentZ, axisStates[2],
            ref coefficient, ref projection, ref constant);
    }

    private static void AddBoxAxisPolynomial(
        Fixed64 start,
        Fixed64 end,
        Fixed64 minimum,
        Fixed64 maximum,
        sbyte state,
        ref Signed320 coefficient,
        ref Signed320 projection,
        ref Signed320 constant)
    {
        if (state == 0)
            return;

        Signed192 direction = WideArithmetic.Difference(end, start);
        Signed192 offset = WideArithmetic.Difference(start, state < 0 ? minimum : maximum);
        coefficient = WideArithmetic.AddSigned320(
            coefficient,
            WideArithmetic.MultiplySigned192(direction, direction));
        projection = WideArithmetic.AddSigned320(
            projection,
            WideArithmetic.MultiplySigned192(offset, direction));
        constant = WideArithmetic.AddSigned320(
            constant,
            WideArithmetic.MultiplySigned192(offset, offset));
    }

    private static void AddProjectedBoxAxisPolynomial(
        Signed192 start,
        Signed192 end,
        Signed192 extent,
        sbyte state,
        ref Signed320 coefficient,
        ref Signed320 projection,
        ref Signed320 constant)
    {
        if (state == 0)
            return;

        Signed192 direction = WideArithmetic.SubtractSigned192(end, start);
        Signed192 boundary = state < 0
            ? WideArithmetic.SubtractSigned192(default, extent)
            : extent;
        Signed192 offset = WideArithmetic.SubtractSigned192(start, boundary);
        coefficient = WideArithmetic.AddSigned320(
            coefficient,
            WideArithmetic.MultiplySigned192(direction, direction));
        projection = WideArithmetic.AddSigned320(
            projection,
            WideArithmetic.MultiplySigned192(offset, direction));
        constant = WideArithmetic.AddSigned320(
            constant,
            WideArithmetic.MultiplySigned192(offset, offset));
    }

    private static bool IsWithinProjectedRoundedBox(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Signed192 extentX,
        Signed192 extentY,
        Signed192 extentZ,
        Signed192 radiusNumerator)
    {
        Signed192 offsetX = GetProjectedBoxAxisOffset(x, extentX);
        Signed192 offsetY = GetProjectedBoxAxisOffset(y, extentY);
        Signed192 offsetZ = GetProjectedBoxAxisOffset(z, extentZ);
        Signed320 squaredDistance = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(offsetX, offsetX),
                WideArithmetic.MultiplySigned192(offsetY, offsetY)),
            WideArithmetic.MultiplySigned192(offsetZ, offsetZ));
        Signed320 squaredRadius = WideArithmetic.MultiplySigned192(
            radiusNumerator,
            radiusNumerator);
        return WideArithmetic.SubtractSigned320(
            squaredDistance,
            squaredRadius).Sign <= 0;
    }

    private static Signed192 GetProjectedBoxAxisOffset(
        Signed192 coordinate,
        Signed192 extent)
    {
        Signed192 minimum = WideArithmetic.SubtractSigned192(default, extent);
        if (WideArithmetic.SubtractSigned192(coordinate, minimum).Sign < 0)
            return WideArithmetic.SubtractSigned192(coordinate, minimum);
        if (WideArithmetic.SubtractSigned192(coordinate, extent).Sign > 0)
            return WideArithmetic.SubtractSigned192(coordinate, extent);
        return default;
    }
}
