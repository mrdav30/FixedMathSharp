//=======================================================================
// WideFiniteAxisIntersection.RoundedBox.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

internal static partial class WideFiniteAxisIntersection
{
    private const int BoxAxisCount = 3;
    private const int MaximumBoxBreakpointCount = BoxAxisCount * 2;

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

        Signed320 unit = WideArithmetic.ExtendToSigned320(One);
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
        Signed192 direction = GetDifference(end, start);
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
            WideArithmetic.ExtendToSigned320(GetDifference(boundary, start)),
            WideArithmetic.ExtendToSigned320(direction));
        Signed320 unit = WideArithmetic.ExtendToSigned320(One);
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
        Signed192 radius = WideArithmetic.FromSignedRaw(sphericalExpansion.m_rawValue);
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

        Signed192 direction = GetDifference(end, start);
        Signed192 offset = GetDifference(start, state < 0 ? minimum : maximum);
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

    private static Signed192 GetDifference(Fixed64 end, Fixed64 start) =>
        WideArithmetic.SubtractSigned192(
            WideArithmetic.FromSignedRaw(end.m_rawValue),
            WideArithmetic.FromSignedRaw(start.m_rawValue));

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
}
