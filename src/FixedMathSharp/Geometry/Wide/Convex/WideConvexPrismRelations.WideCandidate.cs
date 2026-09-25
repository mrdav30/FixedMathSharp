//=======================================================================
// WideConvexPrismRelations.WideCandidate.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Wide-precision candidate axis helpers for cylinder-pair separating-axis
/// tests, using high-word arithmetic to build and evaluate axis projections
/// without precision loss.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static void BuildWideAxisSquared(
        WideCandidateAxis3 axis,
        Span<ulong> result)
    {
        Span<ulong> xSquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> ySquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> zSquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> sum =
            stackalloc ulong[WideCandidateWords];
        WideArithmetic.MultiplyMagnitudes(axis.X, axis.X, xSquared);
        WideArithmetic.MultiplyMagnitudes(axis.Y, axis.Y, ySquared);
        WideArithmetic.MultiplyMagnitudes(axis.Z, axis.Z, zSquared);
        WideArithmetic.AddEqualMagnitudes(xSquared, ySquared, sum);
        WideArithmetic.AddEqualMagnitudes(sum, zSquared, result);
    }

    private static void BuildWideDot(
        WideCandidateAxis3 axis,
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Span<ulong> result,
        out int sign)
    {
        Span<ulong> xMagnitude = stackalloc ulong[3];
        Span<ulong> yMagnitude = stackalloc ulong[3];
        Span<ulong> zMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            x,
            out xMagnitude[2],
            out xMagnitude[1],
            out xMagnitude[0]);
        WideArithmetic.GetMagnitude(
            y,
            out yMagnitude[2],
            out yMagnitude[1],
            out yMagnitude[0]);
        WideArithmetic.GetMagnitude(
            z,
            out zMagnitude[2],
            out zMagnitude[1],
            out zMagnitude[0]);
        Span<ulong> xTerm =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> yTerm =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> zTerm =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> xy =
            stackalloc ulong[WideCandidateWords];
        WideArithmetic.MultiplyMagnitudes(axis.X, xMagnitude, xTerm);
        WideArithmetic.MultiplyMagnitudes(axis.Y, yMagnitude, yTerm);
        WideArithmetic.MultiplyMagnitudes(axis.Z, zMagnitude, zTerm);
        CombineWideSignedMagnitudes(
            xTerm,
            axis.XSign * x.Sign,
            yTerm,
            axis.YSign * y.Sign,
            xy,
            out int xySign);
        CombineWideSignedMagnitudes(
            xy,
            xySign,
            zTerm,
            axis.ZSign * z.Sign,
            result,
            out sign);
    }

    private static void BuildWideScaledAlignment(
        ReadOnlySpan<ulong> alignment,
        Fixed64 length,
        Signed192 otherDenominator,
        Span<ulong> result)
    {
        Span<ulong> lengthMagnitude = stackalloc ulong[3];
        Span<ulong> denominatorMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            Signed192.Raw(length),
            out lengthMagnitude[2],
            out lengthMagnitude[1],
            out lengthMagnitude[0]);
        WideArithmetic.GetMagnitude(
            otherDenominator,
            out denominatorMagnitude[2],
            out denominatorMagnitude[1],
            out denominatorMagnitude[0]);
        Span<ulong> withLength =
            stackalloc ulong[WideCandidateWords];
        WideArithmetic.MultiplyMagnitudes(
            alignment,
            lengthMagnitude,
            withLength);
        WideArithmetic.MultiplyMagnitudes(
            withLength,
            denominatorMagnitude,
            result);
    }

    private static void BuildWideScaledCenter(
        ReadOnlySpan<ulong> projection,
        Signed192 firstDenominator,
        Signed192 secondDenominator,
        Span<ulong> result)
    {
        Span<ulong> firstMagnitude = stackalloc ulong[3];
        Span<ulong> secondMagnitude = stackalloc ulong[3];
        Span<ulong> twiceMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            firstDenominator,
            out firstMagnitude[2],
            out firstMagnitude[1],
            out firstMagnitude[0]);
        WideArithmetic.GetMagnitude(
            secondDenominator,
            out secondMagnitude[2],
            out secondMagnitude[1],
            out secondMagnitude[0]);
        WideArithmetic.GetMagnitude(
            Signed192.Raw(Fixed64.Two),
            out twiceMagnitude[2],
            out twiceMagnitude[1],
            out twiceMagnitude[0]);
        Span<ulong> firstProduct =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> secondProduct =
            stackalloc ulong[WideCandidateWords];
        WideArithmetic.MultiplyMagnitudes(
            projection,
            firstMagnitude,
            firstProduct);
        WideArithmetic.MultiplyMagnitudes(
            firstProduct,
            secondMagnitude,
            secondProduct);
        WideArithmetic.MultiplyMagnitudes(
            secondProduct,
            twiceMagnitude,
            result);
    }

    private static void BuildWidePlaneSquared(
        ReadOnlySpan<ulong> axisSquared,
        Signed320 shapeAxisSquared,
        ReadOnlySpan<ulong> alignment,
        Span<ulong> result)
    {
        Span<ulong> shapeAxisMagnitude = stackalloc ulong[5];
        GetMagnitude(shapeAxisSquared, shapeAxisMagnitude);
        Span<ulong> first =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> second =
            stackalloc ulong[WideCandidateWords];
        WideArithmetic.MultiplyMagnitudes(
            axisSquared,
            shapeAxisMagnitude,
            first);
        WideArithmetic.MultiplyMagnitudes(
            alignment,
            alignment,
            second);
        WideArithmetic.SubtractEqualMagnitudes(first, second, result);
    }

    private static void GetWideRadialCoefficient(
        Signed192 common,
        Fixed64 radius,
        Span<ulong> result)
    {
        Signed320 coefficient =
            WideArithmetic.MultiplySigned192(
                common,
                Signed192.Raw(radius));
        GetMagnitude(coefficient, result);
    }

}
