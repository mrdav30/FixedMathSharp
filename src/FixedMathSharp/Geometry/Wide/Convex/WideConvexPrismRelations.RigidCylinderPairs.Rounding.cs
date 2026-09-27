//=======================================================================
// WideConvexPrismRelations.RigidCylinderPairs.Rounding.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Parallel cylinder depth materialization through the shared projection and
/// quadratic-contact rounding authorities.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static void GetRoundedCylinderCylinderDepth(
        in CylinderCylinderPenetration penetration,
        out Fixed64 result,
        out bool isClamped)
    {
        CylinderPairDepth depth = penetration.Depth;
        if (depth.RadiusRaw <= (ulong)long.MaxValue)
        {
            var projection = new ProjectionDepth(
                depth.Rational, depth.Common, Fixed64.FromRaw((long)depth.RadiusRaw),
                depth.RadiusRaw == 0UL ? RadialKind.None : RadialKind.Capsule,
                depth.AxisSquared, Signed320.ExtendValue(Signed192.Signed(1L)), default);
            result = GetRoundedDepth(projection, out isClamped);
            return;
        }

        // Only an unsigned radius sum above Fixed64.MaxValue needs the shared
        // quadratic path. For g=R/(C sqrt(A))+r, its squared raw gap is
        // (R²+(rC)²A+2RrC sqrt(A))/(C²A).
        // The admitted axes give |R|<2^502, C<2^163, A<2^540, r<2^64:
        // gap slots A/B/C/D use <1005/730/540/866 bits respectively.
        // Thus every construction product fits the existing forty-word slots.
        const int words = ConvexContactCandidate.Words;
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        values.Clear();
        signs.Clear();
        GetMagnitude(penetration.Axis.X, values[..words]);
        GetMagnitude(penetration.Axis.Y, values.Slice(words, words));
        GetMagnitude(penetration.Axis.Z, values.Slice(2 * words, words));
        signs[0] = penetration.Axis.X.Sign;
        signs[1] = penetration.Axis.Y.Sign;
        signs[2] = penetration.Axis.Z.Sign;

        Signed320 coefficient = depth.RadialCoefficient;
        Span<ulong> rational = values.Slice(7 * words, words);
        Span<ulong> radical = values.Slice(8 * words, words);
        Span<ulong> radicand = values.Slice(9 * words, words);
        Span<ulong> denominator = values.Slice(10 * words, words);
        Span<ulong> rationalSquared = stackalloc ulong[words];
        BuildProduct(depth.Rational, depth.Rational, rationalSquared);
        BuildProduct(coefficient, coefficient, depth.AxisSquared, rational);
        WideArithmetic.AddEqualMagnitudes(rational, rationalSquared, rational);
        BuildProduct(depth.Rational,
            Signed704.ExtendValue(Signed576.ExtendValue(coefficient)), radical);
        ShiftLeft(radical, 1);
        WideArithmetic.GetMagnitude(depth.AxisSquared, radicand[..9]);
        Signed320 common = Signed320.ExtendValue(depth.Common);
        BuildProduct(common, common, depth.AxisSquared, denominator);
        signs[7] = 1;
        signs[8] = depth.Rational.Sign;
        GetRoundedConvexContactCandidateDepth(
            new ConvexContactCandidate(values, signs, GetCylinderPairDepthSign(depth)),
            Fixed64.Zero, out result, out isClamped);
    }
}
