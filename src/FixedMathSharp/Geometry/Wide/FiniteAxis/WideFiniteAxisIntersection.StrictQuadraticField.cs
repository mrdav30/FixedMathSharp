//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact bounded quadratic-field endpoint and vertex signs for the offset
/// side of a finite cone. This does not isolate or approximate algebraic roots.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private const int StrictFieldWords = 64;
    private static readonly Signed832 StrictOne = Signed832.ExtendValue(Signed192.Signed(1));

    private readonly struct StrictFieldBound
    {
        internal readonly Signed832 Rational;
        internal readonly Signed832 Radical;
        internal readonly Signed832 Denominator;

        internal StrictFieldBound(Signed832 rational, Signed832 radical, Signed832 denominator)
        {
            bool negative = denominator.Sign < 0;
            Rational = negative ? NegateStrict(rational) : rational;
            Radical = negative ? NegateStrict(radical) : radical;
            Denominator = negative ? NegateStrict(denominator) : denominator;
        }
    }

    private static Signed832 NegateStrict(Signed832 value) =>
        WideArithmetic.SubtractSigned832(default, value);

    private static bool GetStrictChordBounds(Signed320 y, Signed320 direction,
        Signed320 minimum, Signed320 maximum, Signed832 shiftCoefficient, Signed192 radicand,
        out StrictFieldBound lower, out StrictFieldBound upper)
    {
        lower = new StrictFieldBound(default, default, StrictOne);
        upper = new StrictFieldBound(StrictOne, default, StrictOne);
        Signed832 k = Signed832.ExtendValue(radicand);
        if (direction.IsZero)
        {
            var point = new StrictFieldBound(Signed832.ExtendValue(Signed576.ExtendValue(y)), default, StrictOne);
            var low = new StrictFieldBound(Signed832.ExtendValue(WideArithmetic.MultiplySigned320(minimum, radicand)),
                shiftCoefficient, k);
            var high = new StrictFieldBound(Signed832.ExtendValue(WideArithmetic.MultiplySigned320(maximum, radicand)),
                shiftCoefficient, k);
            return CompareStrictFieldBounds(point, low, k) > 0
                && CompareStrictFieldBounds(point, high, k) < 0;
        }
        Signed832 denominator = Signed832.ExtendValue(WideArithmetic.MultiplySigned320(direction, radicand));
        var first = new StrictFieldBound(Signed832.ExtendValue(WideArithmetic.MultiplySigned320(
            WideArithmetic.SubtractSigned320(minimum, y), radicand)), shiftCoefficient, denominator);
        var second = new StrictFieldBound(Signed832.ExtendValue(WideArithmetic.MultiplySigned320(
            WideArithmetic.SubtractSigned320(maximum, y), radicand)), shiftCoefficient, denominator);
        StrictFieldBound lowParameter = direction.Sign > 0 ? first : second;
        StrictFieldBound highParameter = direction.Sign > 0 ? second : first;
        if (CompareStrictFieldBounds(lowParameter, lower, k) > 0)
            lower = lowParameter;
        if (CompareStrictFieldBounds(highParameter, upper, k) < 0)
            upper = highParameter;
        return CompareStrictFieldBounds(lower, upper, k) < 0;
    }

    private static bool HasNegativeStrictFieldQuadratic(Signed832 a0, Signed832 a1, Signed832 a2,
        Signed832 b0, Signed832 b1, Signed192 radicand, StrictFieldBound lower, StrictFieldBound upper)
    {
        Signed832 k = Signed832.ExtendValue(radicand);
        if (GetStrictFieldQuadraticSign(a0, a1, a2, b0, b1, k, lower) < 0
            || GetStrictFieldQuadraticSign(a0, a1, a2, b0, b1, k, upper) < 0)
            return true;
        if (a2.Sign <= 0)
            return false;
        var vertex = new StrictFieldBound(NegateStrict(a1), NegateStrict(b1),
            WideArithmetic.AddSigned832(a2, a2));
        return CompareStrictFieldBounds(vertex, lower, k) > 0
            && CompareStrictFieldBounds(vertex, upper, k) < 0
            && GetStrictFieldQuadraticSign(a0, a1, a2, b0, b1, k, vertex) < 0;
    }

    private static int CompareStrictFieldBounds(StrictFieldBound first, StrictFieldBound second, Signed832 k)
    {
        Span<ulong> rational = stackalloc ulong[StrictFieldWords];
        Span<ulong> radical = stackalloc ulong[StrictFieldWords];
        rational.Clear();
        radical.Clear();
        sbyte rationalSign = 0, radicalSign = 0;
        AccumulateStrictProduct(rational, ref rationalSign, first.Rational, second.Denominator, StrictOne, StrictOne);
        AccumulateStrictProduct(rational, ref rationalSign, NegateStrict(second.Rational), first.Denominator, StrictOne, StrictOne);
        AccumulateStrictProduct(radical, ref radicalSign, first.Radical, second.Denominator, StrictOne, StrictOne);
        AccumulateStrictProduct(radical, ref radicalSign, NegateStrict(second.Radical), first.Denominator, StrictOne, StrictOne);
        return GetStrictFieldSign(rational, rationalSign, radical, radicalSign, k);
    }

    private static int GetStrictFieldQuadraticSign(Signed832 a0, Signed832 a1, Signed832 a2,
        Signed832 b0, Signed832 b1, Signed832 k, StrictFieldBound parameter)
    {
        // For t=(n+m√k)/d, multiply by positive d². The rational and
        // radical coefficients below are exact, including cancellation.
        Signed832 n = parameter.Rational, m = parameter.Radical, d = parameter.Denominator;
        Span<ulong> rational = stackalloc ulong[StrictFieldWords];
        Span<ulong> radical = stackalloc ulong[StrictFieldWords];
        rational.Clear();
        radical.Clear();
        sbyte rationalSign = 0, radicalSign = 0;
        AccumulateStrictProduct(rational, ref rationalSign, a2, n, n, StrictOne);
        AccumulateStrictProduct(rational, ref rationalSign, a2, m, m, k);
        AccumulateStrictProduct(rational, ref rationalSign, a1, n, d, StrictOne);
        AccumulateStrictProduct(rational, ref rationalSign, a0, d, d, StrictOne);
        AccumulateStrictProduct(rational, ref rationalSign, b1, m, k, d);
        AccumulateStrictProduct(radical, ref radicalSign, WideArithmetic.AddSigned832(a2, a2), n, m, StrictOne);
        AccumulateStrictProduct(radical, ref radicalSign, a1, m, d, StrictOne);
        AccumulateStrictProduct(radical, ref radicalSign, b1, n, d, StrictOne);
        AccumulateStrictProduct(radical, ref radicalSign, b0, d, d, StrictOne);
        return GetStrictFieldSign(rational, rationalSign, radical, radicalSign, k);
    }

    private static void AccumulateStrictProduct(Span<ulong> total, ref sbyte totalSign,
        Signed832 first, Signed832 second, Signed832 third, Signed832 fourth)
    {
        int sign = first.Sign * second.Sign * third.Sign * fourth.Sign;
        if (sign == 0)
            return;
        Span<Signed832> factors = stackalloc Signed832[4] { first, second, third, fourth };
        Span<ulong> factor = stackalloc ulong[StrictFieldWords];
        Span<ulong> product = stackalloc ulong[StrictFieldWords];
        Span<ulong> next = stackalloc ulong[StrictFieldWords];
        product.Clear();
        product[0] = 1;
        for (int i = 0; i < factors.Length; i++)
        {
            factor.Clear();
            WideArithmetic.GetMagnitude(factors[i], factor[..13]);
            MultiplyRoundedCylinderWide(product, factor, next);
            next.CopyTo(product);
        }
        AddRoundedCylinderSigned(total, totalSign, product, (sbyte)sign, next, out totalSign);
        next.CopyTo(total);
    }

    private static int GetStrictFieldSign(ReadOnlySpan<ulong> rational, sbyte rationalSign,
        ReadOnlySpan<ulong> radical, sbyte radicalSign, Signed832 radicand)
    {
        // All callers supply a positive radicand: one for rational queries,
        // h^2+r^2 for capsules, or the positive generator discriminant for pairs.
        if (rationalSign == 0)
            return radicalSign;
        if (radicalSign == 0 || radicalSign == rationalSign)
            return rationalSign;
        // For the capsule caller, dual-rigid rational coefficients/bound components are
        // below 2^604; their radical coefficients are below 2^540 and
        // k<2^127. Evaluating the cone quadratic needs fewer than 1820 bits;
        // both square comparisons (including k<2^127) fit 4096 bits. The
        // cylinder/cone pair caller proves its separate bound at its call site.
        Span<ulong> first = stackalloc ulong[StrictFieldWords];
        Span<ulong> second = stackalloc ulong[StrictFieldWords];
        Span<ulong> k = stackalloc ulong[StrictFieldWords];
        Span<ulong> product = stackalloc ulong[StrictFieldWords];
        k.Clear();
        WideArithmetic.GetMagnitude(radicand, k[..13]);
        MultiplyRoundedCylinderWide(rational, rational, first);
        MultiplyRoundedCylinderWide(radical, radical, second);
        MultiplyRoundedCylinderWide(second, k, product);
        int comparison = WideArithmetic.CompareMagnitudeEqualLength(first, product);
        return comparison == 0 ? 0 : comparison > 0 ? rationalSign : radicalSign;
    }
}
