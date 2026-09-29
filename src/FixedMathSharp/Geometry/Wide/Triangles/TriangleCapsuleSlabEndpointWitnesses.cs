//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Joint radial/core rounding at a selected stadium endpoint.</summary>
internal static class TriangleCapsuleSlabEndpointWitnesses
{
    internal static Vector3d GetCore(Vector2d axis, Fixed64 length, int coreSign,
        out FixedPointAnchorTerm3d term)
    {
        Fixed64 signedLength = coreSign > 0 ? -length : length;
        Signed576 denominator = Signed576.ExtendValue(Signed320.ExtendValue(FixedPointAnchorTerm3d.Denominator));
        bool representedX = Fixed64.TryGetSignedRawRatio(Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.X), Signed192.Raw(signedLength))), denominator, out Fixed64 x);
        bool representedZ = Fixed64.TryGetSignedRawRatio(Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(Signed192.Raw(axis.Y), Signed192.Raw(signedLength))), denominator, out Fixed64 z);
        System.Diagnostics.Debug.Assert(representedX && representedZ);
        var core = new Vector3d(x, Fixed64.Zero, z);
        term = CreateTerm(axis, signedLength, core, 5);
        return core;
    }

    internal static FixedPointAnchorTerm3d CreateTerm(Vector2d axis, Fixed64 signedLength, Vector3d core, int integralMask) =>
        FixedPointAnchorTerm3d.CreateCenteredAxisSupport(
            new Vector3d((integralMask & 1) != 0 ? axis.X : Fixed64.Zero, Fixed64.Zero,
                (integralMask & 4) != 0 ? axis.Y : Fixed64.Zero), signedLength,
            Vector3d.Zero, Fixed64.Zero,
            new Vector3d((integralMask & 1) != 0 ? core.X : Fixed64.Zero, Fixed64.Zero,
                (integralMask & 4) != 0 ? core.Z : Fixed64.Zero), Vector3d.Zero);

    internal static Vector3d AdjustAnalytic(ReadOnlySpan<ulong> direction, ReadOnlySpan<int> signs,
        Fixed64 radius, Vector3d core, FixedPointAnchorTerm3d term, Vector3d radial, out int integralMask)
    {
        Span<ulong> norm = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        WideArithmetic.MultiplyMagnitudes(Slot(direction, 0), Slot(direction, 0), norm);
        WideArithmetic.MultiplyMagnitudes(Slot(direction, 2), Slot(direction, 2), product);
        WideArithmetic.AddMagnitudeInto(product, norm);
        Fixed64 x = AdjustAnalyticComponent(Slot(direction, 0), -signs[0], norm, radius,
            core.X, term.X, radial.X, out bool integralX);
        Fixed64 z = AdjustAnalyticComponent(Slot(direction, 2), -signs[2], norm, radius,
            core.Z, term.Z, radial.Z, out bool integralZ);
        integralMask = (integralX ? 1 : 0) | (integralZ ? 4 : 0);
        return new Vector3d(x, radial.Y, z);
    }

    private static Fixed64 AdjustAnalyticComponent(ReadOnlySpan<ulong> component, int sign,
        ReadOnlySpan<ulong> norm, Fixed64 radius, Fixed64 core, long residual, Fixed64 rounded, out bool integral)
    {
        Span<ulong> scaled = stackalloc ulong[Words];
        Span<ulong> factor = stackalloc ulong[Words];
        // Compare radial with T/(2D): sign(2DR*n_i - T*sqrt(K)).
        // D=2Q, direction<630 bits: the rational term stays below730 bits;
        // K<1262 and the existing quadratic sign owner sizes its own squares.
        Signed320 multiplier = WideArithmetic.MultiplySigned192(Signed192.Raw(radius),
            WideArithmetic.AddSigned192(FixedPointAnchorTerm3d.Denominator, FixedPointAnchorTerm3d.Denominator));
        Import(multiplier, factor);
        WideArithmetic.MultiplyMagnitudes(component, factor, scaled);
        integral = CompareAnalytic(scaled, sign, norm, Threshold(rounded, 0, 0)) == 0;
        if (integral)
            return rounded;
        int lower = CompareAnalytic(scaled, sign, norm, Threshold(rounded, -1, residual));
        int upper = CompareAnalytic(scaled, sign, norm, Threshold(rounded, 1, residual));
        return Correct(rounded, core, lower, upper);
    }

    private static int CompareAnalytic(ReadOnlySpan<ulong> scaled, int sign, ReadOnlySpan<ulong> norm, Signed192 threshold)
    {
        Span<ulong> magnitude = stackalloc ulong[Words];
        Import(Signed320.ExtendValue(threshold), magnitude);
        return WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(scaled, sign, magnitude, -threshold.Sign, norm);
    }

    internal static Vector3d AdjustRoot(ref FiniteAxisValueRoot root, scoped ReadOnlySpan<ulong> gradient,
        scoped ReadOnlySpan<sbyte> signs, Fixed64 radius, Vector3d core,
        FixedPointAnchorTerm3d term, Vector3d radial, out int integralMask)
    {
        int count = signs.Length / 3, words = gradient.Length / signs.Length;
        int squareCount = 2 * count - 1;
        Span<ulong> norm = stackalloc ulong[squareCount * 2 * words];
        Span<sbyte> normSigns = stackalloc sbyte[squareCount];
        ConvexContactValueRoot.BuildSquaredLength(gradient, signs, norm, normSigns);
        Fixed64 x = AdjustRootComponent(ref root, gradient[..(count * words)], signs[..count], norm, normSigns,
            radius, core.X, term.X, radial.X, out bool integralX);
        Fixed64 z = AdjustRootComponent(ref root, gradient[(2 * count * words)..], signs[(2 * count)..], norm, normSigns,
            radius, core.Z, term.Z, radial.Z, out bool integralZ);
        integralMask = (integralX ? 1 : 0) | (integralZ ? 4 : 0);
        return new Vector3d(x, radial.Y, z);
    }

    private static Fixed64 AdjustRootComponent(ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> component, scoped ReadOnlySpan<sbyte> signs,
        scoped ReadOnlySpan<ulong> norm, scoped ReadOnlySpan<sbyte> normSigns,
        Fixed64 radius, Fixed64 core, long residual, Fixed64 rounded, out bool integral)
    {
        int sign = -WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, component, signs);
        int count = normSigns.Length, words = norm.Length / count + 4;
        Span<ulong> squared = stackalloc ulong[count * words];
        Span<sbyte> squaredSigns = stackalloc sbyte[count];
        Span<ulong> product = stackalloc ulong[words];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(component, signs, component, signs, squared, squaredSigns, product);
        Signed320 multiplier = WideArithmetic.MultiplySigned192(Signed192.Raw(radius),
            WideArithmetic.AddSigned192(FixedPointAnchorTerm3d.Denominator, FixedPointAnchorTerm3d.Denominator));
        Span<ulong> factor = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(WideArithmetic.MultiplySigned320(multiplier, multiplier), factor);
        for (int index = 0; index < count; index++)
        {
            Span<ulong> coefficient = squared.Slice(index * words, words);
            WideArithmetic.MultiplyMagnitudes(coefficient, factor, product);
            product.CopyTo(coefficient);
        }
        // Sign-gated squaring compares (2DR)^2*C(t)^2 with T^2*K(t).
        // T<98 bits and the multiplier<98 bits: four additional coefficient
        // words cover both products and subtraction, independent of root width.
        integral = CompareRoot(ref root, squared, squaredSigns, norm, normSigns, sign, Threshold(rounded, 0, 0)) == 0;
        if (integral)
            return rounded;
        int lower = CompareRoot(ref root, squared, squaredSigns, norm, normSigns, sign, Threshold(rounded, -1, residual));
        int upper = CompareRoot(ref root, squared, squaredSigns, norm, normSigns, sign, Threshold(rounded, 1, residual));
        return Correct(rounded, core, lower, upper);
    }

    private static int CompareRoot(ref FiniteAxisValueRoot root, scoped ReadOnlySpan<ulong> squared,
        scoped ReadOnlySpan<sbyte> squaredSigns, scoped ReadOnlySpan<ulong> norm,
        scoped ReadOnlySpan<sbyte> normSigns, int sign, Signed192 threshold)
    {
        if (sign != threshold.Sign)
            return sign.CompareTo(threshold.Sign);
        if (sign == 0)
            return 0;
        int count = normSigns.Length, words = squared.Length / count, normWords = norm.Length / count;
        Span<ulong> query = stackalloc ulong[squared.Length];
        Span<sbyte> querySigns = stackalloc sbyte[count];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> factor = stackalloc ulong[5];
        Signed320 square = WideArithmetic.MultiplySigned192(threshold, threshold);
        WideArithmetic.GetMagnitude(square, out factor[4], out factor[3], out factor[2], out factor[1], out factor[0]);
        squared.CopyTo(query); squaredSigns.CopyTo(querySigns);
        for (int index = 0; index < count; index++)
        {
            WideArithmetic.MultiplyMagnitudes(norm.Slice(index * normWords, normWords), factor, product);
            int coefficientSign = querySigns[index];
            Add(product, -normSigns[index], query.Slice(index * words, words), ref coefficientSign);
            querySigns[index] = (sbyte)coefficientSign;
        }
        return sign * WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
    }

    private static Signed192 Threshold(Fixed64 rounded, int halfStep, long residual)
    {
        Signed192 twice = WideArithmetic.AddSigned192(Signed192.Raw(rounded), Signed192.Raw(rounded));
        twice = WideArithmetic.AddSigned192(twice, Signed192.Signed(halfStep));
        Signed192 numerator = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(twice, FixedPointAnchorTerm3d.Denominator));
        return WideArithmetic.SubtractSigned192(numerator,
            WideArithmetic.AddSigned192(Signed192.Signed(residual), Signed192.Signed(residual)));
    }

    private static Fixed64 Correct(Fixed64 rounded, Fixed64 core, int lower, int upper)
    {
        // Core residual is within half a raw unit, so only adjacent rounding
        // cells are possible. Parity belongs to the total, not either addend.
        // Nonintegral |radial|<R<=Max keeps the corrected displacement scalar
        // even when the additive endpoint+radial coordinate exceeds Max.
        bool odd = ((rounded.m_rawValue ^ core.m_rawValue) & 1L) != 0;
        int adjustment = lower < 0 || lower == 0 && odd ? -1 : upper > 0 || upper == 0 && odd ? 1 : 0;
        return Fixed64.FromRaw(rounded.m_rawValue + adjustment);
    }
}
