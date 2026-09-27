//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Exact admission and materialization of cylinder rim stationary values.
/// Construction scratch is released before any algebraic-root evaluation.
/// </summary>
internal static class CylinderPairRimFeatures
{
    // In the primitive first-rotation frame, weighted coefficient heights
    // are <7372 bits (value), <7376 bits (cap-direction derivative), and
    // <7212 bits (world-direction derivative), after S=2^ValueShift*t.
    internal const int CoefficientWords = 116;
    private const int Count = CylinderPairValuePolynomial.CoefficientCount;
    internal const int AdmissionCoefficientCount = 4 * Count;

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static int BuildValues(in CylinderPairGeometry geometry, int firstSign, int secondSign,
        Span<ulong> coefficients, Span<sbyte> signs)
    {
        Span<ulong> invariants = stackalloc ulong[CylinderPairValuePolynomial.InvariantCount * CoefficientWords];
        Span<sbyte> invariantSigns = stackalloc sbyte[CylinderPairValuePolynomial.InvariantCount];
        Span<ulong> scratch = stackalloc ulong[CylinderPairValuePolynomial.ScratchCoefficientCount * CoefficientWords];
        Span<sbyte> scratchSigns = stackalloc sbyte[CylinderPairValuePolynomial.ScratchCoefficientCount];
        geometry.WriteInvariants(firstSign, secondSign, invariants, invariantSigns, CoefficientWords);
        int degree = CylinderPairValuePolynomial.Build(invariants, invariantSigns, CoefficientWords,
            coefficients, signs, scratch, scratchSigns);
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(coefficients, Count, geometry.ValueShift);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(coefficients, signs);
        return degree;
    }

    /// <summary>
    /// Returns the derivative sign after the caller has excluded repeated roots.
    /// Isolation guarantees nonroot cell endpoints, so a simple crossing has
    /// the opposite sign to its lower endpoint. Exact dyadic roots use F' directly.
    /// </summary>
    internal static int GetSimpleValueSlopeSign(FiniteAxisValueRoot root) =>
        (root.IsRational ? 1 : -1) * WideFiniteAxisIntersection.EvaluateFiniteRootPolynomial(
            root.Coefficients, root.Signs, root.LowerNumerator, root.DenominatorShift,
            root.IsRational ? 1 : 0);

    /// <summary>
    /// Reuses four caller-owned directional polynomials across this cap pair's
    /// simple roots. Reset readyMask to zero whenever geometry or cap signs change.
    /// Query storage has AdmissionCoefficientCount equal-width slots and signs.
    /// </summary>
    internal static bool TryAdmitSimple(in CylinderPairGeometry geometry, int firstSign, int secondSign,
        ref FiniteAxisValueRoot root, int valueSlopeSign, scoped Span<ulong> derivatives,
        scoped Span<sbyte> derivativeSigns, ref int readyMask, out int gapSign)
    {
        // For a simple value root, v=-W_c/(2W_S), p.v=-rho*W_rho/W_S,
        // q.v=-tau*W_tau/W_S. Both radii must choose the same support side.
        int firstRadial = GetDerivativeSign(geometry, firstSign, secondSign, ref root,
            0, derivatives, derivativeSigns, ref readyMask);
        int secondRadial = GetDerivativeSign(geometry, firstSign, secondSign, ref root,
            1, derivatives, derivativeSigns, ref readyMask);
        gapSign = -firstRadial * valueSlopeSign;
        if (firstRadial == 0 || firstRadial != secondRadial)
            return false;
        int firstCap = GetDerivativeSign(geometry, firstSign, secondSign, ref root,
            2, derivatives, derivativeSigns, ref readyMask);
        if (firstRadial * firstSign * firstCap < 0)
            return false;
        int secondCap = GetDerivativeSign(geometry, firstSign, secondSign, ref root,
            3, derivatives, derivativeSigns, ref readyMask);
        return firstRadial * secondSign * secondCap >= 0;
    }

    internal static Vector3d GetSimpleNormal(in CylinderPairGeometry geometry, int firstSign, int secondSign,
        ref FiniteAxisValueRoot root, int valueSlopeSign, int gapSign)
    {
        Span<ulong> gradient = stackalloc ulong[3 * Count * CoefficientWords];
        Span<sbyte> gradientSigns = stackalloc sbyte[3 * Count];
        WideRationalBasis3d basis = geometry.WorldBasis;
        for (int axis = 0; axis < 3; axis++)
        {
            // Rows of the rotation map the first-frame gradient to world
            // coordinates. A shared positive denominator cancels on normalization.
            WideAxis3 direction = axis == 0
                ? Axis(basis.Xx, basis.Yx, basis.Zx)
                : axis == 1 ? Axis(basis.Xy, basis.Yy, basis.Zy) : Axis(basis.Xz, basis.Yz, basis.Zz);
            BuildDerivative(geometry, firstSign, secondSign, 0, 0, direction,
                gradient.Slice(axis * Count * CoefficientWords, Count * CoefficientWords),
                gradientSigns.Slice(axis * Count, Count));
        }
        // All three derivatives retain the same raw polynomial factor. Remove
        // only common content, never an independent scale per component.
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(gradient, gradientSigns);
        return GetNormalizedDirection(ref root, gradient, gradientSigns, -gapSign * valueSlopeSign);
    }

    internal static Vector3d GetNormalizedDirection(ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> gradient, scoped ReadOnlySpan<sbyte> gradientSigns, int orientation)
    {
        int count = gradientSigns.Length / 3;
        int words = gradient.Length / gradientSigns.Length;
        int squareCount = 2 * count - 1;
        Span<ulong> squaredLength = stackalloc ulong[squareCount * 2 * words];
        Span<sbyte> squaredLengthSigns = stackalloc sbyte[squareCount];
        BuildSquaredLength(gradient, gradientSigns, squaredLength, squaredLengthSigns);
        return new Vector3d(
            GetRoundedNormalComponent(ref root, gradient[..(count * words)], gradientSigns[..count],
                squaredLength, squaredLengthSigns, orientation),
            GetRoundedNormalComponent(ref root, gradient.Slice(count * words, count * words),
                gradientSigns.Slice(count, count), squaredLength, squaredLengthSigns, orientation),
            GetRoundedNormalComponent(ref root, gradient[(2 * count * words)..], gradientSigns[(2 * count)..],
                squaredLength, squaredLengthSigns, orientation));
    }

    internal static void GetRoundedDepth(in CylinderPairGeometry geometry, ref FiniteAxisValueRoot root,
        out Fixed64 depth, out bool clamped)
    {
        Span<ulong> scale = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(geometry.RawScale, out scale[2], out scale[1], out scale[0]);
        Span<ulong> squaredScale = stackalloc ulong[6];
        WideArithmetic.MultiplyMagnitudes(scale, scale, squaredScale);
        int maximum = CompareDepthToTwiceRaw(root, squaredScale, geometry.ValueShift,
            unchecked((ulong)long.MaxValue << 1));
        if (maximum >= 0)
        {
            depth = Fixed64.MaxValue;
            clamped = maximum > 0;
            return;
        }
        // All rounding thresholds lie on this dyadic grid. Once the exact
        // selected root occupies one grid cell, none can be strictly inside
        // it: subsequent comparisons reuse the cell without rebuilding Sturm
        // chains. A root on a grid point is retained as an exact singleton.
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, geometry.ValueShift + 2);
        ulong lower = 0;
        ulong upper = 1UL << 63;
        while (lower < upper)
        {
            ulong midpoint = lower + ((upper - lower) >> 1);
            if (CompareDepthToTwiceRaw(root, squaredScale, geometry.ValueShift, midpoint << 1) >= 0)
                lower = midpoint + 1;
            else
                upper = midpoint;
        }
        ulong floor = lower - 1;
        int comparison = CompareDepthToTwiceRaw(root, squaredScale, geometry.ValueShift, (floor << 1) | 1);
        depth = Fixed64.FromRaw((long)(floor + (comparison > 0 || (comparison == 0 && (floor & 1) != 0) ? 1UL : 0)));
        clamped = false;
    }

    private static int CompareDepthToTwiceRaw(FiniteAxisValueRoot root,
        ReadOnlySpan<ulong> squaredScale, int valueShift, ulong twiceRaw)
    {
        Fixed64.Multiply64To128(twiceRaw, twiceRaw, out ulong high, out ulong low);
        Span<ulong> squaredRaw = stackalloc ulong[2] { low, high };
        Span<ulong> threshold = stackalloc ulong[8];
        WideArithmetic.MultiplyMagnitudes(squaredScale, squaredRaw, threshold);
        return WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root, threshold, valueShift + 2);
    }

    private static int GetDerivativeSign(in CylinderPairGeometry geometry, int firstSign, int secondSign,
        ref FiniteAxisValueRoot root, int query, scoped Span<ulong> derivatives,
        scoped Span<sbyte> derivativeSigns, ref int readyMask)
    {
        Span<ulong> derivative = derivatives.Slice(query * Count * CoefficientWords, Count * CoefficientWords);
        Span<sbyte> signs = derivativeSigns.Slice(query * Count, Count);
        int bit = 1 << query;
        if ((readyMask & bit) == 0)
        {
            WideAxis3 direction = query == 2 ? geometry.FirstHalf : query == 3 ? geometry.SecondHalf : default;
            BuildDerivative(geometry, firstSign, secondSign, query == 0 ? 1 : 0, query == 1 ? 1 : 0,
                direction, derivative, signs);
            // These independent positive scales preserve signs. World-gradient
            // components still use one common scale in GetSimpleNormal.
            WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(derivative, signs);
            readyMask |= bit;
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, derivative, signs);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildDerivative(in CylinderPairGeometry geometry, int firstSign, int secondSign,
        int rhoDirection, int tauDirection, WideAxis3 direction, Span<ulong> coefficients, Span<sbyte> signs)
    {
        Span<ulong> invariants = stackalloc ulong[CylinderPairValuePolynomial.InvariantCount * CoefficientWords];
        Span<sbyte> invariantSigns = stackalloc sbyte[CylinderPairValuePolynomial.InvariantCount];
        Span<ulong> values = stackalloc ulong[Count * CoefficientWords];
        Span<sbyte> valueSigns = stackalloc sbyte[Count];
        Span<ulong> scratch = stackalloc ulong[CylinderPairValuePolynomial.ScratchCoefficientCount * CoefficientWords];
        Span<sbyte> scratchSigns = stackalloc sbyte[CylinderPairValuePolynomial.ScratchCoefficientCount];
        Span<ulong> derivativeInvariants = stackalloc ulong[CylinderPairValueDerivative.InvariantCount * CoefficientWords];
        Span<sbyte> derivativeInvariantSigns = stackalloc sbyte[CylinderPairValueDerivative.InvariantCount];
        Span<ulong> derivativeScratch = stackalloc ulong[CylinderPairValueDerivative.ScratchCoefficientCount * CoefficientWords];
        Span<sbyte> derivativeScratchSigns = stackalloc sbyte[CylinderPairValueDerivative.ScratchCoefficientCount];
        geometry.WriteInvariants(firstSign, secondSign, invariants, invariantSigns, CoefficientWords);
        derivativeInvariants.Clear();
        derivativeInvariantSigns.Clear();
        derivativeInvariants[0] = (ulong)rhoDirection;
        derivativeInvariantSigns[0] = (sbyte)rhoDirection;
        derivativeInvariants[CoefficientWords] = (ulong)tauDirection;
        derivativeInvariantSigns[1] = (sbyte)tauDirection;
        Signed576 center = WideAxis3.Dot(geometry.GetCapOffset(firstSign, secondSign), direction);
        WriteDerivativeInvariant(WideArithmetic.AddSigned576(center, center), derivativeInvariants, derivativeInvariantSigns, 2);
        WriteDerivativeInvariant(WideAxis3.Dot(geometry.PlaneU, direction), derivativeInvariants, derivativeInvariantSigns, 3);
        WriteDerivativeInvariant(WideAxis3.Dot(geometry.PlaneW, direction), derivativeInvariants, derivativeInvariantSigns, 4);
        WriteDerivativeInvariant(WideAxis3.Dot(geometry.SecondAxis, direction), derivativeInvariants, derivativeInvariantSigns, 5);
        var derivative = new CylinderPairValueDerivative(derivativeInvariants, derivativeInvariantSigns,
            coefficients, signs, derivativeScratch, derivativeScratchSigns);
        CylinderPairValuePolynomial.Build(invariants, invariantSigns, CoefficientWords,
            values, valueSigns, scratch, scratchSigns, derivative);
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(coefficients, Count, geometry.ValueShift);
    }

    private static void WriteDerivativeInvariant(Signed576 value, Span<ulong> values, Span<sbyte> signs, int index)
    {
        WideArithmetic.GetMagnitude(value, values.Slice(index * CoefficientWords, 9));
        signs[index] = (sbyte)value.Sign;
    }

    private static WideAxis3 Axis(Signed192 x, Signed192 y, Signed192 z) => new(
        Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z));

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildSquaredLength(ReadOnlySpan<ulong> gradient, ReadOnlySpan<sbyte> signs,
        Span<ulong> squaredLength, Span<sbyte> squaredLengthSigns)
    {
        int count = signs.Length / 3;
        int words = gradient.Length / signs.Length;
        Span<ulong> product = stackalloc ulong[2 * words];
        for (int axis = 0; axis < 3; axis++)
        {
            ReadOnlySpan<ulong> component = gradient.Slice(axis * count * words, count * words);
            ReadOnlySpan<sbyte> componentSigns = signs.Slice(axis * count, count);
            WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(component, componentSigns,
                component, componentSigns, squaredLength, squaredLengthSigns, product, axis != 0);
        }
    }

    private static Fixed64 GetRoundedNormalComponent(ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> component, scoped ReadOnlySpan<sbyte> componentSigns,
        scoped ReadOnlySpan<ulong> squaredLength, scoped ReadOnlySpan<sbyte> squaredLengthSigns, int orientation)
    {
        int sign = orientation * WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, component, componentSigns);
        if (sign == 0)
            return Fixed64.Zero;
        int squareCount = squaredLengthSigns.Length;
        int queryWords = squaredLength.Length / squareCount + 2;
        Span<ulong> query = stackalloc ulong[squareCount * queryWords];
        Span<sbyte> querySigns = stackalloc sbyte[squareCount];
        ulong lower = 0;
        ulong upper = (1UL << 32) + 1;
        ulong previousTwiceRaw = 0;
        int removedShift = -1;
        while (lower < upper)
        {
            ulong midpoint = lower + ((upper - lower) >> 1);
            BuildNormalThreshold(component, componentSigns, squaredLength, squaredLengthSigns,
                midpoint << 1, ref previousTwiceRaw, ref removedShift, query, querySigns);
            if (WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns) >= 0)
                lower = midpoint + 1;
            else
                upper = midpoint;
        }
        ulong floor = lower - 1;
        BuildNormalThreshold(component, componentSigns, squaredLength, squaredLengthSigns,
            (floor << 1) | 1, ref previousTwiceRaw, ref removedShift, query, querySigns);
        int comparison = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
        ulong rounded = floor + (comparison > 0 || (comparison == 0 && (floor & 1) != 0) ? 1UL : 0);
        return Fixed64.FromRaw(sign * (long)rounded);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildNormalThreshold(ReadOnlySpan<ulong> component, ReadOnlySpan<sbyte> componentSigns,
        ReadOnlySpan<ulong> squaredLength, ReadOnlySpan<sbyte> squaredLengthSigns,
        ulong twiceRaw, ref ulong previousTwiceRaw, ref int removedShift,
        Span<ulong> query, Span<sbyte> querySigns)
    {
        int squareCount = squaredLengthSigns.Length;
        int squareWords = squaredLength.Length / squareCount;
        int queryWords = query.Length / querySigns.Length;
        Span<ulong> product = stackalloc ulong[queryWords];
        if (removedShift < 0)
        {
            WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(component, componentSigns,
                component, componentSigns, query, querySigns, product);
            removedShift = 66;
        }
        // Q_k=2^66*C²-k²*L. Restore the previous normalization and use
        // Q_new=Q_old+(old²-new²)*L, retaining C² without a separate buffer.
        // Both k values are <=2^33+1, so the signed difference needs at most
        // 67 magnitude bits. The raw polynomial has exactly the old height
        // bound; the existing two extra coefficient words remain sufficient.
        Fixed64.Multiply64To128(previousTwiceRaw, previousTwiceRaw, out ulong previousHigh, out ulong previousLow);
        Fixed64.Multiply64To128(twiceRaw, twiceRaw, out ulong high, out ulong low);
        Signed192 difference = WideArithmetic.SubtractSigned192(
            new Signed192(0, previousHigh, previousLow), new Signed192(0, high, low));
        Span<ulong> threshold = stackalloc ulong[2];
        WideArithmetic.GetMagnitude(difference, out _, out threshold[1], out threshold[0]);
        for (int index = 0; index < squareCount; index++)
        {
            Span<ulong> target = query.Slice(index * queryWords, queryWords);
            target.CopyTo(product);
            int shiftedSign = 0;
            target.Clear();
            WideArithmetic.AddShiftedSignedMagnitude(product, querySigns[index], removedShift, target, ref shiftedSign);
            WideArithmetic.MultiplyMagnitudes(squaredLength.Slice(index * squareWords, squareWords), threshold, product);
            WideArithmetic.AddShiftedSignedMagnitude(product, difference.Sign * squaredLengthSigns[index],
                0, target, ref shiftedSign);
            querySigns[index] = (sbyte)shiftedSign;
        }
        previousTwiceRaw = twiceRaw;
        removedShift = WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(query, querySigns);
    }
}
