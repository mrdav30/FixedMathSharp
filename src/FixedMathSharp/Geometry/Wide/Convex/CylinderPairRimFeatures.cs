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
        return ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, gradientSigns, -gapSign * valueSlopeSign);
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

}
