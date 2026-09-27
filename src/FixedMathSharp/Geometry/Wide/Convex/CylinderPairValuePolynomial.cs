//=======================================================================
// CylinderPairValuePolynomial.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Constructs the exact, unnormalized squared-distance discriminant for two rims.
/// </summary>
internal static class CylinderPairValuePolynomial
{
    internal const int InvariantCount = 11;
    internal const int CoefficientCount = 9;
    internal const int ScratchCoefficientCount = 73;

    /// <summary>
    /// Invariant slots are u², w², b², ra², rb², c², c.u, c.w, b.u,
    /// b.w, b.c. Every scalar has <paramref name="words"/> little-endian
    /// magnitude words and a separate canonical sign. Returns the degree,
    /// or -1 for zero, of three times the cubic-pencil discriminant.
    /// </summary>
    /// <remarks>
    /// The caller supplies disjoint input, output, and scratch storage: eleven,
    /// nine, and seventy-three coefficient slots, respectively. The basis u,w
    /// is orthogonal in the first circle's plane; independently reduced basis
    /// vectors are allowed. Geometry admission and physical S scaling belong
    /// to the caller. No coefficient content or repeated factor is removed.
    ///
    /// Every intermediate must fit words*64 bits. A conservative sufficient
    /// bound for arbitrary input magnitudes below 2^b, b at least one, is
    /// words*64 at least 32*b+128. A caller may instead use a tighter proven
    /// geometry-specific bound, including convolution sum carries. Magnitude
    /// operations do not detect truncation. There is no nested stack scratch;
    /// construction storage need not remain live during root isolation.
    /// An optional direction adds seventy-two scratch slots and includes its
    /// input magnitudes in b. The raw scale is W=3*discriminant for both value
    /// and derivative: allow up to two additional coefficient bits relative
    /// to a bound for the unscaled discriminant. Default direction storage
    /// performs no derivative convolutions.
    /// </remarks>
    internal static int Build(ReadOnlySpan<ulong> invariants,
        ReadOnlySpan<sbyte> invariantSigns, int words,
        Span<ulong> coefficients, Span<sbyte> signs,
        Span<ulong> scratch, Span<sbyte> scratchSigns,
        CylinderPairValueDerivative derivative = default)
    {
        scratch = scratch.Slice(0, ScratchCoefficientCount * words);
        scratchSigns = scratchSigns.Slice(0, ScratchCoefficientCount);
        scratch.Clear();
        scratchSigns.Clear();
        invariants.Slice(0, InvariantCount * words).CopyTo(scratch);
        invariantSigns.Slice(0, InvariantCount).CopyTo(scratchSigns);
        bool differentiate = !derivative.Invariants.IsEmpty;
        if (differentiate)
        {
            derivative.Scratch.Slice(0, CylinderPairValueDerivative.ScratchCoefficientCount * words).Clear();
            derivative.ScratchSigns.Slice(0, CylinderPairValueDerivative.ScratchCoefficientCount).Clear();
            for (int index = 0; index < CylinderPairValueDerivative.InvariantCount; index++)
            {
                int target = index < 5 ? index + 3 : 10;
                derivative.Invariants.Slice(index * words, words)
                    .CopyTo(derivative.Scratch.Slice(target * words, words));
                derivative.ScratchSigns[target] = derivative.InvariantSigns[index];
            }
        }

        ConstructionScratch workspace = new(scratch, scratchSigns, words, derivative);
        Polynomial g = workspace.Slice(0, 1);
        Polynomial h = workspace.Slice(1, 1);
        Polynomial delta = workspace.Slice(2, 1);
        Polynomial rho = workspace.Slice(3, 1);
        Polynomial tau = workspace.Slice(4, 1);
        Polynomial cSquared = workspace.Slice(5, 1);
        Polynomial c0 = workspace.Slice(6, 1);
        Polynomial c1 = workspace.Slice(7, 1);
        Polynomial b0 = workspace.Slice(8, 1);
        Polynomial b1 = workspace.Slice(9, 1);
        Polynomial j = workspace.Slice(10, 1);

        Polynomial e = workspace.Slice(11, 1);
        Polynomial bigK = workspace.Slice(12, 1);
        Polynomial s0 = workspace.Slice(13, 1);
        Polynomial s1 = workspace.Slice(14, 1);
        Polynomial s2 = workspace.Slice(15, 1);
        Polynomial s3 = workspace.Slice(16, 1);
        Polynomial s4 = workspace.Slice(17, 1);
        Polynomial k = workspace.Slice(18, 2);
        Polynomial t0 = workspace.Slice(20, 2);
        Polynomial t1 = workspace.Slice(22, 2);
        Polynomial bigL = workspace.Slice(24, 3);
        Polynomial d = workspace.Slice(27, 3);
        Polynomial a = workspace.Slice(30, 1);
        Polynomial b = workspace.Slice(31, 3);
        Polynomial c = workspace.Slice(34, 3);
        Polynomial constant = workspace.Slice(37, 2);
        Polynomial p = workspace.Slice(39, 5);
        Polynomial r = workspace.Slice(44, 5);
        Polynomial n = workspace.Slice(49, 5);
        Polynomial temporary0 = workspace.Slice(54, 9);
        Polynomial temporary1 = workspace.Slice(63, 9);
        Span<ulong> product = scratch.Slice(72 * words, words);

        // e=(c0*b1-c1*b0)^2.
        Multiply(s0, c0, b1, product);
        Multiply(s1, c1, b0, product);
        Add(s0, s1, -1);
        Multiply(e, s0, s0, product);

        // K=delta*(g*c1^2+h*c0^2)+tau*(g*b1^2+h*b0^2).
        Multiply(s0, c1, c1, product);
        Multiply(s1, g, s0, product);
        Multiply(s0, c0, c0, product);
        Multiply(s2, h, s0, product);
        Add(s1, s2, 1);
        Multiply(s3, delta, s1, product);
        Multiply(s0, b1, b1, product);
        Multiply(s1, g, s0, product);
        Multiply(s0, b0, b0, product);
        Multiply(s2, h, s0, product);
        Add(s1, s2, 1);
        Multiply(s4, tau, s1, product);
        Add(bigK, s3, 1);
        Add(bigK, s4, 1);

        // k=c^2+rho-tau-S.
        Add(k, cSquared, 1);
        Add(k, rho, 1);
        Add(k, tau, -1);
        k.Magnitudes[words] = 1UL;
        k.Signs[1] = -1;

        // L=g*(2*j*c1-k*b1)^2+h*(2*j*c0-k*b0)^2.
        Multiply(s0, j, c1, product);
        Add(t0, s0, 2);
        Multiply(t1, k, b1, product);
        Add(t0, t1, -1);
        Multiply(temporary0, t0, t0, product);
        Multiply(bigL, g, temporary0, product);
        t0.Clear();
        Multiply(s0, j, c0, product);
        Add(t0, s0, 2);
        Multiply(t1, k, b0, product);
        Add(t0, t1, -1);
        Multiply(temporary0, t0, t0, product);
        Multiply(temporary1, h, temporary0, product);
        Add(bigL, temporary1, 1);

        // Tzz=d=delta*(k^2-4*tau*S)+4*tau*j^2.
        Multiply(temporary0, k, k, product);
        Multiply(d, delta, temporary0, product);
        Multiply(s0, delta, tau, product);
        Add(d, s0, -4, 1);
        Multiply(s0, j, j, product);
        Multiply(s1, tau, s0, product);
        Add(d, s1, 4);

        // det(T-lambda*C)=A*lambda^3+B*lambda^2+C1*lambda+D.
        Multiply(s0, g, h, product);
        Multiply(a, s0, rho, product);
        Multiply(b, s0, d, product);
        Multiply(s1, rho, bigK, product);
        Add(b, s1, -4);

        temporary0.Clear();
        Multiply(s0, rho, e, product);
        Add(temporary0, s0, 4);
        Add(temporary0, bigK, 4, 1);
        Add(temporary0, bigL, -1);
        Multiply(s0, delta, tau, product);
        Multiply(temporary1, s0, temporary0, product);
        Add(c, temporary1, 4);

        // The completed-square form makes D linear, not quadratic.
        Multiply(s0, delta, delta, product);
        Multiply(s1, tau, tau, product);
        Multiply(s2, s0, s1, product);
        Multiply(s3, s2, e, product);
        Add(constant, s3, -64, 1);

        // P=B^2-3*A*C1; R=C1^2-3*B*D; N=9*A*D-B*C1.
        Multiply(p, b, b, product);
        Multiply(temporary0, a, c, product);
        Add(p, temporary0, -3);
        Multiply(r, c, c, product);
        Multiply(temporary0, b, constant, product);
        Add(r, temporary0, -3);
        Multiply(temporary0, a, constant, product);
        Add(n, temporary0, 9);
        Multiply(temporary0, b, c, product);
        Add(n, temporary0, -1);

        // Three times the discriminant; preserve this exact envelope scale.
        Multiply(temporary0, p, r, product);
        Multiply(temporary1, n, n, product);
        Polynomial output = new(coefficients.Slice(0, CoefficientCount * words),
            signs.Slice(0, CoefficientCount),
            differentiate ? derivative.Coefficients.Slice(0, CoefficientCount * words) : default,
            differentiate ? derivative.Signs.Slice(0, CoefficientCount) : default);
        output.Clear();
        Add(output, temporary0, 4);
        Add(output, temporary1, -1);
        int degree = CoefficientCount - 1;
        while (degree >= 0 && output.Signs[degree] == 0)
            degree--;
        return degree;
    }

    // These are construction views, not retained roots or a second arithmetic
    // implementation. Scalar products and signed sums use WideArithmetic;
    // every destination in the schedule is disjoint from its sources.
    private readonly ref struct Polynomial
    {
        internal readonly Span<ulong> Magnitudes;
        internal readonly Span<sbyte> Signs;
        internal readonly Span<ulong> DerivativeMagnitudes;
        internal readonly Span<sbyte> DerivativeSigns;

        internal Polynomial(Span<ulong> magnitudes, Span<sbyte> signs,
            Span<ulong> derivativeMagnitudes = default, Span<sbyte> derivativeSigns = default)
        {
            Magnitudes = magnitudes;
            Signs = signs;
            DerivativeMagnitudes = derivativeMagnitudes;
            DerivativeSigns = derivativeSigns;
        }

        internal void Clear()
        {
            Magnitudes.Clear();
            Signs.Clear();
            DerivativeMagnitudes.Clear();
            DerivativeSigns.Clear();
        }
    }

    private readonly ref struct ConstructionScratch
    {
        private readonly Span<ulong> _magnitudes;
        private readonly Span<sbyte> _signs;
        private readonly int _words;
        private readonly CylinderPairValueDerivative _derivative;

        internal ConstructionScratch(Span<ulong> magnitudes, Span<sbyte> signs,
            int words, CylinderPairValueDerivative derivative)
        {
            _magnitudes = magnitudes;
            _signs = signs;
            _words = words;
            _derivative = derivative;
        }

        internal Polynomial Slice(int first, int count) =>
            new(_magnitudes.Slice(first * _words, count * _words),
                _signs.Slice(first, count),
                _derivative.Invariants.IsEmpty ? default :
                    _derivative.Scratch.Slice(first * _words, count * _words),
                _derivative.Invariants.IsEmpty ? default :
                    _derivative.ScratchSigns.Slice(first, count));
    }

    private static void Multiply(Polynomial destination, Polynomial left,
        Polynomial right, Span<ulong> product)
    {
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            left.Magnitudes, left.Signs, right.Magnitudes, right.Signs,
            destination.Magnitudes, destination.Signs, product);
        if (!destination.DerivativeSigns.IsEmpty)
        {
            WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
                left.DerivativeMagnitudes, left.DerivativeSigns, right.Magnitudes, right.Signs,
                destination.DerivativeMagnitudes, destination.DerivativeSigns, product);
            WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
                left.Magnitudes, left.Signs, right.DerivativeMagnitudes, right.DerivativeSigns,
                destination.DerivativeMagnitudes, destination.DerivativeSigns, product, accumulate: true);
        }
    }

    private static void Add(Polynomial destination, Polynomial source,
        int multiplier, int degreeOffset = 0)
    {
        WideFiniteAxisIntersection.AddFiniteAxisPolynomial(
            source.Magnitudes, source.Signs, destination.Magnitudes, destination.Signs,
            multiplier, degreeOffset);
        if (!destination.DerivativeSigns.IsEmpty)
            WideFiniteAxisIntersection.AddFiniteAxisPolynomial(
                source.DerivativeMagnitudes, source.DerivativeSigns,
                destination.DerivativeMagnitudes, destination.DerivativeSigns,
                multiplier, degreeOffset);
    }
}
