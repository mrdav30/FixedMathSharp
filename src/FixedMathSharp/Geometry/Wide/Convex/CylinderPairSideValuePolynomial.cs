//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>Exact squared support values on one cylinder's side-normal plane.</summary>
internal static class CylinderPairSideValuePolynomial
{
    internal const int InvariantCount = 8;
    internal const int CoefficientCount = 9;

    /// <summary>
    /// Builds the unscaled squared-gap polynomial. FirstSide means normals
    /// perpendicular to FirstAxis; capSign selects the other cylinder's cap.
    /// The caller retains signed-gap/cap admission and the existing ellipse
    /// parameter root to identify one value. Output has nine equal-width slots.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static int Build(in CylinderPairGeometry geometry, bool firstSide, int capSign,
        Span<ulong> coefficients, Span<sbyte> signs,
        Span<ulong> unshiftedCoefficients = default, Span<sbyte> unshiftedSigns = default)
    {
        int words = coefficients.Length / CoefficientCount;
        Span<ulong> invariants = stackalloc ulong[InvariantCount * words];
        Span<sbyte> invariantSigns = stackalloc sbyte[InvariantCount];
        invariants.Clear();
        WideAxis3 a = firstSide ? geometry.SecondAxis : geometry.FirstAxis;
        WideAxis3 b = firstSide ? geometry.FirstAxis : geometry.SecondAxis;
        WideAxis3 half = firstSide ? geometry.SecondHalf : geometry.FirstHalf;
        WideAxis3 c = new(
            WideArithmetic.SubtractSigned320(capSign > 0 ? half.X : WideArithmetic.Negate(half.X), geometry.CenterDifference.X),
            WideArithmetic.SubtractSigned320(capSign > 0 ? half.Y : WideArithmetic.Negate(half.Y), geometry.CenterDifference.Y),
            WideArithmetic.SubtractSigned320(capSign > 0 ? half.Z : WideArithmetic.Negate(half.Z), geometry.CenterDifference.Z));
        Signed320 ra = firstSide ? geometry.SecondRadius : geometry.FirstRadius;
        Signed320 rb = firstSide ? geometry.FirstRadius : geometry.SecondRadius;
        Write(a.SquaredLength, 0, invariants, invariantSigns, words);
        Write(b.SquaredLength, 1, invariants, invariantSigns, words);
        Write(WideAxis3.Dot(a, b), 2, invariants, invariantSigns, words);
        Write(c.SquaredLength, 3, invariants, invariantSigns, words);
        Write(WideAxis3.Dot(a, c), 4, invariants, invariantSigns, words);
        Write(WideAxis3.Dot(b, c), 5, invariants, invariantSigns, words);
        Write(WideArithmetic.MultiplySigned320(ra, ra), 6, invariants, invariantSigns, words);
        Write(WideArithmetic.MultiplySigned320(rb, rb), 7, invariants, invariantSigns, words);
        return Build(invariants, invariantSigns, words, coefficients, signs, unshiftedCoefficients, unshiftedSigns);
    }

    /// <summary>
    /// Scalar slots: U=a², B=b², C=a.b, q=c², x=a.c, y=b.c, rho=ra²,
    /// tau=rb², from nonzero nonparallel axes and a positive first radius.
    /// Returns degree eight without normalization.
    /// Input/output storage is disjoint; every intermediate must fit words.
    /// </summary>
    /// <remarks>
    /// Geometry gives |a|&lt;2^33, |b|&lt;2^163 (or the reverse), |c|&lt;2^229,
    /// and radii&lt;2^226. Thus H,C²&lt;2^392, q&lt;2^458, rho,tau&lt;2^452.
    /// Weighting coefficient j by 458j gives l,m,v heights 852,853,850.
    /// The five quartic summands have heights 3414,3407,3410,3405,3393;
    /// their sum has height 3417. Substituting z=2^460*t raises this to 3425.
    /// Conjugation has 25 coefficient pairs and binomial absolute sum&lt;=4^8,
    /// so the final scaled octic has height&lt;6871, within 116 words.
    /// Scratch is 56 coefficient slots, with no nested arithmetic allocation.
    /// This construction frame must return before a root arena is allocated.
    /// </remarks>
    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static int Build(ReadOnlySpan<ulong> invariants, ReadOnlySpan<sbyte> invariantSigns,
        int words, Span<ulong> coefficients, Span<sbyte> signs,
        Span<ulong> unshiftedCoefficients = default, Span<sbyte> unshiftedSigns = default)
    {
        unshiftedCoefficients.Clear();
        unshiftedSigns.Clear();
        Span<ulong> scratch = stackalloc ulong[56 * words];
        Span<sbyte> scratchSigns = stackalloc sbyte[56];
        scratch.Clear();
        scratchSigns.Clear();
        invariants.Slice(0, InvariantCount * words).CopyTo(scratch);
        invariantSigns.Slice(0, InvariantCount).CopyTo(scratchSigns);
        Polynomial storage = new(scratch, scratchSigns, words);
        Polynomial u = storage.Slice(0, 1), b = storage.Slice(1, 1), c = storage.Slice(2, 1);
        Polynomial q = storage.Slice(3, 1), x = storage.Slice(4, 1), y = storage.Slice(5, 1);
        Polynomial rho = storage.Slice(6, 1), tau = storage.Slice(7, 1);
        Polynomial h = storage.Slice(8, 1), cSquared = storage.Slice(9, 1), j = storage.Slice(10, 1);
        Polynomial s0 = storage.Slice(11, 1), s1 = storage.Slice(12, 1);
        Polynomial l = storage.Slice(13, 2), m = storage.Slice(15, 2), v = storage.Slice(17, 2);
        Polynomial f = storage.Slice(19, 5);
        Polynomial p0 = storage.Slice(24, 9), p1 = storage.Slice(33, 9);
        Polynomial even = storage.Slice(42, 5), odd = storage.Slice(47, 4);
        Polynomial shifted = storage.Slice(51, 2), radicalSquared = storage.Slice(53, 2);
        Span<ulong> product = scratch.Slice(55 * words, words);

        Multiply(h, u, b, product);
        Multiply(cSquared, c, c, product);
        Add(j, h); Add(j, cSquared);
        // l=H*z+rho*(H+C²)-H*q+U*y².
        Add(l, h, 1, 1);
        Multiply(s0, rho, j, product); Add(l, s0);
        Multiply(s0, h, q, product); Add(l, s0, -1);
        Multiply(s0, y, y, product); Multiply(s1, u, s0, product); Add(l, s1);
        // m=-(H+C²)*z-rho*C²+C²*q+B*x²-2*C*x*y; v=C²*z.
        Add(m, j, -1, 1);
        Multiply(s0, rho, cSquared, product); Add(m, s0, -1);
        Multiply(s0, cSquared, q, product); Add(m, s0);
        Multiply(s0, x, x, product); Multiply(s1, b, s0, product); Add(m, s1);
        Multiply(s0, c, x, product); Multiply(s1, s0, y, product); Add(m, s1, -2);
        Add(v, cSquared, 1, 1);

        // Disc(rho²*v*lambda³+rho*m*lambda²+l*lambda-H)
        // after removing the guaranteed positive rho² factor. The principal
        // ellipse denominator E cancels before construction; no division is needed.
        Multiply(p0, m, m, product); Multiply(p1, l, l, product); Multiply(f, p0, p1, product);
        Multiply(s0, h, rho, product);
        Multiply(p1, p0, m, product); Multiply(p0, s0, p1, product); Add(f, p0, 4);
        Multiply(p0, l, l, product); Multiply(p1, p0, l, product);
        Multiply(p0, v, p1, product); Add(f, p0, -4);
        Multiply(p0, v, m, product); Multiply(p1, p0, l, product);
        Multiply(p0, s0, p1, product); Add(f, p0, -18);
        Multiply(s1, s0, s0, product); Multiply(p0, v, v, product);
        Multiply(p1, s1, p0, product); Add(f, p1, -27);
        if (!unshiftedCoefficients.IsEmpty)
        {
            f.Magnitudes.CopyTo(unshiftedCoefficients);
            f.Signs.CopyTo(unshiftedSigns);
        }

        // F(S+tau+w)=even+w*odd, w²=4*tau*S. Four Horner steps
        // preserve the radius offset, then conjugation gives even²-w²*odd².
        Add(shifted, tau);
        shifted.Magnitudes[words] = 1;
        shifted.Signs[1] = 1;
        Add(radicalSquared, tau, 4, 1);
        Add(even, f.Slice(4, 1));
        for (int index = 3; index >= 0; index--)
        {
            Multiply(p0, shifted, even, product);
            Multiply(p1, radicalSquared, odd, product);
            Add(p0, p1); Add(p0, f.Slice(index, 1));
            Multiply(p1, shifted, odd, product); Add(p1, even);
            Copy(even, p0); Copy(odd, p1);
        }
        Polynomial output = new(coefficients.Slice(0, CoefficientCount * words), signs.Slice(0, CoefficientCount), words);
        Multiply(output, even, even, product);
        Multiply(p0, odd, odd, product); Multiply(p1, radicalSquared, p0, product);
        Add(output, p1, -1);
        // F's leading coefficient is H²*(H-C²)², strictly positive for
        // nonzero nonparallel axes. Conjugation squares that coefficient.
        System.Diagnostics.Debug.Assert(output.Signs[CoefficientCount - 1] > 0);
        return CoefficientCount - 1;
    }

    private static void Multiply(Polynomial result, Polynomial left, Polynomial right, Span<ulong> product) =>
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(left.Magnitudes, left.Signs,
            right.Magnitudes, right.Signs, result.Magnitudes, result.Signs, product);

    private static void Add(Polynomial result, Polynomial source, int multiplier = 1, int offset = 0) =>
        WideFiniteAxisIntersection.AddFiniteAxisPolynomial(source.Magnitudes, source.Signs,
            result.Magnitudes, result.Signs, multiplier, offset);

    private static void Write(Signed576 value, int index, Span<ulong> invariants,
        Span<sbyte> signs, int words)
    {
        WideArithmetic.GetMagnitude(value, invariants.Slice(index * words, 9));
        signs[index] = (sbyte)value.Sign;
    }

    private static void Copy(Polynomial result, Polynomial source)
    {
        source.Magnitudes.Slice(0, result.Magnitudes.Length).CopyTo(result.Magnitudes);
        source.Signs.Slice(0, result.Signs.Length).CopyTo(result.Signs);
    }

    private readonly ref struct Polynomial
    {
        internal readonly Span<ulong> Magnitudes;
        internal readonly Span<sbyte> Signs;
        private readonly int words;
        internal Polynomial(Span<ulong> magnitudes, Span<sbyte> signs, int words)
        {
            Magnitudes = magnitudes;
            Signs = signs;
            this.words = words;
        }
        internal Polynomial Slice(int offset, int count) =>
            new(Magnitudes.Slice(offset * words, count * words), Signs.Slice(offset, count), words);
    }
}
