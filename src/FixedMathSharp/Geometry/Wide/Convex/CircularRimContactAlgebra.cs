//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Exact circular-rim chart, value and direction algebra on a normal plane.</summary>
internal static class CircularRimContactAlgebra
{
    internal const int ValueWords = 56;

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void BuildParameter(in WideDirection3d offset, Signed576 radius, Signed320 rawScale,
        in WideDirection3d first, in WideDirection3d second, Span<ulong> data, Span<sbyte> signs,
        bool useRigidBounds = false)
    {
        if (useRigidBounds)
        {
            // The rigid triangle owner retains its original bounds even after
            // an endpoint-core shift: offsets/radius <232, chart axes <198,
            // and RawScale <164 bits. Reuse its existing coefficient path;
            // exact scaled source frames must keep the wider products below.
            BuildParameter(GetRigidAxis(offset), Signed320.NarrowValue(radius), Signed192.NarrowProven(rawScale),
                GetRigidAxis(first), GetRigidAxis(second), data, signs);
            return;
        }
        int words = data.Length / signs.Length;
        Span<ulong> inputs = stackalloc ulong[11 * words]; Span<sbyte> inputSigns = stackalloc sbyte[9];
        inputs.Clear(); inputSigns.Clear();
        Span<ulong> p = inputs[..(2 * words)], q = inputs.Slice(2 * words, 3 * words),
            metric = inputs.Slice(5 * words, 3 * words), scale = inputs.Slice(8 * words, words),
            square = inputs.Slice(9 * words, words), product = inputs.Slice(10 * words, words);
        Write(p, inputSigns[..2], 0, WideDirection3d.Dot(offset, first), words);
        Write(p, inputSigns[..2], 1, WideDirection3d.Dot(offset, second), words);
        Write(metric, inputSigns.Slice(5, 3), 0, first.SquaredLength, words);
        Write(metric, inputSigns.Slice(5, 3), 1, Twice(WideDirection3d.Dot(first, second)), words);
        Write(metric, inputSigns.Slice(5, 3), 2, second.SquaredLength, words);
        Write(q, inputSigns.Slice(2, 3), 0, RadialDot(first, first), words);
        Write(q, inputSigns.Slice(2, 3), 1, Twice(RadialDot(first, second)), words);
        Write(q, inputSigns.Slice(2, 3), 2, RadialDot(second, second), words);
        Import(WideArithmetic.MultiplySigned576ToSigned832(radius, radius), square);
        for (int index = 0; index < 3; index++)
        {
            WideArithmetic.MultiplyMagnitudes(q.Slice(index * words, words), square, product);
            product.CopyTo(q.Slice(index * words, words));
        }
        Import(rawScale, scale);
        CylinderEdgeContactPolynomial.Build(p, inputSigns[..2], q, inputSigns.Slice(2, 3),
            metric, inputSigns.Slice(5, 3), scale, data[..(26 * words)], signs[..26]);
    }

    private static WideAxis3 GetRigidAxis(in WideDirection3d value) => new(
        Signed320.NarrowValue(value.X), Signed320.NarrowValue(value.Y), Signed320.NarrowValue(value.Z));

    internal static void GetBasis(in WideDirection3d edge, out WideDirection3d first, out WideDirection3d second)
    {
        if (!edge.X.IsZero)
        {
            first = new WideDirection3d(WideArithmetic.Negate(edge.Y), edge.X, default);
            second = new WideDirection3d(WideArithmetic.Negate(edge.Z), default, edge.X);
        }
        else
        {
            first = new WideDirection3d(default, WideArithmetic.Negate(edge.Z), edge.Y);
            second = new WideDirection3d(edge.Y, default, default);
        }
    }

    internal static WideDirection3d Transform(in WideRationalBasis3d basis, in WideDirection3d value) => value.Transform(basis);
    internal static Signed832 RadialDot(in WideDirection3d first, in WideDirection3d second) => WideArithmetic.AddSigned832(
        WideArithmetic.MultiplySigned576ToSigned832(first.X, second.X),
        WideArithmetic.MultiplySigned576ToSigned832(first.Z, second.Z));
    internal static Signed832 Twice(Signed832 value) => WideArithmetic.AddSigned832(value, value);
    internal static void Write(Span<ulong> values, Span<sbyte> signs, int index, Signed832 value, int words = 0)
    {
        if (words == 0) words = values.Length / signs.Length;
        Span<ulong> target = values.Slice(index * words, words);
        WideArithmetic.GetMagnitude(value, target); signs[index] = (sbyte)value.Sign;
    }

    internal static void WriteGradient(in WideDirection3d first, in WideDirection3d second, Span<ulong> values, Span<sbyte> signs)
    {
        values.Clear();
        int words = values.Length / signs.Length;
        for (int component = 0; component < 3; component++)
        {
            Signed576 a = Component(first, component), b = Component(second, component);
            WideArithmetic.GetMagnitude(a, values.Slice(component * 2 * words, words));
            WideArithmetic.GetMagnitude(b, values.Slice((component * 2 + 1) * words, words));
            signs[component * 2] = (sbyte)a.Sign; signs[component * 2 + 1] = (sbyte)b.Sign;
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void BuildValues(in WideDirection3d edge, in WideDirection3d offset, Signed576 radius, int valueShift,
        Span<ulong> values, Span<sbyte> signs, bool useRigidBounds = false)
    {
        if (useRigidBounds)
        {
            BuildValues(GetRigidAxis(edge), GetRigidAxis(offset), Signed320.NarrowValue(radius), valueShift, values, signs);
            return;
        }
        int words = values.Length / signs.Length;
        Span<ulong> invariants = stackalloc ulong[8 * words]; Span<sbyte> invariantSigns = stackalloc sbyte[8];
        invariants.Clear(); invariantSigns.Clear();
        Write(invariants, invariantSigns, 0, Signed832.ExtendValue(Signed192.Signed(1)), words);
        Write(invariants, invariantSigns, 1, edge.SquaredLength, words);
        Write(invariants, invariantSigns, 2, Signed832.ExtendValue(edge.Y), words);
        Write(invariants, invariantSigns, 3, offset.SquaredLength, words);
        Write(invariants, invariantSigns, 4, Signed832.ExtendValue(offset.Y), words);
        Write(invariants, invariantSigns, 5, WideDirection3d.Dot(edge, offset), words);
        Write(invariants, invariantSigns, 6, WideArithmetic.MultiplySigned576ToSigned832(radius, radius), words);
        CylinderPairSideValuePolynomial.BuildUnshifted(invariants, invariantSigns, words, values, signs);
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(values, 5, valueShift);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values, signs);
    }

    /// <summary>
    /// Tests whether the radial support point projects within a finite segment.
    /// halfAxis must be a positive multiple of axis; radius and Q are positive,
    /// Q=radius²|first_h+t*second_h|². Scratch needs seven disjoint forty-word
    /// slots and must not overlap root or Q. No stationarity is assumed here.
    /// Axis/chart components are below 2^198; offset/halfAxis/radius below 2^232,
    /// following the shared circular chart bounds.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static bool HasSegmentSupport(WideAxis3 axis, WideAxis3 halfAxis, WideAxis3 offset,
        Signed320 radius, WideAxis3 first, WideAxis3 second, scoped ref FiniteAxisValueRoot root,
        ReadOnlySpan<ulong> q, ReadOnlySpan<sbyte> qSigns, Span<ulong> scratch)
    {
        // lambda=a+T/sqrt(Q), H=axis.halfAxis>0. All terms use the same
        // coordinate scale, which cancels from -H<=lambda<=H. Do not use the
        // independently normalized stationary/value polynomials in place of Q.
        Signed576 a = WideAxis3.Dot(axis, offset), h = WideAxis3.Dot(axis, halfAxis);
        Span<ulong> t = scratch[..(2 * Words)], quadratic = scratch.Slice(2 * Words, 3 * Words);
        Span<ulong> square = Slot(scratch, 5), product = Slot(scratch, 6);
        Span<sbyte> tSigns = stackalloc sbyte[2], quadraticSigns = stackalloc sbyte[3];
        Write(t, tSigns, 0, RadialDot(axis, first));
        Write(t, tSigns, 1, RadialDot(axis, second));
        Import(WideArithmetic.MultiplySigned320(radius, radius), square);
        for (int index = 0; index < 2; index++)
        {
            WideArithmetic.MultiplyMagnitudes(Slot(t, index), square, product);
            product.CopyTo(Slot(t, index));
        }
        int tSign = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, t, tSigns);
        for (int bound = 0; bound < 2; bound++)
        {
            Signed576 b = bound == 0 ? WideArithmetic.AddSigned576(a, h) : WideArithmetic.SubtractSigned576(a, h);
            int sign;
            if (tSign == 0) sign = b.Sign;
            else if (b.Sign == 0 || b.Sign == tSign) sign = tSign;
            else
            {
                // Opposite unsquared signs: sign(T+b*sqrt(Q)) is
                // sign(T)*sign(T²-b²Q), including exact equality. Equal signs
                // must be handled first; squaring them would admit false support.
                WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(t, tSigns, t, tSigns,
                    quadratic, quadraticSigns, product);
                Import(b, square);
                WideArithmetic.MultiplyMagnitudes(square, square, product);
                product.CopyTo(square);
                for (int index = 0; index < 3; index++)
                {
                    WideArithmetic.MultiplyMagnitudes(Slot(q, index), square, product);
                    int coefficientSign = quadraticSigns[index];
                    Add(product, -qSigns[index], Slot(quadratic, index), ref coefficientSign);
                    quadraticSigns[index] = (sbyte)coefficientSign;
                }
                sign = tSign * WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, quadratic, quadraticSigns);
            }
            if (bound == 0 ? sign < 0 : sign > 0) return false;
        }
        // a,H<2^432; T<2^861; comparison coefficients<2^1723 under the
        // shared chart bounds. Forty-word arithmetic needs no wider owner.
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void BuildParameter(in WideAxis3 offset, Signed320 radius, Signed192 rawScale,
        in WideAxis3 first, in WideAxis3 second, Span<ulong> data, Span<sbyte> signs)
    {
        // Shifted core-region coordinates are <2^232 but chart directions
        // retain original edges (<2^198), never scaled vertex differences.
        // M<2^398,P<2^432,Q<2^852,K<2^832,L<2^1252,W<2^2520,
        // N<2^2534,D<2^2396. Forty words give 2560 bits for either region.
        Span<ulong> inputs = stackalloc ulong[11 * Words];
        Span<sbyte> inputSigns = stackalloc sbyte[9];
        inputs.Clear(); inputSigns.Clear();
        Span<ulong> p = inputs[..(2 * Words)], q = inputs.Slice(2 * Words, 3 * Words);
        Span<ulong> metric = inputs.Slice(5 * Words, 3 * Words), scale = Slot(inputs, 8);
        Span<ulong> radiusSquared = Slot(inputs, 9), product = Slot(inputs, 10);
        Write(p, inputSigns[..2], 0, WideAxis3.Dot(offset, first));
        Write(p, inputSigns[..2], 1, WideAxis3.Dot(offset, second));
        Write(metric, inputSigns.Slice(5, 3), 0, first.SquaredLength);
        Write(metric, inputSigns.Slice(5, 3), 1, Twice(WideAxis3.Dot(first, second)));
        Write(metric, inputSigns.Slice(5, 3), 2, second.SquaredLength);
        Write(q, inputSigns.Slice(2, 3), 0, RadialDot(first, first));
        Write(q, inputSigns.Slice(2, 3), 1, Twice(RadialDot(first, second)));
        Write(q, inputSigns.Slice(2, 3), 2, RadialDot(second, second));
        Import(WideArithmetic.MultiplySigned320(radius, radius), radiusSquared);
        for (int index = 0; index < 3; index++)
        {
            WideArithmetic.MultiplyMagnitudes(Slot(q, index), radiusSquared, product);
            product.CopyTo(Slot(q, index));
        }
        Import(Signed320.ExtendValue(rawScale), scale);
        CylinderEdgeContactPolynomial.Build(p, inputSigns[..2], q, inputSigns.Slice(2, 3),
            metric, inputSigns.Slice(5, 3), scale, data[..(26 * Words)], signs[..26]);

    }

    /// <summary>
    /// Reuses a built chart for the swapped basis (second+s*first). Only the
    /// 26 shared coefficient slots are changed; caller admission slots remain intact.
    /// </summary>
    internal static void ReverseParameterChart(Span<ulong> data, Span<sbyte> signs)
    {
        // t=1/s gives Pbar=s*P(1/s), Qbar=s²*Q(1/s), Mbar=s²*M(1/s).
        // Kbar=-s*K(1/s), Lbar=-s²*L(1/s), Jbar=-s²*J(1/s).
        // W, N and D reverse as quartics. Fixed nominal degrees retain zero
        // ends, and reversal preserves W's and N/D's power-of-two normalization.
        ReverseCoefficients(data, signs, 0, 5);
        ReverseCoefficients(data, signs, 5, 3);
        ReverseCoefficients(data, signs, 8, 2);
        ReverseCoefficients(data, signs, 10, 3);
        ReverseCoefficients(data, signs, 13, 3);
        ReverseCoefficients(data, signs, 16, 5);
        ReverseCoefficients(data, signs, 21, 5);
        for (int index = 8; index < 16; index++) signs[index] = (sbyte)-signs[index];
    }

    private static void ReverseCoefficients(Span<ulong> data, Span<sbyte> signs, int start, int count)
    {
        int words = data.Length / signs.Length;
        signs.Slice(start, count).Reverse();
        for (int index = 0; index < count / 2; index++)
        {
            Span<ulong> first = data.Slice((start + index) * words, words),
                second = data.Slice((start + count - 1 - index) * words, words);
            for (int word = 0; word < words; word++)
                (first[word], second[word]) = (second[word], first[word]);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void BuildValues(in WideAxis3 e, in WideAxis3 c, Signed320 radius, int valueShift,
        Span<ulong> values, Span<sbyte> signs)
    {
        Span<ulong> invariants = stackalloc ulong[8 * ValueWords];
        Span<sbyte> invariantSigns = stackalloc sbyte[8];
        invariants.Clear(); invariantSigns.Clear();
        Write(invariants, invariantSigns, 0, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))), ValueWords);
        Write(invariants, invariantSigns, 1, e.SquaredLength, ValueWords);
        Write(invariants, invariantSigns, 2, Signed576.ExtendValue(e.Y), ValueWords);
        Write(invariants, invariantSigns, 3, c.SquaredLength, ValueWords);
        Write(invariants, invariantSigns, 4, Signed576.ExtendValue(c.Y), ValueWords);
        Write(invariants, invariantSigns, 5, WideAxis3.Dot(e, c), ValueWords);
        Write(invariants, invariantSigns, 6, WideArithmetic.MultiplySigned320(radius, radius), ValueWords);
        CylinderPairSideValuePolynomial.BuildUnshifted(invariants, invariantSigns, ValueWords, values, signs);
        // Retained edges keep B<398,C<198 bits; shifted c gives q<466,
        // x<232,y<432,rho<454. At ValueShift<=472, weighted linear
        // invariant heights are <872. Quartic convolutions including their
        // carries stay below 3500 bits, inside 56 words (3584 bits).
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(values, 5, valueShift);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values, signs);
    }

    internal static void GetBasis(WideAxis3 edge, out WideAxis3 first, out WideAxis3 second)
    {
        if (!edge.X.IsZero)
        {
            first = new WideAxis3(WideArithmetic.Negate(edge.Y), edge.X, default);
            second = new WideAxis3(WideArithmetic.Negate(edge.Z), default, edge.X);
        }
        else
        {
            first = new WideAxis3(default, WideArithmetic.Negate(edge.Z), edge.Y);
            second = new WideAxis3(edge.Y, default, default);
        }
    }

    internal static void WriteGradient(WideAxis3 first, WideAxis3 second, Span<ulong> values, Span<sbyte> signs)
    {
        values.Clear();
        for (int component = 0; component < 3; component++)
        {
            Signed320 a = Component(first, component), b = Component(second, component);
            Span<ulong> firstSlot = values.Slice(component * 10, 5), secondSlot = values.Slice(component * 10 + 5, 5);
            WideArithmetic.GetMagnitude(a, out firstSlot[4], out firstSlot[3], out firstSlot[2], out firstSlot[1], out firstSlot[0]);
            WideArithmetic.GetMagnitude(b, out secondSlot[4], out secondSlot[3], out secondSlot[2], out secondSlot[1], out secondSlot[0]);
            signs[2 * component] = (sbyte)a.Sign; signs[2 * component + 1] = (sbyte)b.Sign;
        }
    }
    internal static WideAxis3 Transform(WideRationalBasis3d basis, WideAxis3 axis) => new(
        TransformComponent(basis.Xx, basis.Yx, basis.Zx, axis),
        TransformComponent(basis.Xy, basis.Yy, basis.Zy, axis),
        TransformComponent(basis.Xz, basis.Yz, basis.Zz, axis));
    internal static Signed320 TransformComponent(Signed192 x, Signed192 y, Signed192 z, WideAxis3 axis) =>
        Signed320.NarrowValue(WideAxis3.Dot(new WideAxis3(Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z)), axis));
    internal static Signed576 RadialDot(in WideAxis3 a, in WideAxis3 b) => WideArithmetic.AddSigned576(
        WideArithmetic.MultiplySigned320(a.X, b.X), WideArithmetic.MultiplySigned320(a.Z, b.Z));
    internal static Signed576 Twice(Signed576 value) => WideArithmetic.AddSigned576(value, value);
    internal static int Sign(ref FiniteAxisValueRoot root, ReadOnlySpan<ulong> data, ReadOnlySpan<sbyte> signs, int start, int count) =>
        WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root,
            data.Slice(start * (data.Length / signs.Length), count * (data.Length / signs.Length)), signs.Slice(start, count));
    internal static void Write(Span<ulong> values, Span<sbyte> signs, int index, Signed576 value, int words = Words)
    {
        Span<ulong> target = values.Slice(index * words, words);
        target.Clear(); WideArithmetic.GetMagnitude(value, target[..9]); signs[index] = (sbyte)value.Sign;
    }
}
