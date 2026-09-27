//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Recovers individual real rim points at repeated cylinder-pair values.
/// </summary>
internal static class CylinderPairRepeatedRim
{
    private const int Words = 244;
    private const int Slots = 64;
    private const int Radical = 48;

    internal static bool TryGetZeroNormal(in CylinderPairGeometry geometry, int firstCapSign,
        int secondCapSign, int branchOrdinal, out Vector3d normal)
    {
        normal = default;
        System.Diagnostics.Debug.Assert((uint)branchOrdinal < 2);
        // E needs <984 bits; the largest radial tangent-cross dot needs <2500.
        const int words = 40;
        Span<ulong> data = stackalloc ulong[Slots * 2 * words];
        Span<sbyte> signs = stackalloc sbyte[Slots * 2];
        Span<ulong> products = stackalloc ulong[2 * words];
        var r = new Ring(data, signs, products, words);
        Prepare(geometry, firstCapSign, secondCapSign, r);
        // b.p=-b.c cuts the first circle in at most two points.
        r.Copy(8, 28); r.Copy(9, 29); r.Copy(10, 30);
        r.Mul(28, 28, 39); r.Mul(1, 39, 40);
        r.Mul(29, 29, 39); r.Mul(0, 39, 41); r.Add(40, 41, 31);
        // Positive: the pair owner excluded parallel axes before rim recovery.
        r.Mul(3, 31, 39); r.Mul(30, 30, 40); r.Mul(11, 40, 41); r.Sub(39, 41, Radical);
        if (r[Radical].Signs[0] < 0 || (r[Radical].Signs[0] == 0 && branchOrdinal != 0)) return false;
        WideAxis3 c = geometry.GetCapOffset(firstCapSign, secondCapSign);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            r.Import(39, Component(geometry.PlaneU, coordinate));
            r.Import(40, Component(geometry.PlaneW, coordinate));
            r.Mul(28, 39, 41); r.Mul(1, 41, 42);
            r.Mul(29, 40, 41); r.Mul(0, 41, 43); r.Add(42, 43, 44);
            r.Mul(30, 44, 33 + coordinate); r.Negate(33 + coordinate, 33 + coordinate);
            r.Mul(29, 39, 41); r.Mul(28, 40, 42); r.Sub(41, 42, 43);
            r.CopyOrdinaryToRadical(43, 33 + coordinate, branchOrdinal == 0 ? 1 : -1);
            r.Import(39, Component(c, coordinate)); r.Mul(39, 31, 40);
            r.Add(33 + coordinate, 40, 36 + coordinate); r.Negate(36 + coordinate, 36 + coordinate);
        }
        DotBaseVectors(r, 36, 36, 49);
        r.Mul(31, 31, 39); r.Mul(4, 39, 40); r.Sub(49, 40, 41);
        if (r.Sign(41) != 0) return false;
        CrossAxis(r, geometry.FirstAxis, 33, 49);
        CrossAxis(r, geometry.SecondAxis, 36, 52);
        CrossBaseVectors(r, 49, 52, 55);
        DotBaseVectors(r, 33, 55, 58); DotBaseVectors(r, 36, 55, 59);
        int orientation = r.Sign(58);
        if (orientation == 0 || orientation != r.Sign(59)) return false;
        // Parallel rim tangents have zero cross product; any nontrivial
        // admitting cone then has a cap/side boundary handled earlier.
        for (int coordinate = 0; coordinate < 3; coordinate++)
        { r.Copy(55 + coordinate, 33 + 2 * coordinate); r.Clear(34 + 2 * coordinate); }
        r.Clear(32); // Only the first radical is needed at exact touch.
        return TryFinishNormal(geometry, firstCapSign, secondCapSign, orientation, r, out normal);
    }

    /// <summary>
    /// Tests one eigenvalue/value/circle branch, numbered zero through seven.
    /// The selected normalized squared value is represented by its primitive
    /// repeated-root radical, of degree at most four and height at most 3714 bits.
    /// </summary>
    internal static bool TryGetRankOneNormal(in CylinderPairGeometry geometry,
        int firstCapSign, int secondCapSign, FiniteAxisValueRoot selectedValue,
        int branchOrdinal, out Vector3d normal)
    {
        normal = default;
        System.Diagnostics.Debug.Assert((uint)branchOrdinal < 8);

        // In the pair x+y*sqrt(E), use weighted height
        // max(bits(x), bits(y)+ceil(bits(E)/2)). Products add heights plus
        // one carry bit; no monic-field leading coefficient is multiplied
        // into every operation. From the exact-frame row weights
        // (33,66,229), the rational-eigenvalue rank minor has coefficient
        // bounds (2573,2115,1657) in physical S. Thus E needs <=4234 bits.
        // Rounded stage ceilings are: line 3008, D 6208, Delta 6720,
        // U 6528, H numerator 8448, local direction 15352, world direction
        // 15424, and cap half-axis dot 15584 <244*64.
        // The 64-pair arena is 249856 bytes. Root matching uses only a
        // quadratic query below 4400 bits, before rounding. Its caller's
        // degree-four repeated-root arena is below 430 KiB, keeping this
        // phase below 720 KiB. Rounding has a separate 133280-byte arena and
        // below-128-KiB nested radical-sign scratch, not a nested root solver.
        Span<ulong> data = stackalloc ulong[Slots * 2 * Words];
        Span<sbyte> signs = stackalloc sbyte[Slots * 2];
        Span<ulong> products = stackalloc ulong[2 * Words];
        var ring = new Ring(data, signs, products, Words);
        Prepare(geometry, firstCapSign, secondCapSign, ring);
        if (!GetValueAndLine(ring, branchOrdinal)
            || !MatchesValue(ring, selectedValue, geometry.ValueShift))
            return false;
        return GetRegularNormal(geometry, firstCapSign, secondCapSign,
            branchOrdinal & 1, ring, out normal);
    }

    private static void Prepare(in CylinderPairGeometry geometry, int firstSign,
        int secondSign, Ring r)
    {
        Span<ulong> values = stackalloc ulong[11 * 9];
        Span<sbyte> signs = stackalloc sbyte[11];
        geometry.WriteInvariants(firstSign, secondSign, values, signs, 9);
        for (int i = 0; i < 11; i++)
            r.Import(i, values.Slice(i * 9, 9), signs[i]);
        r.Mul(0, 1, 11);                           // gh
        r.Add(5, 3, 12);                           // z0=c²+rho
        r.Sub(12, 4, 13);                          // k0=z0-tau
        // Qij=4(delta*ci*cj+tau*bi*bj).
        for (int i = 0; i < 3; i++)
        {
            int x = i == 2 ? 7 : 6;
            int y = i == 0 ? 6 : 7;
            r.Mul(x, y, 39); r.Mul(2, 39, 40);
            r.Mul(x + 2, y + 2, 39); r.Mul(4, 39, 41);
            r.Add(40, 41, 14 + i); r.Scale(14 + i, 4, 14 + i);
        }
        // li=ti0+ti1*S, d=d0+d1*S+d2*S².
        for (int i = 0; i < 2; i++)
        {
            r.Mul(2, 6 + i, 39); r.Scale(39, 2, 39);
            r.Negate(39, 18 + 2 * i);
            r.Mul(39, 13, 40);
            r.Mul(4, 10, 39); r.Mul(39, 8 + i, 41); r.Scale(41, 4, 41);
            r.Add(40, 41, 17 + 2 * i);
        }
        r.Mul(13, 13, 39); r.Mul(2, 39, 40);
        r.Mul(10, 10, 39); r.Mul(4, 39, 41); r.Scale(41, 4, 41);
        r.Add(40, 41, 21);
        r.Scale(4, 2, 39); r.Add(13, 39, 40);
        r.Mul(2, 40, 22); r.Scale(22, 2, 22); r.Negate(22, 22);
        r.Copy(2, 23);
        // C=h*c0²+g*c1², B=h*b0²+g*b1², J=h*c0*b0+g*c1*b1.
        r.Mul(6, 6, 39); r.Mul(1, 39, 40);
        r.Mul(7, 7, 39); r.Mul(0, 39, 41); r.Add(40, 41, 34);
        r.Mul(8, 8, 39); r.Mul(1, 39, 40);
        r.Mul(9, 9, 39); r.Mul(0, 39, 41); r.Add(40, 41, 35);
        r.Mul(6, 8, 39); r.Mul(1, 39, 40);
        r.Mul(7, 9, 39); r.Mul(0, 39, 41); r.Add(40, 41, 36);
        r.Mul(2, 34, 37); r.Mul(4, 35, 38); // A=delta*C, T=tau*B
    }

    private static bool GetValueAndLine(Ring r, int branch)
    {
        int eigenSign = (branch & 4) == 0 ? -1 : 1;
        bool valueUpper = (branch & 2) != 0;
        r.Copy(11, 25); // lambda denominator gh >0
        bool mixedProjection = r.Sign(36) != 0;
        r.Mul(34, 35, 39); r.Mul(36, 36, 40); r.Sub(39, 40, 51);
        bool independentProjection = r.Sign(51) != 0;
        if (mixedProjection && independentProjection)
        {
            if (valueUpper)
                return false;
            // E=(A-T)²+4*delta*tau*J², lambda=2(A+T+eps sqrt(E))/gh.
            r.Sub(37, 38, 39); r.Mul(39, 39, 40);
            r.Mul(36, 36, 39); r.Mul(2, 4, 41); r.Mul(39, 41, 42);
            r.Scale(42, 4, 42); r.Add(40, 42, Radical);
            r.Add(37, 38, 24); r.Scale(24, 2, 24); r.SetRadical(24, 2 * eigenSign);
            // S=k0+j*(T-A+eps sqrt(E))/(delta*J).
            r.Mul(2, 36, 27); r.Mul(27, 13, 39);
            r.Sub(38, 37, 40); r.Mul(10, 40, 41); r.Add(39, 41, 26);
            r.CopyOrdinaryToRadical(10, 26, eigenSign);
            if (r.Sign(27) < 0) { r.Negate(27, 27); r.Negate(26, 26); }
            BuildSpatial(r);
        }
        else
        {
            if (mixedProjection)
            {
                // Dependent projections have eigenvalues 0 and 4(A+T)/gh.
                // In particular the zero eigenvector is perpendicular to
                // both c and b: compatibility cannot be divided by c.v.
                if (eigenSign < 0) r.Clear(24);
                else { r.Add(37, 38, 24); r.Scale(24, 4, 24); }
            }
            else r.Scale(eigenSign < 0 ? 37 : 38, 4, 24);
            BuildSpatial(r);
            int row = r.Sign(31) != 0 ? 0 : r.Sign(33) != 0 ? 1 : -1;
            if (row < 0) return false; // Rank zero is an earlier cap/side continuum.
            int diagonal = row == 0 ? 31 : 33;
            int own = row == 0 ? 17 : 19;
            int other = row == 0 ? 19 : 17;
            r.Mul(32, own, 39); r.Mul(diagonal, other, 40); r.Sub(39, 40, 41);
            r.Mul(32, own + 1, 39); r.Mul(diagonal, other + 1, 40); r.Sub(39, 40, 42);
            if (r.Sign(42) != 0)
            {
                if (valueUpper) return false;
                r.Negate(41, 26); r.Copy(42, 27);
                if (r.Sign(27) < 0) { r.Negate(27, 27); r.Negate(26, 26); }
            }
            else
            {
                if (r.Sign(41) != 0) return false;
                // Remaining rank-one minor: mii*(lambdaDen*d+lambdaNum*rho)
                // -(lambdaDen*li)². Its three coefficients stay rational.
                r.Mul(25, own, 43); r.Mul(25, own + 1, 44);
                for (int power = 0; power < 3; power++)
                {
                    r.Mul(25, 21 + power, 39);
                    if (power == 0) { r.Mul(24, 3, 40); r.Add(39, 40, 39); }
                    r.Mul(diagonal, 39, 40);
                    r.Mul(power == 2 ? 44 : 43, power == 0 ? 43 : 44, 39);
                    if (power == 1) r.Scale(39, 2, 39);
                    r.Sub(40, 39, 45 + power);
                }
                if (!SolveRationalQuadratic(r, 45, 46, 47, valueUpper)) return false;
            }
        }
        if (r.Sign(26) <= 0) return false;

        // Homogeneous M=T-lambda*C at S=N/d. Spatial M has rank one;
        // compatibility and the remaining diagonal minor certify full rank one.
        // The rational branch already rejected rank zero. In the independent
        // mixed branch, (A-T)²+4*delta*tau*J²>0 gives distinct eigenvalues,
        // hence this spatial block has rank one and a nonzero diagonal.
        int selectedRow = r.Sign(31) != 0 ? 0 : 1;
        for (int i = 0; i < 2; i++)
        {
            r.Mul(17 + 2 * i, 27, 39); r.Mul(18 + 2 * i, 26, 40);
            r.Add(39, 40, 41); r.Mul(25, 41, 49 + i); // ld*d*li
        }
#if DEBUG
        // The selected S solves compatibility. In the rational branches this
        // is the linear equation above (or both coefficients vanish); rank
        // one makes the other row dependent. The independent mixed branch's
        // explicit S solves the same null-eigenvector equation with J!=0.
        r.Mul(32, 49, 39); r.Mul(31, 50, 40); r.Sub(39, 40, 41);
        System.Diagnostics.Debug.Assert(r.Sign(41) == 0);
        r.Mul(32, 50, 39); r.Mul(33, 49, 40); r.Sub(39, 40, 41);
        System.Diagnostics.Debug.Assert(r.Sign(41) == 0);
#endif
        r.Mul(27, 27, 43); r.Mul(26, 27, 44); r.Mul(26, 26, 45);
        r.Mul(21, 43, 39); r.Mul(22, 44, 40); r.Add(39, 40, 41);
        r.Mul(23, 45, 39); r.Add(41, 39, 41); r.Mul(25, 41, 42);
        r.Mul(24, 3, 39); r.Mul(39, 43, 40); r.Add(42, 40, 42);
        int mii = selectedRow == 0 ? 31 : 33;
        int ti = selectedRow == 0 ? 49 : 50;
        r.Mul(mii, 42, 39); r.Mul(ti, ti, 40); r.Sub(39, 40, 41);
        if (r.Sign(41) != 0) return false;
        r.Mul(selectedRow == 0 ? 31 : 32, 27, 28);
        r.Mul(selectedRow == 0 ? 32 : 33, 27, 29);
        r.Copy(ti, 30);
        return true;
    }

    private static void BuildSpatial(Ring r)
    {
        r.Mul(25, 14, 39); r.Mul(24, 0, 40); r.Sub(39, 40, 31);
        r.Mul(25, 15, 32);
        r.Mul(25, 16, 39); r.Mul(24, 1, 40); r.Sub(39, 40, 33);
    }

    private static bool SolveRationalQuadratic(Ring r, int c, int b, int a, bool upper)
    {
        // For rank-one recovery, identically zero
        // compatibility puts projected c in range(M). In an orthonormal
        // frame M=diag(m,0), c=(u,0), b=(s,t), its off-diagonal gives st=0.
        // The quadratic coefficient is proportional to 4*delta*tau*(s²-t²),
        // nonzero because nonparallel axes give a nonzero projected b.
        System.Diagnostics.Debug.Assert(r.Sign(a) != 0);
        r.Mul(b, b, 39); r.Mul(a, c, 40); r.Scale(40, 4, 40);
        r.Sub(39, 40, Radical);
        if (r[Radical].Signs[0] < 0 || (r[Radical].Signs[0] == 0 && upper)) return false;
        r.Negate(b, 26); r.SetRadical(26, upper ? 1 : -1); r.Scale(a, 2, 27);
        if (r.Sign(27) < 0) { r.Negate(27, 27); r.Negate(26, 26); }
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool MatchesValue(Ring r, FiniteAxisValueRoot root, int shift)
    {
        // d*2^shift*t-x = y*sqrt(E). Squaring alone would admit the other
        // eigen/value branch, so the unsquared sign is checked as well.
        const int words = 70;
        Span<ulong> data = stackalloc ulong[6 * words];
        data.Clear(); // A zero signed term may skip the magnitude writer.
        Span<sbyte> signs = stackalloc sbyte[6];
        Span<ulong> products = stackalloc ulong[2 * words];
        Span<ulong> e = stackalloc ulong[words];
        CopyMagnitude(r[Radical].Low, e);
        Pair n = r[26], d = r[27];
        Span<ulong> x2 = data[..words];
        Span<ulong> coefficient1 = data.Slice(words, words);
        Span<ulong> coefficient2 = data.Slice(2 * words, words);
        WideArithmetic.MultiplyMagnitudes(n.Low, n.Low, x2);
        int constantSign = MagnitudeSign(x2);
        WideArithmetic.MultiplyMagnitudes(n.High, n.High, products[..words]);
        WideArithmetic.MultiplyMagnitudes(products[..words], e, products[words..]);
        WideArithmetic.AddShiftedSignedMagnitude(products[words..], -MagnitudeSign(products[words..]),
            0, x2, ref constantSign);
        WideArithmetic.MultiplyMagnitudes(n.Low, d.Low, products[..words]);
        int linearSign = 0;
        WideArithmetic.AddShiftedSignedMagnitude(products[..words], -n.Signs[0], shift + 1,
            coefficient1, ref linearSign);
        WideArithmetic.MultiplyMagnitudes(d.Low, d.Low, products[..words]);
        int quadraticSign = 0;
        WideArithmetic.AddShiftedSignedMagnitude(products[..words], 1, 2 * shift,
            coefficient2, ref quadraticSign);
        signs[0] = (sbyte)constantSign; signs[1] = (sbyte)linearSign; signs[2] = (sbyte)quadraticSign;
        if (WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, data[..(3 * words)], signs[..3]) != 0)
            return false;
        Span<ulong> linear = data.Slice(3 * words, 2 * words);
        CopyMagnitude(n.Low, linear[..words]); signs[3] = (sbyte)-n.Signs[0];
        int denominatorSign = 0;
        WideArithmetic.AddShiftedSignedMagnitude(d.Low, 1, shift, linear[words..], ref denominatorSign);
        signs[4] = (sbyte)denominatorSign;
        int actualSign = WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, linear, signs.Slice(3, 2));
        return actualSign == (r[Radical].Signs[0] == 0 ? 0 : n.Signs[1]);
    }

    private static bool GetRegularNormal(in CylinderPairGeometry geometry, int firstSign,
        int secondSign, int circleBranch, Ring r, out Vector3d normal)
    {
        normal = default;
        r.Mul(28, 28, 39); r.Mul(1, 39, 40);
        r.Mul(29, 29, 39); r.Mul(0, 39, 41); r.Add(40, 41, 31); // D
        r.Mul(3, 31, 39); r.Mul(30, 30, 40); r.Mul(11, 40, 41); r.Sub(39, 41, 32); // Delta
        int deltaSign = r.Sign(32);
        if (deltaSign < 0 || (deltaSign == 0 && circleBranch != 0)) return false;
        WideAxis3 c = geometry.GetCapOffset(firstSign, secondSign);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            r.Import(39, Component(geometry.PlaneU, coordinate));
            r.Import(40, Component(geometry.PlaneW, coordinate));
            r.Mul(28, 39, 41); r.Mul(1, 41, 42);
            r.Mul(29, 40, 41); r.Mul(0, 41, 43); r.Add(42, 43, 44);
            r.Mul(30, 44, 33 + 2 * coordinate); r.Negate(33 + 2 * coordinate, 33 + 2 * coordinate);
            r.Mul(29, 39, 41); r.Mul(28, 40, 42); r.Sub(41, 42, 34 + 2 * coordinate);
            if (circleBranch != 0) r.Negate(34 + 2 * coordinate, 34 + 2 * coordinate);
            r.Import(39, Component(c, coordinate)); r.Mul(39, 31, 40);
            r.Add(33 + 2 * coordinate, 40, 33 + 2 * coordinate); // U=cD+P
        }
        Dot(r, c, 33, 43, 44);
        r.Add(3, 4, 39); r.Sub(39, 5, 39); r.Mul(27, 31, 40);
        r.Mul(39, 40, 41); r.Mul(26, 31, 42); r.Sub(41, 42, 28);
        r.Mul(27, 43, 39); r.Scale(39, 2, 39); r.Add(28, 39, 28);
        r.Mul(27, 44, 29); r.Scale(29, 2, 29); // Hn=d*D*H
        int hSign = SignExtension(r[28], r[29], r[32], r[Radical].Low);
        if (hSign == 0) return false; // Earlier cap/side candidates dominate H=0.
        r.Mul(27, 4, 39); r.Mul(39, 31, 30); r.Scale(30, 2, 30); // 2*d*tau*D
        Dot(r, geometry.SecondAxis, 33, 49, 50);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            int u0 = 33 + 2 * coordinate, u1 = u0 + 1;
            int v0 = coordinate == 0 ? 0 : coordinate == 1 ? 6 : 8, v1 = v0 + 1;
            r.Mul(28, u0, 39); r.Mul(29, u1, 40); r.Mul(40, 32, 41);
            r.Add(39, 41, 42); r.Mul(2, 42, v0);
            r.Mul(28, u1, 39); r.Mul(29, u0, 40); r.Add(39, 40, 41); r.Mul(2, 41, v1);
            r.Import(43, Component(geometry.SecondAxis, coordinate));
            for (int part = 0; part < 2; part++)
            {
                r.Mul(2, u0 + part, 39); r.Mul(43, 49 + part, 40); r.Sub(39, 40, 41);
                r.Mul(30, 41, 42); r.Sub(v0 + part, 42, v0 + part);
            }
        }
        // p.v has sign [dD*lambdaNum+2delta*lambdaDen*(Hn-2d*tau*D)]/Hn.
        r.Mul(27, 31, 39); r.Mul(39, 24, 40);
        r.Mul(2, 25, 41); r.Scale(41, 2, 41);
        r.Sub(28, 30, 39); r.Mul(41, 39, 42); r.Add(40, 42, 43);
        r.Mul(41, 29, 44);
        int firstRadial = SignExtension(r[43], r[44], r[32], r[Radical].Low) * hSign;
        r.Sub(30, 28, 43); r.Negate(29, 44);
        int secondRadial = SignExtension(r[43], r[44], r[32], r[Radical].Low);
        if (firstRadial == 0 || firstRadial != secondRadial) return false;
        // Negative radial signs give a unique closest disk pair with Hessian
        // JᵀJ+diag(-p.v,-q.v)>0. A rank-one pencil restricts to a squared
        // line on the first circle: two same-level points or fourth-order
        // tangency, neither compatible with that unique nondegenerate minimum.
        System.Diagnostics.Debug.Assert(firstRadial > 0);
        // Pack V contiguously once U is dead; cap vectors include zero half-height.
        r.Copy(0, 33); r.Copy(1, 34); r.Copy(6, 35); r.Copy(7, 36); r.Copy(8, 37); r.Copy(9, 38);
        return TryFinishNormal(geometry, firstSign, secondSign, hSign, r, out normal);
    }

    private static bool TryFinishNormal(in CylinderPairGeometry geometry, int firstSign,
        int secondSign, int orientation, Ring r, out Vector3d normal)
    {
        normal = default;
        Dot(r, geometry.FirstHalf, 33, 43, 44);
        if (firstSign * orientation * SignExtension(r[43], r[44], r[32], r[Radical].Low) < 0)
            return false;
        Dot(r, geometry.SecondHalf, 33, 43, 44);
        if (secondSign * orientation * SignExtension(r[43], r[44], r[32], r[Radical].Low) < 0)
            return false;
        Rotate(r, geometry.WorldBasis);
        normal = RoundNormal(r.data.Slice(49 * 2 * r.words, 12 * r.words),
            r.signs.Slice(49 * 2, 12), r.data.Slice(32 * 2 * r.words, 2 * r.words),
            r[32].Signs, r[Radical].Low, orientation);
        return true;
    }


    private static void DotBaseVectors(Ring r, int first, int second, int result)
    {
        r.Clear(result);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        { r.Mul(first + coordinate, second + coordinate, 39); r.Add(result, 39, result); }
    }

    private static void CrossAxis(Ring r, WideAxis3 axis, int vector, int result)
    {
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            int next = (coordinate + 1) % 3, last = (coordinate + 2) % 3;
            r.Import(39, Component(axis, next)); r.Mul(39, vector + last, 40);
            r.Import(39, Component(axis, last)); r.Mul(39, vector + next, 41);
            r.Sub(40, 41, result + coordinate);
        }
    }

    private static void CrossBaseVectors(Ring r, int first, int second, int result)
    {
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            int next = (coordinate + 1) % 3, last = (coordinate + 2) % 3;
            r.Mul(first + next, second + last, 39); r.Mul(first + last, second + next, 40);
            r.Sub(39, 40, result + coordinate);
        }
    }

    private static void Dot(Ring r, WideAxis3 axis, int vector, int ordinary, int radical)
    {
        r.Clear(ordinary); r.Clear(radical);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            r.Import(45, Component(axis, coordinate));
            r.Mul(45, vector + 2 * coordinate, 46); r.Add(ordinary, 46, ordinary);
            r.Mul(45, vector + 2 * coordinate + 1, 46); r.Add(radical, 46, radical);
        }
    }

    private static void Rotate(Ring r, WideRationalBasis3d basis)
    {
        Span<Signed192> rows = stackalloc Signed192[9]
        { basis.Xx, basis.Yx, basis.Zx, basis.Xy, basis.Yy, basis.Zy, basis.Xz, basis.Yz, basis.Zz };
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            r.Clear(49 + 2 * coordinate); r.Clear(50 + 2 * coordinate);
            for (int source = 0; source < 3; source++)
            {
                r.Import(45, Signed320.ExtendValue(rows[3 * coordinate + source]));
                for (int part = 0; part < 2; part++)
                {
                    r.Mul(45, 33 + 2 * source + part, 46);
                    r.Add(49 + 2 * coordinate + part, 46, 49 + 2 * coordinate + part);
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    // Borrow the three world coordinates as (a+b√E)+(c+d√E)√Delta.
    // Normalization does not own or mutate the geometric recovery workspace.
    // Inputs retain the recovery's weighted-height bounds, not arbitrary
    // coefficients filling every word independently of their radicands.
    internal static Vector3d RoundNormal(Span<ulong> coordinates, Span<sbyte> coordinateSigns,
        Span<ulong> delta, Span<sbyte> deltaSigns, ReadOnlySpan<ulong> e, int orientation)
    {
        // Squaring and the 66-bit midpoint shift need two input widths plus
        // two words. This remains 490 words for the largest rank-one path.
        int inputWords = coordinates.Length / 12;
        int words = 2 * inputWords + 2;
        Span<ulong> data = stackalloc ulong[16 * 2 * words];
        Span<sbyte> signs = stackalloc sbyte[32];
        Span<ulong> products = stackalloc ulong[2 * words];
        var r = new Ring(data, signs, products, words, 15);
        r.Import(15, e, MagnitudeSign(e));
        var deltaPair = new Pair(delta, deltaSigns, inputWords);
        r.Copy(deltaPair, 14);
        for (int component = 0; component < 3; component++)
        {
            var ordinary = new Pair(coordinates.Slice(4 * component * inputWords, 2 * inputWords),
                coordinateSigns.Slice(4 * component, 2), inputWords);
            var radical = new Pair(coordinates.Slice((4 * component + 2) * inputWords, 2 * inputWords),
                coordinateSigns.Slice(4 * component + 2, 2), inputWords);
            r.Copy(ordinary, 10); r.Copy(radical, 11);
            r.Mul(10, 10, 12); r.Mul(11, 11, 13); r.Mul(13, 14, 10);
            r.Add(12, 10, 2 * component);
            r.Copy(ordinary, 10);
            r.Mul(10, 11, 2 * component + 1); r.Scale(2 * component + 1, 2, 2 * component + 1);
            r.Add(6, 2 * component, 6); r.Add(7, 2 * component + 1, 7);
        }
        Span<long> result = stackalloc long[3];
        result.Clear();
        for (int component = 0; component < 3; component++)
        {
            var ordinary = new Pair(coordinates.Slice(4 * component * inputWords, 2 * inputWords),
                coordinateSigns.Slice(4 * component, 2), inputWords);
            var radical = new Pair(coordinates.Slice((4 * component + 2) * inputWords, 2 * inputWords),
                coordinateSigns.Slice(4 * component + 2, 2), inputWords);
            int sign = SignExtension(ordinary, radical, deltaPair, e) * orientation;
            if (sign == 0) continue;
            ulong lower = 0, upper = 1UL << 32;
            while (lower < upper)
            {
                ulong middle = lower + (upper - lower + 1) / 2;
                if (CompareNormal(r, component, 2 * middle) >= 0) lower = middle;
                else upper = middle - 1;
            }
            int midpoint = CompareNormal(r, component, 2 * lower + 1);
            if (midpoint > 0 || (midpoint == 0 && (lower & 1) != 0)) lower++;
            result[component] = sign * (long)lower;
        }
        return new Vector3d(Fixed64.FromRaw(result[0]), Fixed64.FromRaw(result[1]), Fixed64.FromRaw(result[2]));
    }

    private static int CompareNormal(Ring r, int component, ulong twiceRaw)
    {
        for (int part = 0; part < 2; part++)
        {
            r.Scale(6 + part, twiceRaw, 10); r.Scale(10, twiceRaw, 10);
            r.Shift(2 * component + part, 66, 11); r.Sub(11, 10, 8 + part);
        }
        return SignExtension(r[8], r[9], r[14], r[15].Low);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int SignExtension(Pair ordinary, Pair radical, Pair delta, ReadOnlySpan<ulong> e)
    {
        int u = SignPair(ordinary, e), v = SignPair(radical, e);
        if (v == 0 || SignPair(delta, e) == 0) return u;
        if (u == 0 || u == v) return v;
        int words = 2 * Math.Max(ordinary.Low.Length, radical.Low.Length);
        Span<ulong> data = stackalloc ulong[4 * 2 * words];
        Span<sbyte> signs = stackalloc sbyte[8];
        Span<ulong> products = stackalloc ulong[2 * words];
        var r = new Ring(data, signs, products, words, 3);
        r.Import(3, e, MagnitudeSign(e));
        r.Mul(ordinary, ordinary, 0); r.Mul(radical, radical, 1);
        r.Mul(r[1], delta, 2); r.Sub(0, 2, 1);
        return u * r.Sign(1);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int SignPair(Pair value, ReadOnlySpan<ulong> e) =>
        WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(
            value.Low, value.Signs[0], value.High, value.Signs[1], e);

    private static Signed320 Component(WideAxis3 value, int coordinate) =>
        coordinate == 0 ? value.X : coordinate == 1 ? value.Y : value.Z;

    private static int MagnitudeSign(ReadOnlySpan<ulong> value) =>
        WideArithmetic.GetActiveMagnitudeLength(value) == 0 ? 0 : 1;

    private static void CopyMagnitude(ReadOnlySpan<ulong> source, Span<ulong> destination)
    {
        int count = Math.Min(source.Length, destination.Length);
        source[..count].CopyTo(destination);
        destination[count..].Clear();
    }

    private readonly ref struct Pair
    {
        internal readonly Span<ulong> Low;
        internal readonly Span<ulong> High;
        internal readonly Span<sbyte> Signs;
        internal Pair(Span<ulong> data, Span<sbyte> signs, int words)
        { Low = data[..words]; High = data.Slice(words, words); Signs = signs; }
    }

    // Focused two-coefficient arithmetic for the two cylinder recovery radicals.
    // No inversion, field normalization, polynomial framework, or heap storage.
    private readonly ref struct Ring
    {
        internal readonly Span<ulong> data;
        internal readonly Span<sbyte> signs;
        private readonly Span<ulong> first;
        private readonly Span<ulong> second;
        internal readonly int words;
        private readonly int e;

        internal Ring(Span<ulong> data, Span<sbyte> signs, Span<ulong> products, int words, int e = Radical)
        {
            this.data = data; this.signs = signs; this.words = words; this.e = e;
            first = products[..words]; second = products.Slice(words, words);
            data.Clear(); signs.Clear();
        }

        internal Pair this[int index] => new(data.Slice(index * 2 * words, 2 * words), signs.Slice(index * 2, 2), words);
        internal void Clear(int index) { Pair p = this[index]; p.Low.Clear(); p.High.Clear(); p.Signs.Clear(); }
        internal int Sign(int index) => SignPair(this[index], this[e].Low);
        internal void Copy(int source, int target) => Copy(this[source], target);
        internal void Copy(Pair source, int target)
        {
            Pair p = this[target]; CopyMagnitude(source.Low, p.Low); CopyMagnitude(source.High, p.High);
            source.Signs.CopyTo(p.Signs);
        }
        internal void Negate(int source, int target)
        { Copy(source, target); Pair p = this[target]; p.Signs[0] *= -1; p.Signs[1] *= -1; }
        internal void Import(int target, ReadOnlySpan<ulong> magnitude, int sign)
        { Clear(target); Pair p = this[target]; CopyMagnitude(magnitude, p.Low); p.Signs[0] = (sbyte)sign; }
        internal void Import(int target, Signed320 value)
        {
            Clear(target); Pair p = this[target];
            WideArithmetic.GetMagnitude(value, out p.Low[4], out p.Low[3], out p.Low[2], out p.Low[1], out p.Low[0]);
            p.Signs[0] = (sbyte)value.Sign;
        }
        internal void SetRadical(int target, int value)
        { Pair p = this[target]; p.High.Clear(); p.High[0] = (ulong)Math.Abs(value); p.Signs[1] = (sbyte)Math.Sign(value); }
        internal void CopyOrdinaryToRadical(int source, int target, int sign)
        { Pair p = this[target]; CopyMagnitude(this[source].Low, p.High); p.Signs[1] = (sbyte)(this[source].Signs[0] * sign); }
        internal void Add(int x, int y, int target) => Sum(x, y, target, 1);
        internal void Sub(int x, int y, int target) => Sum(x, y, target, -1);
        private void Sum(int x, int y, int target, int sign)
        {
            Pair a = this[x], b = this[y], p = this[target];
            CopyMagnitude(a.Low, first); int lowSign = a.Signs[0];
            WideArithmetic.AddShiftedSignedMagnitude(b.Low, sign * b.Signs[0], 0, first, ref lowSign);
            CopyMagnitude(a.High, second); int highSign = a.Signs[1];
            WideArithmetic.AddShiftedSignedMagnitude(b.High, sign * b.Signs[1], 0, second, ref highSign);
            first.CopyTo(p.Low); second.CopyTo(p.High); p.Signs[0] = (sbyte)lowSign; p.Signs[1] = (sbyte)highSign;
        }
        internal void Mul(int x, int y, int target) => Mul(this[x], this[y], target);
        internal void Mul(Pair a, Pair b, int target)
        {
            Pair p = this[target];
            WideArithmetic.MultiplyMagnitudes(a.High, b.High, first);
            WideArithmetic.MultiplyMagnitudes(first, this[e].Low, second);
            WideArithmetic.MultiplyMagnitudes(a.Low, b.Low, p.Low);
            int lowSign = a.Signs[0] * b.Signs[0];
            WideArithmetic.AddShiftedSignedMagnitude(second, a.Signs[1] * b.Signs[1] * this[e].Signs[0], 0, p.Low, ref lowSign);
            WideArithmetic.MultiplyMagnitudes(a.Low, b.High, first);
            int highSign = a.Signs[0] * b.Signs[1];
            WideArithmetic.MultiplyMagnitudes(a.High, b.Low, second);
            WideArithmetic.AddShiftedSignedMagnitude(second, a.Signs[1] * b.Signs[0], 0, first, ref highSign);
            first.CopyTo(p.High); p.Signs[0] = (sbyte)lowSign; p.Signs[1] = (sbyte)highSign;
        }
        internal void Scale(int source, ulong factor, int target)
        {
            // Callers use positive constants or positive normal midpoints.
            System.Diagnostics.Debug.Assert(factor > 0);
            Span<ulong> scalar = stackalloc ulong[1] { factor };
            Pair a = this[source], p = this[target];
            WideArithmetic.MultiplyMagnitudes(a.Low, scalar, first);
            WideArithmetic.MultiplyMagnitudes(a.High, scalar, second);
            first.CopyTo(p.Low); second.CopyTo(p.High);
            p.Signs[0] = a.Signs[0]; p.Signs[1] = a.Signs[1];
        }
        internal void Shift(int source, int shift, int target)
        {
            Pair a = this[source], p = this[target]; first.Clear(); second.Clear();
            int lowSign = 0, highSign = 0;
            WideArithmetic.AddShiftedSignedMagnitude(a.Low, a.Signs[0], shift, first, ref lowSign);
            WideArithmetic.AddShiftedSignedMagnitude(a.High, a.Signs[1], shift, second, ref highSign);
            first.CopyTo(p.Low); second.CopyTo(p.High); p.Signs[0] = (sbyte)lowSign; p.Signs[1] = (sbyte)highSign;
        }
    }
}
