//=======================================================================
// ConePlaneRayCharts.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Borrowed quadratic parameters for finite cone plane-section event charts.</summary>
internal static class ConePlaneRayCharts
{
    internal const int Words = 64;
    internal const int RootWords = 40;

    // Apex coordinates use F(p)=H²(px²+pz²)-R²py² and plane N.p=c.
    // For one normal orientation, F(p+tN)=A*t²+2B(p)*t+C(p).
    //
    // Candidate completeness is a boundary-stratum argument. A maximum
    // strictly inside the admitted section projects a global support face.
    // Otherwise p is on a triangle edge, the lower cone side, or lower base.
    // A switch between upper side and upper base is on the projected upper
    // rim; every remaining boundary is an endpoint of one of these strata.
    //
    // On the LOWER side C(p)=0. Its nonzero side root is -2B(p)/A;
    // admission of that root additionally requires B(p)<=0 (the derivative
    // at the upper root is -2B(p)). Choosing the other crossing would leave
    // the finite cone immediately and later enter the opposite nappe.
    // With g=(Rcos(theta),H,Rsin(theta)), D=N.g and p=c*g/D,
    // E=N^T M g=H²D-H(H²+R²)Ny. The side depth is -2cE/(AD).
    // Its derivative vanishes at D'=0 or is constant. Triangle, axial,
    // and switch boundaries are lines on this generator circle. c=0 and
    // D=0 retain finite generator intervals rather than dividing by zero.
    //
    // On the UPPER rim q=g, t=(N.q-c)/|N|² and p=q-tN. Triangle walls W
    // have W.N=0, so W.p=W.q. Lower-cone admission factors as
    // F(p)=t*(A*t-2B(q)); t>0 makes the remaining circle predicate linear.
    // t=0 is admitted separately before that division. Interior stationary
    // rim values are exactly global rim supports; clipping endpoints and
    // rim/apex tangencies supply the remaining extrema.
    //
    // The four signed quadrant charts use u in [0,1]:
    // cos=sx*(1-u²)/(1+u²), sin=sz*2u/(1+u²).
    // Every admitted circle line is therefore a quadratic in u. On a
    // clipped straight edge G(s,t)=a*s²+2(b+m*t)*s+c+2n*t+A*t²;
    // eliminating stationary s gives a*(c+2n*t+A*t²)-(b+m*t)²=0.
    // Axial endpoints are rational. Lower-side edge roots need only the
    // original quadratic followed by t=-2B/A. No nested radical is needed.
    //
    // Full-domain ledger for the shared unreduced exact rigid frame:
    // point/edge/apex differences <198 bits, N <260, |N|² <522, c <460;
    // raw H/R squares <126, scaled base H/R <193. Authored normal-cross-
    // edge walls transformed without narrowing are <326; wall offsets
    // <526. Lower-circle triangle lines <990 and projected-rim lower-cone
    // lines <1118. Edge stationary polynomial coefficients <1184. Thus
    // face-stratum defining coefficients <1248 and discriminants <2498 bits.
    // Rational axis/generator point rays additionally have A<1302, B<1241,
    // C<1181; their discriminants remain <2485 bits.
    // Forty words retain the complete discriminant; sixty-four words retain
    // the unreduced evaluated depth numerator/denominator (<4096 bits).
    // Admission products and depth comparisons use larger transient storage;
    // these bounds do not authorize materializing those products in a field.

    /// <summary>Retains one branch of a*u²+2*b*u+c=0 without rounding.</summary>
    /// <remarks>
    /// Inputs are rational integer coefficients satisfying the discriminant ledger above.
    /// Outputs borrow caller storage. A zero polynomial denotes a family and
    /// returns false; callers must retain its interval/endpoints separately.
    /// Branches are -1 and +1; the linear and repeated cases use +1 only.
    /// </remarks>
    internal static bool TryGetParameter(ContactQuadratic a, ContactQuadratic b, ContactQuadratic c,
        int branch, Span<ulong> root, ContactQuadratic numerator, ContactQuadratic denominator)
    {
        root.Clear(); numerator.Clear(); denominator.Clear();
        if (a.Signs[0] == 0)
        {
            if (b.Signs[0] == 0 || branch < 0)
                return false;
            c.CopyTo(numerator, -b.Signs[0]);
            b.CopyTo(denominator, b.Signs[0]); denominator.Add(denominator);
            return true;
        }
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic discriminant = ContactQuadratic.At(work, signs, 0, Words);
        ContactQuadratic product = ContactQuadratic.At(work, signs, 1, Words);
        ContactQuadratic.Multiply(b, b, root, discriminant);
        ContactQuadratic.Multiply(a, c, root, product);
        discriminant.Add(product, -1);
        if (discriminant.Signs[0] < 0 || discriminant.Signs[0] == 0 && branch < 0)
            return false;
        int active = WideArithmetic.GetActiveMagnitudeLength(discriminant.Rational);
        System.Diagnostics.Debug.Assert(active <= RootWords);
        discriminant.Rational[..active].CopyTo(root);
        int orientation = a.Signs[0];
        b.CopyTo(numerator, -orientation);
        numerator.Radical[0] = 1;
        numerator.Signs[1] = branch;
        a.CopyTo(denominator, orientation);
        return true;
    }

    internal static bool IsUnitParameter(ContactQuadratic numerator, ContactQuadratic denominator,
        ReadOnlySpan<ulong> root)
    {
        if (numerator.Sign(root) < 0)
            return false;
        Span<ulong> work = stackalloc ulong[2 * Words];
        Span<int> signs = stackalloc int[2];
        ContactQuadratic difference = ContactQuadratic.At(work, signs, 0, Words);
        numerator.CopyTo(difference); difference.Add(denominator, -1);
        return difference.Sign(root) <= 0;
    }
}
