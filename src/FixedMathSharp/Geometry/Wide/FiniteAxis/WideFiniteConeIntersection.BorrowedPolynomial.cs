//=======================================================================
// WideFiniteConeIntersection.BorrowedPolynomial.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Retains affine-query polynomial coefficients through the existing exact quadratic root owner.</content>
internal static partial class WideFiniteConeIntersection
{
    internal static bool TrySolveBoundedUnitPolynomial(scoped ContactQuadratic a, scoped ContactQuadratic b,
        scoped ContactQuadratic c, Signed832 lowerN, Signed832 lowerD, Signed832 upperN, Signed832 upperD,
        Fixed64 outputScale, out Fixed64 entry, out Fixed64 exit)
    {
        entry = exit = default;
        int words = Math.Max(a.FieldWords, Math.Max(b.FieldWords,c.FieldWords)) + 14;
        Span<ulong> storage=stackalloc ulong[20*words]; Span<int> signs=stackalloc int[20];
        var loN=ContactQuadratic.At(storage,signs,0,words);var loD=ContactQuadratic.At(storage,signs,1,words);
        var hiN=ContactQuadratic.At(storage,signs,2,words);var hiD=ContactQuadratic.At(storage,signs,3,words);
        var n=ContactQuadratic.At(storage,signs,4,words);var d=ContactQuadratic.At(storage,signs,5,words);
        var lower=ContactQuadratic.At(storage,signs,6,words);var lowerDivisor=ContactQuadratic.At(storage,signs,7,words);
        var upper=ContactQuadratic.At(storage,signs,8,words);var upperDivisor=ContactQuadratic.At(storage,signs,9,words);
        lower.Set(lowerN);lowerDivisor.Set(lowerD);upper.Set(upperN);upperDivisor.Set(upperD);
        bool found=false;
        if(GetBorrowedPolynomialSignAtRationalParameter(a,b,c,lowerN,lowerD)<=0)
        {lower.CopyTo(loN);lowerDivisor.CopyTo(loD);lower.CopyTo(hiN);lowerDivisor.CopyTo(hiD);found=true;}
        if(GetBorrowedPolynomialSignAtRationalParameter(a,b,c,upperN,upperD)<=0)
        {if(!found){upper.CopyTo(loN);upperDivisor.CopyTo(loD);}upper.CopyTo(hiN);upperDivisor.CopyTo(hiD);found=true;}
        Span<ulong> root=stackalloc ulong[2*words];
        // Every retained root uses this same discriminant; rational endpoints
        // have a zero radical coefficient and therefore share its field.
        for(int branch=-1;branch<=1;branch+=2)
        {
            if(!ContactQuadratic.TryGetParameter(a,b,c,branch,root,n,d)) continue;
            if(ContactQuadratic.CompareRatios(n,d,root,lower,lowerDivisor,root)<0
                ||ContactQuadratic.CompareRatios(n,d,root,upper,upperDivisor,root)>0)continue;
            if(!found||ContactQuadratic.CompareRatios(n,d,root,loN,loD,root)<0){n.CopyTo(loN);d.CopyTo(loD);}
            if(!found||ContactQuadratic.CompareRatios(n,d,root,hiN,hiD,root)>0){n.CopyTo(hiN);d.CopyTo(hiD);}
            found=true;
        }
        if(!found)return false;
        ContactQuadratic.Scale(loN,Signed576.ExtendValue(Signed192.Raw(outputScale)),n);
        entry=ContactQuadratic.RoundRatio(n,loD,root,0,outputScale.m_rawValue);
        ContactQuadratic.Scale(hiN,Signed576.ExtendValue(Signed192.Raw(outputScale)),n);
        exit=ContactQuadratic.RoundRatio(n,hiD,root,0,outputScale.m_rawValue);
        return true;
    }

    internal static int GetBorrowedPolynomialSignAtRationalParameter(scoped ContactQuadratic a,
        scoped ContactQuadratic b, scoped ContactQuadratic c, Signed832 n, Signed832 d)
    {
        int words=Math.Max(a.FieldWords,Math.Max(b.FieldWords,c.FieldWords))+26;
        Span<ulong> storage=stackalloc ulong[6*words];Span<int> signs=stackalloc int[6];
        var value=ContactQuadratic.At(storage,signs,0,words);var term=ContactQuadratic.At(storage,signs,1,words);
        var work=ContactQuadratic.At(storage,signs,2,words);
        ContactQuadratic.Scale(a,n,work);ContactQuadratic.Scale(work,n,value);
        ContactQuadratic.Scale(b,n,work);ContactQuadratic.Scale(work,d,term);value.Add(term);value.Add(term);
        ContactQuadratic.Scale(c,d,work);ContactQuadratic.Scale(work,d,term);value.Add(term);
        return value.Signs[0];
    }
}
