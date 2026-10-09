//=======================================================================
// SegmentConeSurfaceCandidates.FamilyCircles.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Unit-circle boundary roots and clipped apex-disk vertices.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    private static bool BuildFamilyCircle(SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        scoped ReadOnlySpan<WideAxis3> fan, ConeSurfaceFamilyEvent item, scoped ref ConeSurfaceFamilyWitness witness)
    {
        Span<ulong> work = stackalloc ulong[24 * FamilyWords]; Span<int> signs = stackalloc int[24];
        ContactQuadratic u = At(work, signs, 0, FamilyWords), v = At(work, signs, 1, FamilyWords), d = At(work, signs, 2, FamilyWords);
        ContactQuadratic a = At(work, signs, 3, FamilyWords), b = At(work, signs, 4, FamilyWords), c = At(work, signs, 5, FamilyWords);
        ContactQuadratic aa = At(work, signs, 6, FamilyWords), bb = At(work, signs, 7, FamilyWords), cc = At(work, signs, 8, FamilyWords);
        ContactQuadratic product = At(work, signs, 9, FamilyWords), temporary = At(work, signs, 10, FamilyWords);
        u.Clear(); v.Clear(); d.Set(ExtendFamily(Signed192.Signed(1)));
        if (item.Kind == ConeSurfaceFamilyEventKind.CircleSeam)
            (item.First % 2 == 0 ? u : v).Set(ExtendFamily(Signed192.Signed(item.First < 2 ? 1 : -1)));
        else if (item.Kind == ConeSurfaceFamilyEventKind.CircleBoundary)
        {
            FamilyCircleLine(candidate, geometry, fan, item, witness, item.First, a, b, c);
            Multiply(a, a, witness.Root, d); Multiply(b, b, witness.Root, product); d.Add(product);
            if (d.Sign(witness.Root) == 0) return false;
            Multiply(c, c, witness.Root, product); d.CopyTo(temporary); temporary.Add(product, -1);
            if (temporary.Sign(witness.Root) < 0) return false;
            // a*u+b*v+c=0 intersects u²+v²=1 at
            // (-a*c +/- b*sqrt(D-c²), -b*c -/+ a*sqrt(D-c²))/D.
            // Retaining this field preserves irrational singleton domains.
            Multiply(a, c, witness.Root, u); u.MultiplySign(-1);
            Multiply(b, c, witness.Root, v); v.MultiplySign(-1);
            temporary.Rational.CopyTo(witness.Root);
            b.Rational.CopyTo(u.Radical); u.Signs[1] = item.Branch * b.Signs[0];
            a.Rational.CopyTo(v.Radical); v.Signs[1] = -item.Branch * a.Signs[0];
        }
        else if (item.Kind == ConeSurfaceFamilyEventKind.DiskPair)
        {
            FamilyCircleLine(candidate, geometry, fan, item, witness, item.First, a, b, c);
            FamilyCircleLine(candidate, geometry, fan, item, witness, item.Second, aa, bb, cc);
            Multiply(a, bb, witness.Root, d); Multiply(b, aa, witness.Root, product); d.Add(product, -1);
            int direction = d.Sign(witness.Root);
            if (direction == 0) return false;
            Multiply(b, cc, witness.Root, u); Multiply(c, bb, witness.Root, product); u.Add(product, -1);
            Multiply(c, aa, witness.Root, v); Multiply(a, cc, witness.Root, product); v.Add(product, -1);
            u.MultiplySign(direction); v.MultiplySign(direction); d.MultiplySign(direction);
            Multiply(u, u, witness.Root, temporary); Multiply(v, v, witness.Root, product); temporary.Add(product);
            Multiply(d, d, witness.Root, product); temporary.Add(product, -1);
            if (temporary.Sign(witness.Root) > 0) return false;
        }
        Signed576 radial = candidate.Feature == ConeSurfaceFeature.Rim ? Signed576.ExtendValue(geometry.Radius)
            : ExtendFamily(Signed192.Raw(candidate.Input.Height));
        Scale(u, radial, witness[0]); Scale(v, radial, witness[2]);
        if (candidate.Family == ConeSurfaceFamily.RotationCircle)
        { witness[0].MultiplySign(-1); witness[2].MultiplySign(-1); }
        if (candidate.Feature == ConeSurfaceFeature.Rim)
        {
            TryGetRimPoint(geometry, candidate.RootOrdinal, out _, out _, out Signed320 offset, out _, out _);
            Scale(d, Signed576.ExtendValue(offset), witness[1]);
        }
        else
        {
            Scale(d, ExtendFamily(Signed192.Raw(candidate.Input.Radius)), witness[1]); witness[1].MultiplySign(-1);
        }
        if (candidate.Family == ConeSurfaceFamily.NormalCone)
        {
            for (int axis = 0; axis < 3; axis++) witness[3 + axis].CopyTo(witness[6 + axis]);
            return true;
        }
        BuildFamilyCircleAnchors(candidate, geometry, u, v, d, ref witness);
        return true;
    }

    private static void FamilyCircleLine(SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        scoped ReadOnlySpan<WideAxis3> fan, ConeSurfaceFamilyEvent item, scoped ConeSurfaceFamilyWitness witness,
        int index, ContactQuadratic a, ContactQuadratic b, ContactQuadratic c)
    {
        FamilyConstraint(geometry, fan, item, witness, index, out Signed576 x, out Signed576 y, out Signed576 z);
        Span<ulong> work = stackalloc ulong[2 * FamilyWords]; Span<int> signs = stackalloc int[2];
        ContactQuadratic scalar = At(work, signs, 0, FamilyWords);
        Signed576 radial = candidate.Feature == ConeSurfaceFeature.Rim ? Signed576.ExtendValue(geometry.Radius)
            : ExtendFamily(Signed192.Raw(candidate.Input.Height));
        scalar.Set(x); Scale(scalar, radial, a);
        scalar.Set(z); Scale(scalar, radial, b);
        if (candidate.Family == ConeSurfaceFamily.RotationCircle) { a.MultiplySign(-1); b.MultiplySign(-1); }
        scalar.Set(y);
        if (candidate.Feature == ConeSurfaceFeature.Rim)
        {
            TryGetRimPoint(geometry, candidate.RootOrdinal, out _, out _, out Signed320 offset, out _, out _);
            Scale(scalar, Signed576.ExtendValue(offset), c);
        }
        else { Scale(scalar, ExtendFamily(Signed192.Raw(candidate.Input.Radius)), c); c.MultiplySign(-1); }
    }

    private static void BuildFamilyCircleAnchors(SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        scoped ContactQuadratic u, scoped ContactQuadratic v, scoped ContactQuadratic d, scoped ref ConeSurfaceFamilyWitness witness)
    {
        Span<ulong> work = stackalloc ulong[12 * FamilyWords]; Span<int> signs = stackalloc int[12];
        ContactQuadratic oldD = At(work, signs, 0, FamilyWords), factor = At(work, signs, 1, FamilyWords);
        ContactQuadratic gap = At(work, signs, 2, FamilyWords), product = At(work, signs, 3, FamilyWords);
        ContactQuadratic temporary = At(work, signs, 4, FamilyWords), oldY = At(work, signs, 5, FamilyWords);
        witness[9].CopyTo(oldD); witness[4].CopyTo(oldY);
        Signed576 radius = ExtendFamily(Signed192.Raw(candidate.Input.Radius)), height = ExtendFamily(Signed192.Raw(candidate.Input.Height));
        if (candidate.Feature == ConeSurfaceFeature.Rim)
        {
            d.CopyTo(factor); factor.Add(factor);
            Scale(oldD, radius, gap); gap.Add(gap);
            Multiply(gap, u, witness.Root, witness[6]); Multiply(gap, v, witness.Root, witness[8]);
            Scale(oldD, height, gap); Multiply(gap, d, witness.Root, witness[7]); witness[7].MultiplySign(-1);
        }
        else
        {
            Signed576 metric = Signed576.ExtendValue(WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(Signed192.Raw(candidate.Input.Height), Signed192.Raw(candidate.Input.Height)),
                WideArithmetic.MultiplySigned192(Signed192.Raw(candidate.Input.Radius), Signed192.Raw(candidate.Input.Radius))));
            Scale(d, metric, factor);
            Scale(witness[11], Signed576.ExtendValue(geometry.Finite.ShapeFrame.Cap), gap); gap.Add(oldY, -1);
            Scale(gap, radius, temporary); Scale(temporary, height, gap);
            Multiply(gap, u, witness.Root, witness[6]); Multiply(gap, v, witness.Root, witness[8]);
            Scale(oldY, metric, gap); Scale(temporary, radius, product); gap.Add(product);
            Multiply(gap, d, witness.Root, witness[7]);
        }
        for (int axis = 0; axis < 3; axis++)
        {
            Multiply(witness[3 + axis], factor, witness.Root, product); product.CopyTo(witness[3 + axis]);
        }
        Multiply(oldD, factor, witness.Root, witness[9]);
    }
}
