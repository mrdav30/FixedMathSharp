//=======================================================================
// SegmentConeSurfaceCandidates.FamilyNormalCones.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Rational rim normal intervals and finite-axis polyhedral normal cones.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    private static bool BuildFamilyRimBound(SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        scoped ReadOnlySpan<WideAxis3> fan, ConeSurfaceFamilyEvent item, scoped ref ConeSurfaceFamilyWitness witness)
    {
        Span<ulong> work = stackalloc ulong[20 * FamilyWords]; Span<int> signs = stackalloc int[20];
        ContactQuadratic sx = At(work, signs, 0, FamilyWords), sy = At(work, signs, 1, FamilyWords), sz = At(work, signs, 2, FamilyWords);
        ContactQuadratic cap = At(work, signs, 3, FamilyWords), n = At(work, signs, 4, FamilyWords), d = At(work, signs, 5, FamilyWords);
        ContactQuadratic complement = At(work, signs, 7, FamilyWords);
        ContactQuadratic product = At(work, signs, 8, FamilyWords), term = At(work, signs, 9, FamilyWords);
        // A zero-depth rim point p has rational coordinates and |p_rad|=R.
        // (-H*p.x,-R²,-H*p.z) and (0,R²,0) are exact generators;
        // no square root or rounded radial direction is needed here.
        Signed576 height = ExtendFamily(Signed192.Raw(candidate.Input.Height));
        Signed576 radiusSquared = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(
            Signed192.Raw(candidate.Input.Radius), Signed192.Raw(candidate.Input.Radius)));
        Scale(witness[3], height, sx); sx.MultiplySign(-1);
        Scale(witness[5], height, sz); sz.MultiplySign(-1);
        Scale(witness[9], radiusSquared, cap); cap.CopyTo(sy, -1);
        d.Set(ExtendFamily(Signed192.Signed(1)));
        n.Set(ExtendFamily(Signed192.Signed(item.First == -1 ? 1 : 0)));
        if (item.First >= 0)
        {
            FamilyConstraint(geometry, fan, item, witness, item.First, out Signed576 x, out Signed576 y, out Signed576 z);
            Scale(sx, x, n); Scale(sy, y, term); n.Add(term); Scale(sz, z, term); n.Add(term);
            Scale(cap, y, d); d.Add(n, -1); n.MultiplySign(-1);
            int direction = d.Sign(witness.Root);
            if (direction == 0) return false;
            n.MultiplySign(direction); d.MultiplySign(direction);
            d.CopyTo(complement); complement.Add(n, -1);
            if (n.Sign(witness.Root) < 0 || complement.Sign(witness.Root) < 0) return false;
        }
        d.CopyTo(complement); complement.Add(n, -1);
        Multiply(sx, complement, witness.Root, witness[0]);
        Multiply(sy, complement, witness.Root, witness[1]);
        Multiply(cap, n, witness.Root, product); witness[1].Add(product);
        Multiply(sz, complement, witness.Root, witness[2]);
        for (int axis = 0; axis < 3; axis++) witness[3 + axis].CopyTo(witness[6 + axis]);
        return true;
    }

    private static bool BuildFamilyAxis(in SegmentConeSurfaceGeometry geometry, scoped ReadOnlySpan<WideAxis3> fan,
        ConeSurfaceFamilyEvent item, scoped ref ConeSurfaceFamilyWitness witness)
    {
        if (item.Kind == ConeSurfaceFamilyEventKind.Axis)
            witness[item.First].Set(ExtendFamily(Signed192.Signed(item.Branch)));
        else
        {
            FamilyConstraint(geometry, fan, item, witness, item.First, out Signed576 ax, out Signed576 ay, out Signed576 az);
            Signed576 bx, by, bz;
            if (item.Kind == ConeSurfaceFamilyEventKind.AxisPlane)
            {
                Signed576 unit = ExtendFamily(Signed192.Signed(1));
                bx = item.Second == 0 ? unit : default; by = item.Second == 1 ? unit : default; bz = item.Second == 2 ? unit : default;
            }
            else FamilyConstraint(geometry, fan, item, witness, item.Second, out bx, out by, out bz);
            Span<ulong> work = stackalloc ulong[6 * FamilyWords]; Span<int> signs = stackalloc int[6];
            ContactQuadratic left = At(work, signs, 0, FamilyWords), right = At(work, signs, 1, FamilyWords), product = At(work, signs, 2, FamilyWords);
            for (int axis = 0; axis < 3; axis++)
            {
                left.Set(axis == 0 ? ay : axis == 1 ? az : ax);
                right.Set(axis == 0 ? bz : axis == 1 ? bx : by);
                Multiply(left, right, witness.Root, witness[axis]);
                left.Set(axis == 0 ? az : axis == 1 ? ax : ay);
                right.Set(axis == 0 ? by : axis == 1 ? bz : bx);
                Multiply(left, right, witness.Root, product); witness[axis].Add(product, -1); witness[axis].MultiplySign(item.Branch);
            }
        }
        for (int axis = 0; axis < 3; axis++) witness[3 + axis].CopyTo(witness[6 + axis]);
        return true;
    }
}
