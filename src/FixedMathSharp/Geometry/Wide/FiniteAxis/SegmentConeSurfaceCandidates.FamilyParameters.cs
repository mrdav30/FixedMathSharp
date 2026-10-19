//=======================================================================
// SegmentConeSurfaceCandidates.FamilyParameters.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.ContactQuadratic;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>Exact segment strata and intrinsic family normal constraints.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    private static bool BuildFamilyEvent(in SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        scoped ReadOnlySpan<WideAxis3> fan, ConeSurfaceFamilyEvent item, scoped ref ConeSurfaceFamilyWitness witness,
        in RationalSegmentInterval clip = default)
    {
        int FamilyWords = witness.FieldWords;
        if (!BuildFamilyParameter(candidate, geometry, item, ref witness, clip)) return false;
        Span<ulong> work = stackalloc ulong[4 * FamilyWords]; Span<int> signs = stackalloc int[4];
        ContactQuadratic term = At(work, signs, 0, FamilyWords);
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(witness[11], Signed832.ExtendValue(Component(geometry.A, axis)), witness[3 + axis]);
            Scale(witness[10], Signed832.ExtendValue(Component(geometry.Edge, axis)), term); witness[3 + axis].Add(term);
        }
        Scale(witness[11], ExtendFamily(geometry.RawScale), witness[9]);
        if (item.Kind == ConeSurfaceFamilyEventKind.Interval)
            return BuildFamilyInterval(candidate, geometry, ref witness);
        if (item.Kind == ConeSurfaceFamilyEventKind.RimBound)
            return BuildFamilyRimBound(candidate, geometry, fan, item, ref witness);
        if (item.Kind >= ConeSurfaceFamilyEventKind.Axis)
            return BuildFamilyAxis(geometry, fan, item, ref witness);
        return BuildFamilyCircle(candidate, geometry, fan, item, ref witness);
    }

    private static bool BuildFamilyParameter(in SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        ConeSurfaceFamilyEvent item, scoped ref ConeSurfaceFamilyWitness witness, in RationalSegmentInterval clip = default)
    {

        if (candidate.Family == ConeSurfaceFamily.SegmentInterval)
            return BuildFamilyIntervalParameter(candidate, geometry, item, ref witness, clip);
        Signed832 numerator, denominator;
        if (candidate.Feature == ConeSurfaceFeature.Apex)
            TryGetApexParameter(geometry, candidate.RootOrdinal, out numerator, out denominator);
        else if (candidate.Feature == ConeSurfaceFeature.Side && candidate.Input.Radius == Fixed64.Zero)
            GetAxisParameter(geometry, out numerator, out denominator);
        else if (candidate.Feature == ConeSurfaceFeature.Side)
        {
            numerator = ExtendFamily(Signed192.Signed(candidate.RootOrdinal));
            denominator = ExtendFamily(Signed192.Signed(1));
        }
        else if (candidate.RootOrdinal <= -3)
        {
            // A zero-depth meridional rim point lies in the base plane.
            // Its nonhorizontal segment therefore has the rational parameter
            // -(A.y+cap)/E.y even when its original stationary chart used a root.
            numerator = Signed832.ExtendValue(WideArithmetic.Negate(geometry.BaseOffset.Y));
            denominator = Signed832.ExtendValue(geometry.Edge.Y);
            if (denominator.Sign < 0)
            {
                numerator = WideArithmetic.SubtractSigned832(default, numerator);
                denominator = WideArithmetic.SubtractSigned832(default, denominator);
            }
        }
        else
            TryGetRimPoint(geometry, candidate.RootOrdinal, out _, out _, out _, out numerator, out denominator);
        witness[10].Set(numerator); witness[11].Set(denominator);
        return (!clip.IsClipped || clip.Contains(witness[10], witness[11], witness.Root))
            && FamilyParameterLocation(witness, clip) == item.PointLocation;
    }

    private static bool BuildFamilyIntervalParameter(in SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        ConeSurfaceFamilyEvent item, scoped ref ConeSurfaceFamilyWitness witness, in RationalSegmentInterval clip)
    {
        int FamilyWords = witness.FieldWords;
        Span<ulong> work = stackalloc ulong[12 * FamilyWords], lowerRoot = stackalloc ulong[FamilyWords], upperRoot = stackalloc ulong[FamilyWords];
        Span<int> signs = stackalloc int[12];
        ContactQuadratic low = At(work, signs, 0, FamilyWords), lowD = At(work, signs, 1, FamilyWords);
        ContactQuadratic high = At(work, signs, 2, FamilyWords), highD = At(work, signs, 3, FamilyWords);
        ContactQuadratic product = At(work, signs, 4, FamilyWords);
        GetFamilyIntervalBound(candidate, geometry, false, low, lowD, lowerRoot, clip);
        GetFamilyIntervalBound(candidate, geometry, true, high, highD, upperRoot, clip);
        if (clip.IsClipped && CompareRatios(low, lowD, lowerRoot, high, highD, upperRoot) > 0) return false;
        (IsZero(lowerRoot) ? upperRoot : lowerRoot).CopyTo(witness.Root);
        ContactQuadratic n = witness[10], d = witness[11];
        if (item.Parameter == 0) { low.CopyTo(n); lowD.CopyTo(d); }
        else if (item.Parameter == 2) { high.CopyTo(n); highD.CopyTo(d); }
        else if (item.Parameter == 1)
        {
            // Both nonlinear section endpoints use the same discriminant;
            // axial/side clipping only replaces an endpoint by a rational.
            Multiply(low, highD, witness.Root, n); Multiply(high, lowD, witness.Root, product); n.Add(product);
            Multiply(lowD, highD, witness.Root, d); d.Add(d);
        }
        else
        {
            if (clip.IsClipped)
            {
                n.Set(item.Parameter == 3 ? clip.WideLowerNumerator : clip.WideUpperNumerator);
                d.Set(item.Parameter == 3 ? clip.WideLowerDenominator : clip.WideUpperDenominator);
            }
            else
            {
                n.Set(ExtendFamily(Signed192.Signed(item.Parameter == 3 ? 0 : 1)));
                d.Set(ExtendFamily(Signed192.Signed(1)));
            }
        }
        if (CompareRatios(n, d, witness.Root, low, lowD, lowerRoot) < 0
            || CompareRatios(n, d, witness.Root, high, highD, upperRoot) > 0)
            return false;
        return FamilyParameterLocation(witness, clip) == item.PointLocation;
    }

    private static void GetFamilyIntervalBound(in SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        bool upper, ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root, in RationalSegmentInterval clip)
    {
        int Words = geometry.Words;
        int FamilyWords = clip.HasWideBounds ? 3 * SegmentConeSurfaceCandidates.FamilyWords : geometry.FamilyWords;
        if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Input.Radius == Fixed64.Zero)
        {
            bool exists = FiniteSectionPoint.TryGetSegmentParameter(candidate.Input.Segment, geometry.Finite, upper, numerator, denominator, root);
            System.Diagnostics.Debug.Assert(exists);
        }
        else
        {
            Span<ulong> work = stackalloc ulong[24 * Words], oldRoot = stackalloc ulong[Words]; Span<int> signs = stackalloc int[24];
            BuildSideNormal(geometry, -1, candidate.Chart, work, signs, oldRoot);
            TryBuildSideFoot(geometry, -1, work, signs, oldRoot, out _);
            FiniteSectionPoint.TryGetSegmentParameter(candidate.Input.Segment, geometry.Finite, upper, At(work, signs, 5, Words), At(work, signs, 6, Words), oldRoot);
            ClipSideEndpoint(geometry, upper, work, signs, oldRoot);
            At(work, signs, 5, Words).CopyTo(numerator); At(work, signs, 6, Words).CopyTo(denominator);
            root.Clear(); oldRoot.CopyTo(root);
        }
        if (!clip.IsClipped) return;
        Span<ulong> bounds = stackalloc ulong[4 * FamilyWords]; Span<int> boundSigns = stackalloc int[4];
        ContactQuadratic bound = At(bounds, boundSigns, 0, FamilyWords), boundD = At(bounds, boundSigns, 1, FamilyWords);
        bound.Set(upper ? clip.WideUpperNumerator : clip.WideLowerNumerator);
        boundD.Set(upper ? clip.WideUpperDenominator : clip.WideLowerDenominator);
        int comparison = CompareRatios(bound, boundD, ReadOnlySpan<ulong>.Empty, numerator, denominator, root);
        if (upper ? comparison < 0 : comparison > 0)
        {
            bound.CopyTo(numerator); boundD.CopyTo(denominator); root.Clear();
        }
    }

    private static ConeSurfacePointLocation FamilyParameterLocation(scoped ConeSurfaceFamilyWitness witness,
        in RationalSegmentInterval clip = default)
    {
        int FamilyWords = witness.FieldWords;
        if (clip.IsClipped)
        {
            Span<ulong> bounds = stackalloc ulong[4 * FamilyWords]; Span<int> clipSigns = stackalloc int[4];
            ContactQuadratic bound = At(bounds, clipSigns, 0, FamilyWords), boundD = At(bounds, clipSigns, 1, FamilyWords);
            bound.Set(clip.WideLowerNumerator); boundD.Set(clip.WideLowerDenominator);
            if (CompareRatios(witness[10], witness[11], witness.Root, bound, boundD, ReadOnlySpan<ulong>.Empty) == 0)
                return ConeSurfacePointLocation.Start;
            bound.Set(clip.WideUpperNumerator); boundD.Set(clip.WideUpperDenominator);
            if (CompareRatios(witness[10], witness[11], witness.Root, bound, boundD, ReadOnlySpan<ulong>.Empty) == 0)
                return ConeSurfacePointLocation.End;
        }
        if (witness[10].Sign(witness.Root) == 0) return ConeSurfacePointLocation.Start;
        Span<ulong> data = stackalloc ulong[2 * FamilyWords]; Span<int> signs = stackalloc int[2];
        ContactQuadratic margin = At(data, signs, 0, FamilyWords);
        witness[11].CopyTo(margin); margin.Add(witness[10], -1);
        return margin.Sign(witness.Root) == 0 ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Interior;
    }

    internal static bool IsInInterval(in SegmentConeSurfaceCandidate candidate, in RationalSegmentInterval clip, bool interiorOnly)
    {
        int Words = candidate.Input.Words;
        if (candidate.Family == ConeSurfaceFamily.SegmentInterval)
            throw new ArgumentException("An interval family requires clipping before family event selection.", nameof(candidate));
        if (!clip.IsClipped) return !interiorOnly || candidate.IsSegmentInterior;
        if (candidate.PointLocation == ConeSurfacePointLocation.Start) return !interiorOnly && clip.WideLowerNumerator.IsZero;
        if (candidate.PointLocation == ConeSurfacePointLocation.End)
            return !interiorOnly && clip.WideUpperNumerator.Equals(clip.WideUpperDenominator);
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        if (candidate.Feature == ConeSurfaceFeature.Rim && candidate.Chart >= 0)
            return IsRimParameterInInterval(geometry, candidate, clip, interiorOnly);
        Span<ulong> values = stackalloc ulong[24 * Words], root = stackalloc ulong[Words];
        Span<int> signs = stackalloc int[24];
        ContactQuadratic n = At(values, signs, 5, Words), d = At(values, signs, 6, Words);
        root.Clear();
        if (candidate.Feature == ConeSurfaceFeature.Side && candidate.Input.Radius != Fixed64.Zero)
        {
            bool normal = BuildSideNormal(geometry, candidate.RootOrdinal, candidate.Chart, values, signs, root);
            bool foot = TryBuildSideFoot(geometry, candidate.RootOrdinal, values, signs, root, out _);
            System.Diagnostics.Debug.Assert(normal && foot);
        }
        else if (candidate.Feature == ConeSurfaceFeature.Rim && candidate.RootOrdinal <= -3)
        {
            bool admitted = BuildMeridionalRim(geometry, candidate.RootOrdinal, candidate.Chart == -2 ? 1 : -1, values, signs, root);
            System.Diagnostics.Debug.Assert(admitted);
        }
        else
        {
            Signed832 numerator, denominator;
            if (candidate.Feature == ConeSurfaceFeature.Apex)
                TryGetApexParameter(geometry, candidate.RootOrdinal, out numerator, out denominator);
            else if (candidate.Feature == ConeSurfaceFeature.Side)
                GetAxisParameter(geometry, out numerator, out denominator);
            else
                TryGetRimPoint(geometry, candidate.RootOrdinal, out _, out _, out _, out numerator, out denominator);
            n.Set(numerator); d.Set(denominator);
        }
        if (!clip.Contains(n, d, root)) return false;
        if (!interiorOnly) return true;
        ContactQuadratic bound = At(values, signs, 0, Words), boundD = At(values, signs, 1, Words);
        bound.Set(clip.WideLowerNumerator); boundD.Set(clip.WideLowerDenominator);
        if (CompareRatios(n, d, root, bound, boundD, ReadOnlySpan<ulong>.Empty) == 0) return false;
        bound.Set(clip.WideUpperNumerator); boundD.Set(clip.WideUpperDenominator);
        return CompareRatios(n, d, root, bound, boundD, ReadOnlySpan<ulong>.Empty) != 0;
    }

    private static void FamilyConstraint(in SegmentConeSurfaceGeometry geometry, scoped ReadOnlySpan<WideAxis3> fan,
        ConeSurfaceFamilyEvent item, scoped ConeSurfaceFamilyWitness witness, int index,
        out Signed832 x, out Signed832 y, out Signed832 z)
    {
        int FamilyWords = witness.FieldWords;
        x = y = z = default;
        if (index < fan.Length)
        {
            WideDirection3d covector = TransformNormalCovector(geometry, fan[index]);
            x = Signed832.ExtendValue(covector.X); y = Signed832.ExtendValue(covector.Y); z = Signed832.ExtendValue(covector.Z);
            return;
        }
        int intrinsic = index - fan.Length;
        if (intrinsic < 2)
        {
            if (intrinsic == 1 && item.PointLocation != ConeSurfacePointLocation.Interior) return;
            int sign = intrinsic == 1 || item.PointLocation == ConeSurfacePointLocation.End ? -1 : 1;
            x = Signed832.ExtendValue(sign < 0 ? WideArithmetic.Negate(geometry.Edge.X) : geometry.Edge.X);
            y = Signed832.ExtendValue(sign < 0 ? WideArithmetic.Negate(geometry.Edge.Y) : geometry.Edge.Y);
            z = Signed832.ExtendValue(sign < 0 ? WideArithmetic.Negate(geometry.Edge.Z) : geometry.Edge.Z);
            return;
        }
        if (geometry.Input.Radius != Fixed64.Zero || item.Kind < ConeSurfaceFamilyEventKind.Axis) return;
        Span<ulong> work = stackalloc ulong[4 * FamilyWords]; Span<int> signs = stackalloc int[4];
        ContactQuadratic twiceY = At(work, signs, 0, FamilyWords), cap = At(work, signs, 1, FamilyWords);
        witness[4].CopyTo(twiceY); twiceY.Add(twiceY);
        Scale(witness[9], ExtendFamily(Signed192.Raw(geometry.Input.Height)), cap);
        cap.Add(twiceY, -1); bool apex = cap.Sign(witness.Root) == 0;
        Scale(witness[9], ExtendFamily(Signed192.Raw(geometry.Input.Height)), cap);
        cap.Add(twiceY); bool bottom = cap.Sign(witness.Root) == 0;
        if (intrinsic == 3 && (apex || bottom)) return;
        y = ExtendFamily(Signed192.Signed(intrinsic == 3 || bottom ? -1 : 1));
    }

    private static bool BuildFamilyInterval(in SegmentConeSurfaceCandidate candidate, in SegmentConeSurfaceGeometry geometry,
        scoped ref ConeSurfaceFamilyWitness witness)
    {
        int Words = geometry.Words;
        int FamilyWords = witness.FieldWords;
        if (candidate.Feature == ConeSurfaceFeature.Base)
        {
            witness[1].Set(ExtendFamily(Signed192.Signed(1)));
            witness[3].CopyTo(witness[6]); witness[5].CopyTo(witness[8]);
            Scale(witness[11], Signed832.ExtendValue(WideArithmetic.Negate(geometry.Cap)), witness[7]);
            return true;
        }
        Span<ulong> old = stackalloc ulong[24 * Words], oldRoot = stackalloc ulong[Words]; Span<int> oldSigns = stackalloc int[24];
        BuildSideNormal(geometry, -1, candidate.Chart, old, oldSigns, oldRoot);
        TryBuildSideFoot(geometry, -1, old, oldSigns, oldRoot, out _);
        Span<ulong> work = stackalloc ulong[6 * FamilyWords]; Span<int> signs = stackalloc int[6];
        ContactQuadratic product = At(work, signs, 0, FamilyWords), displacement = At(work, signs, 1, FamilyWords);
        ContactQuadratic temporary = At(work, signs, 2, FamilyWords);
        for (int axis = 0; axis < 3; axis++)
        {
            At(old, oldSigns, axis, Words).CopyTo(witness[axis]);
            Multiply(witness[3 + axis], At(old, oldSigns, 7, Words), witness.Root, product); product.CopyTo(witness[3 + axis]);
            Multiply(At(old, oldSigns, 8, Words), witness[axis], witness.Root, temporary);
            Multiply(temporary, witness[11], witness.Root, displacement);
            product.Add(displacement, -1); product.CopyTo(witness[6 + axis]);
        }
        Multiply(witness[9], At(old, oldSigns, 7, Words), witness.Root, product); product.CopyTo(witness[9]);
        return true;
    }
}
