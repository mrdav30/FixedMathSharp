//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>Finite cap, radial-side and circular-rim witnesses, including clipped rotational families.</content>
internal static partial class SegmentSlabSurfaceContact
{
    private static void AccumulateCurved(scoped Input input, scoped ref Selection selection)
    {
        Span<ulong> values = stackalloc ulong[StorageWords], root = stackalloc ulong[Words];
        Span<int> signs = stackalloc int[2 * Fields];
        var candidate = new Candidate(values, signs, root);
        if (!input.InteriorOnly)
        {
            candidate.Clear(); SetPoint(input, candidate);
            AccumulateCurvedPoint(input, candidate, ref selection);
            if (!input.Edge.IsZero)
            {
                candidate.Clear(); candidate.Parameter.Set(One); SetPoint(input, candidate);
                AccumulateCurvedPoint(input, candidate, ref selection);
            }
        }
        if (input.Edge.IsZero)
        {
            if (input.Length != Fixed64.Zero) AccumulateStraight(input, ref selection);
            return;
        }
        if (input.Edge.Y.IsZero && SetSectionParameter(input, candidate, projected: true))
            for (int cap = -1; cap <= 1; cap += 2)
            {
                SetPoint(input, candidate);
                if (SetCap(input, candidate, cap)) Keep(input, candidate, ref selection);
            }
        int start = input.Length == Fixed64.Zero ? 0 : -1;
        for (int region = start; region <= (input.Length == Fixed64.Zero ? 0 : 1); region += 2)
        {
            Signed320 center = region < 0 ? WideArithmetic.Negate(input.Core) : region > 0 ? input.Core : default;
            WideAxis3 offset = new(input.First.X, default, WideArithmetic.SubtractSigned320(input.First.Z, center));
            WideAxis3 planarEdge = new(input.Edge.X, default, input.Edge.Z);
            Signed576 square = planarEdge.SquaredLength;
            if (!square.IsZero)
            {
                candidate.Clear();
                candidate.Parameter.Set(WideArithmetic.SubtractSigned576(default, WideAxis3.Dot(offset, planarEdge)));
                candidate.ParameterDenominator.Set(square); SetPoint(input, candidate);
                AccumulateArcFoot(input, candidate, region, cap: 0, ref selection);
                if (input.Edge.Y.IsZero)
                    for (int cap = -1; cap <= 1; cap += 2)
                    {
                        SetPoint(input, candidate);
                        AccumulateArcFoot(input, candidate, region, cap, ref selection);
                    }
            }
            else if (SetSectionParameter(input, candidate))
            {
                SetPoint(input, candidate);
                AccumulateArcFoot(input, candidate, region, cap: 0, ref selection);
            }
        }
        if (input.Length != Fixed64.Zero) AccumulateStraight(input, ref selection);
        AccumulateMeridionalRims(input, ref selection);
        AccumulatePrincipalRims(input, ref selection);
    }

    private static void AccumulateCurvedPoint(scoped Input input, scoped Candidate candidate, scoped ref Selection selection)
    {
        for (int cap = -1; cap <= 1; cap += 2)
        { SetPoint(input, candidate); if (SetCap(input, candidate, cap)) Keep(input, candidate, ref selection); }
        int start = input.Length == Fixed64.Zero ? 0 : -1;
        for (int region = start; region <= (input.Length == Fixed64.Zero ? 0 : 1); region += 2)
            for (int cap = -1; cap <= 1; cap++)
            {
                SetPoint(input, candidate);
                AccumulateArcFoot(input, candidate, region, cap, ref selection);
            }
    }

    private static bool SetCap(scoped Input input, scoped Candidate candidate, int cap)
    {
        ContactQuadratic.Scale(candidate.Denominator, Extend(input.Cap), candidate.Boundary(1));
        candidate.Boundary(1).MultiplySign(cap);
        candidate.Boundary(1).CopyTo(candidate.Normal(1)); candidate.Normal(1).Add(candidate.Point(1), -1);
        if (candidate.Normal(1).Sign(candidate.Root) * cap < 0) return false;
        candidate.Normal(0).Clear(); candidate.Normal(1).Set(One); candidate.Normal(1).MultiplySign(cap); candidate.Normal(2).Clear();
        return true;
    }

    private static bool SetSectionParameter(scoped Input input, scoped Candidate candidate, bool projected = false)
    {
        candidate.Clear();
        if (!input.RequireSourceInside && !projected)
        {
            if (!CapsuleFiniteSectionFrame.TryGetCenteredAxisInterval(input.First.Y, input.Edge.Y, input.Cap,
                out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud)) return false;
            candidate.Parameter.Set(WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(ln, ud),
                WideArithmetic.MultiplySigned320(un, ld)));
            Signed576 denominator = WideArithmetic.MultiplySigned320(ld, ud);
            candidate.ParameterDenominator.Set(WideArithmetic.AddSigned576(denominator, denominator));
            return true;
        }
        bool admitted = input.RequireSourceInside
            ? input.Frame.TryGetFirstSegmentParameter(input.Segment, input.Offsets,
                candidate.Parameter, candidate.ParameterDenominator, candidate.Root)
            : input.Frame.TryGetFirstProjectedSegmentParameter(input.Segment, input.Offsets,
                candidate.Parameter, candidate.ParameterDenominator, candidate.Root);
        if (!admitted) return false;
        if (!input.InteriorOnly || candidate.Parameter.Sign(candidate.Root) != 0) return true;
        var reverse = new FixedSegment(input.Segment.End, input.Segment.Start);
        admitted = input.RequireSourceInside
            ? input.Frame.TryGetFirstSegmentParameter(reverse, input.Offsets,
                candidate.Parameter, candidate.ParameterDenominator, candidate.Root)
            : input.Frame.TryGetFirstProjectedSegmentParameter(reverse, input.Offsets,
                candidate.Parameter, candidate.ParameterDenominator, candidate.Root);
        // Reversal preserves the same nonempty closed section interval.
        System.Diagnostics.Debug.Assert(admitted);
        candidate.Parameter.MultiplySign(-1); candidate.Parameter.Add(candidate.ParameterDenominator);
        if (candidate.Parameter.Sign(candidate.Root) == 0) return false;
        Span<ulong> values = stackalloc ulong[2 * Words]; Span<int> signs = stackalloc int[2];
        var remainder = new ContactQuadratic(values, signs);
        candidate.ParameterDenominator.CopyTo(remainder); remainder.Add(candidate.Parameter, -1);
        if (remainder.Sign(candidate.Root) == 0)
        {
            candidate.Clear(); candidate.Parameter.Set(One);
            candidate.ParameterDenominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(2))));
        }
        return true;
    }

    private static void AccumulateArcFoot(scoped Input input, scoped Candidate source, int region, int cap, scoped ref Selection selection)
    {
        Span<ulong> saved = stackalloc ulong[StorageWords], savedRoot = stackalloc ulong[Words];
        Span<int> savedSigns = stackalloc int[2 * Fields];
        var candidate = new Candidate(saved, savedSigns, savedRoot);
        Span<ulong> values = stackalloc ulong[14 * Words]; Span<int> signs = stackalloc int[14];
        ContactQuadratic x = ContactQuadratic.At(values, signs, 0), z = ContactQuadratic.At(values, signs, 1),
            center = ContactQuadratic.At(values, signs, 2), square = ContactQuadratic.At(values, signs, 3),
            term = ContactQuadratic.At(values, signs, 4), coefficient = ContactQuadratic.At(values, signs, 5),
            remainder = ContactQuadratic.At(values, signs, 6);
        source.Point(0).CopyTo(x); source.Point(2).CopyTo(z);
        ContactQuadratic.Scale(source.Denominator, Extend(input.Core), center); center.MultiplySign(region);
        z.Add(center, -1);
        ContactQuadratic.Multiply(x, x, source.Root, square);
        ContactQuadratic.Multiply(z, z, source.Root, term); square.Add(term);
        // These feet use a rational segment parameter. A cap interval's
        // quadratic boundary parameter is handled separately, never folded
        // into a second independent radical field.
        System.Diagnostics.Debug.Assert(square.Signs[1] == 0);
        if (input.Radius == Fixed64.Zero || square.Signs[0] == 0)
        {
            AccumulateRadialFamily(input, source, region, cap, ref selection);
            return;
        }
        for (int branch = -1; branch <= 1; branch += 2)
        {
            source.CopyTo(candidate);
            square.Rational.CopyTo(candidate.Root);
            // Cancel the common sqrt(S) rather than retaining S in every
            // coordinate. A stationary rational foot can have 600-bit
            // numerators; this keeps its squared metric below 2400 bits.
            coefficient.Clear(); coefficient.Radical[0] = 1; coefficient.Signs[1] = 1;
            ScaleCoordinates(candidate, coefficient);
            for (int axis = 0; axis < 3; axis += 2)
            {
                ContactQuadratic planar = axis == 0 ? x : z;
                ContactQuadratic.Scale(planar, Extend(input.Radial), term);
                ContactQuadratic.Multiply(term, source.Denominator, source.Root, coefficient);
                ContactQuadratic q = candidate.Boundary(axis);
                coefficient.CopyTo(q, branch);
                if (axis == 2)
                {
                    ContactQuadratic.Scale(candidate.Denominator, Extend(input.Core), term); term.MultiplySign(region); q.Add(term);
                }
            }
            if (cap != 0 && !SetCap(input, candidate, cap)) continue;
            if (!AdmitsArc(input, candidate, region, remainder, term)) continue;
            // The matched displacement must be in the curved side/rim's
            // normal cone, rather than point from an outer arc inward.
            candidate.Boundary(0).CopyTo(term); term.Add(candidate.Point(0), -1);
            ContactQuadratic.Multiply(term, x, candidate.Root, remainder);
            candidate.Boundary(2).CopyTo(term); term.Add(candidate.Point(2), -1);
            ContactQuadratic.Multiply(term, z, candidate.Root, coefficient); remainder.Add(coefficient);
            if (remainder.Sign(candidate.Root) * branch < 0) continue;
            for (int axis = 0; axis < 3; axis++)
            {
                candidate.Boundary(axis).CopyTo(candidate.Normal(axis)); candidate.Normal(axis).Add(candidate.Point(axis), -1);
            }
            if (candidate.Normal(0).Sign(candidate.Root) == 0 && candidate.Normal(1).Sign(candidate.Root) == 0
                && candidate.Normal(2).Sign(candidate.Root) == 0)
                AccumulateTouchNormals(input, candidate, region, cap, ref selection);
            else Keep(input, candidate, ref selection);
        }
    }

    private static bool AdmitsArc(scoped Input input, scoped Candidate candidate, int region, scoped ContactQuadratic remainder, scoped ContactQuadratic term)
    {
        if (region == 0) return true;
        ContactQuadratic.Scale(candidate.Denominator, Extend(input.Core), term); term.MultiplySign(region);
        candidate.Boundary(2).CopyTo(remainder); remainder.Add(term, -1);
        return remainder.Sign(candidate.Root) * region >= 0;
    }

    private static void AccumulateRadialFamily(scoped Input input, scoped Candidate source, int region, int cap, scoped ref Selection selection)
    {
        // At a planar axis point every radial orientation has equal distance.
        // A nonempty halfspace-clipped circle either includes a principal
        // axis or an exact intersection with one of its linear walls.
        Span<ulong> values = stackalloc ulong[StorageWords], root = stackalloc ulong[Words];
        Span<int> signs = stackalloc int[2 * Fields];
        var candidate = new Candidate(values, signs, root);
        if (input.Radius == Fixed64.Zero)
        {
            // A collapsed radial stratum has a normal plane or half-plane;
            // its zero displacement cannot stand in for that free normal.
            source.CopyTo(candidate); candidate.Boundary(0).Clear();
            ContactQuadratic.Scale(source.Denominator, Extend(input.Core), candidate.Boundary(2));
            candidate.Boundary(2).MultiplySign(region);
            if (cap != 0 && !SetCap(input, candidate, cap)) return;
            Span<WideAxis3> cone = stackalloc WideAxis3[3];
            int count = 0;
            cone[count++] = Axis(0, cap == 0 ? 1 : -cap, 0);
            if (cap == 0) cone[count++] = Axis(0, -1, 0);
            if (region != 0) cone[count++] = Axis(0, 0, -region);
            KeepPlanar(input, candidate, cone[..count], ref selection);
            return;
        }
        Span<ulong> work = stackalloc ulong[16 * Words]; Span<int> workSigns = stackalloc int[16];
        ContactQuadratic a = ContactQuadratic.At(work, workSigns, 0), b = ContactQuadratic.At(work, workSigns, 1),
            c = ContactQuadratic.At(work, workSigns, 2), metric = ContactQuadratic.At(work, workSigns, 3),
            discriminant = ContactQuadratic.At(work, workSigns, 4), term = ContactQuadratic.At(work, workSigns, 5),
            radial = ContactQuadratic.At(work, workSigns, 6), axial = ContactQuadratic.At(work, workSigns, 7);
        ContactQuadratic.Scale(source.Denominator, Extend(input.Radial), radial);
        axial.Clear();
        if (cap != 0)
        {
            ContactQuadratic.Scale(source.Denominator, Extend(input.Cap), axial); axial.MultiplySign(cap);
            axial.Add(source.Point(1), -1);
            if (axial.Sign(source.Root) * cap < 0) return;
            // The positive metric scaling below preserves this cap-direction admission.
        }
        for (int axis = 0; axis < 3; axis += 2)
            for (int branch = -1; branch <= 1; branch += 2)
            {
                source.CopyTo(candidate);
                candidate.Boundary(0).Clear(); candidate.Boundary(2).Clear();
                radial.CopyTo(candidate.Boundary(axis), branch);
                ContactQuadratic.Scale(source.Denominator, Extend(input.Core), term); term.MultiplySign(region); candidate.Boundary(2).Add(term);
                if (cap != 0) SetCap(input, candidate, cap);
                SetFamilyNormal(candidate, axis, branch, cap);
                if (AdmitsArc(input, candidate, region, c, term)) Keep(input, candidate, ref selection);
            }
        for (int wall = 0; wall < input.Fan.Length + (region == 0 ? 0 : 1); wall++)
        {
            WideAxis3 constraint = wall < input.Fan.Length ? input.Fan[wall]
                : new(default, default, Signed320.ExtendValue(Signed192.Signed(-region)));
            a.Set(Extend(constraint.X)); b.Set(Extend(constraint.Z));
            ContactQuadratic.Multiply(a, a, source.Root, metric);
            ContactQuadratic.Multiply(b, b, source.Root, term); metric.Add(term);
            if (metric.Signs[0] == 0) continue;
            ContactQuadratic.Scale(axial, Extend(constraint.Y), c); c.MultiplySign(-1);
            ContactQuadratic.Multiply(radial, radial, source.Root, discriminant);
            ContactQuadratic.Multiply(discriminant, metric, source.Root, discriminant);
            ContactQuadratic.Multiply(c, c, source.Root, term); discriminant.Add(term, -1);
            if (discriminant.Signs[0] < 0) continue;
            for (int branch = -1; branch <= 1; branch += 2)
            {
                source.CopyTo(candidate);
                discriminant.Rational.CopyTo(candidate.Root);
                ScaleCoordinates(candidate, metric);
                ContactQuadratic.Multiply(c, a, source.Root, candidate.Boundary(0));
                b.Rational.CopyTo(candidate.Boundary(0).Radical); candidate.Boundary(0).Signs[1] = branch * b.Signs[0];
                ContactQuadratic.Multiply(c, b, source.Root, candidate.Boundary(2));
                a.Rational.CopyTo(candidate.Boundary(2).Radical); candidate.Boundary(2).Signs[1] = -branch * a.Signs[0];
                ContactQuadratic.Scale(candidate.Denominator, Extend(input.Core), term); term.MultiplySign(region); candidate.Boundary(2).Add(term);
                if (cap != 0) SetCap(input, candidate, cap);
                candidate.Boundary(0).CopyTo(candidate.Normal(0));
                axial.CopyTo(candidate.Normal(1));
                candidate.Boundary(2).CopyTo(candidate.Normal(2));
                if (AdmitsArc(input, candidate, region, c, term)) Keep(input, candidate, ref selection);
            }
        }
    }

    private static void SetFamilyNormal(scoped Candidate candidate, int axis, int branch, int cap)
    {
        for (int component = 0; component < 3; component++) candidate.Normal(component).Clear();
        candidate.Normal(axis).Set(One); candidate.Normal(axis).MultiplySign(branch);
        if (cap != 0) candidate.Normal(1).Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(cap))));
    }

    private static void AccumulateTouchNormals(scoped Input input, scoped Candidate candidate, int region, int cap, scoped ref Selection selection)
    {
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic radial = ContactQuadratic.At(values, signs, 0), up = ContactQuadratic.At(values, signs, 1),
            term = ContactQuadratic.At(values, signs, 2), projection = ContactQuadratic.At(values, signs, 3);
        candidate.Boundary(0).CopyTo(candidate.Normal(0)); candidate.Normal(1).Clear();
        ContactQuadratic.Scale(candidate.Denominator, Extend(input.Core), term); term.MultiplySign(region);
        candidate.Boundary(2).CopyTo(candidate.Normal(2)); candidate.Normal(2).Add(term, -1);
        Keep(input, candidate, ref selection);
        if (cap == 0) return;
        for (int wall = 0; wall <= input.Fan.Length; wall++)
        {
            // The normal cone is the nonnegative span of this radial normal
            // and the cap normal. Its clipped endpoints are generators or
            // one exact fan-wall crossing; no angular sampling is needed.
            WideAxis3 constraint = wall < input.Fan.Length ? input.Fan[wall] : default;
            candidate.Boundary(0).CopyTo(candidate.Normal(0)); candidate.Normal(1).Clear();
            candidate.Boundary(2).CopyTo(candidate.Normal(2)); candidate.Normal(2).Add(term, -1);
            Dot(candidate, constraint, radial, projection);
            up.Set(Extend(constraint.Y)); up.MultiplySign(cap);
            if (wall < input.Fan.Length && radial.Sign(candidate.Root) * up.Sign(candidate.Root) >= 0) continue;
            for (int axis = 0; axis < 3; axis += 2)
            {
                if (wall == input.Fan.Length) candidate.Normal(axis).Clear();
                else
                {
                    up.MultiplySign(up.Sign(candidate.Root));
                    ContactQuadratic.Multiply(candidate.Normal(axis), up, candidate.Root, projection); projection.CopyTo(candidate.Normal(axis));
                }
            }
            if (wall == input.Fan.Length) candidate.Normal(1).Set(One);
            else radial.CopyTo(candidate.Normal(1), radial.Sign(candidate.Root));
            candidate.Normal(1).MultiplySign(cap);
            Keep(input, candidate, ref selection);
        }
    }

    private static void AccumulateMeridionalRims(scoped Input input, scoped ref Selection selection)
    {
        if (input.Edge.Y.IsZero) return;
        Signed576 radialSquare = CircularRimContactAlgebra.RadialDot(input.Edge, input.Edge);
        if (radialSquare.IsZero) return;
        Span<ulong> values = stackalloc ulong[StorageWords], root = stackalloc ulong[Words];
        Span<int> signs = stackalloc int[2 * Fields]; var candidate = new Candidate(values, signs, root);
        Span<ulong> work = stackalloc ulong[12 * Words]; Span<int> workSigns = stackalloc int[12];
        ContactQuadratic qx = ContactQuadratic.At(work, workSigns, 0), qy = ContactQuadratic.At(work, workSigns, 1),
            qz = ContactQuadratic.At(work, workSigns, 2), a = ContactQuadratic.At(work, workSigns, 3),
            b = ContactQuadratic.At(work, workSigns, 4), term = ContactQuadratic.At(work, workSigns, 5);
        int firstRegion = input.Length == Fixed64.Zero ? 0 : -1;
        for (int region = firstRegion; region <= (input.Length == Fixed64.Zero ? 0 : 1); region += 2)
            for (int cap = -1; cap <= 1; cap += 2)
                for (int branch = -1; branch <= 1; branch += 2)
                {
                    candidate.Clear(); Import(radialSquare, candidate.Root);
                    a.Set(Extend(input.Radial)); b.Set(Extend(input.Edge.X)); ContactQuadratic.Multiply(a, b, root, term);
                    qx.Clear(); term.Rational.CopyTo(qx.Radical); qx.Signs[1] = branch * term.Signs[0];
                    b.Set(Extend(input.Edge.Z)); ContactQuadratic.Multiply(a, b, root, term);
                    qz.Clear(); term.Rational.CopyTo(qz.Radical); qz.Signs[1] = branch * term.Signs[0];
                    a.Set(radialSquare); ContactQuadratic.Scale(a, Extend(input.Core), term); term.MultiplySign(region); qz.Add(term);
                    ContactQuadratic.Scale(a, Extend(input.Cap), qy); qy.MultiplySign(cap);
                    candidate.Parameter.Clear();
                    for (int axis = 0; axis < 3; axis++)
                    {
                        ContactQuadratic q = axis == 0 ? qx : axis == 1 ? qy : qz;
                        ContactQuadratic.Scale(a, Extend(Component(input.First, axis)), term);
                        q.CopyTo(b); b.Add(term, -1);
                        ContactQuadratic.Scale(b, Extend(Component(input.Edge, axis)), term); candidate.Parameter.Add(term);
                    }
                    ContactQuadratic.Scale(a, input.Edge.SquaredLength, candidate.ParameterDenominator);
                    SetPoint(input, candidate);
                    ContactQuadratic.Scale(qx, input.Edge.SquaredLength, candidate.Boundary(0));
                    ContactQuadratic.Scale(qy, input.Edge.SquaredLength, candidate.Boundary(1));
                    ContactQuadratic.Scale(qz, input.Edge.SquaredLength, candidate.Boundary(2));
                    KeepRim(input, candidate, region, cap, ref selection);
                }
    }

    private static void AccumulatePrincipalRims(scoped Input input, scoped ref Selection selection)
    {
        if (input.Edge.Y.IsZero || input.Edge.IsZero) return;
        // The quartic charts isolate (0,1]; their exact principal-axis ends
        // are separate rational feet, rather than angular samples.
        Span<ulong> values = stackalloc ulong[StorageWords], root = stackalloc ulong[Words];
        Span<int> signs = stackalloc int[2 * Fields]; var candidate = new Candidate(values, signs, root);
        int firstRegion = input.Length == Fixed64.Zero ? 0 : -1;
        for (int region = firstRegion; region <= (input.Length == Fixed64.Zero ? 0 : 1); region += 2)
            for (int cap = -1; cap <= 1; cap += 2)
                for (int axis = 0; axis < 3; axis += 2)
                    for (int branch = -1; branch <= 1; branch += 2)
                    {
                        Signed320 x = axis == 0 ? branch < 0 ? WideArithmetic.Negate(input.Radial) : input.Radial : default;
                        Signed320 z = region < 0 ? WideArithmetic.Negate(input.Core) : region > 0 ? input.Core : default;
                        if (axis == 2) z = WideArithmetic.AddSigned320(z, branch < 0 ? WideArithmetic.Negate(input.Radial) : input.Radial);
                        WideAxis3 q = new(x, cap < 0 ? WideArithmetic.Negate(input.Cap) : input.Cap, z);
                        candidate.Clear(); candidate.Parameter.Set(WideAxis3.Dot(TriangleCircularGeometry.Subtract(q, input.First), input.Edge));
                        candidate.ParameterDenominator.Set(input.Edge.SquaredLength); SetPoint(input, candidate);
                        for (int component = 0; component < 3; component++)
                            ContactQuadratic.Scale(candidate.Denominator, Extend(Component(q, component)), candidate.Boundary(component));
                        KeepRim(input, candidate, region, cap, ref selection);
                    }
    }

    private static void KeepRim(scoped Input input, scoped Candidate candidate, int region, int cap, scoped ref Selection selection)
    {
        Span<ulong> values = stackalloc ulong[10 * Words]; Span<int> signs = stackalloc int[10];
        ContactQuadratic radial = ContactQuadratic.At(values, signs, 0), term = ContactQuadratic.At(values, signs, 1),
            cross = ContactQuadratic.At(values, signs, 2), dot = ContactQuadratic.At(values, signs, 3),
            remainder = ContactQuadratic.At(values, signs, 4);
        if (!AdmitsArc(input, candidate, region, remainder, term)) return;
        for (int axis = 0; axis < 3; axis++)
        { candidate.Boundary(axis).CopyTo(candidate.Normal(axis)); candidate.Normal(axis).Add(candidate.Point(axis), -1); }
        if (candidate.Normal(1).Sign(candidate.Root) * cap < 0) return;
        ContactQuadratic.Scale(candidate.Denominator, Extend(input.Core), term); term.MultiplySign(region);
        candidate.Boundary(2).CopyTo(radial); radial.Add(term, -1);
        ContactQuadratic.Multiply(candidate.Normal(0), radial, candidate.Root, cross);
        ContactQuadratic.Multiply(candidate.Normal(2), candidate.Boundary(0), candidate.Root, term); cross.Add(term, -1);
        if (cross.Sign(candidate.Root) != 0) return;
        ContactQuadratic.Multiply(candidate.Normal(0), candidate.Boundary(0), candidate.Root, dot);
        ContactQuadratic.Multiply(candidate.Normal(2), radial, candidate.Root, term); dot.Add(term);
        if (dot.Sign(candidate.Root) < 0) return;
        if (candidate.Normal(0).Sign(candidate.Root) == 0 && candidate.Normal(1).Sign(candidate.Root) == 0
            && candidate.Normal(2).Sign(candidate.Root) == 0)
            AccumulateTouchNormals(input, candidate, region, cap, ref selection);
        else Keep(input, candidate, ref selection);
    }
}
