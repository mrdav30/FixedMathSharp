//=======================================================================
// CapsulePlaneRayEvents.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact finite capsule-plane admission and boundary-ray critical events.</summary>
internal static class CapsulePlaneRayEvents
{
    private const int Words = CapsulePlaneRaySelection.FieldWords;
    private static ContactQuadratic Field(Span<ulong> values, Span<int> signs, int index, int words = Words) =>
        ContactQuadratic.At(values, signs, index, words);

    internal static bool VisitEvents(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        int orientation, CapsulePlaneRayEventVisitor visitor)
    {
        if (visitor == null) throw new ArgumentNullException(nameof(visitor));
        // Visitor dispatch never retains a global maximum; the caller owns
        // finite-region ranking, so no unused coefficient banks are needed.
        CapsulePlaneRaySelection maximum = default;
        return Dispatch(source, frame, orientation, ref maximum, visitor, false, default, default);
    }

    internal static bool Accumulate(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        int orientation, ref CapsulePlaneRaySelection maximum) =>
        Dispatch(source, frame, orientation, ref maximum, null, false, default, default);

    internal static bool VisitEvents(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        int orientation, in RationalSegmentInterval interval, CapsulePlaneRayEventVisitor visitor)
    {
        if (visitor == null) throw new ArgumentNullException(nameof(visitor));
        CapsulePlaneRaySelection maximum = default;
        return Dispatch(source, frame, orientation, ref maximum, visitor, false, default, interval);
    }

    internal static bool Reconstruct(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        CapsulePlaneRayEvent descriptor, int orientation, ref CapsulePlaneRaySelection selection) =>
        Dispatch(source, frame, orientation, ref selection, null, true, descriptor, default);

    internal static bool Reconstruct(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        CapsulePlaneRayEvent descriptor, int orientation, in RationalSegmentInterval interval,
        ref CapsulePlaneRaySelection selection) =>
        Dispatch(source, frame, orientation, ref selection, null, true, descriptor, interval);

    private static bool Dispatch(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        int orientation, ref CapsulePlaneRaySelection maximum, CapsulePlaneRayEventVisitor? visitor,
        bool targeted, CapsulePlaneRayEvent target, in RationalSegmentInterval interval)
    {
        if (orientation is not (1 or -1)) throw new ArgumentOutOfRangeException(nameof(orientation));
        if (interval.IsClipped && !source.IsSegment)
            throw new ArgumentException("Only finite segment sources can have a clipped interval.", nameof(source));
        if (frame.NormalSquared.IsZero) return false;
        // A retained non-projection event has no support/endpoint work to replay.
        if (targeted && target.Kind is not (CapsulePlaneRayEventKind.Support or CapsulePlaneRayEventKind.Endpoint))
            return source.IsSegment
                ? AccumulateSegment(source, frame, orientation, ref maximum, visitor, true, target, interval)
                : AccumulateIntrinsic(source, frame, orientation, ref maximum, visitor, true, target);
        int words = GetProducerWords(frame, CapsulePlaneRayEventKind.Support);
        Span<ulong> values = stackalloc ulong[12 * words]; Span<int> signs = stackalloc int[12];
        ContactQuadratic x = Field(values, signs, 0, words), y = Field(values, signs, 1, words), z = Field(values, signs, 2, words);
        ContactQuadratic d = Field(values, signs, 3, words), gap = Field(values, signs, 4, words), term = Field(values, signs, 5, words);
        bool found = false;
        for (int endpoint = 0; endpoint < (source.IsSegment || frame.Finite.Length != Fixed64.Zero ? 2 : 1); endpoint++)
        {
            var descriptor = new CapsulePlaneRayEvent(source.IsSegment ? CapsulePlaneRayEventKind.Endpoint
                : CapsulePlaneRayEventKind.Support, endpoint);
            if (targeted && !target.Matches(descriptor)) continue;
            if (source.IsSegment)
            {
                if (interval.IsClipped) SetClippedEndpoint(source, frame.Finite, interval, endpoint, x, y, z, d);
                else
                {
                    WideAxis3 point = frame.Finite.Transform(endpoint == 0 ? source.A : source.B);
                    x.Set(Signed576.ExtendValue(point.X)); y.Set(Signed576.ExtendValue(point.Y));
                    z.Set(Signed576.ExtendValue(point.Z)); d.Set(Integer(1));
                }
            }
            else
            {
                // Project the endpoint center, not its rounded radial support.
                // The radial normal component cancels under plane projection.
                int endpointSign = endpoint == 0 ? -1 : 1;
                if (!targeted && frame.Finite.Length != Fixed64.Zero && !frame.Normal.Y.IsZero
                    && endpointSign != orientation * frame.Normal.Y.Sign) continue;
                Signed320 cap = endpointSign < 0 ? WideArithmetic.Negate(frame.Finite.ShapeFrame.Cap) : frame.Finite.ShapeFrame.Cap;
                gap.Set(frame.PlaneConstant);
                term.Set(Signed576.ExtendValue(cap));
                ContactQuadratic.Scale(term, Signed576.ExtendValue(frame.Normal.Y), y); gap.Add(y, -1);
                ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.X), x);
                ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Y), y);
                term.Set(frame.NormalSquared); ContactQuadratic.Scale(term, Signed576.ExtendValue(cap), z); y.Add(z);
                ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Z), z); d.Set(frame.NormalSquared);
            }
            found |= EmitPoint(source, frame, descriptor, orientation, x, y, z, d, ReadOnlySpan<ulong>.Empty,
                ref maximum, visitor);
        }
        if (targeted) return found;
        if (source.IsSegment)
            found |= AccumulateSegment(source, frame, orientation, ref maximum, visitor, targeted, target, interval);
        else
            found |= AccumulateIntrinsic(source, frame, orientation, ref maximum, visitor, targeted, target);
        return found;
    }

    private static void SetClippedEndpoint(in CapsulePlaneRayEventSource source, in CapsuleFiniteSectionFrame finite,
        in RationalSegmentInterval interval, int endpoint, ContactQuadratic x, ContactQuadratic y,
        ContactQuadratic z, ContactQuadratic denominator)
    {
        WideAxis3 start = finite.Transform(source.A), end = finite.Transform(source.B);
        Signed320 n = endpoint == 0 ? interval.LowerNumerator : interval.UpperNumerator;
        Signed320 d = endpoint == 0 ? interval.LowerDenominator : interval.UpperDenominator;
        // Doubled rigid coordinates/differences are <198 bits and rational
        // bounds <319 bits. Their complete affine sums fit Signed576.
        x.Set(Coordinate(start.X, end.X)); y.Set(Coordinate(start.Y, end.Y)); z.Set(Coordinate(start.Z, end.Z));
        denominator.Set(Signed576.ExtendValue(d));

        Signed576 Coordinate(Signed320 first, Signed320 last) =>
            WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(first, d),
                WideArithmetic.MultiplySigned320(WideArithmetic.SubtractSigned320(last, first), n));
    }

    private static bool AccumulateIntrinsic(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        int orientation, ref CapsulePlaneRaySelection maximum, CapsulePlaneRayEventVisitor? visitor,
        bool targeted, CapsulePlaneRayEvent target)
    {
        if (frame.Finite.Length == Fixed64.Zero)
            return AccumulateCircle(source, frame, 2, default, false, orientation, ref maximum, visitor, targeted, target);
        if (targeted && target.Kind == CapsulePlaneRayEventKind.Circle)
        {
            if (!frame.Normal.X.IsZero || !frame.Normal.Z.IsZero) return false;
            // Axial planes in the closed core interval own the body circle,
            // including seams. Cap-circle descriptors there remain stale even
            // though their endpoint sphere intersects the same geometric circle.
            Signed576 extent = WideArithmetic.MultiplySigned320(frame.Normal.Y, frame.Finite.ShapeFrame.Cap);
            if (extent.Sign < 0) extent = WideArithmetic.SubtractSigned576(default, extent);
            if (WideArithmetic.AddSigned576(frame.PlaneConstant, extent).Sign >= 0
                && WideArithmetic.SubtractSigned576(frame.PlaneConstant, extent).Sign <= 0)
                return AccumulateCircle(source, frame, 0, default, true, orientation, ref maximum, visitor, true, target);
            if (target.Feature is not (1 or 2)) return false;
            Signed320 cap = target.Feature == 1
                ? WideArithmetic.Negate(frame.Finite.ShapeFrame.Cap) : frame.Finite.ShapeFrame.Cap;
            return AccumulateCircle(source, frame, target.Feature, cap, false, orientation, ref maximum, visitor, true, target);
        }
        int words = GetProducerWords(frame, CapsulePlaneRayEventKind.Sagittal);
        Span<ulong> values = stackalloc ulong[28 * words]; Span<int> signs = stackalloc int[28];
        Span<ulong> root = stackalloc ulong[CapsulePlaneRaySelection.RootWords];
        ContactQuadratic x = Field(values, signs, 0, words), y = Field(values, signs, 1, words), z = Field(values, signs, 2, words), d = Field(values, signs, 3, words);
        ContactQuadratic radial = Field(values, signs, 4, words), metric = Field(values, signs, 5, words), r2 = Field(values, signs, 6, words);
        ContactQuadratic gap = Field(values, signs, 7, words), radicand = Field(values, signs, 8, words), term = Field(values, signs, 9, words);
        ContactQuadratic work = Field(values, signs, 10, words), coefficient = Field(values, signs, 11, words);
        ContactQuadratic radx = Field(values, signs, 12, words), rady = Field(values, signs, 13, words);
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius, frame.Finite.ShapeFrame.Denominator);
        radial.Set(WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(frame.Normal.X, frame.Normal.X),
            WideArithmetic.MultiplySigned320(frame.Normal.Z, frame.Normal.Z)));
        metric.Set(frame.NormalSquared); r2.Set(WideArithmetic.MultiplySigned320(radius, radius));
        bool found = false;
        if (radial.Signs[0] == 0)
        {
            // A plane perpendicular to the core cuts a full body circle even
            // when it is strictly between the two body/cap seams.
            gap.Set(frame.PlaneConstant);
            ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Y), y); metric.CopyTo(d);
            if (ContainsPrimitivePoint(frame, y, d, ReadOnlySpan<ulong>.Empty, 0))
                return AccumulateCircle(source, frame, 0, default, true, orientation, ref maximum, visitor,
                    targeted, target);
            for (int endpoint = 0; endpoint < 2; endpoint++)
            {
                Signed320 cap = endpoint == 0 ? WideArithmetic.Negate(frame.Finite.ShapeFrame.Cap) : frame.Finite.ShapeFrame.Cap;
                found |= AccumulateCircle(source, frame, endpoint + 1, cap, false, orientation, ref maximum, visitor,
                    targeted, target);
            }
            return found;
        }
        if (!frame.Normal.Y.IsZero)
        {
            // The sagittal diameter contains the ray extrema. Its transverse
            // diameter supplies independent finite coverage on the same ellipse.
            for (int kind = 0; kind < 2; kind++)
            for (int branch = -1; branch <= 1; branch += 2)
            {
                var descriptor = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Sagittal, kind == 0 ? 0 : 3, branch);
                if (targeted && !target.Matches(descriptor)) continue;
                CopyRadicand(radial, root);
                ContactQuadratic.Scale(radial, Signed576.ExtendValue(frame.Normal.Y), d);
                ContactQuadratic.Scale(radial, frame.PlaneConstant, y);
                coefficient.Set(Signed576.ExtendValue(radius));
                ContactQuadratic.Scale(coefficient, Signed576.ExtendValue(kind == 0 ? frame.Normal.X : frame.Normal.Z), term);
                if (kind != 0) term.MultiplySign(-1);
                ContactQuadratic.Scale(term, Signed576.ExtendValue(frame.Normal.Y), work); SetRadical(x, work, branch);
                ContactQuadratic.Scale(coefficient, Signed576.ExtendValue(kind == 0 ? frame.Normal.Z : frame.Normal.X), term);
                ContactQuadratic.Scale(term, Signed576.ExtendValue(frame.Normal.Y), work); SetRadical(z, work, branch);
                if (kind == 0)
                {
                    ContactQuadratic.Scale(radial, Signed576.ExtendValue(radius), work);
                    SetRadical(term, work, -branch); y.Add(term);
                }
                if (d.Signs[0] < 0) { x.MultiplySign(-1); y.MultiplySign(-1); z.MultiplySign(-1); d.MultiplySign(-1); }
                if (ContainsPrimitivePoint(frame, y, d, root, 0))
                    found |= EmitPoint(source, frame, descriptor, orientation, x, y, z, d, root, ref maximum, visitor, admitted: true);
            }
        }
        for (int endpoint = 0; endpoint < 2; endpoint++)
        {
            Signed320 cap = endpoint == 0 ? WideArithmetic.Negate(frame.Finite.ShapeFrame.Cap) : frame.Finite.ShapeFrame.Cap;
            int primitive = endpoint + 1;
            gap.Set(WideArithmetic.SubtractSigned576(frame.PlaneConstant, WideArithmetic.MultiplySigned320(frame.Normal.Y, cap)));
            for (int kind = 0; kind < 3; kind++)
            {
                var eventKind = kind == 1 ? CapsulePlaneRayEventKind.Seam : CapsulePlaneRayEventKind.Sagittal;
                int feature = kind == 2 ? primitive + 3 : primitive;
                if (targeted && (target.Kind != eventKind || target.Feature != feature)) continue;
                ContactQuadratic.Multiply(r2, kind == 1 ? radial : metric, ReadOnlySpan<ulong>.Empty, radicand);
                ContactQuadratic.Multiply(gap, gap, ReadOnlySpan<ulong>.Empty, term); radicand.Add(term, -1);
                if (radicand.Signs[0] < 0) continue;
                if (kind != 1)
                {
                    ContactQuadratic.Multiply(radicand, radial, ReadOnlySpan<ulong>.Empty, work);
                    if (kind == 2) ContactQuadratic.Multiply(work, metric, ReadOnlySpan<ulong>.Empty, radicand);
                    else work.CopyTo(radicand);
                    ContactQuadratic.Multiply(metric, radial, ReadOnlySpan<ulong>.Empty, d);
                    if (kind == 0)
                    {
                        // The cap section's sagittal tangent is (-Ny*Nx,T,-Ny*Nz).
                        coefficient.Set(Signed576.ExtendValue(frame.Normal.Y));
                        ContactQuadratic.Scale(coefficient, Signed576.ExtendValue(frame.Normal.X), radx); radx.MultiplySign(-1);
                        radial.CopyTo(rady);
                    }
                    else
                    {
                        // Its transverse tangent (-Nz,0,Nx) completes finite
                        // coverage even when the circle lies wholly in one cap.
                        radx.Set(Signed576.ExtendValue(frame.Normal.Z)); radx.MultiplySign(-1); rady.Clear();
                    }
                }
                else { radial.CopyTo(d); radx.Set(Signed576.ExtendValue(frame.Normal.Z)); rady.Clear(); }
                CopyRadicand(radicand, root);
                for (int branch = -1; branch <= 1; branch += 2)
                {
                    var descriptor = new CapsulePlaneRayEvent(eventKind, feature, branch);
                    if (targeted && !target.Matches(descriptor)) continue;
                    ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.X), x);
                    ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Z), z);
                    if (kind != 1)
                    {
                        ContactQuadratic.Multiply(x, radial, ReadOnlySpan<ulong>.Empty, work); work.CopyTo(x);
                        ContactQuadratic.Multiply(z, radial, ReadOnlySpan<ulong>.Empty, work); work.CopyTo(z);
                        ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Y), y);
                        ContactQuadratic.Scale(metric, Signed576.ExtendValue(cap), term); y.Add(term);
                        ContactQuadratic.Multiply(y, radial, ReadOnlySpan<ulong>.Empty, work); work.CopyTo(y);
                        SetRadical(term, rady, branch); y.Add(term);
                        if (kind == 0)
                        {
                            coefficient.Set(Signed576.ExtendValue(frame.Normal.Y));
                            ContactQuadratic.Scale(coefficient, Signed576.ExtendValue(frame.Normal.Z), work); work.MultiplySign(-1);
                        }
                        else work.Set(Signed576.ExtendValue(frame.Normal.X));
                    }
                    else
                    {
                        ContactQuadratic.Scale(radial, Signed576.ExtendValue(cap), y);
                        work.Set(Signed576.ExtendValue(frame.Normal.X)); work.MultiplySign(-1);
                    }
                    SetRadical(term, radx, branch); x.Add(term); SetRadical(term, work, branch); z.Add(term);
                    if (kind == 1 || ContainsPrimitivePoint(frame, y, d, root, primitive))
                        found |= EmitPoint(source, frame, descriptor, orientation, x, y, z, d, root, ref maximum, visitor, admitted: true);
                }
            }
        }
        // N<264, C/gap<465, cap/radius<198 bits. Diameter/seam q fields
        // are below 1264 bits, D below 1060, radicands below 1992; the existing
        // circular-family 2256-bit bound still owns the widest point root.
        return found;
    }

    private static bool AccumulateCircle(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        int primitive, Signed320 cap, bool body, int orientation, ref CapsulePlaneRaySelection maximum,
        CapsulePlaneRayEventVisitor? visitor, bool targeted, CapsulePlaneRayEvent target)
    {
        if (targeted && (target.Kind != CapsulePlaneRayEventKind.Circle || target.Feature != primitive)) return false;
        int words = GetProducerWords(frame, CapsulePlaneRayEventKind.Circle);
        Span<ulong> values = stackalloc ulong[28 * words]; Span<int> signs = stackalloc int[28];
        Span<ulong> root = stackalloc ulong[CapsulePlaneRaySelection.RootWords];
        ContactQuadratic x = Field(values, signs, 0, words), y = Field(values, signs, 1, words), z = Field(values, signs, 2, words), d = Field(values, signs, 3, words);
        ContactQuadratic gap = Field(values, signs, 4, words), section = Field(values, signs, 5, words), metric = Field(values, signs, 6, words);
        ContactQuadratic v2 = Field(values, signs, 7, words), radicand = Field(values, signs, 8, words), term = Field(values, signs, 9, words);
        ContactQuadratic vx = Field(values, signs, 10, words), vy = Field(values, signs, 11, words), vz = Field(values, signs, 12, words);
        ContactQuadratic work = Field(values, signs, 13, words);
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius, frame.Finite.ShapeFrame.Denominator);
        metric.Set(frame.NormalSquared); section.Set(WideArithmetic.MultiplySigned320(radius, radius));
        ContactQuadratic.Multiply(section, metric, ReadOnlySpan<ulong>.Empty, work); work.CopyTo(section);
        gap.Set(WideArithmetic.SubtractSigned576(frame.PlaneConstant, WideArithmetic.MultiplySigned320(frame.Normal.Y, cap)));
        if (!body) { ContactQuadratic.Multiply(gap, gap, ReadOnlySpan<ulong>.Empty, term); section.Add(term, -1); }
        if (section.Signs[0] < 0) return false;
        // Author the family basis in the mesh plane. Capsule yaw (and a
        // zero-core sphere's arbitrary axis) must not change finite samples.
        WideAxis3 seed = frame.AuthoredNormal.X.IsZero && frame.AuthoredNormal.Z.IsZero
            ? new(Signed320.ExtendValue(Signed192.Signed(1)), default, default)
            : new(default, Signed320.ExtendValue(Signed192.Signed(1)), default);
        WideAxis3 first = WideAxis3.Cross(frame.AuthoredNormal, seed);
        bool found = false;
        for (int tangent = 0; tangent < 2; tangent++)
        {
            if (targeted && tangent != target.Branch / 2) continue;
            WideAxis3 authored = tangent == 0 ? first : WideAxis3.Cross(frame.AuthoredNormal, first);
            WideRigidProjection.TransformLocalAxis(frame.Finite.ShapeFrame.Basis, authored,
                out Signed576 ax, out Signed576 ay, out Signed576 az);
            vx.Set(ax); vy.Set(ay); vz.Set(az); Squared(vx, vy, vz, v2, term);
            ContactQuadratic.Multiply(section, metric, ReadOnlySpan<ulong>.Empty, work);
            ContactQuadratic.Multiply(work, v2, ReadOnlySpan<ulong>.Empty, radicand); CopyRadicand(radicand, root);
            ContactQuadratic.Multiply(metric, v2, ReadOnlySpan<ulong>.Empty, d);
            for (int branch = -1; branch <= 1; branch += 2)
            {
                var descriptor = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Circle,
                    primitive, 2 * tangent + (branch > 0 ? 1 : 0));
                if (targeted && !target.Matches(descriptor)) continue;
                ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.X), work);
                ContactQuadratic.Multiply(work, v2, ReadOnlySpan<ulong>.Empty, x);
                ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Y), work);
                ContactQuadratic.Scale(metric, Signed576.ExtendValue(cap), term); work.Add(term);
                ContactQuadratic.Multiply(work, v2, ReadOnlySpan<ulong>.Empty, y);
                ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Z), work);
                ContactQuadratic.Multiply(work, v2, ReadOnlySpan<ulong>.Empty, z);
                SetRadical(term, vx, branch); x.Add(term); SetRadical(term, vy, branch); y.Add(term);
                SetRadical(term, vz, branch); z.Add(term);
                if (body || ContainsPrimitivePoint(frame, y, d, root, primitive))
                    found |= EmitPoint(source, frame, descriptor, orientation, x, y, z, d, root, ref maximum, visitor, admitted: true);
            }
        }
        // Authored tangents <262 bits transform to <395-bit components.
        // q fields <1523, denominators <1322, radicands <2256 bits;
        // the opposite ray norm remains <9444 bits (160 limbs).
        return found;
    }

    private static void SetRadical(ContactQuadratic target, ContactQuadratic coefficient, int multiplier)
    {
        target.Clear(); coefficient.Rational.CopyTo(target.Radical); target.Signs[1] = coefficient.Signs[0] * multiplier;
    }

    private static void CopyRadicand(ContactQuadratic value, Span<ulong> root)
    {
        root.Clear(); value.Rational[..WideArithmetic.GetActiveMagnitudeLength(value.Rational)].CopyTo(root);
    }

    private static bool ContainsPrimitivePoint(in CapsulePlaneRayFrame frame, ContactQuadratic y,
        ContactQuadratic d, ReadOnlySpan<ulong> root, int primitive)
    {
        if (frame.Finite.Length == Fixed64.Zero) return true;
        Span<ulong> values = stackalloc ulong[4 * Words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic extent = Field(values, signs, 0), query = Field(values, signs, 1);
        ContactQuadratic.Scale(d, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), extent);
        y.CopyTo(query); query.Add(extent, -1); int upper = query.Sign(root);
        y.CopyTo(query); query.Add(extent); int lower = query.Sign(root);
        return primitive == 0 ? upper <= 0 && lower >= 0 : primitive == 1 ? lower <= 0 : upper >= 0;
    }

    private static bool AccumulateSegment(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        int orientation, ref CapsulePlaneRaySelection maximum, CapsulePlaneRayEventVisitor? visitor,
        bool targeted, CapsulePlaneRayEvent target, in RationalSegmentInterval interval)
    {
        if (targeted && target.Kind is not (CapsulePlaneRayEventKind.Section or CapsulePlaneRayEventKind.Stationary)) return false;
        WideAxis3 p = frame.Finite.Transform(source.A), end = frame.Finite.Transform(source.B);
        WideAxis3 e = new(WideArithmetic.SubtractSigned320(end.X, p.X), WideArithmetic.SubtractSigned320(end.Y, p.Y),
            WideArithmetic.SubtractSigned320(end.Z, p.Z));
        // Primitive section roots are already capsule boundary certificates.
        // Their plane certificate is shared only by genuinely coplanar sources;
        // arbitrary internal segment inputs retain the full point admission.
        bool sourceOnPlane = WideArithmetic.SubtractSigned576(WideAxis3.Dot(p, frame.Normal), frame.PlaneConstant).IsZero
            && WideAxis3.Dot(e, frame.Normal).IsZero;
        int words = GetSegmentWords(frame, p, end, e);
        Span<ulong> values = stackalloc ulong[40 * words]; Span<int> signs = stackalloc int[40];
        Span<ulong> root = stackalloc ulong[CapsulePlaneRaySelection.RootWords];
        ContactQuadratic a = Field(values, signs, 0, words), b = Field(values, signs, 1, words), c = Field(values, signs, 2, words);
        ContactQuadratic m = Field(values, signs, 3, words), n = Field(values, signs, 4, words), A = Field(values, signs, 5, words);
        ContactQuadratic qa = Field(values, signs, 6, words), qb = Field(values, signs, 7, words), qc = Field(values, signs, 8, words);
        ContactQuadratic t = Field(values, signs, 9, words), td = Field(values, signs, 10, words), u = Field(values, signs, 11, words), ud = Field(values, signs, 12, words);
        ContactQuadratic work = Field(values, signs, 13, words), term = Field(values, signs, 14, words);
        ContactQuadratic x = Field(values, signs, 15, words), y = Field(values, signs, 16, words), z = Field(values, signs, 17, words), d = Field(values, signs, 18, words);
        ContactQuadratic exitY = Field(values, signs, 19, words);
        Span<ulong> candidateValues = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> candidateSigns = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> candidateValueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var candidate = new CapsulePlaneRaySelection(candidateValues, candidateSigns, candidateValueSigns);
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius, frame.Finite.ShapeFrame.Denominator);
        Signed576 radiusSquared = WideArithmetic.MultiplySigned320(radius, radius);
        bool found = false;
        for (int primitive = frame.Finite.Length == Fixed64.Zero ? 2 : 0; primitive < 3; primitive++)
        {
            if (targeted && primitive != target.Feature) continue;
            bool body = primitive == 0;
            Signed320 cap = primitive == 1 ? WideArithmetic.Negate(frame.Finite.ShapeFrame.Cap) : frame.Finite.ShapeFrame.Cap;
            WideAxis3 basePoint = new(p.X, body ? default : WideArithmetic.SubtractSigned320(p.Y, cap), p.Z);
            WideAxis3 delta = new(e.X, body ? default : e.Y, e.Z);
            WideAxis3 normal = new(frame.Normal.X, body ? default : frame.Normal.Y, frame.Normal.Z);
            a.Set(WideAxis3.Dot(delta, delta)); b.Set(WideAxis3.Dot(basePoint, delta));
            c.Set(WideArithmetic.SubtractSigned576(WideAxis3.Dot(basePoint, basePoint), radiusSquared));
            bool wantSection = !targeted || target.Kind == CapsulePlaneRayEventKind.Section;
            if (wantSection)
            {
                for (int branch = -1; branch <= 1; branch += 2)
                {
                    var descriptor = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Section, primitive, branch);
                    if (targeted && !target.Matches(descriptor)) continue;
                    if (!ContactQuadratic.TryGetParameter(a, b, c, branch, root, u, ud)
                        || !IsUnitParameter(u, ud, root, term)
                        || interval.IsClipped && !interval.Contains(u, ud, root)) continue;
                    SetAffinePoint(p, e, u, ud, x, y, z, d, term);
                    if (ContainsPrimitivePoint(frame, y, d, root, primitive))
                        found |= EmitPoint(source, frame, descriptor, orientation, x, y, z, d, root, ref maximum, visitor,
                            admitted: sourceOnPlane);
                }
            }
            if (a.Signs[0] == 0 || targeted && target.Kind != CapsulePlaneRayEventKind.Stationary) continue;
            // Eliminate u from G(u,t)=0 and G_u(u,t)=0. A zero
            // edge coefficient is a constant family covered by its endpoints.
            m.Set(WideAxis3.Dot(delta, normal)); n.Set(WideAxis3.Dot(basePoint, normal)); A.Set(WideAxis3.Dot(normal, normal));
            ContactQuadratic.Multiply(a, A, ReadOnlySpan<ulong>.Empty, qa);
            ContactQuadratic.Multiply(m, m, ReadOnlySpan<ulong>.Empty, term); qa.Add(term, -1);
            ContactQuadratic.Multiply(a, n, ReadOnlySpan<ulong>.Empty, qb);
            ContactQuadratic.Multiply(b, m, ReadOnlySpan<ulong>.Empty, term); qb.Add(term, -1);
            ContactQuadratic.Multiply(a, c, ReadOnlySpan<ulong>.Empty, qc);
            ContactQuadratic.Multiply(b, b, ReadOnlySpan<ulong>.Empty, term); qc.Add(term, -1);
            for (int branch = -1; branch <= 1; branch += 2)
            {
                if (!ContactQuadratic.TryGetParameter(qa, qb, qc, branch, root, t, td)) continue;
                int constructionOrientation = t.Sign(root) < 0 ? -1 : 1;
                var descriptor = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Stationary, primitive, branch, constructionOrientation);
                if (targeted && !target.Matches(descriptor)) continue;
                // Untargeted visitation preserves the directional event
                // inventory; targeted replay can still evaluate the other ray.
                if (visitor != null && constructionOrientation != orientation) continue;
                if (m.Signs[0] == 0)
                {
                    // An edge perpendicular to the ray has u=-b/a. Keeping
                    // the unrelated depth denominator here would needlessly
                    // square its large common factor on opposite-ray replay.
                    b.CopyTo(u, -1); a.CopyTo(ud);
                }
                else
                {
                    ContactQuadratic.Multiply(b, td, root, u);
                    ContactQuadratic.Multiply(m, t, root, term); u.Add(term); u.MultiplySign(-1);
                    ContactQuadratic.Multiply(a, td, root, ud);
                }
                if (!IsUnitParameter(u, ud, root, term)
                    || interval.IsClipped && !interval.Contains(u, ud, root)) continue;
                SetAffinePoint(p, e, u, ud, x, y, z, d, term);
                ContactQuadratic.Multiply(y, td, root, exitY);
                ContactQuadratic.Multiply(d, t, root, term);
                ContactQuadratic.Scale(term, Signed576.ExtendValue(frame.Normal.Y), work); exitY.Add(work);
                ContactQuadratic.Multiply(d, td, root, work);
                if (!ContainsPrimitivePoint(frame, exitY, work, root, primitive)) continue;
                if (constructionOrientation == orientation)
                {
                    if (!ContainsPoint(frame, x, y, z, d, root)) continue;
                    candidate.HasValue = false;
                    candidate.CurrentSource = source; candidate.CurrentEvent = descriptor;
                    t.CopyTo(term, orientation);
                    candidate.Keep(x, y, z, d, term, td, root, primitive, frame);
                    if (visitor == null) maximum.KeepEvaluated(candidate, frame);
                    else visitor(source, frame, descriptor, orientation, candidate);
                    found = true;
                }
                else if (targeted)
                    found |= EmitPoint(source, frame, descriptor, orientation, x, y, z, d, root, ref maximum, visitor);
            }
        }
        return found;
    }

    private static bool IsUnitParameter(ContactQuadratic numerator, ContactQuadratic denominator,
        ReadOnlySpan<ulong> root, ContactQuadratic query)
    {
        if (numerator.Sign(root) < 0 || denominator.Sign(root) <= 0) return false;
        denominator.CopyTo(query); query.Add(numerator, -1); return query.Sign(root) >= 0;
    }

    private static void SetAffinePoint(WideAxis3 start, WideAxis3 delta, ContactQuadratic u, ContactQuadratic ud,
        ContactQuadratic x, ContactQuadratic y, ContactQuadratic z, ContactQuadratic d, ContactQuadratic term)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic point = axis == 0 ? x : axis == 1 ? y : z;
            ContactQuadratic.Scale(ud, Signed576.ExtendValue(axis == 0 ? start.X : axis == 1 ? start.Y : start.Z), point);
            ContactQuadratic.Scale(u, Signed576.ExtendValue(axis == 0 ? delta.X : axis == 1 ? delta.Y : delta.Z), term); point.Add(term);
        }
        ud.CopyTo(d);
    }

    private static bool EmitPoint(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        CapsulePlaneRayEvent descriptor, int orientation, scoped ContactQuadratic x, scoped ContactQuadratic y,
        scoped ContactQuadratic z, scoped ContactQuadratic d, scoped ReadOnlySpan<ulong> root,
        ref CapsulePlaneRaySelection maximum, CapsulePlaneRayEventVisitor? visitor, bool admitted = false)
    {
        // Normalization only shifts right, so copied input coefficients never
        // need more than their current active width.
        int words = Math.Max(9, Math.Max(Math.Max(ContactQuadratic.ActiveWords(x), ContactQuadratic.ActiveWords(y)),
            Math.Max(ContactQuadratic.ActiveWords(z), ContactQuadratic.ActiveWords(d))));
        Span<ulong> pointValues = stackalloc ulong[8 * words]; Span<int> pointSigns = stackalloc int[8];
        Span<ulong> pointRoot = stackalloc ulong[CapsulePlaneRaySelection.RootWords];
        ContactQuadratic nx = Field(pointValues, pointSigns, 0, words), ny = Field(pointValues, pointSigns, 1, words);
        ContactQuadratic nz = Field(pointValues, pointSigns, 2, words), nd = Field(pointValues, pointSigns, 3, words);
        x.CopyTo(nx); y.CopyTo(ny); z.CopyTo(nz); d.CopyTo(nd);
        root.CopyTo(pointRoot); pointRoot[root.Length..].Clear();
        // Keep source branch-loop coefficients and the output selection apart:
        // cap admission can retain an exit before testing the opposite cap.
        WideFiniteAxisIntersection.NormalizeQuadraticPointPowerOfTwo(nx, ny, nz, nd, pointRoot);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var candidate = new CapsulePlaneRaySelection(values, signs, valueSigns) { CurrentSource = source, CurrentEvent = descriptor };
        if (descriptor.Kind == CapsulePlaneRayEventKind.Support
            && (frame.Finite.Length == Fixed64.Zero || frame.Normal.Y.IsZero
                || (descriptor.Feature == 0 ? -1 : 1) == orientation * frame.Normal.Y.Sign))
        {
            if (!ContainsPoint(frame, nx, ny, nz, nd, pointRoot)) return false;
            // This endpoint is the capsule's support hemisphere. Its exact
            // support projects to q, so the first exit is
            // (r*sqrt(S)-orientation*(C-Ny*cap))/S. A replay toward the other
            // hemisphere retains the general finite body/cap solver below.
            Signed320 cap = descriptor.Feature == 0 ? WideArithmetic.Negate(frame.Finite.ShapeFrame.Cap)
                : frame.Finite.ShapeFrame.Cap;
            Signed576 gap = WideArithmetic.SubtractSigned576(frame.PlaneConstant,
                WideArithmetic.MultiplySigned320(frame.Normal.Y, cap));
            Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius,
                frame.Finite.ShapeFrame.Denominator);
            ContactQuadratic numerator = candidate.DepthNumerator, divisor = candidate.DepthDenominator;
            numerator.Set(orientation > 0 ? WideArithmetic.SubtractSigned576(default, gap) : gap);
            divisor.Set(Signed576.ExtendValue(radius));
            divisor.Rational.CopyTo(numerator.Radical); numerator.Signs[1] = divisor.Signs[0];
            divisor.Set(frame.NormalSquared); CopyRadicand(divisor, pointRoot);
            candidate.Keep(nx, ny, nz, nd, numerator, divisor, pointRoot,
                frame.Finite.Length == Fixed64.Zero ? 2 : frame.Normal.Y.IsZero ? 0 : descriptor.Feature + 1, frame);
        }
        else if (admitted && descriptor.Kind == CapsulePlaneRayEventKind.Circle
            && (frame.Finite.Length == Fixed64.Zero || descriptor.Feature == 0))
        {
            ContactQuadratic numerator = candidate.DepthNumerator, divisor = candidate.DepthDenominator;
            Signed576 depthNumerator;
            if (frame.Finite.Length == Fixed64.Zero)
            {
                // A sphere rim has q.N=C. Its outward upper root is the
                // nonnegative part of -2*orientation*C/S, for every family q.
                depthNumerator = WideArithmetic.AddSigned576(frame.PlaneConstant, frame.PlaneConstant);
                if (orientation > 0) depthNumerator = WideArithmetic.SubtractSigned576(default, depthNumerator);
                if (depthNumerator.Sign < 0) depthNumerator = default;
            }
            else
            {
                // The axial body family lies on a tube generator. Its first
                // exit is the forward closed seam: (|Ny|*Cap-orientation*C)/S.
                Signed320 normalY = frame.Normal.Y.Sign < 0 ? WideArithmetic.Negate(frame.Normal.Y) : frame.Normal.Y;
                Signed576 extent = WideArithmetic.MultiplySigned320(normalY, frame.Finite.ShapeFrame.Cap);
                depthNumerator = orientation > 0 ? WideArithmetic.SubtractSigned576(extent, frame.PlaneConstant)
                    : WideArithmetic.AddSigned576(extent, frame.PlaneConstant);
            }
            numerator.Set(depthNumerator); divisor.Set(frame.NormalSquared);
            candidate.Keep(nx, ny, nz, nd, numerator, divisor, pointRoot,
                frame.Finite.Length == Fixed64.Zero ? 2 : 0, frame);
        }
        else if (!AccumulateQuadraticPoint(frame, nx, ny, nz, nd, pointRoot, orientation, ref candidate, admitted)) return false;
        // A visitor owns finite-region ranking. An unconsumed global maximum
        // would compare every event again, including outside-region points.
        if (visitor == null) maximum.KeepEvaluated(candidate, frame);
        else visitor(source, frame, descriptor, orientation, candidate);
        return true;
    }

    private static Signed576 Integer(long value) => Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(value)));

    /// <summary>Accumulates the actual first outward ray through one rational admitted plane point.</summary>
    /// <remarks>Point coefficients are rational; quadratic finite-edge points use the retained value-root branch.</remarks>
    internal static bool AccumulateRationalPoint(in CapsulePlaneRayFrame frame,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator,
        int orientation, ref CapsulePlaneRaySelection selection)
        => AccumulateQuadraticPoint(frame, x, y, z, denominator, ReadOnlySpan<ulong>.Empty, orientation, ref selection);

    private static bool AccumulateQuadraticPoint(in CapsulePlaneRayFrame frame,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator,
        scoped ReadOnlySpan<ulong> pointRoot, int orientation, ref CapsulePlaneRaySelection selection, bool admitted = false)
    {
        if (orientation is not (1 or -1)) throw new ArgumentOutOfRangeException(nameof(orientation));
        if (frame.NormalSquared.IsZero || denominator.Sign(pointRoot) <= 0) return false;
        int words = GetPointRayWords(frame, x, y, z, denominator, pointRoot);
        Span<ulong> values = stackalloc ulong[32 * words]; Span<int> signs = stackalloc int[32];
        Span<ulong> root = stackalloc ulong[CapsulePlaneRaySelection.RootWords];
        ContactQuadratic a = Field(values, signs, 0, words), b = Field(values, signs, 1, words), c = Field(values, signs, 2, words);
        ContactQuadratic rx = Field(values, signs, 3, words), ry = Field(values, signs, 4, words), rz = Field(values, signs, 5, words);
        ContactQuadratic squaredDenominator = Field(values, signs, 6, words), term = Field(values, signs, 7, words), work = Field(values, signs, 8, words);
        ContactQuadratic numerator = Field(values, signs, 9, words), divisor = Field(values, signs, 10, words);
        ContactQuadratic px = Field(values, signs, 11, words), py = Field(values, signs, 12, words), pz = Field(values, signs, 13, words);
        ContactQuadratic pd = Field(values, signs, 14, words), extent = Field(values, signs, 15, words);
        ContactQuadratic.Multiply(denominator, denominator, pointRoot, squaredDenominator);
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius, frame.Finite.ShapeFrame.Denominator);
        Signed576 radiusSquared = WideArithmetic.MultiplySigned320(radius, radius);
        if (!admitted && !ContainsQuadraticPoint(frame, x, y, z, denominator, radiusSquared, squaredDenominator,
                rx, ry, rz, c, term, work, pointRoot)) return false;
        bool rational = x.Signs[1] == 0 && y.Signs[1] == 0 && z.Signs[1] == 0 && denominator.Signs[1] == 0;
        Span<ulong> valueStorage = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        Span<sbyte> coefficientSigns = stackalloc sbyte[6];
        var value = new CapsulePlaneRayValue(valueStorage, valueSigns);
        bool found = false;
        // A ray parallel to the finite core leaves only through its forward
        // cap. The opposite cap and zero-quadratic body cannot own that exit.
        bool axial = frame.Normal.X.IsZero && frame.Normal.Z.IsZero;
        int firstPrimitive = frame.Finite.Length == Fixed64.Zero ? 2
            : axial ? orientation * frame.Normal.Y.Sign < 0 ? 1 : 2 : 0;
        int primitiveEnd = axial ? firstPrimitive + 1 : 3;
        for (int primitive = firstPrimitive; primitive < primitiveEnd; primitive++)
        {
            x.CopyTo(rx); y.CopyTo(ry); z.CopyTo(rz);
            bool body = primitive == 0;
            if (body) ry.Clear();
            else
            {
                Signed320 endpoint = primitive == 1 ? WideArithmetic.Negate(frame.Finite.ShapeFrame.Cap) : frame.Finite.ShapeFrame.Cap;
                ContactQuadratic.Scale(denominator, Signed576.ExtendValue(endpoint), term); ry.Add(term, -1);
            }
            Signed576 metric = body ? WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(frame.Normal.X, frame.Normal.X),
                WideArithmetic.MultiplySigned320(frame.Normal.Z, frame.Normal.Z)) : frame.NormalSquared;
            ContactQuadratic.Scale(squaredDenominator, metric, a);
            Dot(rx, ry, rz, frame.Normal, b, term);
            ContactQuadratic.Multiply(b, denominator, pointRoot, work); work.CopyTo(b); b.MultiplySign(orientation);
            Squared(rx, ry, rz, c, term, pointRoot);
            ContactQuadratic.Scale(squaredDenominator, radiusSquared, term); c.Add(term, -1);
            // Remove only a common positive scale. This preserves the original
            // quadratic field and every root/hemisphere certificate, while
            // avoiding fixed-point powers of two in discriminant products.
            for (int index = 0; index < coefficientSigns.Length; index++)
                coefficientSigns[index] = (sbyte)signs[index];
            WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values[..(6 * words)], coefficientSigns);
            bool zeroConstant = c.Sign(pointRoot) == 0, repeated = false;
            if (!rational && !zeroConstant)
                repeated = IsRepeatedRoot(a, b, c, pointRoot);
            bool valueRoot = !rational && !zeroConstant && !repeated;
            if (valueRoot)
            {
                if (!value.TrySetUpperRoot(a, b, c, pointRoot, GetDepthValueShift(frame))) continue;
                // Keep the original section field: the two radicals can be
                // independent even when the exit depth itself is quadratic.
                y.CopyTo(term); ContactQuadratic.Scale(denominator, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), extent);
                term.Add(extent, -1);
                ContactQuadratic.Scale(denominator, Signed576.ExtendValue(frame.Normal.Y), work); work.MultiplySign(orientation);
                int upper = value.SignLinear(term, work, pointRoot);
                if (body)
                {
                    if (upper > 0) continue;
                    y.CopyTo(term); term.Add(extent);
                    if (value.SignLinear(term, work, pointRoot) < 0) continue;
                }
                else if (primitive == 1)
                {
                    y.CopyTo(term); term.Add(extent);
                    if (value.SignLinear(term, work, pointRoot) > 0) continue;
                }
                else if (frame.Finite.Length != Fixed64.Zero && upper < 0) continue;
                selection.KeepValue(x, y, z, denominator, pointRoot, value, primitive, frame); found = true;
                // A positive-quadratic body exit inside the closed core interval
                // is the unique outward capsule exit, including the C1 seam.
                if (body) return true;
                continue;
            }
            if (rational && !zeroConstant)
            {
                if (!ContactQuadratic.TryGetParameter(a, b, c, 1, root, numerator, divisor)) continue;
            }
            else
            {
                // CopyTo's multiplier changes only signs. Scale the magnitude
                // as well for the nonzero exit of a*t²+2*b*t=0.
                ContactQuadratic.Scale(b, Integer(repeated ? -1 : -2), numerator); a.CopyTo(divisor);
                if (numerator.Sign(pointRoot) < 0) numerator.Clear();
                pointRoot.CopyTo(root); root[pointRoot.Length..].Clear();
            }
            if (numerator.Sign(root) < 0 || divisor.Sign(root) <= 0) continue;
            ContactQuadratic.Multiply(denominator, divisor, root, pd);
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic source = axis == 0 ? x : axis == 1 ? y : z;
                ContactQuadratic target = axis == 0 ? px : axis == 1 ? py : pz;
                ContactQuadratic.Multiply(source, divisor, root, target);
                ContactQuadratic.Multiply(denominator, numerator, root, term);
                Signed320 direction = axis == 0 ? frame.Normal.X : axis == 1 ? frame.Normal.Y : frame.Normal.Z;
                ContactQuadratic.Scale(term, Signed576.ExtendValue(direction), work); target.Add(work, orientation);
            }
            ContactQuadratic.Scale(pd, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), extent);
            py.CopyTo(term); term.Add(extent, -1);
            if (body)
            {
                if (term.Sign(root) > 0) continue;
                py.CopyTo(term); term.Add(extent);
                if (term.Sign(root) < 0) continue;
            }
            else if (primitive == 1)
            {
                py.CopyTo(term); term.Add(extent);
                if (term.Sign(root) > 0) continue;
            }
            else if (frame.Finite.Length != Fixed64.Zero && term.Sign(root) < 0) continue;
            selection.Keep(x, y, z, denominator, numerator, divisor, root, primitive, frame); found = true;
            if (body) return true;
        }
        // Rational support projections have Q<727 and D<528 bits; their ray
        // discriminants are <3050 bits. Perpendicular-edge stationary points
        // use u=-b/a directly: Q<600 and D<400 give discriminants <2533 bits.
        // The existing 4096-bit coefficients and 3072-bit root bank own both.
        return found;
    }

    private static bool IsRepeatedRoot(scoped ContactQuadratic a, scoped ContactQuadratic b,
        scoped ContactQuadratic c, scoped ReadOnlySpan<ulong> root)
    {
        // Before cancellation a quadratic-field product can contain two
        // coefficient magnitudes and the entire point radicand. The compact
        // ray banks are not a sufficient discriminant scratch width.
        int words = 2 * Math.Max(ContactQuadratic.ActiveWords(a),
            Math.Max(ContactQuadratic.ActiveWords(b), ContactQuadratic.ActiveWords(c)))
            + WideArithmetic.GetActiveMagnitudeLength(root) + 1;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic discriminant = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic product = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic.Multiply(b, b, root, discriminant);
        ContactQuadratic.Multiply(a, c, root, product); discriminant.Add(product, -1);
        return discriminant.Sign(root) == 0;
    }

    private static bool ContainsQuadraticPoint(in CapsulePlaneRayFrame frame,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator,
        Signed576 radiusSquared, ContactQuadratic squaredDenominator, ContactQuadratic rx,
        ContactQuadratic ry, ContactQuadratic rz, ContactQuadratic value, ContactQuadratic term, ContactQuadratic work,
        ReadOnlySpan<ulong> root)
    {
        Dot(x, y, z, frame.Normal, value, term);
        ContactQuadratic.Scale(denominator, frame.PlaneConstant, term); value.Add(term, -1);
        if (value.Sign(root) != 0) return false;
        x.CopyTo(rx); y.CopyTo(ry); z.CopyTo(rz);
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), term);
        y.CopyTo(value); value.Add(term, -1);
        if (value.Sign(root) > 0) ry.Add(term, -1);
        else
        {
            y.CopyTo(value); value.Add(term);
            if (value.Sign(root) < 0) ry.Add(term);
            else ry.Clear();
        }
        Squared(rx, ry, rz, value, work, root);
        ContactQuadratic.Scale(squaredDenominator, radiusSquared, term); value.Add(term, -1);
        return value.Sign(root) <= 0;
    }

    private static bool ContainsPoint(in CapsulePlaneRayFrame frame, scoped ContactQuadratic x,
        scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator,
        scoped ReadOnlySpan<ulong> root)
    {
        // Private producers construct positive denominators; segment producers
        // additionally certify their finite parameter before reaching this owner.
        int words = GetPointRayWords(frame, x, y, z, denominator, root);
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        ContactQuadratic d2 = Field(values, signs, 0, words), rx = Field(values, signs, 1, words), ry = Field(values, signs, 2, words);
        ContactQuadratic rz = Field(values, signs, 3, words), value = Field(values, signs, 4, words);
        ContactQuadratic term = Field(values, signs, 5, words), work = Field(values, signs, 6, words);
        ContactQuadratic.Multiply(denominator, denominator, root, d2);
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius, frame.Finite.ShapeFrame.Denominator);
        return ContainsQuadraticPoint(frame, x, y, z, denominator, WideArithmetic.MultiplySigned320(radius, radius),
            d2, rx, ry, rz, value, term, work, root);
    }

    private static int GetSegmentWords(in CapsulePlaneRayFrame frame, WideAxis3 start, WideAxis3 end, WideAxis3 delta)
    {
        Span<int> scalarBits = stackalloc int[6]; GetGeometryScalarBits(frame, scalarBits);
        Span<ulong> magnitude = stackalloc ulong[9];
        int point = 0, edge = 0;
        for (int axis = 0; axis < 3; axis++)
        {
            Signed320 first = axis == 0 ? start.X : axis == 1 ? start.Y : start.Z;
            Signed320 last = axis == 0 ? end.X : axis == 1 ? end.Y : end.Z;
            Signed320 difference = axis == 0 ? delta.X : axis == 1 ? delta.Y : delta.Z;
            WideArithmetic.GetMagnitude(Signed576.ExtendValue(first), magnitude);
            point = Math.Max(point, WideArithmetic.GetMagnitudeBitLength(magnitude));
            WideArithmetic.GetMagnitude(Signed576.ExtendValue(last), magnitude);
            point = Math.Max(point, WideArithmetic.GetMagnitudeBitLength(magnitude));
            WideArithmetic.GetMagnitude(Signed576.ExtendValue(difference), magnitude);
            edge = Math.Max(edge, WideArithmetic.GetMagnitudeBitLength(magnitude));
        }
        int basis = Math.Max(point, scalarBits[0]) + 1;
        int normal = Math.Max(scalarBits[2], Math.Max(scalarBits[3], scalarBits[4]));
        // B bounds cap-relative points, E the edge, N the normal and R the
        // radius. Section/stationary coefficients need 2E+2max(B,R)+6 bits;
        // recovered q and its complete exit/cap-incidence products need
        // B+6E+4N+16. Their denominators are rational, so these bank products
        // never contain two radical factors. Discriminants use separate scratch.
        // Zero quadratic coefficients describe constant families: projected
        // delta=0 or Cauchy equality also annihilates the linear coefficient.
        int bits = Math.Max(2 * edge + 2 * Math.Max(basis, scalarBits[1]) + 6,
            basis + 6 * edge + 4 * normal + 16);
        return Math.Max(9, Math.Min(Words, (bits + 4 + 63) / 64));
    }

    private static int GetProducerWords(in CapsulePlaneRayFrame frame, CapsulePlaneRayEventKind kind)
    {
        Span<int> scalarBits = stackalloc int[6]; GetGeometryScalarBits(frame, scalarBits);
        int normal = Math.Max(scalarBits[2], Math.Max(scalarBits[3], scalarBits[4]));
        Span<ulong> magnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(frame.NormalSquared, magnitude);
        int metric = WideArithmetic.GetMagnitudeBitLength(magnitude), gap = scalarBits[5];
        Signed576 end = WideArithmetic.MultiplySigned320(frame.Normal.Y, frame.Finite.ShapeFrame.Cap);
        for (int orientation = -1; orientation <= 1; orientation += 2)
        {
            Signed576 value = orientation < 0 ? WideArithmetic.SubtractSigned576(frame.PlaneConstant, end)
                : WideArithmetic.AddSigned576(frame.PlaneConstant, end);
            WideArithmetic.GetMagnitude(value, magnitude);
            gap = Math.Max(gap, WideArithmetic.GetMagnitudeBitLength(magnitude));
        }
        int bits;
        if (kind == CapsulePlaneRayEventKind.Support)
            bits = Math.Max(gap + normal, metric + scalarBits[0]) + 1;
        else
        {
            // H owns the complete r²*S-gap² before cancellation. Since T<=S,
            // intrinsic transverse radicands need H+2S; cap q needs P+2S+1.
            int section = Math.Max(2 * scalarBits[1] + metric, 2 * gap) + 1;
            if (kind == CapsulePlaneRayEventKind.Circle)
            {
                int authored = 0;
                for (int axis = 0; axis < 3; axis++)
                {
                    Signed320 component = axis == 0 ? frame.AuthoredNormal.X
                        : axis == 1 ? frame.AuthoredNormal.Y : frame.AuthoredNormal.Z;
                    WideArithmetic.GetMagnitude(Signed576.ExtendValue(component), magnitude);
                    authored = Math.Max(authored, WideArithmetic.GetMagnitudeBitLength(magnitude));
                }
                WideArithmetic.GetMagnitude(Signed576.ExtendValue(Signed320.ExtendValue(frame.Finite.ShapeFrame.Basis.Denominator)), magnitude);
                // The authored tangents use at most two normal products. A
                // rigid numerator is bounded by its positive denominator, so
                // transformed V<=2A+B+3 and its three-term square W<=2V+2.
                int tangent = 2 * authored + WideArithmetic.GetMagnitudeBitLength(magnitude) + 3;
                int tangentSquared = 2 * tangent + 2;
                bits = Math.Max(section + metric + tangentSquared, scalarBits[0] + metric + tangentSquared + 1);
            }
            else bits = Math.Max(section + 2 * metric, scalarBits[0] + 2 * metric + 1);
        }
        // Four spare carry bits cover all sums; complete Signed576 imports
        // still require nine words. Retained point/root banks stay unchanged.
        return Math.Max(9, Math.Min(Words, (bits + 4 + 63) / 64));
    }

    private static void GetGeometryScalarBits(in CapsulePlaneRayFrame frame, Span<int> bits)
    {
        Span<ulong> magnitude = stackalloc ulong[9];
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius, frame.Finite.ShapeFrame.Denominator);
        for (int scalar = 0; scalar < bits.Length; scalar++)
        {
            Signed576 value = scalar == 0 ? Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap)
                : scalar == 1 ? Signed576.ExtendValue(radius) : scalar == 2 ? Signed576.ExtendValue(frame.Normal.X)
                : scalar == 3 ? Signed576.ExtendValue(frame.Normal.Y) : scalar == 4 ? Signed576.ExtendValue(frame.Normal.Z)
                : frame.PlaneConstant;
            WideArithmetic.GetMagnitude(value, magnitude);
            bits[scalar] = WideArithmetic.GetMagnitudeBitLength(magnitude);
        }
    }

    private static int GetPointRayWords(in CapsulePlaneRayFrame frame, scoped ContactQuadratic x,
        scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator,
        scoped ReadOnlySpan<ulong> root)
    {
        int coefficientBits = 0;
        for (int coordinate = 0; coordinate < 4; coordinate++)
        {
            ContactQuadratic value = coordinate == 0 ? x : coordinate == 1 ? y : coordinate == 2 ? z : denominator;
            coefficientBits = Math.Max(coefficientBits, Math.Max(WideArithmetic.GetMagnitudeBitLength(value.Rational),
                WideArithmetic.GetMagnitudeBitLength(value.Radical)));
        }
        Span<int> geometryBits = stackalloc int[6]; GetGeometryScalarBits(frame, geometryBits);
        int scalarBits = 0;
        for (int scalar = 0; scalar < geometryBits.Length; scalar++) scalarBits = Math.Max(scalarBits, geometryBits[scalar]);
        // C bounds point coefficients, M unsquared geometry scalars, and K the
        // radicand. Complete exit/hemisphere products need 3C+3M+2K+8 bits;
        // two spare carry bits cover sums. Signed576 imports require nine words.
        int bits = 3 * coefficientBits + 3 * scalarBits + 2 * WideArithmetic.GetMagnitudeBitLength(root) + 10;
        return Math.Max(9, Math.Min(Words, (bits + 63) / 64));
    }

    private static int GetDepthValueShift(in CapsulePlaneRayFrame frame)
    {
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius, frame.Finite.ShapeFrame.Denominator);
        Signed320 bound = WideArithmetic.AddSigned320(frame.Finite.ShapeFrame.Cap, radius);
        Span<ulong> magnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(Signed576.ExtendValue(bound), magnitude);
        int boundBits = WideArithmetic.GetMagnitudeBitLength(magnitude);
        WideArithmetic.GetMagnitude(frame.NormalSquared, magnitude);
        int normalPower = (WideArithmetic.GetMagnitudeBitLength(magnitude) - 1) / 2;
        // An admitted exit is at most the capsule diameter divided by |N|.
        // Infinite-body roots beyond that bound cannot pass finite admission.
        return Math.Max(0, boundBits + 2 - normalPower);
    }

    private static void Dot(ContactQuadratic x, ContactQuadratic y, ContactQuadratic z, WideAxis3 direction,
        ContactQuadratic result, ContactQuadratic term)
    {
        ContactQuadratic.Scale(x, Signed576.ExtendValue(direction.X), result);
        ContactQuadratic.Scale(y, Signed576.ExtendValue(direction.Y), term); result.Add(term);
        ContactQuadratic.Scale(z, Signed576.ExtendValue(direction.Z), term); result.Add(term);
    }

    private static void Squared(ContactQuadratic x, ContactQuadratic y, ContactQuadratic z,
        ContactQuadratic result, ContactQuadratic term, ReadOnlySpan<ulong> root = default)
    {
        ContactQuadratic.Multiply(x, x, root, result);
        ContactQuadratic.Multiply(y, y, root, term); result.Add(term);
        ContactQuadratic.Multiply(z, z, root, term); result.Add(term);
    }

    /// <summary>Tests the closed finite triangle against the capsule's exact plane section.</summary>
    internal static bool IntersectsTriangle(in CapsuleFiniteSectionFrame finite, FixedTriangle triangle)
    {
        if (finite.ContainsPoint(triangle.A) || finite.ContainsPoint(triangle.B) || finite.ContainsPoint(triangle.C)) return true;
        for (int edge = 0; edge < 3; edge++)
            if (finite.IntersectsSegment(triangle.GetEdge(edge))) return true;
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        if (frame.NormalSquared.IsZero) return false;
        // With no admitted boundary crossing, an intersecting capsule section
        // is wholly inside the triangle. Its projected nearest core point is
        // an interior certificate; endpoint projection alone is not admission.
        const int words = 20;
        Span<ulong> values = stackalloc ulong[20 * words];
        Span<int> signs = stackalloc int[20];
        ContactQuadratic x = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic y = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic z = ContactQuadratic.At(values, signs, 2, words);
        ContactQuadratic denominator = ContactQuadratic.At(values, signs, 3, words);
        ContactQuadratic gap = ContactQuadratic.At(values, signs, 4, words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 5, words);
        ContactQuadratic left = ContactQuadratic.At(values, signs, 6, words);
        ContactQuadratic right = ContactQuadratic.At(values, signs, 7, words);
        ContactQuadratic squaredRadius = ContactQuadratic.At(values, signs, 8, words);
        ContactQuadratic metric = ContactQuadratic.At(values, signs, 9, words);
        Signed576 projectedEnd = WideArithmetic.MultiplySigned320(frame.Normal.Y, finite.ShapeFrame.Cap);
        Signed576 absoluteEnd = projectedEnd.Sign < 0 ? WideArithmetic.SubtractSigned576(default, projectedEnd) : projectedEnd;
        x.Clear(); y.Clear(); z.Clear();
        bool crossesCore = !frame.Normal.Y.IsZero
            && WideArithmetic.AddSigned576(frame.PlaneConstant, absoluteEnd).Sign >= 0
            && WideArithmetic.SubtractSigned576(frame.PlaneConstant, absoluteEnd).Sign <= 0;
        if (crossesCore)
        {
            y.Set(frame.PlaneConstant); y.MultiplySign(frame.Normal.Y.Sign);
            Signed320 positive = frame.Normal.Y.Sign < 0 ? WideArithmetic.Negate(frame.Normal.Y) : frame.Normal.Y;
            denominator.Set(Signed576.ExtendValue(positive));
        }
        else
        {
            int endpointSign = frame.Normal.Y.IsZero ? 0 : frame.PlaneConstant.Sign * frame.Normal.Y.Sign;
            Signed320 endpoint = endpointSign < 0 ? WideArithmetic.Negate(finite.ShapeFrame.Cap)
                : endpointSign > 0 ? finite.ShapeFrame.Cap : default;
            gap.Set(WideArithmetic.SubtractSigned576(frame.PlaneConstant,
                WideArithmetic.MultiplySigned320(frame.Normal.Y, endpoint)));
            ContactQuadratic.Multiply(gap, gap, ReadOnlySpan<ulong>.Empty, left);
            squaredRadius.Set(WideArithmetic.MultiplySigned320(
                WideArithmetic.MultiplySigned192(finite.ShapeFrame.Radius, finite.ShapeFrame.Denominator),
                WideArithmetic.MultiplySigned192(finite.ShapeFrame.Radius, finite.ShapeFrame.Denominator)));
            metric.Set(frame.NormalSquared);
            ContactQuadratic.Multiply(squaredRadius, metric, ReadOnlySpan<ulong>.Empty, right);
            left.Add(right, -1);
            if (left.Sign(ReadOnlySpan<ulong>.Empty) > 0) return false;
            ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.X), x);
            ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Y), y);
            ContactQuadratic.Scale(gap, Signed576.ExtendValue(frame.Normal.Z), z);
            denominator.Set(frame.NormalSquared);
            ContactQuadratic.Scale(denominator, Signed576.ExtendValue(endpoint), term); y.Add(term);
        }
        // Rational witness coordinates are below 727 bits; gap squares and
        // radius²*normal² below 930 bits fit the 1280-bit local bank.
        return frame.ContainsTrianglePoint(triangle, x, y, z, denominator, ReadOnlySpan<ulong>.Empty);
    }
}
