//=======================================================================
// WideConvexPrismRelations.CapsuleEdges.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Finite capsule-core/prism-edge matching when a canonical support projection is empty.</content>
internal static partial class WideConvexPrismRelations
{
    private const int PrismEdgeWords = 32;

    private static bool TryGetPrismEdgeContact(RigidAxis3 shapeAxis, Vector3d center,
        FixedQuaternion shapeRotation, Fixed64 length, Fixed64 radius,
        Vector3d prismOrigin, Fixed64 prismYaw, FixedQuaternion prismOrientation,
        ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness, out FixedContactAnchors contact,
        IPrismCapsuleContactDomain? domain = null)
    {
        contact = default;
        Signed192 twiceRotation = WideArithmetic.AddSigned192(shapeAxis.RotationDenominator, shapeAxis.RotationDenominator);
        Signed192 coordinateD = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(twiceRotation, Scale));
        ReadOnlySpan<Signed192> axis = stackalloc Signed192[3] { shapeAxis.X, shapeAxis.Y, shapeAxis.Z };
        Span<Signed192> core = stackalloc Signed192[3], u = stackalloc Signed192[3];
        for (int component = 0; component < 3; component++)
        {
            Signed320 midpoint = WideArithmetic.MultiplySigned192(WideArithmetic.Difference(center[component], prismOrigin[component]), coordinateD);
            Signed320 half = WideArithmetic.MultiplySigned192(axis[component], Signed192.Raw(length));
            core[component] = Signed192.NarrowProven(WideArithmetic.SubtractSigned320(midpoint, half));
            u[component] = Signed192.NarrowProven(WideArithmetic.AddSigned320(half, half));
        }
        // Ordinary full-domain endpoints/core use <165 bits, Gram terms <330
        // and determinant <656. Compact native clipping adds at most 320 bits
        // to one Gram term; retained point numerators still fit Signed832.
        // Filtered candidates retain their gap while the incident-fan checks borrow scratch.
        int banks = domain == null ? 12 : 16;
        Span<ulong> values = stackalloc ulong[banks * PrismEdgeWords]; Span<int> signs = stackalloc int[banks];
        Span<Signed832> best = stackalloc Signed832[7];
        var yaw = new WideConvex2dRelations.RotationFrame2d(prismYaw);
        var selection = new PrismEdgeSelection(core, u, coordinateD, values, signs, best,
            domain, offsets, halfThickness, yaw, twiceRotation, radius);
        Span<Signed192> first = stackalloc Signed192[3], second = stackalloc Signed192[3];
        Span<Signed192> v = stackalloc Signed192[3], w = stackalloc Signed192[3];
        Span<Signed832> previousTarget = stackalloc Signed832[4];
        Signed576 aa = PrismEdgeDot(u, u);
        bool any = false;
        for (int index = 0; index < offsets.Length; index++)
        {
            int next = index + 1 == offsets.Length ? 0 : index + 1;
            for (int kind = 0; kind < 3; kind++)
            {
                GetPrismEdgeVertex(yaw, offsets[index], kind == 1 ? halfThickness : -halfThickness, twiceRotation, first);
                GetPrismEdgeVertex(yaw, kind == 2 ? offsets[index] : offsets[next],
                    kind == 0 ? -halfThickness : halfThickness, twiceRotation, second);
                for (int component = 0; component < 3; component++)
                {
                    v[component] = WideArithmetic.SubtractSigned192(second[component], first[component]);
                    w[component] = WideArithmetic.SubtractSigned192(core[component], first[component]);
                }
                selection.SetEdge(first, v, index, kind);
                bool hasPreviousTarget = false;
                Signed576 bb = PrismEdgeDot(v, v), ab = PrismEdgeDot(u, v);
                Signed576 ao = PrismEdgeDot(u, w), bo = PrismEdgeDot(v, w);
                ReadOnlySpan<RationalSegmentInterval> intervals = domain == null ? FullPrismEdgeInterval : domain.GetEdgeIntervals(index, kind);
                for (int interval = 0; interval < intervals.Length; interval++)
                {
                    if (domain != null) selection.Reset();
                    RationalSegmentInterval clip = intervals[interval];
                    if (bb.IsZero)
                    {
                        Signed576 d = aa.IsZero ? Signed576.ExtendValue(Signed192.Signed(1)) : aa;
                        selection.Keep(Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.Negate(ao), d)), default, Signed832.ExtendValue(d));
                    }
                    else
                    {
                        KeepPrismEdgeCandidates(ref selection, aa, bb, ab, ao, bo, clip, domain != null);
                    }
                    if (domain != null && selection.TryGetContact(radius, center, shapeRotation, prismOrigin, prismOrientation, out var physical))
                    {
                        // Closed adjacent height bands can share one unique
                        // closest point; it is one physical constraint.
                        if (hasPreviousTarget && selection.SameTarget(previousTarget)) continue;
                        var native = new CapsulePrismContactDepth(selection.BestQ, selection.BestD, radius);
                        domain.AddEdgeContact(physical, native, selection.BestNormal, index, kind, interval);
                        selection.CopyTarget(previousTarget); hasPreviousTarget = true;
                        contact = physical; any = true;
                    }
                }
            }
        }
        if (domain != null) return any;
        System.Diagnostics.Debug.Assert(selection.Found && selection.BestQ.Signs[0] > 0);
        return selection.TryGetContact(radius, center, shapeRotation, prismOrigin, prismOrientation, out contact);
    }

    private static readonly RationalSegmentInterval[] FullPrismEdgeInterval = { default };

    private static void KeepPrismEdgeCandidates(ref PrismEdgeSelection selection,
        Signed576 aa, Signed576 bb, Signed576 ab, Signed576 ao, Signed576 bo,
        in RationalSegmentInterval clip, bool filtered)
    {
        Signed832 ln = default, ld = Signed832.ExtendValue(Signed192.Signed(1)), un = ld, ud = ld;
        if (clip.IsClipped)
        { ln = clip.WideLowerNumerator; ld = clip.WideLowerDenominator; un = clip.WideUpperNumerator; ud = clip.WideUpperDenominator; }
        Signed832 tn, td;
        if (!clip.IsClipped)
        { tn = Signed832.ExtendValue(ClampPrismEdgeParameter(bo, bb)); td = Signed832.ExtendValue(bb); }
        else GetClippedEdgeParameter(Signed832.ExtendValue(bo), Signed832.ExtendValue(bb), ln, ld, un, ud, out tn, out td);
        selection.Keep(default, tn, td);
        if (aa.IsZero) return;
        if (!clip.IsClipped)
        { tn = Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.AddSigned576(bo, ab), bb)); td = Signed832.ExtendValue(bb); }
        else GetClippedEdgeParameter(Signed832.ExtendValue(WideArithmetic.AddSigned576(bo, ab)), Signed832.ExtendValue(bb),
            ln, ld, un, ud, out tn, out td);
        selection.Keep(td, tn, td);
        if (!clip.IsClipped)
        {
            selection.Keep(Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.Negate(ao), aa)), default, Signed832.ExtendValue(aa));
            selection.Keep(Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.SubtractSigned576(ab, ao), aa)),
                Signed832.ExtendValue(aa), Signed832.ExtendValue(aa));
        }
        else
        {
            KeepPrismEdgeEndpoint(ref selection, aa, ab, ao, clip.LowerNumerator, clip.LowerDenominator);
            KeepPrismEdgeEndpoint(ref selection, aa, ab, ao, clip.UpperNumerator, clip.UpperDenominator);
        }
        Signed576 twiceBb = WideArithmetic.AddSigned576(bb, bb);
        if (!clip.IsClipped)
            selection.Keep(Signed832.ExtendValue(bb),
                Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(bo, bo), ab), twiceBb)),
                Signed832.ExtendValue(twiceBb));
        else
        {
            GetClippedEdgeParameter(Signed832.ExtendValue(WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(bo, bo), ab)),
                Signed832.ExtendValue(twiceBb), ln, ld, un, ud, out tn, out td);
            selection.Keep(td, WideArithmetic.AddSigned832(tn, tn), WideArithmetic.AddSigned832(td, td));
        }
        Signed832 determinant = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(aa, bb), WideArithmetic.MultiplySigned576ToSigned832(ab, ab));
        if (determinant.Sign > 0)
        {
            Signed832 s = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(ab, bo),
                WideArithmetic.MultiplySigned576ToSigned832(bb, ao));
            Signed832 t = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(aa, bo),
                WideArithmetic.MultiplySigned576ToSigned832(ab, ao));
            if (s.Sign < 0 || t.Sign < 0 || WideArithmetic.SubtractSigned832(determinant, s).Sign < 0
                || WideArithmetic.SubtractSigned832(determinant, t).Sign < 0
                || clip.IsClipped && (RationalSegmentInterval.CompareParameters(t, determinant, ln, ld) < 0
                    || RationalSegmentInterval.CompareParameters(t, determinant, un, ud) > 0)) return;
            selection.Keep(s, t, determinant);
        }
        else if (filtered
            && WideTriangleRelations.ClipScaledHalfPlane(Signed832.ExtendValue(WideArithmetic.Negate(ao)),
                Signed832.ExtendValue(WideArithmetic.SubtractSigned576(ab, ao)), ref ln, ref ld, ref un, ref ud)
            && WideTriangleRelations.ClipScaledHalfPlane(Signed832.ExtendValue(WideArithmetic.AddSigned576(aa, ao)),
                Signed832.ExtendValue(WideArithmetic.SubtractSigned576(WideArithmetic.AddSigned576(aa, ao), ab)),
                ref ln, ref ld, ref un, ref ud))
        {
            // An endpoint fan can reject the old midpoint although an open
            // parallel closest family remains exposed. Retain its true interior.
            GetPrismInteriorParameter(new RationalSegmentInterval(ln, ld, un, ud), out tn, out td);
            KeepPrismEdgeEndpoint(ref selection, aa, ab, ao, NarrowPrismEdgeParameter(tn), NarrowPrismEdgeParameter(td));
        }
    }

    private static Signed320 NarrowPrismEdgeParameter(Signed832 value) =>
        new(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0);

    private static void GetClippedEdgeParameter(Signed832 n, Signed832 d,
        Signed832 ln, Signed832 ld, Signed832 un, Signed832 ud, out Signed832 resultN, out Signed832 resultD)
    {
        resultN = n; resultD = d;
        if (n.Sign < 0 || RationalSegmentInterval.CompareParameters(n, d, ln, ld) < 0)
        { resultN = ln; resultD = ld; }
        else if (RationalSegmentInterval.CompareParameters(n, d, un, ud) > 0)
        { resultN = un; resultD = ud; }
    }

    private static void KeepPrismEdgeEndpoint(ref PrismEdgeSelection selection, Signed576 aa, Signed576 ab, Signed576 ao,
        Signed320 tn, Signed320 td)
    {
        Signed832 d = WideArithmetic.MultiplySigned576ToSigned832(aa, td);
        Signed832 s = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(ab, tn),
            WideArithmetic.MultiplySigned576ToSigned832(ao, td));
        if (s.Sign < 0) s = default;
        else if (WideArithmetic.SubtractSigned832(s, d).Sign > 0) s = d;
        selection.Keep(s, WideArithmetic.MultiplySigned576ToSigned832(aa, tn), d);
    }

    private ref struct PrismEdgeSelection
    {
        private readonly ReadOnlySpan<Signed192> _core, _u;
        private readonly Signed192 _coordinateD, _twiceRotation, _radiusSquared;
        private readonly ContactQuadratic _currentQ, _currentD, _term, _product;
        private readonly ContactQuadratic _gapQ, _gapD;
        internal readonly ContactQuadratic BestQ, BestD;
        private readonly Span<Signed832> _best;
        private readonly IPrismCapsuleContactDomain? _domain;
        private readonly ReadOnlySpan<Vector2d> _offsets;
        private readonly Fixed64 _half;
        private readonly WideConvex2dRelations.RotationFrame2d _yaw;
        private ReadOnlySpan<Signed192> _edge, _v;
        private int _vertex, _kind;
        private Signed832 _middleN, _middleD;
        internal bool Found;
        internal WideDirection3d BestNormal;

        internal PrismEdgeSelection(ReadOnlySpan<Signed192> core, ReadOnlySpan<Signed192> u, Signed192 coordinateD,
            Span<ulong> values, Span<int> signs, Span<Signed832> best, IPrismCapsuleContactDomain? domain,
            ReadOnlySpan<Vector2d> offsets, Fixed64 half, WideConvex2dRelations.RotationFrame2d yaw, Signed192 twiceRotation, Fixed64 radius)
        {
            this = default; _core = core; _u = u; _coordinateD = coordinateD; _best = best; _domain = domain;
            _offsets = offsets; _half = half; _yaw = yaw; _twiceRotation = twiceRotation;
            // A nonnegative scalar radius has at most 63 magnitude bits; its exact
            // raw square fits Signed192 and is shared by all retained candidates.
            _radiusSquared = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(Signed192.Raw(radius), Signed192.Raw(radius)));
            BestQ = ContactQuadratic.At(values, signs, 0, PrismEdgeWords); BestD = ContactQuadratic.At(values, signs, 1, PrismEdgeWords);
            _currentQ = ContactQuadratic.At(values, signs, 2, PrismEdgeWords); _currentD = ContactQuadratic.At(values, signs, 3, PrismEdgeWords);
            _term = ContactQuadratic.At(values, signs, 4, PrismEdgeWords); _product = ContactQuadratic.At(values, signs, 5, PrismEdgeWords);
            _gapQ = domain == null ? _currentQ : ContactQuadratic.At(values, signs, 6, PrismEdgeWords);
            _gapD = domain == null ? _currentD : ContactQuadratic.At(values, signs, 7, PrismEdgeWords);
        }

        internal void SetEdge(ReadOnlySpan<Signed192> edge, ReadOnlySpan<Signed192> v, int vertex, int kind)
        { _edge = edge; _v = v; _vertex = vertex; _kind = kind; }

        internal void Reset() { Found = false; _middleN = _middleD = default; }

        internal void Keep(Signed832 s, Signed832 t, Signed832 d)
        {
            // Each prism cap vertex belongs to its original vertical edge.
            // Horizontal edge endpoints must not emit duplicate corner rows.
            if (_domain != null && _kind != 2 && (t.IsZero || WideArithmetic.SubtractSigned832(t, d).IsZero)) return;
            Span<Signed832> points = stackalloc Signed832[7];
            for (int component = 0; component < 3; component++)
            {
                points[component] = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned832(d, _core[component]),
                    WideArithmetic.MultiplySigned832(s, _u[component]));
                points[component + 3] = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned832(d, _edge[component]),
                    WideArithmetic.MultiplySigned832(t, _v[component]));
            }
            points[6] = WideArithmetic.MultiplySigned832(d, _coordinateD);
            _gapQ.Clear();
            for (int component = 0; component < 3; component++)
            {
                _term.Set(WideArithmetic.SubtractSigned832(points[component + 3], points[component]));
                ContactQuadratic.Multiply(_term, _term, ReadOnlySpan<ulong>.Empty, _product); _gapQ.Add(_product);
            }
            _term.Set(points[6]); ContactQuadratic.Multiply(_term, _term, ReadOnlySpan<ulong>.Empty, _gapD);
            // Most unrelated retained edges are outside the capsule radius.
            // Reject those before GCD reduction, whole-solid support and fan tests.
            WideDirection3d normal = default;
            if (_domain != null && (!WithinRadius(_gapQ, _gapD) || !TryGetPrismEdgeDirection(points, out normal)
                || !Supports(points, d, normal) || !_domain.AdmitEdgePoint(_vertex, _kind, t, d, normal))) return;
            int comparison = Found ? ContactQuadratic.CompareRatios(_gapQ, _gapD, ReadOnlySpan<ulong>.Empty,
                BestQ, BestD, ReadOnlySpan<ulong>.Empty) : -1;
            Signed832 middle = WideArithmetic.SubtractSigned832(WideArithmetic.AddSigned832(s, s), d);
            if (middle.Sign < 0) middle = WideArithmetic.SubtractSigned832(default, middle);
            if (comparison > 0 || comparison == 0 && WideArithmetic.CompareNonNegativeProducts(middle, _middleD, _middleN, d) >= 0) return;
            _gapQ.CopyTo(BestQ); _gapD.CopyTo(BestD); points.CopyTo(_best);
            _middleN = middle; _middleD = d; Found = true; BestNormal = normal;
        }

        internal bool SameTarget(scoped ReadOnlySpan<Signed832> other)
        {
            _currentD.Set(_best[6]); _product.Set(other[3]);
            for (int axis = 0; axis < 3; axis++)
            {
                _currentQ.Set(_best[axis + 3]); _term.Set(other[axis]);
                if (ContactQuadratic.CompareRatios(_currentQ, _currentD, ReadOnlySpan<ulong>.Empty,
                    _term, _product, ReadOnlySpan<ulong>.Empty) != 0) return false;
            }
            return true;
        }

        internal void CopyTarget(scoped Span<Signed832> target)
        {
            _best.Slice(3, 3).CopyTo(target); target[3] = _best[6];
        }

        internal bool TryGetContact(Fixed64 radius, Vector3d center, FixedQuaternion rotation,
            Vector3d origin, FixedQuaternion prismRotation, out FixedContactAnchors contact)
        {
            contact = default;
            if (!Found) return false;
            if (!WithinRadius(BestQ, BestD)) return false;
            contact = MaterializePrismEdgeContact(_best, BestQ.Rational, center, rotation, origin, prismRotation, radius);
            return true;
        }

        private bool WithinRadius(scoped ContactQuadratic squared, scoped ContactQuadratic denominator)
        {
            _term.Set(Signed576.ExtendValue(_radiusSquared));
            ContactQuadratic.Multiply(_term, denominator, ReadOnlySpan<ulong>.Empty, _product);
            _product.Add(squared, -1);
            return _product.Signs[0] >= 0;
        }

        private bool Supports(scoped ReadOnlySpan<Signed832> points, Signed832 d, in WideDirection3d normal)
        {
            Span<Signed192> vertex = stackalloc Signed192[3];
            if (!SupportsVertex(_core, points[..3], d, normal, true)) return false;
            for (int axis = 0; axis < 3; axis++) vertex[axis] = WideArithmetic.AddSigned192(_core[axis], _u[axis]);
            if (!SupportsVertex(vertex, points[..3], d, normal, true)) return false;
            foreach (Vector2d offset in _offsets)
                for (int sign = -1; sign <= 1; sign += 2)
                {
                    GetPrismEdgeVertex(_yaw, offset, sign < 0 ? -_half : _half, _twiceRotation, vertex);
                    if (!SupportsVertex(vertex, points[3..6], d, normal, false)) return false;
                }
            return true;
        }

        private bool SupportsVertex(scoped ReadOnlySpan<Signed192> vertex, scoped ReadOnlySpan<Signed832> selected,
            Signed832 d, in WideDirection3d normal, bool maximum)
        {
            _currentQ.Clear();
            Signed832 projection = default;
            for (int axis = 0; axis < 3; axis++)
            {
                Signed576 component = axis == 0 ? normal.X : axis == 1 ? normal.Y : normal.Z;
                projection = WideArithmetic.AddSigned832(projection, WideArithmetic.MultiplySigned576ToSigned832(component, Signed320.ExtendValue(vertex[axis])));
                _term.Set(selected[axis]); ContactQuadratic.Scale(_term, component, _product); _currentQ.Add(_product);
            }
            _term.Set(d); ContactQuadratic.Scale(_term, projection, _currentD); _currentD.Add(_currentQ, -1);
            return maximum ? _currentD.Signs[0] <= 0 : _currentD.Signs[0] >= 0;
        }
    }

    private static bool TryGetPrismEdgeDirection(ReadOnlySpan<Signed832> points, out WideDirection3d direction)
    {
        direction = default;
        Span<Signed832> delta = stackalloc Signed832[3];
        Span<ulong> content = stackalloc ulong[13], magnitude = stackalloc ulong[13], quotient = stackalloc ulong[13],
            remainder = stackalloc ulong[13], scratch = stackalloc ulong[27];
        content.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            delta[axis] = WideArithmetic.SubtractSigned832(points[axis + 3], points[axis]);
            WideArithmetic.GetMagnitude(delta[axis], magnitude);
            if (WideArithmetic.GetActiveMagnitudeLength(content) == 0) magnitude.CopyTo(content);
            else WideArithmetic.GetMagnitudeGreatestCommonDivisor(content, magnitude, scratch);
        }
        if (WideArithmetic.GetActiveMagnitudeLength(content) == 0) return false;
        Span<Signed576> normal = stackalloc Signed576[3];
        for (int axis = 0; axis < 3; axis++)
        {
            WideArithmetic.GetMagnitude(delta[axis], magnitude);
            WideArithmetic.DivideMagnitudes(magnitude, content, quotient, remainder, scratch);
            // For bounded rigid core/edge pairs, a primitive residual is either
            // the line cross (<300 bits) or endpoint projection (<450 bits).
            var value = new Signed576(quotient[8], quotient[7], quotient[6], quotient[5], quotient[4], quotient[3], quotient[2], quotient[1], quotient[0]);
            normal[axis] = delta[axis].Sign < 0 ? WideArithmetic.Negate(value) : value;
        }
        direction = new WideDirection3d(normal[0], normal[1], normal[2]); return true;
    }

    private static void GetPrismEdgeVertex(WideConvex2dRelations.RotationFrame2d yaw, Vector2d offset,
        Fixed64 y, Signed192 twiceRotation, Span<Signed192> point)
    {
        WideConvex2dRelations.GetRotatedOffset(yaw, offset, out Signed192 x, out Signed192 z);
        point[0] = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(x, twiceRotation));
        point[1] = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            Signed192.NarrowProven(WideArithmetic.MultiplySigned192(Signed192.Raw(y), Scale)), twiceRotation));
        point[2] = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(z, twiceRotation));
    }

    private static Signed576 PrismEdgeDot(ReadOnlySpan<Signed192> first, ReadOnlySpan<Signed192> second) =>
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(first[0]), Signed320.ExtendValue(second[0])),
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(first[1]), Signed320.ExtendValue(second[1]))),
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(first[2]), Signed320.ExtendValue(second[2])));

    private static Signed576 ClampPrismEdgeParameter(Signed576 value, Signed576 denominator) =>
        value.Sign < 0 ? default : WideArithmetic.SubtractSigned576(value, denominator).Sign > 0 ? denominator : value;

    private static FixedContactAnchors MaterializePrismEdgeContact(ReadOnlySpan<Signed832> points,
        ReadOnlySpan<ulong> root, Vector3d center, FixedQuaternion rotation,
        Vector3d prismOrigin, FixedQuaternion prismRotation, Fixed64 radius)
    {
        Span<ulong> normals = stackalloc ulong[7 * PrismEdgeWords]; Span<int> normalSigns = stackalloc int[7];
        normals.Clear(); normalSigns.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            Signed832 delta = WideArithmetic.SubtractSigned832(points[axis + 3], points[axis]);
            WideArithmetic.GetMagnitude(delta, normals.Slice(axis * PrismEdgeWords, PrismEdgeWords)); normalSigns[axis] = delta.Sign;
        }
        Vector3d normal = GetConvexContactCandidateNormal(new ConvexContactCandidate(normals, normalSigns, 1, PrismEdgeWords));
        Span<ulong> values = stackalloc ulong[6 * PrismEdgeWords]; Span<int> signs = stackalloc int[6];
        ContactQuadratic denominator = ContactQuadratic.At(values, signs, 0, PrismEdgeWords);
        ContactQuadratic numerator = ContactQuadratic.At(values, signs, 1, PrismEdgeWords);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 2, PrismEdgeWords);
        denominator.Set(points[6]); numerator.Clear();
        term.Set(Signed576.ExtendValue(Signed192.Raw(radius)));
        ContactQuadratic.Multiply(denominator, term, ReadOnlySpan<ulong>.Empty, numerator);
        numerator.Radical[0] = 1; numerator.Signs[1] = -1;
        Fixed64 depth = ContactQuadratic.RoundRatio(numerator, denominator, root, 0, long.MaxValue);
        FixedPointAnchor shape = MaterializePrismEdgeAnchor(points, root, center, rotation, prismOrigin, radius, true);
        FixedPointAnchor prism = MaterializePrismEdgeAnchor(points, root, prismOrigin, prismRotation, prismOrigin, radius, false);
        // The core is outside the prism, so 0<=radius-distance<=radius fits.
        return new FixedContactAnchors(shape, prism, normal, depth, false);
    }

    private static FixedPointAnchor MaterializePrismEdgeAnchor(ReadOnlySpan<Signed832> points,
        ReadOnlySpan<ulong> root, Vector3d origin, FixedQuaternion rotation, Vector3d prismOrigin, Fixed64 radius, bool capsule)
    {
        Span<ulong> values = stackalloc ulong[18 * PrismEdgeWords]; Span<int> signs = stackalloc int[18];
        ContactQuadratic denominator = ContactQuadratic.At(values, signs, 3, PrismEdgeWords);
        ContactQuadratic localD = ContactQuadratic.At(values, signs, 4, PrismEdgeWords);
        ContactQuadratic first = ContactQuadratic.At(values, signs, 5, PrismEdgeWords);
        ContactQuadratic second = ContactQuadratic.At(values, signs, 6, PrismEdgeWords);
        ContactQuadratic numerator = ContactQuadratic.At(values, signs, 7, PrismEdgeWords);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 8, PrismEdgeWords);
        denominator.Set(points[6]);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic coordinate = ContactQuadratic.At(values, signs, axis, PrismEdgeWords);
            if (!capsule) { coordinate.Set(points[axis + 3]); continue; }
            Signed832 relative = WideArithmetic.SubtractSigned832(points[axis],
                WideArithmetic.MultiplySigned832(points[6], WideArithmetic.Difference(origin[axis], prismOrigin[axis])));
            first.Set(WideArithmetic.SubtractSigned832(points[axis + 3], points[axis]));
            ContactQuadratic.Scale(first, Signed576.ExtendValue(Signed192.Raw(radius)), second);
            ContactQuadratic.Multiply(second, denominator, ReadOnlySpan<ulong>.Empty, coordinate);
            WideArithmetic.GetMagnitude(relative, coordinate.Radical); coordinate.Signs[1] = relative.Sign;
        }
        if (capsule)
        {
            denominator.Clear(); WideArithmetic.GetMagnitude(points[6], denominator.Radical); denominator.Signs[1] = 1;
        }
        WideRationalBasis3d basis = new(rotation);
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(basis.Denominator), localD);
        ReadOnlySpan<Signed192> coefficients = stackalloc Signed192[9]
            { basis.Xx, basis.Xy, basis.Xz, basis.Yx, basis.Yy, basis.Yz, basis.Zx, basis.Zy, basis.Zz };
        Vector3d point = default, displacement = default;
        for (int axis = 0; axis < 3; axis++)
        {
            numerator.Clear();
            for (int component = 0; component < 3; component++)
            {
                ContactQuadratic.Scale(ContactQuadratic.At(values, signs, component, PrismEdgeWords),
                    Signed576.ExtendValue(coefficients[3 * axis + component]), term); numerator.Add(term);
            }
            ContactQuadratic.SplitRound(numerator, localD, root, out Fixed64 a, out Fixed64 b);
            point[axis] = a; displacement[axis] = b;
        }
        return new FixedPointAnchor(origin, rotation, point, displacement);
    }
}
