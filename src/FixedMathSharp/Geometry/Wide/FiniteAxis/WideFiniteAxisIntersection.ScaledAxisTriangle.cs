//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact closest-feature ingress for affinely scaled triangles and finite capsule cores.</content>
internal static partial class WideFiniteAxisIntersection
{
    // Core endpoints retain the established scalar normalized axis and odd half
    // length. Source world points have at most 319 bits, their bounded edges
    // at most 305 after a second rigid core frame, and core/edge Gram minors
    // at most 1304 bits. Finite face witnesses
    // and clipped endpoint witnesses require fewer than 2600 coordinate bits;
    // squared distance and the final radius comparison fit these 6144-bit banks.
    private const int ScaledAxisWords = 96;

    internal ref struct ScaledAxisTriangleCandidate
    {
        internal const int StorageWords = 18 * ScaledAxisWords, SignCount = 18;
        internal ContactQuadratic TriangleCoordinate(int axis) => At(axis);
        internal ContactQuadratic AxisCoordinate(int axis) => At(axis + 3);
        internal ContactQuadratic Denominator => At(6);
        private readonly Span<ulong> _values;
        private readonly Span<int> _signs;
        internal bool HasValue;
        internal int EdgeMask, Endpoint;
        internal ScaledAxisTriangleCandidate(Span<ulong> values, Span<int> signs)
        { _values = values; _signs = signs; HasValue = false; EdgeMask = Endpoint = 0; }
        internal ContactQuadratic At(int index) => ContactQuadratic.At(_values, _signs, index, ScaledAxisWords);
        internal void Keep(scoped ScaledAxisTriangleCandidate candidate)
        {
            if (HasValue && ContactQuadratic.CompareRatios(candidate.At(7), candidate.At(8), ReadOnlySpan<ulong>.Empty,
                At(7), At(8), ReadOnlySpan<ulong>.Empty) >= 0) return;
            candidate._values.CopyTo(_values); candidate._signs.CopyTo(_signs);
            HasValue = true; EdgeMask = candidate.EdgeMask; Endpoint = candidate.Endpoint;
        }
        internal void Complete()
        {
            Span<ulong> values = stackalloc ulong[4 * ScaledAxisWords]; Span<int> signs = stackalloc int[4];
            var delta = ContactQuadratic.At(values, signs, 0, ScaledAxisWords);
            var square = ContactQuadratic.At(values, signs, 1, ScaledAxisWords);
            At(7).Clear();
            for (int axis = 0; axis < 3; axis++)
            {
                At(axis + 3).CopyTo(delta); delta.Add(At(axis), -1);
                ContactQuadratic.Multiply(delta, delta, ReadOnlySpan<ulong>.Empty, square); At(7).Add(square);
            }
            ContactQuadratic.Multiply(At(6), At(6), ReadOnlySpan<ulong>.Empty, At(8));
            HasValue = true;
        }
    }

    internal static bool TryGetScaledCenteredCapsuleTriangleContact(in FixedTriangle triangle,
        in ScaledRigidFrame3d frame, Vector3d center, FixedQuaternion capsuleRotation,
        Fixed64 length, Fixed64 radius, Vector3d fallback, out FixedContactAnchors contact,
        out AxisTriangleContactFeature feature, out int endpoint, in RationalSegmentInterval interval = default)
    {
        contact = default; feature = default; endpoint = 0;
        Vector3d axis = length == Fixed64.Zero ? Vector3d.Up : capsuleRotation.Rotate(Vector3d.Up).Normalized;
        Span<ulong> bestValues = stackalloc ulong[ScaledAxisTriangleCandidate.StorageWords];
        Span<int> bestSigns = stackalloc int[ScaledAxisTriangleCandidate.SignCount];
        var best = new ScaledAxisTriangleCandidate(bestValues, bestSigns);
        GetClosestScaledAxisTriangleCandidate(triangle, frame, center, axis, length, ref best, interval);
        Span<ulong> workValues = stackalloc ulong[8 * ScaledAxisWords]; Span<int> workSigns = stackalloc int[8];
        var n = ContactQuadratic.At(workValues, workSigns, 0, ScaledAxisWords);
        var d = ContactQuadratic.At(workValues, workSigns, 1, ScaledAxisWords);
        var term = ContactQuadratic.At(workValues, workSigns, 2, ScaledAxisWords);
        var threshold = ContactQuadratic.At(workValues, workSigns, 3, ScaledAxisWords);
        ContactQuadratic.Scale(best.At(6), Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(radius))), n);
        ContactQuadratic.Multiply(n, n, ReadOnlySpan<ulong>.Empty, threshold);
        best.At(7).CopyTo(term); term.Add(threshold, -1);
        if (term.Signs[0] > 0) return false;
        Span<Signed832> separation = stackalloc Signed832[3];
        GetScaledAxisPrimitiveSeparation(best, separation);
        int vertex = best.EdgeMask == 5 ? 0 : best.EdgeMask == 3 ? 1 : best.EdgeMask == 6 ? 2 : -1;
        int edgeMask = WideTriangleRelations.GetScaledTriangleLocalNormal(triangle, frame).IsZero ? 7 : best.EdgeMask;
        feature = new AxisTriangleContactFeature(edgeMask, separation[0], separation[1], separation[2], vertex);
        endpoint = best.Endpoint;
        Vector3d normal = WideNormalization.GetNormalized(separation[0], separation[1], separation[2]);
        if (normal.IsZero) normal = fallback;
        Vector3d source = default;
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            if (!TryRoundScaledAxisCoordinate(best.At(coordinate), best.At(6), out Fixed64 sourceValue,
                    center[coordinate].m_rawValue)) return false;
            source[coordinate] = sourceValue;
        }
        if (best.At(7).Signs[0] == 0 && length != Fixed64.Zero)
            return TryGetScaledCoincidentCapsuleContact(best, center, axis, length, radius, normal, source, out contact);
        // radius - sqrt(distanceSquared) remains one exact quadratic ratio;
        // rounding the distance first would reverse nearest-even ties when
        // radius.Raw is odd.
        n.Radical.Clear(); n.Radical[0] = 1; n.Signs[1] = -1;
        best.At(6).CopyTo(d);
        Span<ulong> one = stackalloc ulong[1] { 1 };
        bool represented = ContactQuadratic.TryRoundRootRatio(n, d, best.At(7).Rational, one, one, out Fixed64 depth);
        // Exact radius admission above bounds this nonnegative depth by the
        // already representable radius, including nearest-even materialization.
        System.Diagnostics.Debug.Assert(represented);
        // Both anchors use the bounded physical chart. Authored vertices may
        // lie far outside the scalar chart after inverse scale cancellation.
        Span<ulong> exitValues = stackalloc ulong[8 * ScaledAxisWords]; Span<int> exitSigns = stackalloc int[8];
        var x = ScaledAxisField(exitValues, exitSigns, 0); var y = ScaledAxisField(exitValues, exitSigns, 1);
        var z = ScaledAxisField(exitValues, exitSigns, 2); var exitD = ScaledAxisField(exitValues, exitSigns, 3);
        ContactQuadratic.Scale(best.At(6), RayExtend(ParameterScale), exitD);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            var target = coordinate == 0 ? x : coordinate == 1 ? y : z;
            ContactQuadratic.Scale(best.At(coordinate + 3), RayExtend(ParameterScale), target);
            ContactQuadratic.Scale(best.At(6), RayExtend(Fixed64.GetExactRawProduct(normal[coordinate], radius)), term);
            target.Add(term, -1);
        }
        if (!QuadraticPointMaterialization.TryGetPointInFrame(center, FixedQuaternion.Identity,
            x, y, z, exitD, ReadOnlySpan<ulong>.Empty, center, out Vector3d exit)) return false;
        contact = new FixedContactAnchors(new FixedPointAnchor(center, FixedQuaternion.Identity, source),
            new FixedPointAnchor(center, FixedQuaternion.Identity, exit), normal, depth, false);
        return true;
    }

    internal static void GetClosestScaledAxisTriangleCandidate(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        Vector3d center, Vector3d normalizedAxis, Fixed64 length, scoped ref ScaledAxisTriangleCandidate best,
        in RationalSegmentInterval interval = default)
    {
        RationalPoint3d first = GetScaledCoreEndpoint(center, normalizedAxis, length, -1),
            last = GetScaledCoreEndpoint(center, normalizedAxis, length, 1);
        GetClosestScaledAxisTriangleCandidate(triangle, frame, first, last, center, ref best, interval);
    }

    internal static void GetClosestScaledAxisTriangleCandidate(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d first, in RationalPoint3d last, Vector3d center, scoped ref ScaledAxisTriangleCandidate best,
        in RationalSegmentInterval interval = default)
    {
        Span<ulong> values = stackalloc ulong[ScaledAxisTriangleCandidate.StorageWords];
        Span<int> signs = stackalloc int[ScaledAxisTriangleCandidate.SignCount];
        var candidate = new ScaledAxisTriangleCandidate(values, signs);
        if (!interval.IsClipped && AccumulateScaledAxisFace(triangle, frame, first, last, center, candidate, ref best)) return;
        Span<Vector3d> vertices = stackalloc Vector3d[3] { triangle.A, triangle.B, triangle.C };
        int count = interval.IsClipped ? 1 : 3;
        for (int edge = 0; edge < count; edge++)
            AccumulateScaledAxisEdge(frame.GetPoint(vertices[edge]), frame.GetPoint(vertices[(edge + 1) % 3]),
                first, last, center, edge, interval, candidate, ref best);
        // Even a point triangle has a finite source endpoint whose projection
        // onto the closed core is a closest candidate.
        System.Diagnostics.Debug.Assert(best.HasValue);
    }
    private static bool TryGetScaledCoincidentCapsuleContact(scoped in ScaledAxisTriangleCandidate candidate,
        Vector3d center, Vector3d axis, Fixed64 length, Fixed64 radius, Vector3d normal, Vector3d source,
        out FixedContactAnchors contact)
    {
        contact = default;
        Span<ulong> values = stackalloc ulong[16 * ScaledAxisWords]; Span<int> signs = stackalloc int[16];
        Span<ulong> root = stackalloc ulong[2 * ScaledAxisWords + 1];
        var n = ScaledAxisField(values, signs, 0); var d = ScaledAxisField(values, signs, 1);
        var x = ScaledAxisField(values, signs, 2); var y = ScaledAxisField(values, signs, 3);
        var z = ScaledAxisField(values, signs, 4); var divisor = ScaledAxisField(values, signs, 5);
        var term = ScaledAxisField(values, signs, 6); var coefficient = ScaledAxisField(values, signs, 7);
        GetCoincidentCapsuleRayExit(candidate.At(3), candidate.At(4), candidate.At(5), candidate.At(6),
            axis, length, radius, -normal, n, d, root, out _);
        GetCapsuleRayDepth(n, d, root, normal, out Fixed64 depth, out bool clamped);
        if (clamped) return false;
        ContactQuadratic.Multiply(candidate.At(6), d, root, divisor);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            var target = coordinate == 0 ? x : coordinate == 1 ? y : z;
            ContactQuadratic.Multiply(candidate.At(coordinate + 3), d, root, target);
            ContactQuadratic.Scale(candidate.At(6), RayExtend(Signed192.Raw(-normal[coordinate])), coefficient);
            ContactQuadratic.Multiply(coefficient, n, root, term); target.Add(term);
        }
        if (!QuadraticPointMaterialization.TryGetPointInFrame(center, FixedQuaternion.Identity,
            x, y, z, divisor, root, center, out Vector3d exit)) return false;
        contact = new FixedContactAnchors(new FixedPointAnchor(center, FixedQuaternion.Identity, source),
            new FixedPointAnchor(center, FixedQuaternion.Identity, exit), normal, depth, false);
        return true;
    }
    private static RationalPoint3d GetScaledCoreEndpoint(Vector3d center, Vector3d axis, Fixed64 length, int sign)
    {
        Signed320 denominator = new(0, 0, 0, 2, 0);
        Signed320 Coordinate(Fixed64 origin, Fixed64 direction)
        {
            Signed192 product = Fixed64.GetExactRawProduct(direction, length);
            Signed320 offset = WideArithmetic.MultiplySigned192(product, Signed192.Signed(1L << 32));
            if (sign < 0) offset = WideArithmetic.Negate(offset);
            return Signed320.NarrowValue(WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(origin)), Signed576.ExtendValue(offset)));
        }
        return new RationalPoint3d(Coordinate(center.X, axis.X), Coordinate(center.Y, axis.Y),
            Coordinate(center.Z, axis.Z), denominator);
    }

    private static bool TryRoundScaledAxisCoordinate(ContactQuadratic n, ContactQuadratic d, out Fixed64 value, long parityOffset = 0) =>
        Fixed64.TryGetSignedRawRatio(n.Rational, d.Rational, n.Signs[0] < 0, out value, parityOffset);

    private static bool AccumulateScaledAxisFace(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d first, in RationalPoint3d last, Vector3d center,
        scoped ScaledAxisTriangleCandidate candidate, scoped ref ScaledAxisTriangleCandidate best)
    {
        if (!WideTriangleRelations.TryGetProjectedSegmentInterval(triangle, frame, first, last,
            out Signed832 ln, out Signed832 ld, out Signed832 un, out Signed832 ud)) return false;
        WideDirection3d normal = WideTriangleRelations.GetScaledTriangleLocalNormal(triangle, frame).Transform(frame.GeometryBasis);
        RationalPoint3d reference = frame.GetPoint(triangle.A);
        Signed832 a = WideTriangleRelations.DotScaledDirections(normal,
            WideTriangleRelations.GetScaledPointDifference(first, reference, frame.RotationDenominator));
        Signed832 b = WideTriangleRelations.DotScaledDirections(normal,
            WideTriangleRelations.GetScaledPointDifference(last, reference, frame.RotationDenominator));
        Span<ulong> values = stackalloc ulong[20 * ScaledAxisWords]; Span<int> signs = stackalloc int[20];
        var n = ScaledAxisField(values, signs, 0); var d = ScaledAxisField(values, signs, 1);
        var remaining = ScaledAxisField(values, signs, 2); var plane = ScaledAxisField(values, signs, 3);
        var term = ScaledAxisField(values, signs, 4); var coefficient = ScaledAxisField(values, signs, 5);
        var metric = ScaledAxisField(values, signs, 6); var scale = ScaledAxisField(values, signs, 7);
        var coordinate = ScaledAxisField(values, signs, 8); var query = ScaledAxisField(values, signs, 9);
        Signed832 delta = WideArithmetic.SubtractSigned832(a, b);
        bool lowerBound = delta.IsZero;
        if (delta.IsZero)
        {
            // A reduced mediant is a reversal-symmetric interior representative
            // of this constant-distance face family, without multiplying its
            // independently authored interval denominators.
            SetScaledAxisMediant(ln, ld, un, ud, n, d);
        }
        else
        {
            Signed832 pn = a, pd = delta;
            if (pd.Sign < 0) { pn = WideArithmetic.SubtractSigned832(default, pn); pd = WideArithmetic.SubtractSigned832(default, pd); }
            Signed832 globalN = pn;
            if (globalN.Sign < 0) globalN = default;
            else if (WideArithmetic.SubtractSigned832(globalN, pd).Sign > 0) globalN = pd;
            lowerBound = RationalSegmentInterval.CompareParameters(globalN, pd, ln, ld) >= 0
                && RationalSegmentInterval.CompareParameters(globalN, pd, un, ud) <= 0;
            // The interval comparator accepts nonnegative unit parameters.
            // Clamp the unbounded plane foot to the finite core first: a
            // negative foot must never be interpreted as an unsigned product.
            pn = globalN;
            if (RationalSegmentInterval.CompareParameters(pn, pd, ln, ld) < 0) { pn = ln; pd = ld; }
            else if (RationalSegmentInterval.CompareParameters(pn, pd, un, ud) > 0) { pn = un; pd = ud; }
            n.Set(pn); d.Set(pd);
        }
        d.CopyTo(remaining); remaining.Add(n, -1);
        Span<Signed832> firstWalls = stackalloc Signed832[3], lastWalls = stackalloc Signed832[3];
        _ = WideTriangleRelations.GetScaledTriangleHalfPlanes(triangle, frame, first, firstWalls);
        _ = WideTriangleRelations.GetScaledTriangleHalfPlanes(triangle, frame, last, lastWalls);
        int mask = 0;
        for (int edge = 0; edge < 3; edge++)
        {
            ContactQuadratic.Scale(remaining, firstWalls[edge], query);
            ContactQuadratic.Scale(n, lastWalls[edge], term); query.Add(term);
            System.Diagnostics.Debug.Assert(query.Signs[0] >= 0);
            if (query.Signs[0] == 0) mask |= 1 << edge;
        }
        ContactQuadratic.Scale(remaining, a, plane); ContactQuadratic.Scale(n, b, term); plane.Add(term);
        metric.Set(normal.SquaredLength);
        coefficient.Set(Signed576.ExtendValue(first.Denominator));
        ContactQuadratic.Scale(coefficient, Signed576.ExtendValue(Signed320.ExtendValue(frame.RotationDenominator)), scale);
        ContactQuadratic.Multiply(scale, d, ReadOnlySpan<ulong>.Empty, term);
        ContactQuadratic.Multiply(term, metric, ReadOnlySpan<ulong>.Empty, candidate.At(6));
        for (int axis = 0; axis < 3; axis++)
        {
            Signed576 start = ScaledAxisRelative(first, center, axis), end = ScaledAxisRelative(last, center, axis);
            ContactQuadratic.Scale(remaining, start, coordinate); ContactQuadratic.Scale(n, end, term); coordinate.Add(term);
            ContactQuadratic.Scale(coordinate, Signed576.ExtendValue(Signed320.ExtendValue(frame.RotationDenominator)), query);
            ContactQuadratic.Multiply(query, metric, ReadOnlySpan<ulong>.Empty, candidate.At(axis + 3));
            candidate.At(axis + 3).CopyTo(candidate.At(axis));
            ContactQuadratic.Scale(plane, axis == 0 ? normal.X : axis == 1 ? normal.Y : normal.Z, term);
            candidate.At(axis).Add(term, -1);
        }
        candidate.EdgeMask = mask; candidate.Endpoint = 0; candidate.Complete(); best.Keep(candidate);
        return lowerBound;
    }

    private static void AccumulateScaledAxisEdge(in RationalPoint3d first, in RationalPoint3d last,
        in RationalPoint3d coreFirst, in RationalPoint3d coreLast, Vector3d center, int edge,
        in RationalSegmentInterval interval, scoped ScaledAxisTriangleCandidate candidate, scoped ref ScaledAxisTriangleCandidate best)
    {
        Span<ulong> values = stackalloc ulong[14 * ScaledAxisWords]; Span<int> signs = stackalloc int[14];
        var edgeN = ScaledAxisField(values, signs, 0); var coreN = ScaledAxisField(values, signs, 1);
        var d = ScaledAxisField(values, signs, 2); var otherD = ScaledAxisField(values, signs, 3);
        var term = ScaledAxisField(values, signs, 4);
        var n = ScaledAxisField(values, signs, 5); var divisor = ScaledAxisField(values, signs, 6);
        Signed192 sourceR = WideArithmetic.TruncateSigned320By64(first.Denominator),
            coreR = WideArithmetic.TruncateSigned320By64(coreFirst.Denominator);
        WideDirection3d e = ScaledAxisDifference(last, first, coreR),
            f = ScaledAxisDifference(coreLast, coreFirst, sourceR),
            r = WideTriangleRelations.GetScaledPointDifference(first, coreFirst, coreR);
        Signed832 ee = WideTriangleRelations.DotScaledDirections(e, e), ff = WideTriangleRelations.DotScaledDirections(f, f);
        Signed832 er = WideTriangleRelations.DotScaledDirections(e, r), fr = WideTriangleRelations.DotScaledDirections(f, r),
            ef = WideTriangleRelations.DotScaledDirections(e, f);
        if (WideTriangleRelations.TryGetClosestNonparallelSegmentParameters(first, last, coreFirst, coreLast,
            edgeN, d, coreN, otherD) && ScaledAxisParameterInInterval(edgeN, d, interval))
            KeepScaledAxisPair(first, last, coreFirst, coreLast, center, edge, interval, edgeN, coreN, d, candidate, ref best);
        if (!ee.IsZero && !ff.IsZero && WideDirection3d.AreParallel(e, f))
        {
            for (int bound = 0; bound < 2; bound++)
            {
                int sourceBound = ef.Sign < 0 ? 1 - bound : bound;
                Signed832 sn = interval.IsClipped ? sourceBound == 0 ? interval.WideLowerNumerator : interval.WideUpperNumerator
                    : sourceBound == 0 ? default : Signed832.ExtendValue(Signed192.Signed(1));
                Signed832 sd = interval.IsClipped ? sourceBound == 0 ? interval.WideLowerDenominator : interval.WideUpperDenominator
                    : Signed832.ExtendValue(Signed192.Signed(1));
                n.Set(sn); divisor.Set(sd);
                var parameterN = bound == 0 ? edgeN : coreN;
                var parameterD = bound == 0 ? d : otherD;
                ContactQuadratic.Scale(divisor, fr, parameterN); ContactQuadratic.Scale(n, ef, term); parameterN.Add(term);
                ContactQuadratic.Scale(divisor, ff, parameterD);
            }
            if (edgeN.Signs[0] < 0) edgeN.Clear();
            if (ContactQuadratic.CompareRatios(coreN, otherD, ReadOnlySpan<ulong>.Empty,
                    otherD, otherD, ReadOnlySpan<ulong>.Empty) > 0) otherD.CopyTo(coreN);
            if (ContactQuadratic.CompareRatios(edgeN, d, ReadOnlySpan<ulong>.Empty,
                    coreN, otherD, ReadOnlySpan<ulong>.Empty) < 0)
            {
                NormalizeScaledAxisRatio(edgeN, d); NormalizeScaledAxisRatio(coreN, otherD);
                edgeN.CopyTo(n); n.Add(coreN); d.CopyTo(divisor); divisor.Add(otherD);
                ContactQuadratic.Scale(n, ff, edgeN); ContactQuadratic.Scale(divisor, fr, term); edgeN.Add(term, -1);
                ContactQuadratic.Scale(n, ef, coreN); ContactQuadratic.Scale(divisor, ef, d);
                if (d.Signs[0] < 0) { d.MultiplySign(-1); edgeN.MultiplySign(-1); coreN.MultiplySign(-1); }
                KeepScaledAxisPair(first, last, coreFirst, coreLast, center, edge, interval,
                    edgeN, coreN, d, candidate, ref best);
                return;
            }
        }        // Constrained minima are either an unconstrained line foot or a finite
        // endpoint projected onto the opposite segment. These candidates also
        // cover parallel and point-degenerate cores without division by zero.
        for (int coreEnd = 0; coreEnd < 2; coreEnd++)
        {
            if (ee.IsZero) break;
            n.Set(coreEnd == 0 ? WideArithmetic.SubtractSigned832(default, er)
                : WideArithmetic.SubtractSigned832(ef, er));
            d.Set(ee); ClampScaledAxisParameter(n, d);
            if (!ScaledAxisParameterInInterval(n, d, interval)) continue;
            n.CopyTo(edgeN); if (coreEnd == 0) coreN.Clear(); else d.CopyTo(coreN);
            KeepScaledAxisPair(first, last, coreFirst, coreLast, center, edge, interval, edgeN, coreN, d, candidate, ref best);
        }
        for (int sourceEnd = 0; sourceEnd < 2; sourceEnd++)
        {
            Signed832 sourceN = interval.IsClipped ? sourceEnd == 0 ? interval.WideLowerNumerator : interval.WideUpperNumerator
                : sourceEnd == 0 ? default : Signed832.ExtendValue(Signed192.Signed(1));
            Signed832 sourceD = interval.IsClipped ? sourceEnd == 0 ? interval.WideLowerDenominator : interval.WideUpperDenominator
                : Signed832.ExtendValue(Signed192.Signed(1));
            n.Set(sourceN); divisor.Set(sourceD);
            if (ff.IsZero)
            { n.CopyTo(edgeN); divisor.CopyTo(d); coreN.Clear(); }
            else
            {
                ContactQuadratic.Scale(divisor, fr, coreN); ContactQuadratic.Scale(n, ef, term); coreN.Add(term);
                ContactQuadratic.Scale(divisor, ff, d); ClampScaledAxisParameter(coreN, d);
                ContactQuadratic.Scale(n, ff, edgeN);
            }
            KeepScaledAxisPair(first, last, coreFirst, coreLast, center, edge, interval, edgeN, coreN, d, candidate, ref best);
        }

    }

    private static void KeepScaledAxisPair(in RationalPoint3d first, in RationalPoint3d last,
        in RationalPoint3d coreFirst, in RationalPoint3d coreLast, Vector3d center, int edge,
        in RationalSegmentInterval interval, scoped ContactQuadratic edgeN, scoped ContactQuadratic coreN, scoped ContactQuadratic d,
        scoped ScaledAxisTriangleCandidate candidate, scoped ref ScaledAxisTriangleCandidate best)
    {
        Signed192 sourceRotationD = WideArithmetic.TruncateSigned320By64(first.Denominator),
            coreRotationD = WideArithmetic.TruncateSigned320By64(coreFirst.Denominator);
        Span<ulong> values = stackalloc ulong[4 * ScaledAxisWords]; Span<int> signs = stackalloc int[4];
        var coefficient = ScaledAxisField(values, signs, 0); var term = ScaledAxisField(values, signs, 1);
            Signed576 common = WideArithmetic.MultiplySigned320(first.Denominator, coreRotationD);
            coefficient.Set(common); ContactQuadratic.Multiply(coefficient, d, ReadOnlySpan<ulong>.Empty, candidate.At(6));
            for (int axis = 0; axis < 3; axis++)
            {
                Signed576 start = WideArithmetic.MultiplySigned576(ScaledAxisRelative(first, center, axis), coreRotationD);
                Signed576 end = WideArithmetic.MultiplySigned576(ScaledAxisRelative(last, center, axis), coreRotationD);
                ContactQuadratic.Scale(d, start, candidate.At(axis));
                ContactQuadratic.Scale(edgeN, WideArithmetic.SubtractSigned576(end, start), term); candidate.At(axis).Add(term);
                start = WideArithmetic.MultiplySigned576(ScaledAxisRelative(coreFirst, center, axis), sourceRotationD);
                end = WideArithmetic.MultiplySigned576(ScaledAxisRelative(coreLast, center, axis), sourceRotationD);
                ContactQuadratic.Scale(d, start, candidate.At(axis + 3));
                ContactQuadratic.Scale(coreN, WideArithmetic.SubtractSigned576(end, start), term); candidate.At(axis + 3).Add(term);
            }
            int bit = 1 << edge;
            candidate.EdgeMask = edgeN.Signs[0] == 0 ? bit | (1 << ((edge + 2) % 3))
                : ContactQuadratic.CompareRatios(edgeN, d, ReadOnlySpan<ulong>.Empty,
                    d, d, ReadOnlySpan<ulong>.Empty) == 0 ? bit | (1 << ((edge + 1) % 3)) : bit;
            candidate.Endpoint = interval.IsClipped ? GetScaledAxisIntervalEndpoint(edgeN, d, interval) : 0;
            candidate.Complete(); best.Keep(candidate);

    }

    private static ContactQuadratic ScaledAxisField(Span<ulong> values, Span<int> signs, int index) =>
        ContactQuadratic.At(values, signs, index, ScaledAxisWords);

    private static Signed576 ScaledAxisRelative(in RationalPoint3d point, Vector3d center, int axis) =>
        WideArithmetic.SubtractSigned576(Signed576.ExtendValue(axis == 0 ? point.X : axis == 1 ? point.Y : point.Z),
            WideArithmetic.MultiplySigned320(point.Denominator, Signed192.Raw(center[axis])));

    private static WideDirection3d ScaledAxisDifference(in RationalPoint3d end, in RationalPoint3d start, Signed192 otherD) => new(
        WideArithmetic.MultiplySigned320(WideArithmetic.SubtractSigned320(end.X, start.X), otherD),
        WideArithmetic.MultiplySigned320(WideArithmetic.SubtractSigned320(end.Y, start.Y), otherD),
        WideArithmetic.MultiplySigned320(WideArithmetic.SubtractSigned320(end.Z, start.Z), otherD));

    private static void ClampScaledAxisParameter(ContactQuadratic n, ContactQuadratic d)
    {
        if (n.Signs[0] < 0) n.Clear();
        else if (ContactQuadratic.CompareRatios(n, d, ReadOnlySpan<ulong>.Empty, d, d, ReadOnlySpan<ulong>.Empty) > 0) d.CopyTo(n);
    }

    private static bool ScaledAxisParameterInInterval(ContactQuadratic n, ContactQuadratic d, in RationalSegmentInterval interval)
    {
        if (!interval.IsClipped) return true;
        return interval.Contains(n, d, ReadOnlySpan<ulong>.Empty);
    }

    private static int GetScaledAxisIntervalEndpoint(ContactQuadratic n, ContactQuadratic d, in RationalSegmentInterval interval)
    {
        Span<ulong> values = stackalloc ulong[4 * ScaledAxisWords]; Span<int> signs = stackalloc int[4];
        var boundN = ScaledAxisField(values, signs, 0); var boundD = ScaledAxisField(values, signs, 1);
        boundN.Set(interval.WideLowerNumerator); boundD.Set(interval.WideLowerDenominator);
        if (ContactQuadratic.CompareRatios(n, d, ReadOnlySpan<ulong>.Empty, boundN, boundD, ReadOnlySpan<ulong>.Empty) == 0) return -1;
        boundN.Set(interval.WideUpperNumerator); boundD.Set(interval.WideUpperDenominator);
        return ContactQuadratic.CompareRatios(n, d, ReadOnlySpan<ulong>.Empty, boundN, boundD, ReadOnlySpan<ulong>.Empty) == 0 ? 1 : 0;
    }

    private static void SetScaledAxisMediant(Signed832 ln, Signed832 ld, Signed832 un, Signed832 ud,
        ContactQuadratic n, ContactQuadratic d)
    {
        Span<ulong> values = stackalloc ulong[8 * ScaledAxisWords]; Span<int> signs = stackalloc int[8];
        var leftN = ScaledAxisField(values, signs, 0); var leftD = ScaledAxisField(values, signs, 1);
        var rightN = ScaledAxisField(values, signs, 2); var rightD = ScaledAxisField(values, signs, 3);
        leftN.Set(ln); leftD.Set(ld); rightN.Set(un); rightD.Set(ud);
        NormalizeScaledAxisRatio(leftN, leftD); NormalizeScaledAxisRatio(rightN, rightD);
        leftN.CopyTo(n); n.Add(rightN); leftD.CopyTo(d); d.Add(rightD);
    }

    private static void NormalizeScaledAxisRatio(ContactQuadratic n, ContactQuadratic d)
    {
        int words = n.FieldWords;
        Span<ulong> values = stackalloc ulong[4 * words];
        Span<ulong> gcd = values[..words], copy = values.Slice(words, words),
            quotient = values.Slice(2 * words, words), remainder = values.Slice(3 * words, words);
        Span<ulong> scratch = stackalloc ulong[2 * words + 1];
        n.Rational.CopyTo(gcd); d.Rational.CopyTo(copy);
        WideArithmetic.GetMagnitudeGreatestCommonDivisor(gcd, copy, scratch);
        WideArithmetic.DivideMagnitudes(n.Rational, gcd, quotient, remainder, scratch); quotient.CopyTo(n.Rational);
        WideArithmetic.DivideMagnitudes(d.Rational, gcd, quotient, remainder, scratch); quotient.CopyTo(d.Rational);
    }
    private static void GetScaledAxisPrimitiveSeparation(scoped in ScaledAxisTriangleCandidate candidate, Span<Signed832> result)
    {
        int words = ScaledAxisWords;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[6];
        Span<ulong> gcd = values.Slice(6 * words, words), copy = values.Slice(7 * words, words);
        Span<ulong> scratch = stackalloc ulong[2 * words + 1], remainder = stackalloc ulong[words];
        gcd.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            var component = ContactQuadratic.At(values, signs, axis, words);
            candidate.At(axis + 3).CopyTo(component); component.Add(candidate.At(axis), -1);
            if (component.Signs[0] == 0) continue;
            if (WideArithmetic.GetActiveMagnitudeLength(gcd) == 0) component.Rational.CopyTo(gcd);
            else { component.Rational.CopyTo(copy); WideArithmetic.GetMagnitudeGreatestCommonDivisor(gcd, copy, scratch); }
        }
        if (WideArithmetic.GetActiveMagnitudeLength(gcd) == 0) { result.Clear(); return; }
        for (int axis = 0; axis < 3; axis++)
        {
            var component = ContactQuadratic.At(values, signs, axis, words);
            WideArithmetic.DivideMagnitudes(component.Rational, gcd, copy, remainder, scratch);
            // Face directions are the exact scaled cofactor; skew edges use
            // their cross product. Endpoint/edge residuals have degree three
            // in the bounded geometry and require fewer than 800 bits after
            // their common rational parameter factors have been removed.
            if (WideArithmetic.GetActiveMagnitudeLength(copy) > 13 || (copy[12] >> 63) != 0)
                throw new OverflowException("The reduced finite-axis separation must fit Signed832.");
            Signed832 value = new(copy[12], copy[11], copy[10], copy[9], copy[8], copy[7], copy[6], copy[5], copy[4], copy[3], copy[2], copy[1], copy[0]);
            result[axis] = component.Signs[0] < 0 ? WideArithmetic.SubtractSigned832(default, value) : value;
        }
    }
}
