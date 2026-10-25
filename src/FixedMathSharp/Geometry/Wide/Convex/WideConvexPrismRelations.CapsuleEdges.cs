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
        ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness, out FixedContactAnchors contact)
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
        // Native vertices/core endpoints share D=2*Q*rotationD (<98 bits).
        // Normalized bases and authored axes give core C<163,u<162, native
        // points<163,v<164,w<165; aa<326,bb<330,ab<328,ao<329,bo<330.
        // Thus det<656 and valid 0<=s,t<=det bound point numerators<821,
        // denominator<754 and squared distance<1648 bits. Final quadratic
        // coordinate coefficients remain below 1720, within 2048-bit banks.
        Span<ulong> values = stackalloc ulong[12 * PrismEdgeWords];
        Span<int> signs = stackalloc int[12];
        ContactQuadratic bestQ = ContactQuadratic.At(values, signs, 0, PrismEdgeWords);
        ContactQuadratic bestD = ContactQuadratic.At(values, signs, 1, PrismEdgeWords);
        ContactQuadratic currentQ = ContactQuadratic.At(values, signs, 2, PrismEdgeWords);
        ContactQuadratic currentD = ContactQuadratic.At(values, signs, 3, PrismEdgeWords);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 4, PrismEdgeWords);
        ContactQuadratic product = ContactQuadratic.At(values, signs, 5, PrismEdgeWords);
        Span<Signed832> best = stackalloc Signed832[7];
        Signed832 bestMiddleN = default, bestMiddleD = default;
        bool found = false;
        var yaw = new WideConvex2dRelations.RotationFrame2d(prismYaw);
        Span<Signed192> first = stackalloc Signed192[3], second = stackalloc Signed192[3];
        Span<Signed192> v = stackalloc Signed192[3], w = stackalloc Signed192[3];
        Signed576 aa = PrismEdgeDot(u, u);
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
                Signed576 bb = PrismEdgeDot(v, v), ab = PrismEdgeDot(u, v);
                Signed576 ao = PrismEdgeDot(u, w), bo = PrismEdgeDot(v, w);
                if (bb.IsZero)
                {
                    Signed576 d = aa.IsZero ? Signed576.ExtendValue(Signed192.Signed(1)) : aa;
                    KeepPrismEdgePair(core, u, first, v, coordinateD,
                        Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.Negate(ao), d)), default, Signed832.ExtendValue(d),
                        currentQ, currentD, term, product, bestQ, bestD, best,
                        ref bestMiddleN, ref bestMiddleD, ref found);
                    continue;
                }
                KeepPrismEdgePair(core, u, first, v, coordinateD, default,
                    Signed832.ExtendValue(ClampPrismEdgeParameter(bo, bb)), Signed832.ExtendValue(bb),
                    currentQ, currentD, term, product, bestQ, bestD, best,
                    ref bestMiddleN, ref bestMiddleD, ref found);
                if (aa.IsZero) continue;
                KeepPrismEdgePair(core, u, first, v, coordinateD, Signed832.ExtendValue(bb),
                    Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.AddSigned576(bo, ab), bb)), Signed832.ExtendValue(bb),
                    currentQ, currentD, term, product, bestQ, bestD, best,
                    ref bestMiddleN, ref bestMiddleD, ref found);
                KeepPrismEdgePair(core, u, first, v, coordinateD,
                    Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.Negate(ao), aa)), default, Signed832.ExtendValue(aa),
                    currentQ, currentD, term, product, bestQ, bestD, best,
                    ref bestMiddleN, ref bestMiddleD, ref found);
                KeepPrismEdgePair(core, u, first, v, coordinateD,
                    Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.SubtractSigned576(ab, ao), aa)),
                    Signed832.ExtendValue(aa), Signed832.ExtendValue(aa),
                    currentQ, currentD, term, product, bestQ, bestD, best,
                    ref bestMiddleN, ref bestMiddleD, ref found);
                // Parallel closest families retain the admitted core midpoint,
                // then the nearest finite parameter; earlier edges own ties.
                Signed576 twiceBb = WideArithmetic.AddSigned576(bb, bb);
                KeepPrismEdgePair(core, u, first, v, coordinateD, Signed832.ExtendValue(bb),
                    Signed832.ExtendValue(ClampPrismEdgeParameter(WideArithmetic.AddSigned576(
                        WideArithmetic.AddSigned576(bo, bo), ab), twiceBb)), Signed832.ExtendValue(twiceBb),
                    currentQ, currentD, term, product, bestQ, bestD, best,
                    ref bestMiddleN, ref bestMiddleD, ref found);
                Signed832 determinant = WideArithmetic.SubtractSigned832(
                    WideArithmetic.MultiplySigned576ToSigned832(aa, bb), WideArithmetic.MultiplySigned576ToSigned832(ab, ab));
                if (determinant.Sign <= 0) continue;
                Signed832 s = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(ab, bo),
                    WideArithmetic.MultiplySigned576ToSigned832(bb, ao));
                Signed832 t = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(aa, bo),
                    WideArithmetic.MultiplySigned576ToSigned832(ab, ao));
                if (s.Sign < 0 || t.Sign < 0 || WideArithmetic.SubtractSigned832(determinant, s).Sign < 0
                    || WideArithmetic.SubtractSigned832(determinant, t).Sign < 0) continue;
                KeepPrismEdgePair(core, u, first, v, coordinateD, s, t, determinant,
                    currentQ, currentD, term, product, bestQ, bestD, best,
                    ref bestMiddleN, ref bestMiddleD, ref found);
            }
        }
        System.Diagnostics.Debug.Assert(found && bestQ.Signs[0] > 0);
        term.Set(Signed576.ExtendValue(Signed192.Raw(radius)));
        ContactQuadratic.Multiply(term, term, ReadOnlySpan<ulong>.Empty, product);
        ContactQuadratic.Multiply(product, bestD, ReadOnlySpan<ulong>.Empty, currentD);
        bestQ.CopyTo(currentQ); currentQ.Add(currentD, -1);
        if (currentQ.Signs[0] > 0) return false;
        contact = MaterializePrismEdgeContact(best, bestQ.Rational, center, shapeRotation,
            prismOrigin, prismOrientation, radius);
        return true;
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

    private static void KeepPrismEdgePair(ReadOnlySpan<Signed192> core, ReadOnlySpan<Signed192> u,
        ReadOnlySpan<Signed192> edge, ReadOnlySpan<Signed192> v, Signed192 coordinateD,
        Signed832 s, Signed832 t, Signed832 d, ContactQuadratic q, ContactQuadratic squaredD,
        ContactQuadratic term, ContactQuadratic product, ContactQuadratic bestQ, ContactQuadratic bestD,
        Span<Signed832> best, ref Signed832 bestMiddleN, ref Signed832 bestMiddleD, ref bool found)
    {
        Span<Signed832> points = stackalloc Signed832[7];
        q.Clear();
        for (int component = 0; component < 3; component++)
        {
            points[component] = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned832(d, core[component]),
                WideArithmetic.MultiplySigned832(s, u[component]));
            points[component + 3] = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned832(d, edge[component]),
                WideArithmetic.MultiplySigned832(t, v[component]));
            term.Set(WideArithmetic.SubtractSigned832(points[component + 3], points[component]));
            ContactQuadratic.Multiply(term, term, ReadOnlySpan<ulong>.Empty, product); q.Add(product);
        }
        points[6] = WideArithmetic.MultiplySigned832(d, coordinateD);
        term.Set(points[6]); ContactQuadratic.Multiply(term, term, ReadOnlySpan<ulong>.Empty, squaredD);
        int comparison = found ? ContactQuadratic.CompareRatios(q, squaredD, ReadOnlySpan<ulong>.Empty,
            bestQ, bestD, ReadOnlySpan<ulong>.Empty) : -1;
        Signed832 middle = WideArithmetic.SubtractSigned832(WideArithmetic.AddSigned832(s, s), d);
        if (middle.Sign < 0) middle = WideArithmetic.SubtractSigned832(default, middle);
        if (comparison > 0 || comparison == 0 && WideArithmetic.CompareNonNegativeProducts(
                middle, bestMiddleD, bestMiddleN, d) >= 0) return;
        q.CopyTo(bestQ); squaredD.CopyTo(bestD); points.CopyTo(best);
        bestMiddleN = middle; bestMiddleD = d; found = true;
    }

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
