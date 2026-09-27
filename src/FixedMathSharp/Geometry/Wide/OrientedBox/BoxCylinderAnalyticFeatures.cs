//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Exact rational support directions and vertex/rim separation certificates
/// for a box and a finite cylinder. Coordinates stay in the authored box frame.
/// </summary>
/// <remarks>
/// The support sphere splits at box-edge planes, the cylinder's side plane
/// and its two poles. This owner handles their intersections and side arcs;
/// BoxCylinderEdgeContacts handles the remaining edge/rim arcs. Inside an
/// open vertex/rim patch, zero meridional curvature rules out a positive
/// minimum. Its outward residual can still separate, so that certificate
/// must not be omitted merely because it cannot supply penetration depth.
/// </remarks>
internal static class BoxCylinderAnalyticFeatures
{
    private const int Words = ConvexContactCandidate.Words;

    internal static bool TryGetBest(in BoxCylinderGeometry geometry,
        Span<ulong> bestValues, Span<int> bestSigns, out int bestGapSign,
        out Vector3d localSupportSigns)
    {
        var selection = new Selection(bestValues, bestSigns);
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> directionSigns = stackalloc int[3];
        WriteDirection(geometry.Axis, direction, directionSigns);
        KeepAxis(geometry, direction, directionSigns, ref selection);
        for (int axis = 0; axis < 3 && selection.GapSign >= 0; axis++)
        {
            WideAxis3 unit = Unit(axis);
            WriteDirection(unit, direction, directionSigns);
            KeepAxis(geometry, direction, directionSigns, ref selection);
            WriteDirection(WideAxis3.Cross(unit, geometry.Axis), direction, directionSigns);
            KeepAxis(geometry, direction, directionSigns, ref selection);
        }

        if (!geometry.Radius.IsZero)
        {
            // On a cylinder's side-normal plane, each box vertex contributes
            // a linear term plus a circle. Its radial direction and the plane's
            // box-edge boundaries exhaust the possible minima.
            for (int corner = 0; corner < 8 && selection.GapSign >= 0; corner++)
            {
                WideAxis3 offset = geometry.GetCapVertexOffset(corner, 0);
                ProjectPerpendicular(geometry.Axis, offset, direction, directionSigns);
                KeepAxis(geometry, direction, directionSigns, ref selection);
                for (int cap = -1; cap <= 1; cap += 2)
                {
                    if (HasVertexRimSeparation(geometry, corner, cap))
                    {
                        bestGapSign = -1;
                        localSupportSigns = default;
                        return false;
                    }
                }
            }

            // The edge quartic eliminates sqrt(Q) using K=q-p*t. K=0 is a
            // rational direction, not permission to divide by zero. Include
            // it here using the full support function, independent of a cone.
            // When p=q=0 the projected disk's two principal axes suffice.
            for (int edge = 0; edge < 3 && selection.GapSign >= 0; edge++)
            {
                int j = (edge + 1) % 3;
                int k = (edge + 2) % 3;
                Signed320 aj = BoxCylinderGeometry.GetComponent(geometry.Axis, j);
                Signed320 ak = BoxCylinderGeometry.GetComponent(geometry.Axis, k);
                WritePlaneDirection(j, k, aj, ak, direction, directionSigns);
                KeepAxis(geometry, direction, directionSigns, ref selection);
                WritePlaneDirection(j, k, WideArithmetic.Negate(ak), aj, direction, directionSigns);
                KeepAxis(geometry, direction, directionSigns, ref selection);
                for (int cone = 0; cone < 4 && selection.GapSign >= 0; cone++)
                {
                    int corner = ((cone & 1) != 0 ? 1 << j : 0)
                        | ((cone & 2) != 0 ? 1 << k : 0);
                    for (int cap = -1; cap <= 1; cap += 2)
                    {
                        WideAxis3 c = geometry.GetCapVertexOffset(corner, cap);
                        Signed320 cj = BoxCylinderGeometry.GetComponent(c, j);
                        Signed320 ck = BoxCylinderGeometry.GetComponent(c, k);
                        int sj = (cone & 1) != 0 ? 1 : -1;
                        int sk = (cone & 2) != 0 ? 1 : -1;
                        if (cj.Sign * sj * ck.Sign * sk > 0)
                        {
                            WritePlaneDirection(j, k, cj, ck, direction, directionSigns);
                            KeepAxis(geometry, direction, directionSigns, ref selection);
                        }
                    }
                }
            }
        }
        bestGapSign = selection.GapSign;
        localSupportSigns = selection.LocalSigns;
        return bestGapSign >= 0;
    }

    internal static bool IsEdgeConeDominated(in BoxCylinderGeometry geometry,
        int edge, int first, int second, int firstSign, int secondSign, int cap,
        in ConvexContactCandidate best)
    {
        int corner = (firstSign > 0 ? 1 << first : 0) | (secondSign > 0 ? 1 << second : 0);
        WideAxis3 offset = geometry.GetCapVertexOffset(corner, cap);
        Signed320 cu = BoxCylinderGeometry.GetComponent(offset, first);
        Signed320 cv = BoxCylinderGeometry.GetComponent(offset, second);
        if (cu.Sign * firstSign < 0 || cv.Sign * secondSign < 0)
            return false;

        // In this signed unit quadrant, cu*nu+cv*nv >= min(cu,cv).
        // The projected disk support is at least R*|a_edge|/sqrt(U).
        // Their sum bounds the entire cone, including both parameter charts.
        // It is positive here: radius and the edge's axis component are nonzero.
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        values.Clear(); signs.Clear();
        values[0] = 1; signs[0] = 1; // Comparison-only candidate: arbitrary unit normal.
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<ulong> m = Slot(work, 0), other = Slot(work, 1), re = Slot(work, 2);
        Span<ulong> product = Slot(work, 3);
        Import(cu, m); Import(cv, other);
        if (WideArithmetic.CompareMagnitudeEqualLength(m, other) > 0)
            other.CopyTo(m);
        Import(geometry.Radius, other);
        Import(BoxCylinderGeometry.GetComponent(geometry.Axis, edge), product);
        WideArithmetic.MultiplyMagnitudes(other, product, re);
        Import(geometry.Axis.SquaredLength, Slot(values, 9));
        // b²=(A+B*sqrt(U))/D, A=m²U+(Re)², B=2mRe, D=S²U.
        // Conservative geometry bounds put A below 817 bits and D below 682.
        WideArithmetic.MultiplyMagnitudes(m, m, product);
        WideArithmetic.MultiplyMagnitudes(product, Slot(values, 9), Slot(values, 7));
        WideArithmetic.MultiplyMagnitudes(re, re, product);
        WideArithmetic.AddMagnitudeInto(product, Slot(values, 7));
        WideArithmetic.MultiplyMagnitudes(m, re, product);
        WideArithmetic.AddEqualMagnitudes(product, product, Slot(values, 8));
        Import(Signed320.ExtendValue(geometry.RawScale), other);
        WideArithmetic.MultiplyMagnitudes(other, other, product);
        WideArithmetic.MultiplyMagnitudes(product, Slot(values, 9), Slot(values, 10));
        signs[7] = 1;
        signs[8] = IsZero(m) ? 0 : 1;
        var bound = new ConvexContactCandidate(values, signs, 1);
        return WideConvexPrismRelations.CompareConvexContactCandidates(bound, best) >= 0;
    }

    private static void KeepAxis(in BoxCylinderGeometry geometry,
        scoped ReadOnlySpan<ulong> direction, scoped ReadOnlySpan<int> directionSigns, ref Selection selection)
    {
        if (selection.HasValue && selection.GapSign < 0)
            return;
        if (directionSigns[0] == 0 && directionSigns[1] == 0 && directionSigns[2] == 0)
            return;
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        BuildAxis(geometry, direction, directionSigns, values, signs, out int gapSign, out int orientation);
        var candidate = new ConvexContactCandidate(values, signs, gapSign);
        if (selection.HasValue && WideConvexPrismRelations.CompareConvexContactCandidates(candidate,
                new ConvexContactCandidate(selection.Values, selection.Signs, selection.GapSign)) >= 0)
            return;
        values.CopyTo(selection.Values);
        signs.CopyTo(selection.Signs);
        selection.GapSign = gapSign;
        selection.HasValue = true;
        selection.LocalSigns = new Vector3d((Fixed64)(orientation * directionSigns[0]),
            (Fixed64)(orientation * directionSigns[1]), (Fixed64)(orientation * directionSigns[2]));
    }

    private static void BuildAxis(in BoxCylinderGeometry geometry,
        ReadOnlySpan<ulong> direction, ReadOnlySpan<int> directionSigns,
        Span<ulong> values, Span<int> signs, out int gapSign, out int orientation)
    {
        values.Clear();
        signs.Clear();
        Span<ulong> work = stackalloc ulong[11 * Words];
        Span<ulong> u = Slot(work, 0), n2 = Slot(work, 1), axial = Slot(work, 2);
        Span<ulong> rational = Slot(work, 3), temporary = Slot(work, 4), product = Slot(work, 5);
        Span<ulong> p = Slot(work, 6), p2 = Slot(work, 7), q = Slot(work, 8);
        Span<ulong> radius = Slot(work, 9), scale = Slot(work, 10);
        Import(geometry.Axis.SquaredLength, u);
        SumSquares(direction, n2);
        Dot(geometry.CenterDifference, direction, directionSigns, rational, out int centerSign);
        orientation = centerSign < 0 ? -1 : 1;
        int rationalSign = IsZero(rational) ? 0 : -1;
        Dot(geometry.Half, direction, directionSigns, temporary, out _);
        Add(temporary, 1, rational, ref rationalSign);
        for (int component = 0; component < 3; component++)
        {
            Import(BoxCylinderGeometry.GetComponent(geometry.HalfExtents, component), temporary);
            WideArithmetic.MultiplyMagnitudes(temporary, Slot(direction, component), product);
            Add(product, 1, rational, ref rationalSign);
        }
        if (IsZero(rational))
            rationalSign = 0;

        // g=(p+sqrt(q))/(U*S*|n|), p=U*T,
        // q=R²*U*(U*n²-(a.n)²), T=h_box(n)+|H.n|-|d.n|.
        // Projected vertex directions need up to 577 bits. With authored
        // coordinates below 2^237, p²,q and every denominator fit 2,560 bits.
        WideArithmetic.MultiplyMagnitudes(rational, u, p);
        WideArithmetic.MultiplyMagnitudes(p, p, p2);
        Dot(geometry.Axis, direction, directionSigns, axial, out _);
        WideArithmetic.MultiplyMagnitudes(axial, axial, temporary);
        WideArithmetic.MultiplyMagnitudes(u, n2, product);
        WideArithmetic.SubtractEqualMagnitudes(product, temporary, q);
        Import(geometry.Radius, radius);
        WideArithmetic.MultiplyMagnitudes(radius, radius, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, u, product);
        WideArithmetic.MultiplyMagnitudes(product, q, temporary);
        temporary.CopyTo(q);
        WideArithmetic.AddEqualMagnitudes(p2, q, Slot(values, 7));
        WideArithmetic.AddEqualMagnitudes(p, p, Slot(values, 8));
        q.CopyTo(Slot(values, 9));
        Import(Signed320.ExtendValue(geometry.RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(u, scale, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, temporary, product);
        WideArithmetic.MultiplyMagnitudes(product, n2, Slot(values, 10));
        signs[7] = IsZero(Slot(values, 7)) ? 0 : 1;
        signs[8] = rationalSign;
        gapSign = rationalSign < 0 ? WideArithmetic.CompareMagnitudeEqualLength(q, p2)
            : rationalSign > 0 || !IsZero(q) ? 1 : 0;
        for (int component = 0; component < 3; component++)
        {
            WideRationalBasis3d basis = geometry.WorldBasis;
            WideAxis3 row = component == 0 ? Axis(basis.Xx, basis.Yx, basis.Zx)
                : component == 1 ? Axis(basis.Xy, basis.Yy, basis.Zy) : Axis(basis.Xz, basis.Yz, basis.Zz);
            Dot(row, direction, directionSigns, Slot(values, component), out int sign);
            signs[component] = orientation * sign;
        }
    }

    private static bool HasVertexRimSeparation(in BoxCylinderGeometry geometry, int corner, int cap)
    {
        WideAxis3 c = geometry.GetCapVertexOffset(corner, cap);
        Signed576 axial = WideAxis3.Dot(geometry.Axis, c);
        if (axial.Sign * cap > 0)
            return false;
        Span<ulong> work = stackalloc ulong[10 * Words];
        Span<ulong> u = Slot(work, 0), q = Slot(work, 1), radius = Slot(work, 2);
        Span<ulong> first = Slot(work, 3), second = Slot(work, 4), root = Slot(work, 5);
        Span<ulong> radial = work.Slice(6 * Words, 3 * Words), coordinate = Slot(work, 9);
        Span<int> radialSigns = stackalloc int[3];
        Import(geometry.Axis.SquaredLength, u);
        Import(c.SquaredLength, first);
        WideArithmetic.MultiplyMagnitudes(u, first, q);
        Import(axial, first);
        WideArithmetic.MultiplyMagnitudes(first, first, second);
        WideArithmetic.SubtractEqualMagnitudes(q, second, q);
        if (IsZero(q))
            return false; // A cap pole already supplies this normal.
        Import(geometry.Radius, radius);
        WideArithmetic.MultiplyMagnitudes(radius, radius, first);
        WideArithmetic.MultiplyMagnitudes(first, u, second);
        int radialComparison = WideArithmetic.CompareMagnitudeEqualLength(q, second);
        // At equality the residual is purely axial (or zero). The cylinder
        // pole was tested first, including separation; this is not a new rim.
        if (radialComparison <= 0)
            return false;
        WideArithmetic.MultiplyMagnitudes(u, q, root);
        ProjectPerpendicular(geometry.Axis, c, radial, radialSigns);
        // A smooth vertex/rim patch has zero meridional curvature. A positive
        // support gap therefore has a negative second variation there and
        // cannot be the minimum penetration. Only its outward residual can
        // separate: n=R*P_a(c)-c*sqrt(U*q). Test its exact support cone.
        for (int component = 0; component < 3; component++)
        {
            WideArithmetic.MultiplyMagnitudes(radius, Slot(radial, component), first);
            Signed320 value = BoxCylinderGeometry.GetComponent(c, component);
            Import(value, coordinate);
            int sign = WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(
                first, radialSigns[component], coordinate, -value.Sign, root);
            int supportSign = (corner & (1 << component)) != 0 ? 1 : -1;
            if (sign * supportSign < 0)
                return false;
        }
        return true;
    }

    private static void ProjectPerpendicular(WideAxis3 axis, WideAxis3 value,
        Span<ulong> direction, Span<int> signs)
    {
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<ulong> u = Slot(work, 0), dot = Slot(work, 1), component = Slot(work, 2), product = Slot(work, 3);
        Import(axis.SquaredLength, u);
        Signed576 projection = WideAxis3.Dot(axis, value);
        Import(projection, dot);
        for (int index = 0; index < 3; index++)
        {
            Signed320 v = BoxCylinderGeometry.GetComponent(value, index);
            Signed320 a = BoxCylinderGeometry.GetComponent(axis, index);
            Import(v, component);
            WideArithmetic.MultiplyMagnitudes(u, component, Slot(direction, index));
            int sign = v.Sign;
            Import(a, component);
            WideArithmetic.MultiplyMagnitudes(dot, component, product);
            Add(product, -projection.Sign * a.Sign, Slot(direction, index), ref sign);
            signs[index] = sign;
        }
    }

    private static void Dot(WideAxis3 axis, ReadOnlySpan<ulong> direction, ReadOnlySpan<int> signs,
        Span<ulong> result, out int sign)
    {
        Span<ulong> component = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        result.Clear();
        sign = 0;
        for (int index = 0; index < 3; index++)
        {
            Signed320 value = BoxCylinderGeometry.GetComponent(axis, index);
            Import(value, component);
            WideArithmetic.MultiplyMagnitudes(component, Slot(direction, index), product);
            Add(product, value.Sign * signs[index], result, ref sign);
        }
    }

    private static void SumSquares(ReadOnlySpan<ulong> direction, Span<ulong> result)
    {
        Span<ulong> product = stackalloc ulong[Words];
        result.Clear();
        for (int index = 0; index < 3; index++)
        {
            ReadOnlySpan<ulong> component = Slot(direction, index);
            WideArithmetic.MultiplyMagnitudes(component, component, product);
            WideArithmetic.AddMagnitudeInto(product, result);
        }
    }

    private static void WritePlaneDirection(int j, int k, Signed320 u, Signed320 v,
        Span<ulong> direction, Span<int> signs)
    {
        direction.Clear();
        signs.Clear();
        Import(u, Slot(direction, j)); signs[j] = u.Sign;
        Import(v, Slot(direction, k)); signs[k] = v.Sign;
    }

    private static void WriteDirection(WideAxis3 axis, Span<ulong> direction, Span<int> signs)
    {
        for (int index = 0; index < 3; index++)
        {
            Signed320 value = BoxCylinderGeometry.GetComponent(axis, index);
            Import(value, Slot(direction, index));
            signs[index] = value.Sign;
        }
    }

    private static void Add(ReadOnlySpan<ulong> value, int sign, Span<ulong> destination, ref int destinationSign) =>
        WideArithmetic.AddShiftedSignedMagnitude(value, sign, 0, destination, ref destinationSign);
    private static bool IsZero(ReadOnlySpan<ulong> value) => WideArithmetic.GetActiveMagnitudeLength(value) == 0;
    private static Span<ulong> Slot(Span<ulong> values, int index) => values.Slice(index * Words, Words);
    private static ReadOnlySpan<ulong> Slot(ReadOnlySpan<ulong> values, int index) => values.Slice(index * Words, Words);
    private static void Import(Signed320 value, Span<ulong> result) => Import(Signed576.ExtendValue(value), result);
    private static void Import(Signed576 value, Span<ulong> result)
    {
        result.Clear();
        WideArithmetic.GetMagnitude(value, result[..9]);
    }
    private static WideAxis3 Unit(int axis) => new(
        Signed320.ExtendValue(Signed192.Signed(axis == 0 ? 1 : 0)),
        Signed320.ExtendValue(Signed192.Signed(axis == 1 ? 1 : 0)),
        Signed320.ExtendValue(Signed192.Signed(axis == 2 ? 1 : 0)));
    private static WideAxis3 Axis(Signed192 x, Signed192 y, Signed192 z) => new(
        Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z));

    private ref struct Selection
    {
        internal Span<ulong> Values;
        internal Span<int> Signs;
        internal int GapSign;
        internal bool HasValue;
        internal Vector3d LocalSigns;
        internal Selection(Span<ulong> values, Span<int> signs)
        {
            Values = values;
            Signs = signs;
            GapSign = 0;
            HasValue = false;
            LocalSigns = default;
        }
    }
}
