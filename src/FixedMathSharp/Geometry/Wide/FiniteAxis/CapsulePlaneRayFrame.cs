//=======================================================================
// CapsulePlaneRayFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;









/// <summary>Canonical finite plane and exact capsule-local coordinates for admitted plane rays.</summary>
internal readonly struct CapsulePlaneRayFrame
{
    internal readonly CapsuleFiniteSectionFrame Finite;
    internal readonly WideDirection3d Normal;
    internal readonly WideAxis3 AuthoredNormal;
    internal readonly Signed832 PlaneConstant, NormalSquared;
    private readonly bool _wideParameters;
    internal bool UsesWideBanks => Finite.ShapeFrame.HasExactScale || _wideParameters;
    // Affine event points retain <7680-bit radicands and <12288-bit
    // coefficients. Opposite-ray value norms square the coefficients and
    // include the point radicand plus <=618 conditioning bits per factor;
    // 544 words cover that complete norm. Ordinary banks stay unchanged.
    internal int FieldWords => UsesWideBanks ? 192 : CapsulePlaneRaySelection.FieldWords;
    internal int RootWords => UsesWideBanks ? 120 : CapsulePlaneRaySelection.RootWords;
    internal int ValueWords => UsesWideBanks ? 544 : CapsulePlaneRayValue.PolynomialWords;
    internal int SelectionStorageWords => CapsulePlaneRaySelection.GetStorageWords(FieldWords, RootWords, ValueWords);
    internal CapsulePlaneRayFrame WithWideParameters() => UsesWideBanks ? this : new CapsulePlaneRayFrame(this);
    private CapsulePlaneRayFrame(in CapsulePlaneRayFrame source) { this = source; _wideParameters = true; }
    internal ScaledRigidFrame3d SourceFrame => Finite.ShapeFrame.SourceFrame;

    /// <summary>Prepares a explicit sphere plane ray; its rational base point is supplied separately.</summary>
    /// <remarks>Only the rational-point accumulator is valid for this frame, not authored plane/edge reconstruction.</remarks>
    internal static CapsulePlaneRayFrame ForSpherePointRay(Fixed64 radius, in WideDirection3d direction,
        in Signed832 planeConstant, bool wideParameters = false) => new(radius, direction, planeConstant, wideParameters);

    private CapsulePlaneRayFrame(Fixed64 radius, in WideDirection3d direction, in Signed832 planeConstant, bool wideParameters)
    {
        this = default;
        Finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, radius);
        Normal = direction; PlaneConstant = planeConstant;
        AuthoredNormal = new WideAxis3(Signed320.NarrowValue(direction.X),
            Signed320.NarrowValue(direction.Y), Signed320.NarrowValue(direction.Z));
        NormalSquared = direction.SquaredLength;
        _wideParameters = wideParameters;
    }

    /// <summary>Prepares a capsule-local ray chart for exact pretransformed native geometry.</summary>
    internal static CapsulePlaneRayFrame ForCapsulePointRay(Fixed64 length, Fixed64 radius,
        Signed320 coordinateDenominator, in WideDirection3d direction, in Signed832 planeConstant) =>
        new(length, radius, coordinateDenominator, direction, planeConstant);

    private CapsulePlaneRayFrame(Fixed64 length, Fixed64 radius, Signed320 coordinateDenominator,
        in WideDirection3d direction, in Signed832 planeConstant)
    {
        this = default;
        Finite = new CapsuleFiniteSectionFrame(length, radius, coordinateDenominator);
        Normal = direction; PlaneConstant = planeConstant; NormalSquared = direction.SquaredLength;
        AuthoredNormal = new WideAxis3(Signed320.NarrowValue(direction.X),
            Signed320.NarrowValue(direction.Y), Signed320.NarrowValue(direction.Z));
    }

    internal CapsulePlaneRayFrame(FixedTriangle planeTriangle, in CapsuleFiniteSectionFrame finite)
    {
        _wideParameters = false;
        Finite = finite;
        planeTriangle.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        Span<Signed320> primitive = stackalloc Signed320[3]
        {
            Signed320.ExtendValue(nx), Signed320.ExtendValue(ny), Signed320.ExtendValue(nz)
        };
        if (!nx.IsZero || !ny.IsZero || !nz.IsZero) WideArithmetic.ReduceCommonScale(primitive);
        int sign = primitive[0].Sign != 0 ? primitive[0].Sign
            : primitive[1].Sign != 0 ? primitive[1].Sign : primitive[2].Sign;
        if (sign < 0)
            for (int index = 0; index < 3; index++) primitive[index] = WideArithmetic.Negate(primitive[index]);
        AuthoredNormal = new WideAxis3(primitive[0], primitive[1], primitive[2]);
        WideAxis3 transformed = WideRigidProjection.TransformLocalAxis(finite.ShapeFrame.Basis,
            Signed192.NarrowProven(primitive[0]), Signed192.NarrowProven(primitive[1]), Signed192.NarrowProven(primitive[2]));
        primitive[0] = transformed.X; primitive[1] = transformed.Y; primitive[2] = transformed.Z;
        if (!transformed.IsZero) WideArithmetic.ReduceCommonScale(primitive);
        var compactNormal = new WideAxis3(primitive[0], primitive[1], primitive[2]);
        Normal = new WideDirection3d(compactNormal);
        PlaneConstant = Signed832.ExtendValue(WideAxis3.Dot(compactNormal, finite.Transform(planeTriangle.A)));
        NormalSquared = Signed832.ExtendValue(WideAxis3.Dot(compactNormal, compactNormal));
    }

    internal CapsulePlaneRayFrame(FixedTriangle planeTriangle, in ScaledRigidFrame3d sourceFrame,
        in CapsuleFiniteSectionFrame finite)
    {
        if (sourceFrame.IsRigid)
        {
            this = new CapsulePlaneRayFrame(planeTriangle, finite);
            return;
        }
        _wideParameters = false;
        Finite = finite;
        WideDirection3d first = WideDirection3d.GetLocalEdge(sourceFrame, planeTriangle.A, planeTriangle.B);
        WideDirection3d second = WideDirection3d.GetLocalEdge(sourceFrame, planeTriangle.A, planeTriangle.C);
        WideDirection3d cross = WideDirection3d.Cross(first, second);
        Span<Signed320> primitive = stackalloc Signed320[3]
        {
            Signed320.NarrowValue(cross.X), Signed320.NarrowValue(cross.Y), Signed320.NarrowValue(cross.Z)
        };
        if (!cross.IsZero) WideArithmetic.ReduceCommonScale(primitive);
        int sign = primitive[0].Sign != 0 ? primitive[0].Sign
            : primitive[1].Sign != 0 ? primitive[1].Sign : primitive[2].Sign;
        if (sign < 0)
            for (int index = 0; index < 3; index++) primitive[index] = WideArithmetic.Negate(primitive[index]);
        AuthoredNormal = new WideAxis3(primitive[0], primitive[1], primitive[2]);
        WideDirection3d transformed = new WideDirection3d(AuthoredNormal).Transform(finite.ShapeFrame.Basis);
        Span<Signed576> reduced = stackalloc Signed576[3] { transformed.X, transformed.Y, transformed.Z };
        if (!transformed.IsZero) WideArithmetic.ReduceCommonScale(reduced);
        Normal = new WideDirection3d(reduced[0], reduced[1], reduced[2]);
        PlaneConstant = WideDirection3d.Dot(Normal, finite.Transform(planeTriangle.A));
        NormalSquared = Normal.SquaredLength;
    }

    internal Vector3d GetWorldNormal(FixedQuaternion authoredRotation)
    {
        if (Finite.ShapeFrame.HasExactScale)
            return new WideDirection3d(AuthoredNormal).Transform(SourceFrame.GeometryBasis).GetNormal(false);
        WideAxis3 normal = WideRigidProjection.TransformLocalAxis(new WideRationalBasis3d(authoredRotation),
            Signed192.NarrowProven(AuthoredNormal.X), Signed192.NarrowProven(AuthoredNormal.Y),
            Signed192.NarrowProven(AuthoredNormal.Z));
        return WideNormalization.GetNormalized(normal.X, normal.Y, normal.Z);
    }

    /// <summary>Compares exact points in the authored frame; the common translation cancels.</summary>
    internal int ComparePoints(scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z,
        scoped ContactQuadratic d, scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic otherX,
        scoped ContactQuadratic otherY, scoped ContactQuadratic otherZ, scoped ContactQuadratic otherD,
        scoped ReadOnlySpan<ulong> otherRoot)
    {
        // Basis covectors add at most 130 bits and two carries; retained bank
        // capacity does not determine the width of the actual point products.
        int words = Math.Max(9, Math.Max(Math.Max(ContactQuadratic.ActiveWords(x),
            Math.Max(ContactQuadratic.ActiveWords(y), ContactQuadratic.ActiveWords(z))),
            Math.Max(ContactQuadratic.ActiveWords(otherX), Math.Max(ContactQuadratic.ActiveWords(otherY),
                ContactQuadratic.ActiveWords(otherZ)))) + 3);
        Span<ulong> values = stackalloc ulong[6 * words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic first = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic second = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 2, words);
        WideRationalBasis3d basis = Finite.ShapeFrame.Basis;
        for (int axis = 0; axis < 3; axis++)
        {
            Signed192 bx = axis == 0 ? basis.Xx : axis == 1 ? basis.Yx : basis.Zx;
            Signed192 by = axis == 0 ? basis.Xy : axis == 1 ? basis.Yy : basis.Zy;
            Signed192 bz = axis == 0 ? basis.Xz : axis == 1 ? basis.Yz : basis.Zz;
            DotPoint(Signed576.ExtendValue(Signed320.ExtendValue(bx)),
                Signed576.ExtendValue(Signed320.ExtendValue(by)), Signed576.ExtendValue(Signed320.ExtendValue(bz)),
                x, y, z, first, term);
            DotPoint(Signed576.ExtendValue(Signed320.ExtendValue(bx)),
                Signed576.ExtendValue(Signed320.ExtendValue(by)), Signed576.ExtendValue(Signed320.ExtendValue(bz)),
                otherX, otherY, otherZ, second, term);
            int comparison = ContactQuadratic.CompareRatios(first, d, root, second, otherD, otherRoot);
            if (comparison != 0) return comparison;
        }
        return 0;
    }

    /// <summary>Tests an exact homogeneous capsule-local point against the closed finite triangle.</summary>
    internal bool ContainsTrianglePoint(FixedTriangle triangle, ContactQuadratic x, ContactQuadratic y,
        ContactQuadratic z, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        if (NormalSquared.IsZero || denominator.Sign(root) <= 0) return false;
        // Plane/wall products add at most 526 bits to a point coefficient.
        // Input banks may be larger than this owner's rational witness bank.
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(x), ContactQuadratic.ActiveWords(y)),
            Math.Max(ContactQuadratic.ActiveWords(z), ContactQuadratic.ActiveWords(denominator)))
            + (Finite.ShapeFrame.HasExactScale ? 13 : 9);
        Span<ulong> values = stackalloc ulong[4 * words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic value = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 1, words);
        DotPoint(Normal.X, Normal.Y, Normal.Z,
            x, y, z, value, term);
        ContactQuadratic.Scale(denominator, PlaneConstant, term); value.Add(term, -1);
        if (value.Sign(root) != 0) return false;
        for (int edge = 0; edge < 3; edge++)
        {
            GetTriangleEdgeWall(triangle, edge, out Signed832 wx, out Signed832 wy, out Signed832 wz, out Signed832 offset);
            DotPoint(wx, wy, wz, x, y, z, value, term);
            ContactQuadratic.Scale(denominator, offset, term); value.Add(term, -1);
            if (value.Sign(root) < 0) return false;
        }
        return true;
    }

    internal void GetTriangleEdgeWall(FixedTriangle triangle, int index,
        out Signed832 x, out Signed832 y, out Signed832 z, out Signed832 offset)
    {
        FixedSegment edge = triangle.GetEdge(index);
        if (Finite.ShapeFrame.HasExactScale)
        {
            WideDirection3d deltaExact = WideDirection3d.GetLocalEdge(SourceFrame, edge.Start, edge.End);
            WideDirection3d wallExact = WideDirection3d.Cross(new WideDirection3d(AuthoredNormal), deltaExact)
                .Transform(Finite.ShapeFrame.Basis);
            x = Signed832.ExtendValue(wallExact.X); y = Signed832.ExtendValue(wallExact.Y);
            z = Signed832.ExtendValue(wallExact.Z);
            offset = WideDirection3d.Dot(wallExact, Finite.Transform(edge.Start));
            WideAxis3 oppositeExact = Finite.Transform(index == 0 ? triangle.C : index == 1 ? triangle.A : triangle.B);
            if (WideArithmetic.SubtractSigned832(WideDirection3d.Dot(wallExact, oppositeExact), offset).Sign < 0)
            {
                x = WideArithmetic.SubtractSigned832(default, x); y = WideArithmetic.SubtractSigned832(default, y);
                z = WideArithmetic.SubtractSigned832(default, z); offset = WideArithmetic.SubtractSigned832(default, offset);
            }
            return;
        }
        WideAxis3 delta = new(
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.X), Signed192.Raw(edge.Start.X))),
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.Y), Signed192.Raw(edge.Start.Y))),
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.Z), Signed192.Raw(edge.Start.Z))));
        // Form the authored wall before transforming it: 130-bit normal,
        // 65-bit edge, 130-bit relative basis keep the wall below 327 bits.
        WideAxis3 wall = WideAxis3.Cross(AuthoredNormal, delta);
        WideRigidProjection.TransformLocalAxis(Finite.ShapeFrame.Basis, wall, out Signed576 wx, out Signed576 wy, out Signed576 wz);
        x = Signed832.ExtendValue(wx); y = Signed832.ExtendValue(wy); z = Signed832.ExtendValue(wz);
        offset = Signed832.ExtendValue(DotWall(wx, wy, wz, Finite.Transform(edge.Start)));
        WideAxis3 opposite = Finite.Transform(index == 0 ? triangle.C : index == 1 ? triangle.A : triangle.B);
        if (WideArithmetic.SubtractSigned832(Signed832.ExtendValue(DotWall(wx, wy, wz, opposite)), offset).Sign < 0)
        {
            x = WideArithmetic.SubtractSigned832(default, x); y = WideArithmetic.SubtractSigned832(default, y);
            z = WideArithmetic.SubtractSigned832(default, z); offset = WideArithmetic.SubtractSigned832(default, offset);
        }
    }

    private static Signed576 DotWall(Signed576 x, Signed576 y, Signed576 z, WideAxis3 point) =>
        Signed576.NarrowValue(WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(WideArithmetic.MultiplySigned576ToSigned704(x, point.X),
                WideArithmetic.MultiplySigned576ToSigned704(y, point.Y)),
            WideArithmetic.MultiplySigned576ToSigned704(z, point.Z)));

    private static void DotPoint(Signed576 wx, Signed576 wy, Signed576 wz, ContactQuadratic x,
        ContactQuadratic y, ContactQuadratic z, ContactQuadratic result, ContactQuadratic term)
    {
        ContactQuadratic.Scale(x, wx, result);
        ContactQuadratic.Scale(y, wy, term); result.Add(term);
        ContactQuadratic.Scale(z, wz, term); result.Add(term);
    }

    private static void DotPoint(Signed832 wx, Signed832 wy, Signed832 wz, ContactQuadratic x,
        ContactQuadratic y, ContactQuadratic z, ContactQuadratic result, ContactQuadratic term)
    {
        ContactQuadratic.Scale(x, wx, result);
        ContactQuadratic.Scale(y, wy, term); result.Add(term);
        ContactQuadratic.Scale(z, wz, term); result.Add(term);
    }
}
