//=======================================================================
// CapsulePlaneRayFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>An unbounded plane or one authored finite perimeter segment.</summary>
internal readonly struct CapsulePlaneRayEventSource
{
    internal readonly bool IsSegment;
    internal readonly Vector3d A, B;
    internal static CapsulePlaneRayEventSource Plane => default;
    internal CapsulePlaneRayEventSource(FixedSegment segment)
    { IsSegment = true; A = segment.Start; B = segment.End; }
}

internal enum CapsulePlaneRayEventKind : byte { Support, Sagittal, Seam, Endpoint, Section, Stationary, Circle }

/// <summary>A reconstructible exact point in the plane or its finite perimeter.</summary>
internal readonly struct CapsulePlaneRayEvent
{
    internal readonly CapsulePlaneRayEventKind Kind;
    internal readonly sbyte Feature, Branch, Orientation;
    internal CapsulePlaneRayEvent(CapsulePlaneRayEventKind kind, int feature = 0, int branch = 0, int orientation = 0)
    { Kind = kind; Feature = (sbyte)feature; Branch = (sbyte)branch; Orientation = (sbyte)orientation; }
    internal bool Matches(in CapsulePlaneRayEvent other) =>
        Kind == other.Kind && Feature == other.Feature && Branch == other.Branch && Orientation == other.Orientation;
}

/// <summary>Synchronous exact event consumption; borrowed candidate storage cannot escape.</summary>
internal delegate void CapsulePlaneRayEventVisitor(in CapsulePlaneRayEventSource source,
    in CapsulePlaneRayFrame frame, CapsulePlaneRayEvent descriptor, int orientation,
    scoped in CapsulePlaneRaySelection candidate);

/// <summary>Canonical finite plane and exact capsule-local coordinates for admitted plane rays.</summary>
internal readonly struct CapsulePlaneRayFrame
{
    internal readonly CapsuleFiniteSectionFrame Finite;
    internal readonly WideAxis3 Normal, AuthoredNormal;
    internal readonly Signed576 PlaneConstant, NormalSquared;

    internal CapsulePlaneRayFrame(FixedTriangle planeTriangle, in CapsuleFiniteSectionFrame finite)
    {
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
        Normal = new WideAxis3(primitive[0], primitive[1], primitive[2]);
        PlaneConstant = WideAxis3.Dot(Normal, finite.Transform(planeTriangle.A));
        NormalSquared = WideAxis3.Dot(Normal, Normal);
    }

    internal Vector3d GetWorldNormal(FixedQuaternion authoredRotation)
    {
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
            Math.Max(ContactQuadratic.ActiveWords(z), ContactQuadratic.ActiveWords(denominator))) + 9;
        Span<ulong> values = stackalloc ulong[4 * words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic value = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 1, words);
        DotPoint(Signed576.ExtendValue(Normal.X), Signed576.ExtendValue(Normal.Y), Signed576.ExtendValue(Normal.Z),
            x, y, z, value, term);
        ContactQuadratic.Scale(denominator, PlaneConstant, term); value.Add(term, -1);
        if (value.Sign(root) != 0) return false;
        for (int edge = 0; edge < 3; edge++)
        {
            GetTriangleEdgeWall(triangle, edge, out Signed576 wx, out Signed576 wy, out Signed576 wz, out Signed576 offset);
            DotPoint(wx, wy, wz, x, y, z, value, term);
            ContactQuadratic.Scale(denominator, offset, term); value.Add(term, -1);
            if (value.Sign(root) < 0) return false;
        }
        return true;
    }

    internal void GetTriangleEdgeWall(FixedTriangle triangle, int index,
        out Signed576 x, out Signed576 y, out Signed576 z, out Signed576 offset)
    {
        FixedSegment edge = triangle.GetEdge(index);
        WideAxis3 delta = new(
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.X), Signed192.Raw(edge.Start.X))),
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.Y), Signed192.Raw(edge.Start.Y))),
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.Z), Signed192.Raw(edge.Start.Z))));
        // Form the authored wall before transforming it: 130-bit normal,
        // 65-bit edge, 130-bit relative basis keep the wall below 327 bits.
        WideAxis3 wall = WideAxis3.Cross(AuthoredNormal, delta);
        WideRigidProjection.TransformLocalAxis(Finite.ShapeFrame.Basis, wall, out x, out y, out z);
        offset = DotWall(x, y, z, Finite.Transform(edge.Start));
        WideAxis3 opposite = Finite.Transform(index == 0 ? triangle.C : index == 1 ? triangle.A : triangle.B);
        if (WideArithmetic.SubtractSigned576(DotWall(x, y, z, opposite), offset).Sign < 0)
        {
            x = WideArithmetic.SubtractSigned576(default, x); y = WideArithmetic.SubtractSigned576(default, y);
            z = WideArithmetic.SubtractSigned576(default, z); offset = WideArithmetic.SubtractSigned576(default, offset);
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
}
