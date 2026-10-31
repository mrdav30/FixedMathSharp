//=======================================================================
// WideConvex2dRelations.CapsuleBoundaryContacts.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact circle and capsule witnesses on a retained finite planar face.</content>
internal static partial class WideConvex2dRelations
{
    /// <summary>
    /// Returns the deepest finite circle/capsule face witnesses. The caller
    /// supplies nonnegative dimensions and bounded mover-relative witnesses;
    /// origins may span the full scalar domain. A wholly buried body requires
    /// a separate recovery path rather than a remote supporting-plane contact.
    /// Plane admission may be borrowed only from an actual crossing of the
    /// same oriented plane in a positively connected declared surface.
    /// </summary>
    internal static int GetCapsuleBoundaryFaceContacts(Vector2d moverOrigin, Fixed64 moverRotation,
        Fixed64 coreLength, Fixed64 radius, Vector2d targetOrigin, Fixed64 targetRotation,
        FixedSegment2d edge, Vector2d inward, in RationalSegmentInterval clip,
        Span<FixedContactAnchors2d> contacts, bool planeAdmitted = false)
    {
        if (contacts.Length < 2) throw new ArgumentException("Two contact slots are required.", nameof(contacts));
        if (clip.HasWideBounds) throw new ArgumentException("Planar coverage bounds must fit Signed320.", nameof(clip));
        RotationFrame2d frame = new(targetRotation);
        GetTransformedEdge(frame, edge.Start, edge.End, out Signed192 ex, out Signed192 ey);
        Signed320 squared = WideArithmetic.AddProducts(ex, ex, ey, ey);
        if (squared.IsZero) return 0;
        GetTransformedEdge(frame, edge.Start, inward, out Signed192 ix, out Signed192 iy);
        WideAxis2d normal = new(WideArithmetic.Negate(ey), ex);
        int orientation = WideArithmetic.AddProducts(normal.X, ix, normal.Y, iy).Sign;
        if (orientation == 0) return 0;
        if (orientation < 0) normal = -normal;
        GetRelativePointNumerators(targetOrigin, targetRotation, edge.Start, moverOrigin, out Signed192 ax, out Signed192 ay);
        Signed320 centerU = WideArithmetic.Negate(WideArithmetic.AddProducts(ax, ex, ay, ey));
        Signed320 centerV = WideArithmetic.Negate(WideArithmetic.AddProducts(ax, normal.X, ay, normal.Y));
        Signed320 du = default, dv = default;
        bool hasCore = coreLength != Fixed64.Zero;
        if (hasCore)
        {
            _ = Vector2d.TryRotate(Vector2d.Forward, moverRotation, out Vector2d axis);
            Signed192 cx = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(Signed192.Raw(coreLength), Signed192.Raw(axis.X)));
            Signed192 cy = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(Signed192.Raw(coreLength), Signed192.Raw(axis.Y)));
            du = WideArithmetic.AddProducts(cx, ex, cy, ey); dv = WideArithmetic.AddProducts(cx, normal.X, cy, normal.Y);
        }
        Signed192 r = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(Signed192.Raw(radius), Signed192.One));
        Signed576 radialSquared = WideArithmetic.MultiplySigned320(WideArithmetic.MultiplySigned192(r, r), squared);
        // All coordinates share the exact scalar Sin/Cos chart. Doubled core
        // centers retain odd raw lengths; no clipped endpoint is materialized.
        Span<ulong> values = stackalloc ulong[18 * 2 * CapsuleFaceCandidates.Words];
        Span<int> signs = stackalloc int[18 * 2];
        Span<ulong> roots = stackalloc ulong[3 * CapsuleFaceCandidates.Words];
        var candidates = new CapsuleFaceCandidates(values, signs, roots, squared, clip);
        Signed320 u0 = WideArithmetic.SubtractSigned320(WideArithmetic.AddSigned320(centerU, centerU), du);
        Signed320 v0 = WideArithmetic.SubtractSigned320(WideArithmetic.AddSigned320(centerV, centerV), dv);
        candidates.AddDisk(u0, v0, radialSquared);
        if (hasCore)
        {
            candidates.AddDisk(WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(centerU, centerU), du),
                WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(centerV, centerV), dv), radialSquared);
            candidates.AddSides(u0, v0, du, dv, radialSquared);
        }
        if (!candidates.HasContact(planeAdmitted)) return 0;
        Vector2d direction = WideNormalization.GetNormalized(normal.X, normal.Y);
        contacts[0] = candidates.Materialize(6, moverOrigin, targetOrigin, targetRotation, edge, ex, ey, normal, ax, ay, direction);
        if (!candidates.HasTwo) return 1;
        contacts[1] = candidates.Materialize(9, moverOrigin, targetOrigin, targetRotation, edge, ex, ey, normal, ax, ay, direction);
        return 2;
    }

    private ref struct CapsuleFaceCandidates
    {
        // Full-domain point/edge projections need <200 bits, core projections
        // <230 and retained cut bounds <320. Radicands need <1040 bits and
        // fields <1120; 1280-bit banks retain all complete transient products.
        internal const int Words = 20;
        private readonly Span<ulong> _values, _roots;
        private readonly Span<int> _signs;
        private readonly Signed320 _squared, _lowerD, _upperD;
        private readonly Signed576 _lower, _upper;
        private bool _found, _outside;
        internal bool HasContact(bool planeAdmitted) => _found && (_outside || planeAdmitted);
        internal bool HasTwo => ContactQuadratic.CompareRatios(Q(6), Q(8), Root(1), Q(9), Q(11), Root(2)) != 0;
        private ContactQuadratic Q(int index) => ContactQuadratic.At(_values, _signs, index, Words);
        private Span<ulong> Root(int index) => _roots.Slice(index * Words, Words);

        internal CapsuleFaceCandidates(Span<ulong> values, Span<int> signs, Span<ulong> roots,
            Signed320 squared, in RationalSegmentInterval clip)
        {
            _values = values; _signs = signs; _roots = roots; _squared = squared;
            _lowerD = clip.IsClipped ? clip.LowerDenominator : Signed320.One;
            _upperD = clip.IsClipped ? clip.UpperDenominator : Signed320.One;
            _lower = WideArithmetic.MultiplySigned320(squared, clip.IsClipped ? clip.LowerNumerator : default);
            _upper = WideArithmetic.MultiplySigned320(squared, clip.IsClipped ? clip.UpperNumerator : Signed320.One);
            _found = _outside = false;
        }

        internal void AddDisk(Signed320 u, Signed320 v, Signed576 radialSquared)
        {
            for (int branch = -1; branch <= 1; branch += 2)
            {
                Q(0).Set(Signed576.ExtendValue(u)); Q(1).Set(Signed576.ExtendValue(v)); Q(2).Set(Integer(2));
                SetRoot(WideArithmetic.MultiplySigned576(radialSquared, 4));
                Q(1).Radical[0] = 1; Q(1).Signs[1] = branch;
                Keep();
            }
            for (int bound = 0; bound < 2; bound++)
            {
                Signed320 d = bound == 0 ? _lowerD : _upperD;
                Signed576 cut = bound == 0 ? _lower : _upper;
                Q(12).Set(WideArithmetic.MultiplySigned576(cut, 2));
                Q(13).Set(WideArithmetic.MultiplySigned320(u, d)); Q(12).Add(Q(13), -1);
                Q(13).Set(Signed576.ExtendValue(d));
                ContactQuadratic.Multiply(Q(13), Q(13), ReadOnlySpan<ulong>.Empty, Q(14));
                ContactQuadratic.Scale(Q(14), WideArithmetic.MultiplySigned576(radialSquared, 4), Q(15));
                ContactQuadratic.Multiply(Q(12), Q(12), ReadOnlySpan<ulong>.Empty, Q(14)); Q(15).Add(Q(14), -1);
                if (Q(15).Signs[0] < 0) continue;
                Q(15).Rational.CopyTo(Root(0));
                for (int branch = -1; branch <= 1; branch += 2)
                {
                    Q(0).Set(WideArithmetic.MultiplySigned576(cut, 2));
                    Q(1).Set(WideArithmetic.MultiplySigned320(v, d)); Q(1).Radical[0] = 1; Q(1).Signs[1] = branch;
                    Q(2).Set(WideArithmetic.MultiplySigned576(Signed576.ExtendValue(d), 2));
                    Keep();
                }
            }
        }

        internal void AddSides(Signed320 u, Signed320 v, Signed320 du, Signed320 dv, Signed576 radialSquared)
        {
            if (du.IsZero) return; // Vertical sides have no interior tangent-cut extremum.
            Signed576 h = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(du, du), WideArithmetic.MultiplySigned320(dv, dv));
            Q(12).Set(h); Q(13).Set(radialSquared);
            ContactQuadratic.Multiply(Q(12), Q(13), ReadOnlySpan<ulong>.Empty, Q(14)); Q(14).Rational.CopyTo(Root(0));
            for (int bound = 0; bound < 2; bound++)
            for (int side = -1; side <= 1; side += 2)
            {
                Signed320 d = bound == 0 ? _lowerD : _upperD;
                Signed576 cut = bound == 0 ? _lower : _upper;
                // q=(cut-u/2+side*dv*sqrt(QH)/H)/du must stay on
                // the finite core. Its two signed tests retain the radical.
                Q(12).Set(WideArithmetic.MultiplySigned576(cut, 2)); Q(13).Set(WideArithmetic.MultiplySigned320(u, d)); Q(12).Add(Q(13), -1);
                ContactQuadratic.Scale(Q(12), h, Q(15));
                Q(13).Set(WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(dv, d), side * 2));
                Q(13).Rational.CopyTo(Q(15).Radical); Q(15).Signs[1] = Q(13).Signs[0];
                Q(12).Set(WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(du, d), 2));
                ContactQuadratic.Scale(Q(12), h, Q(16));
                if (du.Sign < 0) { Q(15).MultiplySign(-1); Q(16).MultiplySign(-1); }
                if (Q(15).Sign(Root(0)) < 0) continue;
                Q(16).Add(Q(15), -1); if (Q(16).Sign(Root(0)) < 0) continue;
                Q(0).Set(WideArithmetic.MultiplySigned576(cut, 2));
                ContactQuadratic.Scale(Q(0), Signed576.ExtendValue(du), Q(12)); Q(12).CopyTo(Q(0));
                Q(12).Set(WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(v, du), WideArithmetic.MultiplySigned320(u, dv)));
                ContactQuadratic.Scale(Q(12), Signed576.ExtendValue(d), Q(1));
                Q(12).Set(WideArithmetic.MultiplySigned576(cut, 2)); ContactQuadratic.Scale(Q(12), Signed576.ExtendValue(dv), Q(13)); Q(1).Add(Q(13));
                Q(12).Set(WideArithmetic.MultiplySigned576(Signed576.ExtendValue(d), side * 2));
                Q(12).Rational.CopyTo(Q(1).Radical); Q(1).Signs[1] = Q(12).Signs[0];
                Q(2).Set(WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(du, d), 2));
                if (du.Sign < 0) { Q(0).MultiplySign(-1); Q(1).MultiplySign(-1); Q(2).MultiplySign(-1); }
                Keep();
            }
        }

        private void Keep()
        {
            ContactQuadratic.Scale(Q(0), Signed576.ExtendValue(_lowerD), Q(3));
            ContactQuadratic.Scale(Q(2), _lower, Q(4)); Q(3).Add(Q(4), -1);
            if (Q(3).Sign(Root(0)) < 0) return;
            ContactQuadratic.Scale(Q(2), _upper, Q(3));
            ContactQuadratic.Scale(Q(0), Signed576.ExtendValue(_upperD), Q(4)); Q(3).Add(Q(4), -1);
            if (Q(3).Sign(Root(0)) < 0) return;
            int sign = Q(1).Sign(Root(0));
            if (sign <= 0) _outside = true;
            if (sign < 0) return;
            int comparison = _found ? ContactQuadratic.CompareRatios(Q(1), Q(2), Root(0), Q(7), Q(8), Root(1)) : 1;
            if (comparison > 0) { _found = true; Copy(6, 1); Copy(9, 2); }
            else if (comparison == 0)
            {
                if (ContactQuadratic.CompareRatios(Q(0), Q(2), Root(0), Q(6), Q(8), Root(1)) < 0) Copy(6, 1);
                if (ContactQuadratic.CompareRatios(Q(0), Q(2), Root(0), Q(9), Q(11), Root(2)) > 0) Copy(9, 2);
            }
        }

        private void Copy(int slot, int root)
        { Q(0).CopyTo(Q(slot)); Q(1).CopyTo(Q(slot + 1)); Q(2).CopyTo(Q(slot + 2)); Root(0).CopyTo(Root(root)); }

        private void SetRoot(Signed576 value) { Q(17).Set(value); Q(17).Rational.CopyTo(Root(0)); }
        private static Signed576 Integer(long value) => Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(value)));

        internal FixedContactAnchors2d Materialize(int slot, Vector2d moverOrigin, Vector2d targetOrigin,
            Fixed64 targetRotation, FixedSegment2d edge, Signed192 ex, Signed192 ey, WideAxis2d normal,
            Signed192 ax, Signed192 ay, Vector2d direction)
        {
            ReadOnlySpan<ulong> root = Root(slot == 6 ? 1 : 2);
            Span<ulong> metric = stackalloc ulong[9], scale = stackalloc ulong[9];
            WideArithmetic.GetMagnitude(Signed576.ExtendValue(_squared), metric);
            Signed576 scaledLength = WideArithmetic.MultiplySigned320(_squared, Signed192.One);
            WideArithmetic.GetMagnitude(scaledLength, scale);
            bool represented = ContactQuadratic.TryRoundRootRatio(Q(slot + 1), Q(slot + 2), root, metric, scale, out Fixed64 depth);
            if (!represented) depth = Fixed64.MaxValue;
            Vector2d moving = new(Coordinate(slot, root, ex, normal.X, ax, true), Coordinate(slot, root, ey, normal.Y, ay, true));
            Vector2d target = new(Coordinate(slot, root, WideArithmetic.Difference(edge.End.X, edge.Start.X), default, Signed192.Raw(edge.Start.X), false),
                Coordinate(slot, root, WideArithmetic.Difference(edge.End.Y, edge.Start.Y), default, Signed192.Raw(edge.Start.Y), false));
            return new FixedContactAnchors2d(new FixedPointAnchor2d(moverOrigin, Fixed64.Zero, moving),
                new FixedPointAnchor2d(targetOrigin, targetRotation, target), direction, depth, !represented);
        }

        private Fixed64 Coordinate(int slot, ReadOnlySpan<ulong> root, Signed192 tangent, Signed192 normal,
            Signed192 origin, bool world)
        {
            ContactQuadratic.Scale(Q(slot), Signed576.ExtendValue(Signed320.ExtendValue(tangent)), Q(3));
            ContactQuadratic.Scale(Q(slot + 1), Signed576.ExtendValue(Signed320.ExtendValue(normal)), Q(4)); Q(3).Add(Q(4));
            ContactQuadratic.Scale(Q(slot + 2), WideArithmetic.MultiplySigned320(_squared, origin), Q(4)); Q(3).Add(Q(4));
            ContactQuadratic.Scale(Q(slot + 2), world ? WideArithmetic.MultiplySigned320(_squared, Signed192.One) : Signed576.ExtendValue(_squared), Q(5));
            return ContactQuadratic.RoundRatio(Q(3), Q(5), root);
        }
    }
}
