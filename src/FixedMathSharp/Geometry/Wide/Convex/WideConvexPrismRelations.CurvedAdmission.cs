//=======================================================================
// WideConvexPrismRelations.CurvedAdmission.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Owns complete closed cylinder/cone admission against the finite prism boundary.</content>
internal static partial class WideConvexPrismRelations
{
    internal static bool IntersectsCurvedPrism(bool cone, Vector3d center, FixedQuaternion rotation,
        Vector3d localAxis, Fixed64 length, Fixed64 radius, Vector3d origin, Fixed64 yaw,
        ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness)
    {
        WideOrientedBox.GetRotatedLocalAxisNumerators(rotation, localAxis,
            out Signed192 x, out Signed192 y, out Signed192 z, out Signed192 denominator);
        return IntersectsCurvedPrism(cone ? FiniteShapeKind.Cone : FiniteShapeKind.Cylinder,
            center, new RigidAxis3(x, y, z, denominator), length, radius, origin, yaw, offsets, halfThickness);
    }

    private static bool IntersectsCurvedPrism(FiniteShapeKind kind, Vector3d center,
        RigidAxis3 axis, Fixed64 length, Fixed64 radius, Vector3d origin, Fixed64 yaw,
        ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness)
    {
        Signed192 centerY = WideArithmetic.Difference(center.Y, origin.Y);
        bool footprintContainsCenter = WideConvex2dRelations.ContainsPoint(new Vector2d(center.X, center.Z),
            new Vector2d(origin.X, origin.Z), yaw, offsets);
        if (footprintContainsCenter
            && WideArithmetic.AddSigned192(centerY, Signed192.Raw(halfThickness)).Sign >= 0
            && WideArithmetic.SubtractSigned192(centerY, Signed192.Raw(halfThickness)).Sign <= 0)
            return true;

        // The projected contact already proved world-Up overlap. An upright
        // finite core therefore meets the prism when its footprint is inside.
        if (footprintContainsCenter && axis.X.IsZero && axis.Z.IsZero)
            return true;

        var geometry = new CurvedPrismAdmission(kind, axis, center, length, radius,
            origin, yaw, offsets, halfThickness);
        // Every certified section point is already a positive intersection
        // certificate. Try these before constructing any edge quadratics.
        Axis3 bottom = geometry.Vertex(0, -1), top = geometry.Vertex(0, 1);
        Axis3 up = FromDirection(Vector3d.Up);
        if (geometry.IntersectsFace(up, bottom) || geometry.IntersectsFace(up, top))
            return true;
        for (int index = 0; index < offsets.Length; index++)
        {
            int next = index + 1 == offsets.Length ? 0 : index + 1;
            Axis3 edge = GetPlanarEdge(yaw, offsets[index], offsets[next]);
            if (!edge.IsZero && geometry.IntersectsFace(
                    new Axis3(edge.Z, default, WideArithmetic.Negate(edge.X)), geometry.Vertex(index, -1)))
                return true;
        }

        // A convex plane section that meets a face without meeting its boundary
        // lies wholly in that face. Thus one intrinsic point per nonempty
        // section, together with all real prism edges, is complete. Center
        // containment covers a primitive wholly enclosed by the prism.
        for (int index = 0; index < offsets.Length; index++)
        {
            int next = index + 1 == offsets.Length ? 0 : index + 1;
            bottom = geometry.Vertex(index, -1); top = geometry.Vertex(index, 1);
            if (geometry.IntersectsEdge(bottom, top)
                || geometry.IntersectsEdge(bottom, geometry.Vertex(next, -1))
                || geometry.IntersectsEdge(top, geometry.Vertex(next, 1)))
                return true;
        }
        return false;
    }

    private readonly ref struct CurvedPrismAdmission
    {
        // Native yaw numerators V use a Q denominator. P=2*D*V,
        // H=Lraw*A and R=2*D*Q*Rraw therefore share one exact coordinate
        // system without materializing world points or a rounded axis frame.
        // D<2^65, |A|<2^98, |P|<2^164: axial values fit 320 bits.
        // For side normals |n|<2^98, J=q*n²-(n.A)² fits 396 bits;
        // cap-foot numerators fit 562 bits and containment/disk products
        // fit 832 bits. Every narrowing below follows these bounds.
        private readonly FiniteShapeKind _kind;
        private readonly RigidAxis3 _axis;
        private readonly Axis3 _direction;
        private readonly Vector3d _center, _origin;
        private readonly Fixed64 _yaw, _halfThickness;
        private readonly ReadOnlySpan<Vector2d> _offsets;
        private readonly Signed192 _length;
        private readonly Signed320 _squared, _cap;
        private readonly Signed576 _radiusSquared;

        internal CurvedPrismAdmission(FiniteShapeKind kind, RigidAxis3 axis, Vector3d center,
            Fixed64 length, Fixed64 radius, Vector3d origin, Fixed64 yaw,
            ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness)
        {
            _kind = kind; _axis = axis; _direction = axis.ToWide();
            _center = center; _origin = origin; _yaw = yaw;
            _offsets = offsets; _halfThickness = halfThickness; _length = Signed192.Raw(length);
            _squared = GetAxisSquared(axis);
            _cap = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                _squared, Signed320.ExtendValue(_length)));
            Signed320 radial = WideArithmetic.MultiplySigned192(axis.RotationDenominator, Signed192.Raw(radius));
            radial = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(radial, Signed320.ExtendValue(Scale)));
            radial = WideArithmetic.AddSigned320(radial, radial);
            _radiusSquared = WideArithmetic.MultiplySigned320(radial, radial);
        }

        internal Axis3 Vertex(int index, int verticalSign)
        {
            WideConvex2dRelations.GetRelativePointNumerators(
                new Vector2d(_origin.X, _origin.Z), _yaw, _offsets[index],
                new Vector2d(_center.X, _center.Z), out Signed192 x, out Signed192 z);
            Signed192 y = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
                WideArithmetic.AddSigned192(WideArithmetic.SubtractSigned192(Signed192.Raw(_origin.Y),
                    Signed192.Raw(_center.Y)), Signed192.Raw(verticalSign * _halfThickness)), Scale));
            return new Axis3(ScaleCoordinate(x), ScaleCoordinate(y), ScaleCoordinate(z));
        }

        private Signed320 ScaleCoordinate(Signed192 value)
        {
            Signed320 result = WideArithmetic.MultiplySigned192(value, _axis.RotationDenominator);
            return WideArithmetic.AddSigned320(result, result);
        }

        internal bool Contains(Signed576 x, Signed576 y, Signed576 z, Signed576 denominator)
        {
            Axis3 first = Vertex(0, -1), top = Vertex(0, 1);
            if (Relative(y, first.Y, denominator).Sign < 0 || Relative(y, top.Y, denominator).Sign > 0)
                return false;
            Signed320 minimumX = first.X, maximumX = first.X, minimumZ = first.Z, maximumZ = first.Z;
            int side = 0;
            for (int index = 0; index < _offsets.Length; index++)
            {
                Axis3 start = Vertex(index, -1);
                Axis3 end = Vertex(index + 1 == _offsets.Length ? 0 : index + 1, -1);
                Signed320 edgeX = WideArithmetic.SubtractSigned320(end.X, start.X);
                Signed320 edgeZ = WideArithmetic.SubtractSigned320(end.Z, start.Z);
                Signed832 cross = WideArithmetic.SubtractSigned832(
                    WideArithmetic.MultiplySigned576ToSigned832(Relative(z, start.Z, denominator), edgeX),
                    WideArithmetic.MultiplySigned576ToSigned832(Relative(x, start.X, denominator), edgeZ));
                if (cross.Sign != 0)
                {
                    if (side != 0 && side != cross.Sign) return false;
                    side = cross.Sign;
                }
                if (WideArithmetic.SubtractSigned320(start.X, minimumX).Sign < 0) minimumX = start.X;
                if (WideArithmetic.SubtractSigned320(start.X, maximumX).Sign > 0) maximumX = start.X;
                if (WideArithmetic.SubtractSigned320(start.Z, minimumZ).Sign < 0) minimumZ = start.Z;
                if (WideArithmetic.SubtractSigned320(start.Z, maximumZ).Sign > 0) maximumZ = start.Z;
            }
            // Collinear and point footprints have no interior halfspaces;
            // their finite coordinate ranges must still contain the point.
            return side != 0 || (Relative(x, minimumX, denominator).Sign >= 0
                && Relative(x, maximumX, denominator).Sign <= 0
                && Relative(z, minimumZ, denominator).Sign >= 0
                && Relative(z, maximumZ, denominator).Sign <= 0);
        }

        internal bool IntersectsFace(Axis3 normal, Axis3 point)
        {
            Signed320 c = Signed320.NarrowValue(CurvedDot(normal, point));
            Signed320 b = Signed320.NarrowValue(CurvedDot(normal, _direction));
            if (b.IsZero)
            {
                // Center containment was already rejected; a coplanar section
                // can enter this face only through its real boundary edges.
                if (c.IsZero) return false;
            }
            else
            {
                Signed320 absoluteB = b.Sign < 0 ? WideArithmetic.Negate(b) : b;
                Signed320 absoluteC = c.Sign < 0 ? WideArithmetic.Negate(c) : c;
                if (WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(
                        absoluteB, Signed320.ExtendValue(_length)), Signed576.ExtendValue(absoluteC)).Sign >= 0)
                {
                    int sign = b.Sign;
                    Signed320 numerator = sign < 0 ? WideArithmetic.Negate(c) : c;
                    return Contains(WideArithmetic.MultiplySigned320(_direction.X, numerator),
                        WideArithmetic.MultiplySigned320(_direction.Y, numerator),
                        WideArithmetic.MultiplySigned320(_direction.Z, numerator), Signed576.ExtendValue(absoluteB));
                }
            }

            Signed320 normalSquared = Signed320.NarrowValue(GetAxisSquared(normal));
            Signed576 j = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(_squared, normalSquared),
                WideArithmetic.MultiplySigned320(b, b));
            if (j.IsZero) return false;
            Axis3 w = new(CurvedDifferenceProduct(_squared, normal.X, b, _direction.X),
                CurvedDifferenceProduct(_squared, normal.Y, b, _direction.Y),
                CurvedDifferenceProduct(_squared, normal.Z, b, _direction.Z));
            // The plane residual divided by the cylinder's constant radius or
            // cone's affine radius is monotone away from an axis crossing.
            // Without a finite axis crossing, a nonempty section therefore
            // meets a cap disk (only the base disk for a cone).
            for (int capSign = -1; capSign <= (_kind == FiniteShapeKind.Cone ? -1 : 1); capSign += 2)
            {
                Signed192 length = capSign < 0 ? WideArithmetic.Negate(_length) : _length;
                Signed320 delta = WideArithmetic.SubtractSigned320(c,
                    Signed320.NarrowValue(WideArithmetic.MultiplySigned320(b, Signed320.ExtendValue(length))));
                Signed576 deltaSquared = WideArithmetic.MultiplySigned320(delta, delta);
                if (WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(deltaSquared, _squared),
                        WideArithmetic.MultiplySigned576ToSigned832(_radiusSquared, j)).Sign > 0)
                    continue;
                return Contains(FootCoordinate(_direction.X, length, j, delta, w.X),
                    FootCoordinate(_direction.Y, length, j, delta, w.Y),
                    FootCoordinate(_direction.Z, length, j, delta, w.Z), j);
            }
            return false;
        }

        internal bool IntersectsEdge(Axis3 start, Axis3 end)
        {
            Axis3 edge = new(WideArithmetic.SubtractSigned320(end.X, start.X),
                WideArithmetic.SubtractSigned320(end.Y, start.Y), WideArithmetic.SubtractSigned320(end.Z, start.Z));
            Signed320 axialStart = Signed320.NarrowValue(CurvedDot(_direction, start));
            Signed320 axialDelta = Signed320.NarrowValue(CurvedDot(_direction, edge));
            if (!CapsuleFiniteSectionFrame.TryGetCenteredAxisInterval(axialStart, axialDelta, _cap,
                    out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud))
                return false;
            Axis3 radialStart = Cross(_direction, start), radialEdge = Cross(_direction, edge);
            Signed576 a = CurvedDot(radialEdge, radialEdge), b = CurvedDot(radialStart, radialEdge),
                c = CurvedDot(radialStart, radialStart);
            if (_kind == FiniteShapeKind.Cylinder)
            {
                c = WideArithmetic.SubtractSigned576(c, Signed576.NarrowValue(
                    WideArithmetic.MultiplySigned576ToSigned832(_radiusSquared, _squared)));
                return CapsuleFiniteSectionFrame.IntersectsQuadratic(a, b, c, ln, ld, un, ud);
            }

            // Exact normalized quaternion/yaw bounds give |A| < 2^98 and
            // |P| < 2^164 in the shared 2*D*Q raw-coordinate denominator.
            // Cross products fit 320 bits, radial coefficients 576 bits;
            // cone coefficients need fewer than 870 bits and use 16 limbs.
            const int words = 16;
            Span<ulong> storage = stackalloc ulong[10 * words];
            Span<int> signs = stackalloc int[10];
            ContactQuadratic qa = ContactQuadratic.At(storage, signs, 0, words),
                qb = ContactQuadratic.At(storage, signs, 1, words), qc = ContactQuadratic.At(storage, signs, 2, words),
                work = ContactQuadratic.At(storage, signs, 3, words), term = ContactQuadratic.At(storage, signs, 4, words);
            Signed576 scale = WideArithmetic.MultiplySigned320(
                WideArithmetic.MultiplySigned192(_length, _length), _squared);
            scale = WideArithmetic.AddSigned576(scale, scale); scale = WideArithmetic.AddSigned576(scale, scale);
            Signed320 axial = WideArithmetic.SubtractSigned320(_cap, axialStart);
            Signed320 velocity = WideArithmetic.Negate(axialDelta);
            SetConeCoefficient(a, velocity, velocity, scale, _radiusSquared, qa, work, term);
            SetConeCoefficient(b, axial, velocity, scale, _radiusSquared, qb, work, term);
            SetConeCoefficient(c, axial, axial, scale, _radiusSquared, qc, work, term);
            return CapsuleFiniteSectionFrame.IntersectsQuadratic(qa, qb, qc, ln, ld, un, ud);
        }
    }

    private static Signed576 CurvedDot(Axis3 first, Axis3 second) => WideArithmetic.AddSigned576(
        WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(first.X, second.X),
            WideArithmetic.MultiplySigned320(first.Y, second.Y)), WideArithmetic.MultiplySigned320(first.Z, second.Z));

    private static Signed576 Relative(Signed576 coordinate, Signed320 origin, Signed576 denominator) =>
        WideArithmetic.SubtractSigned576(coordinate, Signed576.NarrowValue(
            WideArithmetic.MultiplySigned576ToSigned832(denominator, origin)));

    private static Signed320 CurvedDifferenceProduct(Signed320 a, Signed320 b, Signed320 c, Signed320 d) =>
        Signed320.NarrowValue(WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(a, b),
            WideArithmetic.MultiplySigned320(c, d)));

    private static Signed576 FootCoordinate(Signed320 axis, Signed192 length, Signed576 j, Signed320 delta, Signed320 w) =>
        WideArithmetic.AddSigned576(Signed576.NarrowValue(WideArithmetic.MultiplySigned576ToSigned832(j,
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(axis, Signed320.ExtendValue(length))))),
            WideArithmetic.MultiplySigned320(delta, w));

    private static void SetConeCoefficient(Signed576 radial, Signed320 first, Signed320 second,
        Signed576 scale, Signed576 radiusSquared, ContactQuadratic result, ContactQuadratic work, ContactQuadratic term)
    {
        result.Set(radial); ContactQuadratic.Scale(result, scale, work);
        result.Set(WideArithmetic.MultiplySigned320(first, second)); ContactQuadratic.Scale(result, radiusSquared, term);
        work.CopyTo(result); result.Add(term, -1);
    }
}
