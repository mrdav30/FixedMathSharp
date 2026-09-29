//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact edge/face minimum-axial reduction in the triangle's authored frame.</summary>
internal static class WideTriangleConeIntersection
{
    private static readonly Signed192 Scale = Signed192.Raw(Fixed64.One);
    private static readonly Signed192 MaximumParameter = Signed192.Raw(Fixed64.MaxValue);

    internal static bool TryGetMinimumAxialPoint(FixedTriangle triangle, Vector3d origin,
        FixedQuaternion rotation, Vector3d apex, Vector3d axis, Fixed64 height, Fixed64 radius,
        out Vector3d point)
    {
        var frame = new QueryFrame(origin, rotation, apex, axis);
        bool found = false;
        point = default;
        Fixed64 bestAxial = Fixed64.MaxValue;
        for (int index = 0; index < 3; index++)
        {
            FixedSegment edge = triangle.GetEdge(index);
            if (!frame.TryGetEdgePoint(edge, height, radius, out Vector3d candidate))
                continue;
            // Equal candidates retain authored AB, BC, CA order after the same
            // local-point rounding and cone-parameter projection in either frame.
            Fixed64 axial = frame.GetAxialParameter(candidate, height);
            if (found && axial >= bestAxial)
                continue;
            found = true;
            point = candidate;
            bestAxial = axial;
        }
        triangle.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out Signed320 nSquared);
        if (!WideGeometry.IsQ128MagnitudeAtMostEpsilon(nSquared)
            && TryGetFacePoint(triangle, frame, Axis(nx, ny, nz), nSquared, height, radius, found,
                out Vector3d facePoint, out Fixed64 faceAxial)
            && (!found || faceAxial < bestAxial))
        {
            found = true;
            point = facePoint;
        }
        return found;
    }

    private readonly struct QueryFrame
    {
        internal readonly WideAxis3 Apex, Axis;
        internal readonly Signed192 Denominator, AxisSquared;
        internal readonly bool Compact;
        internal readonly Vector3d LocalApex;
        private readonly WideRationalBasis3d basis;
        private readonly Vector3d origin, worldApex, worldAxis;
        private readonly bool identity;

        internal QueryFrame(Vector3d origin, FixedQuaternion rotation, Vector3d apex, Vector3d axis)
        {
            this.origin = origin; worldApex = apex; worldAxis = axis;
            identity = rotation == FixedQuaternion.Identity;
            basis = identity ? default : new WideRationalBasis3d(rotation);
            Vector3d localApex = default;
            Compact = identity && Vector3d.TrySubtract(apex, origin, out localApex);
            LocalApex = localApex;
            AxisSquared = Signed192.NarrowProven(Dot(Raw(axis), Raw(axis)));
            if (identity)
            {
                Denominator = Signed192.Signed(1);
                Apex = Difference(apex, origin);
                Axis = Raw(axis);
                return;
            }
            WideOrientedBox.GetRelativeLocalPointNumerators(apex, origin, basis,
                out Signed192 ax, out Signed192 ay, out Signed192 az);
            WideOrientedBox.GetRelativeLocalPointNumerators(axis, Vector3d.Zero, basis,
                out Signed192 dx, out Signed192 dy, out Signed192 dz);
            Span<Signed320> values = stackalloc Signed320[7]
            {
                Signed320.ExtendValue(basis.Denominator), Signed320.ExtendValue(ax),
                Signed320.ExtendValue(ay), Signed320.ExtendValue(az), Signed320.ExtendValue(dx),
                Signed320.ExtendValue(dy), Signed320.ExtendValue(dz)
            };
            WideArithmetic.ReduceCommonScale(values);
            Denominator = Signed192.NarrowProven(values[0]);
            Apex = new WideAxis3(values[1], values[2], values[3]);
            Axis = new WideAxis3(values[4], values[5], values[6]);
        }

        internal Fixed64 GetAxialParameter(Vector3d point, Fixed64 height)
        {
            if (Compact)
                return FixedMath.Min(Vector3d.ProjectNonNegativeDifferenceParameter(point, LocalApex, worldAxis), height);

            // Compare the same nearest-even cone parameter as face candidates,
            // not a floored dot: an admitted unit axis need not have exact length 1.
            Signed320 numerator = Dot(Subtract(Multiply(Raw(point), Denominator), Apex), Axis);
            Signed320 denominator = Signed320.NarrowValue(WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(WideArithmetic.MultiplySigned192(AxisSquared, Denominator)), Denominator));
            return FixedMath.Clamp(Fixed64.GetSignedRatio(numerator, denominator), Fixed64.Zero, height);
        }

        internal bool TryGetEdgePoint(FixedSegment edge, Fixed64 height, Fixed64 radius, out Vector3d point)
        {
            // Only an exact translation qualifies for the compact segment path.
            // Rounded rotation or saturating origin subtraction cannot do so.
            if (Compact)
                return edge.TryGetFiniteConeIntersectionMinimumAxialPoint(LocalApex, worldAxis, height, radius, out point);

            WideAxis3 start, delta;
            Signed192 denominator;
            if (identity)
            {
                start = Subtract(Raw(edge.Start), Apex);
                delta = Difference(edge.End, edge.Start);
                denominator = Signed192.Signed(1);
            }
            else
            {
                WideAxis3 offset = Difference(origin, worldApex);
                start = Add(Transform(edge.Start), Multiply(offset, basis.Denominator));
                WideAxis3 localDelta = Difference(edge.End, edge.Start);
                delta = WideRigidProjection.TransformLocalAxis(basis, Narrow(localDelta.X), Narrow(localDelta.Y), Narrow(localDelta.Z));
                denominator = basis.Denominator;
            }
            Signed192 startProjection = Signed192.NarrowProven(Dot(start, Raw(worldAxis)));
            Signed192 velocityProjection = Signed192.NarrowProven(Dot(delta, Raw(worldAxis)));
            Signed320 startAxial = WideArithmetic.MultiplySigned192(startProjection, Scale);
            Signed320 velocity = WideArithmetic.MultiplySigned192(velocityProjection, Scale);
            Signed320 maximum = Signed320.NarrowValue(WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(WideArithmetic.MultiplySigned192(AxisSquared, Signed192.Raw(height))), denominator));
            Signed320 lowerN = default, lowerD = Signed320.ExtendValue(Signed192.Signed(1)), upperN = Signed320.ExtendValue(Signed192.Signed(1)), upperD = Signed320.ExtendValue(Signed192.Signed(1));
            if (velocity.IsZero)
            {
                if (startAxial.Sign < 0 || WideArithmetic.SubtractSigned320(startAxial, maximum).Sign > 0)
                { point = default; return false; }
            }
            else
            {
                Signed320 first = WideArithmetic.Negate(startAxial);
                Signed320 second = WideArithmetic.SubtractSigned320(maximum, startAxial);
                Signed320 divisor = velocity;
                if (velocity.Sign < 0)
                {
                    (first, second) = (WideArithmetic.Negate(second), WideArithmetic.Negate(first));
                    divisor = WideArithmetic.Negate(divisor);
                }
                if (second.Sign < 0 || WideArithmetic.SubtractSigned320(first, divisor).Sign > 0)
                { point = default; return false; }
                if (first.Sign > 0) { lowerN = first; lowerD = divisor; }
                if (WideArithmetic.SubtractSigned320(second, divisor).Sign < 0) { upperN = second; upperD = divisor; }
            }
            Signed576 a = GetEdgeCoefficient(Dot(delta, delta), velocityProjection, velocityProjection, AxisSquared, height, radius);
            Signed576 b = GetEdgeCoefficient(Dot(start, delta), startProjection, velocityProjection, AxisSquared, height, radius);
            Signed576 c = GetEdgeCoefficient(Dot(start, start), startProjection, startProjection, AxisSquared, height, radius);
            if (!WideFiniteConeIntersection.TrySolveBoundedUnitPolynomial(
                    Signed832.ExtendValue(a), Signed832.ExtendValue(b), Signed832.ExtendValue(c),
                    lowerN, lowerD, upperN, upperD, Fixed64.MaxValue, out Fixed64 entry, out Fixed64 exit))
            { point = default; return false; }
            point = edge.GetPointAtDistance(velocity.Sign < 0 ? exit : entry, Fixed64.MaxValue);
            return true;
        }

        private WideAxis3 Transform(Vector3d point) => WideRigidProjection.TransformLocalAxis(basis,
            Signed192.Raw(point.X), Signed192.Raw(point.Y), Signed192.Raw(point.Z));
    }

    private static Signed576 GetEdgeCoefficient(Signed320 squared, Signed192 first, Signed192 second,
        Signed192 q, Fixed64 height, Fixed64 radius)
    {
        // For P(t)=start+t*delta and q=axis.axis, radial distance is
        // (q*P.P-(P.axis)^2)/q. Keep t's axial clipping separate and expand
        // H²*q*(q*P.P-(P.axis)^2)-R²*Q²*(P.axis)^2 <= 0 exactly.
        // P and E have <133 bits, their axial dots <168. After retaining the
        // common rigid denominator, cone coefficients still fit below 530 bits.
        Signed576 axial = WideArithmetic.MultiplySigned320(Signed320.ExtendValue(first), Signed320.ExtendValue(second));
        Signed576 radial = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(squared), q), axial);
        return WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(radial, q, Signed192.Raw(height), Signed192.Raw(height)),
            WideArithmetic.MultiplySigned576(axial, Scale, Scale, Signed192.Raw(radius), Signed192.Raw(radius)));
    }

    private static bool TryGetFacePoint(FixedTriangle triangle, in QueryFrame frame, WideAxis3 normal,
        Signed320 normalSquared, Fixed64 height, Fixed64 radius, bool hasEdge,
        out Vector3d point, out Fixed64 axial)
    {
        point = default; axial = default;
        // The plane is n.(X-apex)=c/D and its axial slope is b/D.
        // p=|n|²*|axisNumerator|²-b² is the squared radial plane normal.
        // At each axial parameter the stationary radial point is the first
        // place that plane's cone section can touch the triangle interior.
        Signed320 b = Dot(normal, frame.Axis);
        Signed320 c = Dot(normal, Subtract(Multiply(Raw(triangle.A), frame.Denominator), frame.Apex));
        Signed320 qScaled = Signed320.NarrowValue(WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(WideArithmetic.MultiplySigned192(frame.AxisSquared, frame.Denominator)), frame.Denominator));
        Signed576 p = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(normalSquared, qScaled),
            WideArithmetic.MultiplySigned320(b, b));
        bool crossesAxis = TryGetAxisPlaneRatio(b, c, height, out Signed320 axisN, out Signed320 axisD);
        if (p.IsZero)
        {
            if (hasEdge || !crossesAxis || !ContainsAxisPoint(triangle, normal, frame, axisN, axisD))
                return false;
            point = new Vector3d(AxisCoordinate(frame.Apex.X, frame.Axis.X, frame.Denominator, axisN, axisD),
                AxisCoordinate(frame.Apex.Y, frame.Axis.Y, frame.Denominator, axisN, axisD),
                AxisCoordinate(frame.Apex.Z, frame.Axis.Z, frame.Denominator, axisN, axisD));
            axial = Ratio(WideArithmetic.MultiplySigned832(Extend(axisN), Scale), Extend(axisD));
            return true;
        }
        // n<130, b<230, c<263, p<460 bits. Cancel the common D² from
        // the polynomial before multiplying: coefficients then need <660 bits.
        Signed832 coefficient = WideArithmetic.SubtractSigned832(
            Product(WideArithmetic.MultiplySigned320(b, b), frame.AxisSquared, Signed192.Raw(height), Signed192.Raw(height)),
            Product(p, Signed192.Raw(radius), Signed192.Raw(radius), Scale, Scale));
        Signed832 projection = WideArithmetic.SubtractSigned832(default,
            Product(WideArithmetic.MultiplySigned320(b, c), frame.AxisSquared, Scale, Signed192.Raw(height)));
        Signed832 constant = Product(WideArithmetic.MultiplySigned320(c, c), frame.AxisSquared, Scale, Scale);
        if (!WideFiniteConeIntersection.TrySolveBoundedUnitPolynomial(coefficient, projection, constant,
                default, Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(1)), Fixed64.MaxValue, out Fixed64 entry, out _))
            return false;
        bool contained;
        if (!hasEdge)
            contained = crossesAxis ? ContainsAxisPoint(triangle, normal, frame, axisN, axisD)
                : ContainsStationaryPoint(triangle, normal, normalSquared, frame, b, c, p, height, Fixed64.MaxValue);
        else
        {
            int sign = WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(coefficient, projection, constant,
                Signed320.ExtendValue(Signed192.Raw(entry)), Signed320.ExtendValue(MaximumParameter));
            Fixed64 lower = sign < 0 ? Fixed64.FromRaw(entry.m_rawValue - 1) : entry;
            Fixed64 upper = sign > 0 ? Fixed64.FromRaw(entry.m_rawValue + 1) : entry;
            contained = ContainsStationaryPoint(triangle, normal, normalSquared, frame, b, c, p, height, lower)
                && (lower == upper || ContainsStationaryPoint(triangle, normal, normalSquared, frame, b, c, p, height, upper));
        }
        if (!contained) return false;
        point = new Vector3d(WitnessCoordinate(frame.Apex.X, frame.Axis.X, normal.X, frame.Denominator, qScaled, b, c, p, height, entry),
            WitnessCoordinate(frame.Apex.Y, frame.Axis.Y, normal.Y, frame.Denominator, qScaled, b, c, p, height, entry),
            WitnessCoordinate(frame.Apex.Z, frame.Axis.Z, normal.Z, frame.Denominator, qScaled, b, c, p, height, entry));
        axial = Fixed64.GetSignedRawRatio(WideArithmetic.MultiplySigned192(Signed192.Raw(height), Signed192.Raw(entry)), MaximumParameter);
        return true;
    }

    private static bool TryGetAxisPlaneRatio(Signed320 b, Signed320 c, Fixed64 height, out Signed320 numerator, out Signed320 denominator)
    {
        denominator = b.Sign < 0 ? WideArithmetic.Negate(b) : b;
        numerator = b.Sign < 0 ? WideArithmetic.Negate(c) : c;
        return !b.IsZero && numerator.Sign >= 0 && WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(numerator), Scale),
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(denominator), Signed192.Raw(height))).Sign <= 0;
    }

    private static bool ContainsAxisPoint(FixedTriangle triangle, WideAxis3 normal, in QueryFrame frame, Signed320 numerator, Signed320 denominator)
    {
        for (int index = 0; index < 3; index++)
        {
            GetEdgeProducts(triangle.GetEdge(index), normal, frame, out Signed576 h, out Signed576 l);
            if (SumProductSign(Extend(denominator), h, Extend(numerator), l) < 0)
                return false;
        }
        return true;
    }

    private static bool ContainsStationaryPoint(FixedTriangle triangle, WideAxis3 normal, Signed320 normalSquared,
        in QueryFrame frame, Signed320 b, Signed320 c, Signed576 p, Fixed64 height, Fixed64 parameter)
    {
        Signed832 originScale = Product(p, MaximumParameter, Scale);
        Signed832 axisScale = WideArithmetic.SubtractSigned832(
            Product(Signed576.ExtendValue(normalSquared), frame.AxisSquared, Signed192.Raw(height), Signed192.Raw(parameter), frame.Denominator, frame.Denominator),
            Product(WideArithmetic.MultiplySigned320(b, c), MaximumParameter, Scale));
        for (int index = 0; index < 3; index++)
        {
            GetEdgeProducts(triangle.GetEdge(index), normal, frame, out Signed576 h, out Signed576 l);
            // These halfspace products can exceed 832 bits; compare magnitudes
            // in existing transient product storage rather than truncating.
            if (SumProductSign(originScale, h, axisScale, l) < 0)
                return false;
        }
        return true;
    }

    private static void GetEdgeProducts(FixedSegment edge, WideAxis3 normal, in QueryFrame frame, out Signed576 h, out Signed576 l)
    {
        WideAxis3 inward = WideAxis3.Cross(normal, Difference(edge.End, edge.Start));
        h = WideAxis3.Dot(inward, Subtract(frame.Apex, Multiply(Raw(edge.Start), frame.Denominator)));
        l = WideAxis3.Dot(inward, frame.Axis);
    }

    private static Fixed64 WitnessCoordinate(Signed320 apex, Signed320 axis, Signed320 normal, Signed192 denominator,
        Signed320 q, Signed320 b, Signed320 c, Signed576 p, Fixed64 height, Fixed64 parameter)
    {
        Signed576 v = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(q, normal), WideArithmetic.MultiplySigned320(b, axis));
        Signed832 w = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(p, axis),
            WideArithmetic.MultiplySigned576ToSigned832(v, b));
        Signed832 first = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(p, apex),
            WideArithmetic.MultiplySigned576ToSigned832(v, c));
        first = WideArithmetic.MultiplySigned832(WideArithmetic.MultiplySigned832(first, MaximumParameter), Scale);
        Signed832 second = WideArithmetic.MultiplySigned832(WideArithmetic.MultiplySigned832(w, Signed192.Raw(height)), Signed192.Raw(parameter));
        return Ratio(WideArithmetic.AddSigned832(first, second), Product(p, denominator, MaximumParameter, Scale));
    }

    private static Fixed64 AxisCoordinate(Signed320 apex, Signed320 axis, Signed192 frameD, Signed320 numerator, Signed320 denominator) =>
        Ratio(Signed832.ExtendValue(WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(apex, denominator),
            WideArithmetic.MultiplySigned320(axis, numerator))),
            WideArithmetic.MultiplySigned832(Extend(denominator), frameD));

    private static int SumProductSign(Signed832 a, Signed576 b, Signed832 c, Signed576 d)
    {
        int first = a.Sign * b.Sign, second = c.Sign * d.Sign;
        if (first == 0) return second;
        if (second == 0 || first == second) return first;
        return first * WideArithmetic.CompareNonNegativeProducts(a, Signed832.ExtendValue(b), c, Signed832.ExtendValue(d));
    }

    private static Fixed64 Ratio(Signed832 numerator, Signed832 denominator)
    {
        bool represented = Fixed64.TryGetSignedRawRatio(numerator, denominator, 0, out Fixed64 value);
        System.Diagnostics.Debug.Assert(represented);
        return value;
    }

    private static Signed832 Product(Signed576 value, Signed192 a, Signed192 b) =>
        WideArithmetic.MultiplySigned832(WideArithmetic.MultiplySigned832(Signed832.ExtendValue(value), a), b);
    private static Signed832 Product(Signed576 value, Signed192 a, Signed192 b, Signed192 c) =>
        WideArithmetic.MultiplySigned832(Product(value, a, b), c);
    private static Signed832 Product(Signed576 value, Signed192 a, Signed192 b, Signed192 c, Signed192 d) =>
        WideArithmetic.MultiplySigned832(Product(value, a, b, c), d);
    private static Signed832 Product(Signed576 value, Signed192 a, Signed192 b, Signed192 c, Signed192 d, Signed192 e) =>
        WideArithmetic.MultiplySigned832(Product(value, a, b, c, d), e);
    private static Signed832 Extend(Signed320 value) => Signed832.ExtendValue(Signed576.ExtendValue(value));
    private static Signed192 Narrow(Signed320 value) => Signed192.NarrowProven(value);
    private static WideAxis3 Axis(Signed192 x, Signed192 y, Signed192 z) => new(Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z));
    private static WideAxis3 Raw(Vector3d value) => Axis(Signed192.Raw(value.X), Signed192.Raw(value.Y), Signed192.Raw(value.Z));
    private static WideAxis3 Difference(Vector3d a, Vector3d b) => Axis(WideArithmetic.Difference(a.X, b.X), WideArithmetic.Difference(a.Y, b.Y), WideArithmetic.Difference(a.Z, b.Z));
    private static WideAxis3 Add(WideAxis3 a, WideAxis3 b) => new(WideArithmetic.AddSigned320(a.X, b.X), WideArithmetic.AddSigned320(a.Y, b.Y), WideArithmetic.AddSigned320(a.Z, b.Z));
    private static WideAxis3 Subtract(WideAxis3 a, WideAxis3 b) => new(WideArithmetic.SubtractSigned320(a.X, b.X), WideArithmetic.SubtractSigned320(a.Y, b.Y), WideArithmetic.SubtractSigned320(a.Z, b.Z));
    private static WideAxis3 Multiply(WideAxis3 value, Signed192 scale) => new(WideArithmetic.MultiplySigned192(Narrow(value.X), scale), WideArithmetic.MultiplySigned192(Narrow(value.Y), scale), WideArithmetic.MultiplySigned192(Narrow(value.Z), scale));
    private static Signed320 Dot(WideAxis3 a, WideAxis3 b) => Signed320.NarrowValue(WideAxis3.Dot(a, b));
}
