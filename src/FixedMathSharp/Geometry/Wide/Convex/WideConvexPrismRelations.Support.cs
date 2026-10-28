//=======================================================================
// WideConvexPrismRelations.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>
/// Support types and helpers for convex prism relations, including finite shape kinds,
/// radial classifications, and high-precision axis representations used in narrow-phase math.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static readonly Signed192 Scale = Signed192.One;

    #region Nested Types

    private enum FiniteShapeKind
    {
        Cylinder,
        Cone,
    }

    internal enum RadialKind
    {
        None,
        Capsule,
    }

    internal readonly struct Axis3
    {
        internal readonly Signed320 X;
        internal readonly Signed320 Y;
        internal readonly Signed320 Z;

        internal Axis3(Signed320 x, Signed320 y, Signed320 z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        internal bool IsZero => X.IsZero && Y.IsZero && Z.IsZero;
    }

    private readonly struct RigidAxis3
    {
        internal readonly Signed192 X;
        internal readonly Signed192 Y;
        internal readonly Signed192 Z;
        internal readonly Signed192 RotationDenominator;

        internal RigidAxis3(
            Signed192 x,
            Signed192 y,
            Signed192 z,
            Signed192 rotationDenominator)
        {
            X = x;
            Y = y;
            Z = z;
            RotationDenominator = rotationDenominator;
        }

        internal Axis3 ToWide() =>
            new(
                Signed320.ExtendValue(X),
                Signed320.ExtendValue(Y),
                Signed320.ExtendValue(Z));
    }

    private readonly struct PenetrationCandidate
    {
        internal readonly Axis3 Axis;
        internal readonly bool Negate;
        internal readonly Vector3d PrismOffset;
        internal readonly ProjectionDepth ExactDepth;

        internal PenetrationCandidate(
            Axis3 axis,
            bool negate,
            Vector3d prismOffset,
            ProjectionDepth exactDepth)
        {
            Axis = axis;
            Negate = negate;
            PrismOffset = prismOffset;
            ExactDepth = exactDepth;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    internal readonly struct ProjectionDepth
    {
        internal readonly Signed704 Rational;
        internal readonly Signed192 Common;
        internal readonly Fixed64 Radius;
        internal readonly RadialKind RadialKind;
        internal readonly Signed576 AxisSquared;
        internal readonly Signed320 ShapeAxisSquared;

        internal ProjectionDepth(
            Signed704 rational,
            Signed192 common,
            Fixed64 radius,
            RadialKind radialKind,
            Signed576 axisSquared,
            Signed320 shapeAxisSquared)
        {
            Rational = rational;
            Common = common;
            Radius = radius;
            RadialKind = radialKind;
            AxisSquared = axisSquared;
            ShapeAxisSquared = shapeAxisSquared;
        }
    }

    internal readonly struct CylinderCylinderPenetration
    {
        internal readonly Axis3 Axis;
        internal readonly bool Negate;
        internal readonly CylinderPairDepth Depth;

        internal CylinderCylinderPenetration(
            Axis3 axis,
            bool negate,
            CylinderPairDepth depth)
        {
            Axis = axis;
            Negate = negate;
            Depth = depth;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    /// <summary>
    /// Parallel cap/perpendicular depth R / (C sqrt(A)) + radiusRaw.
    /// Authored axes need not have an exact unit squared length.
    /// </summary>
    internal readonly struct CylinderPairDepth
    {
        internal readonly Signed704 Rational;
        internal readonly Signed192 Common;
        internal readonly Signed576 AxisSquared;
        internal readonly ulong RadiusRaw;

        internal CylinderPairDepth(
            Signed704 rational,
            Signed192 common,
            Signed576 axisSquared,
            ulong radiusRaw)
        {
            Rational = rational;
            Common = common;
            AxisSquared = axisSquared;
            RadiusRaw = radiusRaw;
        }

        internal Signed320 RadialCoefficient => WideArithmetic.MultiplySigned192(
            Common, new Signed192(0UL, 0UL, RadiusRaw));
    }

    #endregion

    private static bool TryMaterializeContact(
        in PenetrationCandidate candidate,
        RigidAxis3 shapeAxis,
        Vector3d center,
        FixedQuaternion shapeRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d prismOrigin,
        Fixed64 prismYaw,
        FixedQuaternion prismOrientation,
        ReadOnlySpan<Vector2d> prismOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        Axis3 orientedAxis = candidate.Negate
            ? Negate(candidate.Axis)
            : candidate.Axis;
        Vector3d normal = WideNormalization.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        int axialSign = GetAxisProjection(
            orientedAxis,
            shapeAxis).Sign;
        FixedPointAnchor shapeAnchor = WideGeometry.GetCenteredCapsuleSupportAnchor(
            center,
            shapeRotation,
            localAxisDirection,
            axisLength,
            Fixed64.Zero,
            normal,
            axialSign);
        if (!TryGetMatchedPrismAnchor(candidate, orientedAxis, shapeAxis,
                center, shapeRotation, localAxisDirection, axisLength, normal, axialSign,
                prismOrigin, prismYaw, prismOrientation, prismOffsets, prismHalfThickness,
                ref shapeAnchor, out FixedPointAnchor prismAnchor))
            return TryGetPrismEdgeContact(shapeAxis, center, shapeRotation, axisLength,
                radius, prismOrigin, prismYaw, prismOrientation, prismOffsets, prismHalfThickness, out contact);
        shapeAnchor = new FixedPointAnchor(center, shapeRotation, shapeAnchor.LocalPoint,
            GetPrismCapsuleRadialOffset(orientedAxis, shapeRotation, radius), shapeAnchor.ExactLocalTerm);
        Fixed64 depth = GetRoundedDepth(candidate.ExactDepth, out bool depthIsClamped);
        contact = new FixedContactAnchors(shapeAnchor, prismAnchor, normal, depth, depthIsClamped);
        return true;
    }

    private static bool TryGetMatchedPrismAnchor(in PenetrationCandidate candidate, Axis3 normal,
        RigidAxis3 shapeAxis, Vector3d center, FixedQuaternion rotation, Vector3d localAxis,
        Fixed64 length, Vector3d direction, int axialSign, Vector3d origin, Fixed64 yaw,
        FixedQuaternion orientation, ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness,
        ref FixedPointAnchor shapeAnchor, out FixedPointAnchor prismAnchor,
        PrismPointAdmission? admission = null, bool materialize = true,
        IPrismCapsuleContactDomain? capsuleDomain = null)
    {
        prismAnchor = default;
        if (capsuleDomain != null && !capsuleDomain.IncludeRecovery)
        {
            // A physical supporting plane must intersect the capsule's complete
            // projected width. Subtracting its exit depth leaves one radial term,
            // which the existing exact projection-sign owner can classify.
            Signed704 axial = GetMagnitude(WideArithmetic.MultiplySigned576ToSigned704(
                GetAxisProjection(normal, shapeAxis), Signed320.ExtendValue(Signed192.Raw(length))));
            var depth = candidate.ExactDepth;
            var remaining = new ProjectionDepth(WideArithmetic.SubtractSigned704(
                WideArithmetic.AddSigned704(axial, axial), depth.Rational), depth.Common,
                depth.Radius, depth.RadialKind, depth.AxisSquared, depth.ShapeAxisSquared);
            if (!IsProjectionNonNegative(remaining)) return false;
        }
        Span<Signed832> projected = stackalloc Signed832[6];
        if (!TryGetPrismSupportInterval(candidate, normal, shapeAxis, center, length, axialSign, origin, yaw,
                offsets, halfThickness, projected, out Signed832 pointD, out var solidClip)) return false;
        Signed832 tN = Signed832.ExtendValue(Signed192.Signed(1)), tD = Signed832.ExtendValue(Signed192.Signed(2));
        if (capsuleDomain != null)
        {
            // A fixed tip projects to one point. A free core is parallel to
            // the plane, so radial displacement is constant over the whole family.
            if (!capsuleDomain.IncludeRecovery && !IsProjectedPrismPointWithinCapsule(projected, pointD,
                    tN, tD, center - origin, rotation, length, candidate.ExactDepth.Radius)) return false;
            var wideNormal = new WideDirection3d(normal.X, normal.Y, normal.Z);
            ReadOnlySpan<RationalSegmentInterval> intervals = capsuleDomain.GetSupportIntervals(wideNormal, projected, pointD, solidClip);
            foreach (RationalSegmentInterval retained in intervals)
                if (TryGetPrismDomainParameter(normal, candidate.ExactDepth, projected, pointD, yaw,
                        retained, admission!, out tN, out tD)) return true;
            return false;
        }
        if (admission != null && !AdmitPrismSpherePoint(normal, candidate.ExactDepth, projected, pointD, yaw, admission)) return false;
        if (!materialize) return true;
        if (WideArithmetic.SubtractSigned832(WideArithmetic.AddSigned832(
                solidClip.WideLowerNumerator, solidClip.WideLowerNumerator), solidClip.WideLowerDenominator).Sign > 0)
        { tN = solidClip.WideLowerNumerator; tD = solidClip.WideLowerDenominator; }
        else if (WideArithmetic.SubtractSigned832(WideArithmetic.AddSigned832(
                solidClip.WideUpperNumerator, solidClip.WideUpperNumerator), solidClip.WideUpperDenominator).Sign < 0)
        { tN = solidClip.WideUpperNumerator; tD = solidClip.WideUpperDenominator; }
        if (axialSign == 0 && length > Fixed64.Zero)
            shapeAnchor = GetPrismCoreAnchor(shapeAnchor, localAxis, length, direction, tN, tD);
        prismAnchor = MaterializePrismProjection(projected, pointD, tN, tD, origin, orientation);
        return true;
    }

    private static bool TryGetPrismDomainParameter(Axis3 normal, in ProjectionDepth depth,
        ReadOnlySpan<Signed832> projected, Signed832 pointD, Fixed64 yaw,
        in RationalSegmentInterval retained, PrismPointAdmission admission, out Signed832 tN, out Signed832 tD)
    {
        tN = Signed832.ExtendValue(Signed192.Signed(1)); tD = Signed832.ExtendValue(Signed192.Signed(2));
        if (retained.IsClipped)
        {
            if (RationalSegmentInterval.CompareParameters(tN, tD, retained.WideLowerNumerator, retained.WideLowerDenominator) < 0)
            { tN = retained.WideLowerNumerator; tD = retained.WideLowerDenominator; }
            else if (RationalSegmentInterval.CompareParameters(tN, tD, retained.WideUpperNumerator, retained.WideUpperDenominator) > 0)
            { tN = retained.WideUpperNumerator; tD = retained.WideUpperDenominator; }
        }
        if (AdmitPrismSpherePoint(normal, depth, projected, pointD, yaw, admission, tN, tD)) return true;
        GetPrismInteriorParameter(retained, out tN, out tD);
        return AdmitPrismSpherePoint(normal, depth, projected, pointD, yaw, admission, tN, tD);
    }

    private static FixedPointAnchor GetPrismCoreAnchor(in FixedPointAnchor anchor, Vector3d localAxis,
        Fixed64 length, Vector3d direction, Signed832 tN, Signed832 tD)
    {
        Signed832 axialN = WideArithmetic.MultiplySigned832(WideArithmetic.SubtractSigned832(
            WideArithmetic.AddSigned832(tN, tN), tD), Signed192.Raw(length));
        Signed832 axialD = WideArithmetic.AddSigned832(tD, tD);
        Signed832 magnitude = axialN.Sign < 0 ? WideArithmetic.SubtractSigned832(default, axialN) : axialN;
        bool endpoint = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(axialD, Signed192.Raw(length)),
            WideArithmetic.AddSigned832(magnitude, magnitude)).IsZero;
        return endpoint ? WideGeometry.GetCenteredCapsuleSupportAnchor(anchor.Origin, anchor.Rotation, localAxis,
            length, Fixed64.Zero, direction, axialN.Sign)
            : new FixedPointAnchor(anchor.Origin, anchor.Rotation, WideTriangleRelations.MaterializeCorePoint(localAxis, axialN, axialD),
                anchor.LocalDisplacement, anchor.ExactLocalTerm);
    }

    private static bool TryGetPrismSupportInterval(in PenetrationCandidate candidate, Axis3 normal,
        RigidAxis3 shapeAxis, Vector3d center, Fixed64 length, int axialSign, Vector3d origin, Fixed64 yaw,
        ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness, Span<Signed832> projected,
        out Signed832 pointD, out RationalSegmentInterval solidClip)
    {
        solidClip = default;
        // Ordinary capsule axes fit 201 bits: projection N/D 571/503,
        // half-planes 704 and free-core parameters 803. Bounded zero-core sphere
        // axes fit 244 bits and keep fixed-core projections within Signed832.
        var frame = new WideConvex2dRelations.RotationFrame2d(yaw);
        GetPrismSupportProjection(candidate.PrismOffset, normal, shapeAxis, center, length, axialSign, origin,
            frame, projected, out pointD, out Signed192 vx, out Signed192 vz);
        // Projection removes the radial term. Clip the finite core against the
        // actual native-yaw prism, never its rounded quaternion approximation.
        Signed832 lowerN = default, lowerD = Signed832.ExtendValue(Signed192.Signed(1));
        Signed832 upperN = lowerD, upperD = lowerD;
        int winding = WideConvex2dRelations.GetPlanarSweepWinding(offsets);
        Signed192 minX = vx, maxX = vx, minZ = vz, maxZ = vz;
        for (int index = 0; index < offsets.Length; index++)
        {
            WideConvex2dRelations.GetRotatedOffset(frame, offsets[index], out Signed192 x, out Signed192 z);
            WideConvex2dRelations.GetRotatedOffset(frame, offsets[index + 1 == offsets.Length ? 0 : index + 1],
                out Signed192 nextX, out Signed192 nextZ);
            if (WideArithmetic.SubtractSigned192(x, minX).Sign < 0) minX = x;
            if (WideArithmetic.SubtractSigned192(x, maxX).Sign > 0) maxX = x;
            if (WideArithmetic.SubtractSigned192(z, minZ).Sign < 0) minZ = z;
            if (WideArithmetic.SubtractSigned192(z, maxZ).Sign > 0) maxZ = z;
            Signed192 dx = WideArithmetic.SubtractSigned192(nextX, x), dz = WideArithmetic.SubtractSigned192(nextZ, z);
            Signed832 first = GetPrismEdgeConstraint(projected[..3], pointD, x, z, dx, dz);
            Signed832 second = GetPrismEdgeConstraint(projected[3..], pointD, x, z, dx, dz);
            if (winding < 0) { first = WideArithmetic.SubtractSigned832(default, first); second = WideArithmetic.SubtractSigned832(default, second); }
            if (!WideTriangleRelations.ClipScaledHalfPlane(first, second, ref lowerN, ref lowerD, ref upperN, ref upperD))
                return false;
        }
        Signed192 height = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(Signed192.Raw(halfThickness), Scale));
        ReadOnlySpan<Signed192> minimum = stackalloc Signed192[3] { minX, WideArithmetic.Negate(height), minZ };
        ReadOnlySpan<Signed192> maximum = stackalloc Signed192[3] { maxX, height, maxZ };
        for (int axis = 0; axis < 3; axis++)
        {
            // Real polygon edges bound X/Z already; collapsed footprints also
            // need their finite endpoint bounds along the surviving line.
            if (winding != 0 && axis != 1) continue;
            Signed832 first = WideArithmetic.MultiplySigned832(projected[axis], Scale);
            Signed832 second = WideArithmetic.MultiplySigned832(projected[axis + 3], Scale);
            Signed832 min = WideArithmetic.MultiplySigned832(pointD, minimum[axis]);
            Signed832 max = WideArithmetic.MultiplySigned832(pointD, maximum[axis]);
            if (!WideTriangleRelations.ClipScaledHalfPlane(WideArithmetic.SubtractSigned832(first, min),
                    WideArithmetic.SubtractSigned832(second, min), ref lowerN, ref lowerD, ref upperN, ref upperD)
                || !WideTriangleRelations.ClipScaledHalfPlane(WideArithmetic.SubtractSigned832(max, first),
                    WideArithmetic.SubtractSigned832(max, second), ref lowerN, ref lowerD, ref upperN, ref upperD))
                return false;
        }
        solidClip = new RationalSegmentInterval(lowerN, lowerD, upperN, upperD);
        return true;
    }

    private static void GetPrismSupportProjection(Vector3d vertexOffset, Axis3 normal, RigidAxis3 shapeAxis,
        Vector3d center, Fixed64 length, int axialSign, Vector3d origin, WideConvex2dRelations.RotationFrame2d frame,
        Span<Signed832> projected, out Signed832 pointD, out Signed192 vx, out Signed192 vz)
    {
        Signed192 twiceRotation = WideArithmetic.AddSigned192(shapeAxis.RotationDenominator, shapeAxis.RotationDenominator);
        Signed192 coreD = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(twiceRotation, Scale));
        ReadOnlySpan<Signed320> n = stackalloc Signed320[3] { normal.X, normal.Y, normal.Z };
        ReadOnlySpan<Signed192> a = stackalloc Signed192[3] { shapeAxis.X, shapeAxis.Y, shapeAxis.Z };
        Signed576 metric = WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(n[0], n[0]), WideArithmetic.MultiplySigned320(n[1], n[1])),
            WideArithmetic.MultiplySigned320(n[2], n[2]));
        WideConvex2dRelations.GetRotatedOffset(frame, new Vector2d(vertexOffset.X, vertexOffset.Z),
            out vx, out vz);
        ReadOnlySpan<Signed192> vertex = stackalloc Signed192[3]
            { vx, Signed192.NarrowProven(WideArithmetic.MultiplySigned192(Signed192.Raw(vertexOffset.Y), Scale)), vz };
        Span<Signed576> core = stackalloc Signed576[6], support = stackalloc Signed576[3];
        bool freeCore = axialSign == 0 && length > Fixed64.Zero;
        for (int axis = 0; axis < 3; axis++)
        {
            Signed320 midpoint = WideArithmetic.MultiplySigned192(WideArithmetic.Difference(center[axis], origin[axis]), coreD);
            Signed320 half = WideArithmetic.MultiplySigned192(a[axis], Signed192.Raw(length));
            core[axis] = Signed576.ExtendValue(WideArithmetic.AddSigned320(midpoint,
                freeCore ? WideArithmetic.Negate(half) : axialSign < 0 ? WideArithmetic.Negate(half) : axialSign > 0 ? half : default));
            core[axis + 3] = freeCore ? Signed576.ExtendValue(WideArithmetic.AddSigned320(midpoint, half)) : core[axis];
            support[axis] = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(vertex[axis], twiceRotation));
        }
        Signed576 plane = DotPrismSupport(support, n);
        for (int end = 0; end < 2; end++)
        {
            Signed576 distance = WideArithmetic.SubtractSigned576(plane, DotPrismSupport(core.Slice(3 * end, 3), n));
            for (int axis = 0; axis < 3; axis++)
                projected[3 * end + axis] = WideArithmetic.AddSigned832(
                    WideArithmetic.MultiplySigned576ToSigned832(metric, core[3 * end + axis]),
                    WideArithmetic.MultiplySigned576ToSigned832(distance, n[axis]));
        }
        pointD = WideArithmetic.MultiplySigned576ToSigned832(metric, Signed320.ExtendValue(coreD));
    }

    internal static void GetProjectedCapsuleSupport(Vector3d center, FixedQuaternion rotation, Fixed64 length,
        Vector3d origin, Fixed64 yaw, Vector3d pointOnPlane, in WideDirection3d normal,
        Span<Signed832> projected, out Signed832 denominator)
    {
        WideOrientedBox.GetRotatedLocalAxisNumerators(rotation, Vector3d.Up, out var x, out var y, out var z, out var d);
        var shapeAxis = new RigidAxis3(x, y, z, d);
        var direction = new Axis3(Signed320.NarrowValue(normal.X), Signed320.NarrowValue(normal.Y), Signed320.NarrowValue(normal.Z));
        GetPrismSupportProjection(pointOnPlane, direction, shapeAxis, center, length,
            GetAxisProjection(direction, shapeAxis).Sign, origin, new(yaw), projected, out denominator, out _, out _);
    }

    private static Vector3d GetPrismCapsuleRadialOffset(Axis3 normal, FixedQuaternion rotation, Fixed64 radius)
    {
        WideRigidProjection.GetLocalAxisProjections(new WideAxis3(normal.X, normal.Y, normal.Z),
            new WideRationalBasis3d(rotation), out Signed576 x, out Signed576 y, out Signed576 z);
        // Scale the exact local direction before rounding: a rounded unit
        // vector's error would otherwise grow with full-domain radii.
        const int words = 9;
        Span<ulong> values = stackalloc ulong[7 * words]; Span<int> signs = stackalloc int[7];
        values.Clear(); signs.Clear();
        WideArithmetic.GetMagnitude(x, values[..words]); signs[0] = x.Sign;
        WideArithmetic.GetMagnitude(y, values.Slice(words, words)); signs[1] = y.Sign;
        WideArithmetic.GetMagnitude(z, values.Slice(2 * words, words)); signs[2] = z.Sign;
        return GetConvexContactCandidateScaledNormal(new ConvexContactCandidate(values, signs, 1, words), radius);
    }

    private static Signed576 DotPrismSupport(ReadOnlySpan<Signed576> point, ReadOnlySpan<Signed320> axis) =>
        Signed576.NarrowValue(WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(point[0], axis[0]),
            WideArithmetic.MultiplySigned576ToSigned832(point[1], axis[1])),
            WideArithmetic.MultiplySigned576ToSigned832(point[2], axis[2])));

    private static Signed832 GetPrismEdgeConstraint(ReadOnlySpan<Signed832> point, Signed832 denominator,
        Signed192 x, Signed192 z, Signed192 dx, Signed192 dz) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(WideArithmetic.SubtractSigned832(
                WideArithmetic.MultiplySigned832(point[2], Scale), WideArithmetic.MultiplySigned832(denominator, z)), dx),
            WideArithmetic.MultiplySigned832(WideArithmetic.SubtractSigned832(
                WideArithmetic.MultiplySigned832(point[0], Scale), WideArithmetic.MultiplySigned832(denominator, x)), dz));

    private static FixedPointAnchor MaterializePrismProjection(ReadOnlySpan<Signed832> projected, Signed832 pointD,
        Signed832 tN, Signed832 tD, Vector3d origin, FixedQuaternion rotation)
    {
        const int words = 32;
        Span<ulong> values = stackalloc ulong[22 * words]; Span<int> signs = stackalloc int[22];
        ContactQuadratic first = At(values, signs, 3, words), second = At(values, signs, 4, words),
            remaining = At(values, signs, 5, words), parameter = At(values, signs, 6, words),
            denominator = At(values, signs, 7, words), localD = At(values, signs, 8, words),
            numerator = At(values, signs, 9, words), term = At(values, signs, 10, words);
        remaining.Set(WideArithmetic.SubtractSigned832(tD, tN)); parameter.Set(tN);
        for (int axis = 0; axis < 3; axis++)
        {
            first.Set(projected[axis]); second.Set(projected[axis + 3]);
            ContactQuadratic coordinate = At(values, signs, axis, words);
            Multiply(first, remaining, ReadOnlySpan<ulong>.Empty, coordinate);
            Multiply(second, parameter, ReadOnlySpan<ulong>.Empty, term); coordinate.Add(term);
        }
        first.Set(pointD); second.Set(tD); Multiply(first, second, ReadOnlySpan<ulong>.Empty, denominator);
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
                ContactQuadratic.Scale(At(values, signs, component, words), Signed576.ExtendValue(coefficients[3 * axis + component]), term);
                numerator.Add(term);
            }
            SplitRound(numerator, localD, ReadOnlySpan<ulong>.Empty, out Fixed64 a, out Fixed64 b);
            point[axis] = a; displacement[axis] = b;
        }
        return new FixedPointAnchor(origin, rotation, point, displacement);
    }

    private static Axis3 FromDirection(Vector3d direction) =>
        new(
            Signed320.ExtendValue(Signed192.Raw(direction.X)),
            Signed320.ExtendValue(Signed192.Raw(direction.Y)),
            Signed320.ExtendValue(Signed192.Raw(direction.Z)));

    private static int Compare(Signed576 left, Signed576 right) =>
        WideArithmetic.SubtractSigned576(left, right).Sign;

    private static Axis3 Negate(Axis3 axis) =>
        new(
            WideArithmetic.Negate(axis.X),
            WideArithmetic.Negate(axis.Y),
            WideArithmetic.Negate(axis.Z));
}
