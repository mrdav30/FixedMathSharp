//=======================================================================
// WideFiniteAxisIntersection.CenteredAxisTriangle.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Retains exact closest-point feature identity and separation before contact rounding.</summary>
internal readonly struct AxisTriangleContactFeature
{
    private readonly Signed832 _separationX, _separationY, _separationZ;
    private readonly bool _wide;

    internal AxisTriangleContactFeature(int edgeMask, Signed576 separationX,
        Signed576 separationY, Signed576 separationZ, int triangleVertex = -1)
    {
        _wide = false;
        EdgeMask = edgeMask;
        TriangleVertex = triangleVertex;
        _separationX = Signed832.ExtendValue(separationX); _separationY = Signed832.ExtendValue(separationY); _separationZ = Signed832.ExtendValue(separationZ);
    }

    internal AxisTriangleContactFeature(int edgeMask, Signed832 x, Signed832 y, Signed832 z, int triangleVertex = -1)
    {
        _wide = true;
        EdgeMask = edgeMask; TriangleVertex = triangleVertex;
        _separationX = x; _separationY = y; _separationZ = z;
    }

    /// <summary>Maps a retained rigid-local separation into world orientation without scalar rounding.</summary>
    internal AxisTriangleContactFeature GetWorldFeatureFromRigidLocal(FixedQuaternion rotation)
    {
        if (rotation == FixedQuaternion.Identity) return this;
        // This ingress is used only for the legacy local closest chart, whose
        // complete components fit Signed576. One normalized basis adds <65
        // bits; the resulting world direction fits Signed832.
        var basis = new WideRationalBasis3d(rotation);
        Signed576 x = Signed576.NarrowValue(_separationX), y = Signed576.NarrowValue(_separationY), z = Signed576.NarrowValue(_separationZ);
        Signed832 Component(Signed192 a, Signed192 b, Signed192 c) =>
            WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, Signed320.ExtendValue(a)),
                WideArithmetic.MultiplySigned576ToSigned832(y, Signed320.ExtendValue(b))),
                WideArithmetic.MultiplySigned576ToSigned832(z, Signed320.ExtendValue(c)));
        return new AxisTriangleContactFeature(EdgeMask, Component(basis.Xx, basis.Yx, basis.Zx),
            Component(basis.Xy, basis.Yy, basis.Zy), Component(basis.Xz, basis.Yz, basis.Zz), TriangleVertex);
    }
    /// <summary>Zero edge halfspaces: AB=1, BC=2, CA=4; 7 declines degenerate classification.</summary>
    internal int EdgeMask { get; }

    /// <summary>Exact authored vertex ownership; -1 denotes a nonvertex witness.</summary>
    internal int TriangleVertex { get; }

    /// <summary>Whether the retained core and triangle witnesses coincide exactly.</summary>
    internal bool IsZeroSeparation => _separationX.IsZero && _separationY.IsZero && _separationZ.IsZero;

    /// <summary>Gets the exact separation sign along a wide direction in the selected contact chart.</summary>
    internal int CompareSeparationProjection(Signed192 x, Signed192 y, Signed192 z)
    {
        if (_wide) return CompareSeparationProjection(new WideAxis3(Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z)));
        return WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(Signed576.NarrowValue(_separationX), Signed320.ExtendValue(x)),
            WideArithmetic.MultiplySigned576ToSigned832(Signed576.NarrowValue(_separationY), Signed320.ExtendValue(y))),
            WideArithmetic.MultiplySigned576ToSigned832(Signed576.NarrowValue(_separationZ), Signed320.ExtendValue(z))).Sign;
    }

    /// <summary>Gets the same exact sign for a rigidly transformed wide axis.</summary>
    internal int CompareSeparationProjection(in WideAxis3 direction)
    {
        // Full 832-by-320 products plus carries need 1154 signed bits. The
        // existing quadratic owner retains this rational-only dot without
        // narrowing exceptional world-chart plane normals to Signed192.
        int words = _wide ? 19 : 15;
        Span<ulong> values = stackalloc ulong[6 * words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic sum = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic product = ContactQuadratic.At(values, signs, 2, words);
        sum.Clear();
        Span<Signed832> components = stackalloc Signed832[3] { _separationX, _separationY, _separationZ };
        Span<Signed320> axes = stackalloc Signed320[3] { direction.X, direction.Y, direction.Z };
        for (int index = 0; index < 3; index++)
        {
            if (_wide) term.Set(components[index]); else term.Set(Signed576.NarrowValue(components[index]));
            ContactQuadratic.Scale(term, Signed576.ExtendValue(axes[index]), product);
            sum.Add(product);
        }
        return sum.Sign(ReadOnlySpan<ulong>.Empty);
    }

    /// <summary>Gets the exact sign of separation dotted with a ray in the selected contact chart.</summary>
    internal int CompareSeparationProjection(Vector3d first, Vector3d second)
    {
        Signed320 x = Signed320.ExtendValue(WideArithmetic.Difference(second.X, first.X));
        Signed320 y = Signed320.ExtendValue(WideArithmetic.Difference(second.Y, first.Y));
        Signed320 z = Signed320.ExtendValue(WideArithmetic.Difference(second.Z, first.Z));
        if (!_wide)
            return WideArithmetic.AddSigned704(WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(Signed576.NarrowValue(_separationX), x),
                WideArithmetic.MultiplySigned576ToSigned704(Signed576.NarrowValue(_separationY), y)),
                WideArithmetic.MultiplySigned576ToSigned704(Signed576.NarrowValue(_separationZ), z)).Sign;
        return CompareSeparationProjection(new WideAxis3(x, y, z));
    }
}

/// <content>
/// Provides high-precision closest-point and overlap tests between a
/// <see cref="FixedTriangle"/> and a centered, finite-length axis segment
/// (e.g. a capsule's core axis), using wide fixed-point arithmetic.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    #region Nested Types

    private readonly struct AxisTriangleCandidate
    {
        internal readonly Signed576 TriangleX;
        internal readonly Signed576 TriangleY;
        internal readonly Signed576 TriangleZ;
        internal readonly Signed576 AxisX;
        internal readonly Signed576 AxisY;
        internal readonly Signed576 AxisZ;
        internal readonly Signed576 Denominator;
        internal readonly Signed832 SquaredDistanceNumerator;
        internal readonly int EdgeMask;

        internal AxisTriangleCandidate(
            Signed576 triangleX,
            Signed576 triangleY,
            Signed576 triangleZ,
            Signed576 axisX,
            Signed576 axisY,
            Signed576 axisZ,
            Signed576 denominator,
            Signed832 squaredDistanceNumerator,
            int edgeMask)
        {
            TriangleX = triangleX;
            TriangleY = triangleY;
            TriangleZ = triangleZ;
            AxisX = axisX;
            AxisY = axisY;
            AxisZ = axisZ;
            Denominator = denominator;
            SquaredDistanceNumerator = squaredDistanceNumerator;
            EdgeMask = edgeMask;
        }
    }

    #endregion

    internal static bool TryGetClosestPointsToCenteredAxis(
        FixedTriangle triangle,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        out Vector3d pointOnTriangle,
        out Vector3d pointOnAxis)
    {
        GetClosestCenteredAxisTriangleCandidate(
            triangle,
            center,
            axisDirection,
            axisLength,
            out AxisTriangleCandidate candidate);
        return TryMaterialize(
            candidate,
            out pointOnTriangle,
            out pointOnAxis);
    }

    internal static bool DoesCenteredCapsuleTriangleOverlap(
        FixedTriangle triangle,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        GetClosestCenteredAxisTriangleCandidate(
            triangle,
            center,
            axisDirection,
            axisLength,
            out AxisTriangleCandidate candidate);
        Signed576 radiusNumerator = WideArithmetic.MultiplySigned576(
            candidate.Denominator,
            Signed192.Signed(radius.m_rawValue));
        return WideArithmetic.SubtractSigned832(
            candidate.SquaredDistanceNumerator,
            WideArithmetic.MultiplySigned576ToSigned832(
                radiusNumerator,
                radiusNumerator)).Sign <= 0;
    }

    internal static bool TryGetCenteredCapsuleTriangleContact(
        FixedTriangle triangle,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackNormal,
        out Vector3d pointOnTriangle,
        out Vector3d pointOnCapsule,
        out Vector3d normal,
        out Fixed64 depth)
    {
        GetClosestCenteredAxisTriangleCandidate(
            triangle,
            center,
            axisDirection,
            axisLength,
            out AxisTriangleCandidate candidate);
        Signed576 radiusNumerator = WideArithmetic.MultiplySigned576(
            candidate.Denominator,
            Signed192.Signed(radius.m_rawValue));
        if (WideArithmetic.SubtractSigned832(
                candidate.SquaredDistanceNumerator,
                WideArithmetic.MultiplySigned576ToSigned832(
                    radiusNumerator,
                    radiusNumerator)).Sign > 0
            || !TryMaterialize(candidate, out pointOnTriangle, out Vector3d pointOnAxis))
        {
            pointOnTriangle = default;
            pointOnCapsule = default;
            normal = default;
            depth = default;
            return false;
        }

        normal = WideNormalization.GetNormalized(
            WideArithmetic.SubtractSigned576(candidate.AxisX, candidate.TriangleX),
            WideArithmetic.SubtractSigned576(candidate.AxisY, candidate.TriangleY),
            WideArithmetic.SubtractSigned576(candidate.AxisZ, candidate.TriangleZ));
        if (normal.IsZero)
            normal = fallbackNormal;
        if (candidate.SquaredDistanceNumerator.IsZero && axisLength != Fixed64.Zero)
        {
            Span<ulong> values = stackalloc ulong[4 * CapsuleTriangleRayWords];
            Span<int> signs = stackalloc int[4];
            Span<ulong> root = stackalloc ulong[CapsuleTriangleRayRootWords];
            ContactQuadratic n = RayField(values, signs, 0), d = RayField(values, signs, 1);
            GetCoincidentCapsuleRayExit(candidate, center, axisDirection, axisLength, radius,
                -normal, n, d, root, out _);
            GetCapsuleRayDepth(n, d, root, normal, out depth, out bool clamped);
            if (!clamped && TryMaterializeCapsuleRayPoint(candidate, -normal, n, d, root, out pointOnCapsule))
                return true;
            pointOnTriangle = pointOnCapsule = normal = default;
            depth = default;
            return false;
        }
        if (!TryOffsetPoint(pointOnAxis, -normal, radius, out pointOnCapsule))
        {
            pointOnTriangle = default;
            pointOnCapsule = default;
            normal = default;
            depth = default;
            return false;
        }
        GetCenteredCapsuleTriangleDepth(candidate, radius, out depth, out _);

        return true;
    }

    internal static bool TryGetCenteredCapsuleTriangleLocalContact(
        FixedTriangle triangle,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackNormal,
        FixedQuaternion radialSourceRotation,
        FixedQuaternion radialTargetRotation,
        out Vector3d pointOnTriangle,
        out Fixed64 axisParameter,
        out Vector3d normal,
        out Fixed64 depth,
        out bool depthIsClamped,
        bool retainFeature,
        out AxisTriangleContactFeature feature,
        out Vector3d rayRadialOffset,
        out int rayEndpointSign,
        out bool hasBoundaryRay,
        out int intervalEndpoint,
        in RationalSegmentInterval interval = default)
    {
        feature = default;
        rayRadialOffset = default;
        rayEndpointSign = 0;
        hasBoundaryRay = false;
        intervalEndpoint = 0;
        AxisTriangleCandidate candidate;
        if (interval.IsClipped)
            candidate = GetClosestClippedAxisSegment(triangle.A, triangle.B, center, axisDirection,
                axisLength, interval, out intervalEndpoint);
        else
            GetClosestCenteredAxisTriangleCandidate(triangle, center, axisDirection, axisLength, out candidate);
        Signed576 radiusNumerator = WideArithmetic.MultiplySigned576(
            candidate.Denominator,
            Signed192.Signed(radius.m_rawValue));
        if (WideArithmetic.SubtractSigned832(
                candidate.SquaredDistanceNumerator,
                WideArithmetic.MultiplySigned576ToSigned832(
                    radiusNumerator,
                    radiusNumerator)).Sign > 0)
        {
            pointOnTriangle = default;
            axisParameter = default;
            normal = default;
            depth = default;
            depthIsClamped = default;
            return false;
        }

        Signed576 differenceX = WideArithmetic.SubtractSigned576(
            candidate.AxisX,
            candidate.TriangleX);
        Signed576 differenceY = WideArithmetic.SubtractSigned576(
            candidate.AxisY,
            candidate.TriangleY);
        Signed576 differenceZ = WideArithmetic.SubtractSigned576(
            candidate.AxisZ,
            candidate.TriangleZ);
        if (retainFeature)
        {
            int vertex = candidate.EdgeMask == 5 ? 0 : candidate.EdgeMask == 3 ? 1 : candidate.EdgeMask == 6 ? 2 : -1;
            if (candidate.EdgeMask == 7)
                vertex = IsVertex(triangle.A) ? 0 : IsVertex(triangle.B) ? 1 : IsVertex(triangle.C) ? 2 : -1;
            feature = new AxisTriangleContactFeature(candidate.EdgeMask, differenceX, differenceY, differenceZ, vertex);

            bool IsVertex(Vector3d point) =>
                SubtractVertex(candidate.TriangleX, point.X, candidate.Denominator).IsZero
                && SubtractVertex(candidate.TriangleY, point.Y, candidate.Denominator).IsZero
                && SubtractVertex(candidate.TriangleZ, point.Z, candidate.Denominator).IsZero;
        }

        normal = WideNormalization.GetNormalized(
            differenceX,
            differenceY,
            differenceZ);
        if (normal.IsZero)
            normal = fallbackNormal;

        hasBoundaryRay = candidate.SquaredDistanceNumerator.IsZero && axisLength != Fixed64.Zero;
        if (hasBoundaryRay)
        {
            Span<ulong> values = stackalloc ulong[4 * CapsuleTriangleRayWords];
            Span<int> signs = stackalloc int[4];
            Span<ulong> root = stackalloc ulong[CapsuleTriangleRayRootWords];
            ContactQuadratic n = RayField(values, signs, 0), d = RayField(values, signs, 1);
            GetCoincidentCapsuleRayExit(candidate, center, axisDirection, axisLength, radius,
                -normal, n, d, root, out rayEndpointSign);
            GetCapsuleRayDepth(n, d, root, normal, out depth, out depthIsClamped);
            MaterializeCapsuleRayComponents(candidate, center, axisDirection, axisLength,
                -normal, rayEndpointSign, n, d, root, radialSourceRotation, radialTargetRotation,
                out axisParameter, out rayRadialOffset);
            _ = TryMaterialize(candidate, out pointOnTriangle, out _);
            return true;
        }

        // Both witnesses are convex points on representable local features:
        // the triangle point stays within its vertex component intervals and
        // the axis offset stays within the representable half length.
        _ = TryGetRelativeCoordinate(
            candidate.TriangleX,
            Fixed64.Zero,
            candidate.Denominator,
            out Fixed64 triangleX);
        _ = TryGetRelativeCoordinate(
            candidate.TriangleY,
            Fixed64.Zero,
            candidate.Denominator,
            out Fixed64 triangleY);
        _ = TryGetRelativeCoordinate(
            candidate.TriangleZ,
            Fixed64.Zero,
            candidate.Denominator,
            out Fixed64 triangleZ);
        axisParameter = Fixed64.Zero;
        if (axisLength != Fixed64.Zero)
        {
            _ = TryGetRelativeCoordinate(
                candidate.AxisX,
                center.X,
                candidate.Denominator,
                out Fixed64 axisX);
            _ = TryGetRelativeCoordinate(
                candidate.AxisY,
                center.Y,
                candidate.Denominator,
                out Fixed64 axisY);
            _ = TryGetRelativeCoordinate(
                candidate.AxisZ,
                center.Z,
                candidate.Denominator,
                out Fixed64 axisZ);
            axisParameter = Vector3d.Dot(new Vector3d(axisX, axisY, axisZ), axisDirection);
        }
        GetCenteredCapsuleTriangleDepth(
            candidate,
            radius,
            out depth,
            out depthIsClamped);

        pointOnTriangle = new Vector3d(
            triangleX,
            triangleY,
            triangleZ);
        return true;
    }

    private static void GetClosestCenteredAxisTriangleCandidate(
        FixedTriangle triangle,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        out AxisTriangleCandidate best)
    {
        if (axisLength == Fixed64.Zero && triangle.TryGetClosestPointBarycentrics(center,
            out Signed320 numeratorB, out Signed320 numeratorC, out Signed320 pointDenominator, out int edgeMask))
        {
            // Select the sphere's single feature before constructing any witness.
            // Common barycentric scale is exact and has no geometric meaning.
            Span<Signed320> weights = stackalloc Signed320[3] { numeratorB, numeratorC, pointDenominator };
            WideArithmetic.ReduceCommonScale(weights);
            numeratorB = weights[0]; numeratorC = weights[1]; pointDenominator = weights[2];
            best = CreateAxisTriangleCandidate(
                GetBarycentricCoordinateNumerator(triangle.A.X, triangle.B.X, triangle.C.X, numeratorB, numeratorC, pointDenominator),
                GetBarycentricCoordinateNumerator(triangle.A.Y, triangle.B.Y, triangle.C.Y, numeratorB, numeratorC, pointDenominator),
                GetBarycentricCoordinateNumerator(triangle.A.Z, triangle.B.Z, triangle.C.Z, numeratorB, numeratorC, pointDenominator),
                WideArithmetic.MultiplySigned320(Signed320.ExtendValue(Signed192.Raw(center.X)), pointDenominator),
                WideArithmetic.MultiplySigned320(Signed320.ExtendValue(Signed192.Raw(center.Y)), pointDenominator),
                WideArithmetic.MultiplySigned320(Signed320.ExtendValue(Signed192.Raw(center.Z)), pointDenominator),
                Signed576.ExtendValue(pointDenominator), edgeMask);
            return;
        }
        bool found = TryGetCenteredAxisTriangleFaceCandidate(
            triangle,
            center,
            axisDirection,
            axisLength,
            out best,
            out bool degenerate);
        // The closest segment point to the infinite plane is a lower bound
        // for every triangle point; an admitted projection cannot lose to an edge.
        if (found) return;
        KeepCenteredAxisTriangleEdgeCandidate(
            triangle.A,
            triangle.B,
            center,
            axisDirection,
            axisLength,
            degenerate ? 7 : 1, 5, 3,
            ref found,
            ref best);
        KeepCenteredAxisTriangleEdgeCandidate(
            triangle.B,
            triangle.C,
            center,
            axisDirection,
            axisLength,
            degenerate ? 7 : 2, 3, 6,
            ref found,
            ref best);
        KeepCenteredAxisTriangleEdgeCandidate(
            triangle.C,
            triangle.A,
            center,
            axisDirection,
            axisLength,
            degenerate ? 7 : 4, 6, 5,
            ref found,
            ref best);
    }

    private static Signed576 GetBarycentricCoordinateNumerator(Fixed64 a, Fixed64 b, Fixed64 c,
        Signed320 numeratorB, Signed320 numeratorC, Signed320 denominator) =>
        // Full-domain differences are 65 bits, barycentrics at most 262 bits;
        // each coordinate and its squared separation fit the existing 576/832 domains.
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(Signed192.Raw(a)), denominator),
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(GetComponentDifference(b, a)), numeratorB)),
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(GetComponentDifference(c, a)), numeratorC));

    private static bool TryGetCenteredAxisTriangleFaceCandidate(
        FixedTriangle triangle,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        out AxisTriangleCandidate candidate,
        out bool degenerate)
    {
        GetTriangleNormal(
            triangle,
            out Signed192 normalX,
            out Signed192 normalY,
            out Signed192 normalZ);
        Signed320 normalSquared = WideGeometry.GetSquaredMagnitude(
            normalX,
            normalY,
            normalZ,
            out _,
            out _,
            out _);
        degenerate = normalSquared.IsZero;
        if (degenerate)
        {
            candidate = default;
            return false;
        }

        // A positive common normal scale carries no plane geometry; cancel
        // it before it multiplies every rational distance and depth coefficient.
        Span<Signed320> primitive = stackalloc Signed320[3]
        {
            Signed320.ExtendValue(normalX), Signed320.ExtendValue(normalY), Signed320.ExtendValue(normalZ)
        };
        WideArithmetic.ReduceCommonScale(primitive);
        normalX = Signed192.NarrowProven(primitive[0]);
        normalY = Signed192.NarrowProven(primitive[1]);
        normalZ = Signed192.NarrowProven(primitive[2]);
        normalSquared = WideGeometry.GetSquaredMagnitude(normalX, normalY, normalZ, out _, out _, out _);

        Signed320 planeProjection = GetWideDot(
            normalX,
            normalY,
            normalZ,
            center,
            triangle.A);
        Signed320 axisProjection = GetWideDot(
            normalX,
            normalY,
            normalZ,
            axisDirection,
            Vector3d.Zero);
        if (axisProjection.IsZero)
        {
            // A parallel core has constant plane distance. Test center and
            // exact finite endpoints before edges cover any projected crossings.
            if (TryCreateCenteredAxisTriangleFaceCandidate(triangle, center, axisDirection,
                normalX, normalY, normalZ, normalSquared, default,
                Signed320.ExtendValue(Signed192.Signed(1)), out candidate)) return true;
            if (axisLength == Fixed64.Zero) return false;
            Signed320 capDenominator = Signed320.ExtendValue(DoubleParameterScale);
            if (TryCreateCenteredAxisTriangleFaceCandidate(triangle, center, axisDirection,
                normalX, normalY, normalZ, normalSquared,
                Signed320.ExtendValue(Signed192.Signed(-axisLength.m_rawValue)),
                capDenominator, out candidate)) return true;
            return TryCreateCenteredAxisTriangleFaceCandidate(triangle, center, axisDirection,
                normalX, normalY, normalZ, normalSquared,
                Signed320.ExtendValue(Signed192.Raw(axisLength)), capDenominator, out candidate);
        }

        Signed320 parameterNumerator =
            WideArithmetic.SubtractSigned320(default, planeProjection);
        Signed320 parameterDenominator = axisProjection;
        if (parameterDenominator.Sign < 0)
        {
            parameterNumerator =
                WideArithmetic.SubtractSigned320(default, parameterNumerator);
            parameterDenominator =
                WideArithmetic.SubtractSigned320(default, parameterDenominator);
        }
        ClampCenteredAxisParameter(
            ref parameterNumerator,
            ref parameterDenominator,
            axisLength);

        return TryCreateCenteredAxisTriangleFaceCandidate(triangle, center, axisDirection,
            normalX, normalY, normalZ, normalSquared, parameterNumerator, parameterDenominator, out candidate);
    }

    private static bool TryCreateCenteredAxisTriangleFaceCandidate(
        FixedTriangle triangle, Vector3d center, Vector3d axisDirection,
        Signed192 normalX, Signed192 normalY, Signed192 normalZ, Signed320 normalSquared,
        Signed320 parameterNumerator, Signed320 parameterDenominator,
        out AxisTriangleCandidate candidate)
    {
        // Zero axial displacement has denominator one; retain no artificial
        // parameter scale in every downstream plane-distance coefficient.
        if (parameterNumerator.IsZero)
            parameterDenominator = Signed320.ExtendValue(Signed192.Signed(1));
        GetRationalPoint(
            center,
            axisDirection,
            parameterNumerator,
            parameterDenominator,
            out Signed576 axisX,
            out Signed576 axisY,
            out Signed576 axisZ);
        Signed576 wideParameterDenominator =
            Signed576.ExtendValue(parameterDenominator);
        if (!ContainsTriangleProjection(
                triangle,
                normalX,
                normalY,
                normalZ,
                axisX,
                axisY,
                axisZ,
                wideParameterDenominator,
                out int edgeMask))
        {
            candidate = default;
            return false;
        }

        Signed576 planeDistanceNumerator = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    SubtractVertex(axisX, triangle.A.X, parameterDenominator),
                    normalX),
                WideArithmetic.MultiplySigned576(
                    SubtractVertex(axisY, triangle.A.Y, parameterDenominator),
                    normalY)),
            WideArithmetic.MultiplySigned576(
                SubtractVertex(axisZ, triangle.A.Z, parameterDenominator),
                normalZ));
        // A full-domain center scaled by the face parameter denominator plus
        // one normalized-axis parameter product needs fewer than 320 signed bits.
        Signed320 narrowAxisX = new(
            axisX.Word4,
            axisX.Word3,
            axisX.Word2,
            axisX.Word1,
            axisX.Word0);
        Signed320 narrowAxisY = new(
            axisY.Word4,
            axisY.Word3,
            axisY.Word2,
            axisY.Word1,
            axisY.Word0);
        Signed320 narrowAxisZ = new(
            axisZ.Word4,
            axisZ.Word3,
            axisZ.Word2,
            axisZ.Word1,
            axisZ.Word0);
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            parameterDenominator,
            normalSquared);
        Signed576 scaledAxisX = WideArithmetic.MultiplySigned320(
            narrowAxisX,
            normalSquared);
        Signed576 scaledAxisY = WideArithmetic.MultiplySigned320(
            narrowAxisY,
            normalSquared);
        Signed576 scaledAxisZ = WideArithmetic.MultiplySigned320(
            narrowAxisZ,
            normalSquared);
        Signed576 triangleX = WideArithmetic.SubtractSigned576(
            scaledAxisX,
            WideArithmetic.MultiplySigned576(planeDistanceNumerator, normalX));
        Signed576 triangleY = WideArithmetic.SubtractSigned576(
            scaledAxisY,
            WideArithmetic.MultiplySigned576(planeDistanceNumerator, normalY));
        Signed576 triangleZ = WideArithmetic.SubtractSigned576(
            scaledAxisZ,
            WideArithmetic.MultiplySigned576(planeDistanceNumerator, normalZ));
        candidate = CreateAxisTriangleCandidate(
            triangleX,
            triangleY,
            triangleZ,
            scaledAxisX,
            scaledAxisY,
            scaledAxisZ,
            denominator,
            edgeMask);
        return true;
    }

    private static void KeepCenteredAxisTriangleEdgeCandidate(
        Vector3d edgeStart,
        Vector3d edgeEnd,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        int edgeMask, int startMask, int endMask,
        ref bool found,
        ref AxisTriangleCandidate best)
    {
        GetClosestCenteredAxisSegmentParameters(
            center,
            axisDirection,
            axisLength,
            edgeStart,
            edgeEnd,
            out Signed320 axisNumerator,
            out Signed320 edgeNumerator,
            out Signed320 denominator);
        GetRationalPoint(
            center,
            axisDirection,
            axisNumerator,
            denominator,
            out Signed576 axisX,
            out Signed576 axisY,
            out Signed576 axisZ);
        GetRationalSegmentPoint(
            edgeStart,
            edgeEnd,
            edgeNumerator,
            denominator,
            out Signed576 edgeX,
            out Signed576 edgeY,
            out Signed576 edgeZ);
        // Retain the exact clamped edge feature before anchor rounding. A
        // degenerate triangle has no trusted face and keeps the conservative mask.
        if (edgeMask != 7)
        {
            if (edgeNumerator.IsZero) edgeMask = startMask;
            else if (WideArithmetic.SubtractSigned320(edgeNumerator, denominator).IsZero) edgeMask = endMask;
        }
        AxisTriangleCandidate candidate = CreateAxisTriangleCandidate(
            edgeX,
            edgeY,
            edgeZ,
            axisX,
            axisY,
            axisZ,
            Signed576.ExtendValue(denominator),
            edgeMask);
        if (found
            && WideArithmetic.CompareNonNegativeSquaredRatios(
                candidate.SquaredDistanceNumerator,
                candidate.Denominator,
                best.SquaredDistanceNumerator,
                best.Denominator) >= 0)
        {
            return;
        }

        found = true;
        best = candidate;
    }

    private static void GetClosestCenteredAxisSegmentParameters(
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        Vector3d edgeStart,
        Vector3d edgeEnd,
        out Signed320 axisNumerator,
        out Signed320 edgeNumerator,
        out Signed320 denominator)
    {
        Signed192 axisSquared = GetDot(axis, Vector3d.Zero, axis, Vector3d.Zero);
        Signed192 directionsDot = GetDot(axis, Vector3d.Zero, edgeEnd, edgeStart);
        Signed192 edgeSquared = GetDot(edgeEnd, edgeStart, edgeEnd, edgeStart);
        Signed192 axisDotDifference = GetDot(center, edgeStart, axis, Vector3d.Zero);
        Signed192 edgeDotDifference = GetDot(center, edgeStart, edgeEnd, edgeStart);
        if (edgeSquared.IsZero)
        {
            axisNumerator = Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(default, axisDotDifference));
            denominator = Signed320.ExtendValue(axisSquared);
            ClampCenteredAxisParameter(
                ref axisNumerator,
                ref denominator,
                axisLength);
            edgeNumerator = default;
            return;
        }

        if (axisLength == Fixed64.Zero)
        {
            // A sphere core is one point: its exact edge fraction needs no
            // line-line determinant, axial clamp or endpoint reprojection.
            axisNumerator = default;
            edgeNumerator = Signed320.ExtendValue(edgeDotDifference);
            denominator = Signed320.ExtendValue(edgeSquared);
            ClampUnitParameter(ref edgeNumerator, ref denominator);
            Span<Signed320> ratio = stackalloc Signed320[2] { edgeNumerator, denominator };
            WideArithmetic.ReduceCommonScale(ratio);
            edgeNumerator = ratio[0]; denominator = ratio[1];
            return;
        }

        Signed320 determinant = WideArithmetic.MultiplySubtract(
            axisSquared,
            edgeSquared,
            directionsDot,
            directionsDot);
        if (determinant.Sign > 0)
        {
            axisNumerator = WideArithmetic.MultiplySubtract(
                directionsDot,
                edgeDotDifference,
                edgeSquared,
                axisDotDifference);
            edgeNumerator = WideArithmetic.MultiplySubtract(
                axisSquared,
                edgeDotDifference,
                directionsDot,
                axisDotDifference);
            denominator = determinant;
            if (!ClampCenteredAxisParameter(
                    ref axisNumerator,
                    ref denominator,
                    axisLength))
            {
                if (!ClampUnitParameter(ref edgeNumerator, ref denominator))
                    return;

                ProjectAxisFromEdgeCap(
                    edgeNumerator,
                    axisSquared,
                    directionsDot,
                    axisDotDifference,
                    out axisNumerator,
                    out denominator);
                if (!ClampCenteredAxisParameter(
                        ref axisNumerator,
                        ref denominator,
                        axisLength))
                {
                    edgeNumerator = WideArithmetic.MultiplySigned192(
                        new Signed192(
                            edgeNumerator.Word2,
                            edgeNumerator.Word1,
                            edgeNumerator.Word0),
                        axisSquared);
                    return;
                }

                SetUnitCapAtCenteredDenominator(
                    ref edgeNumerator,
                    ref denominator);
                return;
            }
        }
        else
        {
            // Parallel finite segments have a whole closest-point interval.
            // Keep its exact midpoint when the interval has positive length;
            // selecting an arbitrary cap would turn an interior edge family
            // into a corner contact. All pre-product terms fit Signed192
            // (normalized axis <=34 bits, full-domain differences <=65 bits).
            Signed320 first = WideArithmetic.MultiplySigned192(DoubleParameterScale,
                WideArithmetic.SubtractSigned192(default, axisDotDifference));
            Signed320 second = WideArithmetic.MultiplySigned192(DoubleParameterScale,
                WideArithmetic.SubtractSigned192(directionsDot, axisDotDifference));
            if (WideArithmetic.SubtractSigned320(first, second).Sign > 0)
                (first, second) = (second, first);
            Signed320 cap = WideArithmetic.MultiplySigned192(Signed192.Raw(axisLength), axisSquared);
            Signed320 negativeCap = WideArithmetic.SubtractSigned320(default, cap);
            if (WideArithmetic.SubtractSigned320(first, negativeCap).Sign < 0) first = negativeCap;
            if (WideArithmetic.SubtractSigned320(second, cap).Sign > 0) second = cap;
            if (WideArithmetic.SubtractSigned320(second, first).Sign > 0)
            {
                Signed192 midpoint = Signed192.NarrowProven(WideArithmetic.AddSigned320(first, second));
                Signed320 common = WideArithmetic.MultiplySigned192(DoubleParameterScale, axisSquared);
                Signed192 twiceCommon = Signed192.NarrowProven(WideArithmetic.AddSigned320(common, common));
                edgeNumerator = WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(edgeDotDifference, twiceCommon),
                    WideArithmetic.MultiplySigned192(directionsDot, midpoint));
                axisNumerator = WideArithmetic.MultiplySigned192(midpoint, edgeSquared);
                denominator = WideArithmetic.MultiplySigned192(twiceCommon, edgeSquared);
                return;
            }
            axisNumerator = Signed320.ExtendValue(
                Signed192.Signed(-axisLength.m_rawValue));
            denominator = Signed320.ExtendValue(DoubleParameterScale);
        }

        ProjectEdgeFromCenteredCap(
            axisNumerator,
            directionsDot,
            edgeSquared,
            edgeDotDifference,
            out edgeNumerator,
            out denominator);
        if (!ClampUnitParameter(ref edgeNumerator, ref denominator))
        {
            axisNumerator = WideArithmetic.MultiplySigned192(
                new Signed192(
                    axisNumerator.Word2,
                    axisNumerator.Word1,
                    axisNumerator.Word0),
                edgeSquared);
            return;
        }

        ProjectAxisFromEdgeCap(
            edgeNumerator,
            axisSquared,
            directionsDot,
            axisDotDifference,
            out axisNumerator,
            out denominator);
        if (!ClampCenteredAxisParameter(
                ref axisNumerator,
                ref denominator,
                axisLength))
        {
            edgeNumerator = WideArithmetic.MultiplySigned192(
                new Signed192(
                    edgeNumerator.Word2,
                    edgeNumerator.Word1,
                    edgeNumerator.Word0),
                axisSquared);
            return;
        }

        SetUnitCapAtCenteredDenominator(
            ref edgeNumerator,
            ref denominator);
    }

    private static bool ClampUnitParameter(
        ref Signed320 numerator,
        ref Signed320 denominator)
    {
        if (numerator.Sign < 0)
        {
            numerator = default;
            denominator = Signed320.ExtendValue(ParameterScale);
            return true;
        }
        if (WideArithmetic.SubtractSigned320(numerator, denominator).Sign > 0)
        {
            numerator = Signed320.ExtendValue(ParameterScale);
            denominator = Signed320.ExtendValue(ParameterScale);
            return true;
        }

        return false;
    }

    private static void ProjectEdgeFromCenteredCap(
        Signed320 axisCapNumerator,
        Signed192 directionsDot,
        Signed192 edgeSquared,
        Signed192 edgeDotDifference,
        out Signed320 edgeNumerator,
        out Signed320 denominator)
    {
        Signed192 capNumerator = new(
            axisCapNumerator.Word2,
            axisCapNumerator.Word1,
            axisCapNumerator.Word0);
        edgeNumerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                DoubleParameterScale,
                edgeDotDifference),
            WideArithmetic.MultiplySigned192(
                capNumerator,
                directionsDot));
        denominator = WideArithmetic.MultiplySigned192(
            DoubleParameterScale,
            edgeSquared);
    }

    private static void ProjectAxisFromEdgeCap(
        Signed320 edgeCapNumerator,
        Signed192 axisSquared,
        Signed192 directionsDot,
        Signed192 axisDotDifference,
        out Signed320 axisNumerator,
        out Signed320 denominator)
    {
        Signed192 capNumerator = new(
            edgeCapNumerator.Word2,
            edgeCapNumerator.Word1,
            edgeCapNumerator.Word0);
        axisNumerator = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                capNumerator,
                directionsDot),
            WideArithmetic.MultiplySigned192(
                ParameterScale,
                axisDotDifference));
        denominator = WideArithmetic.MultiplySigned192(
            ParameterScale,
            axisSquared);
    }

    private static void SetUnitCapAtCenteredDenominator(
        ref Signed320 numerator,
        ref Signed320 denominator)
    {
        bool positive = numerator.Sign > 0;
        numerator = positive
            ? Signed320.ExtendValue(DoubleParameterScale)
            : default;
        denominator = Signed320.ExtendValue(DoubleParameterScale);
    }

    private static void GetRationalPoint(
        Vector3d center,
        Vector3d direction,
        Signed320 parameterNumerator,
        Signed320 denominator,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z)
    {
        x = GetRationalCoordinate(
            center.X,
            Signed192.Signed(direction.X.m_rawValue),
            parameterNumerator,
            denominator);
        y = GetRationalCoordinate(
            center.Y,
            Signed192.Signed(direction.Y.m_rawValue),
            parameterNumerator,
            denominator);
        z = GetRationalCoordinate(
            center.Z,
            Signed192.Signed(direction.Z.m_rawValue),
            parameterNumerator,
            denominator);
    }

    private static void GetRationalSegmentPoint(
        Vector3d start,
        Vector3d end,
        Signed320 parameterNumerator,
        Signed320 denominator,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z)
    {
        x = GetRationalCoordinate(
            start.X,
            GetComponentDifference(end.X, start.X),
            parameterNumerator,
            denominator);
        y = GetRationalCoordinate(
            start.Y,
            GetComponentDifference(end.Y, start.Y),
            parameterNumerator,
            denominator);
        z = GetRationalCoordinate(
            start.Z,
            GetComponentDifference(end.Z, start.Z),
            parameterNumerator,
            denominator);
    }

    private static Signed576 GetRationalCoordinate(
        Fixed64 origin,
        Signed192 direction,
        Signed320 parameterNumerator,
        Signed320 denominator) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(
                    Signed192.Signed(origin.m_rawValue)),
                denominator),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(direction),
                parameterNumerator));

    private static AxisTriangleCandidate CreateAxisTriangleCandidate(
        Signed576 triangleX,
        Signed576 triangleY,
        Signed576 triangleZ,
        Signed576 axisX,
        Signed576 axisY,
        Signed576 axisZ,
        Signed576 denominator,
        int edgeMask)
    {
        Signed576 differenceX =
            WideArithmetic.SubtractSigned576(axisX, triangleX);
        Signed576 differenceY =
            WideArithmetic.SubtractSigned576(axisY, triangleY);
        Signed576 differenceZ =
            WideArithmetic.SubtractSigned576(axisZ, triangleZ);
        Signed832 squaredDistance = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(
                    differenceX,
                    differenceX),
                WideArithmetic.MultiplySigned576ToSigned832(
                    differenceY,
                    differenceY)),
            WideArithmetic.MultiplySigned576ToSigned832(
                differenceZ,
                differenceZ));
        return new AxisTriangleCandidate(
            triangleX,
            triangleY,
            triangleZ,
            axisX,
            axisY,
            axisZ,
            denominator,
            squaredDistance,
            edgeMask);
    }


    private static void GetCenteredCapsuleTriangleDepth(
        AxisTriangleCandidate candidate,
        Fixed64 radius,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        GetCenteredRadiusDepth(
            candidate.SquaredDistanceNumerator,
            candidate.Denominator,
            Signed192.Signed(radius.m_rawValue),
            out depth,
            out depthIsClamped);
    }
}
