//=======================================================================
// WideFiniteAxisIntersection.CenteredAxisTriangle.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

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

        internal AxisTriangleCandidate(
            Signed576 triangleX,
            Signed576 triangleY,
            Signed576 triangleZ,
            Signed576 axisX,
            Signed576 axisY,
            Signed576 axisZ,
            Signed576 denominator,
            Signed832 squaredDistanceNumerator)
        {
            TriangleX = triangleX;
            TriangleY = triangleY;
            TriangleZ = triangleZ;
            AxisX = axisX;
            AxisY = axisY;
            AxisZ = axisZ;
            Denominator = denominator;
            SquaredDistanceNumerator = squaredDistanceNumerator;
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

        normal = WideGeometry.GetDirection(pointOnTriangle, pointOnAxis);
        if (normal.IsZero)
            normal = fallbackNormal;
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
        out Vector3d pointOnTriangle,
        out Fixed64 axisParameter,
        out Vector3d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
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
        normal = WideGeometry.GetNormalized(
            differenceX,
            differenceY,
            differenceZ);
        if (normal.IsZero)
            normal = fallbackNormal;

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
        GetCenteredCapsuleTriangleDepth(
            candidate,
            radius,
            out depth,
            out depthIsClamped);

        pointOnTriangle = new Vector3d(
            triangleX,
            triangleY,
            triangleZ);
        axisParameter = Vector3d.Dot(
            new Vector3d(axisX, axisY, axisZ),
            axisDirection);
        return true;
    }

    private static void GetClosestCenteredAxisTriangleCandidate(
        FixedTriangle triangle,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        out AxisTriangleCandidate best)
    {
        bool found = TryGetCenteredAxisTriangleFaceCandidate(
            triangle,
            center,
            axisDirection,
            axisLength,
            out best);
        KeepCenteredAxisTriangleEdgeCandidate(
            triangle.A,
            triangle.B,
            center,
            axisDirection,
            axisLength,
            ref found,
            ref best);
        KeepCenteredAxisTriangleEdgeCandidate(
            triangle.B,
            triangle.C,
            center,
            axisDirection,
            axisLength,
            ref found,
            ref best);
        KeepCenteredAxisTriangleEdgeCandidate(
            triangle.C,
            triangle.A,
            center,
            axisDirection,
            axisLength,
            ref found,
            ref best);
    }

    private static bool TryGetCenteredAxisTriangleFaceCandidate(
        FixedTriangle triangle,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        out AxisTriangleCandidate candidate)
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
        if (normalSquared.IsZero)
        {
            candidate = default;
            return false;
        }

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
            candidate = default;
            return false;
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
                wideParameterDenominator))
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
            denominator);
        return true;
    }

    private static void KeepCenteredAxisTriangleEdgeCandidate(
        Vector3d edgeStart,
        Vector3d edgeEnd,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
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
        AxisTriangleCandidate candidate = CreateAxisTriangleCandidate(
            edgeX,
            edgeY,
            edgeZ,
            axisX,
            axisY,
            axisZ,
            Signed576.ExtendValue(denominator));
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
        Signed576 denominator)
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
            squaredDistance);
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
