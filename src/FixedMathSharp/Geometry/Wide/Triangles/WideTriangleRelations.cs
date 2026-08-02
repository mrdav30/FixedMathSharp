//=======================================================================
// WideTriangleRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns exact full-domain relations for <see cref="FixedTriangle"/>.
/// </summary>
internal static partial class WideTriangleRelations
{
    internal static bool TryGetContact(
        FixedTriangle first,
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        FixedTriangle second,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        out FixedContactAnchors contact)
    {
        first.GetExactNormal(
            out Signed192 firstNormalX,
            out Signed192 firstNormalY,
            out Signed192 firstNormalZ,
            out Signed320 firstNormalSquared);
        second.GetExactNormal(
            out Signed192 secondNormalX,
            out Signed192 secondNormalY,
            out Signed192 secondNormalZ,
            out Signed320 secondNormalSquared);
        if (firstNormalSquared.IsZero || secondNormalSquared.IsZero)
        {
            contact = default;
            return false;
        }

        WideRationalBasis3d firstBasis = new(firstRotation);
        WideRationalBasis3d secondBasis = new(secondRotation);
        WideAxis3 firstNormal = WideRigidProjection.TransformLocalAxis(
            firstBasis,
            firstNormalX,
            firstNormalY,
            firstNormalZ);
        WideAxis3 secondNormal = WideRigidProjection.TransformLocalAxis(
            secondBasis,
            secondNormalX,
            secondNormalY,
            secondNormalZ);
        var best = default(WidePointSpanPenetration);
        if (!TryKeepPairAxis(
                firstNormal,
                first,
                firstOrigin,
                firstBasis,
                second,
                secondOrigin,
                secondBasis,
                ref best)
            || !TryKeepPairAxis(
                secondNormal,
                first,
                firstOrigin,
                firstBasis,
                second,
                secondOrigin,
                secondBasis,
                ref best))
        {
            contact = default;
            return false;
        }

        for (int firstEdgeIndex = 0;
            firstEdgeIndex < FixedTriangle.EdgeCount;
            firstEdgeIndex++)
        {
            GetLocalEdge(
                first,
                firstEdgeIndex,
                out Signed192 firstEdgeX,
                out Signed192 firstEdgeY,
                out Signed192 firstEdgeZ);
            WideAxis3 firstEdge = WideRigidProjection.TransformLocalAxis(
                firstBasis,
                firstEdgeX,
                firstEdgeY,
                firstEdgeZ);
            if (!TryKeepPairAxis(
                    TransformLocalNormalCrossEdge(
                        firstBasis,
                        firstNormalX,
                        firstNormalY,
                        firstNormalZ,
                        firstEdgeX,
                        firstEdgeY,
                        firstEdgeZ),
                    first,
                    firstOrigin,
                    firstBasis,
                    second,
                    secondOrigin,
                    secondBasis,
                    ref best))
            {
                contact = default;
                return false;
            }

            GetLocalEdge(
                second,
                firstEdgeIndex,
                out Signed192 secondOwnEdgeX,
                out Signed192 secondOwnEdgeY,
                out Signed192 secondOwnEdgeZ);
            if (!TryKeepPairAxis(
                    TransformLocalNormalCrossEdge(
                        secondBasis,
                        secondNormalX,
                        secondNormalY,
                        secondNormalZ,
                        secondOwnEdgeX,
                        secondOwnEdgeY,
                        secondOwnEdgeZ),
                    first,
                    firstOrigin,
                    firstBasis,
                    second,
                    secondOrigin,
                    secondBasis,
                    ref best))
            {
                contact = default;
                return false;
            }

            for (int secondEdgeIndex = 0;
                secondEdgeIndex < FixedTriangle.EdgeCount;
                secondEdgeIndex++)
            {
                GetLocalEdge(
                    second,
                    secondEdgeIndex,
                    out Signed192 secondEdgeX,
                    out Signed192 secondEdgeY,
                    out Signed192 secondEdgeZ);
                WideAxis3 secondEdge = WideRigidProjection.TransformLocalAxis(
                    secondBasis,
                    secondEdgeX,
                    secondEdgeY,
                    secondEdgeZ);
                if (!TryKeepPairAxis(
                        WideAxis3.Cross(firstEdge, secondEdge),
                        first,
                        firstOrigin,
                        firstBasis,
                        second,
                        secondOrigin,
                        secondBasis,
                        ref best))
                {
                    contact = default;
                    return false;
                }
            }
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideNormalization.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        FixedPointAnchor secondCentroid = FixedPointAnchor.FromValidatedFrame(
            secondOrigin,
            secondRotation,
            second.Centroid);
        FixedPointAnchor firstAnchor = GetClosestPointAnchor(
            first,
            firstOrigin,
            firstRotation,
            secondCentroid);
        FixedPointAnchor secondAnchor = GetClosestPointAnchor(
            second,
            secondOrigin,
            secondRotation,
            firstAnchor);
        Fixed64 depth = best.GetRoundedDepth(out bool depthIsClamped);
        contact = new FixedContactAnchors(
            firstAnchor,
            secondAnchor,
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryKeepPairAxis(
        in WideAxis3 axis,
        in FixedTriangle first,
        Vector3d firstOrigin,
        in WideRationalBasis3d firstBasis,
        in FixedTriangle second,
        Vector3d secondOrigin,
        in WideRationalBasis3d secondBasis,
        ref WidePointSpanPenetration best)
    {
        if (axis.IsZero)
            return true;

        GetProjectionInterval(
            first,
            firstBasis,
            axis,
            secondBasis.Denominator,
            out Signed576 firstMinimum,
            out Signed576 firstMaximum,
            out Signed576 firstSum);
        GetProjectionInterval(
            second,
            secondBasis,
            axis,
            firstBasis.Denominator,
            out Signed576 secondMinimum,
            out Signed576 secondMaximum,
            out Signed576 secondSum);
        Signed576 originCommon = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideRigidProjection.GetWorldOriginDifferenceProjection(
                    secondOrigin,
                    firstOrigin,
                    axis),
                firstBasis.Denominator),
            secondBasis.Denominator);
        secondMinimum = WideArithmetic.AddSigned576(
            secondMinimum,
            originCommon);
        secondMaximum = WideArithmetic.AddSigned576(
            secondMaximum,
            originCommon);

        Signed576 pushNegative = WideArithmetic.SubtractSigned576(
            firstMaximum,
            secondMinimum);
        Signed576 pushPositive = WideArithmetic.SubtractSigned576(
            secondMaximum,
            firstMinimum);
        if (pushNegative.Sign < 0 || pushPositive.Sign < 0)
            return false;

        Signed576 overlap = WideArithmetic.SubtractSigned576(
                pushPositive,
                pushNegative).Sign < 0
            ? pushPositive
            : pushNegative;
        Signed320 commonDenominator = WideArithmetic.MultiplySigned192(
            firstBasis.Denominator,
            secondBasis.Denominator);
        Signed576 squaredAxisLength = axis.SquaredLength;
        if (!best.ShouldReplace(
                overlap,
                squaredAxisLength,
                commonDenominator))
        {
            return true;
        }

        Signed576 secondCentroidProjection = WideArithmetic.AddSigned576(
            secondSum,
            WideArithmetic.AddSigned576(
                WideArithmetic.AddSigned576(originCommon, originCommon),
                originCommon));
        bool negate = WideArithmetic.SubtractSigned576(
            secondCentroidProjection,
            firstSum).Sign < 0;
        best = new WidePointSpanPenetration(
            axis,
            negate,
            overlap,
            squaredAxisLength,
            commonDenominator);
        return true;
    }

    private static void GetProjectionInterval(
        in FixedTriangle triangle,
        in WideRationalBasis3d basis,
        in WideAxis3 axis,
        Signed192 otherDenominator,
        out Signed576 minimum,
        out Signed576 maximum,
        out Signed576 sum)
    {
        WideRigidProjection.GetLocalAxisProjections(
            axis,
            basis,
            out Signed576 localAxisX,
            out Signed576 localAxisY,
            out Signed576 localAxisZ);
        Signed576 first = WideArithmetic.MultiplySigned576(
            WideRigidProjection.GetLocalOffsetProjection(
                triangle.A,
                localAxisX,
                localAxisY,
                localAxisZ),
            otherDenominator);
        minimum = first;
        maximum = first;
        Signed576 second = WideArithmetic.MultiplySigned576(
            WideRigidProjection.GetLocalOffsetProjection(
                triangle.B,
                localAxisX,
                localAxisY,
                localAxisZ),
            otherDenominator);
        Signed576 third = WideArithmetic.MultiplySigned576(
            WideRigidProjection.GetLocalOffsetProjection(
                triangle.C,
                localAxisX,
                localAxisY,
                localAxisZ),
            otherDenominator);
        WideRigidProjection.IncludeProjection(
            second,
            ref minimum,
            ref maximum);
        WideRigidProjection.IncludeProjection(
            third,
            ref minimum,
            ref maximum);
        sum = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(first, second),
            third);
    }

    private static WideAxis3 TransformLocalNormalCrossEdge(
        WideRationalBasis3d basis,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Signed192 edgeX,
        Signed192 edgeY,
        Signed192 edgeZ) =>
        WideRigidProjection.TransformLocalTriangleNormalCrossEdgeAxis(
            basis,
            WideArithmetic.MultiplySubtract(
                normalY,
                edgeZ,
                normalZ,
                edgeY),
            WideArithmetic.MultiplySubtract(
                normalZ,
                edgeX,
                normalX,
                edgeZ),
            WideArithmetic.MultiplySubtract(
                normalX,
                edgeY,
                normalY,
                edgeX));

    private static void GetLocalEdge(
        FixedTriangle triangle,
        int index,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z)
    {
        Vector3d start;
        Vector3d end;
        if (index == 0)
        {
            start = triangle.A;
            end = triangle.B;
        }
        else if (index == 1)
        {
            start = triangle.B;
            end = triangle.C;
        }
        else
        {
            start = triangle.C;
            end = triangle.A;
        }

        x = WideArithmetic.SubtractSigned192(
            Signed192.Raw(end.X),
            Signed192.Raw(start.X));
        y = WideArithmetic.SubtractSigned192(
            Signed192.Raw(end.Y),
            Signed192.Raw(start.Y));
        z = WideArithmetic.SubtractSigned192(
            Signed192.Raw(end.Z),
            Signed192.Raw(start.Z));
    }
}
