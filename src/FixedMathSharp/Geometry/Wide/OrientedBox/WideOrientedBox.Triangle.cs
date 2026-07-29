//=======================================================================
// WideOrientedBox.Triangle.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Oriented box vs. triangle contact generation using the separating axis theorem (SAT).
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetTriangleContact(
        Vector3d boxCenter,
        FixedQuaternion boxOrientation,
        Vector3d boxHalfExtents,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        out FixedContactAnchors contact)
    {
        if (triangle.IsDegenerate)
        {
            contact = default;
            return false;
        }

        WideRationalBasis3d boxBasis = new(boxOrientation);
        WideRationalBasis3d triangleBasis = new(triangleRotation);
        Span<WideAxis3> boxAxes = stackalloc WideAxis3[3]
        {
            GetBasisAxis(boxBasis, 0),
            GetBasisAxis(boxBasis, 1),
            GetBasisAxis(boxBasis, 2),
        };
        var best = default(PointSpanPenetration);
        for (int index = 0; index < boxAxes.Length; index++)
        {
            if (!TryKeepTriangleAxis(
                    boxAxes[index],
                    boxCenter,
                    boxHalfExtents,
                    boxBasis,
                    triangleOrigin,
                    triangleBasis,
                    triangle,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        WideGeometry.GetDifferenceCrossProduct3D(
            triangle.B.X,
            triangle.A.X,
            triangle.B.Y,
            triangle.A.Y,
            triangle.B.Z,
            triangle.A.Z,
            triangle.C.X,
            triangle.A.X,
            triangle.C.Y,
            triangle.A.Y,
            triangle.C.Z,
            triangle.A.Z,
            out Signed192 normalX,
            out Signed192 normalY,
            out Signed192 normalZ);
        if (!TryKeepTriangleAxis(
                TransformLocalAxis(
                    triangleBasis,
                    normalX,
                    normalY,
                    normalZ),
                boxCenter,
                boxHalfExtents,
                boxBasis,
                triangleOrigin,
                triangleBasis,
                triangle,
                ref best)
            || !TryKeepTriangleEdgeAxes(
                triangle.A,
                triangle.B,
                boxAxes,
                boxCenter,
                boxHalfExtents,
                boxBasis,
                triangleOrigin,
                triangleBasis,
                triangle,
                ref best)
            || !TryKeepTriangleEdgeAxes(
                triangle.B,
                triangle.C,
                boxAxes,
                boxCenter,
                boxHalfExtents,
                boxBasis,
                triangleOrigin,
                triangleBasis,
                triangle,
                ref best)
            || !TryKeepTriangleEdgeAxes(
                triangle.C,
                triangle.A,
                boxAxes,
                boxCenter,
                boxHalfExtents,
                boxBasis,
                triangleOrigin,
                triangleBasis,
                triangle,
                ref best))
        {
            contact = default;
            return false;
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d contactNormal = WideNormalization.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        Vector3d triangleLocalPoint = GetTriangleSupportLocalPoint(
            triangle,
            triangleBasis,
            orientedAxis);
        GetPointSpanDepth(
            best.ExactOverlap,
            best.ExactSquaredAxisLength,
            best.ExactCommonDenominator,
            out Fixed64 depth,
            out bool depthIsClamped);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                boxCenter,
                boxOrientation,
                GetLocalSupportPoint(
                    contactNormal,
                    boxOrientation,
                    boxHalfExtents)),
            new FixedPointAnchor(
                triangleOrigin,
                triangleRotation,
                triangleLocalPoint),
            contactNormal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryKeepTriangleEdgeAxes(
        Vector3d edgeStart,
        Vector3d edgeEnd,
        ReadOnlySpan<WideAxis3> boxAxes,
        Vector3d boxCenter,
        Vector3d boxHalfExtents,
        WideRationalBasis3d boxBasis,
        Vector3d triangleOrigin,
        WideRationalBasis3d triangleBasis,
        FixedTriangle triangle,
        ref PointSpanPenetration best)
    {
        WideAxis3 edge = TransformLocalAxis(
            triangleBasis,
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(edgeEnd.X),
                Signed192.Raw(edgeStart.X)),
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(edgeEnd.Y),
                Signed192.Raw(edgeStart.Y)),
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(edgeEnd.Z),
                Signed192.Raw(edgeStart.Z)));
        for (int axisIndex = 0; axisIndex < boxAxes.Length; axisIndex++)
        {
            if (!TryKeepTriangleAxis(
                    Cross(boxAxes[axisIndex], edge),
                    boxCenter,
                    boxHalfExtents,
                    boxBasis,
                    triangleOrigin,
                    triangleBasis,
                    triangle,
                    ref best))
            {
                return false;
            }
        }
        return true;
    }

    private static bool TryKeepTriangleAxis(
        WideAxis3 axis,
        Vector3d boxCenter,
        Vector3d boxHalfExtents,
        WideRationalBasis3d boxBasis,
        Vector3d triangleOrigin,
        WideRationalBasis3d triangleBasis,
        FixedTriangle triangle,
        ref PointSpanPenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 boxRadius = GetBoxProjectionRadiusNumerator(
            axis,
            boxHalfExtents,
            boxBasis);
        Signed576 first = GetTransformedOffsetProjection(
            triangle.A,
            triangleBasis,
            axis);
        Signed576 second = GetTransformedOffsetProjection(
            triangle.B,
            triangleBasis,
            axis);
        Signed576 third = GetTransformedOffsetProjection(
            triangle.C,
            triangleBasis,
            axis);
        Signed576 minimum = first;
        Signed576 maximum = first;
        KeepProjection(second, ref minimum, ref maximum);
        KeepProjection(third, ref minimum, ref maximum);

        Signed576 originProjection = GetDifferenceProjection(
            triangleOrigin,
            boxCenter,
            axis);
        Signed576 originNumerator = WideArithmetic.MultiplySigned576(
            originProjection,
            triangleBasis.Denominator);
        minimum = WideArithmetic.MultiplySigned576(
            WideArithmetic.AddSigned576(originNumerator, minimum),
            boxBasis.Denominator);
        maximum = WideArithmetic.MultiplySigned576(
            WideArithmetic.AddSigned576(originNumerator, maximum),
            boxBasis.Denominator);
        boxRadius = WideArithmetic.MultiplySigned576(
            boxRadius,
            triangleBasis.Denominator);
        Signed576 pushBoxNegative = WideArithmetic.SubtractSigned576(
            boxRadius,
            minimum);
        Signed576 pushBoxPositive = WideArithmetic.AddSigned576(
            maximum,
            boxRadius);
        if (pushBoxNegative.Sign < 0 || pushBoxPositive.Sign < 0)
            return false;

        bool negate =
            CompareSigned(pushBoxPositive, pushBoxNegative) < 0;
        Signed576 overlap = negate
            ? pushBoxPositive
            : pushBoxNegative;
        Signed320 commonDenominator = WideArithmetic.MultiplySigned192(
            boxBasis.Denominator,
            triangleBasis.Denominator);
        Signed576 squaredAxisLength = GetSquaredLength(axis);
        if (ShouldReplacePointSpan(
            overlap,
            squaredAxisLength,
            commonDenominator,
            best))
        {
            best = new PointSpanPenetration(
                axis,
                negate,
                overlap,
                squaredAxisLength,
                commonDenominator);
        }
        return true;
    }

    private static Vector3d GetTriangleSupportLocalPoint(
        FixedTriangle triangle,
        WideRationalBasis3d triangleBasis,
        WideAxis3 boxToTriangleAxis)
    {
        Vector3d bestPoint = triangle.A;
        Signed576 bestProjection = GetTransformedOffsetProjection(
            bestPoint,
            triangleBasis,
            boxToTriangleAxis);
        KeepTriangleSupportPoint(
            triangle.B,
            triangleBasis,
            boxToTriangleAxis,
            ref bestPoint,
            ref bestProjection);
        KeepTriangleSupportPoint(
            triangle.C,
            triangleBasis,
            boxToTriangleAxis,
            ref bestPoint,
            ref bestProjection);
        return bestPoint;
    }

    private static void KeepTriangleSupportPoint(
        Vector3d candidate,
        WideRationalBasis3d triangleBasis,
        WideAxis3 boxToTriangleAxis,
        ref Vector3d bestPoint,
        ref Signed576 bestProjection)
    {
        Signed576 candidateProjection = GetTransformedOffsetProjection(
            candidate,
            triangleBasis,
            boxToTriangleAxis);
        if (CompareSigned(candidateProjection, bestProjection) >= 0)
            return;

        bestPoint = candidate;
        bestProjection = candidateProjection;
    }

    internal static void GetTriangleFaceContacts(
        Vector3d boxCenter,
        FixedQuaternion boxOrientation,
        Vector3d boxHalfExtents,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        FixedContactAnchors primary,
        Span<FixedContactLocalPoints> contacts,
        out int count)
    {
        count = 0;
        WideRationalBasis3d boxBasis = new(boxOrientation);
        WideRationalBasis3d triangleBasis = new(triangleRotation);
        WideGeometry.GetDifferenceCrossProduct3D(
            triangle.B.X,
            triangle.A.X,
            triangle.B.Y,
            triangle.A.Y,
            triangle.B.Z,
            triangle.A.Z,
            triangle.C.X,
            triangle.A.X,
            triangle.C.Y,
            triangle.A.Y,
            triangle.C.Z,
            triangle.A.Z,
            out Signed192 normalX,
            out Signed192 normalY,
            out Signed192 normalZ);
        WideAxis3 triangleNormal = TransformLocalAxis(
            triangleBasis,
            normalX,
            normalY,
            normalZ);
        int faceAxis = GetParallelTriangleFaceAxis(
            boxBasis,
            triangleNormal,
            primary.Normal);
        if (faceAxis < 0)
            return;

        WideAxis3 selectedAxis = GetBasisAxis(boxBasis, faceAxis);
        Fixed64 faceSign = GetDirectionProjection(
                selectedAxis,
                primary.Normal).Sign < 0
            ? -Fixed64.One
            : Fixed64.One;
        for (int corner = 0; corner < 4; corner++)
        {
            Fixed64 firstSign = (corner & 1) == 0
                ? -Fixed64.One
                : Fixed64.One;
            Fixed64 secondSign = (corner & 2) == 0
                ? -Fixed64.One
                : Fixed64.One;
            Vector3d boxLocalPoint = faceAxis switch
            {
                0 => new Vector3d(
                    faceSign * boxHalfExtents.X,
                    firstSign * boxHalfExtents.Y,
                    secondSign * boxHalfExtents.Z),
                1 => new Vector3d(
                    firstSign * boxHalfExtents.X,
                    faceSign * boxHalfExtents.Y,
                    secondSign * boxHalfExtents.Z),
                _ => new Vector3d(
                    firstSign * boxHalfExtents.X,
                    secondSign * boxHalfExtents.Y,
                    faceSign * boxHalfExtents.Z),
            };
            var boxPoint = new FixedPointAnchor(
                boxCenter,
                boxOrientation,
                boxLocalPoint);
            if (!boxPoint.TryGetLocalPointIn(
                    triangleOrigin,
                    triangleRotation,
                    out Vector3d pointInTriangleFrame)
                || !triangle.ContainsProjection(pointInTriangleFrame))
            {
                continue;
            }

            contacts[count++] = new FixedContactLocalPoints(
                boxLocalPoint,
                triangle.ClosestPoint(pointInTriangleFrame));
        }
    }

    private static int GetParallelTriangleFaceAxis(
        WideRationalBasis3d boxBasis,
        WideAxis3 triangleNormal,
        Vector3d primaryNormal)
    {
        int bestIndex = 0;
        WideAxis3 bestAxis = GetBasisAxis(boxBasis, 0);
        Signed576 bestAlignment = GetAxisProjection(
            bestAxis,
            triangleNormal);
        for (int index = 1; index < 3; index++)
        {
            WideAxis3 candidateAxis = GetBasisAxis(boxBasis, index);
            Signed576 candidateAlignment = GetAxisProjection(
                candidateAxis,
                triangleNormal);
            if (CompareSigned(
                    GetMagnitude(candidateAlignment),
                    GetMagnitude(bestAlignment)) <= 0)
            {
                continue;
            }

            bestIndex = index;
            bestAxis = candidateAxis;
            bestAlignment = candidateAlignment;
        }

        Signed576 boxSquared = GetSquaredLength(bestAxis);
        if (!IsParallel(
                bestAlignment,
                boxSquared,
                GetSquaredLength(triangleNormal)))
        {
            return -1;
        }

        WideAxis3 primaryAxis = new(
            Signed320.ExtendValue(Signed192.Raw(primaryNormal.X)),
            Signed320.ExtendValue(Signed192.Raw(primaryNormal.Y)),
            Signed320.ExtendValue(Signed192.Raw(primaryNormal.Z)));
        return IsParallel(
                GetAxisProjection(bestAxis, primaryAxis),
                boxSquared,
                GetSquaredLength(primaryAxis))
            ? bestIndex
            : -1;
    }

    private static Signed576 GetDirectionProjection(
        WideAxis3 axis,
        Vector3d direction) =>
        GetAxisProjection(
            axis,
            new WideAxis3(
                Signed320.ExtendValue(Signed192.Raw(direction.X)),
                Signed320.ExtendValue(Signed192.Raw(direction.Y)),
                Signed320.ExtendValue(Signed192.Raw(direction.Z))));

    private static Signed576 GetAxisProjection(
        WideAxis3 first,
        WideAxis3 second) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(first.X, second.X),
                WideArithmetic.MultiplySigned320(first.Y, second.Y)),
            WideArithmetic.MultiplySigned320(first.Z, second.Z));

    private static Signed576 GetSquaredLength(WideAxis3 axis) =>
        GetAxisProjection(axis, axis);

    private static bool IsParallel(
        Signed576 alignment,
        Signed576 firstSquaredLength,
        Signed576 secondSquaredLength)
    {
        Signed832 alignmentSquared =
            WideArithmetic.MultiplySigned576ToSigned832(
                alignment,
                alignment);
        Signed832 lengthProduct =
            WideArithmetic.MultiplySigned576ToSigned832(
                firstSquaredLength,
                secondSquaredLength);
        Signed320 thresholdSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(ParallelFaceAlignmentThreshold),
            Signed192.Raw(ParallelFaceAlignmentThreshold));
        Signed320 scaleSquared = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.One);
        return WideArithmetic.CompareNonNegativeProducts(
                alignmentSquared,
                Signed832.ExtendValue(
                    Signed576.ExtendValue(scaleSquared)),
                lengthProduct,
                Signed832.ExtendValue(
                    Signed576.ExtendValue(thresholdSquared))) >= 0;
    }

    private static void KeepProjection(
        Signed576 candidate,
        ref Signed576 minimum,
        ref Signed576 maximum)
    {
        if (CompareSigned(candidate, minimum) < 0)
            minimum = candidate;
        if (CompareSigned(candidate, maximum) > 0)
            maximum = candidate;
    }
}
