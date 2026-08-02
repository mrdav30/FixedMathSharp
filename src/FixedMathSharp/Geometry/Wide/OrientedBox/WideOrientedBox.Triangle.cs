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
        WideRationalBasis3d relativeBasis =
            WideRationalBasis3d.CreateRelative(boxBasis, triangleBasis);
        Signed320 unit = Signed320.One;
        Span<WideAxis3> boxAxes = stackalloc WideAxis3[3]
        {
            new(unit, default, default),
            new(default, unit, default),
            new(default, default, unit),
        };
        GetRelativeLocalPointNumerators(
            triangleOrigin,
            boxCenter,
            boxBasis,
            out Signed192 translationX,
            out Signed192 translationY,
            out Signed192 translationZ);
        var triangleTranslation = new WideAxis3(
            WideArithmetic.MultiplySigned192(
                translationX,
                triangleBasis.Denominator),
            WideArithmetic.MultiplySigned192(
                translationY,
                triangleBasis.Denominator),
            WideArithmetic.MultiplySigned192(
                translationZ,
                triangleBasis.Denominator));
        var best = default(WidePointSpanPenetration);
        for (int index = 0; index < boxAxes.Length; index++)
        {
            if (!TryKeepTriangleAxis(
                    boxAxes[index],
                    boxHalfExtents,
                    relativeBasis,
                    triangleTranslation,
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
                WideRigidProjection.TransformLocalAxis(
                    relativeBasis,
                    normalX,
                    normalY,
                    normalZ),
                boxHalfExtents,
                relativeBasis,
                triangleTranslation,
                triangle,
                ref best)
            || !TryKeepTriangleEdgeAxes(
                triangle.A,
                triangle.B,
                boxAxes,
                boxHalfExtents,
                relativeBasis,
                triangleTranslation,
                triangle,
                ref best)
            || !TryKeepTriangleEdgeAxes(
                triangle.B,
                triangle.C,
                boxAxes,
                boxHalfExtents,
                relativeBasis,
                triangleTranslation,
                triangle,
                ref best)
            || !TryKeepTriangleEdgeAxes(
                triangle.C,
                triangle.A,
                boxAxes,
                boxHalfExtents,
                relativeBasis,
                triangleTranslation,
                triangle,
                ref best))
        {
            contact = default;
            return false;
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        WideRigidProjection.TransformLocalAxis(
            boxBasis,
            orientedAxis,
            out Signed576 worldX,
            out Signed576 worldY,
            out Signed576 worldZ);
        Vector3d contactNormal = WideNormalization.GetNormalized(
            worldX,
            worldY,
            worldZ);
        Vector3d triangleLocalPoint = GetTriangleSupportLocalPoint(
            triangle,
            relativeBasis,
            orientedAxis);
        Fixed64 depth = best.GetRoundedDepth(out bool depthIsClamped);
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
        Vector3d boxHalfExtents,
        in WideRationalBasis3d relativeBasis,
        in WideAxis3 triangleTranslation,
        in FixedTriangle triangle,
        ref WidePointSpanPenetration best)
    {
        WideAxis3 edge = WideRigidProjection.TransformLocalAxis(
            relativeBasis,
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
                    WideAxis3.Cross(boxAxes[axisIndex], edge),
                    boxHalfExtents,
                    relativeBasis,
                    triangleTranslation,
                    triangle,
                    ref best))
            {
                return false;
            }
        }
        return true;
    }

    private static bool TryKeepTriangleAxis(
        in WideAxis3 axis,
        Vector3d boxHalfExtents,
        in WideRationalBasis3d relativeBasis,
        in WideAxis3 triangleTranslation,
        in FixedTriangle triangle,
        ref WidePointSpanPenetration best)
    {
        if (axis.IsZero)
            return true;

        WideRigidProjection.GetLocalAxisProjections(
            axis,
            relativeBasis,
            out Signed576 localAxisX,
            out Signed576 localAxisY,
            out Signed576 localAxisZ);
        Signed576 first = WideRigidProjection.GetLocalOffsetProjection(
            triangle.A,
            localAxisX,
            localAxisY,
            localAxisZ);
        Signed576 second = WideRigidProjection.GetLocalOffsetProjection(
            triangle.B,
            localAxisX,
            localAxisY,
            localAxisZ);
        Signed576 third = WideRigidProjection.GetLocalOffsetProjection(
            triangle.C,
            localAxisX,
            localAxisY,
            localAxisZ);
        Signed576 minimum = first;
        Signed576 maximum = first;
        WideRigidProjection.IncludeProjection(second, ref minimum, ref maximum);
        WideRigidProjection.IncludeProjection(third, ref minimum, ref maximum);

        Signed576 originProjection = WideAxis3.Dot(
            axis,
            triangleTranslation);
        minimum = WideArithmetic.AddSigned576(originProjection, minimum);
        maximum = WideArithmetic.AddSigned576(originProjection, maximum);
        Signed576 boxRadius = WideArithmetic.MultiplySigned576(
            GetLocalBoxProjectionRadiusNumerator(axis, boxHalfExtents),
            relativeBasis.Denominator);
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
        Signed320 commonDenominator =
            Signed320.ExtendValue(relativeBasis.Denominator);
        Signed576 squaredAxisLength = axis.SquaredLength;
        if (best.ShouldReplace(
            overlap,
            squaredAxisLength,
            commonDenominator))
        {
            best = new WidePointSpanPenetration(
                axis,
                negate,
                overlap,
                squaredAxisLength,
                commonDenominator);
        }
        return true;
    }

    private static Vector3d GetTriangleSupportLocalPoint(
        in FixedTriangle triangle,
        in WideRationalBasis3d relativeBasis,
        in WideAxis3 boxLocalAxis)
    {
        WideRigidProjection.GetLocalAxisProjections(
            boxLocalAxis,
            relativeBasis,
            out Signed576 localAxisX,
            out Signed576 localAxisY,
            out Signed576 localAxisZ);
        Vector3d bestPoint = triangle.A;
        Signed576 bestProjection = WideRigidProjection.GetLocalOffsetProjection(
            bestPoint,
            localAxisX,
            localAxisY,
            localAxisZ);
        Signed576 secondProjection = WideRigidProjection.GetLocalOffsetProjection(
            triangle.B,
            localAxisX,
            localAxisY,
            localAxisZ);
        if (CompareSigned(secondProjection, bestProjection) < 0)
        {
            bestPoint = triangle.B;
            bestProjection = secondProjection;
        }
        Signed576 thirdProjection = WideRigidProjection.GetLocalOffsetProjection(
            triangle.C,
            localAxisX,
            localAxisY,
            localAxisZ);
        if (CompareSigned(thirdProjection, bestProjection) < 0)
            bestPoint = triangle.C;
        return bestPoint;
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
        WideAxis3 triangleNormal = WideRigidProjection.TransformLocalAxis(
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

        WideAxis3 selectedAxis = boxBasis.GetAxis(faceAxis);
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
        WideAxis3 bestAxis = boxBasis.GetAxis(0);
        Signed576 bestAlignment = WideAxis3.Dot(
            bestAxis,
            triangleNormal);
        for (int index = 1; index < 3; index++)
        {
            WideAxis3 candidateAxis = boxBasis.GetAxis(index);
            Signed576 candidateAlignment = WideAxis3.Dot(
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

        Signed576 boxSquared = bestAxis.SquaredLength;
        if (!IsParallel(
                bestAlignment,
                boxSquared,
                triangleNormal.SquaredLength))
        {
            return -1;
        }

        WideAxis3 primaryAxis = new(
            Signed320.ExtendValue(Signed192.Raw(primaryNormal.X)),
            Signed320.ExtendValue(Signed192.Raw(primaryNormal.Y)),
            Signed320.ExtendValue(Signed192.Raw(primaryNormal.Z)));
        return IsParallel(
                WideAxis3.Dot(bestAxis, primaryAxis),
                boxSquared,
                primaryAxis.SquaredLength)
            ? bestIndex
            : -1;
    }

    private static Signed576 GetDirectionProjection(
        in WideAxis3 axis,
        Vector3d direction) =>
        WideAxis3.Dot(
            axis,
            new WideAxis3(
                Signed320.ExtendValue(Signed192.Raw(direction.X)),
                Signed320.ExtendValue(Signed192.Raw(direction.Y)),
                Signed320.ExtendValue(Signed192.Raw(direction.Z))));

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

}
