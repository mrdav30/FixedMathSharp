//=======================================================================
// WideOrientedBox.TriangleCircleSlabSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Computes swept-circle vs. triangle intersection within a bounded slab region,
/// using high-precision wide arithmetic for exact edge and interior overlap tests.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct TriangleSweepVertex
    {
        internal readonly Signed320 X;
        internal readonly Signed320 Y;
        internal readonly Signed320 Z;
        internal readonly Signed320 ReferenceYOffset;
        internal readonly Vector3d LocalPoint;

        internal TriangleSweepVertex(
            Signed320 x,
            Signed320 y,
            Signed320 z,
            Signed320 referenceYOffset,
            Vector3d localPoint)
        {
            X = x;
            Y = y;
            Z = z;
            ReferenceYOffset = referenceYOffset;
            LocalPoint = localPoint;
        }

        internal TriangleSweepPoint ToSweepPoint(
            Signed192 basisDenominator) =>
            new(
                new SweepRationalPoint(
                    Signed576.ExtendValue(X),
                    Signed576.ExtendValue(Z),
                    Signed576.ExtendValue(
                        Signed320.ExtendValue(
                            basisDenominator))),
                ReferenceYOffset,
                LocalPoint);

    }

    private readonly struct TriangleSweepPoint
    {
        internal readonly SweepRationalPoint RationalPoint;
        internal readonly Signed320 ReferenceYOffset;
        internal readonly Vector3d LocalPoint;

        internal TriangleSweepPoint(
            SweepRationalPoint rationalPoint,
            Signed320 referenceYOffset,
            Vector3d localPoint)
        {
            RationalPoint = rationalPoint;
            ReferenceYOffset = referenceYOffset;
            LocalPoint = localPoint;
        }
    }

    #endregion

    internal static bool TryGetFiniteSlabProjectedCircleSweep(
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector2d circleStart,
        Vector2d direction,
        Fixed64 maximumDistance,
        Fixed64 circleRadius,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        out Fixed64 distance,
        out FixedPointAnchor triangleContact)
    {
        WideRationalBasis3d basis = new(triangleRotation);
        Signed320 referenceYNumerator =
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(slabCenterY),
                basis.Denominator);
        TriangleSweepVertex first = GetTriangleSweepVertex(
            triangleOrigin,
            basis,
            triangle.A,
            referenceYNumerator);
        TriangleSweepVertex second = GetTriangleSweepVertex(
            triangleOrigin,
            basis,
            triangle.B,
            referenceYNumerator);
        TriangleSweepVertex third = GetTriangleSweepVertex(
            triangleOrigin,
            basis,
            triangle.C,
            referenceYNumerator);
        Signed192 lowerY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(slabCenterY),
            Signed192.Raw(slabHalfThickness));
        Signed192 upperY = WideArithmetic.AddSigned192(
            Signed192.Raw(slabCenterY),
            Signed192.Raw(slabHalfThickness));
        Signed320 lowerNumerator =
            WideArithmetic.MultiplySigned192(lowerY, basis.Denominator);
        Signed320 upperNumerator =
            WideArithmetic.MultiplySigned192(upperY, basis.Denominator);
        if (TryGetTriangleProjectionInteriorPoint(
                first,
                second,
                third,
                circleStart,
                lowerNumerator,
                upperNumerator,
                basis.Denominator,
                out Vector3d interiorLocalPoint))
        {
            distance = Fixed64.Zero;
            triangleContact = new FixedPointAnchor(
                triangleOrigin,
                triangleRotation,
                interiorLocalPoint);
            return true;
        }

        bool found = false;
        Fixed64 bestDistance = default;
        Vector3d bestLocalPoint = default;
        Signed576 bestReferenceYOffset = default;
        bool firstSecondCollapsed =
            AreSameProjectedPoint(first, second);
        bool secondThirdCollapsed =
            AreSameProjectedPoint(second, third);
        bool thirdFirstCollapsed =
            AreSameProjectedPoint(third, first);
        bool allCollapsed =
            firstSecondCollapsed
            && secondThirdCollapsed
            && thirdFirstCollapsed;
        if (allCollapsed || !firstSecondCollapsed)
        {
            KeepTriangleEdgeSweep(
                first,
                second,
                lowerNumerator,
                upperNumerator,
                basis.Denominator,
                referenceYNumerator,
                circleStart,
                direction,
                maximumDistance,
                circleRadius,
                ref found,
                ref bestDistance,
                ref bestLocalPoint,
                ref bestReferenceYOffset);
        }
        if (allCollapsed || !secondThirdCollapsed)
        {
            KeepTriangleEdgeSweep(
                second,
                third,
                lowerNumerator,
                upperNumerator,
                basis.Denominator,
                referenceYNumerator,
                circleStart,
                direction,
                maximumDistance,
                circleRadius,
                ref found,
                ref bestDistance,
                ref bestLocalPoint,
                ref bestReferenceYOffset);
        }
        if (allCollapsed || !thirdFirstCollapsed)
        {
            KeepTriangleEdgeSweep(
                third,
                first,
                lowerNumerator,
                upperNumerator,
                basis.Denominator,
                referenceYNumerator,
                circleStart,
                direction,
                maximumDistance,
                circleRadius,
                ref found,
                ref bestDistance,
                ref bestLocalPoint,
                ref bestReferenceYOffset);
        }
        KeepTrianglePlaneEdgeSweep(
            first,
            second,
            third,
            lowerNumerator,
            basis.Denominator,
            referenceYNumerator,
            circleStart,
            direction,
            maximumDistance,
            circleRadius,
            ref found,
            ref bestDistance,
            ref bestLocalPoint,
            ref bestReferenceYOffset);
        if (CompareSigned(lowerNumerator, upperNumerator) != 0)
        {
            KeepTrianglePlaneEdgeSweep(
                first,
                second,
                third,
                upperNumerator,
                basis.Denominator,
                referenceYNumerator,
                circleStart,
                direction,
                maximumDistance,
                circleRadius,
                ref found,
                ref bestDistance,
                ref bestLocalPoint,
                ref bestReferenceYOffset);
        }

        if (!found)
        {
            distance = default;
            triangleContact = default;
            return false;
        }

        distance = bestDistance;
        triangleContact = new FixedPointAnchor(
            triangleOrigin,
            triangleRotation,
            bestLocalPoint);
        return true;
    }

    private static bool TryGetTriangleProjectionInteriorPoint(
        TriangleSweepVertex first,
        TriangleSweepVertex second,
        TriangleSweepVertex third,
        Vector2d point,
        Signed320 lowerY,
        Signed320 upperY,
        Signed192 basisDenominator,
        out Vector3d localPoint)
    {
        Signed320 firstEdgeX =
            WideArithmetic.SubtractSigned320(second.X, first.X);
        Signed320 firstEdgeZ =
            WideArithmetic.SubtractSigned320(second.Z, first.Z);
        Signed320 secondEdgeX =
            WideArithmetic.SubtractSigned320(third.X, first.X);
        Signed320 secondEdgeZ =
            WideArithmetic.SubtractSigned320(third.Z, first.Z);
        Signed576 determinant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                firstEdgeX,
                secondEdgeZ),
            WideArithmetic.MultiplySigned320(
                firstEdgeZ,
                secondEdgeX));
        if (determinant.IsZero)
        {
            localPoint = default;
            return false;
        }

        Signed320 pointX = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(point.X),
                basisDenominator),
            first.X);
        Signed320 pointZ = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(point.Y),
                basisDenominator),
            first.Z);
        Signed576 firstParameter = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                pointX,
                secondEdgeZ),
            WideArithmetic.MultiplySigned320(
                pointZ,
                secondEdgeX));
        Signed576 secondParameter = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                firstEdgeX,
                pointZ),
            WideArithmetic.MultiplySigned320(
                firstEdgeZ,
                pointX));
        if (determinant.Sign < 0)
        {
            determinant =
                WideArithmetic.SubtractSigned576(default, determinant);
            firstParameter =
                WideArithmetic.SubtractSigned576(default, firstParameter);
            secondParameter =
                WideArithmetic.SubtractSigned576(default, secondParameter);
        }

        if (firstParameter.Sign < 0
            || secondParameter.Sign < 0
            || WideArithmetic.SubtractSigned576(
                WideArithmetic.AddSigned576(
                    firstParameter,
                    secondParameter),
                determinant).Sign > 0)
        {
            localPoint = default;
            return false;
        }

        Signed320 firstEdgeY =
            WideArithmetic.SubtractSigned320(second.Y, first.Y);
        Signed320 secondEdgeY =
            WideArithmetic.SubtractSigned320(third.Y, first.Y);
        Signed832 yNumerator = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(
                    Signed576.ExtendValue(first.Y),
                    determinant),
                WideArithmetic.MultiplySigned576ToSigned832(
                    Signed576.ExtendValue(firstEdgeY),
                    firstParameter)),
            WideArithmetic.MultiplySigned576ToSigned832(
                Signed576.ExtendValue(secondEdgeY),
                secondParameter));
        Signed832 lowerNumerator =
            WideArithmetic.MultiplySigned576ToSigned832(
                Signed576.ExtendValue(lowerY),
                determinant);
        Signed832 upperNumerator =
            WideArithmetic.MultiplySigned576ToSigned832(
                Signed576.ExtendValue(upperY),
                determinant);
        if (WideArithmetic.SubtractSigned832(
                yNumerator,
                lowerNumerator).Sign < 0
            || WideArithmetic.SubtractSigned832(
                yNumerator,
                upperNumerator).Sign > 0)
        {
            localPoint = default;
            return false;
        }

        Signed320 firstLocalX =
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                Signed192.Raw(second.LocalPoint.X),
                Signed192.Raw(first.LocalPoint.X)));
        Signed320 firstLocalY =
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                Signed192.Raw(second.LocalPoint.Y),
                Signed192.Raw(first.LocalPoint.Y)));
        Signed320 firstLocalZ =
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                Signed192.Raw(second.LocalPoint.Z),
                Signed192.Raw(first.LocalPoint.Z)));
        Signed320 secondLocalX =
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                Signed192.Raw(third.LocalPoint.X),
                Signed192.Raw(first.LocalPoint.X)));
        Signed320 secondLocalY =
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                Signed192.Raw(third.LocalPoint.Y),
                Signed192.Raw(first.LocalPoint.Y)));
        Signed320 secondLocalZ =
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                Signed192.Raw(third.LocalPoint.Z),
                Signed192.Raw(first.LocalPoint.Z)));
        Signed832 denominator =
            Signed832.ExtendValue(determinant);
        // A barycentric point lies inside the convex hull of three
        // representable local vertices, so every final local component is
        // representable by construction.
        _ = TryGetTriangleInteriorLocalCoordinate(
                first.LocalPoint.X,
                firstLocalX,
                secondLocalX,
                firstParameter,
                secondParameter,
                determinant,
                denominator,
                out Fixed64 localX);
        _ = TryGetTriangleInteriorLocalCoordinate(
                first.LocalPoint.Y,
                firstLocalY,
                secondLocalY,
                firstParameter,
                secondParameter,
                determinant,
                denominator,
                out Fixed64 localY);
        _ = TryGetTriangleInteriorLocalCoordinate(
                first.LocalPoint.Z,
                firstLocalZ,
                secondLocalZ,
                firstParameter,
                secondParameter,
                determinant,
                denominator,
                out Fixed64 localZ);
        localPoint = new Vector3d(localX, localY, localZ);
        return true;
    }

    private static bool TryGetTriangleInteriorLocalCoordinate(
        Fixed64 origin,
        Signed320 firstEdge,
        Signed320 secondEdge,
        Signed576 firstParameter,
        Signed576 secondParameter,
        Signed576 determinant,
        Signed832 denominator,
        out Fixed64 coordinate)
    {
        Signed832 numerator = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(
                    Signed576.ExtendValue(
                        Signed320.ExtendValue(Signed192.Raw(origin))),
                    determinant),
                WideArithmetic.MultiplySigned576ToSigned832(
                    Signed576.ExtendValue(firstEdge),
                    firstParameter)),
            WideArithmetic.MultiplySigned576ToSigned832(
                Signed576.ExtendValue(secondEdge),
                secondParameter));
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            numeratorLeftShift: 0,
            out coordinate);
    }

    private static TriangleSweepVertex GetTriangleSweepVertex(
        Vector3d origin,
        WideRationalBasis3d basis,
        Vector3d localPoint,
        Signed320 referenceYNumerator)
    {
        Signed320 x = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(origin.X),
                basis.Denominator),
            GetLocalOffsetNumerator(
                basis.Xx,
                basis.Yx,
                basis.Zx,
                localPoint));
        Signed320 y = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(origin.Y),
                basis.Denominator),
            GetLocalOffsetNumerator(
                basis.Xy,
                basis.Yy,
                basis.Zy,
                localPoint));
        Signed320 z = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(origin.Z),
                basis.Denominator),
            GetLocalOffsetNumerator(
                basis.Xz,
                basis.Yz,
                basis.Zz,
                localPoint));
        return new TriangleSweepVertex(
            x,
            y,
            z,
            WideArithmetic.SubtractSigned320(
                y,
                referenceYNumerator),
            localPoint);
    }

    private static bool AreSameProjectedPoint(
        TriangleSweepVertex first,
        TriangleSweepVertex second) =>
        CompareSigned(first.X, second.X) == 0
        && CompareSigned(first.Z, second.Z) == 0;

    private static void KeepTriangleEdgeSweep(
        TriangleSweepVertex first,
        TriangleSweepVertex second,
        Signed320 lowerY,
        Signed320 upperY,
        Signed192 basisDenominator,
        Signed320 referenceYNumerator,
        Vector2d circleStart,
        Vector2d direction,
        Fixed64 maximumDistance,
        Fixed64 radius,
        ref bool found,
        ref Fixed64 bestDistance,
        ref Vector3d bestLocalPoint,
        ref Signed576 bestReferenceYOffset)
    {
        Signed320 minimum = CompareSigned(first.Y, second.Y) <= 0
            ? first.Y
            : second.Y;
        Signed320 maximum = CompareSigned(first.Y, second.Y) <= 0
            ? second.Y
            : first.Y;
        if (CompareSigned(maximum, lowerY) < 0
            || CompareSigned(minimum, upperY) > 0)
        {
            return;
        }

        TriangleSweepPoint clippedFirst =
            GetClippedTriangleEdgeEndpoint(
                first,
                second,
                lowerY,
                upperY,
                basisDenominator,
                referenceYNumerator);
        TriangleSweepPoint clippedSecond =
            GetClippedTriangleEdgeEndpoint(
                second,
                first,
                lowerY,
                upperY,
                basisDenominator,
                referenceYNumerator);
        KeepTriangleSweepFeature(
            clippedFirst,
            clippedSecond,
            circleStart,
            direction,
            maximumDistance,
            radius,
            ref found,
            ref bestDistance,
            ref bestLocalPoint,
            ref bestReferenceYOffset);
    }

    private static TriangleSweepPoint GetClippedTriangleEdgeEndpoint(
        TriangleSweepVertex endpoint,
        TriangleSweepVertex other,
        Signed320 lowerY,
        Signed320 upperY,
        Signed192 basisDenominator,
        Signed320 referenceYNumerator)
    {
        if (CompareSigned(endpoint.Y, lowerY) >= 0
            && CompareSigned(endpoint.Y, upperY) <= 0)
        {
            return endpoint.ToSweepPoint(basisDenominator);
        }

        Signed320 plane = CompareSigned(endpoint.Y, lowerY) < 0
            ? lowerY
            : upperY;
        return GetTrianglePlaneIntersection(
            endpoint,
            other,
            plane,
            basisDenominator,
            referenceYNumerator);
    }

    private static void KeepTrianglePlaneEdgeSweep(
        TriangleSweepVertex first,
        TriangleSweepVertex second,
        TriangleSweepVertex third,
        Signed320 planeY,
        Signed192 basisDenominator,
        Signed320 referenceYNumerator,
        Vector2d circleStart,
        Vector2d direction,
        Fixed64 maximumDistance,
        Fixed64 radius,
        ref bool found,
        ref Fixed64 bestDistance,
        ref Vector3d bestLocalPoint,
        ref Signed576 bestReferenceYOffset)
    {
        Span<TriangleSweepPoint> intersections =
            stackalloc TriangleSweepPoint[3];
        int count = 0;
        TryAddTrianglePlaneIntersection(
            first,
            second,
            planeY,
            basisDenominator,
            referenceYNumerator,
            intersections,
            ref count);
        TryAddTrianglePlaneIntersection(
            second,
            third,
            planeY,
            basisDenominator,
            referenceYNumerator,
            intersections,
            ref count);
        TryAddTrianglePlaneIntersection(
            third,
            first,
            planeY,
            basisDenominator,
            referenceYNumerator,
            intersections,
            ref count);
        if (count < 2)
            return;

        KeepTriangleSweepFeature(
            intersections[0],
            intersections[1],
            circleStart,
            direction,
            maximumDistance,
            radius,
            ref found,
            ref bestDistance,
            ref bestLocalPoint,
            ref bestReferenceYOffset);
    }

    private static void TryAddTrianglePlaneIntersection(
        TriangleSweepVertex first,
        TriangleSweepVertex second,
        Signed320 planeY,
        Signed192 basisDenominator,
        Signed320 referenceYNumerator,
        Span<TriangleSweepPoint> intersections,
        ref int count)
    {
        int firstSide = CompareSigned(first.Y, planeY);
        int secondSide = CompareSigned(second.Y, planeY);
        if (firstSide == 0)
        {
            AddUniqueTriangleSweepPoint(
                first.ToSweepPoint(basisDenominator),
                intersections,
                ref count);
        }
        if (secondSide == 0)
        {
            AddUniqueTriangleSweepPoint(
                second.ToSweepPoint(basisDenominator),
                intersections,
                ref count);
        }
        if (firstSide == 0
            || secondSide == 0
            || (firstSide < 0) == (secondSide < 0))
        {
            return;
        }

        AddUniqueTriangleSweepPoint(
            GetTrianglePlaneIntersection(
                first,
                second,
                planeY,
                basisDenominator,
                referenceYNumerator),
            intersections,
            ref count);
    }

    private static TriangleSweepPoint GetTrianglePlaneIntersection(
        TriangleSweepVertex first,
        TriangleSweepVertex second,
        Signed320 planeY,
        Signed192 basisDenominator,
        Signed320 referenceYNumerator)
    {
        Signed320 deltaY =
            WideArithmetic.SubtractSigned320(second.Y, first.Y);
        Signed320 firstWeight =
            WideArithmetic.SubtractSigned320(second.Y, planeY);
        Signed320 secondWeight =
            WideArithmetic.SubtractSigned320(planeY, first.Y);
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(first.X, firstWeight),
            WideArithmetic.MultiplySigned320(second.X, secondWeight));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(first.Z, firstWeight),
            WideArithmetic.MultiplySigned320(second.Z, secondWeight));
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(basisDenominator),
            deltaY);
        if (denominator.Sign < 0)
        {
            x = WideArithmetic.SubtractSigned576(default, x);
            z = WideArithmetic.SubtractSigned576(default, z);
            denominator =
                WideArithmetic.SubtractSigned576(default, denominator);
            firstWeight =
                WideArithmetic.SubtractSigned320(default, firstWeight);
            secondWeight =
                WideArithmetic.SubtractSigned320(default, secondWeight);
            deltaY = WideArithmetic.SubtractSigned320(default, deltaY);
        }

        Vector3d localPoint = GetWeightedLocalPoint(
            first.LocalPoint,
            second.LocalPoint,
            secondWeight,
            deltaY);
        return new TriangleSweepPoint(
            new SweepRationalPoint(x, z, denominator),
            WideArithmetic.SubtractSigned320(
                planeY,
                referenceYNumerator),
            localPoint);
    }

    private static Vector3d GetWeightedLocalPoint(
        Vector3d first,
        Vector3d second,
        Signed320 secondWeight,
        Signed320 denominator)
    {
        Span<Vector3d> points = stackalloc Vector3d[2]
        {
            first,
            second,
        };
        // This ratio is a dimensionless interpolation parameter, so promote
        // it to Q32.32 rather than treating the integer quotient as raw.
        _ = Fixed64.TryGetSignedRawRatio(
            Signed832.ExtendValue(
                Signed576.ExtendValue(secondWeight)),
            Signed832.ExtendValue(
                Signed576.ExtendValue(denominator)),
            FixedMath.SHIFT_AMOUNT_I,
            out Fixed64 parameter);
        Span<Fixed64> weights = stackalloc Fixed64[2]
        {
            Fixed64.One - parameter,
            parameter,
        };
        _ = Vector3d.TryGetWeightedAverage(
            points,
            weights,
            out Vector3d localPoint);
        return localPoint;
    }

    private static void AddUniqueTriangleSweepPoint(
        TriangleSweepPoint point,
        Span<TriangleSweepPoint> points,
        ref int count)
    {
        for (int index = 0; index < count; index++)
        {
            if (AreSameSweepPoint(
                    points[index].RationalPoint,
                    point.RationalPoint))
            {
                return;
            }
        }

        points[count++] = point;
    }

    private static void KeepTriangleSweepFeature(
        TriangleSweepPoint first,
        TriangleSweepPoint second,
        Vector2d circleStart,
        Vector2d direction,
        Fixed64 maximumDistance,
        Fixed64 radius,
        ref bool found,
        ref Fixed64 bestDistance,
        ref Vector3d bestLocalPoint,
        ref Signed576 bestReferenceYOffset)
    {
        if (TryGetRationalPointSweepDistance(
                first.RationalPoint,
                circleStart,
                direction,
                maximumDistance,
                radius,
                out Fixed64 firstDistance))
        {
            KeepTriangleSweepCandidate(
                firstDistance,
                first.LocalPoint,
                ScaleReferenceYOffset(first.ReferenceYOffset),
                ref found,
                ref bestDistance,
                ref bestLocalPoint,
                ref bestReferenceYOffset);
        }
        if (!AreSameSweepPoint(
                first.RationalPoint,
                second.RationalPoint)
            && TryGetRationalPointSweepDistance(
                second.RationalPoint,
                circleStart,
                direction,
                maximumDistance,
                radius,
                out Fixed64 secondDistance))
        {
            KeepTriangleSweepCandidate(
                secondDistance,
                second.LocalPoint,
                ScaleReferenceYOffset(second.ReferenceYOffset),
                ref found,
                ref bestDistance,
                ref bestLocalPoint,
                ref bestReferenceYOffset);
        }
        if (!TryGetRationalSegmentSweepDistance(
                first.RationalPoint,
                second.RationalPoint,
                circleStart,
                direction,
                maximumDistance,
                radius,
                out Fixed64 segmentDistance,
                out Fixed64 segmentParameter))
        {
            return;
        }

        Span<Vector3d> points = stackalloc Vector3d[2]
        {
            first.LocalPoint,
            second.LocalPoint,
        };
        Span<Fixed64> weights = stackalloc Fixed64[2]
        {
            Fixed64.One - segmentParameter,
            segmentParameter,
        };
        _ = Vector3d.TryGetWeightedAverage(
            points,
            weights,
            out Vector3d segmentLocalPoint);
        Signed576 segmentReferenceYOffset =
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    first.ReferenceYOffset,
                    Signed320.ExtendValue(
                        Signed192.Raw(weights[0]))),
                WideArithmetic.MultiplySigned320(
                    second.ReferenceYOffset,
                    Signed320.ExtendValue(
                        Signed192.Raw(weights[1]))));
        KeepTriangleSweepCandidate(
            segmentDistance,
            segmentLocalPoint,
            segmentReferenceYOffset,
            ref found,
            ref bestDistance,
            ref bestLocalPoint,
            ref bestReferenceYOffset);
    }

    private static void KeepTriangleSweepCandidate(
        Fixed64 candidateDistance,
        Vector3d candidateLocalPoint,
        Signed576 candidateReferenceYOffset,
        ref bool found,
        ref Fixed64 bestDistance,
        ref Vector3d bestLocalPoint,
        ref Signed576 bestReferenceYOffset)
    {
        if (found)
        {
            if (candidateDistance > bestDistance)
                return;
            if ((candidateDistance == bestDistance)
                & (WideArithmetic.CompareNonNegative(
                    GetAbsoluteReferenceYOffset(candidateReferenceYOffset),
                    GetAbsoluteReferenceYOffset(bestReferenceYOffset)) >= 0))
            {
                return;
            }
        }

        found = true;
        bestDistance = candidateDistance;
        bestLocalPoint = candidateLocalPoint;
        bestReferenceYOffset = candidateReferenceYOffset;
    }

    private static Signed576 ScaleReferenceYOffset(
        Signed320 value) =>
        WideArithmetic.MultiplySigned320(
            value,
            Signed320.ExtendValue(
                Signed192.One));

    private static Signed576 GetAbsoluteReferenceYOffset(Signed576 value) =>
        value.Sign < 0
            ? WideArithmetic.SubtractSigned576(default, value)
            : value;
}
