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
        => TryGetFiniteSlabProjectedCircleSweepCore(
            triangle, triangleOrigin, triangleRotation, circleStart, new SweepTrajectory(direction),
            maximumDistance, circleRadius, slabCenterY, slabHalfThickness, out distance, out triangleContact);

    internal static bool TryGetFiniteSlabProjectedCircleSweep(
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedSegment2d segment,
        Fixed64 totalDistance,
        Fixed64 circleRadius,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        out Fixed64 distance,
        out FixedPointAnchor triangleContact)
        => TryGetFiniteSlabProjectedCircleSweepCore(
            triangle, triangleOrigin, triangleRotation, segment.Start, new SweepTrajectory(segment, totalDistance),
            totalDistance, circleRadius, slabCenterY, slabHalfThickness, out distance, out triangleContact);

    private static bool TryGetFiniteSlabProjectedCircleSweepCore(
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector2d circleStart,
        in SweepTrajectory trajectory,
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
                trajectory,
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
                trajectory,
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
                trajectory,
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
            trajectory,
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
                trajectory,
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
        in SweepTrajectory trajectory,
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
            trajectory,
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
        in SweepTrajectory trajectory,
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
            trajectory,
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
        in SweepTrajectory trajectory,
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
                trajectory,
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
                trajectory,
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
                first,
                second,
                circleStart,
                trajectory,
                maximumDistance,
                radius,
                out Fixed64 segmentDistance,
                out Vector3d segmentLocalPoint,
                out Signed576 segmentReferenceYOffset))
        {
            return;
        }

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
    // Exact affine geometry retains the authored frame, while final witnesses
    // use a caller-supplied nearby origin rather than a potentially distant host.
    internal static bool TryGetFiniteSlabProjectedCircleSweep(in FixedTriangle triangle,
        in ScaledRigidFrame3d frame, Vector3d materializationOrigin, in FixedSegment2d segment, Fixed64 length, Fixed64 radius,
        Fixed64 slabCenterY, Fixed64 halfThickness, out Fixed64 distance, out FixedPointAnchor contact)
    {
        if (frame.IsRigid)
            return TryGetFiniteSlabProjectedCircleSweep(triangle, frame.RigidOrigin, frame.Rotation,
                segment, length, radius, slabCenterY, halfThickness, out distance, out contact);
        Vector3d origin = new(segment.Start.X, slabCenterY, segment.Start.Y);
        Span<WideDirection3d> points = stackalloc WideDirection3d[3];
        Signed320 denominator = frame.Denominator;
        for (int i = 0; i < 3; i++)
        {
            frame.GetPoint(i == 0 ? triangle.A : i == 1 ? triangle.B : triangle.C)
                .GetLocalPointNumerators(origin, FixedQuaternion.Identity,
                    out Signed576 x, out Signed576 y, out Signed576 z, out denominator);
            points[i] = new(x, y, z);
        }
        // All point coordinates share one denominator. Remove only their
        // exact common integer factor before forming quadratic products.
        Span<Signed576> reduced = stackalloc Signed576[10];
        for (int i = 0; i < 3; i++) { reduced[3 * i] = points[i].X; reduced[3 * i + 1] = points[i].Y; reduced[3 * i + 2] = points[i].Z; }
        reduced[9] = Signed576.ExtendValue(denominator); WideArithmetic.ReduceCommonScale(reduced);
        denominator = Signed320.NarrowValue(reduced[9]);
        for (int i = 0; i < 3; i++) points[i] = new(reduced[3 * i], reduced[3 * i + 1], reduced[3 * i + 2]);
        if (TryGetAffineSlabSweepInterior(points, denominator, halfThickness, origin,
            materializationOrigin, out contact)) { distance = Fixed64.Zero; return true; }
        Signed192 dx = WideArithmetic.Difference(segment.End.X, segment.Start.X),
            dz = WideArithmetic.Difference(segment.End.Y, segment.Start.Y);
        if (length == Fixed64.Zero) dx = dz = default;
        // Admitted local vertices/placements retain <262 coordinate bits and
        // <400-bit wall coefficients. Clipped point coefficients are <525 bits;
        // edge/face roots and finite witnesses remain <2700 coefficient bits.
        // Keep these banks only on the affine path. The ordinary owner retains
        // its original fixed-width points, trajectory and compact root banks.
        const int words = 64;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        Span<ulong> bestRoot = stackalloc ulong[2 * words];
        bool found = false;
        WideDirection3d localNormal = WideTriangleRelations.GetScaledTriangleLocalNormal(triangle, frame);
        WideDirection3d normal = ReduceAffineSlabDirection(localNormal.Transform(frame.GeometryBasis));
        Span<WideDirection3d> walls = stackalloc WideDirection3d[3];
        for (int i = 0; i < 3; i++)
        {
            FixedSegment edge = triangle.GetEdge(i);
            walls[i] = ReduceAffineSlabDirection(WideDirection3d.Cross(localNormal,
                WideDirection3d.GetLocalEdge(frame, edge.Start, edge.End)).Transform(frame.GeometryBasis));
        }
        for (int feature = 0; feature < 14; feature++)
        {
            KeepAffineSlabSweepFeature(points, denominator, normal, walls, dx, dz, radius,
                halfThickness, feature, values, signs, bestRoot, ref found);
            // Zero time and zero height offset are both global lower bounds.
            if (found && AffineSlabField(values, signs, 0).Sign(bestRoot) == 0
                && AffineSlabField(values, signs, 3).Sign(bestRoot) == 0) break;
        }
        distance = default; contact = default;
        if (!found) return false;
        var n = AffineSlabField(values, signs, 0); var d = AffineSlabField(values, signs, 1);
        Span<ulong> workValues = stackalloc ulong[2 * words]; Span<int> workSigns = stackalloc int[2];
        var work = AffineSlabField(workValues, workSigns, 0);
        ContactQuadratic.Scale(n, Signed576.ExtendValue(Signed192.Raw(length)), work);
        distance = ContactQuadratic.RoundRatio(work, d, bestRoot, 0, length.m_rawValue);
        if (!QuadraticPointMaterialization.TryGetPointInFrame(origin, FixedQuaternion.Identity,
                AffineSlabField(values, signs, 2), AffineSlabField(values, signs, 3),
                AffineSlabField(values, signs, 4), AffineSlabField(values, signs, 5), bestRoot,
                materializationOrigin, out Vector3d local)) return false;
        contact = new FixedPointAnchor(materializationOrigin, FixedQuaternion.Identity, local);
        return true;
    }

    private static ContactQuadratic AffineSlabField(Span<ulong> values, Span<int> signs, int index) =>
        ContactQuadratic.At(values, signs, index, 64);

    private static bool TryGetAffineSlabSweepInterior(ReadOnlySpan<WideDirection3d> p,
        Signed320 denominator, Fixed64 halfThickness, Vector3d origin, Vector3d anchorOrigin,
        out FixedPointAnchor contact)
    {
        contact = default;
        WideDirection3d e = AffineSlabDifference(p[1], p[0]), f = AffineSlabDifference(p[2], p[0]);
        Signed832 det = AffineSlabCross(e.X, e.Z, f.X, f.Z);
        if (det.IsZero) return false;
        Signed832 n = AffineSlabCross(WideArithmetic.SubtractSigned576(default, p[0].X),
            WideArithmetic.SubtractSigned576(default, p[0].Z), f.X, f.Z),
            m = AffineSlabCross(e.X, e.Z, WideArithmetic.SubtractSigned576(default, p[0].X),
                WideArithmetic.SubtractSigned576(default, p[0].Z));
        if (det.Sign < 0) { det = AffineSlabNegate(det); n = AffineSlabNegate(n); m = AffineSlabNegate(m); }
        if (n.Sign < 0 || m.Sign < 0 || WideArithmetic.SubtractSigned832(WideArithmetic.AddSigned832(n, m), det).Sign > 0)
            return false;
        Span<ulong> values = stackalloc ulong[12 * 64]; Span<int> signs = stackalloc int[12];
        var y = AffineSlabField(values, signs, 0); var d = AffineSlabField(values, signs, 1);
        var term = AffineSlabField(values, signs, 2); var query = AffineSlabField(values, signs, 3);
        var zero = AffineSlabField(values, signs, 4); zero.Clear();
        y.Set(det); ContactQuadratic.Scale(y, p[0].Y, query); query.CopyTo(y);
        term.Set(n); ContactQuadratic.Scale(term, e.Y, query); y.Add(query);
        term.Set(m); ContactQuadratic.Scale(term, f.Y, query); y.Add(query);
        term.Set(det); ContactQuadratic.Scale(term, Signed576.ExtendValue(denominator), d);
        ContactQuadratic.Scale(d, Signed576.ExtendValue(Signed192.Raw(halfThickness)), term);
        term.CopyTo(query); query.Add(y, -1); if (query.Signs[0] < 0) return false;
        term.CopyTo(query); query.Add(y); if (query.Signs[0] < 0) return false;
        if (!QuadraticPointMaterialization.TryGetPointInFrame(origin, FixedQuaternion.Identity,
                zero, y, zero, d, ReadOnlySpan<ulong>.Empty, anchorOrigin, out Vector3d point)) return false;
        contact = new FixedPointAnchor(anchorOrigin, FixedQuaternion.Identity, point); return true;
    }

    private static void KeepAffineSlabSweepFeature(ReadOnlySpan<WideDirection3d> points, Signed320 denominator,
        in WideDirection3d normal, ReadOnlySpan<WideDirection3d> walls, Signed192 dx, Signed192 dz,
        Fixed64 radius, Fixed64 halfThickness, int feature, Span<ulong> bestValues,
        Span<int> bestSigns, Span<ulong> bestRoot, ref bool found)
    {
        const int words = 64;
        Span<ulong> values = stackalloc ulong[48 * words]; Span<int> signs = stackalloc int[48];
        var a = AffineSlabField(values, signs, 0); var b = AffineSlabField(values, signs, 1);
        var c = AffineSlabField(values, signs, 2); var n = AffineSlabField(values, signs, 3);
        var d = AffineSlabField(values, signs, 4); var work = AffineSlabField(values, signs, 5);
        var term = AffineSlabField(values, signs, 6); var temporary = AffineSlabField(values, signs, 7);
        var px = AffineSlabField(values, signs, 8); var py = AffineSlabField(values, signs, 9);
        var pz = AffineSlabField(values, signs, 10); var pd = AffineSlabField(values, signs, 11);
        var vx = AffineSlabField(values, signs, 12); var vz = AffineSlabField(values, signs, 13);
        var tx = AffineSlabField(values, signs, 14); var ty = AffineSlabField(values, signs, 15);
        var tz = AffineSlabField(values, signs, 16); var td = AffineSlabField(values, signs, 17);
        var t = AffineSlabField(values, signs, 18); var metric = AffineSlabField(values, signs, 19);
        WideDirection3d velocity = new(WideArithmetic.MultiplySigned320(denominator, dx), default,
            WideArithmetic.MultiplySigned320(denominator, dz));
        int vertex = feature < 3 ? feature : feature < 6 ? feature - 3 : feature < 12 ? (feature - 6) / 2 : 0;
        WideDirection3d point = points[vertex], edge = AffineSlabDifference(points[(vertex + 1) % 3], point);
        Signed576 cap = WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(halfThickness));
        Signed832 ee = default, ev = default, ep = default, planeK = default, planeV = default;
        bool line = feature is >= 3 and < 6, face = feature >= 12;
        if (line)
        {
            ee = AffineSlabDot(edge, edge); if (ee.IsZero) return;
            ev = AffineSlabDot(edge, velocity); ep = AffineSlabDot(edge, point);
            Signed832 vv = AffineSlabDot(velocity, velocity), vp = AffineSlabDot(velocity, point), pp = AffineSlabDot(point, point);
            Signed576 expanded = WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(radius));
            Signed832 r2 = WideArithmetic.MultiplySigned576ToSigned832(expanded, expanded);
            AffineSlabProduct(ee, vv, work, a); AffineSlabProduct(ev, ev, work, term); a.Add(term, -1);
            AffineSlabProduct(ev, ep, work, b); AffineSlabProduct(ee, vp, work, term); b.Add(term, -1);
            AffineSlabProduct(ee, WideArithmetic.SubtractSigned832(pp, r2), work, c);
            AffineSlabProduct(ep, ep, work, term); c.Add(term, -1);
        }
        else if (face)
        {
            ee = AffineSlabDot(normal, normal); if (ee.IsZero) return;
            Signed576 height = feature == 12 ? WideArithmetic.SubtractSigned576(default, cap) : cap;
            planeK = WideArithmetic.SubtractSigned832(WideDirection3d.Dot(normal, point),
                WideArithmetic.MultiplySigned576ToSigned832(normal.Y, height));
            planeV = AffineSlabDot(normal, velocity);
            Signed576 expanded = WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(radius));
            Signed832 r2 = WideArithmetic.MultiplySigned576ToSigned832(expanded, expanded);
            AffineSlabProduct(planeV, planeV, work, a);
            AffineSlabProduct(planeV, planeK, work, b); b.MultiplySign(-1);
            AffineSlabProduct(planeK, planeK, work, c); AffineSlabProduct(ee, r2, work, term); c.Add(term, -1);
        }
        else
        {
            if (feature < 3)
            {
                if (WideArithmetic.SubtractSigned576(point.Y, cap).Sign > 0 || WideArithmetic.AddSigned576(point.Y, cap).Sign < 0) return;
                px.Set(point.X); py.Set(point.Y); pz.Set(point.Z); pd.Set(Signed576.ExtendValue(denominator));
            }
            else
            {
                Signed576 height = (feature & 1) == 0 ? WideArithmetic.SubtractSigned576(default, cap) : cap;
                Signed576 weight = WideArithmetic.SubtractSigned576(height, point.Y), divisor = edge.Y;
                if (divisor.IsZero) return;
                if (divisor.Sign < 0) { weight = WideArithmetic.SubtractSigned576(default, weight); divisor = WideArithmetic.SubtractSigned576(default, divisor); }
                if (weight.Sign < 0 || WideArithmetic.SubtractSigned576(weight, divisor).Sign > 0) return;
                pd.Set(divisor); ContactQuadratic.Scale(pd, Signed576.ExtendValue(denominator), work); work.CopyTo(pd);
                // Weight/divisor uses the original edge orientation even after
                // its signs are normalized above.
                px.Set(divisor); ContactQuadratic.Scale(px, point.X, work);
                temporary.Set(weight); ContactQuadratic.Scale(temporary, edge.X, term); work.Add(term); work.CopyTo(px);
                py.Set(divisor); ContactQuadratic.Scale(py, height, work); work.CopyTo(py);
                pz.Set(divisor); ContactQuadratic.Scale(pz, point.Z, work);
                temporary.Set(weight); ContactQuadratic.Scale(temporary, edge.Z, term); work.Add(term); work.CopyTo(pz);
            }
            ContactQuadratic.Scale(pd, Signed576.ExtendValue(dx), vx); ContactQuadratic.Scale(pd, Signed576.ExtendValue(dz), vz);
            ContactQuadratic.Multiply(vx, vx, ReadOnlySpan<ulong>.Empty, a);
            ContactQuadratic.Multiply(vz, vz, ReadOnlySpan<ulong>.Empty, term); a.Add(term);
            ContactQuadratic.Multiply(vx, px, ReadOnlySpan<ulong>.Empty, b);
            ContactQuadratic.Multiply(vz, pz, ReadOnlySpan<ulong>.Empty, term); b.Add(term); b.MultiplySign(-1);
            ContactQuadratic.Multiply(px, px, ReadOnlySpan<ulong>.Empty, c);
            ContactQuadratic.Multiply(pz, pz, ReadOnlySpan<ulong>.Empty, term); c.Add(term);
            ContactQuadratic.Multiply(pd, pd, ReadOnlySpan<ulong>.Empty, work);
            ContactQuadratic.Scale(work, Signed576.ExtendValue(Fixed64.GetExactRawProduct(radius, radius)), term); c.Add(term, -1);
        }
        Span<ulong> root = stackalloc ulong[2 * words];
        for (int branch = 0; branch < 3; branch++)
        {
            if (branch == 0)
            { if (c.Signs[0] > 0) continue; n.Clear(); d.Set(Signed576.ExtendValue(Signed192.Signed(1))); root.Clear(); }
            else if (!ContactQuadratic.TryGetParameter(a, b, c, branch == 1 ? -1 : 1, root, n, d)) continue;
            if (n.Sign(root) < 0) continue;
            d.CopyTo(work); work.Add(n, -1); if (work.Sign(root) < 0) continue;
            if (line)
            {
                ContactQuadratic.Scale(n, ev, t); ContactQuadratic.Scale(d, ep, term); t.Add(term, -1);
                if (t.Sign(root) < 0) continue;
                ContactQuadratic.Scale(d, ee, metric); metric.CopyTo(work); work.Add(t, -1); if (work.Sign(root) < 0) continue;
                ContactQuadratic.Scale(d, ee, work); ContactQuadratic.Scale(work, point.Y, ty);
                ContactQuadratic.Scale(t, edge.Y, term); ty.Add(term);
                ContactQuadratic.Scale(metric, cap, work); work.CopyTo(term); term.Add(ty, -1); if (term.Sign(root) < 0) continue;
                work.CopyTo(term); term.Add(ty); if (term.Sign(root) < 0) continue;
                ContactQuadratic.Scale(metric, Signed576.ExtendValue(denominator), td);
                ContactQuadratic.Scale(metric, point.X, tx); ContactQuadratic.Scale(t, edge.X, term); tx.Add(term);
                ContactQuadratic.Scale(metric, point.Z, tz); ContactQuadratic.Scale(t, edge.Z, term); tz.Add(term);
            }
            else if (face)
            {
                ContactQuadratic.Scale(n, planeV, t); ContactQuadratic.Scale(d, planeK, term); t.Add(term, -1);
                metric.Set(ee); ContactQuadratic.Scale(metric, Signed576.ExtendValue(denominator), work);
                ContactQuadratic.Multiply(work, d, root, td);
                ContactQuadratic.Scale(n, Signed576.ExtendValue(dx), temporary); ContactQuadratic.Multiply(temporary, work, root, tx);
                ContactQuadratic.Scale(t, normal.X, term); tx.Add(term, -1);
                ContactQuadratic.Scale(n, Signed576.ExtendValue(dz), temporary); ContactQuadratic.Multiply(temporary, work, root, tz);
                ContactQuadratic.Scale(t, normal.Z, term); tz.Add(term, -1);
                ContactQuadratic.Multiply(metric, d, root, work);
                ContactQuadratic.Scale(work, feature == 12 ? WideArithmetic.SubtractSigned576(default, cap) : cap, ty);
                if (!ContainsAffineSlabSweepFace(walls, points, denominator, tx, ty, tz, td, root, work, term, temporary)) continue;
            }
            else
            {
                ContactQuadratic.Multiply(px, d, root, tx); ContactQuadratic.Multiply(py, d, root, ty);
                ContactQuadratic.Multiply(pz, d, root, tz); ContactQuadratic.Multiply(pd, d, root, td);
            }
            if (found)
            {
                int order = ContactQuadratic.CompareRatios(n, d, root,
                    AffineSlabField(bestValues, bestSigns, 0), AffineSlabField(bestValues, bestSigns, 1), bestRoot);
                if (order > 0) continue;
                if (order == 0)
                {
                    // As in the rigid owner, equal-time witnesses prefer the
                    // closest height to the slab center. Retain the original
                    // quadratic ratios instead of comparing rounded anchors.
                    ty.CopyTo(work, ty.Sign(root) < 0 ? -1 : 1);
                    var bestY = AffineSlabField(bestValues, bestSigns, 3);
                    bestY.CopyTo(term, bestY.Sign(bestRoot) < 0 ? -1 : 1);
                    if (ContactQuadratic.CompareRatios(work, td, root, term,
                            AffineSlabField(bestValues, bestSigns, 5), bestRoot) >= 0) continue;
                }
            }
            n.CopyTo(AffineSlabField(bestValues, bestSigns, 0)); d.CopyTo(AffineSlabField(bestValues, bestSigns, 1));
            tx.CopyTo(AffineSlabField(bestValues, bestSigns, 2)); ty.CopyTo(AffineSlabField(bestValues, bestSigns, 3));
            tz.CopyTo(AffineSlabField(bestValues, bestSigns, 4)); td.CopyTo(AffineSlabField(bestValues, bestSigns, 5));
            root.CopyTo(bestRoot); found = true;
            if (branch == 0) return;
        }
    }

    private static bool ContainsAffineSlabSweepFace(ReadOnlySpan<WideDirection3d> walls,
        ReadOnlySpan<WideDirection3d> points, Signed320 denominator, scoped ContactQuadratic x,
        scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic d,
        ReadOnlySpan<ulong> root, scoped ContactQuadratic work, scoped ContactQuadratic term,
        scoped ContactQuadratic temporary)
    {
        for (int i = 0; i < 3; i++)
        {
            WideDirection3d wall = walls[i];
            ContactQuadratic.Scale(x, wall.X, work); ContactQuadratic.Scale(y, wall.Y, term); work.Add(term);
            ContactQuadratic.Scale(z, wall.Z, term); work.Add(term);
            ContactQuadratic.Scale(work, Signed576.ExtendValue(denominator), temporary);
            ContactQuadratic.Scale(d, WideDirection3d.Dot(wall, points[i]), term); temporary.Add(term, -1);
            if (temporary.Sign(root) < 0) return false;
        }
        return true;
    }

    private static WideDirection3d ReduceAffineSlabDirection(in WideDirection3d value)
    {
        if (value.IsZero) return value;
        Span<Signed576> components = stackalloc Signed576[3] { value.X, value.Y, value.Z };
        WideArithmetic.ReduceCommonScale(components);
        return new(components[0], components[1], components[2]);
    }

    private static WideDirection3d AffineSlabDifference(in WideDirection3d a, in WideDirection3d b) =>
        new(WideArithmetic.SubtractSigned576(a.X, b.X), WideArithmetic.SubtractSigned576(a.Y, b.Y), WideArithmetic.SubtractSigned576(a.Z, b.Z));
    private static Signed832 AffineSlabDot(in WideDirection3d a, in WideDirection3d b) =>
        WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(a.X, b.X), WideArithmetic.MultiplySigned576ToSigned832(a.Z, b.Z));
    private static Signed832 AffineSlabCross(Signed576 ax, Signed576 az, Signed576 bx, Signed576 bz) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(ax, bz), WideArithmetic.MultiplySigned576ToSigned832(az, bx));
    private static Signed832 AffineSlabNegate(Signed832 value) => WideArithmetic.SubtractSigned832(default, value);
    private static void AffineSlabProduct(Signed832 a, Signed832 b, scoped ContactQuadratic work, scoped ContactQuadratic result)
    { work.Set(a); ContactQuadratic.Scale(work, b, result); }

}
