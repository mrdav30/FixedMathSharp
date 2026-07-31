//=======================================================================
// WidePlanarProjection.Polygon.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact rational polygon projection and closest-feature reduction.
/// </content>
internal static partial class WidePlanarProjection
{
    private const int DistanceComparisonWordCount = 36;

    internal static bool TryGetOrientedBoxRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        FixedOrientedBox box,
        out PlanarProjectionRelation relation)
    {
        WideRationalBasis3d basis = new(box.Orientation);
        Span<RationalPoint> corners = stackalloc RationalPoint[8];
        int index = 0;
        for (int x = -1; x <= 1; x += 2)
        {
            for (int y = -1; y <= 1; y += 2)
            {
                for (int z = -1; z <= 1; z += 2)
                {
                    corners[index++] = GetWorldPoint(
                        box.Center,
                        basis,
                        new Vector3d(
                            x * box.HalfExtents.X,
                            y * box.HalfExtents.Y,
                            z * box.HalfExtents.Z));
                }
            }
        }

        Span<RationalPoint> hull = stackalloc RationalPoint[8];
        int count = BuildConvexHull(corners, hull);
        return TryGetConvexRelation(
            circleCenter,
            circleRadius,
            basis.Denominator,
            hull[..count],
            out relation);
    }

    internal static bool TryGetTriangleRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        out PlanarProjectionRelation relation)
    {
        WideRationalBasis3d basis = new(triangleRotation);
        Span<RationalPoint> vertices = stackalloc RationalPoint[3];
        vertices[0] = GetWorldPoint(
            triangleOrigin,
            basis,
            triangle.A);
        vertices[1] = GetWorldPoint(
            triangleOrigin,
            basis,
            triangle.B);
        vertices[2] = GetWorldPoint(
            triangleOrigin,
            basis,
            triangle.C);

        if (ContainsPoint(
                circleCenter,
                basis.Denominator,
                vertices))
        {
            relation = PlanarProjectionRelation.Contained;
            return true;
        }

        RationalDistance best = GetSegmentDistance(
            circleCenter,
            basis.Denominator,
            vertices[0],
            vertices[1]);
        KeepCloser(
            GetSegmentDistance(
                circleCenter,
                basis.Denominator,
                vertices[1],
                vertices[2]),
            ref best);
        KeepCloser(
            GetSegmentDistance(
                circleCenter,
                basis.Denominator,
                vertices[2],
                vertices[0]),
            ref best);
        return TryCreateRelation(
            best,
            circleRadius,
            out relation);
    }

    private static bool TryGetConvexRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Signed192 denominator,
        ReadOnlySpan<RationalPoint> vertices,
        out PlanarProjectionRelation relation)
    {
        if (ContainsPoint(
                circleCenter,
                denominator,
                vertices))
        {
            relation = PlanarProjectionRelation.Contained;
            return true;
        }

        RationalDistance best = GetSegmentDistance(
            circleCenter,
            denominator,
            vertices[0],
            vertices[1]);
        for (int index = 1; index < vertices.Length; index++)
        {
            KeepCloser(
                GetSegmentDistance(
                    circleCenter,
                    denominator,
                    vertices[index],
                    vertices[
                        index + 1 == vertices.Length
                            ? 0
                            : index + 1]),
                ref best);
        }

        return TryCreateRelation(
            best,
            circleRadius,
            out relation);
    }

    private static bool TryCreateRelation(
        RationalDistance distance,
        Fixed64 circleRadius,
        out PlanarProjectionRelation relation)
    {
        if (distance.SquaredDistance.IsZero)
        {
            relation = PlanarProjectionRelation.Contained;
            return true;
        }

        if (!TryGetDistance(
                distance,
                circleRadius,
                out Fixed64 result))
        {
            relation = default;
            return false;
        }

        relation = new PlanarProjectionRelation(
            result,
            GetOffset(distance),
            WideNormalization.GetNormalized(
                distance.X,
                distance.Z));
        return true;
    }

    private static bool ContainsPoint(
        Vector2d point,
        Signed192 denominator,
        ReadOnlySpan<RationalPoint> vertices)
    {
        int winding = 0;
        Signed320 pointX = WideArithmetic.MultiplySigned192(
            Signed192.Raw(point.X),
            denominator);
        Signed320 pointZ = WideArithmetic.MultiplySigned192(
            Signed192.Raw(point.Y),
            denominator);
        for (int index = 0; index < vertices.Length; index++)
        {
            RationalPoint first = vertices[index];
            RationalPoint second = vertices[
                index + 1 == vertices.Length ? 0 : index + 1];
            Signed576 cross = Cross(
                WideArithmetic.SubtractSigned320(
                    second.X,
                    first.X),
                WideArithmetic.SubtractSigned320(
                    second.Z,
                    first.Z),
                WideArithmetic.SubtractSigned320(
                    pointX,
                    first.X),
                WideArithmetic.SubtractSigned320(
                    pointZ,
                    first.Z));
            int sign = cross.Sign;
            if (sign == 0)
                continue;
            if (winding == 0)
                winding = sign;
            else if (winding != sign)
                return false;
        }

        return winding != 0;
    }

    private static int BuildConvexHull(
        Span<RationalPoint> points,
        Span<RationalPoint> hull)
    {
        SortAndDeduplicate(points, out int uniqueCount);
        Span<RationalPoint> work =
            stackalloc RationalPoint[16];
        int count = 0;
        for (int index = 0; index < uniqueCount; index++)
        {
            while (count >= 2
                   && GetTurn(
                       work[count - 2],
                       work[count - 1],
                       points[index]) <= 0)
            {
                count--;
            }
            work[count++] = points[index];
        }

        int lowerCount = count;
        for (int index = uniqueCount - 2; index >= 0; index--)
        {
            while (count > lowerCount
                   && GetTurn(
                       work[count - 2],
                       work[count - 1],
                       points[index]) <= 0)
            {
                count--;
            }
            work[count++] = points[index];
        }

        count--;
        work[..count].CopyTo(hull);
        return count;
    }

    private static void SortAndDeduplicate(
        Span<RationalPoint> points,
        out int count)
    {
        for (int index = 1; index < points.Length; index++)
        {
            RationalPoint value = points[index];
            int destination = index;
            while (destination > 0
                   && ComparePoint(
                       value,
                       points[destination - 1]) < 0)
            {
                points[destination] =
                    points[destination - 1];
                destination--;
            }
            points[destination] = value;
        }

        count = 1;
        for (int index = 1; index < points.Length; index++)
        {
            if (ComparePoint(points[index], points[count - 1]) == 0)
                continue;
            points[count++] = points[index];
        }
    }

    private static int ComparePoint(
        RationalPoint first,
        RationalPoint second)
    {
        int comparison = WideArithmetic.SubtractSigned320(
            first.X,
            second.X).Sign;
        return comparison != 0
            ? comparison
            : WideArithmetic.SubtractSigned320(
                first.Z,
                second.Z).Sign;
    }

    private static int GetTurn(
        RationalPoint first,
        RationalPoint second,
        RationalPoint third) =>
        Cross(
            WideArithmetic.SubtractSigned320(
                second.X,
                first.X),
            WideArithmetic.SubtractSigned320(
                second.Z,
                first.Z),
            WideArithmetic.SubtractSigned320(
                third.X,
                second.X),
            WideArithmetic.SubtractSigned320(
                third.Z,
                second.Z)).Sign;

    private static Signed576 Cross(
        Signed320 firstX,
        Signed320 firstZ,
        Signed320 secondX,
        Signed320 secondZ) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                firstX,
                secondZ),
            WideArithmetic.MultiplySigned320(
                firstZ,
                secondX));

    private static void KeepCloser(
        RationalDistance candidate,
        ref RationalDistance best)
    {
        if (CompareDistances(candidate, best) < 0)
            best = candidate;
    }

    private static int CompareDistances(
        RationalDistance first,
        RationalDistance second)
    {
        Signed832 firstDenominatorSquared =
            WideArithmetic.MultiplySigned576ToSigned832(
                first.Denominator,
                first.Denominator);
        Signed832 secondDenominatorSquared =
            WideArithmetic.MultiplySigned576ToSigned832(
                second.Denominator,
                second.Denominator);
        Span<ulong> firstSquaredMagnitude = stackalloc ulong[13];
        Span<ulong> secondSquaredMagnitude = stackalloc ulong[13];
        Span<ulong> firstDenominatorMagnitude = stackalloc ulong[13];
        Span<ulong> secondDenominatorMagnitude = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(
            first.SquaredDistance,
            firstSquaredMagnitude);
        WideArithmetic.GetMagnitude(
            second.SquaredDistance,
            secondSquaredMagnitude);
        WideArithmetic.GetMagnitude(
            firstDenominatorSquared,
            firstDenominatorMagnitude);
        WideArithmetic.GetMagnitude(
            secondDenominatorSquared,
            secondDenominatorMagnitude);
        Span<ulong> left =
            stackalloc ulong[DistanceComparisonWordCount];
        Span<ulong> right =
            stackalloc ulong[DistanceComparisonWordCount];
        WideArithmetic.MultiplyMagnitudes(
            firstSquaredMagnitude,
            secondDenominatorMagnitude,
            left);
        WideArithmetic.MultiplyMagnitudes(
            secondSquaredMagnitude,
            firstDenominatorMagnitude,
            right);
        return WideArithmetic.CompareMagnitudeEqualLength(
            left,
            right);
    }
}
