//=======================================================================
// WideFiniteAxisIntersection.CenteredAxisTriangle.Materialization.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Helpers for materializing wide-precision axis/triangle intersection
/// candidates back into <see cref="Fixed64"/>/<see cref="Vector3d"/> results,
/// including triangle containment tests and normal/edge computations.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private static bool ContainsTriangleProjection(
        FixedTriangle triangle,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed576 denominator) =>
        IsInsideTriangleEdge(
            triangle.A,
            triangle.B,
            normalX,
            normalY,
            normalZ,
            pointX,
            pointY,
            pointZ,
            denominator)
        && IsInsideTriangleEdge(
            triangle.B,
            triangle.C,
            normalX,
            normalY,
            normalZ,
            pointX,
            pointY,
            pointZ,
            denominator)
        && IsInsideTriangleEdge(
            triangle.C,
            triangle.A,
            normalX,
            normalY,
            normalZ,
            pointX,
            pointY,
            pointZ,
            denominator);

    private static bool IsInsideTriangleEdge(
        Vector3d start,
        Vector3d end,
        Signed192 normalX,
        Signed192 normalY,
        Signed192 normalZ,
        Signed576 pointX,
        Signed576 pointY,
        Signed576 pointZ,
        Signed576 denominator)
    {
        Signed192 edgeX = GetComponentDifference(end.X, start.X);
        Signed192 edgeY = GetComponentDifference(end.Y, start.Y);
        Signed192 edgeZ = GetComponentDifference(end.Z, start.Z);
        Signed320 inwardX = WideArithmetic.MultiplySubtract(
            normalY,
            edgeZ,
            normalZ,
            edgeY);
        Signed320 inwardY = WideArithmetic.MultiplySubtract(
            normalZ,
            edgeX,
            normalX,
            edgeZ);
        Signed320 inwardZ = WideArithmetic.MultiplySubtract(
            normalX,
            edgeY,
            normalY,
            edgeX);
        Signed704 side = WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    SubtractVertex(pointX, start.X, denominator),
                    inwardX),
                WideArithmetic.MultiplySigned576ToSigned704(
                    SubtractVertex(pointY, start.Y, denominator),
                    inwardY)),
            WideArithmetic.MultiplySigned576ToSigned704(
                SubtractVertex(pointZ, start.Z, denominator),
                inwardZ));
        return side.Sign >= 0;
    }

    private static Signed576 SubtractVertex(
        Signed576 point,
        Fixed64 vertex,
        Signed320 denominator) =>
        WideArithmetic.SubtractSigned576(
            point,
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(
                    Signed192.Signed(vertex.m_rawValue)),
                denominator));

    private static Signed576 SubtractVertex(
        Signed576 point,
        Fixed64 vertex,
        Signed576 denominator) =>
        WideArithmetic.SubtractSigned576(
            point,
            WideArithmetic.MultiplySigned576(
                denominator,
                Signed192.Signed(vertex.m_rawValue)));

    private static Signed320 GetWideDot(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Vector3d end,
        Vector3d start) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    x,
                    GetComponentDifference(end.X, start.X)),
                WideArithmetic.MultiplySigned192(
                    y,
                    GetComponentDifference(end.Y, start.Y))),
            WideArithmetic.MultiplySigned192(
                z,
                GetComponentDifference(end.Z, start.Z)));

    private static void GetTriangleNormal(
        FixedTriangle triangle,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z) =>
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
            out x,
            out y,
            out z);

    private static bool TryMaterialize(
        AxisTriangleCandidate candidate,
        out Vector3d pointOnTriangle,
        out Vector3d pointOnAxis)
    {
        // A selected triangle candidate is a convex combination of authored
        // vertices, so its coordinates remain representable.
        _ = Fixed64.TryGetSignedRawRatio(
            candidate.TriangleX,
            candidate.Denominator,
            out Fixed64 triangleX);
        _ = Fixed64.TryGetSignedRawRatio(
            candidate.TriangleY,
            candidate.Denominator,
            out Fixed64 triangleY);
        _ = Fixed64.TryGetSignedRawRatio(
            candidate.TriangleZ,
            candidate.Denominator,
            out Fixed64 triangleZ);
        bool representable = Fixed64.TryGetSignedRawRatio(
            candidate.AxisX,
            candidate.Denominator,
            out Fixed64 axisX);
        representable &= Fixed64.TryGetSignedRawRatio(
            candidate.AxisY,
            candidate.Denominator,
            out Fixed64 axisY);
        representable &= Fixed64.TryGetSignedRawRatio(
            candidate.AxisZ,
            candidate.Denominator,
            out Fixed64 axisZ);
        if (!representable)
        {
            pointOnTriangle = default;
            pointOnAxis = default;
            return false;
        }

        pointOnTriangle = new Vector3d(triangleX, triangleY, triangleZ);
        pointOnAxis = new Vector3d(axisX, axisY, axisZ);
        return true;
    }

    private static bool TryOffsetPoint(
        Vector3d point,
        Vector3d direction,
        Fixed64 distance,
        out Vector3d result)
    {
        bool representable =
            Fixed64.TryMultiplyAdd(direction.X, distance, point.X, out Fixed64 x)
            & Fixed64.TryMultiplyAdd(direction.Y, distance, point.Y, out Fixed64 y)
            & Fixed64.TryMultiplyAdd(direction.Z, distance, point.Z, out Fixed64 z);
        result = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    private static bool TryGetRelativeCoordinate(
        Signed576 pointNumerator,
        Fixed64 origin,
        Signed576 denominator,
        out Fixed64 coordinate)
    {
        Signed576 originNumerator = WideArithmetic.MultiplySigned576(
            denominator,
            Signed192.Signed(origin.m_rawValue));
        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.SubtractSigned576(
                pointNumerator,
                originNumerator),
            denominator,
            out coordinate);
    }

}
