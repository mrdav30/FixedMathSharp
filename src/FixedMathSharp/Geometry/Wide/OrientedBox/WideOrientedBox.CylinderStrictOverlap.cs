//=======================================================================
// WideOrientedBox.CylinderStrictOverlap.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Complete strict finite-cylinder tests against triangles and convex solids.
/// Cap clipping and radial minimization preserve exact rigid-frame fractions.
/// </content>
internal static partial class WideOrientedBox
{
    private readonly struct CylinderPolytopeFrame
    {
        internal readonly WideRationalBasis3d Basis;
        internal readonly WideAxis3 Translation;
        internal readonly Signed192 Denominator;
        internal readonly Signed320 Cap;
        internal readonly Signed192 Radius;

        internal CylinderPolytopeFrame(Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
            Fixed64 height, Fixed64 radius, Vector3d origin, FixedQuaternion rotation)
        {
            WideRationalBasis3d cylinderBasis = new(cylinderRotation);
            WideRationalBasis3d shapeBasis = new(rotation);
            Basis = WideRationalBasis3d.CreateRelative(cylinderBasis, shapeBasis);
            Denominator = Basis.Denominator;
            GetRelativeLocalPointNumerators(origin, cylinderCenter, cylinderBasis,
                out Signed192 x, out Signed192 y, out Signed192 z);
            Translation = new WideAxis3(
                WideArithmetic.MultiplySigned192(x, shapeBasis.Denominator),
                WideArithmetic.MultiplySigned192(y, shapeBasis.Denominator),
                WideArithmetic.MultiplySigned192(z, shapeBasis.Denominator));
            // Work with doubled cylinder-local coordinates so odd raw full
            // heights need no rounded half-height. All vertices share D.
            Cap = WideArithmetic.MultiplySigned192(Signed192.Raw(height), Denominator);
            Radius = WideArithmetic.AddSigned192(Signed192.Raw(radius), Signed192.Raw(radius));
        }

        internal WideAxis3 Transform(Vector3d point)
        {
            WideAxis3 local = WideRigidProjection.TransformLocalAxis(Basis,
                Signed192.Raw(point.X), Signed192.Raw(point.Y), Signed192.Raw(point.Z));
            Signed320 x = WideArithmetic.AddSigned320(local.X, Translation.X);
            Signed320 y = WideArithmetic.AddSigned320(local.Y, Translation.Y);
            Signed320 z = WideArithmetic.AddSigned320(local.Z, Translation.Z);
            return new WideAxis3(WideArithmetic.AddSigned320(x, x),
                WideArithmetic.AddSigned320(y, y), WideArithmetic.AddSigned320(z, z));
        }
    }

    /// <summary>
    /// Tests positive-volume overlap of a local-+Y cylinder with an oriented
    /// box. Rotations are normalized, height/radius and all half-extents positive.
    /// No world vertex, witness, normal, or penetration depth is materialized.
    /// </summary>
    internal static bool DoesCenteredCylinderPenetrateBox(
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Fixed64 height, Fixed64 radius,
        Vector3d boxCenter, FixedQuaternion boxRotation, Vector3d halfExtents)
    {
        GetRelativeLocalPointNumerators(cylinderCenter, boxCenter, boxRotation,
            out Signed192 x, out Signed192 y, out Signed192 z, out Signed192 d);
        if (IsStrictCylinderBoxCoordinateInside(x, halfExtents.X, d)
            && IsStrictCylinderBoxCoordinateInside(y, halfExtents.Y, d)
            && IsStrictCylinderBoxCoordinateInside(z, halfExtents.Z, d))
        {
            return true;
        }

        CylinderPolytopeFrame frame = new(cylinderCenter, cylinderRotation, height, radius,
            boxCenter, boxRotation);
        Span<WideAxis3> vertices = stackalloc WideAxis3[8];
        for (int index = 0; index < vertices.Length; index++)
        {
            vertices[index] = frame.Transform(new Vector3d(
                (index & 1) == 0 ? -halfExtents.X : halfExtents.X,
                (index & 2) == 0 ? -halfExtents.Y : halfExtents.Y,
                (index & 4) == 0 ? -halfExtents.Z : halfExtents.Z));
        }
        ReadOnlySpan<int> triangles = stackalloc int[]
        {
            0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5,
            0, 4, 5, 0, 5, 1, 2, 3, 7, 2, 7, 6,
            0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3
        };
        for (int index = 0; index < triangles.Length; index += 3)
        {
            if (DoesCylinderTrianglePenetrate(frame, vertices[triangles[index]],
                    vertices[triangles[index + 1]], vertices[triangles[index + 2]]))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Tests whether an authored triangle contains any point strictly inside a
    /// local-+Y cylinder. Height/radius are positive and rotations normalized.
    /// Degenerate triangles retain their segment/point meaning; cap and radial
    /// tangencies are excluded independently.
    /// </summary>
    internal static bool DoesCenteredCylinderPenetrateTriangle(
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Fixed64 height, Fixed64 radius,
        FixedTriangle triangle, Vector3d triangleOrigin, FixedQuaternion triangleRotation)
    {
        CylinderPolytopeFrame frame = new(cylinderCenter, cylinderRotation, height, radius,
            triangleOrigin, triangleRotation);
        return DoesCylinderTrianglePenetrate(frame, frame.Transform(triangle.A),
            frame.Transform(triangle.B), frame.Transform(triangle.C));
    }

    /// <summary>
    /// Tests a cylinder against validated convex triangle topology. Closed
    /// volumes include cylinder enclosure; open coplanar surfaces use triangle
    /// intrusion only. Height/radius are positive, frames normalized, indices
    /// valid, and closed-volume triangles have consistent shell winding.
    /// </summary>
    internal static bool DoesCenteredCylinderPenetrateConvexHull(
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Fixed64 height, Fixed64 radius,
        Vector3d hullOrigin, FixedQuaternion hullRotation, ReadOnlySpan<Vector3d> localPoints,
        ReadOnlySpan<int> triangleIndices, bool closedVolume)
    {
        if (closedVolume && IsCylinderCenterStrictlyInsideHull(cylinderCenter,
                hullOrigin, hullRotation, localPoints, triangleIndices))
            return true;

        CylinderPolytopeFrame frame = new(cylinderCenter, cylinderRotation, height, radius,
            hullOrigin, hullRotation);
        for (int index = 0; index < triangleIndices.Length; index += 3)
        {
            if (DoesCylinderTrianglePenetrate(frame,
                    frame.Transform(localPoints[triangleIndices[index]]),
                    frame.Transform(localPoints[triangleIndices[index + 1]]),
                    frame.Transform(localPoints[triangleIndices[index + 2]])))
                return true;
        }
        return false;
    }

    private static bool IsStrictCylinderBoxCoordinateInside(Signed192 coordinate,
        Fixed64 extent, Signed192 denominator) =>
        WideArithmetic.CompareMagnitude(Signed320.ExtendValue(coordinate),
            WideArithmetic.MultiplySigned192(Signed192.Raw(extent), denominator)) < 0;

    private static bool IsCylinderCenterStrictlyInsideHull(Vector3d center,
        Vector3d origin, FixedQuaternion rotation, ReadOnlySpan<Vector3d> vertices,
        ReadOnlySpan<int> triangles)
    {
        GetRelativeLocalPointNumerators(center, origin, rotation,
            out Signed192 x, out Signed192 y, out Signed192 z, out Signed192 denominator);
        int interiorSign = 0;
        for (int index = 0; index < triangles.Length; index += 3)
        {
            Vector3d a = vertices[triangles[index]];
            Vector3d b = vertices[triangles[index + 1]];
            Vector3d c = vertices[triangles[index + 2]];
            WideGeometry.GetDifferenceCrossProduct3D(b.X, a.X, b.Y, a.Y, b.Z, a.Z,
                c.X, a.X, c.Y, a.Y, c.Z, a.Z,
                out Signed192 nx, out Signed192 ny, out Signed192 nz);
            if (nx.IsZero && ny.IsZero && nz.IsZero)
                continue;
            Signed192 dx = WideArithmetic.SubtractSigned192(x, Signed192.NarrowProven(
                WideArithmetic.MultiplySigned192(Signed192.Raw(a.X), denominator)));
            Signed192 dy = WideArithmetic.SubtractSigned192(y, Signed192.NarrowProven(
                WideArithmetic.MultiplySigned192(Signed192.Raw(a.Y), denominator)));
            Signed192 dz = WideArithmetic.SubtractSigned192(z, Signed192.NarrowProven(
                WideArithmetic.MultiplySigned192(Signed192.Raw(a.Z), denominator)));
            int sign = WideArithmetic.GetDotProduct3D(nx, ny, nz, dx, dy, dz).Sign;
            if (sign == 0 || (interiorSign != 0 && sign != interiorSign))
                return false;
            interiorSign = sign;
        }
        return interiorSign != 0;
    }

    private static bool DoesCylinderTrianglePenetrate(in CylinderPolytopeFrame frame,
        in WideAxis3 a, in WideAxis3 b, in WideAxis3 c)
    {
        Signed320 lower = WideArithmetic.SubtractSigned320(default, frame.Cap);
        // The closed clipped polygon alone would admit a triangle coplanar
        // with a cap. Require an axial-interior point before minimizing radius.
        if ((CompareSigned(a.Y, lower) <= 0 && CompareSigned(b.Y, lower) <= 0 && CompareSigned(c.Y, lower) <= 0)
            || (CompareSigned(a.Y, frame.Cap) >= 0 && CompareSigned(b.Y, frame.Cap) >= 0 && CompareSigned(c.Y, frame.Cap) >= 0))
            return false;

        if (IsCylinderAxisInsideTriangleProjection(a, b, c, lower, frame.Cap))
            return true;
        if (DoesCylinderClippedEdgePenetrate(a, b, lower, frame.Cap, frame)
            || DoesCylinderClippedEdgePenetrate(b, c, lower, frame.Cap, frame)
            || DoesCylinderClippedEdgePenetrate(c, a, lower, frame.Cap, frame))
            return true;

        // These are the remaining two possible edges of the clipped polygon.
        return DoesCylinderCapSectionPenetrate(a, b, c, lower, frame)
            || DoesCylinderCapSectionPenetrate(a, b, c, frame.Cap, frame);
    }

    private static bool IsCylinderAxisInsideTriangleProjection(in WideAxis3 a,
        in WideAxis3 b, in WideAxis3 c, Signed320 lower, Signed320 upper)
    {
        Signed576 wa = GetCylinderRadialCross(b, c);
        Signed576 wb = GetCylinderRadialCross(c, a);
        Signed576 wc = GetCylinderRadialCross(a, b);
        Signed576 area = WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(wa, wb), wc);
        if (area.IsZero)
            return false;
        if ((wa.Sign != 0 && wa.Sign != area.Sign) || (wb.Sign != 0 && wb.Sign != area.Sign)
            || (wc.Sign != 0 && wc.Sign != area.Sign))
            return false;
        if (area.Sign < 0)
        {
            wa = WideArithmetic.SubtractSigned576(default, wa);
            wb = WideArithmetic.SubtractSigned576(default, wb);
            wc = WideArithmetic.SubtractSigned576(default, wc);
            area = WideArithmetic.SubtractSigned576(default, area);
        }
        Signed832 y = WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(wa, a.Y),
            WideArithmetic.MultiplySigned576ToSigned832(wb, b.Y)),
            WideArithmetic.MultiplySigned576ToSigned832(wc, c.Y));
        return WideArithmetic.SubtractSigned832(y,
                WideArithmetic.MultiplySigned576ToSigned832(area, lower)).Sign >= 0
            && WideArithmetic.SubtractSigned832(y,
                WideArithmetic.MultiplySigned576ToSigned832(area, upper)).Sign <= 0;
    }

    private static Signed576 GetCylinderRadialCross(in WideAxis3 a, in WideAxis3 b) =>
        WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(a.X, b.Z),
            WideArithmetic.MultiplySigned320(a.Z, b.X));

    private static bool DoesCylinderClippedEdgePenetrate(in WideAxis3 a, in WideAxis3 b,
        Signed320 lower, Signed320 upper, in CylinderPolytopeFrame frame)
    {
        if ((CompareSigned(a.Y, lower) < 0 && CompareSigned(b.Y, lower) < 0)
            || (CompareSigned(a.Y, upper) > 0 && CompareSigned(b.Y, upper) > 0))
            return false;
        SweepRationalPoint first = GetCylinderClippedEndpoint(a, b, lower, upper, frame.Denominator);
        SweepRationalPoint second = GetCylinderClippedEndpoint(b, a, lower, upper, frame.Denominator);
        return IsCylinderRadialSegmentInside(first, second, frame.Radius);
    }

    private static SweepRationalPoint GetCylinderClippedEndpoint(in WideAxis3 endpoint,
        in WideAxis3 other, Signed320 lower, Signed320 upper, Signed192 denominator)
    {
        if (CompareSigned(endpoint.Y, lower) < 0)
            return GetCylinderPlaneIntersection(endpoint, other, lower, denominator);
        if (CompareSigned(endpoint.Y, upper) > 0)
            return GetCylinderPlaneIntersection(endpoint, other, upper, denominator);
        return GetCylinderRadialPoint(endpoint, denominator);
    }

    private static SweepRationalPoint GetCylinderRadialPoint(in WideAxis3 point, Signed192 denominator) =>
        new(Signed576.ExtendValue(point.X), Signed576.ExtendValue(point.Z),
            Signed576.ExtendValue(Signed320.ExtendValue(denominator)));

    private static SweepRationalPoint GetCylinderPlaneIntersection(in WideAxis3 a,
        in WideAxis3 b, Signed320 plane, Signed192 denominator)
    {
        Signed320 wa = WideArithmetic.SubtractSigned320(b.Y, plane);
        Signed320 wb = WideArithmetic.SubtractSigned320(plane, a.Y);
        Signed576 x = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(a.X, wa),
            WideArithmetic.MultiplySigned320(b.X, wb));
        Signed576 z = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(a.Z, wa),
            WideArithmetic.MultiplySigned320(b.Z, wb));
        Signed576 d = WideArithmetic.MultiplySigned320(WideArithmetic.SubtractSigned320(b.Y, a.Y), denominator);
        if (d.Sign < 0)
        {
            x = WideArithmetic.SubtractSigned576(default, x);
            z = WideArithmetic.SubtractSigned576(default, z);
            d = WideArithmetic.SubtractSigned576(default, d);
        }
        return new SweepRationalPoint(x, z, d);
    }

    private static bool DoesCylinderCapSectionPenetrate(in WideAxis3 a, in WideAxis3 b,
        in WideAxis3 c, Signed320 plane, in CylinderPolytopeFrame frame)
    {
        Span<SweepRationalPoint> points = stackalloc SweepRationalPoint[3];
        int count = 0;
        AddCylinderPlaneIntersection(a, b, plane, frame.Denominator, points, ref count);
        AddCylinderPlaneIntersection(b, c, plane, frame.Denominator, points, ref count);
        AddCylinderPlaneIntersection(c, a, plane, frame.Denominator, points, ref count);
        return count == 1 ? IsCylinderRadialPointInside(points[0], frame.Radius)
            : count >= 2 && IsCylinderRadialSegmentInside(points[0], points[1], frame.Radius);
    }

    private static void AddCylinderPlaneIntersection(in WideAxis3 a, in WideAxis3 b,
        Signed320 plane, Signed192 denominator, Span<SweepRationalPoint> points, ref int count)
    {
        int first = CompareSigned(a.Y, plane);
        int second = CompareSigned(b.Y, plane);
        if (first == 0)
            AddUniqueSweepPoint(GetCylinderRadialPoint(a, denominator), points, ref count);
        if (second == 0)
            AddUniqueSweepPoint(GetCylinderRadialPoint(b, denominator), points, ref count);
        if (first != 0 && second != 0 && first != second)
            AddUniqueSweepPoint(GetCylinderPlaneIntersection(a, b, plane, denominator), points, ref count);
    }

    private static bool IsCylinderRadialPointInside(in SweepRationalPoint point, Signed192 radius)
    {
        Signed832 squared = WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(point.X, point.X),
            WideArithmetic.MultiplySigned576ToSigned832(point.Z, point.Z));
        Signed576 bound = WideArithmetic.MultiplySigned576(point.Denominator, radius);
        return WideArithmetic.SubtractSigned832(squared,
            WideArithmetic.MultiplySigned576ToSigned832(bound, bound)).Sign < 0;
    }

    private static bool IsCylinderRadialSegmentInside(in SweepRationalPoint a,
        in SweepRationalPoint b, Signed192 radius)
    {
        if (IsCylinderRadialPointInside(a, radius) || IsCylinderRadialPointInside(b, radius))
            return true;
        Signed832 ex = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(b.X, a.Denominator),
            WideArithmetic.MultiplySigned576ToSigned832(a.X, b.Denominator));
        Signed832 ez = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(b.Z, a.Denominator),
            WideArithmetic.MultiplySigned576ToSigned832(a.Z, b.Denominator));
        if (ex.IsZero && ez.IsZero)
            return false;
        Span<ulong> scratch = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetDotMagnitude(ex, ez, a.X, a.Z, scratch, out int startDot);
        if (startDot >= 0)
            return false;
        GetDotMagnitude(ex, ez, b.X, b.Z, scratch, out int endDot);
        if (endDot <= 0)
            return false;

        // For A/a, B/b and E=a*B-b*A the interior distance squared is
        // cross(A,B)^2 / dot(E,E): the common a^2 cancels exactly.
        Signed832 cross = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(a.X, b.Z),
            WideArithmetic.MultiplySigned576ToSigned832(a.Z, b.X));
        Span<ulong> crossWords = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(cross, crossWords);
        Span<ulong> left = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(crossWords, crossWords, left);
        GetEdgeSquaredMagnitude(ex, ez, scratch);
        Span<ulong> radiusWords = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(radius, out radiusWords[2], out radiusWords[1], out radiusWords[0]);
        Span<ulong> radiusSquared = stackalloc ulong[6];
        WideArithmetic.MultiplyMagnitudes(radiusWords, radiusWords, radiusSquared);
        Span<ulong> right = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(radiusSquared, scratch, right);
        // Conservatively each normalized basis denominator is <2^68. Relative
        // D<2^136 and doubled transformed vertices <2^204. Clipping ORIGINAL
        // edges once gives numerator <2^410 and denominator <2^341. Hence E
        // fits <2^752, cross(A,B) <2^821, and this comparison <2^1642, below
        // the existing fixed 36-word capacity. No repeated rational clipping.
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) < 0;
    }
}
