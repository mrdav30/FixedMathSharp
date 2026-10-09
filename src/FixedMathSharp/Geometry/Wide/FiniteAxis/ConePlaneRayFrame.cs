//=======================================================================
// ConePlaneRayFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>One canonical authored plane and exact cone frame shared by its finite sections.</summary>
internal readonly struct ConePlaneRayFrame
{
    internal readonly Vector3d ConeCenter;
    internal readonly FixedQuaternion ConeRotation;
    internal readonly Fixed64 Height, Radius;
    internal readonly ConeFiniteSectionFrame Finite;
    internal readonly WideAxis3 Normal;
    internal readonly Signed576 PlaneConstant, NormalSquared;
    private readonly WideAxis3 authoredNormal;

    internal ConePlaneRayFrame(FixedTriangle planeTriangle, Vector3d origin, FixedQuaternion rotation,
        Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        ConeCenter = coneCenter; ConeRotation = coneRotation; Height = height; Radius = radius;
        Finite = new ConeFiniteSectionFrame(origin, rotation, coneCenter, coneRotation, height, radius);
        planeTriangle.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        Span<Signed320> primitive = stackalloc Signed320[3]
        {
            Signed320.ExtendValue(nx), Signed320.ExtendValue(ny), Signed320.ExtendValue(nz)
        };
        if (!nx.IsZero || !ny.IsZero || !nz.IsZero)
            WideArithmetic.ReduceCommonScale(primitive);
        int orientation = primitive[0].Sign != 0 ? primitive[0].Sign
            : primitive[1].Sign != 0 ? primitive[1].Sign : primitive[2].Sign;
        if (orientation < 0)
            for (int axis = 0; axis < 3; axis++)
                primitive[axis] = WideArithmetic.Negate(primitive[axis]);
        authoredNormal = new WideAxis3(primitive[0], primitive[1], primitive[2]);
        WideAxis3 localNormal = WideRigidProjection.TransformLocalAxis(Finite.ShapeFrame.Basis,
            Signed192.NarrowProven(primitive[0]), Signed192.NarrowProven(primitive[1]), Signed192.NarrowProven(primitive[2]));
        // Reflect the centered +Y coordinates into the apex-to-base +Y
        // cone frame. This is an exact coordinate convention, not rounding.
        primitive[0] = localNormal.X; primitive[1] = WideArithmetic.Negate(localNormal.Y); primitive[2] = localNormal.Z;
        // A positive common scale carries no plane or ray geometry. Remove
        // the quaternion basis denominator here so it does not multiply every
        // critical-event coefficient, while preserving the authored orientation.
        if (!localNormal.IsZero) WideArithmetic.ReduceCommonScale(primitive);
        Normal = new WideAxis3(primitive[0], primitive[1], primitive[2]);
        PlaneConstant = WideAxis3.Dot(Normal, Transform(planeTriangle.A));
        NormalSquared = WideAxis3.Dot(Normal, Normal);
    }

    /// <summary>Gets the exact canonical covector in the original authored frame, before normal rounding.</summary>
    internal WideAxis3 AuthoredNormal => authoredNormal;

    /// <summary>Materializes the canonical plane normal in its original authored rigid frame.</summary>
    /// <remarks>The rotation is the same validated authored rotation used to construct this frame.</remarks>
    internal Vector3d GetWorldNormal(FixedQuaternion authoredRotation)
    {
        // Primitive authored normal components are <130 bits and a normalized
        // quaternion's exact basis numerators are <65 bits. The three-term
        // transform stays <197 bits, within Signed320, without rounding a
        // local normal first or losing its canonical leading-component sign.
        WideAxis3 normal = WideRigidProjection.TransformLocalAxis(new WideRationalBasis3d(authoredRotation),
            Signed192.NarrowProven(authoredNormal.X), Signed192.NarrowProven(authoredNormal.Y),
            Signed192.NarrowProven(authoredNormal.Z));
        return WideNormalization.GetNormalized(normal.X, normal.Y, normal.Z);
    }

    internal WideAxis3 Transform(Vector3d point) => Finite.Transform(point);

    internal bool ContainsPoint(Vector3d point)
    {
        WideAxis3 p = Finite.Transform(point);
        return IsCoplanar(p) && Finite.ContainsTransformedPoint(p);
    }

    internal bool IntersectsSegment(FixedSegment segment)
    {
        WideAxis3 start = Finite.Transform(segment.Start), end = Finite.Transform(segment.End);
        return IsCoplanar(start) && IsCoplanar(end) && Finite.IntersectsTransformedSegment(start, end);
    }

    internal bool IsCoplanar(WideAxis3 point) =>
        WideArithmetic.SubtractSigned576(WideAxis3.Dot(Normal, point), PlaneConstant).IsZero;

    internal void GetTriangleEdgeWall(FixedTriangle triangle, int edgeIndex,
        out Signed576 x, out Signed576 y, out Signed576 z, out Signed576 offset)
    {
        FixedSegment edge = triangle.GetEdge(edgeIndex);
        GetEdgeLine(edge, out x, out y, out z, out offset);
        WideAxis3 opposite = Transform(edgeIndex == 0 ? triangle.C : edgeIndex == 1 ? triangle.A : triangle.B);
        if (WideArithmetic.SubtractSigned576(DotWall(x, y, z, opposite), offset).Sign < 0)
        {
            x = WideArithmetic.SubtractSigned576(default, x);
            y = WideArithmetic.SubtractSigned576(default, y);
            z = WideArithmetic.SubtractSigned576(default, z);
            offset = WideArithmetic.SubtractSigned576(default, offset);
        }
    }

    // The sign of an exact line equation is immaterial for its roots. Segment
    // sources therefore need no invented opposite vertex or enclosing triangle.
    internal void GetEdgeLine(FixedSegment edge,
        out Signed576 x, out Signed576 y, out Signed576 z, out Signed576 offset)
    {
        WideAxis3 delta = new(
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.X), Signed192.Raw(edge.Start.X))),
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.Y), Signed192.Raw(edge.Start.Y))),
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(edge.End.Z), Signed192.Raw(edge.Start.Z))));
        // Form the wall in the authored frame first: primitive normal <130,
        // edge <65, wall <196, exact relative basis <130. A direct cross of
        // the transformed normal and transformed edge would need >320 bits.
        WideAxis3 wall = WideAxis3.Cross(authoredNormal, delta);
        WideRigidProjection.TransformLocalAxis(Finite.ShapeFrame.Basis, wall, out x, out y, out z);
        y = WideArithmetic.SubtractSigned576(default, y);
        offset = DotWall(x, y, z, Transform(edge.Start));
    }

    private static Signed576 DotWall(Signed576 x, Signed576 y, Signed576 z, WideAxis3 point)
    {
        // Each transformed authored wall component <326 and point <198;
        // three signed products need <526 bits, safely inside Signed576.
        Signed704 product = WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(WideArithmetic.MultiplySigned576ToSigned704(x, point.X),
                WideArithmetic.MultiplySigned576ToSigned704(y, point.Y)),
            WideArithmetic.MultiplySigned576ToSigned704(z, point.Z));
        return Signed576.NarrowValue(product);
    }
}
