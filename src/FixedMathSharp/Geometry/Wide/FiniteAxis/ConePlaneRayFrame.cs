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
    internal readonly WideDirection3d Normal;
    internal readonly Signed832 PlaneConstant, NormalSquared;
    private readonly WideAxis3 authoredNormal;
    private readonly bool _wideParameters;
    internal ScaledRigidFrame3d SourceFrame => Finite.ShapeFrame.SourceFrame;
    internal bool UsesWideBanks => Finite.ShapeFrame.HasExactScale || _wideParameters;
    internal int FieldWords => UsesWideBanks ? 192 : ConePlaneRaySelection.FieldWords;
    internal int RootWords => UsesWideBanks ? 120 : ConePlaneRaySelection.RootWords;
    internal int CircleWords => UsesWideBanks ? 60 : 20;
    internal int PointStorageWords => 8 * FieldWords;
    internal int SelectionStorageWords => 12 * FieldWords + RootWords;
    internal int AdmissionWords => UsesWideBanks ? 432 : 144;
    internal int VerificationWords => UsesWideBanks ? 528 : 176;
    internal ConePlaneRayFrame WithWideParameters() => UsesWideBanks ? this : new ConePlaneRayFrame(this);

    // Numeric native charts are already cone-local. Materialization supplies
    // their actual physical pose without retransforming or replaying an event.
    internal ConePlaneRayFrame WithConePose(Vector3d center, FixedQuaternion rotation) => new(this, center, rotation);
    private ConePlaneRayFrame(in ConePlaneRayFrame source, Vector3d center, FixedQuaternion rotation)
    { this = source; ConeCenter = center; ConeRotation = rotation; }

    private ConePlaneRayFrame(in ConePlaneRayFrame source)
    {
        this = source;
        _wideParameters = true;
    }

    /// <summary>Shares an exact primitive-local native plane without a rounded source transform.</summary>
    internal ConePlaneRayFrame(Fixed64 height, Fixed64 radius, Signed320 coordinateDenominator,
        in WideDirection3d centeredNormal, Signed832 doubledCenteredConstant)
    {
        this = default;
        ConeRotation = FixedQuaternion.Identity; Height = height; Radius = radius;
        Finite = new ConeFiniteSectionFrame(height, radius, coordinateDenominator);
        Normal = new(centeredNormal.X, WideArithmetic.Negate(centeredNormal.Y), centeredNormal.Z);
        PlaneConstant = WideArithmetic.SubtractSigned832(doubledCenteredConstant,
            WideArithmetic.MultiplySigned576ToSigned832(centeredNormal.Y, Finite.ShapeFrame.Cap));
        NormalSquared = Normal.SquaredLength;
        authoredNormal = new(Signed320.NarrowValue(centeredNormal.X), Signed320.NarrowValue(centeredNormal.Y), Signed320.NarrowValue(centeredNormal.Z));
    }

    internal ConePlaneRayFrame(FixedTriangle planeTriangle, Vector3d origin, FixedQuaternion rotation,
        Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        _wideParameters = false;
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
        Normal = new WideDirection3d(new WideAxis3(primitive[0], primitive[1], primitive[2]));
        PlaneConstant = WideDirection3d.Dot(Normal, Transform(planeTriangle.A));
        NormalSquared = Normal.SquaredLength;
    }

    internal ConePlaneRayFrame(FixedTriangle planeTriangle, in ScaledRigidFrame3d sourceFrame,
        Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        if (sourceFrame.IsRigid)
        {
            this = new ConePlaneRayFrame(planeTriangle, sourceFrame.RigidOrigin, sourceFrame.Rotation,
                coneCenter, coneRotation, height, radius);
            return;
        }
        ConeCenter = coneCenter; ConeRotation = coneRotation; Height = height; Radius = radius;
        _wideParameters = false;
        Finite = new ConeFiniteSectionFrame(sourceFrame, coneCenter, coneRotation, height, radius);
        WideDirection3d edge = WideDirection3d.GetLocalEdge(sourceFrame, planeTriangle.A, planeTriangle.B);
        WideDirection3d side = WideDirection3d.GetLocalEdge(sourceFrame, planeTriangle.A, planeTriangle.C);
        WideDirection3d cross = WideDirection3d.Cross(edge, side);
        Span<Signed320> primitive = stackalloc Signed320[3]
        {
            Signed320.NarrowValue(cross.X), Signed320.NarrowValue(cross.Y), Signed320.NarrowValue(cross.Z)
        };
        if (!cross.IsZero) WideArithmetic.ReduceCommonScale(primitive);
        int orientation = primitive[0].Sign != 0 ? primitive[0].Sign
            : primitive[1].Sign != 0 ? primitive[1].Sign : primitive[2].Sign;
        if (orientation < 0)
            for (int axis = 0; axis < 3; axis++) primitive[axis] = WideArithmetic.Negate(primitive[axis]);
        authoredNormal = new WideAxis3(primitive[0], primitive[1], primitive[2]);
        WideDirection3d local = new WideDirection3d(authoredNormal).Transform(Finite.ShapeFrame.Basis);
        Span<Signed576> localComponents = stackalloc Signed576[3]
        { local.X, WideArithmetic.SubtractSigned576(default, local.Y), local.Z };
        if (!local.IsZero) WideArithmetic.ReduceCommonScale(localComponents);
        Normal = new WideDirection3d(localComponents[0], localComponents[1], localComponents[2]);
        // Scaled edges <110 bits give an authored normal <221; a relative
        // rigid basis adds <130. Plane and norm² stay below 660/705 bits.
        PlaneConstant = WideDirection3d.Dot(Normal, Transform(planeTriangle.A));
        NormalSquared = Normal.SquaredLength;
    }

    /// <summary>Gets the exact canonical covector in the original authored frame, before normal rounding.</summary>
    internal WideAxis3 AuthoredNormal => authoredNormal;

    /// <summary>Materializes the canonical plane normal in its original authored rigid frame.</summary>
    /// <remarks>The rotation is the same validated authored rotation used to construct this frame.</remarks>
    internal Vector3d GetWorldNormal(FixedQuaternion authoredRotation)
    {
        if (Finite.ShapeFrame.HasExactScale)
            return new WideDirection3d(authoredNormal).Transform(SourceFrame.GeometryBasis).GetNormal(false);
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
        WideArithmetic.SubtractSigned832(WideDirection3d.Dot(Normal, point), PlaneConstant).IsZero;

    internal void GetTriangleEdgeWall(FixedTriangle triangle, int edgeIndex,
        out Signed576 x, out Signed576 y, out Signed576 z, out Signed832 offset)
    {
        FixedSegment edge = triangle.GetEdge(edgeIndex);
        GetEdgeLine(edge, out x, out y, out z, out offset);
        WideAxis3 opposite = Transform(edgeIndex == 0 ? triangle.C : edgeIndex == 1 ? triangle.A : triangle.B);
        if (WideArithmetic.SubtractSigned832(DotWall(x, y, z, opposite), offset).Sign < 0)
        {
            x = WideArithmetic.SubtractSigned576(default, x);
            y = WideArithmetic.SubtractSigned576(default, y);
            z = WideArithmetic.SubtractSigned576(default, z);
            offset = WideArithmetic.SubtractSigned832(default, offset);
        }
    }

    // The sign of an exact line equation is immaterial for its roots. Segment
    // sources therefore need no invented opposite vertex or enclosing triangle.
    internal void GetEdgeLine(FixedSegment edge,
        out Signed576 x, out Signed576 y, out Signed576 z, out Signed832 offset)
    {
        if (Finite.ShapeFrame.HasExactScale)
        {
            WideDirection3d direction = WideDirection3d.GetLocalEdge(SourceFrame, edge.Start, edge.End);
            WideDirection3d scaledWall = WideDirection3d.Cross(new WideDirection3d(authoredNormal), direction)
                .Transform(Finite.ShapeFrame.Basis);
            x = scaledWall.X; y = WideArithmetic.SubtractSigned576(default, scaledWall.Y); z = scaledWall.Z;
            offset = DotWall(x, y, z, Transform(edge.Start));
            return;
        }
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

    private static Signed832 DotWall(Signed576 x, Signed576 y, Signed576 z, WideAxis3 point)
    {
        // Each transformed authored wall component <326 and point <198;
        // three signed products need <526 bits, safely inside Signed576.
        return WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(x, point.X),
                WideArithmetic.MultiplySigned576ToSigned832(y, point.Y)),
            WideArithmetic.MultiplySigned576ToSigned832(z, point.Z));
    }
}
