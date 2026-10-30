//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>A single exact cylinder chart shared by every feature of a convex hull.</summary>
internal readonly ref struct CylinderHullSupport
{
    private readonly ReadOnlySpan<Vector3d> points;
    internal readonly CylinderPolytopeFrame Frame;
    private readonly Signed320 divisor;
    internal readonly Signed320 RawScale;
    internal readonly Signed576 HalfHeight, Radius;
    internal readonly WideRationalBasis3d WorldBasis;
    internal readonly int ValueShift;
    internal readonly bool IsAffine;
    internal bool IsEmpty => points.IsEmpty;
    internal int Count => points.Length;

    internal CylinderHullSupport(in ScaledRigidFrame3d source, ReadOnlySpan<Vector3d> points,
        Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius)
    {
        this.points = points;
        IsAffine = !source.IsRigid;
        Frame = IsAffine ? new CylinderPolytopeFrame(source, center, rotation, Signed192.Raw(height), radius)
            : new CylinderPolytopeFrame(center, rotation, height, radius, source.RigidOrigin, source.Rotation);
        WorldBasis = new WideRationalBasis3d(rotation);
        Signed320 scale = WideArithmetic.AddSigned320(Frame.CoordinateDenominator, Frame.CoordinateDenominator);
        Signed320 radial = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(Frame.CoordinateDenominator, Frame.Radius));
        divisor = WideArithmetic.GetGreatestCommonDivisor(WideArithmetic.GetGreatestCommonDivisor(scale, Frame.Cap), radial);
        for (int i = 0; i < points.Length; i++)
        {
            WideAxis3 point = Frame.Transform(points[i]);
            divisor = WideArithmetic.GetGreatestCommonDivisor(divisor, point.X);
            divisor = WideArithmetic.GetGreatestCommonDivisor(divisor, point.Y);
            divisor = WideArithmetic.GetGreatestCommonDivisor(divisor, point.Z);
        }
        RawScale = WideArithmetic.DivideExactSigned320(scale, divisor);
        HalfHeight = Signed576.ExtendValue(WideArithmetic.DivideExactSigned320(Frame.Cap, divisor));
        Radius = Signed576.ExtendValue(WideArithmetic.DivideExactSigned320(radial, divisor));
        Span<Signed576> bounds = stackalloc Signed576[3] { HalfHeight, Radius, default };
        ValueShift = TriangleCircularGeometry.GetValueShift(bounds, 4);
        for (int i = 0; i < points.Length; i++)
        {
            WideDirection3d point = Vertex(i);
            bounds[0] = point.X; bounds[1] = point.Y; bounds[2] = point.Z;
            ValueShift = Math.Max(ValueShift, TriangleCircularGeometry.GetValueShift(bounds, 4));
        }
    }

    internal WideDirection3d Vertex(int index)
    {
        WideAxis3 point = Frame.Transform(points[index]);
        return new WideDirection3d(WideArithmetic.DivideExactSigned320(point.X, divisor),
            WideArithmetic.DivideExactSigned320(point.Y, divisor), WideArithmetic.DivideExactSigned320(point.Z, divisor));
    }

    internal bool Supports(in WideDirection3d point, ReadOnlySpan<ulong> direction, ReadOnlySpan<int> signs)
    {
        Span<ulong> projection = stackalloc ulong[Words];
        for (int i = 0; i < Count; i++)
        {
            Dot(TriangleCircularGeometry.Subtract(point, Vertex(i)), direction, signs, projection, out int sign);
            if (sign < 0) return false;
        }
        return true;
    }

    internal bool HasBallMinimumCertificate(in ScaledRigidFrame3d source, ReadOnlySpan<int> triangles,
        ConvexContactCandidate best)
    {
        // Hull contains B(d) and cylinder contains B(min(R,H)); their sum
        // bounds every exit below by d+min(R,H), with no rounded square root.
        Signed576 ball = WideArithmetic.SubtractSigned576(Radius, HalfHeight).Sign <= 0 ? Radius : HalfHeight;
        Span<ulong> direction = stackalloc ulong[3 * Words], values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> directionSigns = stackalloc int[3], signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> work = stackalloc ulong[6 * Words];
        values.Clear(); signs.Clear();
        for (int index = 0; index < triangles.Length; index += 3)
        {
            int a = triangles[index], b = triangles[index + 1], c = triangles[index + 2];
            var triangle = new FixedTriangle(points[a], points[b], points[c]);
            var geometry = new TriangleCircularGeometry(triangle, source, this, a, b, c);
            if (geometry.FaceNormal.IsZero) return false;
            WriteDirection(geometry.FaceNormal, direction, directionSigns);
            Dot(geometry.A, direction, directionSigns, Slot(work, 0), out int planeSign);
            if (planeSign < 0)
                for (int component = 0; component < 3; component++) directionSigns[component] = -directionSigns[component];
            if (!Supports(geometry.A, direction, directionSigns))
            {
                if (planeSign != 0) return false;
                for (int component = 0; component < 3; component++) directionSigns[component] = -directionSigns[component];
                if (!Supports(geometry.A, direction, directionSigns)) return false;
            }
            SumSquares(direction, Slot(work, 1));
            Import(ball, Slot(work, 2));
            WideArithmetic.MultiplyMagnitudes(Slot(work, 2), Slot(work, 2), Slot(work, 3));
            WideArithmetic.MultiplyMagnitudes(Slot(work, 1), Slot(work, 3), Slot(work, 4));
            Import(Signed576.ExtendValue(RawScale), Slot(work, 2));
            WideArithmetic.MultiplyMagnitudes(Slot(work, 2), Slot(work, 2), Slot(work, 3));
            WideArithmetic.MultiplyMagnitudes(Slot(work, 1), Slot(work, 3), Slot(work, 5));
            int gap = BuildRadialCandidate(Slot(work, 0), planeSign == 0 ? 0 : 1,
                Slot(work, 4), Slot(work, 5), values, signs);
            if (WideConvexPrismRelations.CompareConvexContactCandidates(new ConvexContactCandidate(values, signs, gap), best) < 0)
                return false;
        }
        return true;
    }
}
