//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact triangle coordinates in the cylinder's canonical +Y frame.</summary>
internal readonly struct TriangleCylinderGeometry
{
    internal readonly WideRationalBasis3d WorldBasis;
    internal readonly WideAxis3 A, B, C, FaceNormal;
    internal readonly Signed320 HalfHeight, Radius;
    internal readonly Signed192 RawScale;
    internal readonly int ValueShift;

    internal TriangleCylinderGeometry(FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Signed192 height, Fixed64 radius)
    {
        WorldBasis = new WideRationalBasis3d(cylinderRotation);
        var frame = new CylinderPolytopeFrame(cylinderCenter, cylinderRotation,
            height, radius, origin, rotation);
        WideAxis3 a = frame.Transform(triangle.A), b = frame.Transform(triangle.B), c = frame.Transform(triangle.C);
        Span<Signed320> coordinates = stackalloc Signed320[12]
        {
            Signed320.ExtendValue(WideArithmetic.AddSigned192(frame.Denominator, frame.Denominator)),
            a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z,
            frame.Cap, WideArithmetic.MultiplySigned192(frame.Radius, frame.Denominator)
        };
        WideArithmetic.ReduceCommonScale(coordinates);
        RawScale = Signed192.NarrowProven(coordinates[0]);
        A = new WideAxis3(coordinates[1], coordinates[2], coordinates[3]);
        B = new WideAxis3(coordinates[4], coordinates[5], coordinates[6]);
        C = new WideAxis3(coordinates[7], coordinates[8], coordinates[9]);
        HalfHeight = coordinates[10]; Radius = coordinates[11];
        triangle.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        FaceNormal = WideRigidProjection.TransformLocalAxis(frame.Basis, nx, ny, nz);
        int bits = 0;
        Span<ulong> magnitude = stackalloc ulong[5];
        for (int index = 1; index < coordinates.Length; index++)
        {
            WideArithmetic.GetMagnitude(coordinates[index], out magnitude[4], out magnitude[3],
                out magnitude[2], out magnitude[1], out magnitude[0]);
            bits = Math.Max(bits, WideArithmetic.GetMagnitudeBitLength(magnitude));
        }
        // Each quaternion denominator is <2^65. Relative transformed full-domain
        // points, including doubled coordinates and translated origin differences,
        // are <2^197. Edge/cap differences are <2^198. Squared residual values
        // are below 2^(2*(bits+4)); the common reduction only lowers these bounds.
        ValueShift = 2 * (bits + 4);
    }

    /// <summary>
    /// After nonnegative cap and face support tests, certifies that their
    /// retained minimum is global without enumerating the remaining features.
    /// </summary>
    internal bool HasFaceMinimumCertificate()
    {
        if (FaceNormal.X.IsZero && FaceNormal.Z.IsZero)
        {
            Signed320 axial = A.Y.Sign < 0 ? WideArithmetic.Negate(A.Y) : A.Y;
            Signed320 depth = WideArithmetic.SubtractSigned320(HalfHeight, axial);
            if (WideArithmetic.SubtractSigned320(depth, Radius).Sign > 0)
                return false;
        }
        else if (!FaceNormal.Y.IsZero || WideArithmetic.SubtractSigned320(Radius, HalfHeight).Sign > 0)
            return false;

        // Let q be the origin's orthogonal projection onto the triangle plane.
        // A horizontal face has depth d=H-|q.Y|<=R; a vertical face has
        // d=R-|q.radial|<=R<=H. The caller has proved d>=0. If q is in the
        // triangle, the cylinder contains a radius-d ball centered at q.
        // Thus triangle-cylinder contains the origin-centered radius-d ball,
        // while the tested face support attains d: no other feature is smaller.
        WideAxis3 e = Subtract(B, A), f = Subtract(C, A);
        Signed576 ee = e.SquaredLength, ff = f.SquaredLength, ef = WideAxis3.Dot(e, f);
        Signed576 d1 = WideArithmetic.SubtractSigned576(default, WideAxis3.Dot(A, e));
        Signed576 d2 = WideArithmetic.SubtractSigned576(default, WideAxis3.Dot(A, f));
        // Coordinates/edge differences are <2^198, so dot products are
        // <2^398 and these Gram/barycentric minors are <2^800. Even the
        // three-term alpha numerator fits Signed832, without scalar rounding.
        Signed832 determinant = ProductDifference(ee, ff, ef, ef);
        Signed832 beta = ProductDifference(ff, d1, ef, d2);
        Signed832 gamma = ProductDifference(ee, d2, ef, d1);
        Signed832 alpha = WideArithmetic.SubtractSigned832(
            WideArithmetic.SubtractSigned832(determinant, beta), gamma);
        return alpha.Sign >= 0 && beta.Sign >= 0 && gamma.Sign >= 0;
    }

    private static Signed832 ProductDifference(Signed576 a, Signed576 b, Signed576 c, Signed576 d) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(a, b),
            WideArithmetic.MultiplySigned576ToSigned832(c, d));

    internal WideAxis3 Vertex(int index) => index == 0 ? A : index == 1 ? B : C;
    internal WideAxis3 Edge(int index) => Subtract(Vertex((index + 1) % 3), Vertex(index));
    internal WideAxis3 CapOffset(int vertex, int cap) => new(Vertex(vertex).X,
        WideArithmetic.AddSigned320(Vertex(vertex).Y, cap > 0 ? HalfHeight : WideArithmetic.Negate(HalfHeight)),
        Vertex(vertex).Z);
    internal static WideAxis3 Subtract(WideAxis3 first, WideAxis3 second) => new(
        WideArithmetic.SubtractSigned320(first.X, second.X),
        WideArithmetic.SubtractSigned320(first.Y, second.Y),
        WideArithmetic.SubtractSigned320(first.Z, second.Z));
}
