//=======================================================================
// TriangleCircularGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Exact triangle coordinates and circular extents in a canonical +Y frame.</summary>
internal readonly struct TriangleCircularGeometry
{
    internal readonly WideRationalBasis3d WorldBasis;
    internal readonly WideAxis3 A, B, C, FaceNormal;
    internal readonly Signed320 HalfHeight, Radius;
    internal readonly Signed192 RawScale;
    internal readonly Signed192 EdgeScale;
    internal readonly int ValueShift;
    private readonly WideAxis3 firstEdge, secondEdge;

    internal TriangleCircularGeometry(FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
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
        // IsNormalized bounds each exact quaternion norm denominator by
        // Q^2+(EpsilonRaw+2)*Q: four rounded squares add at most two raw
        // units of error. Thus their product D<2^129. Rational bases are
        // exactly orthogonal, so an unreduced edge 2D*R*(Braw-Araw) has norm
        // <2*2^129*sqrt(3)*2^64<2^195 (and conservatively <2^196).
        // Translation cancels; common reduction can only shrink this bound.
        firstEdge = Subtract(B, A); secondEdge = Subtract(C, B);
        EdgeScale = Signed192.Signed(1);
        HalfHeight = coordinates[10]; Radius = coordinates[11];
        triangle.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        FaceNormal = WideRigidProjection.TransformLocalAxis(frame.Basis, nx, ny, nz);
        // Each quaternion denominator is <2^65. Relative transformed full-domain
        // points, including doubled coordinates and translated origin differences,
        // are <2^197. Edge/cap differences are <2^198. Squared residual values
        // are below 2^(2*(bits+4)); the common reduction only lowers these bounds.
        ValueShift = GetValueShift(coordinates[1..], 4);
    }

    private TriangleCircularGeometry(in TriangleCircularGeometry source, Signed192 multiplier, WideAxis3 coreOffset)
    {
        WorldBasis = source.WorldBasis; FaceNormal = source.FaceNormal;
        A = Multiply(source.A, multiplier);
        B = Multiply(source.B, multiplier);
        C = Multiply(source.C, multiplier);
        HalfHeight = Multiply(source.HalfHeight, multiplier); Radius = Multiply(source.Radius, multiplier);
        RawScale = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(source.RawScale, multiplier));
        EdgeScale = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(source.EdgeScale, multiplier));
        firstEdge = source.firstEdge; secondEdge = source.secondEdge;
        Span<Signed320> bounds = stackalloc Signed320[13]
        {
            A.X, A.Y, A.Z, B.X, B.Y, B.Z, C.X, C.Y, C.Z,
            HalfHeight, Radius, coreOffset.X, coreOffset.Z
        };
        // Include the actual endpoint offset: a long core can dominate a
        // small centered triangle even when its common denominator is small.
        // Two additions cover endpoint/cap translation; the extra margin
        // bounds the squared support residual in both regions by 2^ValueShift.
        ValueShift = GetValueShift(bounds, 5);
    }

    private TriangleCircularGeometry(in TriangleCircularGeometry source, WideAxis3 offset)
    {
        this = source;
        A = Add(source.A, offset); B = Add(source.B, offset); C = Add(source.C, offset);
    }

    internal TriangleCircularGeometry WithCore(Vector2d axis, Fixed64 length, out WideAxis3 coreOffset)
    {
        // The reduced cylinder scale need not retain its original factor two.
        // One common 2Q multiplier represents L*axis/(2Q) at either endpoint,
        // including odd raw lengths, without forming a rounded endpoint.
        Signed192 multiplier = Signed192.Raw(Fixed64.Two);
        Signed320 lengthScale = WideArithmetic.MultiplySigned192(RawScale, Signed192.Raw(length));
        coreOffset = new WideAxis3(Multiply(lengthScale, Signed192.Raw(axis.X)), default,
            Multiply(lengthScale, Signed192.Raw(axis.Y)));
        // Both endpoint regions retain one actual-bound-based value scale.
        return new TriangleCircularGeometry(this, multiplier, coreOffset);
    }

    internal TriangleCircularGeometry AtCoreRegion(WideAxis3 coreOffset, int sign) =>
        new(this, sign > 0 ? coreOffset : -coreOffset);

    internal WideAxis3 EdgeFromTo(int from, int to) => (from + 1) % 3 == to ? Edge(from) : -Edge(to);

    internal static Signed320 Multiply(Signed320 value, Signed192 multiplier) =>
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(value, multiplier));

    private static WideAxis3 Multiply(WideAxis3 value, Signed192 multiplier) => new(
        Multiply(value.X, multiplier), Multiply(value.Y, multiplier), Multiply(value.Z, multiplier));

    private static WideAxis3 Add(WideAxis3 first, WideAxis3 second) => new(
        WideArithmetic.AddSigned320(first.X, second.X), WideArithmetic.AddSigned320(first.Y, second.Y),
        WideArithmetic.AddSigned320(first.Z, second.Z));

    internal static int GetValueShift(ReadOnlySpan<Signed320> coordinates, int carryBits)
    {
        int bits = 0;
        Span<ulong> magnitude = stackalloc ulong[5];
        for (int index = 0; index < coordinates.Length; index++)
        {
            WideArithmetic.GetMagnitude(coordinates[index], out magnitude[4], out magnitude[3],
                out magnitude[2], out magnitude[1], out magnitude[0]);
            bits = Math.Max(bits, WideArithmetic.GetMagnitudeBitLength(magnitude));
        }
        return 2 * (bits + carryBits);
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
        WideAxis3 e = Edge(0), f = -Edge(2);
        Signed576 ee = e.SquaredLength, ff = f.SquaredLength, ef = WideAxis3.Dot(e, f);
        Signed576 d1 = WideArithmetic.SubtractSigned576(default, WideAxis3.Dot(A, e));
        Signed576 d2 = WideArithmetic.SubtractSigned576(default, WideAxis3.Dot(A, f));
        // Retained edges are <196 bits; scaled offsets are <232. Thus Gram
        // dots are <394, offset dots <430, and mixed minors <825. The actual
        // barycentric denominator is EdgeScale*det (not det); alpha stays
        // <827 bits, within Signed832 even in a positive-core region.
        Signed832 determinant = ProductDifference(ee, ff, ef, ef);
        Signed832 beta = ProductDifference(ff, d1, ef, d2);
        Signed832 gamma = ProductDifference(ee, d2, ef, d1);
        Signed832 alpha = WideArithmetic.SubtractSigned832(
            WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(determinant, EdgeScale), beta), gamma);
        return alpha.Sign >= 0 && beta.Sign >= 0 && gamma.Sign >= 0;
    }

    internal bool HasFaceDiskMinimumCertificate(WideAxis3 coreAxis, ConvexContactCandidate best)
    {
        if (!WideAxis3.Cross(FaceNormal, coreAxis).IsZero)
            return false;
        // The caller has tested +/-face with d>=0 and proved that its shape
        // contains the normal-axis segment [-s,s], with d<=s-|a| for plane
        // u.x=a. A face-plane disk(q,d), q=a*u, minus that segment contains
        // the origin ball(d). Its three inward edge clearances therefore
        // certify the retained d globally, without changing its earlier tie.
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> work = stackalloc ulong[6 * Words];
        Span<ulong> zero = Slot(work, 1), scale = Slot(work, 3), squaredScale = Slot(work, 4);
        values.Clear(); signs.Clear(); zero.Clear();
        Import(Signed320.ExtendValue(RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(scale, scale, squaredScale);
        for (int edge = 0; edge < 3; edge++)
        {
            // Use the parallel raw core axis, not the much wider transformed
            // face normal. Parallelism cross<295 bits; tangent<231, offset
            // dot<465 and tangent norm²<464 fit the existing fixed carriers.
            // Scale²*norm²<792 and numerator²<930 fit forty-word candidates.
            WideAxis3 tangent = WideAxis3.Cross(coreAxis, Edge(edge));
            int inward = WideAxis3.Dot(EdgeFromTo(edge, (edge + 2) % 3), tangent).Sign;
            Signed576 projection = WideAxis3.Dot(Vertex(edge), tangent);
            if (!HasFaceDiskClearance(projection, -inward * projection.Sign, tangent,
                    best, work, values, signs))
                return false;
        }
        return true;
    }

    // Shared by single-triangle disks and complete patch tubes. Work slot 1
    // is zero and slot 4 is the squared coordinate scale, prepared once.
    internal static bool HasFaceDiskClearance(Signed576 projection, int projectionSign,
        WideAxis3 tangent, ConvexContactCandidate best, Span<ulong> work,
        Span<ulong> values, Span<int> signs)
    {
        Import(projection, Slot(work, 0));
        Import(tangent.SquaredLength, Slot(work, 2));
        WideArithmetic.MultiplyMagnitudes(Slot(work, 2), Slot(work, 4), Slot(work, 5));
        int gapSign = BuildRadialCandidate(Slot(work, 0), projectionSign,
            Slot(work, 1), Slot(work, 5), values, signs);
        return WideConvexPrismRelations.CompareConvexContactCandidates(
            new ConvexContactCandidate(values, signs, gapSign), best) >= 0;
    }

    private static Signed832 ProductDifference(Signed576 a, Signed576 b, Signed576 c, Signed576 d) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(a, b),
            WideArithmetic.MultiplySigned576ToSigned832(c, d));

    /// <summary>Rounds a rational local normal's support gap, cancelling principal-axis scale exactly.</summary>
    internal void GetAnalyticDepth(ConvexContactCandidate candidate, ReadOnlySpan<int> directionSigns, int mask,
        out Fixed64 depth, out bool clamped)
    {
        int components = (directionSigns[0] == 0 ? 0 : 1)
            + (directionSigns[1] == 0 ? 0 : 1) + (directionSigns[2] == 0 ? 0 : 1);
        if (components != 1)
        {
            WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(candidate, Fixed64.Zero, out depth, out clamped);
            return;
        }
        // A single local component cancels its magnitude from the normalized
        // support gap. Every vertex in the selected mask has this coordinate.
        int axis = directionSigns[0] != 0 ? 0 : directionSigns[1] != 0 ? 1 : 2;
        int vertex = (mask & 1) != 0 ? 0 : (mask & 2) != 0 ? 1 : 2;
        Signed320 coordinate = Component(Vertex(vertex), axis);
        Signed320 numerator = WideArithmetic.AddSigned320(axis == 1 ? HalfHeight : Radius,
            directionSigns[axis] < 0 ? WideArithmetic.Negate(coordinate) : coordinate);
        // Shifted numerator and exact clamp threshold fit 233 bits. Test exact
        // overflow before nearest-even rounding, including Max+fractions.
        Signed320 maximum = WideArithmetic.MultiplySigned192(RawScale, Signed192.Raw(Fixed64.MaxValue));
        clamped = WideArithmetic.SubtractSigned320(numerator, maximum).Sign > 0;
        if (clamped)
        {
            depth = Fixed64.MaxValue;
            return;
        }
        bool represented = Fixed64.TryGetSignedRawRatio(Signed576.ExtendValue(numerator),
            Signed576.ExtendValue(Signed320.ExtendValue(RawScale)), out depth);
        System.Diagnostics.Debug.Assert(represented);
    }

    internal WideAxis3 Vertex(int index) => index == 0 ? A : index == 1 ? B : C;
    internal WideAxis3 Edge(int index) => index == 0 ? firstEdge : index == 1 ? secondEdge : -Add(firstEdge, secondEdge);
    internal WideAxis3 CapOffset(int vertex, int cap) => new(Vertex(vertex).X,
        WideArithmetic.AddSigned320(Vertex(vertex).Y, cap > 0 ? HalfHeight : WideArithmetic.Negate(HalfHeight)),
        Vertex(vertex).Z);
    internal static WideAxis3 Subtract(WideAxis3 first, WideAxis3 second) => new(
        WideArithmetic.SubtractSigned320(first.X, second.X),
        WideArithmetic.SubtractSigned320(first.Y, second.Y),
        WideArithmetic.SubtractSigned320(first.Z, second.Z));
    internal static FixedPointAnchor GetSupport(Vector3d center, FixedQuaternion rotation,
        Signed192 height, Vector3d radialPoint, int cap)
    {
        ulong floor = height.Low >> 1;
        Fixed64 halfHeight = Fixed64.FromRaw((long)(floor + ((height.Low & 1UL) != 0 && (floor & 1UL) != 0 ? 1UL : 0UL)));
        Vector3d axial = new(Fixed64.Zero, cap >= 0 ? -halfHeight : halfHeight, Fixed64.Zero);
        // Public cylinder heights fit one Fixed64 and can retain an odd raw
        // half-height. Slab heights may span 64 unsigned bits but are even,
        // because their authored half-thickness was representable and exact.
        FixedPointAnchorTerm3d term = height.Low <= long.MaxValue
            ? FixedPointAnchorTerm3d.CreateCenteredAxisSupport(Vector3d.Up,
                Fixed64.FromRaw(cap >= 0 ? -(long)height.Low : (long)height.Low),
                Vector3d.Zero, Fixed64.Zero, axial, Vector3d.Zero)
            : default;
        return new FixedPointAnchor(center, rotation, axial, radialPoint, term);
    }
}
