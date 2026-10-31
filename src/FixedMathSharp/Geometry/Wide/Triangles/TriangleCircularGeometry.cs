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
    internal readonly WideDirection3d A, B, C, FaceNormal;
    internal readonly Signed576 HalfHeight, Radius;
    internal readonly Signed320 RawScale;
    internal readonly bool IsAffine;
    internal int ParameterWords => IsAffine ? 80 : CylinderContactAlgebra.Words;
    internal int ValueWords => IsAffine ? 112 : CircularRimContactAlgebra.ValueWords;
    internal readonly Signed192 EdgeScale;
    internal readonly int ValueShift;
    private readonly WideDirection3d firstEdge, secondEdge;
    private readonly WideDirection3d materialOffset;

    internal TriangleCircularGeometry(FixedTriangle triangle, in ScaledRigidFrame3d sourceFrame,
        in CylinderHullSupport hull, int a, int b, int c)
    {
        materialOffset = default;
        IsAffine = hull.IsAffine; WorldBasis = hull.WorldBasis;
        A = hull.Vertex(a); B = hull.Vertex(b); C = hull.Vertex(c);
        RawScale = hull.RawScale; HalfHeight = hull.HalfHeight; Radius = hull.Radius;
        firstEdge = Subtract(B, A); secondEdge = Subtract(C, B);
        EdgeScale = Signed192.Signed(1); ValueShift = hull.ValueShift;
        WideDirection3d localNormal = WideDirection3d.Cross(
            WideDirection3d.GetLocalEdge(sourceFrame, triangle.A, triangle.B),
            WideDirection3d.GetLocalEdge(sourceFrame, triangle.A, triangle.C));
        FaceNormal = localNormal.Transform(hull.Frame.Basis);
    }

    internal TriangleCircularGeometry(FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Signed192 height, Fixed64 radius)
    {
        materialOffset = default;
        IsAffine = false;
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
        RawScale = coordinates[0];
        A = new WideDirection3d(coordinates[1], coordinates[2], coordinates[3]);
        B = new WideDirection3d(coordinates[4], coordinates[5], coordinates[6]);
        C = new WideDirection3d(coordinates[7], coordinates[8], coordinates[9]);
        // IsNormalized bounds each exact quaternion norm denominator by
        // Q^2+(EpsilonRaw+2)*Q: four rounded squares add at most two raw
        // units of error. Thus their product D<2^129. Rational bases are
        // exactly orthogonal, so an unreduced edge 2D*R*(Braw-Araw) has norm
        // <2*2^129*sqrt(3)*2^64<2^195 (and conservatively <2^196).
        // Translation cancels; common reduction can only shrink this bound.
        firstEdge = Subtract(B, A); secondEdge = Subtract(C, B);
        EdgeScale = Signed192.Signed(1);
        HalfHeight = Signed576.ExtendValue(coordinates[10]); Radius = Signed576.ExtendValue(coordinates[11]);
        triangle.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        FaceNormal = new WideDirection3d(WideRigidProjection.TransformLocalAxis(frame.Basis, nx, ny, nz));
        // Each quaternion denominator is <2^65. Relative transformed full-domain
        // points, including doubled coordinates and translated origin differences,
        // are <2^197. Edge/cap differences are <2^198. Squared residual values
        // are below 2^(2*(bits+4)); the common reduction only lowers these bounds.
        ValueShift = GetValueShift(coordinates[1..], 4);
    }

    internal TriangleCircularGeometry(FixedTriangle triangle, in ScaledRigidFrame3d sourceFrame,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Signed192 height, Fixed64 radius)
    {
        // Entry points dispatch rigid frames before constructing affine geometry.
        if (height.Sign < 0 || WideArithmetic.SubtractSigned192(height, Signed192.Raw((Fixed64)8192)).Sign > 0)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero || radius > (Fixed64)4096)
            throw new ArgumentOutOfRangeException(nameof(radius));
        materialOffset = default;
        IsAffine = true;
        WorldBasis = new WideRationalBasis3d(cylinderRotation);
        var frame = new CylinderPolytopeFrame(sourceFrame, cylinderCenter, cylinderRotation, height, radius);
        WideAxis3 a = frame.Transform(triangle.A), b = frame.Transform(triangle.B), c = frame.Transform(triangle.C);
        Span<Signed320> coordinates = stackalloc Signed320[12]
        {
            WideArithmetic.AddSigned320(frame.CoordinateDenominator, frame.CoordinateDenominator),
            a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z,
            frame.Cap, Signed320.NarrowValue(WideArithmetic.MultiplySigned320(frame.CoordinateDenominator, frame.Radius))
        };
        WideArithmetic.ReduceCommonScale(coordinates);
        RawScale = coordinates[0];
        A = new WideDirection3d(coordinates[1], coordinates[2], coordinates[3]);
        B = new WideDirection3d(coordinates[4], coordinates[5], coordinates[6]);
        C = new WideDirection3d(coordinates[7], coordinates[8], coordinates[9]);
        HalfHeight = Signed576.ExtendValue(coordinates[10]); Radius = Signed576.ExtendValue(coordinates[11]);
        firstEdge = Subtract(B, A); secondEdge = Subtract(C, B);
        EdgeScale = Signed192.Signed(1);
        WideDirection3d localNormal = WideDirection3d.Cross(
            WideDirection3d.GetLocalEdge(sourceFrame, triangle.A, triangle.B),
            WideDirection3d.GetLocalEdge(sourceFrame, triangle.A, triangle.C));
        FaceNormal = localNormal.Transform(frame.Basis);
        ValueShift = GetValueShift(coordinates[1..], 4);
    }

    private TriangleCircularGeometry(in TriangleCircularGeometry source, Signed192 multiplier, in WideDirection3d coreOffset,
        int valueShift = 0)
    {
        materialOffset = Multiply(source.materialOffset, multiplier);
        IsAffine = source.IsAffine;
        WorldBasis = source.WorldBasis; FaceNormal = source.FaceNormal;
        A = Multiply(source.A, multiplier);
        B = Multiply(source.B, multiplier);
        C = Multiply(source.C, multiplier);
        HalfHeight = Multiply(source.HalfHeight, multiplier); Radius = Multiply(source.Radius, multiplier);
        RawScale = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(source.RawScale, multiplier));
        EdgeScale = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(source.EdgeScale, multiplier));
        firstEdge = source.firstEdge; secondEdge = source.secondEdge;
        Span<Signed576> bounds = stackalloc Signed576[13]
        {
            A.X, A.Y, A.Z, B.X, B.Y, B.Z, C.X, C.Y, C.Z,
            HalfHeight, Radius, coreOffset.X, coreOffset.Z
        };
        // Include the actual endpoint offset: a long core can dominate a
        // small centered triangle even when its common denominator is small.
        // Two additions cover endpoint/cap translation; the extra margin
        // bounds the squared support residual in both regions by 2^ValueShift.
        ValueShift = Math.Max(valueShift, GetValueShift(bounds, 5));
    }

    internal TriangleCircularGeometry WithCoreChart(in TriangleCircularGeometry chart) =>
        new(this, chart.EdgeScale, default, chart.ValueShift);

    internal TriangleCircularGeometry WithValueShift(int valueShift)
    {
        return new TriangleCircularGeometry(this, Signed192.Signed(1), default, valueShift);
    }

    internal WideDirection3d TransformHullVertex(in WideDirection3d vertex)
    {
        WideDirection3d point = EdgeScale.Equals(Signed192.Signed(1)) ? vertex : Multiply(vertex, EdgeScale);
        return materialOffset.IsZero ? point : Add(point, materialOffset);
    }

    internal int GetSupportMask(ReadOnlySpan<ulong> direction, ReadOnlySpan<int> signs)
    {
        Span<ulong> projection = stackalloc ulong[Words];
        int best = 0, mask = 1;
        for (int i = 1; i < 3; i++)
        {
            Dot(Subtract(Vertex(i), Vertex(best)), direction, signs, projection, out int sign);
            if (sign > 0) { best = i; mask = 1 << i; }
            else if (sign == 0) mask |= 1 << i;
        }
        return mask;
    }

    private TriangleCircularGeometry(in TriangleCircularGeometry source, in WideDirection3d offset)
    {
        this = source;
        materialOffset = Add(source.materialOffset, offset);
        A = Add(source.A, offset); B = Add(source.B, offset); C = Add(source.C, offset);
    }

    // Affine contacts round once in the physical circular frame. Rounding
    // authored coordinates first would amplify one raw unit by the source scale.
    internal Signed576 WitnessCoordinate(FixedTriangle triangle, int vertex, int component)
    {
        if (IsAffine)
            return WideArithmetic.SubtractSigned576(Component(Vertex(vertex), component), Component(materialOffset, component));
        Vector3d point = vertex == 0 ? triangle.A : vertex == 1 ? triangle.B : triangle.C;
        return Signed576.ExtendValue(Signed192.Raw(component == 0 ? point.X : component == 1 ? point.Y : point.Z));
    }

    internal Vector3d WitnessVertex(FixedTriangle triangle, int vertex)
    {
        if (!IsAffine) return vertex == 0 ? triangle.A : vertex == 1 ? triangle.B : triangle.C;
        Signed320 denominator = RawScale;
        return new Vector3d(
            RoundWitnessRatio(WitnessCoordinate(triangle, vertex, 0), denominator),
            RoundWitnessRatio(WitnessCoordinate(triangle, vertex, 1), denominator),
            RoundWitnessRatio(WitnessCoordinate(triangle, vertex, 2), denominator));
    }

    private static Fixed64 RoundWitnessRatio(Signed576 numerator, Signed320 denominator)
    {
        Span<ulong> n = stackalloc ulong[9], d = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(numerator, n);
        WideArithmetic.GetMagnitude(denominator, out d[4], out d[3], out d[2], out d[1], out d[0]);
        bool represented = Fixed64.TryGetSignedRawRatio(n, d, numerator.Sign < 0, out Fixed64 result);
        System.Diagnostics.Debug.Assert(represented);
        return result;
    }

    internal TriangleCircularGeometry WithCore(Vector2d axis, Fixed64 length, out WideDirection3d coreOffset)
    {
        // The reduced cylinder scale need not retain its original factor two.
        // One common 2Q multiplier represents L*axis/(2Q) at either endpoint,
        // including odd raw lengths, without forming a rounded endpoint.
        // Affine callers provide admitted finite cores (length <=8192 and
        // unit-axis components); the scalar path retains its full-domain input.
        Signed192 multiplier = Signed192.Raw(Fixed64.Two);
        Signed576 lengthScale = WideArithmetic.MultiplySigned320(RawScale, Signed192.Raw(length));
        coreOffset = new WideDirection3d(Multiply(lengthScale, Signed192.Raw(axis.X)), default,
            Multiply(lengthScale, Signed192.Raw(axis.Y)));
        if (IsAffine)
        {
            // Cancel the same factor from 2Q and both endpoint offsets before
            // enlarging coordinates. All points, extents, RawScale and EdgeScale
            // then use the reduced multiplier, so no endpoint is rounded.
            // Width proof for the admitted affine frame: before either gcd,
            // coordinates need at most 306 bits; 2Q adds 33. The largest
            // projected squared gap needs at most 2608 bits: edge differences
            // need 307, endpoint/cap sums 341, e^2*v-(e.v)*e at most 959,
            // support dots at most 1302, then squaring/addition fits 2608.
            // If the earlier
            // coordinate gcd is G, g=log2(G), that degree-eight expression loses
            // 8g bits. Its original RawScale contains 2Q^2, hence the endpoint
            // offsets retain at least max(0,65-g) factors of two. This gcd removes
            // f>=min(33,max(0,65-g)) more; the degree-four dependence on scaled
            // points loses 4f bits. Thus 2608-8g-4f<=2476, below the existing
            // 2560-bit analytic bank. No rotation-denominator cancellation is
            // assumed: a translated core need not share such a factor.
            Span<Signed576> factors = stackalloc Signed576[3]
            { coreOffset.X, coreOffset.Z, Signed576.ExtendValue(multiplier) };
            WideArithmetic.ReduceCommonScale(factors);
            coreOffset = new(factors[0], default, factors[1]);
            multiplier = Signed192.NarrowProven(Signed320.NarrowValue(factors[2]));
        }
        // Both endpoint regions retain one actual-bound-based value scale.
        return new TriangleCircularGeometry(this, multiplier, coreOffset);
    }

    internal TriangleCircularGeometry AtCoreRegion(in WideDirection3d coreOffset, int sign) =>
        new(this, sign > 0 ? coreOffset : -coreOffset);

    internal WideDirection3d EdgeFromTo(int from, int to) => (from + 1) % 3 == to ? Edge(from) : -Edge(to);

    internal static Signed576 Multiply(Signed576 value, Signed192 multiplier) =>
        WideArithmetic.MultiplySigned576(value, multiplier);

    private static WideDirection3d Multiply(in WideDirection3d value, Signed192 multiplier) => new(
        Multiply(value.X, multiplier), Multiply(value.Y, multiplier), Multiply(value.Z, multiplier));

    private static WideDirection3d Add(in WideDirection3d first, in WideDirection3d second) => new(
        WideArithmetic.AddSigned576(first.X, second.X), WideArithmetic.AddSigned576(first.Y, second.Y),
        WideArithmetic.AddSigned576(first.Z, second.Z));

    internal static int GetValueShift(ReadOnlySpan<Signed576> coordinates, int carryBits)
    {
        int bits = 0;
        Span<ulong> magnitude = stackalloc ulong[9];
        for (int index = 0; index < coordinates.Length; index++)
        {
            WideArithmetic.GetMagnitude(coordinates[index], magnitude);
            bits = Math.Max(bits, WideArithmetic.GetMagnitudeBitLength(magnitude));
        }
        return 2 * (bits + carryBits);
    }

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
            Signed576 axial = A.Y.Sign < 0 ? WideArithmetic.Negate(A.Y) : A.Y;
            Signed576 depth = WideArithmetic.SubtractSigned576(HalfHeight, axial);
            if (WideArithmetic.SubtractSigned576(depth, Radius).Sign > 0)
                return false;
        }
        else if (!FaceNormal.Y.IsZero || WideArithmetic.SubtractSigned576(Radius, HalfHeight).Sign > 0)
            return false;

        // Let q be the origin's orthogonal projection onto the triangle plane.
        // A horizontal face has depth d=H-|q.Y|<=R; a vertical face has
        // d=R-|q.radial|<=R<=H. The caller has proved d>=0. If q is in the
        // triangle, the cylinder contains a radius-d ball centered at q.
        // Thus triangle-cylinder contains the origin-centered radius-d ball,
        // while the tested face support attains d: no other feature is smaller.
        WideDirection3d e = Edge(0), f = -Edge(2);
        Signed832 ee = e.SquaredLength, ff = f.SquaredLength, ef = WideDirection3d.Dot(e, f);
        Signed832 d1 = WideArithmetic.SubtractSigned832(default, WideDirection3d.Dot(A, e));
        Signed832 d2 = WideArithmetic.SubtractSigned832(default, WideDirection3d.Dot(A, f));
        // Retained edges are <196 bits; scaled offsets are <232. Thus Gram
        // dots are <394, offset dots <430, and mixed minors <825. The actual
        // barycentric denominator is EdgeScale*det (not det); alpha stays
        // <827 bits, within Signed832 even in a positive-core region.
        if (IsAffine)
            return HasNonnegativeGramWeights(ee, ff, ef, d1, d2);
        Signed832 determinant = ProductDifference(Signed576.NarrowValue(ee), Signed576.NarrowValue(ff),
            Signed576.NarrowValue(ef), Signed576.NarrowValue(ef));
        Signed832 beta = ProductDifference(Signed576.NarrowValue(ff), Signed576.NarrowValue(d1),
            Signed576.NarrowValue(ef), Signed576.NarrowValue(d2));
        Signed832 gamma = ProductDifference(Signed576.NarrowValue(ee), Signed576.NarrowValue(d2),
            Signed576.NarrowValue(ef), Signed576.NarrowValue(d1));
        Signed832 alpha = WideArithmetic.SubtractSigned832(
            WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(determinant, EdgeScale), beta), gamma);
        return alpha.Sign >= 0 && beta.Sign >= 0 && gamma.Sign >= 0;
    }

    private bool HasNonnegativeGramWeights(Signed832 ee, Signed832 ff, Signed832 ef, Signed832 d1, Signed832 d2)
    {
        Span<ulong> values = stackalloc ulong[20 * Words]; Span<int> signs = stackalloc int[20];
        ContactQuadratic a = ContactQuadratic.At(values, signs, 0), b = ContactQuadratic.At(values, signs, 1),
            c = ContactQuadratic.At(values, signs, 2), d = ContactQuadratic.At(values, signs, 3),
            e = ContactQuadratic.At(values, signs, 4), determinant = ContactQuadratic.At(values, signs, 5),
            beta = ContactQuadratic.At(values, signs, 6), gamma = ContactQuadratic.At(values, signs, 7),
            alpha = ContactQuadratic.At(values, signs, 8), term = ContactQuadratic.At(values, signs, 9);
        a.Set(ee); b.Set(ff); c.Set(ef); d.Set(d1); e.Set(d2);
        ContactQuadratic.Multiply(a, b, ReadOnlySpan<ulong>.Empty, determinant);
        ContactQuadratic.Multiply(c, c, ReadOnlySpan<ulong>.Empty, term); determinant.Add(term, -1);
        ContactQuadratic.Multiply(b, d, ReadOnlySpan<ulong>.Empty, beta);
        ContactQuadratic.Multiply(c, e, ReadOnlySpan<ulong>.Empty, term); beta.Add(term, -1);
        ContactQuadratic.Multiply(a, e, ReadOnlySpan<ulong>.Empty, gamma);
        ContactQuadratic.Multiply(c, d, ReadOnlySpan<ulong>.Empty, term); gamma.Add(term, -1);
        ContactQuadratic.Scale(determinant, Signed576.ExtendValue(Signed320.ExtendValue(EdgeScale)), alpha);
        alpha.Add(beta, -1); alpha.Add(gamma, -1);
        return alpha.Signs[0] >= 0 && beta.Signs[0] >= 0 && gamma.Signs[0] >= 0;
    }

    internal bool HasFaceDiskMinimumCertificate(in WideDirection3d coreAxis, ConvexContactCandidate best)
    {
        if (!WideDirection3d.Cross(FaceNormal, coreAxis).IsZero)
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
        Import(Signed576.ExtendValue(RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(scale, scale, squaredScale);
        for (int edge = 0; edge < 3; edge++)
        {
            // Use the parallel raw core axis, not the much wider transformed
            // face normal. Parallelism cross<295 bits; tangent<231, offset
            // dot<465 and tangent norm²<464 fit the existing fixed carriers.
            // Scale²*norm²<792 and numerator²<930 fit forty-word candidates.
            WideDirection3d tangent = WideDirection3d.Cross(coreAxis, Edge(edge));
            int inward = WideDirection3d.Dot(EdgeFromTo(edge, (edge + 2) % 3), tangent).Sign;
            Signed832 projection = WideDirection3d.Dot(Vertex(edge), tangent);
            if (!HasFaceDiskClearance(projection, -inward * projection.Sign, tangent,
                    best, work, values, signs))
                return false;
        }
        return true;
    }

    // Shared by single-triangle disks and complete patch tubes. Work slot 1
    // is zero and slot 4 is the squared coordinate scale, prepared once.
    internal static bool HasFaceDiskClearance(Signed832 projection, int projectionSign,
        in WideDirection3d tangent, ConvexContactCandidate best, Span<ulong> work,
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
        Signed576 coordinate = Component(Vertex(vertex), axis);
        Signed576 numerator = WideArithmetic.AddSigned576(axis == 1 ? HalfHeight : Radius,
            directionSigns[axis] < 0 ? WideArithmetic.Negate(coordinate) : coordinate);
        // Shifted numerator and exact clamp threshold fit 233 bits. Test exact
        // overflow before nearest-even rounding, including Max+fractions.
        Signed576 maximum = WideArithmetic.MultiplySigned320(RawScale, Signed192.Raw(Fixed64.MaxValue));
        clamped = WideArithmetic.SubtractSigned576(numerator, maximum).Sign > 0;
        if (clamped)
        {
            depth = Fixed64.MaxValue;
            return;
        }
        bool represented = Fixed64.TryGetSignedRawRatio(numerator, Signed576.ExtendValue(RawScale), out depth);
        System.Diagnostics.Debug.Assert(represented);
    }

    internal WideDirection3d Vertex(int index) => index == 0 ? A : index == 1 ? B : C;
    internal WideDirection3d Edge(int index) => index == 0 ? firstEdge : index == 1 ? secondEdge : -Add(firstEdge, secondEdge);
    internal WideDirection3d CapOffset(int vertex, int cap) => new(Vertex(vertex).X,
        WideArithmetic.AddSigned576(Vertex(vertex).Y, cap > 0 ? HalfHeight : WideArithmetic.Negate(HalfHeight)),
        Vertex(vertex).Z);
    internal static WideDirection3d Subtract(in WideDirection3d first, in WideDirection3d second) => new(
        WideArithmetic.SubtractSigned576(first.X, second.X),
        WideArithmetic.SubtractSigned576(first.Y, second.Y),
        WideArithmetic.SubtractSigned576(first.Z, second.Z));
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
