//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>One canonical source-plane metric shared by all admitted finite domains.</summary>
internal readonly struct FiniteSlabRayFrame
{
    internal readonly FiniteSlabSectionFrame Section;
    internal readonly CapsulePlaneRayFrame Plane;
    internal readonly Vector3d Origin, Center;
    internal readonly FixedQuaternion Rotation, SlabRotation;
    internal readonly Signed320 Radius, CoreCap;
    internal readonly bool HasCore;
    internal readonly FixedTriangle SourcePlane;
    internal readonly WideDirection3d RayNormal;
    internal readonly Signed832 RayNormalSquared;
    private readonly bool _wideParameters;
    internal bool UsesWideBanks => Section.RadialFrame.ShapeFrame.HasExactScale || _wideParameters;
    internal int FieldWords => UsesWideBanks ? FiniteSlabPolynomial.AffineWords : FiniteSlabPolynomial.Words;
    internal int SelectionStorageWords => FiniteSlabRaySelection.GetStorageWords(FieldWords);
    internal FiniteSlabRayFrame WithWideParameters() => UsesWideBanks ? this : new FiniteSlabRayFrame(this);
    private FiniteSlabRayFrame(in FiniteSlabRayFrame source) { this = source; _wideParameters = true; }
    // HalfThickness retains the supplied extent; FullHeight is zero for authored half-thickness slabs.
    internal readonly Fixed64 RadiusValue, HalfThickness, FullHeight;

    internal FiniteSlabRayFrame(FixedTriangle sourcePlane, Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness, bool fullHeight = false)
        : this(sourcePlane, origin, rotation, center, slabRotation, length, radius, halfThickness, fullHeight, default, false)
    { }

    internal FiniteSlabRayFrame(FixedTriangle sourcePlane, in ScaledRigidFrame3d sourceFrame,
        Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness, bool fullHeight = false)
        : this(sourcePlane, sourceFrame.RigidOrigin, sourceFrame.Rotation, center, slabRotation, length, radius,
            halfThickness, fullHeight, sourceFrame, !sourceFrame.IsRigid)
    { }

    private FiniteSlabRayFrame(FixedTriangle sourcePlane, Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness,
        bool fullHeight, in ScaledRigidFrame3d sourceFrame, bool exact)
    {
        _wideParameters = false;
        Section = exact ? new(sourceFrame, center, slabRotation, length, radius, halfThickness, fullHeight)
            : new(origin, rotation, center, slabRotation, length, radius, halfThickness, fullHeight);
        Plane = exact ? new(sourcePlane, sourceFrame, Section.RadialFrame) : new(sourcePlane, Section.RadialFrame);
        Origin = origin; Rotation = rotation; Center = center; SlabRotation = slabRotation;
        Radius = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
            Section.RadialFrame.ShapeFrame.CoordinateDenominator, Section.RadialFrame.ShapeFrame.Radius));
        CoreCap = Section.RadialFrame.ShapeFrame.Cap;
        HasCore = length != Fixed64.Zero;
        SourcePlane = sourcePlane;
        RadiusValue = radius; HalfThickness = halfThickness; FullHeight = fullHeight ? halfThickness : Fixed64.Zero;
        // Match the established plane-ray variable conditioning: numerator
        // coordinates include the rigid frame denominator, so an unscaled
        // primitive normal can make ordinary t enormous and u nearly one.
        Signed320 bound = WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(Radius, CoreCap), Section.Cap);
        Span<ulong> magnitude = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(Signed576.ExtendValue(bound), magnitude);
        int boundBits = WideArithmetic.GetMagnitudeBitLength(magnitude);
        WideArithmetic.GetMagnitude(Plane.NormalSquared, magnitude);
        int normalPower = (WideArithmetic.GetMagnitudeBitLength(magnitude) - 1) / 2;
        int shift = Math.Max(0, boundBits + 2 - normalPower);
        // The resulting components need at most max(original bits,bound+3),
        // below 268 rigid or 352 affine bits; conditioning never adds a
        // fixed shift to an already maximal normal.
        RayNormal = exact ? new(ScalePowerOfTwo(Plane.Normal.X, shift), ScalePowerOfTwo(Plane.Normal.Y, shift),
            ScalePowerOfTwo(Plane.Normal.Z, shift))
            : new WideDirection3d(new WideAxis3(ScalePowerOfTwo(Signed320.NarrowValue(Plane.Normal.X), shift),
                ScalePowerOfTwo(Signed320.NarrowValue(Plane.Normal.Y), shift),
                ScalePowerOfTwo(Signed320.NarrowValue(Plane.Normal.Z), shift)));
        RayNormalSquared = RayNormal.SquaredLength;
    }

    internal Vector3d GetNormal(int orientation) =>
        Plane.GetWorldNormal(Rotation) * (Fixed64)(-orientation);

    /// <summary>Affine dimension of the shape face maximizing the oriented plane functional.</summary>
    internal int GetSupportDimension(ReadOnlySpan<Vector2d> prismOffsets, int orientation)
    {
        if (orientation != -1 && orientation != 1) throw new ArgumentOutOfRangeException(nameof(orientation));
        // A zero normal coefficient leaves that factor of the extrusion free.
        // The stadium disk and core share the X/Z plane, whose dimension is at most two.
        int height = Plane.Normal.Y.IsZero && Section.Cap.Sign > 0 ? 1 : 0;
        bool axial = Plane.Normal.X.IsZero && Plane.Normal.Z.IsZero;
        if (prismOffsets.IsEmpty)
        {
            int footprint = axial && RadiusValue > Fixed64.Zero ? 2
                : HasCore && Plane.Normal.Z.IsZero ? 1 : 0;
            return height + footprint;
        }
        _ = FiniteSlabSectionFrame.GetPrismWinding(prismOffsets);
        if (axial) return height + 2;
        Signed576 maximum = default; Vector2d vertex = default; bool tied = false;
        for (int index = 0; index < prismOffsets.Length; index++)
        {
            Vector2d offset = prismOffsets[index];
            Signed576 projection = WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(Plane.Normal.X, offset.X.m_rawValue),
                WideArithmetic.MultiplySigned576(Plane.Normal.Z, offset.Y.m_rawValue));
            if (orientation < 0) projection = WideArithmetic.SubtractSigned576(default, projection);
            int order = WideArithmetic.SubtractSigned576(projection, maximum).Sign;
            if (index == 0 || order > 0) { maximum = projection; vertex = offset; tied = false; }
            else if (order == 0 && offset != vertex) tied = true;
        }
        return height + (tied ? 1 : 0);
    }

    private static Signed320 ScalePowerOfTwo(Signed320 value, int shift)
    {
        while (shift > 0)
        {
            int step = Math.Min(63, shift);
            WideArithmetic.ShiftLeft(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0, step,
                out ulong w4, out ulong w3, out ulong w2, out ulong w1, out ulong w0);
            value = new(w4, w3, w2, w1, w0); shift -= step;
        }
        return value;
    }

    private static Signed576 ScalePowerOfTwo(Signed576 value, int shift)
    {
        while (shift > 0)
        {
            int step = Math.Min(62, shift);
            value = WideArithmetic.MultiplySigned576(value, 1L << step);
            shift -= step;
        }
        return value;
    }
}

/// <summary>
/// Borrowed polynomial arithmetic for this owner's affine and conic charts.
/// All limb operations remain in the existing wide arithmetic owner.
/// </summary>
internal readonly ref struct FiniteSlabPolynomial
{
    internal const int Words = 64;
    internal const int AffineWords = 128;
    internal int FieldWords => Values.Length / Signs.Length;
    internal readonly Span<ulong> Values;
    internal readonly Span<sbyte> Signs;
    internal int Count => Signs.Length;
    internal bool IsZero { get { foreach (sbyte sign in Signs) if (sign != 0) return false; return true; } }
    internal FiniteSlabPolynomial(Span<ulong> values, Span<sbyte> signs)
    { Values = values; Signs = signs; }
    internal static FiniteSlabPolynomial At(Span<ulong> values, Span<sbyte> signs, int index, int count) =>
        new(values.Slice(index * (values.Length / signs.Length), count * (values.Length / signs.Length)), signs.Slice(index, count));
    internal FiniteSlabPolynomial Slice(int index, int count) => At(Values, Signs, index, count);
    internal void Clear() { Values.Clear(); Signs.Clear(); }
    internal void Set(Signed576 value, int index = 0)
    {
        Span<ulong> slot = Values.Slice(index * FieldWords, FieldWords);
        WideArithmetic.GetMagnitude(value, slot); Signs[index] = (sbyte)value.Sign;
    }
    internal void Set(Signed832 value, int index = 0)
    {
        Span<ulong> slot = Values.Slice(index * FieldWords, FieldWords);
        WideArithmetic.GetMagnitude(value, slot); Signs[index] = (sbyte)value.Sign;
    }
    internal void Set(Signed320 value, int index = 0) => Set(Signed576.ExtendValue(value), index);
    internal void Set(int value, int index = 0) => Set(Signed320.ExtendValue(Signed192.Signed(value)), index);
    internal void Copy(FiniteSlabPolynomial source)
    {
        Clear();
        if (FieldWords == source.FieldWords)
        {
            source.Values.CopyTo(Values); source.Signs.CopyTo(Signs);
        }
        else Add(source);
    }
    internal void Add(FiniteSlabPolynomial source, int multiplier = 1, int offset = 0) =>
        WideFiniteAxisIntersection.AddFiniteAxisPolynomial(source.Values, source.Signs,
            Values, Signs, multiplier, offset);
    internal void Multiply(FiniteSlabPolynomial left, FiniteSlabPolynomial right)
    {
        Span<ulong> product = FieldWords == Words ? stackalloc ulong[Words] : stackalloc ulong[AffineWords];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(left.Values, left.Signs,
            right.Values, right.Signs, Values, Signs, product);
    }
    internal void Scale(FiniteSlabPolynomial source, Signed576 factor)
    {
        if (factor.IsZero) { Clear(); return; }
        // Polynomial coefficients are integers; these units are literal 1,
        // rather than the Q32.32 value retained by Signed576.One.
        if ((factor.Word0 == 1UL && factor.Equals(Signed576.ExtendValue(Signed192.Signed(1))))
            || (factor.Word0 == ulong.MaxValue && factor.Equals(Signed576.ExtendValue(Signed192.Signed(-1)))))
        {
            Clear(); Add(source, factor.Sign);
            return;
        }
        Span<ulong> data = FieldWords == Words ? stackalloc ulong[Words] : stackalloc ulong[AffineWords];
        Span<sbyte> signs = stackalloc sbyte[1];
        FiniteSlabPolynomial scalar = new(data, signs); scalar.Set(factor);
        Multiply(source, scalar);
    }
    internal void Scale(FiniteSlabPolynomial source, Signed832 factor)
    {
        if (factor.IsZero) { Clear(); return; }
        if ((factor.Word0 == 1UL && factor.Equals(Signed832.ExtendValue(Signed192.Signed(1))))
            || (factor.Word0 == ulong.MaxValue && factor.Equals(Signed832.ExtendValue(Signed192.Signed(-1)))))
        {
            Clear(); Add(source, factor.Sign);
            return;
        }
        Span<ulong> data = FieldWords == Words ? stackalloc ulong[Words] : stackalloc ulong[AffineWords];
        Span<sbyte> signs = stackalloc sbyte[1];
        FiniteSlabPolynomial scalar = new(data, signs); scalar.Set(factor);
        Multiply(source, scalar);
    }
    internal void Scale(FiniteSlabPolynomial source, Signed320 factor) => Scale(source, Signed576.ExtendValue(factor));
    internal static int Sign(scoped ref FiniteAxisValueRoot root, scoped FiniteSlabPolynomial query)
    {
        int degree = query.Count - 1;
        while (degree > 0 && query.Signs[degree] == 0) degree--;
        if (degree == 0) return query.Signs[0];
        // A common positive scale cannot change a sign. Strip it from a
        // scratch copy, never from a retained coordinate/denominator ratio;
        // otherwise exact-zero queries pay a needlessly large equality bound.
        Span<ulong> values = stackalloc ulong[query.Values.Length]; query.Values.CopyTo(values);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values, query.Signs);
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, values, query.Signs);
    }
}

internal delegate void FiniteSlabRayCandidateVisitor(scoped in FiniteSlabRaySelection candidate);

/// <summary>Finite-domain clearance of one flat-capped planar convex primitive.</summary>
internal static class FiniteSlabRayExtrema
{
    // Width ledger: exact rigid points/edges <198 bits; conditioned normal
    // components <268; prism linear rows <334; chart radial coordinates <532;
    // conic coefficients <1065; PlaneDisk elimination retains quadratic
    // coefficients <2133. The single radial constraint produces only
    // quadratic roots; reconstructed point coefficients and final depth/sign
    // products remain <3400 bits.
    // Rigid calls retain the 64-word bank. For exact affine ingress, the
    // full clipping rows have <832 bits and coordinates/rays <352. A plane
    // chart has <1186-bit coordinates, <2374-bit conics and <4751-bit
    // eliminated quadratics. Reconstructed witnesses stay <3563 bits;
    // their squared physical metric stays <7834, within 128 words.
    private const int Words = FiniteSlabPolynomial.Words;
    private readonly struct Linear
    {
        internal readonly Signed832 A, B, C, D;
        internal Linear(Signed832 a, Signed832 b, Signed832 c, Signed832 d)
        { A = a; B = b; C = c; D = d; }
        internal Linear(Signed576 a, Signed576 b, Signed576 c, Signed576 d)
            : this(Signed832.ExtendValue(a), Signed832.ExtendValue(b), Signed832.ExtendValue(c), Signed832.ExtendValue(d)) { }
        internal Signed832 this[int index] => index == 0 ? A : index == 1 ? B : index == 2 ? C : D;
    }
    private readonly struct Radial
    {
        internal readonly WideAxis3 A, E, F;
        internal readonly WideDirection3d N;
        internal readonly Signed320 Radius;
        internal Radial(WideAxis3 a, WideAxis3 e, WideAxis3 f, WideDirection3d n, Signed320 radius)
        { A = a; E = e; F = f; N = n; Radius = radius; }
    }
    private readonly ref struct Problem
    {
        internal readonly ReadOnlySpan<Linear> Lines;
        internal readonly ReadOnlySpan<Radial> Disks;
        internal readonly FixedTriangle Triangle;
        internal readonly int Orientation;
        internal readonly FiniteSlabRayCandidateVisitor? Visitor;
        private readonly ReadOnlySpan<Radial> _shape;
        internal ref readonly Radial Shape => ref _shape[0];
        internal readonly Signed320 HalfThickness, CoreCap;
        internal readonly bool HasCore, IsPrism;
        internal readonly int ExitLinearStart;
        internal readonly int DomainEdgeMask;
        internal readonly int ActiveFirst, ActiveSecond, ActiveThird, SolvedDisks;
        internal readonly int DomainLineCount;
        internal readonly int ExitRegion;
        internal readonly int FieldWords;
        internal Problem(ReadOnlySpan<Linear> lines, ReadOnlySpan<Radial> disks,
            FixedTriangle triangle, int orientation, FiniteSlabRayCandidateVisitor? visitor,
            ReadOnlySpan<Radial> shape, Signed320 halfThickness, Signed320 coreCap, bool hasCore, bool isPrism, int exitLinearStart,
            int domainEdgeMask = 0, int activeFirst = -1, int activeSecond = -1, int activeThird = -1, int solvedDisks = 0,
            int domainLineCount = 3, int exitRegion = 0, int fieldWords = Words)
        {
            Lines = lines; Disks = disks; Triangle = triangle; Orientation = orientation; Visitor = visitor;
            _shape = shape; HalfThickness = halfThickness; CoreCap = coreCap; HasCore = hasCore;
            IsPrism = isPrism; ExitLinearStart = exitLinearStart;
            DomainEdgeMask = domainEdgeMask;
            ActiveFirst = activeFirst; ActiveSecond = activeSecond; ActiveThird = activeThird; SolvedDisks = solvedDisks;
            DomainLineCount = domainLineCount;
            ExitRegion = exitRegion; FieldWords = fieldWords;
        }
        internal Problem WithActive(int first, int second = -1, int third = -1) => new(Lines, Disks, Triangle, Orientation, Visitor,
            _shape, HalfThickness, CoreCap, HasCore, IsPrism, ExitLinearStart,
            DomainEdge(first, DomainLineCount) | DomainEdge(second, DomainLineCount) | DomainEdge(third, DomainLineCount),
            first, second, third, SolvedDisks, DomainLineCount, ExitRegion, FieldWords);
        internal Problem WithDisk(int disk) => new(Lines, Disks, Triangle, Orientation, Visitor,
            _shape, HalfThickness, CoreCap, HasCore, IsPrism, ExitLinearStart, DomainEdgeMask,
            ActiveFirst, ActiveSecond, ActiveThird, SolvedDisks | (1 << disk), DomainLineCount, ExitRegion, FieldWords);
        internal Problem WithoutSolvedDisks() => new(Lines, Disks, Triangle, Orientation, Visitor,
            _shape, HalfThickness, CoreCap, HasCore, IsPrism, ExitLinearStart, DomainEdgeMask,
            ActiveFirst, ActiveSecond, ActiveThird, domainLineCount: DomainLineCount,
            exitRegion: ExitRegion, fieldWords: FieldWords);
        internal bool IsActive(int index) => index == ActiveFirst || index == ActiveSecond || index == ActiveThird;
    }

    /// <summary>
    /// Accumulates the maximum nonnegative intersection parameter q+tN,
    /// where q belongs to the finite triangle and q+tN belongs to the slab.
    /// The source q need not intersect the slab at parameter zero. Empty offsets
    /// select a circle/stadium; nonempty offsets select a convex prism. The
    /// plane metric belongs to frame, never to the domain triangle's winding
    /// or cross-product magnitude. Candidate callbacks are synchronous.
    /// </summary>
    internal static void AccumulateTriangle(in FiniteSlabRayFrame frame, FixedTriangle triangle,
        ReadOnlySpan<Vector2d> prismOffsets, int orientation, ref FiniteSlabRaySelection maximum,
        FiniteSlabRayCandidateVisitor? visitor = null) =>
        AccumulateDomain(frame, triangle, prismOffsets, orientation, true, false, ref maximum, visitor);

    /// <summary>Clips the original segment parameter without rounding either rational cut point.</summary>
    internal static void AccumulateSegment(in FiniteSlabRayFrame frame, FixedSegment segment, in RationalSegmentInterval clip,
        ReadOnlySpan<Vector2d> prismOffsets, int orientation, ref FiniteSlabRaySelection maximum,
        FiniteSlabRayCandidateVisitor? visitor = null)
    {
        FixedTriangle triangle = new(segment.Start, segment.End, segment.End);
        if (!clip.IsClipped) { AccumulateTriangle(frame, triangle, prismOffsets, orientation, ref maximum, visitor); return; }
        Span<RationalSegmentInterval> clips = stackalloc RationalSegmentInterval[1] { clip };
        AccumulateDomain(frame, triangle, prismOffsets, orientation, true, false, ref maximum, visitor, clips);
    }

    /// <summary>
    /// Clips physical support charts to a nondegenerate coplanar triangle.
    /// Nonaxial faces with a fixed Y cap use their point/line charts; other
    /// families retain general clipping. Compare with the unbounded plane
    /// peak to certify admission; a lower stationary root is not support.
    /// </summary>
    internal static void AccumulatePhysicalSupportTriangle(in FiniteSlabRayFrame frame, FixedTriangle triangle,
        ReadOnlySpan<Vector2d> prismOffsets, int orientation, ref FiniteSlabRaySelection maximum,
        FiniteSlabRayCandidateVisitor? visitor = null) =>
        AccumulateDomain(frame, triangle, prismOffsets, orientation, true, true, ref maximum, visitor);

    /// <summary>
    /// Accumulates the intrinsic slab/plane family without any finite domain
    /// inequalities. Hosts must independently certify its retained source
    /// projected clearance footprint is covered before using this in a finite
    /// mesh domain. Original-section coverage suffices only for axial rays.
    /// Axial consumers with representable boundary poles may omit the redundant
    /// interior sample; otherwise it remains a potentially sole representable witness.
    /// </summary>
    internal static void AccumulatePlane(in FiniteSlabRayFrame frame,
        ReadOnlySpan<Vector2d> prismOffsets, int orientation, ref FiniteSlabRaySelection maximum,
        FiniteSlabRayCandidateVisitor? visitor = null, bool includeInteriorSample = true) =>
        AccumulateDomain(frame, frame.SourcePlane, prismOffsets, orientation, false, false, ref maximum, visitor,
            ReadOnlySpan<RationalSegmentInterval>.Empty, includeInteriorSample);

    private static void AccumulateDomain(in FiniteSlabRayFrame frame, FixedTriangle triangle,
        ReadOnlySpan<Vector2d> prismOffsets, int orientation, bool finiteDomain, bool physicalSupportOnly,
        ref FiniteSlabRaySelection maximum, FiniteSlabRayCandidateVisitor? visitor) =>
        AccumulateDomain(frame, triangle, prismOffsets, orientation, finiteDomain, physicalSupportOnly, ref maximum, visitor,
            ReadOnlySpan<RationalSegmentInterval>.Empty);

    private static void AccumulateDomain(in FiniteSlabRayFrame frame, FixedTriangle triangle,
        ReadOnlySpan<Vector2d> prismOffsets, int orientation, bool finiteDomain, bool physicalSupportOnly,
        ref FiniteSlabRaySelection maximum, FiniteSlabRayCandidateVisitor? visitor, scoped ReadOnlySpan<RationalSegmentInterval> clips,
        bool includeInteriorSample = true)
    {
        if (orientation != -1 && orientation != 1) throw new ArgumentOutOfRangeException(nameof(orientation));
        if (frame.Plane.NormalSquared.IsZero) return;
        WideAxis3 a = frame.Section.Transform(triangle.A);
        WideAxis3 e = Difference(frame.Section.Transform(triangle.B), a);
        WideAxis3 f = Difference(frame.Section.Transform(triangle.C), a);
        WideDirection3d n = Signed(frame.RayNormal, orientation);
        bool axial = n.X.IsZero && n.Z.IsZero;
        if (!finiteDomain && axial)
        {
            AccumulateHorizontalPlane(frame, triangle, prismOffsets, orientation, a, e, f, n, ref maximum, visitor, includeInteriorSample);
            return;
        }
        int pieces = prismOffsets.IsEmpty && frame.HasCore ? 3 : 1;
        int capacity = 10 + prismOffsets.Length + (clips.IsEmpty ? 0 : 2);
        Span<Linear> lines = stackalloc Linear[capacity]; Span<Radial> disks = stackalloc Radial[1];
        // Every active-set view borrows the same immutable shape for this call.
        Span<Radial> shape = stackalloc Radial[1] { new(a, e, f, n, frame.Radius) };
        for (int exitPiece = 0; exitPiece < pieces; exitPiece++)
        {
            int count = 0, diskCount = 0;
            if (finiteDomain)
            {
                lines[count++] = new(S(1), default, default, default);
                lines[count++] = new(default, S(1), default, default);
                lines[count++] = new(S(-1), S(-1), default, S(1));
            }
            if (!clips.IsEmpty)
            {
                ref readonly RationalSegmentInterval clip = ref clips[0];
                // In the degenerate triangle A,B,B the original parameter is
                // lambda+mu. Cut numerators/positive denominators are <319 bits,
                // below the existing <334-bit prism rows. Active sets still
                // use at most three linear rows: LP determinants <1005 bits,
                // radial chart coefficients <1065, eliminated quadratics
                // <2133 and final witness/sign products <3400 fit the existing
                // 4096-bit retained fields. Constraint count adds no width.
                lines[count++] = new(clip.WideLowerDenominator, clip.WideLowerDenominator, default, Negate(clip.WideLowerNumerator));
                lines[count++] = new(Negate(clip.WideUpperDenominator), Negate(clip.WideUpperDenominator), default, clip.WideUpperNumerator);
            }
            int domainLineCount = count;
            lines[count++] = new(default, default, S(1), default);
            int exitLinearStart = count;
            // Clearance concerns every finite-domain source point that can
            // meet the translated slab. Restricting q to the original slab
            // misses points entering while it moves, and under-corrects even
            // an ordinary tall cylinder against a complete tilted plane.
            AddInterval(lines, ref count, a.Y, e.Y, f.Y, n.Y, frame.Section.Cap);
            if (!prismOffsets.IsEmpty)
            {
                int winding = FiniteSlabSectionFrame.GetPrismWinding(prismOffsets);
                Signed320 scale = WideArithmetic.AddSigned320(frame.Section.RadialFrame.ShapeFrame.CoordinateDenominator,
                    frame.Section.RadialFrame.ShapeFrame.CoordinateDenominator);
                for (int edge = 0; edge < prismOffsets.Length; edge++)
                {
                    Vector2d first = prismOffsets[edge], last = prismOffsets[(edge + 1) % prismOffsets.Length];
                    Signed320 dx = Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(last.X), Signed192.Raw(first.X)));
                    Signed320 dz = Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(last.Y), Signed192.Raw(first.Y)));
                    WideAxis3 offset = new(WideArithmetic.SubtractSigned320(a.X, ScaledOffset(first.X, scale)),
                        a.Y, WideArithmetic.SubtractSigned320(a.Z, ScaledOffset(first.Y, scale)));
                    lines[count++] = new(Signed(Wall(e, dx, dz), winding), Signed(Wall(f, dx, dz), winding),
                        Signed(Wall(n, dx, dz), winding), Signed(Wall(offset, dx, dz), winding));
                }
            }
            else if (frame.HasCore && exitPiece == 0)
            {
                AddInterval(lines, ref count, a.X, e.X, f.X, n.X, frame.Radius);
                AddInterval(lines, ref count, a.Z, e.Z, f.Z, n.Z, frame.CoreCap);
            }
            else
            {
                Signed320 center = frame.HasCore ? Signed(frame.CoreCap, exitPiece == 1 ? -1 : 1) : default;
                WideAxis3 offset = new(a.X, a.Y, WideArithmetic.SubtractSigned320(a.Z, center));
                if (frame.Radius.IsZero)
                {
                    AddInterval(lines, ref count, offset.X, e.X, f.X, n.X, default);
                    AddInterval(lines, ref count, offset.Z, e.Z, f.Z, n.Z, default);
                }
                else disks[diskCount++] = new(offset, e, f, n, frame.Radius);
            }
            Problem problem = new(lines[..count], disks[..diskCount], triangle, orientation, visitor,
                shape, frame.Section.Cap, frame.CoreCap, frame.HasCore,
                !prismOffsets.IsEmpty, exitLinearStart, domainLineCount: domainLineCount,
                exitRegion: frame.HasCore && exitPiece != 0 ? exitPiece == 1 ? -1 : 1 : 0, fieldWords: frame.FieldWords);
            if (!finiteDomain || physicalSupportOnly && !axial && !n.Y.IsZero)
                AccumulatePhysicalSupport(problem, frame, prismOffsets, exitPiece, ref maximum);
            else if (axial) AccumulateAxial(problem, n.Y.Sign, ref maximum);
            else Accumulate(problem, ref maximum);
        }
    }

    private static void AccumulatePhysicalSupport(scoped Problem problem, in FiniteSlabRayFrame frame,
        ReadOnlySpan<Vector2d> offsets, int exitPiece, ref FiniteSlabRaySelection maximum)
    {
        WideDirection3d n = problem.Shape.N;
        if (offsets.IsEmpty && frame.HasCore)
        {
            if (exitPiece == 0 && !n.Z.IsZero) return;
            if (exitPiece != 0 && !n.Z.IsZero && n.Z.Sign != problem.ExitRegion) return;
            if (problem.DomainLineCount != 0 && n.Z.IsZero && exitPiece != 0) return;
        }
        // Without a finite domain, support of the extrusion is the product
        // of an outward Y cap and footprint support; free Y retains both caps.
        int firstCap = problem.ExitLinearStart + (n.Y.Sign > 0 ? 1 : 0);
        int lastCap = n.Y.IsZero ? firstCap + 1 : firstCap;
        Signed576 footprintMaximum = default;
        int firstVertex = 0, lastVertex = 0;
        if (!offsets.IsEmpty)
            for (int vertex = 0; vertex < offsets.Length; vertex++)
            {
                Signed576 projection = FootprintProjection(n, offsets[vertex]);
                int order = WideArithmetic.SubtractSigned576(projection, footprintMaximum).Sign;
                if (vertex == 0 || order > 0)
                { footprintMaximum = projection; firstVertex = lastVertex = vertex; }
                else if (order == 0)
                {
                    Vector2d offset = offsets[vertex], first = offsets[firstVertex], last = offsets[lastVertex];
                    if (offset.X < first.X || offset.X == first.X && offset.Y < first.Y) firstVertex = vertex;
                    if (offset.X > last.X || offset.X == last.X && offset.Y > last.Y) lastVertex = vertex;
                }
            }
        int supportWall = -1;
        // Finite calls enter this owner only when the source normal has a Y
        // component; axial and free-Y families retain the general domain path.
        if (!offsets.IsEmpty && problem.DomainLineCount != 0 && firstVertex != lastVertex)
            for (int edge = 0; edge < offsets.Length; edge++)
            {
                Vector2d first = offsets[edge], last = offsets[(edge + 1) % offsets.Length];
                if (first == last) continue;
                if (WideArithmetic.SubtractSigned576(FootprintProjection(n, first), footprintMaximum).Sign != 0
                    || WideArithmetic.SubtractSigned576(FootprintProjection(n, last), footprintMaximum).Sign != 0) continue;
                supportWall = problem.ExitLinearStart + 2 + edge;
                break;
            }
        for (int capIndex = firstCap; capIndex <= lastCap; capIndex++)
        {
            ref readonly Linear cap = ref problem.Lines[capIndex];
            if (!offsets.IsEmpty)
            {
                if (supportWall >= 0)
                {
                    AccumulateSupportLine(problem, capIndex, supportWall, ref maximum);
                    continue;
                }
                Signed320 scale = WideArithmetic.AddSigned320(frame.Section.RadialFrame.ShapeFrame.CoordinateDenominator,
                    frame.Section.RadialFrame.ShapeFrame.CoordinateDenominator);
                // A convex footprint's tied support is one straight face;
                // its lexical endpoints omit duplicate/collinear authored samples.
                for (int vertex = 0; vertex < (firstVertex == lastVertex ? 1 : 2); vertex++)
                {
                    Vector2d offset = offsets[vertex == 0 ? firstVertex : lastVertex];
                    Linear x = new(S(problem.Shape.E.X), S(problem.Shape.F.X), S(n.X),
                        S(WideArithmetic.SubtractSigned320(problem.Shape.A.X,
                            ScaledOffset(offset.X, scale))));
                    Linear z = new(S(problem.Shape.E.Z), S(problem.Shape.F.Z), S(n.Z),
                        S(WideArithmetic.SubtractSigned320(problem.Shape.A.Z,
                            ScaledOffset(offset.Y, scale))));
                    // Direct coordinates retain authored collinear/duplicate
                    // corners whose adjacent wall equations are dependent.
                    ThreePlanes(problem.WithActive(capIndex), cap, x, z, ref maximum);
                }
            }
            else if (frame.HasCore && exitPiece == 0)
            {
                int xIndex = problem.ExitLinearStart + 2 + (n.X.Sign > 0 ? 1 : 0);
                if (problem.DomainLineCount != 0)
                {
                    AccumulateSupportLine(problem, capIndex, xIndex, ref maximum);
                    continue;
                }
                for (int zIndex = problem.ExitLinearStart + 4; zIndex < problem.ExitLinearStart + 6; zIndex++)
                    ThreePlanes(problem.WithActive(capIndex, xIndex, zIndex),
                        cap, problem.Lines[xIndex], problem.Lines[zIndex], ref maximum);
            }
            else if (frame.Radius.IsZero)
            {
                int xIndex = problem.ExitLinearStart + 2, zIndex = problem.ExitLinearStart + 4;
                ThreePlanes(problem.WithActive(capIndex, xIndex, zIndex),
                    cap, problem.Lines[xIndex], problem.Lines[zIndex], ref maximum);
            }
            else
            {
                Radial disk = problem.Disks[0];
                // The existing cap-constrained stationary chart supplies the
                // radial support pole without enumerating other active strata.
                PlaneDisk(problem.WithActive(capIndex).WithDisk(0), cap, disk, ref maximum);
            }
        }
    }

    private static void AccumulateSupportLine(scoped Problem problem, int cap, int wall,
        ref FiniteSlabRaySelection maximum)
    {
        // Cap and support wall fix a constant-depth line. Its clipped ends
        // occur on an existing footprint/domain constraint, including tangency.
        for (int third = 0; third < problem.Lines.Length; third++)
        {
            if (third == cap || third == wall) continue;
            ThreePlanes(problem.WithActive(cap, wall, third), problem.Lines[cap],
                problem.Lines[wall], problem.Lines[third], ref maximum);
        }
    }

    private static Signed576 FootprintProjection(WideDirection3d n, Vector2d offset) => WideArithmetic.AddSigned576(
        WideArithmetic.MultiplySigned576(n.X, offset.X.m_rawValue),
        WideArithmetic.MultiplySigned576(n.Z, offset.Y.m_rawValue));

    private static void AccumulateHorizontalPlane(in FiniteSlabRayFrame frame, FixedTriangle triangle,
        ReadOnlySpan<Vector2d> offsets, int orientation, WideAxis3 a, WideAxis3 e, WideAxis3 f, WideDirection3d n,
        ref FiniteSlabRaySelection maximum, FiniteSlabRayCandidateVisitor? visitor, bool includeInteriorSample)
    {
        // The intrinsic horizontal section is the complete footprint at one
        // Y coordinate. Every point has the same rational clearance depth;
        // canonical footprint poles/corners cover the constant-depth family
        // directly, without solving redundant circle/line quadratics.
        Span<Linear> lines = stackalloc Linear[3]; int count = 0;
        if (!offsets.IsEmpty) _ = FiniteSlabSectionFrame.GetPrismWinding(offsets);
        lines[count++] = new(default, default, S(1), default);
        AddInterval(lines, ref count, a.Y, e.Y, f.Y, n.Y, frame.Section.Cap);
        int capIndex = 1 + (n.Y.Sign > 0 ? 1 : 0);
        Span<Radial> shape = stackalloc Radial[1] { new(a, e, f, n, frame.Radius) };
        Problem problem = new(lines, ReadOnlySpan<Radial>.Empty, triangle, orientation, visitor,
            shape, frame.Section.Cap, frame.CoreCap, frame.HasCore,
            !offsets.IsEmpty, 1, activeFirst: capIndex, domainLineCount: 0, fieldWords: frame.FieldWords);
        int samples = !offsets.IsEmpty ? offsets.Length : (frame.HasCore ? 6 : 4) + (includeInteriorSample ? 1 : 0);
        Signed320 scale = WideArithmetic.AddSigned320(frame.Section.RadialFrame.ShapeFrame.CoordinateDenominator,
            frame.Section.RadialFrame.ShapeFrame.CoordinateDenominator);
        for (int index = 0; index < samples; index++)
        {
            Signed320 x = default, z = default;
            if (!offsets.IsEmpty)
            {
                x = ScaledOffset(offsets[index].X, scale);
                z = ScaledOffset(offsets[index].Y, scale);
            }
            else if (frame.HasCore)
            {
                if (index < 4)
                { x = Signed(frame.Radius, index % 2 == 0 ? -1 : 1); z = Signed(frame.CoreCap, index < 2 ? -1 : 1); }
                else if (index < 6)
                    z = Signed(WideArithmetic.AddSigned320(frame.CoreCap, frame.Radius), index == 4 ? -1 : 1);
            }
            else if (index < 4)
            {
                if (index < 2) x = Signed(frame.Radius, index == 0 ? -1 : 1);
                else z = Signed(frame.Radius, index == 2 ? -1 : 1);
            }
            Linear qx = new(S(e.X), S(f.X), default, S(WideArithmetic.SubtractSigned320(a.X, x)));
            Linear qz = new(S(e.Z), S(f.Z), default, S(WideArithmetic.SubtractSigned320(a.Z, z)));
            ThreePlanes(problem, lines[capIndex], qx, qz, ref maximum);
        }
    }

    private static void AddInterval(Span<Linear> lines, ref int count,
        Signed320 a, Signed320 e, Signed320 f, Signed576 n, Signed320 cap)
    {
        lines[count++] = new(S(e), S(f), S(n), S(WideArithmetic.AddSigned320(a, cap)));
        lines[count++] = new(S(Negate(e)), S(Negate(f)), S(Signed(n, -1)), S(WideArithmetic.SubtractSigned320(cap, a)));
    }

    private static void Accumulate(scoped Problem problem, ref FiniteSlabRaySelection maximum)
    {
        // There is at most one radial quadratic. An isolated maximum has
        // three active planes, two planes plus that quadratic, or one plane
        // plus its constrained stationary point. A free radial family has a
        // null direction preserving depth; the finite Y caps/domain force it
        // to one of these real planes, so no separate zero-plane chart is needed.
        for (int first = 0; first < problem.Lines.Length; first++)
        {
            ref readonly Linear a = ref problem.Lines[first];
            for (int second = first + 1; second < problem.Lines.Length; second++)
            {
                ref readonly Linear b = ref problem.Lines[second];
                for (int third = second + 1; third < problem.Lines.Length; third++)
                    ThreePlanes(problem.WithActive(first, second, third),
                        a, b, problem.Lines[third], ref maximum);
                for (int disk = 0; disk < problem.Disks.Length; disk++)
                    TwoPlanesDisk(problem.WithActive(first, second).WithDisk(disk),
                        a, b, problem.Disks[disk], ref maximum);
            }
            for (int disk = 0; disk < problem.Disks.Length; disk++)
                PlaneDisk(problem.WithActive(first).WithDisk(disk), a, problem.Disks[disk], ref maximum);
        }
    }

    private static void AccumulateAxial(scoped Problem problem, int direction,
        ref FiniteSlabRaySelection maximum)
    {
        // Every feasible ray leaves through the outward cap. Enumerating only
        // active sets containing it retains all clipped-region extrema and
        // cardinal witnesses, without solving irrelevant radial exit charts.
        int capIndex = problem.ExitLinearStart + (direction > 0 ? 1 : 0);
        ref readonly Linear cap = ref problem.Lines[capIndex];
        for (int first = 0; first < problem.Lines.Length; first++)
        {
            if (first == capIndex) continue;
            for (int second = first + 1; second < problem.Lines.Length; second++)
            {
                if (second == capIndex) continue;
                ThreePlanes(problem.WithActive(capIndex, first, second),
                    cap, problem.Lines[first], problem.Lines[second], ref maximum);
            }
            for (int disk = 0; disk < problem.Disks.Length; disk++)
                TwoPlanesDisk(problem.WithActive(capIndex, first).WithDisk(disk), cap, problem.Lines[first], problem.Disks[disk], ref maximum);
        }
        for (int disk = 0; disk < problem.Disks.Length; disk++)
            PlaneDisk(problem.WithActive(capIndex).WithDisk(disk), cap, problem.Disks[disk], ref maximum);
    }

    private static void ThreePlanes(scoped Problem problem, in Linear a, in Linear b, in Linear c,
        ref FiniteSlabRaySelection maximum)
    {
        int words = problem.FieldWords;
        Span<ulong> values = words == Words ? stackalloc ulong[8 * Words] : stackalloc ulong[8 * FiniteSlabPolynomial.AffineWords];
        Span<sbyte> signs = stackalloc sbyte[8];
        FiniteSlabPolynomial l = P(values, signs, 0), m = P(values, signs, 1),
            t = P(values, signs, 2), d = P(values, signs, 3), n = P(values, signs, 4), u = P(values, signs, 5, 2);
        Determinant(d, a, b, c, 0, 1, 2);
        if (d.IsZero) return;
        Determinant(l, a, b, c, 3, 1, 2); l.Signs[0] *= -1;
        Determinant(m, a, b, c, 0, 3, 2); m.Signs[0] *= -1;
        Determinant(t, a, b, c, 0, 1, 3); t.Signs[0] *= -1;
        if (d.Signs[0] < 0) { l.Signs[0] *= -1; m.Signs[0] *= -1; t.Signs[0] *= -1; d.Signs[0] = 1; }
        if (t.Signs[0] < 0) return;
        // The LP witness is rational. Its source has no algebraic parameter;
        // retain a known half-unit root and the independent rational depth.
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values[..(4 * words)], signs[..4]);
        n.Copy(t); n.Add(d); u.Clear(); u.Add(t); u.Add(d, 2);
        ProcessRational(problem, l, m, t, d, n, u.Slice(0, 1), ref maximum);
    }

    private static void Determinant(FiniteSlabPolynomial result, in Linear a, in Linear b, in Linear c,
        int x, int y, int z)
    {
        // Rigid rows retain their original 576-bit factors and scratch. Only
        // affine coordinates or wide clips require complete 832-bit factors.
        bool compact = result.FieldWords == Words;
        int limbs = compact ? 9 : 13;
        Span<ulong> values = compact ? stackalloc ulong[72] : stackalloc ulong[104];
        Span<ulong> first = values[..limbs], second = values.Slice(limbs, limbs), third = values.Slice(2 * limbs, limbs),
            pair = values.Slice(3 * limbs, 2 * limbs), term = values.Slice(5 * limbs, 3 * limbs);
        result.Clear();
        int sign = 0;
        for (int permutation = 0; permutation < 6; permutation++)
        {
            int i = permutation % 3;
            int j = permutation < 3 ? (i + 1) % 3 : (i + 2) % 3;
            int k = 3 - i - j;
            Signed832 av = a[i == 0 ? x : i == 1 ? y : z], bv = b[j == 0 ? x : j == 1 ? y : z],
                cv = c[k == 0 ? x : k == 1 ? y : z];
            // Sparse cap/footprint rows have many exact zero products; they
            // need no polynomial scratch initialization or multiplication.
            if (av.IsZero || bv.IsZero || cv.IsZero) continue;
            if (compact)
            {
                WideArithmetic.GetMagnitude(Signed576.NarrowValue(av), first);
                WideArithmetic.GetMagnitude(Signed576.NarrowValue(bv), second);
                WideArithmetic.GetMagnitude(Signed576.NarrowValue(cv), third);
            }
            else
            {
                WideArithmetic.GetMagnitude(av, first); WideArithmetic.GetMagnitude(bv, second); WideArithmetic.GetMagnitude(cv, third);
            }
            WideArithmetic.MultiplyMagnitudes(first, second, pair);
            WideArithmetic.MultiplyMagnitudes(pair, third, term);
            WideArithmetic.AddShiftedSignedMagnitude(term, av.Sign * bv.Sign * cv.Sign * (permutation < 3 ? 1 : -1),
                0, result.Values, ref sign);
        }
        result.Signs[0] = (sbyte)sign;
    }

    private static void Process(scoped Problem problem, scoped FiniteSlabPolynomial equation,
        scoped FiniteSlabPolynomial l, scoped FiniteSlabPolynomial m, scoped FiniteSlabPolynomial t,
        scoped FiniteSlabPolynomial d, bool parameterIsDepth,
        scoped FiniteSlabPolynomial depthN, scoped FiniteSlabPolynomial depthD,
        ref FiniteSlabRaySelection maximum)
    {
        int words = problem.FieldWords;
        if (equation.IsZero) return;
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(equation.Values, equation.Signs);
        Span<ulong> cells = stackalloc ulong[64]; Span<int> shifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(equation.Values, equation.Signs, cells, shifts);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(equation.Values, equation.Signs)];
        Span<ulong> candidateValues = stackalloc ulong[problem.Visitor is null ? 0 : FiniteSlabRaySelection.GetStorageWords(words)];
        Span<sbyte> candidateSigns = stackalloc sbyte[problem.Visitor is null ? 0 : FiniteSlabRaySelection.SignCount];
        Span<ulong> half = stackalloc ulong[1] { 1 };
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, equation.Values, equation.Signs, cell);
            // The depth chart maps t>=0 to u>=1/2. Reject its negative
            // branch using the already isolated cell before full admission.
            if (parameterIsDepth && WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root, half, 1) < 0) continue;
            if (!Admitted(problem, ref root, l, m, t, d)) continue;
            maximum.Keep(root, roots.Count, l, m, t, d, parameterIsDepth, depthN, depthD,
                problem.Triangle, problem.Orientation, problem.DomainEdgeMask);
            if (problem.Visitor is not null && IsClearanceExit(problem, ref root, l, m, t, d))
            {
                FiniteSlabRaySelection candidate = new(candidateValues, candidateSigns, words);
                candidate.Keep(root, roots.Count, l, m, t, d, parameterIsDepth, depthN, depthD,
                    problem.Triangle, problem.Orientation, problem.DomainEdgeMask);
                problem.Visitor(candidate);
            }
        }
    }

    private static void ProcessRational(scoped Problem problem,
        scoped FiniteSlabPolynomial l, scoped FiniteSlabPolynomial m, scoped FiniteSlabPolynomial t,
        scoped FiniteSlabPolynomial d, scoped FiniteSlabPolynomial depthN, scoped FiniteSlabPolynomial depthD,
        ref FiniteSlabRaySelection maximum)
    {
        int words = problem.FieldWords;
        Span<ulong> values = stackalloc ulong[2 * words]; Span<sbyte> signs = stackalloc sbyte[2];
        FiniteSlabPolynomial equation = new(values, signs); equation.Clear(); equation.Set(-1); equation.Set(2, 1);
        Span<ulong> cell = stackalloc ulong[1]; cell[0] = 1;
        FiniteAxisValueRoot root = new() { Coefficients = values, Signs = signs,
            LowerNumerator = cell, DenominatorShift = 1, IsRational = true, Ordinal = 0 };
        if (!Admitted(problem, ref root, l, m, t, d)) return;
        maximum.Keep(root, 1, l, m, t, d, false, depthN, depthD,
            problem.Triangle, problem.Orientation, problem.DomainEdgeMask);
        if (problem.Visitor is null || !IsClearanceExit(problem, ref root, l, m, t, d)) return;
        Span<ulong> candidateValues = stackalloc ulong[FiniteSlabRaySelection.GetStorageWords(words)];
        Span<sbyte> candidateSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection candidate = new(candidateValues, candidateSigns, words);
        candidate.Keep(root, 1, l, m, t, d, false, depthN, depthD,
            problem.Triangle, problem.Orientation, problem.DomainEdgeMask);
        problem.Visitor(candidate);
    }

    private static bool Admitted(scoped Problem problem, scoped ref FiniteAxisValueRoot root,
        scoped FiniteSlabPolynomial l, scoped FiniteSlabPolynomial m,
        scoped FiniteSlabPolynomial t, scoped FiniteSlabPolynomial d)
    {
        int words = problem.FieldWords;
        int denominatorSign = FiniteSlabPolynomial.Sign(ref root, d);
        System.Diagnostics.Debug.Assert(denominatorSign != 0);
        Span<ulong> values = stackalloc ulong[10 * words]; Span<sbyte> signs = stackalloc sbyte[10];
        FiniteSlabPolynomial q = P(values, signs, 0, 5), term = P(values, signs, 5, 5);
        if (problem.ExitRegion != 0)
        {
            // End disks are admitted only on their genuine hemispheres.
            // Their cut boundaries z=+/-core already belong to the central
            // rectangle's linear strata, so no extra chart is required.
            // This exact partition removes covered disk families and their
            // duplicate root/depth comparisons without changing the union.
            Coordinate(problem.Shape, 2, l, m, t, d, q, term);
            term.Scale(d, Signed(problem.CoreCap, problem.ExitRegion)); q.Add(term, -1);
            if (FiniteSlabPolynomial.Sign(ref root, q) * denominatorSign * problem.ExitRegion < 0) return false;
        }
        for (int index = 0; index < problem.Lines.Length; index++)
        {
            // Defining planes are exact algebraic identities of this chart.
            // Rechecking them would invoke the full resultant equality bound.
            if (problem.IsActive(index)) continue;
            ref readonly Linear line = ref problem.Lines[index];
            q.Scale(l, line.A); term.Scale(m, line.B); q.Add(term); term.Scale(t, line.C); q.Add(term);
            term.Scale(d, line.D); q.Add(term);
            if (FiniteSlabPolynomial.Sign(ref root, q) * denominatorSign < 0) return false;
        }
        // Prisms and already separated candidates never need radial scratch.
        if (problem.Disks.IsEmpty) return true;
        Span<ulong> radialValues = stackalloc ulong[24 * words]; Span<sbyte> radialSigns = stackalloc sbyte[24];
        FiniteSlabPolynomial x = P(radialValues, radialSigns, 0, 5), z = P(radialValues, radialSigns, 5, 5),
            square = P(radialValues, radialSigns, 10, 9), second = P(radialValues, radialSigns, 19, 5);
        Span<ulong> scratchValues = stackalloc ulong[18 * words]; Span<sbyte> scratchSigns = stackalloc sbyte[18];
        FiniteSlabPolynomial other = new(scratchValues[..(9 * words)], scratchSigns[..9]);
        FiniteSlabPolynomial radiusSquare = new(scratchValues[(9 * words)..], scratchSigns[9..]);
        for (int index = 0; index < problem.Disks.Length; index++)
        {
            if ((problem.SolvedDisks & (1 << index)) != 0) continue;
            Radial disk = problem.Disks[index];
            Coordinate(disk, 0, l, m, t, d, x, term); Coordinate(disk, 2, l, m, t, d, z, term);
            square.Multiply(x, x);
            other.Multiply(z, z); square.Add(other);
            second.Scale(d, disk.Radius); radiusSquare.Multiply(second, second); square.Add(radiusSquare, -1);
            if (FiniteSlabPolynomial.Sign(ref root, square) > 0) return false;
        }
        return true;
    }

    private static void Coordinate(Radial disk, int axis, FiniteSlabPolynomial l, FiniteSlabPolynomial m,
        FiniteSlabPolynomial t, FiniteSlabPolynomial d, FiniteSlabPolynomial result, FiniteSlabPolynomial term)
    {
        result.Scale(d, Component(disk.A, axis)); term.Scale(l, Component(disk.E, axis)); result.Add(term);
        term.Scale(m, Component(disk.F, axis)); result.Add(term); term.Scale(t, Component(disk.N, axis)); result.Add(term);
    }

    private static bool IsClearanceExit(scoped Problem problem, scoped ref FiniteAxisValueRoot root,
        scoped FiniteSlabPolynomial l, scoped FiniteSlabPolynomial m,
        scoped FiniteSlabPolynomial t, scoped FiniteSlabPolynomial d)
    {
        int words = problem.FieldWords;
        for (int index = problem.ExitLinearStart; index < problem.Lines.Length; index++)
            if (problem.IsActive(index) && problem.Lines[index].C.Sign < 0
                && (problem.IsPrism || index < problem.ExitLinearStart + 2)) return true;
        Span<ulong> values = stackalloc ulong[48 * words]; Span<sbyte> signs = stackalloc sbyte[48];
        FiniteSlabPolynomial x = P(values, signs, 0, 5), y = P(values, signs, 5, 5),
            z = P(values, signs, 10, 5), offset = P(values, signs, 15, 5), term = P(values, signs, 20, 5),
            square = P(values, signs, 25, 9), other = P(values, signs, 34, 9), dot = P(values, signs, 43, 5);
        int denominatorSign = FiniteSlabPolynomial.Sign(ref root, d);
        Coordinate(problem.Shape, 0, l, m, t, d, x, term);
        Coordinate(problem.Shape, 1, l, m, t, d, y, term);
        Coordinate(problem.Shape, 2, l, m, t, d, z, term);
        for (int cap = -1; cap <= 1; cap += 2)
        {
            if (problem.Shape.N.Y.Sign * cap <= 0) continue;
            offset.Copy(y); term.Scale(d, Signed(problem.HalfThickness, cap)); offset.Add(term, -1);
            if (FiniteSlabPolynomial.Sign(ref root, offset) == 0) return true;
        }
        if (problem.IsPrism)
        {
            foreach (ref readonly Linear wall in problem.Lines[problem.ExitLinearStart..])
            {
                if (wall.C.Sign >= 0) continue;
                offset.Scale(l, wall.A); term.Scale(m, wall.B); offset.Add(term);
                term.Scale(t, wall.C); offset.Add(term); term.Scale(d, wall.D); offset.Add(term);
                if (FiniteSlabPolynomial.Sign(ref root, offset) == 0) return true;
            }
            return false;
        }
        // Axial charts always mark their outward cap active and returned above.
        System.Diagnostics.Debug.Assert(!problem.Shape.N.X.IsZero || !problem.Shape.N.Z.IsZero);
        if (problem.HasCore)
        {
            // A straight side only blocks a strictly outward X ray. Sliding
            // along it reaches a farther rim and is not an outward-exit sample.
            if (problem.Shape.Radius.IsZero && !problem.Shape.N.X.IsZero) return true;
            square.Multiply(x, x); term.Scale(d, problem.Shape.Radius); other.Multiply(term, term); square.Add(other, -1);
            dot.Scale(x, problem.Shape.N.X);
            if (FiniteSlabPolynomial.Sign(ref root, square) == 0 && FiniteSlabPolynomial.Sign(ref root, dot) * denominatorSign > 0)
            {
                offset.Copy(z); term.Scale(d, problem.CoreCap); offset.Add(term, -1);
                int upper = FiniteSlabPolynomial.Sign(ref root, offset) * denominatorSign;
                offset.Copy(z); offset.Add(term);
                if (upper <= 0 && FiniteSlabPolynomial.Sign(ref root, offset) * denominatorSign >= 0) return true;
            }
        }
        for (int side = -1; side <= 1; side += 2)
        {
            offset.Copy(z);
            if (problem.HasCore) { term.Scale(d, Signed(problem.CoreCap, side)); offset.Add(term, -1); }
            int hemisphere = FiniteSlabPolynomial.Sign(ref root, offset) * denominatorSign * side;
            if (problem.HasCore && hemisphere < 0) continue;
            square.Multiply(x, x); other.Multiply(offset, offset); square.Add(other);
            term.Scale(d, problem.Shape.Radius); other.Multiply(term, term); square.Add(other, -1);
            if (FiniteSlabPolynomial.Sign(ref root, square) != 0) continue;
            dot.Scale(x, problem.Shape.N.X); term.Scale(offset, problem.Shape.N.Z); dot.Add(term);
            int derivative = FiniteSlabPolynomial.Sign(ref root, dot) * denominatorSign;
            if (derivative > 0 || derivative == 0 && (!problem.HasCore || hemisphere > 0 || problem.Shape.N.Z.Sign * side > 0))
                return true;
        }
        return false;
    }

    private static void TwoPlanesDisk(scoped Problem problem, Linear a, in Linear b, Radial disk,
        ref FiniteSlabRaySelection maximum)
    {
        int words = problem.FieldWords;
        Span<ulong> values = stackalloc ulong[22 * words]; Span<sbyte> signs = stackalloc sbyte[22];
        FiniteSlabPolynomial l = P(values, signs, 0, 2), m = P(values, signs, 2, 2),
            t = P(values, signs, 4, 2), d = P(values, signs, 6), n = P(values, signs, 7),
            equation = P(values, signs, 8, 3), x = P(values, signs, 11, 2), z = P(values, signs, 13, 2),
            term = P(values, signs, 15, 2), square = P(values, signs, 17, 3), depthD = P(values, signs, 20), depthN = P(values, signs, 21);
        Cross(d, a.A, b.B, a.B, b.A);
        bool depthParameter = !d.IsZero;
        l.Clear(); m.Clear(); t.Clear();
        if (depthParameter)
        {
            Cross(l.Slice(0, 1), a.B, b.D, a.D, b.B); Cross(l.Slice(1, 1), a.B, b.C, a.C, b.B);
            Cross(m.Slice(0, 1), a.D, b.A, a.A, b.D); Cross(m.Slice(1, 1), a.C, b.A, a.A, b.C);
            t.Add(d, 1, 1); depthN.Clear(); depthD.Clear();
        }
        else
        {
            // Dependent lambda/mu rows fix t, leaving a line of source points.
            // Its endpoints can be quadratic even though the depth is rational.
            if (a.A.IsZero && a.B.IsZero)
            {
                if (a.C.IsZero || b.A.IsZero && b.B.IsZero) return;
                n.Set(Negate(a.D)); d.Set(a.C); a = b;
            }
            else
            {
                bool first = !a.A.IsZero;
                Cross(d, first ? a.A : a.B, b.C, first ? b.A : b.B, a.C);
                if (d.IsZero) return;
                Cross(n, a.D, first ? b.A : b.B, b.D, first ? a.A : a.B);
            }
            // These active planes already fix t=n/d. A negative value is
            // infeasible before any radial polynomial or root is constructed.
            if (n.Signs[0] * d.Signs[0] < 0) return;
            PlaneChart(a, out Signed832 l0, out Signed832 ls, out Signed832 lt,
                out Signed832 m0, out Signed832 ms, out Signed832 mt, out Signed832 denominator);
            l.Scale(d, l0); term.Scale(n, lt); l.Add(term); term.Scale(d, ls); l.Add(term, 1, 1);
            m.Scale(d, m0); term.Scale(n, mt); m.Add(term); term.Scale(d, ms); m.Add(term, 1, 1);
            t.Scale(n, denominator); term.Scale(d, denominator); d.Copy(term.Slice(0, 1));
            depthN.Copy(t.Slice(0, 1)); depthN.Add(d); depthD.Copy(t.Slice(0, 1)); depthD.Add(d, 2);
            if (depthD.Signs[0] < 0) { depthN.Signs[0] *= -1; depthD.Signs[0] *= -1; }
        }
        Coordinate(disk, 0, l, m, t, d, x, term); Coordinate(disk, 2, l, m, t, d, z, term);
        equation.Multiply(x, x); square.Multiply(z, z); equation.Add(square);
        term.Scale(d, disk.Radius); square.Multiply(term, term); equation.Add(square, -1);
        ProcessChart(problem, equation, l, m, t, d, depthParameter, depthN, depthD, ref maximum);
    }

    private static void PlaneDisk(scoped Problem problem, in Linear plane, Radial disk,
        ref FiniteSlabRaySelection maximum)
    {
        int words = problem.FieldWords;
        if (plane.A.IsZero && plane.B.IsZero)
        {
            if (plane.C.IsZero) return;
            if (-plane.D.Sign * plane.C.Sign < 0) return;
            // A constant-depth face is a clipped disk/polygon region. Include
            // its intrinsic center/cardinal witnesses, even when the complete
            // circle lies inside the triangle and no authored edge meets it.
            Linear x = new(S(disk.E.X), S(disk.F.X), S(disk.N.X), S(disk.A.X));
            Linear z = new(S(disk.E.Z), S(disk.F.Z), S(disk.N.Z), S(disk.A.Z));
            // The center is not on the disk that selected this family.
            Problem centerProblem = problem.WithoutSolvedDisks();
            ThreePlanes(centerProblem, plane, x, z, ref maximum);
            for (int index = 0; index < problem.Disks.Length; index++)
            {
                TwoPlanesDisk(centerProblem.WithDisk(index), plane, x, problem.Disks[index], ref maximum);
                TwoPlanesDisk(centerProblem.WithDisk(index), plane, z, problem.Disks[index], ref maximum);
            }
            return;
        }
        Span<ulong> values = stackalloc ulong[25 * words]; Span<sbyte> signs = stackalloc sbyte[25];
        FiniteSlabPolynomial conic = P(values, signs, 0, 6), equation = P(values, signs, 6, 3),
            sn = P(values, signs, 9, 2), sd = P(values, signs, 11),
            l = P(values, signs, 12, 2), m = P(values, signs, 14, 2),
            t = P(values, signs, 16, 2), d = P(values, signs, 18),
            first = P(values, signs, 19), second = P(values, signs, 20), term = P(values, signs, 21, 2),
            depthN = P(values, signs, 23), depthD = P(values, signs, 24);
        PlaneChart(plane, out Signed832 l0, out Signed832 ls, out Signed832 lt,
            out Signed832 m0, out Signed832 ms, out Signed832 mt, out Signed832 denominator);
        BuildConic(disk, l0, ls, lt, m0, ms, mt, denominator, conic);
        FiniteSlabPolynomial a = conic.Slice(0, 1), b = conic.Slice(1, 1), c = conic.Slice(2, 1),
            e = conic.Slice(4, 1), f = conic.Slice(5, 1), h = conic.Slice(3, 1);
        if (a.IsZero) return; // PSD then has no s dependence; a real wall owns the free family.
        sd.Clear(); sd.Add(a, 2); sn.Clear(); sn.Add(h, -1); sn.Add(b, -1, 1);
        l.Scale(sd, l0); term.Scale(sn, ls); l.Add(term); term.Scale(sd, lt); l.Add(term, 1, 1);
        m.Scale(sd, m0); term.Scale(sn, ms); m.Add(term); term.Scale(sd, mt); m.Add(term, 1, 1);
        d.Scale(sd, denominator); t.Clear(); t.Add(d, 1, 1);
        equation.Clear();
        first.Multiply(a, f); second.Multiply(h, h); equation.Add(first, 4); equation.Add(second, -1);
        first.Multiply(a, e); second.Multiply(b, h); equation.Add(first, 4, 1); equation.Add(second, -2, 1);
        first.Multiply(a, c); second.Multiply(b, b); equation.Add(first, 4, 2); equation.Add(second, -1, 2);
        depthN.Clear(); depthD.Clear();
        ProcessChart(problem, equation, l, m, t, d, true, depthN, depthD, ref maximum);
    }

    private static void PlaneChart(in Linear plane, out Signed832 l0, out Signed832 ls, out Signed832 lt,
        out Signed832 m0, out Signed832 ms, out Signed832 mt, out Signed832 denominator)
    {
        if (!plane.B.IsZero)
        {
            denominator = plane.B; l0 = lt = default; ls = denominator;
            m0 = Negate(plane.D); ms = Negate(plane.A); mt = Negate(plane.C);
        }
        else
        {
            denominator = plane.A; m0 = mt = default; ms = denominator;
            l0 = Negate(plane.D); ls = Negate(plane.B); lt = Negate(plane.C);
        }
        if (denominator.Sign < 0)
        {
            denominator = Negate(denominator); l0 = Negate(l0); ls = Negate(ls); lt = Negate(lt);
            m0 = Negate(m0); ms = Negate(ms); mt = Negate(mt);
        }
    }

    private static void BuildConic(Radial disk, Signed832 l0, Signed832 ls, Signed832 lt,
        Signed832 m0, Signed832 ms, Signed832 mt, Signed832 denominator, FiniteSlabPolynomial conic)
    {
        int words = conic.FieldWords;
        Span<ulong> values = stackalloc ulong[10 * words]; Span<sbyte> signs = stackalloc sbyte[10];
        FiniteSlabPolynomial x = P(values, signs, 0, 3), z = P(values, signs, 3, 3),
            first = P(values, signs, 6), second = P(values, signs, 7), factor = P(values, signs, 8), term = P(values, signs, 9);
        for (int axis = 0; axis < 2; axis++)
        {
            FiniteSlabPolynomial coordinate = axis == 0 ? x : z; int component = axis == 0 ? 0 : 2;
            coordinate.Clear();
            for (int index = 0; index < 3; index++)
            {
                first.Set(index == 0 ? l0 : index == 1 ? ls : lt); factor.Set(Component(disk.E, component));
                term.Multiply(first, factor); coordinate.Slice(index, 1).Copy(term);
                first.Set(index == 0 ? m0 : index == 1 ? ms : mt); factor.Set(Component(disk.F, component));
                term.Multiply(first, factor); coordinate.Slice(index, 1).Add(term);
                if (index != 1)
                {
                    first.Set(denominator); factor.Set(index == 0 ? Signed576.ExtendValue(Component(disk.A, component)) : Component(disk.N, component));
                    term.Multiply(first, factor); coordinate.Slice(index, 1).Add(term);
                }
            }
        }
        conic.Clear();
        for (int index = 0; index < 6; index++)
        {
            int i = index == 0 || index == 1 || index == 3 ? 1 : index == 2 || index == 4 ? 2 : 0;
            int j = index == 0 ? 1 : index == 1 || index == 2 ? 2 : 0;
            first.Multiply(x.Slice(i, 1), x.Slice(j, 1)); second.Multiply(z.Slice(i, 1), z.Slice(j, 1));
            conic.Slice(index, 1).Add(first, i == j ? 1 : 2); conic.Slice(index, 1).Add(second, i == j ? 1 : 2);
        }
        first.Set(denominator); factor.Set(disk.Radius); term.Multiply(first, factor); first.Multiply(term, term);
        conic.Slice(5, 1).Add(first, -1);
    }

    private static void Cross(FiniteSlabPolynomial result, Signed832 a, Signed832 b, Signed832 c, Signed832 d)
    {
        int words = result.FieldWords;
        Span<ulong> values = stackalloc ulong[5 * words]; Span<sbyte> signs = stackalloc sbyte[5];
        FiniteSlabPolynomial first = P(values, signs, 0), second = P(values, signs, 1),
            third = P(values, signs, 2), fourth = P(values, signs, 3), term = P(values, signs, 4);
        first.Set(a); second.Set(b); third.Set(c); fourth.Set(d);
        result.Multiply(first, second); term.Multiply(third, fourth); result.Add(term, -1);
    }

    private static void ProcessChart(scoped Problem problem, scoped FiniteSlabPolynomial equation,
        scoped FiniteSlabPolynomial l, scoped FiniteSlabPolynomial m, scoped FiniteSlabPolynomial t,
        scoped FiniteSlabPolynomial d, bool depthParameter,
        scoped FiniteSlabPolynomial depthN, scoped FiniteSlabPolynomial depthD,
        ref FiniteSlabRaySelection maximum)
    {
        int words = problem.FieldWords;
        Span<ulong> values = stackalloc ulong[11 * words]; Span<sbyte> signs = stackalloc sbyte[11];
        FiniteSlabPolynomial transformedEquation = P(values, signs, 0, 3),
            tl = P(values, signs, 3, 2), tm = P(values, signs, 5, 2),
            tt = P(values, signs, 7, 2), td = P(values, signs, 9, 2);
        int degree = Math.Max(Math.Max(Degree(l), Degree(m)), Math.Max(Degree(t), Degree(d)));
        for (int chart = 0; chart < (depthParameter ? 1 : 2); chart++)
        {
            int sign = chart == 0 ? 1 : -1;
            Transform(equation, transformedEquation, Degree(equation), depthParameter, sign);
            Transform(l, tl, degree, depthParameter, sign); Transform(m, tm, degree, depthParameter, sign);
            Transform(t, tt, degree, depthParameter, sign); Transform(d, td, degree, depthParameter, sign);
            // These four homogeneous numerators share one denominator. Strip
            // only their common scale, preserving every exact ratio while
            // avoiding inherited transform powers in all admission products.
            WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values[(3 * words)..], signs[3..]);
            Process(problem, transformedEquation, tl, tm, tt, td, depthParameter, depthN, depthD, ref maximum);
        }
        if (!depthParameter && equation.Signs[0] == 0)
        {
            transformedEquation.Clear(); transformedEquation.Set(-1); transformedEquation.Set(2, 1);
            Process(problem, transformedEquation, l.Slice(0, 1), m.Slice(0, 1), t.Slice(0, 1), d.Slice(0, 1),
                false, depthN, depthD, ref maximum);
        }
    }

    private static void Transform(FiniteSlabPolynomial source, FiniteSlabPolynomial result,
        int degree, bool depthParameter, int parameterSign)
    {
        // t=(2u-1)/(1-u) keeps nonnegative depth in [1/2,1).
        // A free line uses s=+/-u/(1-u); its zero endpoint is separate.
        result.Clear();
        for (int index = 0; index <= Degree(source); index++)
            for (int first = 0; first <= (depthParameter ? index : 0); first++)
                for (int second = 0; second <= degree - index; second++)
                {
                    int exponent = depthParameter ? first + second : index + second;
                    int factor = Choose(degree - index, second) * (second % 2 == 0 ? 1 : -1);
                    if (depthParameter) factor *= Choose(index, first) * (1 << first) * ((index - first) % 2 == 0 ? 1 : -1);
                    else if (parameterSign < 0 && index % 2 != 0) factor = -factor;
                    result.Add(source.Slice(index, 1), factor, exponent);
                }
    }
    private static int Choose(int n, int k)
    { int value = 1; for (int i = 1; i <= k; i++) value = value * (n - i + 1) / i; return value; }
    private static int Degree(FiniteSlabPolynomial polynomial)
    { int degree = polynomial.Count - 1; while (degree > 0 && polynomial.Signs[degree] == 0) degree--; return degree; }

    private static FiniteSlabPolynomial P(Span<ulong> values, Span<sbyte> signs, int index, int count = 1) =>
        FiniteSlabPolynomial.At(values, signs, index, count);
    private static Signed576 S(int value) => S(Signed320.ExtendValue(Signed192.Signed(value)));
    private static Signed576 S(Signed320 value) => Signed576.ExtendValue(value);
    private static Signed576 S(Signed576 value) => value;
    private static Signed320 ScaledOffset(Fixed64 value, Signed320 scale) =>
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(scale, Signed192.Raw(value)));
    private static Signed320 Negate(Signed320 value) => WideArithmetic.Negate(value);
    private static Signed320 Signed(Signed320 value, int sign) => sign < 0 ? Negate(value) : value;
    private static Signed832 Negate(Signed832 value) => WideArithmetic.SubtractSigned832(default, value);
    private static Signed576 Signed(Signed576 value, int sign) => sign < 0 ? WideArithmetic.SubtractSigned576(default, value) : value;
    private static WideDirection3d Signed(WideDirection3d p, int sign) => new(Signed(p.X, sign), Signed(p.Y, sign), Signed(p.Z, sign));
    private static Signed576 Component(WideDirection3d p, int axis) => axis == 0 ? p.X : axis == 1 ? p.Y : p.Z;
    private static Signed320 Component(WideAxis3 p, int axis) => axis == 0 ? p.X : axis == 1 ? p.Y : p.Z;
    private static WideAxis3 Difference(WideAxis3 p, WideAxis3 q) => new(
        WideArithmetic.SubtractSigned320(p.X, q.X), WideArithmetic.SubtractSigned320(p.Y, q.Y),
        WideArithmetic.SubtractSigned320(p.Z, q.Z));
    private static Signed576 Wall(WideAxis3 p, Signed320 x, Signed320 z) => WideArithmetic.SubtractSigned576(
        WideArithmetic.MultiplySigned320(x, p.Z), WideArithmetic.MultiplySigned320(z, p.X));
    private static Signed576 Wall(WideDirection3d p, Signed320 x, Signed320 z) =>
        Signed576.NarrowValue(WideArithmetic.SubtractSigned704(
            WideArithmetic.MultiplySigned576ToSigned704(p.Z, x),
            WideArithmetic.MultiplySigned576ToSigned704(p.X, z)));
    private static int DomainEdge(int line, int count) => count == 0 ? 0 : line == 0 ? 4 : line == 1 ? 1 : line == 2 ? 2 : 0;
}
