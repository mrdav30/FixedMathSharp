//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>Complete signed support minima for a capsule and a flat-capped stadium slab.</content>
internal static partial class WideConvexPrismRelations
{
    private readonly struct CapsuleSlabGeometry
    {
        internal readonly WideRationalBasis3d WorldBasis;
        internal readonly WideAxis3 Center, CapsuleHalf, CapsuleAxis;
        internal readonly Signed320 HalfHeight, HalfCore, Radius;
        internal readonly Signed192 RawScale, AxisDenominator;
        internal readonly int ValueShift;

        internal CapsuleSlabGeometry(Vector3d center, Fixed64 rotation, Fixed64 core, Fixed64 radius,
            Fixed64 halfHeight, Vector3d capsuleCenter, FixedQuaternion capsuleRotation, Fixed64 capsuleCore)
        {
            FixedQuaternion slabRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -rotation);
            WorldBasis = new WideRationalBasis3d(slabRotation);
            var frame = new CylinderPolytopeFrame(center, slabRotation,
                WideArithmetic.AddSigned192(Signed192.Raw(halfHeight), Signed192.Raw(halfHeight)),
                radius, capsuleCenter, capsuleRotation);
            AxisDenominator = frame.Denominator;
            CapsuleAxis = new WideAxis3(Signed320.ExtendValue(frame.Basis.Yx),
                Signed320.ExtendValue(frame.Basis.Yy), Signed320.ExtendValue(frame.Basis.Yz));
            WideAxis3 offset = frame.Transform(Vector3d.Zero);
            Span<Signed320> coordinates = stackalloc Signed320[10]
            {
                Signed320.ExtendValue(WideArithmetic.AddSigned192(frame.Denominator, frame.Denominator)),
                offset.X, offset.Y, offset.Z,
                WideArithmetic.MultiplySigned192(frame.Basis.Yx, Signed192.Raw(capsuleCore)),
                WideArithmetic.MultiplySigned192(frame.Basis.Yy, Signed192.Raw(capsuleCore)),
                WideArithmetic.MultiplySigned192(frame.Basis.Yz, Signed192.Raw(capsuleCore)),
                frame.Cap, WideArithmetic.MultiplySigned192(frame.Denominator, Signed192.Raw(core)),
                WideArithmetic.MultiplySigned192(frame.Denominator, frame.Radius)
            };
            WideArithmetic.ReduceCommonScale(coordinates);
            RawScale = Signed192.NarrowProven(coordinates[0]);
            Center = new WideAxis3(coordinates[1], coordinates[2], coordinates[3]);
            CapsuleHalf = new WideAxis3(coordinates[4], coordinates[5], coordinates[6]);
            HalfHeight = coordinates[7]; HalfCore = coordinates[8]; Radius = coordinates[9];
            int bits = 0;
            Span<ulong> magnitude = stackalloc ulong[5];
            for (int index = 1; index < coordinates.Length; index++)
            {
                WideArithmetic.GetMagnitude(coordinates[index], out magnitude[4], out magnitude[3],
                    out magnitude[2], out magnitude[1], out magnitude[0]);
                bits = Math.Max(bits, WideArithmetic.GetMagnitudeBitLength(magnitude));
            }
            // Extents/offsets <2^200; the margin covers both endpoint additions
            // and the complete support value. All end regions use this scale.
            ValueShift = 2 * (bits + 5);
        }

        internal WideAxis3 CapOffset(int cap, int end) => new(WideArithmetic.Negate(Center.X),
            WideArithmetic.SubtractSigned320(cap > 0 ? HalfHeight : WideArithmetic.Negate(HalfHeight), Center.Y),
            WideArithmetic.SubtractSigned320(end > 0 ? HalfCore : WideArithmetic.Negate(HalfCore), Center.Z));

        internal WideAxis3 EndpointOffset(int cap, int end, int capsule)
        {
            WideAxis3 c = CapOffset(cap, end);
            return new WideAxis3(
                GetCylinderCapsuleEndpointCoordinate(WideArithmetic.Negate(c.X), default, 0, CapsuleHalf.X, capsule),
                GetCylinderCapsuleEndpointCoordinate(WideArithmetic.Negate(c.Y), default, 0, CapsuleHalf.Y, capsule),
                GetCylinderCapsuleEndpointCoordinate(WideArithmetic.Negate(c.Z), default, 0, CapsuleHalf.Z, capsule));
        }
    }

    // Borrowed buffers keep enumeration in one place without allocating or
    // retaining a candidate per feature. Exact ties keep the earlier feature.
    private ref struct CapsuleSlabCandidates
    {
        internal CapsuleSlabGeometry Geometry;
        internal Span<ulong> Values, BestValues, Direction;
        internal Span<int> Signs, BestSigns, DirectionSigns;
        internal int GapSign;
        internal bool HasBest;

        internal void Keep(WideAxis3 direction)
        {
            WriteDirection(direction, Direction, DirectionSigns);
            KeepDirection();
        }

        internal void KeepDirection()
        {
            if (IsZero(Direction)) return;
            int gapSign = BuildCapsuleSlabDirection(Geometry, Direction, DirectionSigns, Values, Signs);
            KeepCylinderCapsuleCandidate(Values, Signs, gapSign,
                BestValues, BestSigns, ref GapSign, ref HasBest);
        }

        internal ConvexContactCandidate Best => new(BestValues, BestSigns, GapSign);
    }

    /// <summary>Returns complete slab-to-capsule penetration from authoritative authored frames.</summary>
    internal static bool TryGetCenteredCapsuleSlabCapsulePenetration(
        Vector3d slabCenter, Fixed64 slabRotation, Fixed64 slabCoreLength, Fixed64 slabRadius, Fixed64 slabHalfThickness,
        Vector3d capsuleCenter, FixedQuaternion capsuleRotation, Fixed64 capsuleCoreLength, Fixed64 capsuleRadius,
        out Vector3d normal, out Fixed64 depth, out bool depthIsClamped)
    {
        if (slabCoreLength == Fixed64.Zero)
            return TryGetCenteredFiniteCylinderCapsulePenetration(slabCenter, FixedQuaternion.Identity, Vector3d.Up,
                WideArithmetic.AddSigned192(Signed192.Raw(slabHalfThickness), Signed192.Raw(slabHalfThickness)), slabRadius,
                capsuleCenter, capsuleRotation, Vector3d.Up, capsuleCoreLength, capsuleRadius,
                out normal, out depth, out depthIsClamped);

        var geometry = new CapsuleSlabGeometry(slabCenter, slabRotation, slabCoreLength, slabRadius,
            slabHalfThickness, capsuleCenter, capsuleRotation, capsuleCoreLength);
        Span<ulong> storage = stackalloc ulong[(2 * ConvexContactCandidate.Slots + 3) * Words];
        Span<int> signs = stackalloc int[2 * ConvexContactCandidate.Slots + 3];
        var candidates = new CapsuleSlabCandidates
        {
            Geometry = geometry, Values = storage[..(11 * Words)], BestValues = storage.Slice(11 * Words, 11 * Words),
            Direction = storage[(22 * Words)..], Signs = signs[..11], BestSigns = signs.Slice(11, 11),
            DirectionSigns = signs[22..]
        };
        WideAxis3 x = new(Signed320.ExtendValue(Signed192.Signed(1)), default, default);
        WideAxis3 y = new(default, x.X, default), z = new(default, default, x.X);
        // Poles and plane intersections own ties before interior stationary roots.
        candidates.Keep(y);
        if (geometry.CapsuleHalf.X.IsZero && geometry.CapsuleHalf.Z.IsZero)
        {
            // Parallel cores form a Cartesian product: a planar stadium and
            // one vertical interval. If either factor contains the center,
            // min support is the smaller of the planar and vertical gaps.
            // Both-outside corners still require the complete rim enumeration.
            int capGapSign = candidates.GapSign;
            Signed320 radialZ = GetCylinderCapsuleEndpointCoordinate(geometry.Center.Z,
                geometry.HalfCore, geometry.Center.Z.Sign, default, 0);
            if (radialZ.Sign != geometry.Center.Z.Sign) radialZ = default;
            WideAxis3 radial = new(geometry.Center.X, default, radialZ);
            candidates.Keep(radial.IsZero ? x : radial);
            if (capGapSign >= 0 || WideArithmetic.SubtractSigned576(
                    WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius), radial.SquaredLength).Sign >= 0)
                return MaterializeCylinderCapsulePenetration(candidates.Best, capsuleRadius,
                    out normal, out depth, out depthIsClamped);
        }
        candidates.Keep(x); candidates.Keep(z);
        WideAxis3 e = geometry.CapsuleAxis;
        candidates.Keep(WideAxis3.Cross(y, e)); candidates.Keep(WideAxis3.Cross(z, e));
        // The straight-side normal plane has n.Z=0, so the slab endpoint
        // does not affect its fan. Enumerate it once, not once per end.
        for (int cap = -1; cap <= 1; cap += 2)
            for (int capsule = -1; capsule <= 1; capsule += 2)
            {
                WideAxis3 q = geometry.EndpointOffset(cap, 1, capsule);
                for (int radial = -1; radial <= 1; radial += 2)
                {
                    WideAxis3 direction = new(WideArithmetic.SubtractSigned320(q.X,
                        radial > 0 ? geometry.Radius : WideArithmetic.Negate(geometry.Radius)), q.Y, default);
                    candidates.Keep(direction);
                    if (HasCapsuleSlabStraightRimCertificate(geometry, direction,
                            WideArithmetic.AddSigned320(q.Z, geometry.HalfCore), cap, radial, capsule))
                        return MaterializeCylinderCapsulePenetration(candidates.Best, capsuleRadius,
                            out normal, out depth, out depthIsClamped);
                }
            }
        for (int end = -1; end <= 1; end += 2)
            for (int capsule = -1; capsule <= 1; capsule += 2)
            {
                WideAxis3 q = geometry.EndpointOffset(1, end, capsule);
                candidates.Keep(new WideAxis3(q.X, default, q.Z));
                for (int cap = -1; cap <= 1; cap += 2)
                {
                    q = geometry.EndpointOffset(cap, end, capsule);
                    if (BuildCapsuleSlabEndpointRim(geometry, q, cap, end, capsule, candidates.Values, candidates.Signs))
                        return MaterializeCylinderCapsulePenetration(new ConvexContactCandidate(candidates.Values, candidates.Signs, -1),
                            capsuleRadius, out normal, out depth, out depthIsClamped);
                }
            }

        if (!geometry.CapsuleHalf.IsZero && !(e.X.IsZero && e.Z.IsZero))
        {
            WideAxis3 principal = WideAxis3.Cross(e, y);
            candidates.Keep(principal); candidates.Keep(WideAxis3.Cross(e, principal));
            CircularRimContactAlgebra.GetBasis(e, out WideAxis3 first, out WideAxis3 second);
            candidates.Keep(first); candidates.Keep(second);
            candidates.Keep(new WideAxis3(WideArithmetic.AddSigned320(first.X, second.X),
                WideArithmetic.AddSigned320(first.Y, second.Y), WideArithmetic.AddSigned320(first.Z, second.Z)));
            candidates.Keep(new WideAxis3(WideArithmetic.SubtractSigned320(first.X, second.X),
                WideArithmetic.SubtractSigned320(first.Y, second.Y), WideArithmetic.SubtractSigned320(first.Z, second.Z)));
            for (int cap = -1; cap <= 1; cap += 2)
                for (int end = -1; end <= 1; end += 2)
                {
                    WideAxis3 c = geometry.CapOffset(cap, end);
                    ProjectPerpendicular(e, c, candidates.Direction, candidates.DirectionSigns);
                    candidates.KeepDirection();
                    if (e.Y.IsZero && !geometry.Radius.IsZero)
                        KeepCapsuleSlabHorizontalCorners(ref candidates, c, principal);
                }
            if (!e.Y.IsZero && !geometry.Radius.IsZero
                && TryImproveCapsuleSlabRim(geometry, candidates.Best, capsuleRadius,
                    out bool intersects, out normal, out depth, out depthIsClamped))
                return intersects;
        }
        return MaterializeCylinderCapsulePenetration(candidates.Best, capsuleRadius, out normal, out depth, out depthIsClamped);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int BuildCapsuleSlabDirection(in CapsuleSlabGeometry geometry,
        ReadOnlySpan<ulong> direction, ReadOnlySpan<int> directionSigns, Span<ulong> values, Span<int> signs)
    {
        values.Clear(); signs.Clear();
        Span<ulong> work = stackalloc ulong[7 * Words];
        Span<ulong> p = Slot(work, 0), q = Slot(work, 1), metric = Slot(work, 2), scale = Slot(work, 3),
            factor = Slot(work, 4), product = Slot(work, 5), denominator = Slot(work, 6);
        CylinderContactAlgebra.Dot(geometry.Center, direction, directionSigns, p, out int centerSign);
        int pSign = IsZero(p) ? 0 : -1;
        Import(geometry.HalfHeight, factor);
        WideArithmetic.MultiplyMagnitudes(factor, Slot(direction, 1), product); Add(product, 1, p, ref pSign);
        Import(geometry.HalfCore, factor);
        WideArithmetic.MultiplyMagnitudes(factor, Slot(direction, 2), product); Add(product, 1, p, ref pSign);
        CylinderContactAlgebra.Dot(geometry.CapsuleHalf, direction, directionSigns, product, out _); Add(product, 1, p, ref pSign);
        WideArithmetic.MultiplyMagnitudes(Slot(direction, 0), Slot(direction, 0), metric);
        WideArithmetic.MultiplyMagnitudes(Slot(direction, 2), Slot(direction, 2), product);
        WideArithmetic.AddMagnitudeInto(product, metric);
        Import(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius), factor);
        WideArithmetic.MultiplyMagnitudes(factor, metric, q);
        SumSquares(direction, metric);
        Import(WideArithmetic.MultiplySigned192(geometry.RawScale, geometry.RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(scale, metric, denominator);
        // The core Minkowski sum is centrally symmetric. Of ±n, the one
        // facing the offset always has the smaller whole-shape support gap.
        WriteWorldDirection(geometry.WorldBasis, direction, directionSigns, values, signs, centerSign < 0 ? -1 : 1);
        return BuildRadialCandidate(p, pSign, q, denominator, values, signs);
    }

    private static bool HasCapsuleSlabStraightRimCertificate(in CapsuleSlabGeometry geometry,
        WideAxis3 residual, Signed320 planarZ, int cap, int radial, int capsule)
    {
        // w=(±R,±H,planarZ)+±B is feasible and supports the entire core
        // along residual=D-w. These exact support signs make w its closest
        // point, including the end seam and a tied capsule endpoint.
        return residual.X.Sign * radial > 0 && residual.Y.Sign * cap > 0
            && WideArithmetic.SubtractSigned320(planarZ, geometry.HalfCore).Sign <= 0
            && WideArithmetic.AddSigned320(planarZ, geometry.HalfCore).Sign >= 0
            && (geometry.CapsuleHalf.IsZero || WideAxis3.Dot(residual, geometry.CapsuleAxis).Sign * capsule >= 0);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void KeepCapsuleSlabHorizontalCorners(ref CapsuleSlabCandidates candidates, WideAxis3 offset, WideAxis3 radial)
    {
        Span<ulong> work = stackalloc ulong[3 * Words];
        Span<ulong> factor = Slot(work, 0), component = Slot(work, 1), product = Slot(work, 2);
        Import(WideArithmetic.MultiplySigned320(candidates.Geometry.Radius, candidates.Geometry.AxisDenominator), factor);
        for (int side = -1; side <= 1; side += 2)
        {
            ProjectPerpendicular(candidates.Geometry.CapsuleAxis, offset, candidates.Direction, candidates.DirectionSigns);
            for (int axis = 0; axis < 3; axis++)
            {
                Signed320 value = Component(radial, axis);
                Import(value, component); WideArithmetic.MultiplyMagnitudes(factor, component, product);
                Add(product, side * value.Sign, Slot(candidates.Direction, axis), ref candidates.DirectionSigns[axis]);
            }
            candidates.KeepDirection();
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool BuildCapsuleSlabEndpointRim(in CapsuleSlabGeometry geometry, WideAxis3 q,
        int cap, int end, int capsule, Span<ulong> values, Span<int> signs)
    {
        if (q.Y.Sign * cap <= 0 || q.Z.Sign * end <= 0) return false;
        Signed576 radialSquared = CircularRimContactAlgebra.RadialDot(q, q);
        Signed576 radiusSquared = WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius);
        if (WideArithmetic.SubtractSigned576(radialSquared, radiusSquared).Sign <= 0) return false;
        Span<ulong> work = stackalloc ulong[6 * Words];
        Span<int> directionSigns = stackalloc int[3];
        Span<ulong> direction = work[..(3 * Words)], root = Slot(work, 3), a = Slot(work, 4), b = Slot(work, 5);
        Import(radialSquared, root);
        if (!geometry.CapsuleHalf.IsZero)
        {
            Signed576 projection = WideAxis3.Dot(q, geometry.CapsuleAxis);
            Signed576 radialProjection = CircularRimContactAlgebra.RadialDot(q, geometry.CapsuleAxis);
            ImportCylinderCapsuleFeature(WideArithmetic.MultiplySigned576ToSigned832(radialProjection, geometry.Radius), a);
            Import(projection, b);
            if (GetConvexContactCandidateQuadraticSign(a, -radialProjection.Sign, b, projection.Sign, root) * capsule <= 0)
                return false;
        }
        values.Clear(); signs.Clear();
        WriteDirection(new WideAxis3(q.X, default, q.Z), direction, directionSigns);
        Import(geometry.Radius, a);
        for (int axis = 0; axis < 3; axis++)
        {
            WideArithmetic.MultiplyMagnitudes(Slot(direction, axis), a, b);
            b.CopyTo(Slot(direction, axis)); directionSigns[axis] = IsZero(b) ? 0 : -directionSigns[axis];
        }
        WriteWorldDirection(geometry.WorldBasis, direction, directionSigns, values, signs);
        WriteDirection(q, direction, directionSigns);
        WriteWorldDirection(geometry.WorldBasis, direction, directionSigns, values[(3 * Words)..], signs[3..]);
        root.CopyTo(Slot(values, 6)); root.CopyTo(Slot(values, 9));
        Import(WideArithmetic.AddSigned576(q.SquaredLength, radiusSquared), Slot(values, 7));
        Import(WideArithmetic.AddSigned320(geometry.Radius, geometry.Radius), Slot(values, 8));
        Import(WideArithmetic.MultiplySigned192(geometry.RawScale, geometry.RawScale), Slot(values, 10));
        signs[7] = 1; signs[8] = geometry.Radius.IsZero ? 0 : -1;
        return true;
    }
}
