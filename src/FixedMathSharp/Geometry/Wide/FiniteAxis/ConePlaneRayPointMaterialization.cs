//=======================================================================
// ConePlaneRayPointMaterialization.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Final nearest-even world and local anchors from admitted exact plane-section points.</summary>
internal static class ConePlaneRayPointMaterialization
{
    /// <summary>Conservatively certifies the world coordinate range of every point in a validated finite cone.</summary>
    /// <remarks>False is inconclusive: individual points may still be representable.</remarks>
    internal static bool IsWorldRangeRepresentable(in ConePlaneRayFrame frame)
        => IsRangeRepresentable(frame, Vector3d.Zero);

    /// <summary>Conservatively certifies rounded world coordinates relative to a common sampling origin.</summary>
    internal static bool IsRangeRepresentable(in ConePlaneRayFrame frame, Vector3d samplingOrigin)
    {
        // The frame's exact rational rotation is orthogonal. Each centered
        // world component is bounded by Radius+Height/2, so doubled raw bounds
        // avoid rounding odd heights. All sums are <68 bits in Signed192.
        // Closed [MinRaw,MaxRaw] bounds also avoid nearest-even half-tie edges.
        Signed192 radius = Signed192.Raw(frame.Radius);
        Signed192 bound = WideArithmetic.AddSigned192(Signed192.Raw(frame.Height), WideArithmetic.AddSigned192(radius, radius));
        Signed192 minimum = WideArithmetic.AddSigned192(Signed192.Signed(long.MinValue), Signed192.Signed(long.MinValue));
        Signed192 maximum = WideArithmetic.AddSigned192(Signed192.Signed(long.MaxValue), Signed192.Signed(long.MaxValue));
        for (int axis = 0; axis < 3; axis++)
        {
            Signed192 center = WideArithmetic.SubtractSigned192(Signed192.Raw(frame.ConeCenter[axis]), Signed192.Raw(samplingOrigin[axis]));
            center = WideArithmetic.AddSigned192(center, center);
            if (WideArithmetic.SubtractSigned192(WideArithmetic.SubtractSigned192(center, bound), minimum).Sign < 0
                || WideArithmetic.SubtractSigned192(WideArithmetic.AddSigned192(center, bound), maximum).Sign > 0)
                return false;
        }
        return true;
    }

    internal static bool TryGetWorldPoint(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, out Vector3d worldPoint)
        => TryGetPointInFrame(frame, point, root, Vector3d.Zero, out worldPoint);

    /// <summary>Rounds once on the world lattice, then expresses that point relative to the sampling origin.</summary>
    internal static bool TryGetPointInFrame(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, Vector3d samplingOrigin, out Vector3d pointInFrame)
        => TryGetPoint(frame, point.X, point.Y, point.Z, point.Denominator, root, false, true, samplingOrigin, out pointInFrame);

    internal static bool TryGetExitWorldPoint(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation, out Vector3d worldPoint)
        => TryGetExitPointInFrame(frame, point, root, numerator, denominator, orientation, Vector3d.Zero, out worldPoint);

    /// <summary>Rounds an exit once on the world lattice, retaining a representable common-frame coordinate.</summary>
    internal static bool TryGetExitPointInFrame(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation, Vector3d samplingOrigin, out Vector3d pointInFrame)
        => TryGetExitPoint(frame, point, root, numerator, denominator, orientation, false, true, samplingOrigin, out pointInFrame);

    /// <summary>Independently rounds a common-frame exit and its cone-local anchor from one exact construction.</summary>
    internal static bool TryGetExitPointsInFrame(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation, Vector3d samplingOrigin, out Vector3d pointInFrame, out Vector3d coneLocalPoint)
        => TryGetExitPoint(frame, point, root, numerator, denominator, orientation,
            coneLocal: false, materialize: true, samplingOrigin, out pointInFrame,
            includeConeLocal: true, out coneLocalPoint);

    /// <summary>Rounds a selected exit directly in the cone's centered local frame, independently of world rounding.</summary>
    internal static bool TryGetExitConeLocalPoint(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation, out Vector3d localPoint)
        => TryGetExitPoint(frame, point, root, numerator, denominator, orientation, true, true, Vector3d.Zero, out localPoint);

    /// <summary>Rounds an admitted point directly in its authored local frame, independently of world rounding.</summary>
    internal static bool TryGetAuthoredPoint(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, out Vector3d localPoint)
    {
        // The retained centered transform is P=2*(B*a+T), with B/D exactly
        // orthogonal. Its inverse is a=B^T*(P-2*T)/(2*D^2). Relative basis
        // entries <130 bits and translation <198 add <333 bits to any retained
        // field; six extra limbs cover every complete sum without narrowing.
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(point.X), ContactQuadratic.ActiveWords(point.Y)),
            Math.Max(ContactQuadratic.ActiveWords(point.Z), ContactQuadratic.ActiveWords(point.Denominator))) + 6;
        Span<ulong> work = stackalloc ulong[16 * words]; Span<int> signs = stackalloc int[16];
        ContactQuadratic x = ContactQuadratic.At(work, signs, 0, words);
        ContactQuadratic y = ContactQuadratic.At(work, signs, 1, words);
        ContactQuadratic z = ContactQuadratic.At(work, signs, 2, words);
        ContactQuadratic d = ContactQuadratic.At(work, signs, 3, words);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 4, words);
        Signed576 one = Extend(Signed192.Signed(1));
        ContactQuadratic.Scale(point.X, one, x); ContactQuadratic.Scale(point.Z, one, z);
        ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), y); y.Add(point.Y, -1);
        for (int axis = 0; axis < 3; axis++)
        {
            Signed320 offset = axis == 0 ? frame.Finite.ShapeFrame.Translation.X
                : axis == 1 ? frame.Finite.ShapeFrame.Translation.Y : frame.Finite.ShapeFrame.Translation.Z;
            ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(offset), term); term.Add(term);
            (axis == 0 ? x : axis == 1 ? y : z).Add(term, -1);
        }
        WideRationalBasis3d basis = frame.Finite.ShapeFrame.Basis;
        Signed192 tripleDenominator = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(basis.Denominator, basis.Denominator), basis.Denominator);
        if (WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(basis.Xx, basis.Yy), basis.Zz).Equals(tripleDenominator))
        {
            // An exact proper orthogonal rotation has trace 3 only at identity.
            // Then B=D*I and B^T*(P-2T)/(2D²) cancels to (P-2T)/(2D),
            // including equal noncardinal authored/cone rotations. Keep this
            // cancellation local; other geometry owners retain their basis ABI.
            ContactQuadratic.Scale(point.Denominator,
                Extend(WideArithmetic.AddSigned192(basis.Denominator, basis.Denominator)), d);
            return TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity, x, y, z, d, root, out localPoint);
        }
        Signed320 square = WideArithmetic.MultiplySigned192(basis.Denominator, basis.Denominator);
        ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(WideArithmetic.AddSigned320(square, square)), d);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic target = ContactQuadratic.At(work, signs, 5 + axis, words);
            ContactQuadratic.Scale(x, Extend(axis == 0 ? basis.Xx : axis == 1 ? basis.Yx : basis.Zx), target);
            ContactQuadratic.Scale(y, Extend(axis == 0 ? basis.Xy : axis == 1 ? basis.Yy : basis.Zy), term); target.Add(term);
            ContactQuadratic.Scale(z, Extend(axis == 0 ? basis.Xz : axis == 1 ? basis.Yz : basis.Zz), term); target.Add(term);
        }
        return TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity,
            ContactQuadratic.At(work, signs, 5, words), ContactQuadratic.At(work, signs, 6, words),
            ContactQuadratic.At(work, signs, 7, words), d, root, out localPoint);
    }

    /// <summary>Tests the final nearest-even exit coordinate range without materializing a world point.</summary>
    internal static bool IsExitWorldPointRepresentable(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation)
        => IsExitPointRepresentable(frame, point, root, numerator, denominator, orientation, Vector3d.Zero);

    /// <summary>Tests final common-frame coordinate range using the world's nearest-even parity.</summary>
    internal static bool IsExitPointRepresentable(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation, Vector3d samplingOrigin)
        => TryGetExitPoint(frame, point, root, numerator, denominator, orientation, false, false, samplingOrigin, out _);

    private static bool TryGetExitPoint(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation, bool coneLocal, bool materialize, Vector3d samplingOrigin, out Vector3d worldPoint)
        => TryGetExitPoint(frame, point, root, numerator, denominator, orientation, coneLocal, materialize,
            samplingOrigin, out worldPoint, includeConeLocal: false, out _);

    private static bool TryGetExitPoint(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation, bool coneLocal, bool materialize, Vector3d samplingOrigin, out Vector3d worldPoint,
        bool includeConeLocal, out Vector3d coneLocalPoint)
    {
        worldPoint = coneLocalPoint = default;
        int numeratorSign = numerator.Sign(root);
        if (numeratorSign < 0 || denominator.Sign(root) <= 0 || point.Denominator.Sign(root) <= 0)
            return false;
        if (numeratorSign == 0)
            return TryGetPoint(frame, point.X, point.Y, point.Z, point.Denominator, root, coneLocal, materialize, samplingOrigin, out worldPoint)
                && (!includeConeLocal || TryGetPoint(frame, point.X, point.Y, point.Z, point.Denominator,
                    root, true, true, Vector3d.Zero, out coneLocalPoint));
        // q=(P*d+orientation*N*n*D)/(D*d). Complete quadratic products
        // need p+e+r limbs and one addition carry. The Signed320 normal
        // adds at most five limbs, and the final sum adds another carry.
        // Count actual coefficients, including the root product; retained
        // padded tails never justify narrowing a complete active product.
        int pointWords = Math.Max(Math.Max(ContactQuadratic.ActiveWords(point.X), ContactQuadratic.ActiveWords(point.Y)),
            Math.Max(ContactQuadratic.ActiveWords(point.Z), ContactQuadratic.ActiveWords(point.Denominator)));
        int exitWords = pointWords + Math.Max(ContactQuadratic.ActiveWords(numerator), ContactQuadratic.ActiveWords(denominator))
            + WideArithmetic.GetActiveMagnitudeLength(root) + 7;
        Span<ulong> work = stackalloc ulong[10 * exitWords]; Span<int> signs = stackalloc int[10];
        ContactQuadratic x = ContactQuadratic.At(work, signs, 0, exitWords);
        ContactQuadratic y = ContactQuadratic.At(work, signs, 1, exitWords);
        ContactQuadratic z = ContactQuadratic.At(work, signs, 2, exitWords);
        ContactQuadratic d = ContactQuadratic.At(work, signs, 3, exitWords);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 4, exitWords);
        ContactQuadratic.Multiply(point.Denominator, numerator, root, d);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic coordinate = axis == 0 ? point.X : axis == 1 ? point.Y : point.Z;
            ContactQuadratic target = axis == 0 ? x : axis == 1 ? y : z;
            Signed320 normal = axis == 0 ? frame.Normal.X : axis == 1 ? frame.Normal.Y : frame.Normal.Z;
            ContactQuadratic.Multiply(coordinate, denominator, root, target);
            ContactQuadratic.Scale(d, Signed576.ExtendValue(normal), term); target.Add(term, orientation);
        }
        ContactQuadratic.Multiply(point.Denominator, denominator, root, d);
        // Reuse the exact exit, then round each output on its own lattice.
        // Inverse-transforming the rounded world output would change local ties.
        return TryGetPoint(frame, x, y, z, d, root, coneLocal, materialize, samplingOrigin, out worldPoint)
            && (!includeConeLocal || TryGetPoint(frame, x, y, z, d, root, true, true, Vector3d.Zero, out coneLocalPoint));
    }

    private static bool TryGetPoint(in ConePlaneRayFrame frame, scoped ContactQuadratic px,
        scoped ContactQuadratic py, scoped ContactQuadratic pz, scoped ContactQuadratic pd,
        scoped ReadOnlySpan<ulong> root, bool coneLocal, bool materialize, Vector3d samplingOrigin, out Vector3d worldPoint)
    {
        // Convert apex coordinates to centered raw cone coordinates before
        // the shared exact rigid transform. This changes no retained field.
        int words = Math.Max(ContactQuadratic.ActiveWords(py), ContactQuadratic.ActiveWords(pd)) + 4;
        Span<ulong> work = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic centeredY = ContactQuadratic.At(work, signs, 0, words);
        ContactQuadratic denominator = ContactQuadratic.At(work, signs, 1, words);
        ContactQuadratic.Scale(pd, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), centeredY);
        centeredY.Add(py, -1);
        Signed192 scale = WideArithmetic.AddSigned192(frame.Finite.ShapeFrame.Denominator, frame.Finite.ShapeFrame.Denominator);
        ContactQuadratic.Scale(pd, Extend(scale), denominator);
        return TryGetWorldPoint(coneLocal ? Vector3d.Zero : frame.ConeCenter,
            coneLocal ? FixedQuaternion.Identity : frame.ConeRotation,
            px, centeredY, pz, denominator, root, materialize, samplingOrigin, out worldPoint);
    }

    /// <summary>Rounds exact raw local coordinates after one rational rigid transform.</summary>
    internal static bool TryGetWorldPoint(Vector3d origin, FixedQuaternion rotation,
        scoped ContactQuadratic px, scoped ContactQuadratic py, scoped ContactQuadratic pz,
        scoped ContactQuadratic pd, scoped ReadOnlySpan<ulong> root, out Vector3d worldPoint)
        => TryGetPointInFrame(origin, rotation, px, py, pz, pd, root, Vector3d.Zero, out worldPoint);

    internal static bool TryGetPointInFrame(Vector3d origin, FixedQuaternion rotation,
        scoped ContactQuadratic px, scoped ContactQuadratic py, scoped ContactQuadratic pz,
        scoped ContactQuadratic pd, scoped ReadOnlySpan<ulong> root, Vector3d samplingOrigin, out Vector3d pointInFrame)
        => TryGetWorldPoint(origin, rotation, px, py, pz, pd, root, true, samplingOrigin, out pointInFrame);

    private static bool TryGetWorldPoint(Vector3d origin, FixedQuaternion rotation,
        scoped ContactQuadratic px, scoped ContactQuadratic py, scoped ContactQuadratic pz,
        scoped ContactQuadratic pd, scoped ReadOnlySpan<ulong> root, bool materialize, Vector3d samplingOrigin, out Vector3d worldPoint)
    {
        worldPoint = default;
        if (pd.Sign(root) <= 0)
            return false;
        // Representable quaternion products add <130 bits, translation <64,
        // and the final doubled representability threshold <65. Four extra
        // limbs cover all complete products and sums for any supplied fields.
        bool identity = rotation == FixedQuaternion.Identity;
        WideRationalBasis3d basis = identity ? default : new(rotation);
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(px), ContactQuadratic.ActiveWords(py)),
            Math.Max(ContactQuadratic.ActiveWords(pz), ContactQuadratic.ActiveWords(pd))) + 4;
        Span<ulong> work = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic denominator = ContactQuadratic.At(work, signs, 0, words);
        ContactQuadratic numerator = ContactQuadratic.At(work, signs, 1, words);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 2, words);
        ContactQuadratic query = ContactQuadratic.At(work, signs, 3, words);
        if (identity) ContactQuadratic.Scale(pd, Extend(Signed192.Signed(1)), denominator);
        else ContactQuadratic.Scale(pd, Extend(basis.Denominator), denominator);
        Signed192 upperHalf = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(
            Signed192.Signed(long.MaxValue), Signed192.Signed(long.MaxValue)), Signed192.Signed(1));
        Signed192 lowerHalf = WideArithmetic.SubtractSigned192(WideArithmetic.AddSigned192(
            Signed192.Signed(long.MinValue), Signed192.Signed(long.MinValue)), Signed192.Signed(1));
        Span<Fixed64> coordinates = stackalloc Fixed64[3];
        for (int axis = 0; axis < 3; axis++)
        {
            Signed192 x = axis == 0 ? basis.Xx : axis == 1 ? basis.Xy : basis.Xz;
            Signed192 y = axis == 0 ? basis.Yx : axis == 1 ? basis.Yy : basis.Yz;
            Signed192 z = axis == 0 ? basis.Zx : axis == 1 ? basis.Zy : basis.Zz;
            // Identity is also the direct local-output route. Preserve the
            // original ratio instead of multiplying it by a redundant S^2
            // quaternion denominator and performing six zero products.
            if (identity) ContactQuadratic.Scale(axis == 0 ? px : axis == 1 ? py : pz, Extend(Signed192.Signed(1)), numerator);
            else
            {
                ContactQuadratic.Scale(px, Extend(x), numerator);
                ContactQuadratic.Scale(py, Extend(y), term); numerator.Add(term);
                ContactQuadratic.Scale(pz, Extend(z), term); numerator.Add(term);
            }
            // Subtract origins before narrowing. A valid point offset can exist
            // even when its conceptual absolute world coordinate is outside Q32.32.
            Signed192 center = WideArithmetic.SubtractSigned192(Signed192.Raw(origin[axis]), Signed192.Raw(samplingOrigin[axis]));
            ContactQuadratic.Scale(denominator, Extend(center), term); numerator.Add(term);
            long parityOffset = samplingOrigin[axis].m_rawValue;
            // Rational coordinates share the raw ratio owner's exact signed
            // range and nearest-even policy, combining admission and rounding.
            if (numerator.Signs[1] == 0 && denominator.Signs[1] == 0)
            {
                if (!Fixed64.TryGetSignedRawRatio(numerator.Rational, denominator.Rational, numerator.Signs[0] < 0, out coordinates[axis], parityOffset))
                    return false;
                continue;
            }
            // Ties belong to the global lattice: an odd sampling origin flips
            // the common-frame parity. Preserve this at both range thresholds
            // and final rounding, so translating cannot change a selected row.
            numerator.CopyTo(term); term.Add(numerator);
            ContactQuadratic.Scale(denominator, Extend(upperHalf), query);
            query.Add(term, -1);
            int upperComparison = query.Sign(root);
            if (upperComparison < 0 || upperComparison == 0 && (parityOffset & 1) == 0)
                return false;
            ContactQuadratic.Scale(denominator, Extend(lowerHalf), query);
            query.Add(term, -1);
            int lowerComparison = query.Sign(root);
            if (lowerComparison > 0 || lowerComparison == 0 && (parityOffset & 1) != 0)
                return false;
            if (materialize) coordinates[axis] = ContactQuadratic.RoundRatio(numerator, denominator, root, parityOffset: parityOffset);
        }
        if (materialize) worldPoint = new Vector3d(coordinates[0], coordinates[1], coordinates[2]);
        return true;
    }

    private static Signed576 Extend(Signed192 value) => Signed576.ExtendValue(Signed320.ExtendValue(value));
}
