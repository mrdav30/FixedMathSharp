//=======================================================================
// QuadraticPointMaterialization.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Final nearest-even rigid-frame points from retained exact quadratic coordinates.</summary>
internal static class QuadraticPointMaterialization
{
    /// <summary>Composes an exact rigid point with an affine inverse before final source-lattice rounding.</summary>
    internal static bool TryGetPointInScaledFrame(Vector3d sourceOrigin, FixedQuaternion sourceRotation,
        in ScaledRigidFrame3d target, scoped ContactQuadratic px, scoped ContactQuadratic py,
        scoped ContactQuadratic pz, scoped ContactQuadratic pd, scoped ReadOnlySpan<ulong> root,
        out Vector3d sourcePoint)
    {
        sourcePoint = default;
        if (pd.Sign(root) <= 0) return false;
        // The rigid composition adds <132 bits, and affine inverse
        // coefficients/offsets add <453. Ten extra limbs retain every sum.
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(px), ContactQuadratic.ActiveWords(py)),
            Math.Max(ContactQuadratic.ActiveWords(pz), ContactQuadratic.ActiveWords(pd))) + 10;
        Span<ulong> work = stackalloc ulong[16 * words]; Span<int> signs = stackalloc int[16];
        ContactQuadratic x = ContactQuadratic.At(work, signs, 0, words),
            y = ContactQuadratic.At(work, signs, 1, words), z = ContactQuadratic.At(work, signs, 2, words),
            denominator = ContactQuadratic.At(work, signs, 3, words), term = ContactQuadratic.At(work, signs, 4, words),
            coordinate = ContactQuadratic.At(work, signs, 5, words), divisor = ContactQuadratic.At(work, signs, 6, words),
            zero = ContactQuadratic.At(work, signs, 7, words);
        zero.Clear();
        TransformPointInRigidFrame(sourceOrigin, sourceRotation, Vector3d.Zero, FixedQuaternion.Identity,
            px, py, pz, pd, x, y, z, denominator, term);
        Span<Fixed64> coordinates = stackalloc Fixed64[3];
        for (int axis = 0; axis < 3; axis++)
        {
            target.GetSourceCoordinateTransform(axis, out Signed576 cx, out Signed576 cy, out Signed576 cz,
                out Signed576 offset, out Signed576 scale);
            if (scale.IsZero) return false;
            ContactQuadratic.Scale(x, cx, coordinate);
            ContactQuadratic.Scale(y, cy, term); coordinate.Add(term);
            ContactQuadratic.Scale(z, cz, term); coordinate.Add(term);
            ContactQuadratic.Scale(denominator, offset, term); coordinate.Add(term);
            ContactQuadratic.Scale(denominator, scale, divisor);
            if (scale.Sign < 0) { coordinate.MultiplySign(-1); divisor.MultiplySign(-1); }
            // Reuse the authoritative scalar range/tie policy; the two zero
            // coordinates add no geometry or intermediate materialization.
            if (!TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity,
                    coordinate, zero, zero, divisor, root, out Vector3d rounded))
                return false;
            coordinates[axis] = rounded.X;
        }
        sourcePoint = new Vector3d(coordinates[0], coordinates[1], coordinates[2]);
        return true;
    }

    /// <summary>Rounds an exact offset after composing its source and target rigid bases.</summary>
    internal static bool TryGetOffsetInFrame(FixedQuaternion sourceRotation, FixedQuaternion targetRotation,
        scoped ContactQuadratic px, scoped ContactQuadratic py, scoped ContactQuadratic pz,
        scoped ContactQuadratic pd, scoped ReadOnlySpan<ulong> root, out Vector3d offset)
        => TryGetPointInRigidFrame(Vector3d.Zero, sourceRotation, Vector3d.Zero, targetRotation,
            px, py, pz, pd, root, out offset);

    /// <summary>Composes both rigid frames before final nearest-even point materialization.</summary>
    internal static bool TryGetPointInRigidFrame(Vector3d sourceOrigin, FixedQuaternion sourceRotation,
        Vector3d targetOrigin, FixedQuaternion targetRotation,
        scoped ContactQuadratic px, scoped ContactQuadratic py, scoped ContactQuadratic pz,
        scoped ContactQuadratic pd, scoped ReadOnlySpan<ulong> root, out Vector3d point)
    {
        if (sourceRotation == targetRotation && sourceOrigin == targetOrigin)
            return TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity, px, py, pz, pd, root, out point);
        if (targetRotation == FixedQuaternion.Identity)
            return TryGetPointInFrame(sourceOrigin, sourceRotation, px, py, pz, pd, root, targetOrigin, out point);

        // Each quaternion basis coefficient is <130 bits. Composing two bases
        // and summing three terms adds <262 bits. An origin difference adds
        // <65 bits, so its complete translated coefficient stays <327 bits.
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(px), ContactQuadratic.ActiveWords(py)),
            Math.Max(ContactQuadratic.ActiveWords(pz), ContactQuadratic.ActiveWords(pd))) + 6;
        Span<ulong> work = stackalloc ulong[10 * words]; Span<int> signs = stackalloc int[10];
        ContactQuadratic x = ContactQuadratic.At(work, signs, 0, words);
        ContactQuadratic y = ContactQuadratic.At(work, signs, 1, words);
        ContactQuadratic z = ContactQuadratic.At(work, signs, 2, words);
        ContactQuadratic denominator = ContactQuadratic.At(work, signs, 3, words);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 4, words);
        TransformPointInRigidFrame(sourceOrigin, sourceRotation, targetOrigin, targetRotation,
            px, py, pz, pd, x, y, z, denominator, term);
        return TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity, x, y, z, denominator, root, out point);
    }

    /// <summary>
    /// Retains exact target-frame coordinates for subsequent algebraic queries.
    /// All output banks and the disjoint scratch bank need input active words+6.
    /// </summary>
    private static void TransformPointInRigidFrame(Vector3d sourceOrigin, FixedQuaternion sourceRotation,
        Vector3d targetOrigin, FixedQuaternion targetRotation,
        scoped ContactQuadratic px, scoped ContactQuadratic py, scoped ContactQuadratic pz, scoped ContactQuadratic pd,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z,
        scoped ContactQuadratic denominator, scoped ContactQuadratic term)
    {
        WideRationalBasis3d source = new(sourceRotation), target = new(targetRotation);
        bool translated = sourceOrigin != targetOrigin;
        ContactQuadratic.Scale(pd, Signed576.ExtendValue(WideArithmetic.MultiplySigned192(
            source.Denominator, target.Denominator)), denominator);
        for (int axis = 0; axis < 3; axis++)
        {
            Signed192 tx = axis == 0 ? target.Xx : axis == 1 ? target.Yx : target.Zx;
            Signed192 ty = axis == 0 ? target.Xy : axis == 1 ? target.Yy : target.Zy;
            Signed192 tz = axis == 0 ? target.Xz : axis == 1 ? target.Yz : target.Zz;
            ContactQuadratic coordinate = axis == 0 ? x : axis == 1 ? y : z;
            ContactQuadratic.Scale(px, Signed576.ExtendValue(WideArithmetic.GetDotProduct3D(
                source.Xx, source.Xy, source.Xz, tx, ty, tz)), coordinate);
            ContactQuadratic.Scale(py, Signed576.ExtendValue(WideArithmetic.GetDotProduct3D(
                source.Yx, source.Yy, source.Yz, tx, ty, tz)), term); coordinate.Add(term);
            ContactQuadratic.Scale(pz, Signed576.ExtendValue(WideArithmetic.GetDotProduct3D(
                source.Zx, source.Zy, source.Zz, tx, ty, tz)), term); coordinate.Add(term);
            if (translated)
            {
                Signed320 translation = WideArithmetic.GetDotProduct3D(
                    WideArithmetic.SubtractSigned192(Signed192.Raw(sourceOrigin.X), Signed192.Raw(targetOrigin.X)),
                    WideArithmetic.SubtractSigned192(Signed192.Raw(sourceOrigin.Y), Signed192.Raw(targetOrigin.Y)),
                    WideArithmetic.SubtractSigned192(Signed192.Raw(sourceOrigin.Z), Signed192.Raw(targetOrigin.Z)), tx, ty, tz);
                ContactQuadratic.Scale(pd, WideArithmetic.MultiplySigned576(Signed576.ExtendValue(translation), source.Denominator), term);
                coordinate.Add(term);
            }
        }
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

    internal static bool TryGetWorldPoint(Vector3d origin, FixedQuaternion rotation,
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
