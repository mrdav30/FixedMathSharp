//=======================================================================
// ConePlaneRayPointMaterialization.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Final nearest-even world anchors from admitted exact plane-section points.</summary>
internal static class ConePlaneRayPointMaterialization
{
    private const int Words = ConePlaneRayPoint.FieldWords;

    internal static bool TryGetWorldPoint(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, out Vector3d worldPoint)
        => TryGetWorldPoint(frame, point.X, point.Y, point.Z, point.Denominator, root, out worldPoint);

    internal static bool TryGetExitWorldPoint(in ConePlaneRayFrame frame, scoped ConePlaneRayPoint point,
        scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        int orientation, out Vector3d worldPoint)
    {
        worldPoint = default;
        if (numerator.Sign(root) < 0 || denominator.Sign(root) <= 0 || point.Denominator.Sign(root) <= 0)
            return false;
        // q=(P*d+orientation*N*n*D)/(D*d). Retained P,D coefficients
        // <3072, n,d<4096 and root<2498 give the product <9667 bits;
        // N<260 makes q's coefficients <9928. The final world transform
        // below needs <9998 bits, within 160 words. These larger fields are
        // temporary and never alter a retained point or event descriptor.
        const int exitWords = 160;
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
        return TryGetWorldPoint(frame, x, y, z, d, root, out worldPoint);
    }

    private static bool TryGetWorldPoint(in ConePlaneRayFrame frame, scoped ContactQuadratic px,
        scoped ContactQuadratic py, scoped ContactQuadratic pz, scoped ContactQuadratic pd,
        scoped ReadOnlySpan<ulong> root, out Vector3d worldPoint)
    {
        worldPoint = default;
        if (pd.Sign(root) <= 0)
            return false;
        // Reflect the apex coordinate back to centered local Y, then apply
        // the exact cone basis. Point fields <3072, cap<193, single quaternion
        // basis<66 and relative denominator<130: world coordinate fields
        // including center translation <3336 bits; the common denominator
        // <3270 bits. No intermediate rounds. A successfully rounded anchor
        // differs by at most half a Q32 raw unit in each world coordinate.
        WideRationalBasis3d basis = new(frame.ConeRotation);
        int words = Math.Max(Words, pd.FieldWords);
        Span<ulong> work = stackalloc ulong[10 * words]; Span<int> signs = stackalloc int[10];
        ContactQuadratic centeredY = ContactQuadratic.At(work, signs, 0, words);
        ContactQuadratic denominator = ContactQuadratic.At(work, signs, 1, words);
        ContactQuadratic numerator = ContactQuadratic.At(work, signs, 2, words);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 3, words);
        ContactQuadratic query = ContactQuadratic.At(work, signs, 4, words);
        ContactQuadratic.Scale(pd, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), centeredY);
        centeredY.Add(py, -1);
        Signed320 commonScale = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Denominator, basis.Denominator);
        commonScale = WideArithmetic.AddSigned320(commonScale, commonScale);
        ContactQuadratic.Scale(pd, Signed576.ExtendValue(commonScale), denominator);
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
            ContactQuadratic.Scale(px, Extend(x), numerator);
            ContactQuadratic.Scale(centeredY, Extend(y), term); numerator.Add(term);
            ContactQuadratic.Scale(pz, Extend(z), term); numerator.Add(term);
            Fixed64 center = axis == 0 ? frame.ConeCenter.X : axis == 1 ? frame.ConeCenter.Y : frame.ConeCenter.Z;
            ContactQuadratic.Scale(denominator, Extend(Signed192.Raw(center)), term); numerator.Add(term);
            // Representability belongs to the rounded coordinate. MaxRaw is
            // odd, so its upper half tie rounds out; MinRaw is even, so its
            // lower half tie rounds in. Compare doubled exact coordinates.
            numerator.CopyTo(term); term.Add(numerator);
            ContactQuadratic.Scale(denominator, Extend(upperHalf), query);
            query.Add(term, -1);
            if (query.Sign(root) <= 0)
                return false;
            ContactQuadratic.Scale(denominator, Extend(lowerHalf), query);
            query.Add(term, -1);
            if (query.Sign(root) > 0)
                return false;
            coordinates[axis] = ContactQuadratic.RoundRatio(numerator, denominator, root);
        }
        worldPoint = new Vector3d(coordinates[0], coordinates[1], coordinates[2]);
        return true;
    }

    private static Signed576 Extend(Signed192 value) => Signed576.ExtendValue(Signed320.ExtendValue(value));
}
