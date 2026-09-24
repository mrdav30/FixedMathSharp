//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact centered-capsule overlap with finite local-+Y solids. The core chord
/// remains rational in both authoritative rigid frames; rim decisions retain
/// complete polynomial signs instead of sampling separating directions.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private readonly struct StrictFiniteChord
    {
        internal readonly Signed320 X, Y, Z, Dx, Dy, Dz;
        internal readonly Signed192 Scale;
        internal readonly Signed320 HalfHeight;

        internal StrictFiniteChord(Signed320 x, Signed320 y, Signed320 z,
            Signed320 dx, Signed320 dy, Signed320 dz, Signed192 scale, Signed320 halfHeight)
        {
            X = x; Y = y; Z = z; Dx = dx; Dy = dy; Dz = dz;
            Scale = scale; HalfHeight = halfHeight;
        }
    }

    /// <summary>
    /// Tests positive separating translation for a capsule and a finite
    /// cylinder. The solid uses local +Y; rotations and the capsule local axis
    /// are admitted unit inputs, height is positive, and radii/length are nonnegative.
    /// </summary>
    internal static bool DoesCenteredFiniteCylinderPenetrateCapsule(
        Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius,
        Vector3d capsuleCenter, FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxis, Fixed64 capsuleLength, Fixed64 capsuleRadius)
    {
        bool centerPenetrates = DoesCenteredFiniteCylinderPenetrateSphere(
            center, rotation, height, radius, capsuleCenter, capsuleRadius);
        if (centerPenetrates || capsuleLength == Fixed64.Zero)
            return centerPenetrates;
        if (radius == Fixed64.Zero)
            return WideOrientedBox.DoCenteredRigidCapsulesOverlap(center, rotation, Vector3d.Up,
                height, Fixed64.Zero, capsuleCenter, capsuleRotation, capsuleLocalAxis,
                capsuleLength, capsuleRadius, strict: true);
        StrictFiniteChord chord = CreateStrictFiniteChord(center, rotation, height,
            capsuleCenter, capsuleRotation, capsuleLocalAxis, capsuleLength);
        Signed320 r = WideArithmetic.MultiplySigned192(Signed192.Raw(radius), chord.Scale);
        Signed320 s = WideArithmetic.MultiplySigned192(Signed192.Raw(capsuleRadius), chord.Scale);
        Span<Signed576> radial = stackalloc Signed576[3];
        GetStrictRadialPolynomial(chord, radial);
        if (DoesStrictChordEnterCylinder(chord, radial,
                WideArithmetic.SubtractSigned320(default, chord.HalfHeight), chord.HalfHeight,
                WideArithmetic.AddSigned320(r, s)))
            return true;
        if (capsuleRadius == Fixed64.Zero)
            return false;
        return DoesStrictChordEnterExpandedDisk(chord, radial, r, s, chord.HalfHeight)
            || DoesStrictChordEnterExpandedDisk(chord, radial, r, s,
                WideArithmetic.SubtractSigned320(default, chord.HalfHeight));
    }

    /// <summary>
    /// Tests positive separating translation against a cone with base at
    /// local -height/2 and apex at +height/2, under the cylinder method's
    /// admitted rigid-frame and nonnegative-size preconditions.
    /// </summary>
    internal static bool DoesCenteredFiniteConePenetrateCapsule(
        Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius,
        Vector3d capsuleCenter, FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxis, Fixed64 capsuleLength, Fixed64 capsuleRadius)
    {
        bool centerPenetrates = DoesCenteredFiniteConePenetrateSphere(
            center, rotation, height, radius, capsuleCenter, capsuleRadius);
        if (centerPenetrates || capsuleLength == Fixed64.Zero)
            return centerPenetrates;
        if (radius == Fixed64.Zero)
            return WideOrientedBox.DoCenteredRigidCapsulesOverlap(center, rotation, Vector3d.Up,
                height, Fixed64.Zero, capsuleCenter, capsuleRotation, capsuleLocalAxis,
                capsuleLength, capsuleRadius, strict: true);
        StrictFiniteChord chord = CreateStrictFiniteChord(center, rotation, height,
            capsuleCenter, capsuleRotation, capsuleLocalAxis, capsuleLength);
        Signed320 r = WideArithmetic.MultiplySigned192(Signed192.Raw(radius), chord.Scale);
        Signed320 s = WideArithmetic.MultiplySigned192(Signed192.Raw(capsuleRadius), chord.Scale);
        Span<Signed576> radial = stackalloc Signed576[3];
        GetStrictRadialPolynomial(chord, radial);
        if (capsuleRadius > Fixed64.Zero)
        {
            if (DoesStrictChordEnterExpandedDisk(chord, radial, r, s,
                    WideArithmetic.SubtractSigned320(default, chord.HalfHeight)))
                return true;
            Span<Signed576> apex = stackalloc Signed576[3];
            GetStrictPointDistancePolynomial(chord, radial, chord.HalfHeight, apex);
            apex[0] = WideArithmetic.SubtractSigned576(apex[0], WideArithmetic.MultiplySigned320(s, s));
            if (HasNegativeStrictQuadratic(apex))
                return true;
        }
        return DoesStrictChordEnterOffsetConeSide(chord, radial, height, radius, s);
    }

    private static StrictFiniteChord CreateStrictFiniteChord(Vector3d center, FixedQuaternion rotation,
        Fixed64 height, Vector3d capsuleCenter, FixedQuaternion capsuleRotation,
        Vector3d localAxis, Fixed64 length)
    {
        WideRationalBasis3d basis = new(rotation);
        WideRationalBasis3d capsuleBasis = new(capsuleRotation);
        WideRationalBasis3d relative = WideRationalBasis3d.CreateRelative(basis, capsuleBasis);
        WideAxis3 axis = WideRigidProjection.TransformLocalAxis(relative,
            Signed192.Raw(localAxis.X), Signed192.Raw(localAxis.Y), Signed192.Raw(localAxis.Z));
        WideOrientedBox.GetRelativeLocalPointNumerators(capsuleCenter, center, rotation,
            out Signed192 x, out Signed192 y, out Signed192 z, out _);
        Signed320 centerScale = WideArithmetic.MultiplySigned192(capsuleBasis.Denominator, Signed192.Raw(Fixed64.Two));
        Signed192 halfScale = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(
            relative.Denominator, Signed192.Raw(Fixed64.One)));
        Signed192 scale = WideArithmetic.AddSigned192(halfScale, halfScale);
        GetStrictChordCoordinate(x, axis.X, centerScale, length, out Signed320 px, out Signed320 dx);
        GetStrictChordCoordinate(y, axis.Y, centerScale, length, out Signed320 py, out Signed320 dy);
        GetStrictChordCoordinate(z, axis.Z, centerScale, length, out Signed320 pz, out Signed320 dz);
        // D_b,D_c<2^66 and admitted |u|<2^33. Chord coordinates are below
        // 2^232, scale below2^165, and scaled dimensions below2^230. These
        // bounds retain 65-bit origin differences and half-raw endpoints.
        return new StrictFiniteChord(px, py, pz, dx, dy, dz, scale,
            WideArithmetic.MultiplySigned192(Signed192.Raw(height), halfScale));
    }

    private static void GetStrictChordCoordinate(Signed192 translation, Signed320 axis,
        Signed320 centerScale, Fixed64 length, out Signed320 start, out Signed320 direction)
    {
        Signed576 center = WideArithmetic.MultiplySigned320(Signed320.ExtendValue(translation), centerScale);
        Signed576 half = WideArithmetic.MultiplySigned320(axis, Signed192.Raw(length));
        start = Signed320.NarrowValue(WideArithmetic.SubtractSigned576(center, half));
        direction = Signed320.NarrowValue(WideArithmetic.AddSigned576(half, half));
    }

    private static void GetStrictRadialPolynomial(StrictFiniteChord chord, Span<Signed576> result)
    {
        result[0] = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(chord.X, chord.X),
            WideArithmetic.MultiplySigned320(chord.Z, chord.Z));
        result[1] = WideArithmetic.MultiplySigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(chord.X, chord.Dx),
            WideArithmetic.MultiplySigned320(chord.Z, chord.Dz)), 2);
        result[2] = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(chord.Dx, chord.Dx),
            WideArithmetic.MultiplySigned320(chord.Dz, chord.Dz));
    }

    private static bool DoesStrictChordEnterCylinder(StrictFiniteChord chord, ReadOnlySpan<Signed576> radial,
        Signed320 lowerY, Signed320 upperY, Signed320 radius)
    {
        Signed192 one = Signed192.Signed(1);
        if (!GetStrictChordBounds(chord.Y, chord.Dy, lowerY, upperY, default, one,
                out StrictFieldBound lower, out StrictFieldBound upper))
            return false;
        return HasNegativeStrictFieldQuadratic(Signed832.ExtendValue(WideArithmetic.SubtractSigned576(
                radial[0], WideArithmetic.MultiplySigned320(radius, radius))),
            Signed832.ExtendValue(radial[1]), Signed832.ExtendValue(radial[2]), default, default, one, lower, upper);
    }

    private static void GetStrictPointDistancePolynomial(StrictFiniteChord chord, ReadOnlySpan<Signed576> radial,
        Signed320 axialPoint, Span<Signed576> result)
    {
        Signed320 y = WideArithmetic.SubtractSigned320(chord.Y, axialPoint);
        result[0] = WideArithmetic.AddSigned576(radial[0], WideArithmetic.MultiplySigned320(y, y));
        result[1] = WideArithmetic.AddSigned576(radial[1], WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(y, chord.Dy), 2));
        result[2] = WideArithmetic.AddSigned576(radial[2], WideArithmetic.MultiplySigned320(chord.Dy, chord.Dy));
    }

    private static bool DoesStrictChordEnterExpandedDisk(StrictFiniteChord chord, ReadOnlySpan<Signed576> radial,
        Signed320 radius, Signed320 expansion, Signed320 cap)
    {
        if (DoesStrictChordEnterCylinder(chord, radial,
                WideArithmetic.SubtractSigned320(cap, expansion), WideArithmetic.AddSigned320(cap, expansion), radius))
            return true;
        Span<Signed576> n = stackalloc Signed576[3];
        GetStrictPointDistancePolynomial(chord, radial, cap, n);
        Signed576 radiusSquared = WideArithmetic.MultiplySigned320(radius, radius);
        n[0] = WideArithmetic.AddSigned576(n[0], WideArithmetic.SubtractSigned576(radiusSquared,
            WideArithmetic.MultiplySigned320(expansion, expansion)));
        // Circle-tube membership is N-2R√Q<0, equivalently N<0 or
        // H=N²-4R²Q<0. Full tubes and disk cores form the expanded disk;
        // no squared-root branch or rim separating direction is discarded.
        if (HasNegativeStrictQuadratic(n))
            return true;
        Span<ulong> coefficients = stackalloc ulong[5 * StrictPolynomialInputWords];
        Span<sbyte> signs = stackalloc sbyte[5];
        coefficients.Clear();
        signs.Clear();
        for (int i = 0; i < 5; i++)
        {
            Span<ulong> destination = coefficients.Slice(i * StrictPolynomialInputWords, StrictPolynomialInputWords);
            for (int j = 0; j < 3; j++)
            {
                if (i - j >= 0 && i - j < 3)
                    AccumulateStrictRimProduct(destination, ref signs[i], n[j], n[i - j], 1);
            }
            if (i < 3)
                AccumulateStrictRimProduct(destination, ref signs[i], radiusSquared, radial[i], -4);
        }
        // N coefficients use fewer than468bits; convolution and the radial
        // subtraction leave every H coefficient below2^960, inside16words.
        return HasNegativeFiniteAxisPolynomial(coefficients, signs);
    }

    private static bool HasNegativeStrictQuadratic(ReadOnlySpan<Signed576> polynomial) =>
        HasNegativeStrictFieldQuadratic(Signed832.ExtendValue(polynomial[0]),
            Signed832.ExtendValue(polynomial[1]), Signed832.ExtendValue(polynomial[2]),
            default, default, Signed192.Signed(1),
            new StrictFieldBound(default, default, StrictOne),
            new StrictFieldBound(StrictOne, default, StrictOne));

    private static void AccumulateStrictRimProduct(Span<ulong> total, ref sbyte sign,
        Signed576 first, Signed576 second, int scale)
    {
        if (first.IsZero || second.IsZero)
            return;
        Span<ulong> a = stackalloc ulong[StrictPolynomialInputWords];
        Span<ulong> b = stackalloc ulong[StrictPolynomialInputWords];
        Span<ulong> product = stackalloc ulong[StrictPolynomialInputWords];
        Span<ulong> scaled = stackalloc ulong[StrictPolynomialInputWords];
        Span<ulong> sum = stackalloc ulong[StrictPolynomialInputWords];
        a.Clear(); b.Clear();
        WideArithmetic.GetMagnitude(first, a[..9]);
        WideArithmetic.GetMagnitude(second, b[..9]);
        MultiplyRoundedCylinderWide(a, b, product);
        MultiplyRoundedCylinderWideByWord(product, (ulong)Math.Abs(scale), scaled);
        sbyte productSign = (sbyte)(first.Sign * second.Sign * Math.Sign(scale));
        AddRoundedCylinderSigned(total, sign, scaled, productSign, sum, out sign);
        sum.CopyTo(total);
    }

    private static bool DoesStrictChordEnterOffsetConeSide(StrictFiniteChord chord,
        ReadOnlySpan<Signed576> radial, Fixed64 height, Fixed64 radius, Signed320 expansion)
    {
        Signed192 h = Signed192.Raw(height), r = Signed192.Raw(radius);
        Signed192 hh = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(h, h));
        Signed192 rr = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(r, r));
        Signed192 k = WideArithmetic.AddSigned192(hh, rr);
        Signed832 shift = Signed832.ExtendValue(WideArithmetic.MultiplySigned320(expansion, r));
        if (!GetStrictChordBounds(chord.Y, chord.Dy,
                WideArithmetic.SubtractSigned320(default, chord.HalfHeight), chord.HalfHeight,
                shift, k, out StrictFieldBound lower, out StrictFieldBound upper))
            return false;
        // The expanded cone is the union of its expanded base disk, apex
        // sphere, and this offset-side frustum. Its y shift is s*r/√k and
        // side inequality is h*ρ+r*(y-h/2)<s√k. Within these bounds the
        // radial right side is positive, so squaring preserves the sign.
        Signed320 y = WideArithmetic.SubtractSigned320(chord.Y, chord.HalfHeight);
        Span<Signed576> axial = stackalloc Signed576[3]
        {
            WideArithmetic.MultiplySigned320(y, y),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(y, chord.Dy), 2),
            WideArithmetic.MultiplySigned320(chord.Dy, chord.Dy)
        };
        Span<Signed832> a = stackalloc Signed832[3];
        for (int i = 0; i < 3; i++)
            a[i] = WideArithmetic.SubtractSigned832(
                WideArithmetic.MultiplySigned832(Signed832.ExtendValue(radial[i]), hh),
                WideArithmetic.MultiplySigned832(Signed832.ExtendValue(axial[i]), rr));
        a[0] = WideArithmetic.SubtractSigned832(a[0], WideArithmetic.MultiplySigned832(
            Signed832.ExtendValue(WideArithmetic.MultiplySigned320(expansion, expansion)), k));
        Signed832 b0 = Signed832.ExtendValue(WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(expansion, y), r), 2));
        Signed832 b1 = Signed832.ExtendValue(WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(expansion, chord.Dy), r), 2));
        return HasNegativeStrictFieldQuadratic(a[0], a[1], a[2], b0, b1, k, lower, upper);
    }

}
