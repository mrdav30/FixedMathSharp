//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Exact cylinder-pair geometry in the first authored rotation frame. Positions
/// and radii use one reduced integer scale per raw coordinate; axis directions
/// and the perpendicular plane basis are independently primitive.
/// </summary>
internal readonly struct CylinderPairGeometry
{
    internal readonly WideRationalBasis3d WorldBasis;
    internal readonly WideAxis3 FirstAxis;
    internal readonly WideAxis3 SecondAxis;
    internal readonly WideAxis3 PlaneU;
    internal readonly WideAxis3 PlaneW;
    internal readonly WideAxis3 FirstHalf;
    internal readonly WideAxis3 SecondHalf;
    internal readonly WideAxis3 CenterDifference;
    internal readonly Signed320 FirstRadius;
    internal readonly Signed320 SecondRadius;
    internal readonly Signed192 RawScale;
    internal readonly int ValueShift;

    internal CylinderPairGeometry(Vector3d firstCenter, FixedQuaternion firstRotation,
        Vector3d firstLocalAxis, Signed192 firstLength, Fixed64 firstRadius,
        Vector3d secondCenter, FixedQuaternion secondRotation, Vector3d secondLocalAxis,
        Signed192 secondLength, Fixed64 secondRadius)
    {
        WorldBasis = new WideRationalBasis3d(firstRotation);
        var secondBasis = new WideRationalBasis3d(secondRotation);
        WideRationalBasis3d relative = WideRationalBasis3d.CreateRelative(WorldBasis, secondBasis);
        var first = new WideAxis3(Signed320.ExtendValue(Signed192.Raw(firstLocalAxis.X)),
            Signed320.ExtendValue(Signed192.Raw(firstLocalAxis.Y)),
            Signed320.ExtendValue(Signed192.Raw(firstLocalAxis.Z)));
        WideAxis3 second = WideRigidProjection.TransformLocalAxis(relative,
            Signed192.Raw(secondLocalAxis.X), Signed192.Raw(secondLocalAxis.Y), Signed192.Raw(secondLocalAxis.Z));
        FirstAxis = Primitive(first);
        SecondAxis = Primitive(second);
        PlaneU = FirstAxis.X.IsZero && FirstAxis.Y.IsZero
            ? new WideAxis3(Signed320.ExtendValue(Signed192.Signed(1)), default, default)
            : Primitive(new WideAxis3(FirstAxis.Y, WideArithmetic.Negate(FirstAxis.X), default));
        PlaneW = Primitive(WideAxis3.Cross(FirstAxis, PlaneU));

        Signed192 twiceOne = Signed192.Raw(Fixed64.Two);
        Signed320 rawScale = WideArithmetic.MultiplySigned192(relative.Denominator, twiceOne);
        WideAxis3 firstHalf = Scale(first,
            WideArithmetic.MultiplySigned192(relative.Denominator, firstLength));
        WideAxis3 secondHalf = Scale(second, Signed320.ExtendValue(secondLength));
        WideOrientedBox.GetRelativeLocalPointNumerators(secondCenter, firstCenter, WorldBasis,
            out Signed192 dx, out Signed192 dy, out Signed192 dz);
        WideAxis3 difference = Scale(new WideAxis3(Signed320.ExtendValue(dx), Signed320.ExtendValue(dy),
            Signed320.ExtendValue(dz)), WideArithmetic.MultiplySigned192(secondBasis.Denominator, twiceOne));
        Signed320 radiusA = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(rawScale, Signed192.Raw(firstRadius)));
        Signed320 radiusB = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(rawScale, Signed192.Raw(secondRadius)));
        Span<Signed320> coordinates = stackalloc Signed320[12]
        {
            rawScale, firstHalf.X, firstHalf.Y, firstHalf.Z, secondHalf.X, secondHalf.Y, secondHalf.Z,
            difference.X, difference.Y, difference.Z, radiusA, radiusB
        };
        WideArithmetic.ReduceCommonScale(coordinates);
        RawScale = Signed192.NarrowProven(coordinates[0]);
        FirstHalf = new WideAxis3(coordinates[1], coordinates[2], coordinates[3]);
        SecondHalf = new WideAxis3(coordinates[4], coordinates[5], coordinates[6]);
        CenterDifference = new WideAxis3(coordinates[7], coordinates[8], coordinates[9]);
        FirstRadius = coordinates[10];
        SecondRadius = coordinates[11];
        int bits = 0;
        Span<ulong> magnitude = stackalloc ulong[5];
        for (int index = 1; index < coordinates.Length; index++)
        {
            Import(coordinates[index], magnitude);
            bits = Math.Max(bits, WideArithmetic.GetMagnitudeBitLength(magnitude));
        }
        // Each residual is c+p+q. The component-wise bound gives magnitude
        // <2^(bits+4); the authored full-domain bound independently gives
        // magnitude <2^230. Use the tighter squared-value upper bound.
        ValueShift = Math.Min(460, 2 * (bits + 4));
    }

    internal WideAxis3 GetCapOffset(int firstSign, int secondSign) => new(
        Offset(FirstHalf.X, SecondHalf.X, CenterDifference.X, firstSign, secondSign),
        Offset(FirstHalf.Y, SecondHalf.Y, CenterDifference.Y, firstSign, secondSign),
        Offset(FirstHalf.Z, SecondHalf.Z, CenterDifference.Z, firstSign, secondSign));

    internal void WriteInvariants(int firstSign, int secondSign,
        Span<ulong> values, Span<sbyte> signs, int words)
    {
        WideAxis3 c = GetCapOffset(firstSign, secondSign);
        Write(PlaneU.SquaredLength, values, signs, 0, words);
        Write(PlaneW.SquaredLength, values, signs, 1, words);
        Write(SecondAxis.SquaredLength, values, signs, 2, words);
        Write(WideArithmetic.MultiplySigned320(FirstRadius, FirstRadius), values, signs, 3, words);
        Write(WideArithmetic.MultiplySigned320(SecondRadius, SecondRadius), values, signs, 4, words);
        Write(c.SquaredLength, values, signs, 5, words);
        Write(WideAxis3.Dot(c, PlaneU), values, signs, 6, words);
        Write(WideAxis3.Dot(c, PlaneW), values, signs, 7, words);
        Write(WideAxis3.Dot(SecondAxis, PlaneU), values, signs, 8, words);
        Write(WideAxis3.Dot(SecondAxis, PlaneW), values, signs, 9, words);
        Write(WideAxis3.Dot(SecondAxis, c), values, signs, 10, words);
    }

    private static WideAxis3 Primitive(WideAxis3 axis)
    {
        Signed192 x = Signed192.NarrowProven(axis.X);
        Signed192 y = Signed192.NarrowProven(axis.Y);
        Signed192 z = Signed192.NarrowProven(axis.Z);
        Signed192 gcd = WideArithmetic.GetGreatestCommonDivisor(
            WideArithmetic.GetGreatestCommonDivisor(x, y), z);
        return new WideAxis3(Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(x, gcd)),
            Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(y, gcd)),
            Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(z, gcd)));
    }

    private static WideAxis3 Scale(WideAxis3 axis, Signed320 scale) => new(
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(axis.X, scale)),
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(axis.Y, scale)),
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(axis.Z, scale)));

    private static Signed320 Offset(Signed320 first, Signed320 second, Signed320 center,
        int firstSign, int secondSign) => WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(firstSign > 0 ? first : WideArithmetic.Negate(first),
                secondSign > 0 ? second : WideArithmetic.Negate(second)), center);

    private static void Import(Signed320 value, Span<ulong> magnitude) =>
        WideArithmetic.GetMagnitude(value, out magnitude[4], out magnitude[3], out magnitude[2],
            out magnitude[1], out magnitude[0]);

    private static void Write(Signed576 value, Span<ulong> values, Span<sbyte> signs, int index, int words)
    {
        Span<ulong> destination = values.Slice(index * words, words);
        destination.Clear();
        WideArithmetic.GetMagnitude(value, destination[..9]);
        signs[index] = (sbyte)value.Sign;
    }
}
