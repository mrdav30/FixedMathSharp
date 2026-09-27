//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Cap-axis and common-perpendicular cylinder support candidates, represented
/// in the shared exact raw-gap and world-normal contact format.
/// </summary>
internal static class CylinderPairAnalyticFeatures
{
    private const int Words = ConvexContactCandidate.Words;

    /// <summary>
    /// Builds axis 0 (first cap), 1 (second cap), or 2 (axis cross product)
    /// for a nonparallel positive-radius pair. Storage is borrowed by
    /// ConvexContactCandidate; its shared ranking and rounding need no radius offset.
    /// </summary>
    internal static void Build(in CylinderPairGeometry geometry, int axisIndex,
        Span<ulong> values, Span<int> signs, out int gapSign)
    {
        System.Diagnostics.Debug.Assert((uint)axisIndex <= 2);
        WideAxis3 direction = axisIndex == 0 ? geometry.FirstAxis
            : axisIndex == 1 ? geometry.SecondAxis : WideAxis3.Cross(geometry.FirstAxis, geometry.SecondAxis);
        System.Diagnostics.Debug.Assert(!direction.IsZero);
        values.Clear();
        signs.Clear();
        Signed576 center = WideAxis3.Dot(geometry.CenterDifference, direction);
        Signed576 rational = WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(Abs(WideAxis3.Dot(geometry.FirstHalf, direction)),
                Abs(WideAxis3.Dot(geometry.SecondHalf, direction))), Abs(center));
        WriteNormal(geometry.WorldBasis, direction, center.Sign < 0 ? -1 : 1, values, signs);

        // |a_i|<2^33, |b_i|<2^163, |(a x b)_i|<2^197. The cap
        // rational terms are <2^264/<2^394, and the cross term <2^428.
        // Thus A<2^857. Coupled (B,C) bounds are (491,724), (621,464),
        // (656,396) bits respectively. D, including RawScale², is <2^724.
        // All construction intermediates fit the existing 2560-bit slots.
        Span<ulong> work = stackalloc ulong[9 * Words];
        Span<ulong> delta = Slot(work, 0);
        Span<ulong> squaredDirection = Slot(work, 1);
        Span<ulong> projection = Slot(work, 2);
        Span<ulong> rationalMagnitude = Slot(work, 3);
        Span<ulong> radius = Slot(work, 4);
        Span<ulong> rationalSquaredDelta = Slot(work, 5);
        Span<ulong> radiusSquaredProjection = Slot(work, 6);
        Span<ulong> temporary = Slot(work, 7);
        Span<ulong> product = Slot(work, 8);
        Import(direction.SquaredLength, squaredDirection);
        if (axisIndex == 2)
        {
            delta.Clear();
            delta[0] = 1;
            squaredDirection.CopyTo(projection);
            Import(Signed576.ExtendValue(WideArithmetic.AddSigned320(geometry.FirstRadius, geometry.SecondRadius)), radius);
        }
        else
        {
            WideAxis3 opposite = axisIndex == 0 ? geometry.SecondAxis : geometry.FirstAxis;
            Import(opposite.SquaredLength, delta);
            WideArithmetic.MultiplyMagnitudes(delta, squaredDirection, product);
            Import(WideAxis3.Dot(opposite, direction), temporary);
            WideArithmetic.MultiplyMagnitudes(temporary, temporary, projection);
            WideArithmetic.SubtractEqualMagnitudes(product, projection, projection);
            Import(Signed576.ExtendValue(axisIndex == 0 ? geometry.SecondRadius : geometry.FirstRadius), radius);
        }

        Import(rational, rationalMagnitude);
        WideArithmetic.MultiplyMagnitudes(rationalMagnitude, rationalMagnitude, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, delta, rationalSquaredDelta);
        WideArithmetic.MultiplyMagnitudes(radius, radius, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, projection, radiusSquaredProjection);
        WideArithmetic.AddEqualMagnitudes(rationalSquaredDelta, radiusSquaredProjection, Slot(values, 7));
        WideArithmetic.MultiplyMagnitudes(rationalMagnitude, radius, temporary);
        WideArithmetic.AddEqualMagnitudes(temporary, temporary, Slot(values, 8));
        WideArithmetic.MultiplyMagnitudes(projection, delta, Slot(values, 9));
        Import(Signed576.ExtendValue(Signed320.ExtendValue(geometry.RawScale)), rationalMagnitude);
        WideArithmetic.MultiplyMagnitudes(rationalMagnitude, rationalMagnitude, temporary);
        WideArithmetic.MultiplyMagnitudes(delta, squaredDirection, product);
        WideArithmetic.MultiplyMagnitudes(temporary, product, Slot(values, 10));
        signs[7] = 1;
        signs[8] = IsZero(Slot(values, 8)) ? 0 : rational.Sign;
        signs[9] = 1;
        signs[10] = 1;
        gapSign = rational.Sign < 0
            ? WideArithmetic.CompareMagnitudeEqualLength(radiusSquaredProjection, rationalSquaredDelta)
            : 1; // Every family has strictly positive radial support.
    }

    private static void WriteNormal(in WideRationalBasis3d basis, WideAxis3 direction,
        int orientation, Span<ulong> values, Span<int> signs)
    {
        for (int component = 0; component < 3; component++)
        {
            WideAxis3 row = component == 0 ? Axis(basis.Xx, basis.Yx, basis.Zx)
                : component == 1 ? Axis(basis.Xy, basis.Yy, basis.Zy) : Axis(basis.Xz, basis.Yz, basis.Zz);
            Signed576 value = WideAxis3.Dot(direction, row);
            Import(value, Slot(values, component));
            signs[component] = orientation * value.Sign;
        }
    }

    private static WideAxis3 Axis(Signed192 x, Signed192 y, Signed192 z) => new(
        Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z));
    private static Signed576 Abs(Signed576 value) => value.Sign < 0 ? WideArithmetic.SubtractSigned576(default, value) : value;
    private static bool IsZero(ReadOnlySpan<ulong> value) => WideArithmetic.GetActiveMagnitudeLength(value) == 0;
    private static Span<ulong> Slot(Span<ulong> values, int slot) => values.Slice(slot * Words, Words);
    private static void Import(Signed576 value, Span<ulong> magnitude)
    {
        magnitude.Clear();
        WideArithmetic.GetMagnitude(value, magnitude[..9]);
    }
}
