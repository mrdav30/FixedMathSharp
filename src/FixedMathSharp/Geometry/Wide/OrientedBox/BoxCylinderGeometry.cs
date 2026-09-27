//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Exact box-cylinder geometry in the box's authored rotation frame. Position,
/// extent, half-axis and radius coordinates share one reduced scale per raw
/// coordinate; the cylinder direction is independently primitive.
/// </summary>
internal readonly struct BoxCylinderGeometry
{
    internal readonly WideRationalBasis3d WorldBasis;
    internal readonly WideAxis3 Axis;
    internal readonly WideAxis3 Half;
    internal readonly WideAxis3 CenterDifference;
    internal readonly WideAxis3 HalfExtents;
    internal readonly Signed320 Radius;
    internal readonly Signed192 RawScale;
    internal readonly int ValueShift;

    internal BoxCylinderGeometry(Vector3d boxCenter, FixedQuaternion boxRotation,
        Vector3d boxHalfExtents, Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxis, Fixed64 cylinderLength, Fixed64 cylinderRadius)
    {
        WorldBasis = new WideRationalBasis3d(boxRotation);
        var cylinderBasis = new WideRationalBasis3d(cylinderRotation);
        WideRationalBasis3d relative = WideRationalBasis3d.CreateRelative(WorldBasis, cylinderBasis);
        WideAxis3 axis = WideRigidProjection.TransformLocalAxis(relative,
            Signed192.Raw(localCylinderAxis.X), Signed192.Raw(localCylinderAxis.Y),
            Signed192.Raw(localCylinderAxis.Z));
        Signed192 axisX = Signed192.NarrowProven(axis.X);
        Signed192 axisY = Signed192.NarrowProven(axis.Y);
        Signed192 axisZ = Signed192.NarrowProven(axis.Z);
        Signed192 axisGcd = WideArithmetic.GetGreatestCommonDivisor(
            WideArithmetic.GetGreatestCommonDivisor(axisX, axisY), axisZ);
        Axis = new WideAxis3(
            Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(axisX, axisGcd)),
            Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(axisY, axisGcd)),
            Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(axisZ, axisGcd)));

        Signed192 twiceOne = Signed192.Raw(Fixed64.Two);
        Signed320 rawScale = WideArithmetic.MultiplySigned192(relative.Denominator, twiceOne);
        WideAxis3 half = Scale(axis, Signed320.ExtendValue(Signed192.Raw(cylinderLength)));
        WideOrientedBox.GetRelativeLocalPointNumerators(cylinderCenter, boxCenter, WorldBasis,
            out Signed192 dx, out Signed192 dy, out Signed192 dz);
        WideAxis3 difference = Scale(new WideAxis3(Signed320.ExtendValue(dx),
            Signed320.ExtendValue(dy), Signed320.ExtendValue(dz)),
            WideArithmetic.MultiplySigned192(cylinderBasis.Denominator, twiceOne));
        WideAxis3 extents = Scale(new WideAxis3(
            Signed320.ExtendValue(Signed192.Raw(boxHalfExtents.X)),
            Signed320.ExtendValue(Signed192.Raw(boxHalfExtents.Y)),
            Signed320.ExtendValue(Signed192.Raw(boxHalfExtents.Z))), rawScale);
        Signed320 radius = Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(rawScale, Signed192.Raw(cylinderRadius)));

        // Admitted quaternion denominators have at most 65 magnitude bits.
        // Relative basis entries are <2^130, the transformed axis <2^165,
        // and rawScale <2^163. Full raw-domain positions give difference
        // components <2^229; half-axis components are <2^228 and extents
        // and radius <2^226. Every product narrowed above therefore fits.
        Span<Signed320> coordinates = stackalloc Signed320[11]
        {
            rawScale, half.X, half.Y, half.Z, difference.X, difference.Y, difference.Z,
            extents.X, extents.Y, extents.Z, radius
        };
        WideArithmetic.ReduceCommonScale(coordinates);
        RawScale = Signed192.NarrowProven(coordinates[0]);
        Half = new WideAxis3(coordinates[1], coordinates[2], coordinates[3]);
        CenterDifference = new WideAxis3(coordinates[4], coordinates[5], coordinates[6]);
        HalfExtents = new WideAxis3(coordinates[7], coordinates[8], coordinates[9]);
        Radius = coordinates[10];

        int bits = 0;
        Span<ulong> magnitude = stackalloc ulong[5];
        for (int index = 1; index < coordinates.Length; index++)
        {
            WideArithmetic.GetMagnitude(coordinates[index], out magnitude[4], out magnitude[3],
                out magnitude[2], out magnitude[1], out magnitude[0]);
            bits = Math.Max(bits, WideArithmetic.GetMagnitudeBitLength(magnitude));
        }
        // Orthogonal projection cannot increase length. The residual formed
        // from vertex + cap half-axis - center and a radius contribution is
        // <2^(bits+4), and the authored bounds above also keep it <2^240.
        ValueShift = Math.Min(480, 2 * (bits + 4));
    }

    internal WideAxis3 GetCapVertexOffset(int cornerBits, int capSign) => new(
        Offset(HalfExtents.X, Half.X, CenterDifference.X, (cornerBits & 1) != 0, capSign),
        Offset(HalfExtents.Y, Half.Y, CenterDifference.Y, (cornerBits & 2) != 0, capSign),
        Offset(HalfExtents.Z, Half.Z, CenterDifference.Z, (cornerBits & 4) != 0, capSign));

    internal static Signed320 GetComponent(in WideAxis3 value, int index) =>
        index switch { 0 => value.X, 1 => value.Y, _ => value.Z };

    private static WideAxis3 Scale(WideAxis3 value, Signed320 scale) => new(
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(value.X, scale)),
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(value.Y, scale)),
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(value.Z, scale)));

    private static Signed320 Offset(Signed320 extent, Signed320 half, Signed320 center,
        bool positiveVertex, int capSign) => WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(positiveVertex ? extent : WideArithmetic.Negate(extent),
                capSign == 0 ? default : capSign > 0 ? half : WideArithmetic.Negate(half)), center);
}
