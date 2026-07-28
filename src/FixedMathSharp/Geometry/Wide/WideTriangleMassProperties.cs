//=======================================================================
// WideTriangleMassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns wide uniform thin-shell integration for indexed triangle surfaces.
/// </summary>
internal static class WideTriangleMassProperties
{
    private static readonly Signed192 Three = Signed192.Signed(3);
    private static readonly Signed192 Six = Signed192.Signed(6);
    private static readonly Signed192 Twelve = Signed192.Signed(12);

    internal static bool TryCreateUniformShell(
        ReadOnlySpan<Vector3d> vertices,
        ReadOnlySpan<int> triangleIndices,
        out FixedMassWeight surfaceWeight,
        out Vector3d centerOfMass,
        out Fixed3x3 unitMassInertiaTensor)
    {
        if (triangleIndices.Length % FixedTriangle.VertexCount != 0)
        {
            throw new ArgumentException(
                "Triangle indices must contain complete index triplets.",
                nameof(triangleIndices));
        }

        Signed576 totalWeight = default;
        Signed576 firstMomentX = default;
        Signed576 firstMomentY = default;
        Signed576 firstMomentZ = default;
        Signed576 secondMomentX = default;
        Signed576 secondMomentY = default;
        Signed576 secondMomentZ = default;
        Signed576 productMomentXY = default;
        Signed576 productMomentXZ = default;
        Signed576 productMomentYZ = default;
        for (int i = 0; i < triangleIndices.Length; i += FixedTriangle.VertexCount)
        {
            GetTriangle(
                vertices,
                triangleIndices,
                i,
                out FixedTriangle triangle);
            FixedMassWeight weight = triangle.AreaWeight;
            Signed320 weightNumerator = weight.Numerator;
            totalWeight = WideArithmetic.AddSigned576(
                totalWeight,
                Signed576.ExtendValue(weightNumerator));
            firstMomentX = AddWeightedVertexSum(
                firstMomentX,
                triangle.A.X,
                triangle.B.X,
                triangle.C.X,
                weightNumerator);
            firstMomentY = AddWeightedVertexSum(
                firstMomentY,
                triangle.A.Y,
                triangle.B.Y,
                triangle.C.Y,
                weightNumerator);
            firstMomentZ = AddWeightedVertexSum(
                firstMomentZ,
                triangle.A.Z,
                triangle.B.Z,
                triangle.C.Z,
                weightNumerator);
            GetProductSums(
                triangle,
                out Signed320 x2,
                out Signed320 y2,
                out Signed320 z2,
                out Signed320 xy,
                out Signed320 xz,
                out Signed320 yz);
            secondMomentX = AddWeighted(
                secondMomentX,
                x2,
                weightNumerator);
            secondMomentY = AddWeighted(
                secondMomentY,
                y2,
                weightNumerator);
            secondMomentZ = AddWeighted(
                secondMomentZ,
                z2,
                weightNumerator);
            productMomentXY = AddWeighted(
                productMomentXY,
                xy,
                weightNumerator);
            productMomentXZ = AddWeighted(
                productMomentXZ,
                xz,
                weightNumerator);
            productMomentYZ = AddWeighted(
                productMomentYZ,
                yz,
                weightNumerator);
        }

        if (totalWeight.IsZero)
        {
            surfaceWeight = default;
            centerOfMass = default;
            unitMassInertiaTensor = default;
            return false;
        }

        surfaceWeight = new FixedMassWeight(
            Signed320.NarrowValue(totalWeight));
        Signed576 centerDenominator =
            WideArithmetic.MultiplySigned576(
                totalWeight,
                Three);
        _ = Fixed64.TryGetSignedRawRatio(
            firstMomentX,
            centerDenominator,
            out Fixed64 centerX);
        _ = Fixed64.TryGetSignedRawRatio(
            firstMomentY,
            centerDenominator,
            out Fixed64 centerY);
        _ = Fixed64.TryGetSignedRawRatio(
            firstMomentZ,
            centerDenominator,
            out Fixed64 centerZ);
        centerOfMass = new Vector3d(
            centerX,
            centerY,
            centerZ);

        Signed576 centeredX = TranslateSquaredMoment(
            secondMomentX,
            firstMomentX,
            totalWeight,
            centerX);
        Signed576 centeredY = TranslateSquaredMoment(
            secondMomentY,
            firstMomentY,
            totalWeight,
            centerY);
        Signed576 centeredZ = TranslateSquaredMoment(
            secondMomentZ,
            firstMomentZ,
            totalWeight,
            centerZ);
        Signed576 centeredXY = TranslateProductMoment(
            productMomentXY,
            firstMomentX,
            firstMomentY,
            totalWeight,
            centerX,
            centerY);
        Signed576 centeredXZ = TranslateProductMoment(
            productMomentXZ,
            firstMomentX,
            firstMomentZ,
            totalWeight,
            centerX,
            centerZ);
        Signed576 centeredYZ = TranslateProductMoment(
            productMomentYZ,
            firstMomentY,
            firstMomentZ,
            totalWeight,
            centerY,
            centerZ);
        Signed576 inertiaXX =
            WideArithmetic.AddSigned576(centeredY, centeredZ);
        Signed576 inertiaYY =
            WideArithmetic.AddSigned576(centeredX, centeredZ);
        Signed576 inertiaZZ =
            WideArithmetic.AddSigned576(centeredX, centeredY);

        Signed576 diagonalDenominator =
            WideArithmetic.MultiplySigned576(
                totalWeight,
                Signed192.One,
                Six);
        Signed576 productDenominator =
            WideArithmetic.MultiplySigned576(
                totalWeight,
                Signed192.One,
                Twelve);
        bool representable = Fixed64.TryGetSignedRawRatio(
                inertiaXX,
                diagonalDenominator,
                out Fixed64 tensorXX)
            & Fixed64.TryGetSignedRawRatio(
                inertiaYY,
                diagonalDenominator,
                out Fixed64 tensorYY)
            & Fixed64.TryGetSignedRawRatio(
                inertiaZZ,
                diagonalDenominator,
                out Fixed64 tensorZZ)
            & Fixed64.TryGetSignedRawRatio(
                WideArithmetic.SubtractSigned576(default, centeredXY),
                productDenominator,
                out Fixed64 tensorXY)
            & Fixed64.TryGetSignedRawRatio(
                WideArithmetic.SubtractSigned576(default, centeredXZ),
                productDenominator,
                out Fixed64 tensorXZ)
            & Fixed64.TryGetSignedRawRatio(
                WideArithmetic.SubtractSigned576(default, centeredYZ),
                productDenominator,
                out Fixed64 tensorYZ);
        if (!representable)
        {
            surfaceWeight = default;
            centerOfMass = default;
            unitMassInertiaTensor = default;
            return false;
        }

        unitMassInertiaTensor = new Fixed3x3(
            tensorXX, tensorXY, tensorXZ,
            tensorXY, tensorYY, tensorYZ,
            tensorXZ, tensorYZ, tensorZZ);
        return true;
    }

    private static Signed576 AddWeightedVertexSum(
        Signed576 total,
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Signed320 weight)
    {
        Signed320 sum = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                Signed320.ExtendValue(Signed192.Raw(first)),
                Signed320.ExtendValue(Signed192.Raw(second))),
            Signed320.ExtendValue(Signed192.Raw(third)));
        return AddWeighted(total, sum, weight);
    }

    private static Signed576 AddWeighted(
        Signed576 total,
        Signed320 value,
        Signed320 weight) =>
        WideArithmetic.AddSigned576(
            total,
            WideArithmetic.MultiplySigned320(
                value,
                weight));

    private static void GetProductSums(
        FixedTriangle triangle,
        out Signed320 x2,
        out Signed320 y2,
        out Signed320 z2,
        out Signed320 xy,
        out Signed320 xz,
        out Signed320 yz)
    {
        Signed192 ax = Signed192.Raw(triangle.A.X);
        Signed192 ay = Signed192.Raw(triangle.A.Y);
        Signed192 az = Signed192.Raw(triangle.A.Z);
        Signed192 bx = Signed192.Raw(triangle.B.X);
        Signed192 by = Signed192.Raw(triangle.B.Y);
        Signed192 bz = Signed192.Raw(triangle.B.Z);
        Signed192 cx = Signed192.Raw(triangle.C.X);
        Signed192 cy = Signed192.Raw(triangle.C.Y);
        Signed192 cz = Signed192.Raw(triangle.C.Z);
        x2 = SquaredBarycentricSum(ax, bx, cx);
        y2 = SquaredBarycentricSum(ay, by, cy);
        z2 = SquaredBarycentricSum(az, bz, cz);
        xy = BarycentricCrossSum(ax, bx, cx, ay, by, cy);
        xz = BarycentricCrossSum(ax, bx, cx, az, bz, cz);
        yz = BarycentricCrossSum(ay, by, cy, az, bz, cz);
    }

    private static Signed320 SquaredBarycentricSum(
        Signed192 first,
        Signed192 second,
        Signed192 third) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(first, first),
                    WideArithmetic.MultiplySigned192(second, second)),
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(third, third),
                    WideArithmetic.MultiplySigned192(first, second))),
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(first, third),
                WideArithmetic.MultiplySigned192(second, third)));

    private static Signed320 BarycentricCrossSum(
        Signed192 firstA,
        Signed192 firstB,
        Signed192 firstC,
        Signed192 secondA,
        Signed192 secondB,
        Signed192 secondC)
    {
        Signed192 firstSum = Signed192.NarrowProven(
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    Signed320.ExtendValue(firstA),
                    Signed320.ExtendValue(firstB)),
                Signed320.ExtendValue(firstC)));
        Signed192 secondSum = Signed192.NarrowProven(
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    Signed320.ExtendValue(secondA),
                    Signed320.ExtendValue(secondB)),
                Signed320.ExtendValue(secondC)));
        Signed320 matching = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(firstA, secondA),
                WideArithmetic.MultiplySigned192(firstB, secondB)),
            WideArithmetic.MultiplySigned192(firstC, secondC));
        return WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                firstSum,
                secondSum),
            matching);
    }

    private static Signed576 TranslateSquaredMoment(
        Signed576 secondMoment,
        Signed576 firstMoment,
        Signed576 totalWeight,
        Fixed64 center)
    {
        Signed192 centerRaw = Signed192.Raw(center);
        Signed576 shifted = WideArithmetic.SubtractSigned576(
            secondMoment,
            WideArithmetic.MultiplySigned576(
                firstMoment,
                centerRaw,
                Signed192.Signed(4)));
        return WideArithmetic.AddSigned576(
            shifted,
            WideArithmetic.MultiplySigned576(
                totalWeight,
                centerRaw,
                centerRaw,
                Six));
    }

    private static Signed576 TranslateProductMoment(
        Signed576 productMoment,
        Signed576 firstMoment,
        Signed576 secondMoment,
        Signed576 totalWeight,
        Fixed64 firstCenter,
        Fixed64 secondCenter)
    {
        Signed192 firstCenterRaw =
            Signed192.Raw(firstCenter);
        Signed192 secondCenterRaw =
            Signed192.Raw(secondCenter);
        Signed576 shifted = WideArithmetic.SubtractSigned576(
            productMoment,
            WideArithmetic.MultiplySigned576(
                secondMoment,
                firstCenterRaw,
                Signed192.Signed(4)));
        shifted = WideArithmetic.SubtractSigned576(
            shifted,
            WideArithmetic.MultiplySigned576(
                firstMoment,
                secondCenterRaw,
                Signed192.Signed(4)));
        return WideArithmetic.AddSigned576(
            shifted,
            WideArithmetic.MultiplySigned576(
                totalWeight,
                firstCenterRaw,
                secondCenterRaw,
                Twelve));
    }

    private static void GetTriangle(
        ReadOnlySpan<Vector3d> vertices,
        ReadOnlySpan<int> triangleIndices,
        int index,
        out FixedTriangle triangle)
    {
        int first = triangleIndices[index];
        int second = triangleIndices[index + 1];
        int third = triangleIndices[index + 2];
        if ((uint)first >= (uint)vertices.Length
            || (uint)second >= (uint)vertices.Length
            || (uint)third >= (uint)vertices.Length)
        {
            throw new ArgumentOutOfRangeException(
                nameof(triangleIndices),
                "Triangle indices must reference supplied vertices.");
        }

        triangle = new FixedTriangle(
            vertices[first],
            vertices[second],
            vertices[third]);
    }
}
