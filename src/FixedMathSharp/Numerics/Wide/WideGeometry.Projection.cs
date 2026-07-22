//=======================================================================
// WideGeometry.Projection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

internal static partial class WideGeometry
{
    /// <summary>
    /// Returns max(0, dot(target - source, direction) / dot(direction, direction))
    /// without narrowing either dot product first.
    /// </summary>
    internal static Fixed64 GetNonNegativeDifferenceProjectionParameter(
        Vector3d target,
        Vector3d source,
        Vector3d direction)
    {
        Signed192 numerator = GetDifferenceDotProduct3D(
            target.X, source.X,
            target.Y, source.Y,
            target.Z, source.Z,
            direction.X, Fixed64.Zero,
            direction.Y, Fixed64.Zero,
            direction.Z, Fixed64.Zero);
        if (numerator.Sign <= 0)
            return Fixed64.Zero;

        Signed192 denominator = GetDifferenceDotProduct3D(
            direction.X, Fixed64.Zero,
            direction.Y, Fixed64.Zero,
            direction.Z, Fixed64.Zero,
            direction.X, Fixed64.Zero,
            direction.Y, Fixed64.Zero,
            direction.Z, Fixed64.Zero);
        return Fixed64.GetSignedRatio(numerator, denominator);
    }

    /// <summary>
    /// Returns normalize(q * vector - normal * dot(vector, normal)), where
    /// q is the exact squared length of the normal.
    /// </summary>
    internal static Vector3d GetNormalizedProjectionOnPlane(
        Vector3d vector,
        Vector3d normal)
    {
        Signed192 q = GetDifferenceDotProduct3D(
            normal.X, Fixed64.Zero,
            normal.Y, Fixed64.Zero,
            normal.Z, Fixed64.Zero,
            normal.X, Fixed64.Zero,
            normal.Y, Fixed64.Zero,
            normal.Z, Fixed64.Zero);
        if (q.Sign == 0)
            return GetNormalized(vector);

        Signed192 projection = GetDifferenceDotProduct3D(
            vector.X, Fixed64.Zero,
            vector.Y, Fixed64.Zero,
            vector.Z, Fixed64.Zero,
            normal.X, Fixed64.Zero,
            normal.Y, Fixed64.Zero,
            normal.Z, Fixed64.Zero);
        Signed192 x = WideArithmetic.FromSignedRaw(vector.X.m_rawValue);
        Signed192 y = WideArithmetic.FromSignedRaw(vector.Y.m_rawValue);
        Signed192 z = WideArithmetic.FromSignedRaw(vector.Z.m_rawValue);
        Signed192 nx = WideArithmetic.FromSignedRaw(normal.X.m_rawValue);
        Signed192 ny = WideArithmetic.FromSignedRaw(normal.Y.m_rawValue);
        Signed192 nz = WideArithmetic.FromSignedRaw(normal.Z.m_rawValue);
        return GetNormalized(
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(q, x),
                WideArithmetic.MultiplySigned192(nx, projection)),
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(q, y),
                WideArithmetic.MultiplySigned192(ny, projection)),
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(q, z),
                WideArithmetic.MultiplySigned192(nz, projection)));
    }
}
