//=======================================================================
// WideRadialGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// Owns exact scalar reductions used by radial geometry.
/// </summary>
internal static class WideRadialGeometry
{
    internal static bool TryGetCircleCrossSectionRadius(
        Fixed64 radius,
        Fixed64 offset,
        out Fixed64 crossSectionRadius)
    {
        Signed192 radiusRaw = WideArithmetic.FromSignedRaw(radius.m_rawValue);
        Signed192 offsetRaw = WideArithmetic.FromSignedRaw(offset.m_rawValue);
        Signed320 squaredRadius = WideArithmetic.MultiplySigned192(radiusRaw, radiusRaw);
        Signed320 squaredOffset = WideArithmetic.MultiplySigned192(offsetRaw, offsetRaw);
        Signed320 squaredCrossSection = WideArithmetic.SubtractSigned320(squaredRadius, squaredOffset);
        if (squaredCrossSection.Sign < 0)
        {
            crossSectionRadius = Fixed64.Zero;
            return false;
        }

        Signed192 root = WideArithmetic.GetFloorSquareRoot(
            squaredCrossSection,
            out Signed192 remainder);
        WideArithmetic.GetMagnitude(root, out _, out _, out ulong rawRadius);
        if (WideArithmetic.CompareMagnitude(remainder, root) > 0)
            rawRadius++;

        crossSectionRadius = Fixed64.FromRaw((long)rawRadius);
        return true;
    }
}
