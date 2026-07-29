//=======================================================================
// WideGeometry.Normalization.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides high-precision normalization and direction calculations
/// for geometric vectors using wide (extended-precision) arithmetic.
/// </content>
internal static partial class WideGeometry
{
    /// <summary>
    /// Returns the normalized direction between two 2D endpoints without
    /// narrowing their component differences.
    /// </summary>
    internal static Vector2d GetDirection(Vector2d start, Vector2d end)
    {
        Signed192 x = WideArithmetic.Difference(end.X, start.X);
        Signed192 y = WideArithmetic.Difference(end.Y, start.Y);
        return GetNormalized(x, y);
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for a nonzero 2D
    /// vector using its exact raw components.
    /// </summary>
    internal static Vector2d GetNormalized(Vector2d value) =>
        GetNormalized(
            Signed192.Signed(value.X.m_rawValue),
            Signed192.Signed(value.Y.m_rawValue));

    internal static Vector2d GetNormalized(Signed192 x, Signed192 y)
    {
        Signed320 squaredMagnitude = GetSquaredMagnitude(
            x,
            y,
            default,
            out Signed320 xSquare,
            out Signed320 ySquare,
            out _);
        if (squaredMagnitude.IsZero)
            return Vector2d.Zero;

        if (Max(GetMagnitudeBitLength(x), GetMagnitudeBitLength(y))
            <= FixedMath.SHIFT_AMOUNT_I + 1)
        {
            return Vector2d.GetScaleNormalized(new Vector2d(
                Fixed64.FromRaw(unchecked((long)x.Low)),
                Fixed64.FromRaw(unchecked((long)y.Low))));
        }

        Signed192 magnitude = WideArithmetic.GetFloorSquareRoot(
            squaredMagnitude,
            out Signed192 remainder);
        Signed192 ceilingMagnitude = remainder.IsZero
            ? magnitude
            : WideArithmetic.AddSigned192(
                magnitude,
                Signed192.Signed(1L));
        return new Vector2d(
            Fixed64.NormalizeWideComponent(
                x,
                xSquare,
                ceilingMagnitude,
                squaredMagnitude),
            Fixed64.NormalizeWideComponent(
                y,
                ySquare,
                ceilingMagnitude,
                squaredMagnitude));
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for exact 2D
    /// components wider than the public scalar domain.
    /// </summary>
    internal static Vector2d GetNormalized(Signed320 x, Signed320 y)
    {
        Signed320 largest =
            WideArithmetic.CompareMagnitude(x, y) >= 0 ? x : y;
        if (largest.IsZero)
            return Vector2d.Zero;

        Signed320 scale = GetPositiveMagnitude(largest);
        return Vector2d.GetScaleNormalized(new Vector2d(
            Fixed64.GetSignedRatio(x, scale),
            Fixed64.GetSignedRatio(y, scale)));
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for exact 2D
    /// components in the nine-word finite-axis domain.
    /// </summary>
    internal static Vector2d GetNormalized(Signed576 x, Signed576 y)
    {
        Signed576 xMagnitude = GetPositiveMagnitude(x);
        Signed576 yMagnitude = GetPositiveMagnitude(y);
        Signed576 scale =
            WideArithmetic.CompareNonNegative(xMagnitude, yMagnitude) >= 0
                ? xMagnitude
                : yMagnitude;
        if (scale.IsZero)
            return Vector2d.Zero;

        Signed320 fixedScale = Signed320.ExtendValue(
            Signed192.Signed(Fixed64.One.m_rawValue));
        Signed704 denominator = Signed704.ExtendValue(scale);
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(x, fixedScale),
            denominator,
            out Fixed64 scaledX);
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(y, fixedScale),
            denominator,
            out Fixed64 scaledY);
        return Vector2d.GetScaleNormalized(new Vector2d(scaledX, scaledY));
    }

    /// <summary>
    /// Returns the normalized direction between two 3D endpoints without
    /// narrowing their component differences.
    /// </summary>
    internal static Vector3d GetDirection(Vector3d start, Vector3d end)
    {
        Signed192 x = WideArithmetic.Difference(end.X, start.X);
        Signed192 y = WideArithmetic.Difference(end.Y, start.Y);
        Signed192 z = WideArithmetic.Difference(end.Z, start.Z);
        return GetNormalized(x, y, z);
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for a nonzero 3D
    /// vector using its exact raw components.
    /// </summary>
    internal static Vector3d GetNormalized(Vector3d value) =>
        GetNormalized(
            Signed192.Signed(value.X.m_rawValue),
            Signed192.Signed(value.Y.m_rawValue),
            Signed192.Signed(value.Z.m_rawValue));

    internal static Vector3d GetNormalized(
        Signed192 x,
        Signed192 y,
        Signed192 z)
    {
        Signed320 squaredMagnitude = GetSquaredMagnitude(
            x,
            y,
            z,
            out Signed320 xSquare,
            out Signed320 ySquare,
            out Signed320 zSquare);
        if (squaredMagnitude.IsZero)
            return Vector3d.Zero;

        if (Max(
                Max(
                    GetMagnitudeBitLength(x),
                    GetMagnitudeBitLength(y)),
                GetMagnitudeBitLength(z))
            <= FixedMath.SHIFT_AMOUNT_I + 1)
        {
            return Vector3d.GetScaleNormalized(new Vector3d(
                Fixed64.FromRaw(unchecked((long)x.Low)),
                Fixed64.FromRaw(unchecked((long)y.Low)),
                Fixed64.FromRaw(unchecked((long)z.Low))));
        }

        Signed192 magnitude = WideArithmetic.GetFloorSquareRoot(
            squaredMagnitude,
            out Signed192 remainder);
        Signed192 ceilingMagnitude = remainder.IsZero
            ? magnitude
            : WideArithmetic.AddSigned192(
                magnitude,
                Signed192.Signed(1L));
        return new Vector3d(
            Fixed64.NormalizeWideComponent(
                x,
                xSquare,
                ceilingMagnitude,
                squaredMagnitude),
            Fixed64.NormalizeWideComponent(
                y,
                ySquare,
                ceilingMagnitude,
                squaredMagnitude),
            Fixed64.NormalizeWideComponent(
                z,
                zSquare,
                ceilingMagnitude,
                squaredMagnitude));
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for exact 3D
    /// components wider than the public scalar domain.
    /// </summary>
    internal static Vector3d GetNormalized(
        Signed320 x,
        Signed320 y,
        Signed320 z)
    {
        Signed320 largest =
            WideArithmetic.CompareMagnitude(x, y) >= 0 ? x : y;
        if (WideArithmetic.CompareMagnitude(z, largest) > 0)
            largest = z;
        if (largest.IsZero)
            return Vector3d.Zero;

        Signed320 scale = GetPositiveMagnitude(largest);
        return Vector3d.GetScaleNormalized(new Vector3d(
            Fixed64.GetSignedRatio(x, scale),
            Fixed64.GetSignedRatio(y, scale),
            Fixed64.GetSignedRatio(z, scale)));
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for exact 3D
    /// components in the nine-word finite-axis domain.
    /// </summary>
    internal static Vector3d GetNormalized(
        Signed576 x,
        Signed576 y,
        Signed576 z)
    {
        Signed576 xMagnitude = GetPositiveMagnitude(x);
        Signed576 yMagnitude = GetPositiveMagnitude(y);
        Signed576 zMagnitude = GetPositiveMagnitude(z);
        Signed576 scale =
            WideArithmetic.CompareNonNegative(xMagnitude, yMagnitude) >= 0
                ? xMagnitude
                : yMagnitude;
        if (WideArithmetic.CompareNonNegative(zMagnitude, scale) > 0)
            scale = zMagnitude;
        if (scale.IsZero)
            return Vector3d.Zero;

        Signed320 fixedScale = Signed320.ExtendValue(
            Signed192.Signed(Fixed64.One.m_rawValue));
        Signed704 denominator = Signed704.ExtendValue(scale);
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(x, fixedScale),
            denominator,
            out Fixed64 scaledX);
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(y, fixedScale),
            denominator,
            out Fixed64 scaledY);
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(z, fixedScale),
            denominator,
            out Fixed64 scaledZ);
        return Vector3d.GetScaleNormalized(
            new Vector3d(scaledX, scaledY, scaledZ));
    }

    /// <summary>
    /// Returns the nearest representable normalized direction for a nonzero 4D
    /// vector using its exact raw components.
    /// </summary>
    internal static Vector4d GetNormalized(Vector4d value)
    {
        Signed192 x = Signed192.Signed(value.X.m_rawValue);
        Signed192 y = Signed192.Signed(value.Y.m_rawValue);
        Signed192 z = Signed192.Signed(value.Z.m_rawValue);
        Signed192 w = Signed192.Signed(value.W.m_rawValue);
        Signed320 squaredMagnitude = GetSquaredMagnitude(
            x,
            y,
            z,
            out Signed320 xSquare,
            out Signed320 ySquare,
            out Signed320 zSquare);
        Signed320 wSquare = WideArithmetic.MultiplySigned192(w, w);
        squaredMagnitude =
            WideArithmetic.AddSigned320(squaredMagnitude, wSquare);
        if (squaredMagnitude.IsZero)
            return Vector4d.Zero;

        Signed192 magnitude = WideArithmetic.GetFloorSquareRoot(
            squaredMagnitude,
            out Signed192 remainder);
        Signed192 ceilingMagnitude = remainder.IsZero
            ? magnitude
            : WideArithmetic.AddSigned192(
                magnitude,
                Signed192.Signed(1L));
        return new Vector4d(
            Fixed64.NormalizeWideComponent(
                x,
                xSquare,
                ceilingMagnitude,
                squaredMagnitude),
            Fixed64.NormalizeWideComponent(
                y,
                ySquare,
                ceilingMagnitude,
                squaredMagnitude),
            Fixed64.NormalizeWideComponent(
                z,
                zSquare,
                ceilingMagnitude,
                squaredMagnitude),
            Fixed64.NormalizeWideComponent(
                w,
                wSquare,
                ceilingMagnitude,
                squaredMagnitude));
    }

    /// <summary>
    /// Returns the nearest representable unit quaternion for nonzero raw
    /// components. The zero quaternion retains its public identity fallback.
    /// </summary>
    internal static FixedQuaternion GetNormalized(FixedQuaternion value)
    {
        Vector4d normalized = GetNormalized(
            new Vector4d(value.X, value.Y, value.Z, value.W));
        return normalized.IsZero
            ? FixedQuaternion.Identity
            : new FixedQuaternion(
                normalized.X,
                normalized.Y,
                normalized.Z,
                normalized.W);
    }

    private static Signed576 GetPositiveMagnitude(Signed576 value) =>
        value.Sign < 0
            ? WideArithmetic.SubtractSigned576(default, value)
            : value;

    private static Signed320 GetPositiveMagnitude(Signed320 value)
    {
        WideArithmetic.GetMagnitude(
            value,
            out ulong word4,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        return new Signed320(word4, word3, word2, word1, word0);
    }

    private static int GetMagnitudeBitLength(Signed192 value)
    {
        WideArithmetic.GetMagnitude(
            value,
            out ulong high,
            out ulong middle,
            out ulong low);
        return WideArithmetic.GetBitLength(high, middle, low);
    }

    private static int Max(int left, int right) =>
        left >= right ? left : right;
}
