//=======================================================================
// RaySlabInterval.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>Clips exact forward ray parameters against axis-aligned slabs.</summary>
internal struct RaySlabInterval
{
    private (ulong Delta, ulong Direction) _entry;
    private (ulong Delta, ulong Direction) _exit;

    internal static RaySlabInterval Forward => new()
    {
        _entry = (0UL, 1UL),
        _exit = (long.MaxValue, (ulong)FixedMath.ONE_L)
    };

    internal bool Clip(Fixed64 origin, Fixed64 direction, Fixed64 min, Fixed64 max)
    {
        long position = origin.m_rawValue;
        long lower = min.m_rawValue;
        long upper = max.m_rawValue;
        long step = direction.m_rawValue;
        if (step == 0L)
            return position >= lower && position <= upper;

        ulong near;
        ulong far;
        if (step > 0L)
        {
            if (position > upper)
                return false;

            near = position < lower ? unchecked((ulong)(lower - position)) : 0UL;
            far = unchecked((ulong)(upper - position));
        }
        else
        {
            if (position < lower)
                return false;

            near = position > upper ? unchecked((ulong)(position - upper)) : 0UL;
            far = unchecked((ulong)(position - lower));
        }

        ulong magnitude = Fixed64.AbsToUInt64(step);
        if (Compare((near, magnitude), _entry) > 0)
            _entry = (near, magnitude);
        if (Compare((far, magnitude), _exit) < 0)
            _exit = (far, magnitude);
        return Compare(_entry, _exit) <= 0;
    }

    internal readonly Fixed64 RoundEntry()
    {
        // A nonempty interval certifies the entry is in [0, Fixed64.MaxValue].
        Signed192 numerator = WideArithmetic.ShiftLeftSigned192By32(new Signed192(0, 0, _entry.Delta));
        return Fixed64.GetSignedRawRatio(
            Signed320.ExtendValue(numerator), new Signed192(0, 0, _entry.Direction));
    }

    private static int Compare((ulong Delta, ulong Direction) left, (ulong Delta, ulong Direction) right)
    {
        if (left.Direction == right.Direction)
            return left.Delta.CompareTo(right.Delta);

        // Nonnegative endpoint differences fit in 64 bits, so both cross-products fit in 128.
        Fixed64.Multiply64To128(left.Delta, right.Direction, out ulong leftHigh, out ulong leftLow);
        Fixed64.Multiply64To128(right.Delta, left.Direction, out ulong rightHigh, out ulong rightLow);
        return WideArithmetic.CompareUnsigned(0, leftHigh, leftLow, 0, rightHigh, rightLow);
    }
}
