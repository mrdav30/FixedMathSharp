//=======================================================================
// EnclosedQ96Coefficient.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Retains a signed Q96 coefficient and its absolute error bound in integer
/// Q96 units. Geometry preparation normalizes its sign and widths once;
/// multiplying certifies only unique, representable half-even results.
/// </summary>
internal readonly struct EnclosedQ96Coefficient
{
    private readonly Signed192 _magnitude;
    private readonly uint _errorUnits;
    private readonly sbyte _sign;
    private readonly bool _fits, _coarse;

    internal EnclosedQ96Coefficient(Signed192 value, uint errorUnits)
    {
        WideArithmetic.GetMagnitude(value, out ulong high, out ulong middle, out ulong low);
        _magnitude = new Signed192(high, middle, low);
        _sign = (sbyte)value.Sign;
        _errorUnits = errorUnits;
        _fits = high == 0;
        _coarse = _fits && (middle >> 36) == 0;
    }

    internal Signed192 Value => _sign < 0 ? WideArithmetic.Negate(_magnitude) : _magnitude;

    internal bool Equals(EnclosedQ96Coefficient other) =>
        _sign == other._sign && _errorUnits == other._errorUnits && _magnitude.Equals(other._magnitude);

    /// <summary>
    /// Returns raw Q32.32 bits on success; returns false/zero for uncertain
    /// rounding, a nonzero product whose coefficient magnitude is at least
    /// 2^128, or scalar overflow.
    /// The default coefficient is zero, including its zero error bound.
    /// </summary>
    // Keep scalar construction at materialization, outside inlined certificate outputs.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryMultiplyRaw(Fixed64 value, out long result)
    {
        result = default;
        if (_sign == 0 || value.m_rawValue == 0)
            return true;
        if (!_fits)
            return false;

        ulong magnitude = Fixed64.AbsToUInt64(value.m_rawValue);
        bool negative = (_sign < 0) != (value.m_rawValue < 0);
        if (_coarse && magnitude < (1UL << 58))
        {
            // Q60 needs one product. Discarding 36 bits plus any uint error
            // loses <2 Q60 units; these widths also prove scalar range safety.
            ulong mantissa = (_magnitude.Middle << 28) | (_magnitude.Low >> 36);
            if (mantissa == 0) return true;
            Fixed64.Multiply64To128(mantissa, magnitude, out ulong upper, out ulong lower);
            ulong fraction = lower & ((1UL << 60) - 1);
            const ulong half = 1UL << 59;
            ulong distance = fraction < half ? half - fraction : fraction - half;
            if (distance > 2 * magnitude)
            {
                // The certificate excludes half ties and proves magnitude <=2^62.
                long raw = (long)((upper << 4) | (lower >> 60)) + (fraction > half ? 1L : 0L);
                result = negative ? -raw : raw;
                return true;
            }
        }
        return TryMultiplyFull(_magnitude.Middle, _magnitude.Low, magnitude, negative, _errorUnits, out result);
    }

    private static bool TryMultiplyFull(ulong coefficientMiddle, ulong coefficientLow,
        ulong magnitude, bool negative, uint errorUnits, out long result)
    {
        result = default;
        Fixed64.Multiply64To128(coefficientLow, magnitude, out ulong lowHigh, out ulong low);
        Fixed64.Multiply64To128(coefficientMiddle, magnitude, out ulong highHigh, out ulong highLow);
        ulong middle = unchecked(lowHigh + highLow);
        ulong high = highHigh + (middle < lowHigh ? 1UL : 0UL);
        if ((high >> 32) != 0)
            return false;

        if (errorUnits != 0)
        {
            // The radius is below 2^95 because |valueRaw| <= 2^63 and
            // errorUnits < 2^32. Only the nearest half-integer can change rounding.
            Fixed64.Multiply64To128(magnitude, errorUnits, out ulong errorHigh, out ulong errorLow);
            ulong fractionHigh = (uint)middle;
            const ulong half = 1UL << 31;
            bool belowHalf = fractionHigh < half;
            ulong distanceHigh = belowHalf
                ? half - fractionHigh - (low != 0 ? 1UL : 0UL)
                : fractionHigh - half;
            ulong distanceLow = belowHalf ? unchecked(0UL - low) : low;
            if (distanceHigh < errorHigh || (distanceHigh == errorHigh && distanceLow <= errorLow))
                return false;
        }

        ulong quotient = (high << 32) | (middle >> 32);
        bool guard = (middle & (1UL << 31)) != 0;
        bool sticky = low != 0 || (middle & ((1UL << 31) - 1)) != 0;
        Fixed64 rounded = Fixed64.RoundAndApplySign(quotient, guard, sticky, negative, out bool representable);
        if (!representable)
            return false;
        result = rounded.m_rawValue;
        return true;
    }
}
