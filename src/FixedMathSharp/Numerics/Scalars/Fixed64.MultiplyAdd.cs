//=======================================================================
// Fixed64.MultiplyAdd.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

public partial struct Fixed64
{
    /// <summary>
    /// Multiplies two values, adds a third, and performs one final
    /// round-half-to-even conversion. An unrepresentable result saturates.
    /// </summary>
    public static Fixed64 MultiplyAdd(Fixed64 left, Fixed64 right, Fixed64 addend)
    {
        return MultiplyAdd(left, right, addend, out _);
    }

    /// <summary>
    /// Attempts to multiply two values and add a third with one final
    /// round-half-to-even conversion and no intermediate saturation.
    /// </summary>
    public static bool TryMultiplyAdd(
        Fixed64 left,
        Fixed64 right,
        Fixed64 addend,
        out Fixed64 result)
    {
        result = MultiplyAdd(left, right, addend, out bool representable);
        if (representable)
            return true;

        result = default;
        return false;
    }

    private static Fixed64 MultiplyAdd(
        Fixed64 left,
        Fixed64 right,
        Fixed64 addend,
        out bool representable)
    {
        long leftRaw = left.m_rawValue;
        long rightRaw = right.m_rawValue;
        Multiply64To128(
            AbsToUInt64(leftRaw),
            AbsToUInt64(rightRaw),
            out ulong productHigh,
            out ulong productLow);
        Signed192 product = new(0UL, productHigh, productLow);
        if ((leftRaw ^ rightRaw) < 0L)
            product = WideArithmetic.SubtractSigned192(default, product);

        long addendRaw = addend.m_rawValue;
        ulong extension = addendRaw < 0L ? ulong.MaxValue : 0UL;
        Signed192 scaledAddend = new(
            extension,
            unchecked((ulong)(addendRaw >> FixedMath.SHIFT_AMOUNT_I)),
            unchecked((ulong)addendRaw << FixedMath.SHIFT_AMOUNT_I));
        Signed192 numerator = WideArithmetic.AddSigned192(product, scaledAddend);
        bool negative = numerator.Sign < 0;
        WideArithmetic.GetMagnitude(
            numerator,
            out _,
            out ulong numeratorMiddle,
            out ulong numeratorLow);

        if ((numeratorMiddle >> FixedMath.SHIFT_AMOUNT_I) != 0UL)
        {
            representable = false;
            return negative ? MinValue : MaxValue;
        }

        ulong quotient = (numeratorMiddle << FixedMath.SHIFT_AMOUNT_I)
            | (numeratorLow >> FixedMath.SHIFT_AMOUNT_I);
        ulong guardMask = 1UL << (FixedMath.SHIFT_AMOUNT_I - 1);
        bool guard = (numeratorLow & guardMask) != 0UL;
        bool sticky = (numeratorLow & (guardMask - 1UL)) != 0UL;
        return RoundAndApplySign(
            quotient,
            guard,
            sticky,
            negative,
            out representable);
    }
}
