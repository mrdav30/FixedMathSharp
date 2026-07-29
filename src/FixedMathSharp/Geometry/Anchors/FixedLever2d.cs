//=======================================================================
// FixedLever2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Represents an exact directed displacement between two 2D point anchors.
/// </summary>
/// <remarks>
/// A lever can retain components outside the Q32.32 scalar domain. Operations
/// return <see langword="false"/> when this value is uninitialized, an explicit
/// divisor is zero, or their final public result is unrepresentable.
/// </remarks>
public readonly struct FixedLever2d
{
    internal readonly Signed320 XNumerator;
    internal readonly Signed320 YNumerator;
    internal readonly Signed320 Denominator;

    internal FixedLever2d(
        Signed320 xNumerator,
        Signed320 yNumerator,
        Signed320 denominator)
    {
        XNumerator = xNumerator;
        YNumerator = yNumerator;
        Denominator = denominator;
    }

    /// <summary>
    /// Attempts to materialize the exact displacement as a Q32.32 vector.
    /// </summary>
    public bool TryGetVector(out Vector2d vector)
    {
        Signed576 denominator = Signed576.ExtendValue(Denominator);
        bool representable = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(XNumerator),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(YNumerator),
                denominator,
                out Fixed64 y);
        vector = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    /// <summary>
    /// Embeds this exact 2D displacement in the X/Z plane.
    /// </summary>
    public FixedLever ToXZLever() =>
        new(
            Signed576.ExtendValue(XNumerator),
            default,
            Signed576.ExtendValue(YNumerator),
            Signed576.ExtendValue(Denominator));

    /// <summary>
    /// Attempts to evaluate the scalar cross product with
    /// <paramref name="vector"/>.
    /// </summary>
    public bool TryGetCrossProduct(
        Vector2d vector,
        out Fixed64 crossProduct) =>
        TryGetScaledCrossProduct(
            vector,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            out crossProduct);

    /// <summary>
    /// Attempts to evaluate the scalar cross product with
    /// <paramref name="vector"/>, followed by one fused multiply-divide.
    /// </summary>
    public bool TryGetScaledCrossProduct(
        Vector2d vector,
        Fixed64 multiplier,
        Fixed64 divisor,
        out Fixed64 crossProduct) =>
        TryGetScaledCrossProduct(
            vector,
            multiplier,
            Fixed64.One,
            divisor,
            out crossProduct);

    /// <summary>
    /// Attempts to evaluate the scalar cross product with
    /// <paramref name="vector"/>, followed by two fused multipliers and one
    /// divisor.
    /// </summary>
    public bool TryGetScaledCrossProduct(
        Vector2d vector,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Fixed64 divisor,
        out Fixed64 crossProduct)
    {
        Signed576 numerator = GetCrossProductNumerator(vector);
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            Denominator,
            Signed320.ExtendValue(Signed192.One));
        Signed320 numeratorScale = WideArithmetic.MultiplySigned192(
            Signed192.Raw(firstMultiplier),
            Signed192.Raw(secondMultiplier));
        Signed320 denominatorScale = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.Raw(divisor));
        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(
                numerator,
                numeratorScale),
            WideArithmetic.MultiplySigned576ToSigned704(
                denominator,
                denominatorScale),
            out crossProduct);
    }

    /// <summary>
    /// Attempts to evaluate the squared scalar cross product with
    /// <paramref name="vector"/>, followed by <paramref name="scale"/>.
    /// </summary>
    public bool TryGetScaledSquaredCrossProduct(
        Vector2d vector,
        Fixed64 scale,
        out Fixed64 squaredCrossProduct)
    {
        Signed576 crossNumerator = GetCrossProductNumerator(vector);
        Signed576 crossDenominator = WideArithmetic.MultiplySigned320(
            Denominator,
            Signed320.ExtendValue(Signed192.One));
        Signed832 numerator = WideArithmetic.MultiplySigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                crossNumerator,
                crossNumerator),
            Signed192.Raw(scale));
        Signed832 denominator =
            WideArithmetic.MultiplySigned576ToSigned832(
                crossDenominator,
                crossDenominator);
        denominator = WideArithmetic.MultiplySigned832(
            WideArithmetic.MultiplySigned832(
                denominator,
                Signed192.One),
            Signed192.One);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            0,
            out squaredCrossProduct);
    }

    private Signed576 GetCrossProductNumerator(Vector2d vector) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                XNumerator,
                Signed320.ExtendValue(
                    Signed192.Raw(vector.Y))),
            WideArithmetic.MultiplySigned320(
                YNumerator,
                Signed320.ExtendValue(
                    Signed192.Raw(vector.X))));
}
