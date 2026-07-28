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
    public bool TryGetVector(out Vector2d vector) =>
        WideVector2dTransform.TryGetLeverVector(this, out vector);

    /// <summary>
    /// Attempts to evaluate the scalar cross product with
    /// <paramref name="vector"/>.
    /// </summary>
    public bool TryGetCrossProduct(
        Vector2d vector,
        out Fixed64 crossProduct) =>
        WideVector2dTransform.TryGetScaledCrossProduct(
            this,
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
        out Fixed64 crossProduct) =>
        WideVector2dTransform.TryGetScaledCrossProduct(
            this,
            vector,
            firstMultiplier,
            secondMultiplier,
            divisor,
            out crossProduct);

    /// <summary>
    /// Attempts to evaluate the squared scalar cross product with
    /// <paramref name="vector"/>, followed by <paramref name="scale"/>.
    /// </summary>
    public bool TryGetScaledSquaredCrossProduct(
        Vector2d vector,
        Fixed64 scale,
        out Fixed64 squaredCrossProduct) =>
        WideVector2dTransform.TryGetScaledSquaredCrossProduct(
            this,
            vector,
            scale,
            out squaredCrossProduct);
}
