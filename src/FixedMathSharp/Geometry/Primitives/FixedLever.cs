//=======================================================================
// FixedLever.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Represents an exact directed displacement between two 3D point anchors.
/// </summary>
/// <remarks>
/// A lever can retain components outside the Q32.32 scalar domain. Operations
/// return <see langword="false"/> when this value is uninitialized, an explicit
/// divisor is zero, or their final public result is unrepresentable.
/// </remarks>
public readonly struct FixedLever
{
    internal readonly Signed576 XNumerator;
    internal readonly Signed576 YNumerator;
    internal readonly Signed576 ZNumerator;
    internal readonly Signed576 Denominator;

    internal FixedLever(
        Signed576 xNumerator,
        Signed576 yNumerator,
        Signed576 zNumerator,
        Signed576 denominator)
    {
        XNumerator = xNumerator;
        YNumerator = yNumerator;
        ZNumerator = zNumerator;
        Denominator = denominator;
    }

    /// <summary>
    /// Attempts to materialize the exact displacement as a Q32.32 vector.
    /// </summary>
    public bool TryGetVector(out Vector3d vector) =>
        WideOrientedBox.TryGetLeverVector(this, out vector);

    /// <summary>
    /// Attempts to evaluate
    /// <c>Dot(Cross(this, crossVector), projectionVector)</c>.
    /// </summary>
    public bool TryGetCrossProductProjection(
        Vector3d crossVector,
        Vector3d projectionVector,
        out Fixed64 projection) =>
        WideOrientedBox.TryGetCrossProductProjection(
            this,
            crossVector,
            projectionVector,
            out projection);

    /// <summary>
    /// Attempts to evaluate
    /// <c>Dot(c, Fixed3x3.TransformDirection(transform, c))</c>, where
    /// <c>c = Cross(this, crossVector)</c>.
    /// </summary>
    /// <remarks>
    /// Matrices use FixedMathSharp's row-vector convention.
    /// </remarks>
    public bool TryGetCrossProductQuadraticForm(
        Vector3d crossVector,
        Fixed3x3 transform,
        out Fixed64 result) =>
        WideOrientedBox.TryGetCrossProductQuadraticForm(
            this,
            crossVector,
            transform,
            out result);

    /// <summary>
    /// Attempts to transform <c>Cross(this, crossVector)</c> and apply one
    /// fused multiply-divide before the final component conversions.
    /// </summary>
    public bool TryGetTransformedScaledCrossProduct(
        Vector3d crossVector,
        Fixed3x3 transform,
        Fixed64 multiplier,
        Fixed64 divisor,
        out Vector3d result) =>
        TryGetTransformedScaledCrossProduct(
            crossVector,
            transform,
            multiplier,
            Fixed64.One,
            divisor,
            out result);

    /// <summary>
    /// Attempts to transform <c>Cross(this, crossVector)</c> and apply two
    /// fused multipliers and one divisor before the final component
    /// conversions.
    /// </summary>
    public bool TryGetTransformedScaledCrossProduct(
        Vector3d crossVector,
        Fixed3x3 transform,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Fixed64 divisor,
        out Vector3d result) =>
        WideOrientedBox.TryGetTransformedScaledCrossProduct(
            this,
            crossVector,
            transform,
            firstMultiplier,
            secondMultiplier,
            divisor,
            out result);
}
