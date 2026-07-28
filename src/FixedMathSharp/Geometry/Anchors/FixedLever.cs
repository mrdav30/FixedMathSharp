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
    /// Attempts to project the relative velocity of two points whose lever
    /// arms may exceed the Q32.32 scalar domain.
    /// </summary>
    /// <remarks>
    /// Evaluates
    /// <c>
    /// Dot(
    /// (secondLinearVelocity + Cross(secondAngularVelocity, secondLever))
    /// - (firstLinearVelocity + Cross(firstAngularVelocity, firstLever)),
    /// projectionAxis)
    /// </c>
    /// and rounds only the final scalar.
    /// </remarks>
    public static bool TryGetRelativePointVelocityProjection(
        Vector3d firstLinearVelocity,
        Vector3d firstAngularVelocity,
        in FixedLever firstLever,
        Vector3d secondLinearVelocity,
        Vector3d secondAngularVelocity,
        in FixedLever secondLever,
        Vector3d projectionAxis,
        out Fixed64 projection) =>
        WideOrientedBox.TryGetRelativePointVelocityProjection(
            firstLinearVelocity,
            firstAngularVelocity,
            firstLever,
            secondLinearVelocity,
            secondAngularVelocity,
            secondLever,
            projectionAxis,
            out projection);

    /// <summary>
    /// Attempts to resolve an unaccumulated unilateral normal response while
    /// retaining exact point velocity, effective mass, and impulse ratios
    /// until the final velocity changes.
    /// </summary>
    /// <remarks>
    /// <paramref name="normal"/> must be normalized.
    /// </remarks>
    public static bool TryGetNormalResponse(
        in FixedLeverResponseOperand3d first,
        in FixedLeverResponseOperand3d second,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out FixedLeverNormalResponse3d response) =>
        WideOrientedBox.TryGetNormalResponse(
            first,
            second,
            normal,
            restitution,
            restitutionVelocityThreshold,
            accumulatedImpulse: Fixed64.Zero,
            positiveImpulseScale: Fixed64.One,
            negativeImpulseScale: Fixed64.One,
            includeAccumulator: false,
            out response);

    /// <summary>
    /// Attempts to resolve an accumulated unilateral normal response while
    /// retaining exact point velocity, effective mass, and impulse ratios
    /// until the final velocity changes.
    /// </summary>
    /// <remarks>
    /// <paramref name="normal"/> must be normalized.
    /// </remarks>
    public static bool TryGetAccumulatedNormalResponse(
        in FixedLeverResponseOperand3d first,
        in FixedLeverResponseOperand3d second,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale,
        out FixedLeverNormalResponse3d response) =>
        WideOrientedBox.TryGetNormalResponse(
            first,
            second,
            normal,
            restitution,
            restitutionVelocityThreshold,
            accumulatedImpulse,
            positiveImpulseScale,
            negativeImpulseScale,
            includeAccumulator: true,
            out response);

    /// <summary>
    /// Attempts to resolve one accumulated tangent impulse against a Coulomb
    /// friction interval while retaining exact response ratios until the final
    /// velocity changes.
    /// </summary>
    /// <remarks>
    /// <paramref name="tangent"/> must be normalized and orthogonal to the
    /// constraint normal. The normal and tangent operands must describe the
    /// same participants. Friction coefficients must be nonnegative.
    /// </remarks>
    public static bool TryGetCoulombLineResponse(
        in FixedLeverNormalConstraint3d normalConstraint,
        in FixedLeverResponseOperand3d firstTangent,
        in FixedLeverResponseOperand3d secondTangent,
        Vector3d tangent,
        Fixed64 accumulatedTangentImpulse,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        out FixedLeverCoulombResponse3d response) =>
        WideOrientedBox.TryGetCoulombLineResponse(
            normalConstraint,
            firstTangent,
            secondTangent,
            tangent,
            accumulatedTangentImpulse,
            staticFriction,
            dynamicFriction,
            out response);

    /// <summary>
    /// Attempts to resolve two orthogonal tangent impulses against a Coulomb
    /// friction disk while retaining exact response ratios until the final
    /// velocity changes.
    /// </summary>
    /// <remarks>
    /// Both tangents must be normalized, mutually orthogonal, and orthogonal to
    /// the constraint normal. All operands must describe the same two
    /// participants. Friction coefficients must be nonnegative.
    /// </remarks>
    public static bool TryGetCoulombDiskResponse(
        in FixedLeverNormalConstraint3d normalConstraint,
        in FixedLeverResponseOperand3d primaryFirst,
        in FixedLeverResponseOperand3d primarySecond,
        Vector3d primaryTangent,
        in FixedLeverResponseOperand3d secondaryFirst,
        in FixedLeverResponseOperand3d secondarySecond,
        Vector3d secondaryTangent,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        out FixedLeverCoulombResponse3d response) =>
        WideOrientedBox.TryGetCoulombDiskResponse(
            normalConstraint,
            primaryFirst,
            primarySecond,
            primaryTangent,
            secondaryFirst,
            secondarySecond,
            secondaryTangent,
            staticFriction,
            dynamicFriction,
            out response);

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
    /// Attempts to transform <c>Cross(this, crossVector)</c>, apply two fused
    /// multipliers, and divide by the exact sum of four terms.
    /// </summary>
    public bool TryGetTransformedScaledCrossProductBySum(
        Vector3d crossVector,
        Fixed3x3 transform,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Fixed64 firstDivisorTerm,
        Fixed64 secondDivisorTerm,
        Fixed64 thirdDivisorTerm,
        Fixed64 fourthDivisorTerm,
        out Vector3d result) =>
        WideOrientedBox.TryGetTransformedScaledCrossProductBySum(
            this,
            crossVector,
            transform,
            firstMultiplier,
            secondMultiplier,
            firstDivisorTerm,
            secondDivisorTerm,
            thirdDivisorTerm,
            fourthDivisorTerm,
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

    /// <summary>
    /// Attempts to transform the cross product of this lever and a
    /// three-term weighted vector, rounding only the completed components.
    /// </summary>
    public bool TryGetTransformedWeightedCrossProduct(
        Vector3d first,
        Fixed64 firstScale,
        Vector3d second,
        Fixed64 secondScale,
        Vector3d third,
        Fixed64 thirdScale,
        Fixed3x3 transform,
        out Vector3d result) =>
        WideOrientedBox.TryGetTransformedWeightedCrossProduct(
            this,
            first,
            firstScale,
            second,
            secondScale,
            third,
            thirdScale,
            transform,
            out result);
}
