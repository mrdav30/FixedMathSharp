namespace FixedMathSharp.Geometry;

/// <summary>
/// Describes the normal constraint that supports a Coulomb friction response.
/// </summary>
public readonly struct FixedLeverNormalConstraint3d
{
    /// <summary>Creates a normal constraint.</summary>
    public FixedLeverNormalConstraint3d(
        in FixedLeverResponseOperand3d first,
        in FixedLeverResponseOperand3d second,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale)
    {
        First = first;
        Second = second;
        Normal = normal;
        Restitution = restitution;
        RestitutionVelocityThreshold = restitutionVelocityThreshold;
        AccumulatedImpulse = accumulatedImpulse;
        PositiveImpulseScale = positiveImpulseScale;
        NegativeImpulseScale = negativeImpulseScale;
    }

    /// <summary>Gets the first normal-response operand.</summary>
    public FixedLeverResponseOperand3d First { get; }

    /// <summary>Gets the second normal-response operand.</summary>
    public FixedLeverResponseOperand3d Second { get; }

    /// <summary>Gets the normalized contact normal.</summary>
    public Vector3d Normal { get; }

    /// <summary>Gets the nonnegative restitution coefficient.</summary>
    public Fixed64 Restitution { get; }

    /// <summary>Gets the nonnegative restitution velocity threshold.</summary>
    public Fixed64 RestitutionVelocityThreshold { get; }

    /// <summary>Gets the nonnegative accumulated normal impulse.</summary>
    public Fixed64 AccumulatedImpulse { get; }

    /// <summary>Gets the nonnegative closing-impulse scale.</summary>
    public Fixed64 PositiveImpulseScale { get; }

    /// <summary>Gets the nonnegative separating-impulse scale.</summary>
    public Fixed64 NegativeImpulseScale { get; }
}