namespace FixedMathSharp.Geometry;

/// <summary>
/// Contains final velocity changes from an exact Coulomb friction response.
/// </summary>
public readonly struct FixedLeverCoulombResponse3d
{
    private readonly Fixed64 _primaryAccumulatedImpulse;
    private readonly Fixed64 _secondaryAccumulatedImpulse;
    private readonly byte _projectionFlags;

    internal FixedLeverCoulombResponse3d(
        bool hasAppliedImpulse,
        Vector3d firstLinearVelocityDelta,
        Vector3d firstAngularVelocityDelta,
        Vector3d secondLinearVelocityDelta,
        Vector3d secondAngularVelocityDelta,
        bool hasPrimaryAccumulatedImpulse,
        Fixed64 primaryAccumulatedImpulse,
        bool hasSecondaryAccumulatedImpulse,
        Fixed64 secondaryAccumulatedImpulse)
    {
        HasAppliedImpulse = hasAppliedImpulse;
        FirstLinearVelocityDelta = firstLinearVelocityDelta;
        FirstAngularVelocityDelta = firstAngularVelocityDelta;
        SecondLinearVelocityDelta = secondLinearVelocityDelta;
        SecondAngularVelocityDelta = secondAngularVelocityDelta;
        _primaryAccumulatedImpulse = primaryAccumulatedImpulse;
        _secondaryAccumulatedImpulse = secondaryAccumulatedImpulse;
        _projectionFlags = (byte)(
            (hasPrimaryAccumulatedImpulse ? 1 : 0)
            | (hasSecondaryAccumulatedImpulse ? 2 : 0));
    }

    /// <summary>Gets whether the exact applied tangent impulse is nonzero.</summary>
    public bool HasAppliedImpulse { get; }

    /// <summary>Gets the first participant's linear velocity change.</summary>
    public Vector3d FirstLinearVelocityDelta { get; }

    /// <summary>Gets the first participant's angular velocity change.</summary>
    public Vector3d FirstAngularVelocityDelta { get; }

    /// <summary>Gets the second participant's linear velocity change.</summary>
    public Vector3d SecondLinearVelocityDelta { get; }

    /// <summary>Gets the second participant's angular velocity change.</summary>
    public Vector3d SecondAngularVelocityDelta { get; }

    /// <summary>
    /// Attempts to project the completed primary tangent accumulator to Q32.32.
    /// </summary>
    public bool TryGetPrimaryAccumulatedImpulse(out Fixed64 value)
    {
        value = _primaryAccumulatedImpulse;
        return (_projectionFlags & 1) != 0;
    }

    /// <summary>
    /// Attempts to project the completed secondary tangent accumulator to Q32.32.
    /// </summary>
    public bool TryGetSecondaryAccumulatedImpulse(out Fixed64 value)
    {
        value = _secondaryAccumulatedImpulse;
        return (_projectionFlags & 2) != 0;
    }
}
