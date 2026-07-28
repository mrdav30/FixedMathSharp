namespace FixedMathSharp.Geometry;

/// <summary>
/// Contains final velocity changes from an exact point-anchor normal response.
/// </summary>
public readonly struct FixedLeverNormalResponse3d
{
    private readonly Fixed64 _normalVelocity;
    private readonly Fixed64 _appliedImpulse;
    private readonly Fixed64 _accumulatedImpulse;
    private readonly byte _projectionFlags;

    internal FixedLeverNormalResponse3d(
        bool isClosing,
        bool hasAppliedImpulse,
        Vector3d firstLinearVelocityDelta,
        Vector3d firstAngularVelocityDelta,
        Vector3d secondLinearVelocityDelta,
        Vector3d secondAngularVelocityDelta,
        bool hasNormalVelocity,
        Fixed64 normalVelocity,
        bool hasAppliedImpulseProjection,
        Fixed64 appliedImpulse,
        bool hasAccumulatedImpulse,
        Fixed64 accumulatedImpulse)
    {
        IsClosing = isClosing;
        HasAppliedImpulse = hasAppliedImpulse;
        FirstLinearVelocityDelta = firstLinearVelocityDelta;
        FirstAngularVelocityDelta = firstAngularVelocityDelta;
        SecondLinearVelocityDelta = secondLinearVelocityDelta;
        SecondAngularVelocityDelta = secondAngularVelocityDelta;
        _normalVelocity = normalVelocity;
        _appliedImpulse = appliedImpulse;
        _accumulatedImpulse = accumulatedImpulse;
        _projectionFlags = (byte)(
            (hasNormalVelocity ? 1 : 0)
            | (hasAppliedImpulseProjection ? 2 : 0)
            | (hasAccumulatedImpulse ? 4 : 0));
    }

    /// <summary>Gets whether the exact relative normal velocity is negative.</summary>
    public bool IsClosing { get; }

    /// <summary>Gets whether the exact applied impulse is nonzero.</summary>
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
    /// Attempts to project the exact relative normal velocity to Q32.32.
    /// </summary>
    public bool TryGetNormalVelocity(out Fixed64 value)
    {
        value = _normalVelocity;
        return (_projectionFlags & 1) != 0;
    }

    /// <summary>
    /// Attempts to project the exact applied impulse to Q32.32.
    /// </summary>
    public bool TryGetAppliedImpulse(out Fixed64 value)
    {
        value = _appliedImpulse;
        return (_projectionFlags & 2) != 0;
    }

    /// <summary>
    /// Attempts to project the completed nonnegative impulse accumulator.
    /// </summary>
    public bool TryGetAccumulatedImpulse(out Fixed64 value)
    {
        value = _accumulatedImpulse;
        return (_projectionFlags & 4) != 0;
    }
}
