namespace FixedMathSharp.Geometry;

/// <summary>
/// Describes one participant in an exact point-anchor normal response.
/// </summary>
public readonly struct FixedLeverResponseOperand3d
{
    /// <summary>
    /// Creates a response operand.
    /// </summary>
    public FixedLeverResponseOperand3d(
        in FixedLever lever,
        Vector3d linearVelocity,
        Vector3d angularVelocity,
        Vector3d linearImpulseAxis,
        Fixed64 inverseMass,
        Fixed3x3 inverseInertia)
    {
        Lever = lever;
        LinearVelocity = linearVelocity;
        AngularVelocity = angularVelocity;
        LinearImpulseAxis = linearImpulseAxis;
        InverseMass = inverseMass;
        InverseInertia = inverseInertia;
    }

    /// <summary>Gets the exact point lever.</summary>
    public FixedLever Lever { get; }

    /// <summary>Gets the point owner's linear velocity.</summary>
    public Vector3d LinearVelocity { get; }

    /// <summary>Gets the point owner's angular velocity.</summary>
    public Vector3d AngularVelocity { get; }

    /// <summary>
    /// Gets the signed, mobility-projected response normal for this
    /// participant.
    /// </summary>
    /// <remarks>
    /// The first participant uses the negative response normal and the second
    /// uses the positive response normal.
    /// </remarks>
    public Vector3d LinearImpulseAxis { get; }

    /// <summary>Gets the effective inverse mass.</summary>
    public Fixed64 InverseMass { get; }

    /// <summary>Gets the constrained inverse inertia tensor.</summary>
    public Fixed3x3 InverseInertia { get; }
}
