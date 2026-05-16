namespace FixedMathSharp;

/// <summary>
/// Describes whether one volume contains, intersects, or is disjoint from another volume.
/// </summary>
public enum ContainmentType
{
    /// <summary>
    /// The volumes do not overlap.
    /// </summary>
    Disjoint,

    /// <summary>
    /// One volume completely contains the tested volume.
    /// </summary>
    Contains,

    /// <summary>
    /// The volumes overlap without complete containment.
    /// </summary>
    Intersects
}
