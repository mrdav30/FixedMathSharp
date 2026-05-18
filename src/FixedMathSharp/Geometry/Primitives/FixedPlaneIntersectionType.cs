namespace FixedMathSharp;

/// <summary>
/// Describes how a plane intersects a bounding volume.
/// </summary>
public enum FixedPlaneIntersectionType
{
    /// <summary>
    /// The volume is entirely in the positive half-space of the plane.
    /// </summary>
    Front,

    /// <summary>
    /// The volume is entirely in the negative half-space of the plane.
    /// </summary>
    Back,

    /// <summary>
    /// The volume intersects the plane.
    /// </summary>
    Intersecting
}
