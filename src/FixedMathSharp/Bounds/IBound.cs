namespace FixedMathSharp;

/// <summary>
/// Defines an axis-aligned bounding volume in three-dimensional space and provides methods for containment,
/// intersection, and projection operations.
/// </summary>
/// <remarks>
/// Implementations of this interface represent a 3D region defined by minimum and maximum coordinates.
/// The interface provides methods to determine whether a point is contained within the bounds, whether two bounds
/// intersect, and to project a point onto the bounds. Typical use cases include spatial queries, collision detection,
/// and geometric computations in 3D environments.
/// </remarks>
public interface IBound
{
    /// <summary>
    /// The minimum bounds of the IBound.
    /// </summary>
    Vector3d Min { get; }

    /// <summary>
    /// The maximum bounds of the IBound.
    /// </summary>
    Vector3d Max { get; }

    /// <summary>
    /// Checks if a point is inside the IBound.
    /// </summary>
    /// <param name="point">The point to check.</param>
    /// <returns>True if the point is inside the IBound, otherwise false.</returns>
    bool Contains(Vector3d point);

    /// <summary>
    /// Checks if the IBound intersects with another IBound.
    /// </summary>
    /// <param name="other">The other IBound to check for intersection.</param>
    /// <returns>True if the IBounds intersect, otherwise false.</returns>
    bool Intersects(IBound other);

    /// <summary>
    /// Projects a point onto the IBound. If the point is outside the IBound, it returns the closest point on the surface.
    /// </summary>
    Vector3d ProjectPoint(Vector3d point);
}