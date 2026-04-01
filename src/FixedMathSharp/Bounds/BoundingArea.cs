using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a lightweight, axis-aligned bounding area with fixed-point precision, optimized for 2D or simplified 3D use cases.
/// </summary>
/// <remarks>
/// The BoundingArea is designed for performance-critical scenarios where only a minimal bounding volume is required.
/// It offers fast containment and intersection checks with other bounds but lacks the full feature set of BoundingBox.
/// 
/// Use Cases:
/// - Efficient spatial queries in 2D or constrained 3D spaces (e.g., terrain maps or collision grids).
/// - Simplified bounding volume checks where rotation or complex shape fitting is not needed.
/// - Can be used as a broad-phase bounding volume to cull objects before more precise checks with BoundingBox or BoundingSphere.
/// </remarks>

[Serializable]
[MemoryPackable]
public partial struct BoundingArea : IBound, IEquatable<BoundingArea>
{
    #region Fields

    /// <summary>
    /// One of the corner points of the bounding area.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Vector3d Corner1;

    /// <summary>
    /// The opposite corner point of the bounding area.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Vector3d Corner2;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance of the BoundingArea struct with corner coordinates.
    /// </summary>
    public BoundingArea(Fixed64 c1x, Fixed64 c1y, Fixed64 c1z, Fixed64 c2x, Fixed64 c2y, Fixed64 c2z)
    {
        Corner1 = new Vector3d(c1x, c1y, c1z);
        Corner2 = new Vector3d(c2x, c2y, c2z);
    }

    /// <summary>
    /// Initializes a new instance of the BoundingArea struct with two corner points.
    /// </summary>
    [JsonConstructor]
    public BoundingArea(Vector3d corner1, Vector3d corner2)
    {
        Corner1 = corner1;
        Corner2 = corner2;
    }

    #endregion

    #region Properties

    // Min/Max properties for easy access to boundaries

    /// <summary>
    /// The minimum corner of the bounding box.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Min
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(MinX, MinY, MinZ);
    }

    /// <summary>
    /// The maximum corner of the bounding box.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Max
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(MaxX, MaxY, MaxZ);
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MinX
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.x < Corner2.x ? Corner1.x : Corner2.x;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MaxX
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.x > Corner2.x ? Corner1.x : Corner2.x;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MinY
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.y < Corner2.y ? Corner1.y : Corner2.y;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MaxY
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.y > Corner2.y ? Corner1.y : Corner2.y;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MinZ
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.z < Corner2.z ? Corner1.z : Corner2.z;
    }

    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MaxZ
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.z > Corner2.z ? Corner1.z : Corner2.z;
    }

    /// <summary>
    /// Calculates the width (X-axis) of the bounding area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Width
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => MaxX - MinX;
    }

    /// <summary>
    /// Calculates the height (Y-axis) of the bounding area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Height
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => MaxY - MinY;
    }

    /// <summary>
    /// Calculates the depth (Z-axis) of the bounding area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Depth
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => MaxZ - MinZ;
    }

    #endregion

    #region Methods (Instance)

    /// <summary>
    /// Determines if a point is inside the bounding area (including boundaries).
    /// </summary>
    public bool Contains(Vector3d point)
    {
        // Check if the point is within the bounds of the area (including boundaries)
        return point.x >= MinX && point.x <= MaxX
            && point.y >= MinY && point.y <= MaxY
            && point.z >= MinZ && point.z <= MaxZ;
    }

    /// <summary>
    /// Checks if another IBound intersects with this bounding area.
    /// </summary>
    /// <remarks>
    /// It checks for overlap on all axes. If there is no overlap on any axis, they do not intersect.
    /// </remarks>
    public bool Intersects(IBound other)
    {
        return other switch
        {
            BoundingBox or BoundingArea => IntersectsBoxLike(other.Min, other.Max),
            BoundingSphere sphere => IntersectsSphere(sphere),
            _ => false
        };
    }

    /// <summary>
    /// Projects a point onto the bounding box. If the point is outside the box, it returns the closest point on the surface.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ProjectPoint(Vector3d point)
    {
        return this.ProjectPointWithinBounds(point);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IntersectsBoxLike(Vector3d otherMin, Vector3d otherMax)
    {
        Vector3d min = Min;
        Vector3d max = Max;

        if (ContainsPoint(otherMin, min, max) && ContainsPoint(otherMax, min, max))
            return true;

        if (IsFlatAlongZ(min, max, otherMin, otherMax))
            return OverlapsOnXY(min, max, otherMin, otherMax);

        if (IsFlatAlongY(min, max, otherMin, otherMax))
            return OverlapsOnXZ(min, max, otherMin, otherMax);

        if (IsFlatAlongX(min, max, otherMin, otherMax))
            return OverlapsOnYZ(min, max, otherMin, otherMax);

        return OverlapsOnAllAxes(min, max, otherMin, otherMax);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IntersectsSphere(BoundingSphere sphere)
    {
        return Vector3d.SqrDistance(sphere.Center, this.ProjectPointWithinBounds(sphere.Center)) <= sphere.SqrRadius;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ContainsPoint(Vector3d point, Vector3d min, Vector3d max)
    {
        return point.x >= min.x && point.x <= max.x
            && point.y >= min.y && point.y <= max.y
            && point.z >= min.z && point.z <= max.z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsFlatAlongX(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
        => min.x == max.x && otherMin.x == otherMax.x;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsFlatAlongY(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
        => min.y == max.y && otherMin.y == otherMax.y;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsFlatAlongZ(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
        => min.z == max.z && otherMin.z == otherMax.z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OverlapsOnXY(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
    {
        return !(max.x < otherMin.x || min.x > otherMax.x ||
                 max.y < otherMin.y || min.y > otherMax.y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OverlapsOnXZ(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
    {
        return !(max.x < otherMin.x || min.x > otherMax.x ||
                 max.z < otherMin.z || min.z > otherMax.z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OverlapsOnYZ(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
    {
        return !(max.y < otherMin.y || min.y > otherMax.y ||
                 max.z < otherMin.z || min.z > otherMax.z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OverlapsOnAllAxes(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
    {
        return !(max.x < otherMin.x || min.x > otherMax.x ||
                 max.y < otherMin.y || min.y > otherMax.y ||
                 max.z < otherMin.z || min.z > otherMax.z);
    }

    #endregion

    #region Operators

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(BoundingArea left, BoundingArea right) => left.Equals(right);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(BoundingArea left, BoundingArea right) => !left.Equals(right);

    #endregion

    #region Equality and HashCode Overrides

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is BoundingArea other && Equals(other);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(BoundingArea other) => Corner1.Equals(other.Corner1) && Corner2.Equals(other.Corner2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 23 + Corner1.GetHashCode();
            hash = hash * 23 + Corner2.GetHashCode();
            return hash;
        }
    }

    #endregion
}
