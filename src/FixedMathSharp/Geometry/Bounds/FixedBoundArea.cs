//=======================================================================
// FixedBoundArea.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Support;
using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Bounds;

/// <summary>
/// Represents a lightweight, axis-aligned bounding area with fixed-point precision, optimized for 2D or simplified 3D use cases.
/// </summary>
/// <remarks>
/// The FixedBoundArea is designed for performance-critical scenarios where only a minimal bounding volume is required.
/// It offers fast containment and intersection checks with other bounds but lacks the full feature set of FixedBoundBox.
/// 
/// Use Cases:
/// - Efficient spatial queries in 2D or constrained 3D spaces (e.g., terrain maps or collision grids).
/// - Simplified bounding volume checks where rotation or complex shape fitting is not needed.
/// - Can be used as a broad-phase bounding volume to cull objects before more precise checks with FixedBoundBox or FixedBoundSphere.
/// </remarks>

[Serializable]
[MemoryPackable]
public partial struct FixedBoundArea : IEquatable<FixedBoundArea>
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
    /// Initializes a new instance of the FixedBoundArea struct with corner coordinates.
    /// </summary>
    public FixedBoundArea(Fixed64 c1x, Fixed64 c1y, Fixed64 c1z, Fixed64 c2x, Fixed64 c2y, Fixed64 c2z)
    {
        Corner1 = new Vector3d(c1x, c1y, c1z);
        Corner2 = new Vector3d(c2x, c2y, c2z);
    }

    /// <summary>
    /// Initializes a new instance of the FixedBoundArea struct with two corner points.
    /// </summary>
    [JsonConstructor]
    public FixedBoundArea(Vector3d corner1, Vector3d corner2)
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

    /// <summary>
    /// The minimum X coordinate of the bounding area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MinX
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.X < Corner2.X ? Corner1.X : Corner2.X;
    }

    /// <summary>
    /// Gets the greater X-coordinate value of the two corners that define the bounding area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MaxX
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.X > Corner2.X ? Corner1.X : Corner2.X;
    }

    /// <summary>
    /// Gets the minimum Y-coordinate value of the bounding area defined by Corner1 and Corner2.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MinY
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.Y < Corner2.Y ? Corner1.Y : Corner2.Y;
    }

    /// <summary>
    /// Gets the maximum Y-coordinate value of the bounding area defined by the two corners.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MaxY
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.Y > Corner2.Y ? Corner1.Y : Corner2.Y;
    }

    /// <summary>
    /// Gets the minimum Z coordinate value of the bounding volume.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MinZ
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.Z < Corner2.Z ? Corner1.Z : Corner2.Z;
    }

    /// <summary>
    /// Gets the maximum Z coordinate value between the two corners of the bounding box.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MaxZ
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Corner1.Z > Corner2.Z ? Corner1.Z : Corner2.Z;
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

    /// <summary>
    /// The center point of the bounding area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector3d Center
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (Min + Max) * Fixed64.Half;
    }

    #endregion

    #region Methods (Instance)

    /// <summary>
    /// Determines if a point is inside the bounding area (including boundaries).
    /// </summary>
    public bool Contains(Vector3d point)
    {
        // Check if the point is within the bounds of the area (including boundaries)
        return point.X >= MinX && point.X <= MaxX
            && point.Y >= MinY && point.Y <= MaxY
            && point.Z >= MinZ && point.Z <= MaxZ;
    }

    /// <summary>
    /// Tests another bounding area against this bounding area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundArea area) => ContainsBoxLike(area.Min, area.Max);

    /// <summary>
    /// Tests a bounding box against this bounding area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundBox box) => ContainsBoxLike(box.Min, box.Max);

    /// <summary>
    /// Tests a bounding sphere against this bounding area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundSphere sphere)
    {
        if (Contains(sphere.Min) && Contains(sphere.Max))
            return FixedEnclosureType.Contains;

        return Intersects(sphere) ? FixedEnclosureType.Intersects : FixedEnclosureType.Disjoint;
    }

    /// <summary>
    /// Tests a bounding frustum against this bounding area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundFrustum frustum)
    {
        FixedThrowHelper.ThrowIfNull(frustum, nameof(frustum));

        if (Contains(frustum.Min) && Contains(frustum.Max))
            return FixedEnclosureType.Contains;

        return Intersects(frustum) ? FixedEnclosureType.Intersects : FixedEnclosureType.Disjoint;
    }

    /// <summary>
    /// Checks whether another bounding area intersects this bounding area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundArea area) => IntersectsBoxLike(area.Min, area.Max);

    /// <summary>
    /// Checks whether a bounding box intersects this bounding area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundBox box) => IntersectsBoxLike(box.Min, box.Max);

    /// <summary>
    /// Checks whether a bounding sphere intersects this bounding area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundSphere sphere) => IntersectsSphere(sphere);

    /// <summary>
    /// Checks whether a bounding frustum intersects this bounding area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundFrustum frustum)
    {
        FixedThrowHelper.ThrowIfNull(frustum, nameof(frustum));

        return frustum.Intersects(this);
    }

    /// <summary>
    /// Projects a point into the bounding area by clamping it to the area extents.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ProjectPoint(Vector3d point)
    {
        return ClampPoint(point);
    }

    /// <summary>
    /// Clamps a point to this bounding area, returning the point unchanged when it is already inside.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ClampPoint(Vector3d point)
    {
        Vector3d min = Min;
        Vector3d max = Max;

        return new Vector3d(
            FixedMath.Clamp(point.X, min.X, max.X),
            FixedMath.Clamp(point.Y, min.Y, max.Y),
            FixedMath.Clamp(point.Z, min.Z, max.Z));
    }

    /// <summary>
    /// Deconstructs the bounding area into its stored corner points.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector3d corner1, out Vector3d corner2)
    {
        corner1 = Corner1;
        corner2 = Corner2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private FixedEnclosureType ContainsBoxLike(Vector3d otherMin, Vector3d otherMax)
    {
        Vector3d min = Min;
        Vector3d max = Max;

        if (ContainsPoint(otherMin, min, max) && ContainsPoint(otherMax, min, max))
            return FixedEnclosureType.Contains;

        return IntersectsBoxLike(otherMin, otherMax)
            ? FixedEnclosureType.Intersects
            : FixedEnclosureType.Disjoint;
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
    private bool IntersectsSphere(FixedBoundSphere sphere)
    {
        return Vector3d.SqrDistance(sphere.Center, ClampPoint(sphere.Center)) <= sphere.SqrRadius;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ContainsPoint(Vector3d point, Vector3d min, Vector3d max)
    {
        return point.X >= min.X && point.X <= max.X
            && point.Y >= min.Y && point.Y <= max.Y
            && point.Z >= min.Z && point.Z <= max.Z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsFlatAlongX(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
        => min.X == max.X && otherMin.X == otherMax.X;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsFlatAlongY(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
        => min.Y == max.Y && otherMin.Y == otherMax.Y;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsFlatAlongZ(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
        => min.Z == max.Z && otherMin.Z == otherMax.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OverlapsOnXY(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
    {
        return !(max.X < otherMin.X || min.X > otherMax.X ||
                 max.Y < otherMin.Y || min.Y > otherMax.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OverlapsOnXZ(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
    {
        return !(max.X < otherMin.X || min.X > otherMax.X ||
                 max.Z < otherMin.Z || min.Z > otherMax.Z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OverlapsOnYZ(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
    {
        return !(max.Y < otherMin.Y || min.Y > otherMax.Y ||
                 max.Z < otherMin.Z || min.Z > otherMax.Z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OverlapsOnAllAxes(Vector3d min, Vector3d max, Vector3d otherMin, Vector3d otherMax)
    {
        return !(max.X < otherMin.X || min.X > otherMax.X ||
                 max.Y < otherMin.Y || min.Y > otherMax.Y ||
                 max.Z < otherMin.Z || min.Z > otherMax.Z);
    }

    #endregion

    #region Static Ops

    /// <summary>
    /// Creates a new bounding area that encloses both specified bounding areas.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea Union(FixedBoundArea a, FixedBoundArea b)
    {
        return new FixedBoundArea(Vector3d.Min(a.Min, b.Min), Vector3d.Max(a.Max, b.Max));
    }

    #endregion

    #region Operators

    /// <summary>
    /// Determines whether two FixedBoundArea instances are equal.
    /// </summary>
    /// <param name="left">The first FixedBoundArea to compare.</param>
    /// <param name="right">The second FixedBoundArea to compare.</param>
    /// <returns>true if the specified FixedBoundArea instances are equal; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedBoundArea left, FixedBoundArea right) => left.Equals(right);

    /// <summary>
    /// Determines whether two FixedBoundArea instances are not equal.
    /// </summary>
    /// <param name="left">The first FixedBoundArea to compare.</param>
    /// <param name="right">The second FixedBoundArea to compare.</param>
    /// <returns>true if the specified FixedBoundArea instances are not equal; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedBoundArea left, FixedBoundArea right) => !left.Equals(right);

    #endregion

    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is FixedBoundArea other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedBoundArea other) => Corner1.Equals(other.Corner1) && Corner2.Equals(other.Corner2);

    /// <inheritdoc/>
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
