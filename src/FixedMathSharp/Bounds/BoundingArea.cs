using MessagePack;
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
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
    [MessagePackObject]
    public struct BoundingArea : IBound, IEquatable<BoundingArea>
    {
        #region Fields

        /// <summary>
        /// One of the corner points of the bounding area.
        /// </summary>
        [Key(0)]
        public Vector3d Corner1;

        /// <summary>
        /// The opposite corner point of the bounding area.
        /// </summary>
        [Key(1)]
        public Vector3d Corner2;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the BoundingArea struct with corner coordinates.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingArea(Fixed64 c1x, Fixed64 c1y, Fixed64 c1z, Fixed64 c2x, Fixed64 c2y, Fixed64 c2z)
        {
            Corner1 = new Vector3d(c1x, c1y, c1z);
            Corner2 = new Vector3d(c2x, c2y, c2z);
        }

        /// <summary>
        /// Initializes a new instance of the BoundingArea struct with two corner points.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingArea(Vector3d corner1, Vector3d corner2)
        {
            Corner1 = corner1;
            Corner2 = corner2;
        }

        #endregion

        #region Properties and Methods (Instance)

        // Min/Max properties for easy access to boundaries

        /// <summary>
        /// The minimum corner of the bounding box.
        /// </summary>
        [IgnoreMember]
        public Vector3d Min
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => new(MinX, MinY, MinZ);
        }

        /// <summary>
        /// The maximum corner of the bounding box.
        /// </summary>
        [IgnoreMember]
        public Vector3d Max
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => new(MaxX, MaxY, MaxZ);
        }

        [IgnoreMember]
        public Fixed64 MinX
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Corner1.x < Corner2.x ? Corner1.x : Corner2.x;
        }

        [IgnoreMember]
        public Fixed64 MaxX
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Corner1.x > Corner2.x ? Corner1.x : Corner2.x;
        }

        [IgnoreMember]
        public Fixed64 MinY
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Corner1.y < Corner2.y ? Corner1.y : Corner2.y;
        }

        [IgnoreMember]
        public Fixed64 MaxY
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Corner1.y > Corner2.y ? Corner1.y : Corner2.y;
        }

        [IgnoreMember]
        public Fixed64 MinZ
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Corner1.z < Corner2.z ? Corner1.z : Corner2.z;
        }

        [IgnoreMember]
        public Fixed64 MaxZ
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Corner1.z > Corner2.z ? Corner1.z : Corner2.z;
        }

        /// <summary>
        /// Calculates the width (X-axis) of the bounding area.
        /// </summary>
        [IgnoreMember]
        public Fixed64 Width
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MaxX - MinX;
        }

        /// <summary>
        /// Calculates the height (Y-axis) of the bounding area.
        /// </summary>
        [IgnoreMember]
        public Fixed64 Height
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MaxY - MinY;
        }

        /// <summary>
        /// Calculates the depth (Z-axis) of the bounding area.
        /// </summary>
        [IgnoreMember]
        public Fixed64 Depth
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MaxZ - MinZ;
        }

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
            switch (other)
            {
                case BoundingBox or BoundingArea:
                    {
                        if (Contains(other.Min) && Contains(other.Max))
                            return true;  // Full containment

                        // Determine which axis is "flat" (thickness zero)
                        bool flatX = Min.x == Max.x && other.Min.x == other.Max.x;
                        bool flatY = Min.y == Max.y && other.Min.y == other.Max.y;
                        bool flatZ = Min.z == Max.z && other.Min.z == other.Max.z;

                        if (flatZ) // Rectangle in XY
                            return !(Max.x < other.Min.x || Min.x > other.Max.x ||
                                     Max.y < other.Min.y || Min.y > other.Max.y);
                        else if (flatY) // Rectangle in XZ
                            return !(Max.x < other.Min.x || Min.x > other.Max.x ||
                                     Max.z < other.Min.z || Min.z > other.Max.z);
                        else if (flatX) // Rectangle in YZ
                            return !(Max.y < other.Min.y || Min.y > other.Max.y ||
                                     Max.z < other.Min.z || Min.z > other.Max.z);
                        else // fallback to 3D volume logic
                            return !(Max.x < other.Min.x || Min.x > other.Max.x ||
                                     Max.y < other.Min.y || Min.y > other.Max.y ||
                                     Max.z < other.Min.z || Min.z > other.Max.z);
                    }
                case BoundingSphere sphere:
                    // Find the closest point on the area to the sphere's center
                    // Intersection occurs if the distance from the closest point to the sphere’s center is within the radius.
                    return Vector3d.SqrDistance(sphere.Center, this.ProjectPointWithinBounds(sphere.Center)) <= sphere.SqrRadius;

                default: return false; // Default case for unknown or unsupported types
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
}