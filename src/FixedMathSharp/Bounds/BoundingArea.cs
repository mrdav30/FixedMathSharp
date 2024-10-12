using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a simple 3D bounding area defined by two opposite corner points.
    /// Used to define rectangular regions in 3D space.
    /// </summary>
    [Serializable]
    public struct BoundingArea : IEquatable<BoundingArea>
    {
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
        public BoundingArea(Vector3d corner1, Vector3d corner2)
        {
            Corner1 = corner1;
            Corner2 = corner2;
        }

        /// <summary>
        /// One of the corner points of the bounding area.
        /// </summary>
        public Vector3d Corner1;

        /// <summary>
        /// The opposite corner point of the bounding area.
        /// </summary>
        public Vector3d Corner2;

        // Min/Max properties for easy access to boundaries

        public Fixed64 MinX => Corner1.x < Corner2.x ? Corner1.x : Corner2.x;
        public Fixed64 MaxX => Corner1.x > Corner2.x ? Corner1.x : Corner2.x;

        public Fixed64 MinY => Corner1.y < Corner2.y ? Corner1.y : Corner2.y;
        public Fixed64 MaxY => Corner1.y > Corner2.y ? Corner1.y : Corner2.y;

        public Fixed64 MinZ => Corner1.z < Corner2.z ? Corner1.z : Corner2.z;
        public Fixed64 MaxZ => Corner1.z > Corner2.z ? Corner1.z : Corner2.z;

        /// <summary>
        /// Calculates the width (X-axis) of the bounding area.
        /// </summary>
        public Fixed64 Width => MaxX - MinX;

        /// <summary>
        /// Calculates the height (Y-axis) of the bounding area.
        /// </summary>
        public Fixed64 Height => MaxY - MinY;

        /// <summary>
        /// Calculates the depth (Z-axis) of the bounding area.
        /// </summary>
        public Fixed64 Depth => MaxZ - MinZ;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(BoundingArea left, BoundingArea right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(BoundingArea left, BoundingArea right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is BoundingArea other && Equals(other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(BoundingArea other)
        {
            return Corner1.Equals(other.Corner1) && Corner2.Equals(other.Corner2);
        }

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
    }
}