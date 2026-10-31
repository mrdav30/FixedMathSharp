//=======================================================================
// FixedBoundBox.BoundingBoxState.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp.Geometry;

/// <content>BoundingBoxState declaration for FixedBoundBox.</content>
public partial struct FixedBoundBox
{
    /// <summary>
    /// Represents the state of a three-dimensional axis-aligned bounding box using its minimum and maximum coordinates.
    /// </summary>
    /// <remarks>
    /// The bounding box is defined by two points: the minimum and maximum corners in 3D space. This
    /// structure is immutable and can be used to describe spatial boundaries for geometric computations, collision
    /// detection, or spatial queries.
    /// </remarks>
    [Serializable]
    [MemoryPackable]
    public readonly partial struct BoundingBoxState
    {
        /// <inheritdoc cref="FixedBoundBox.Min"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector3d Min;

        /// <inheritdoc cref="FixedBoundBox.Max"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector3d Max;

        /// <summary>
        /// Initializes a new instance of the BoundingBoxState class with the specified minimum and maximum coordinates.
        /// </summary>
        [JsonConstructor]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingBoxState(Vector3d min, Vector3d max)
        {
            Min = Vector3d.Min(min, max);
            Max = Vector3d.Max(min, max);
        }
    }
}
