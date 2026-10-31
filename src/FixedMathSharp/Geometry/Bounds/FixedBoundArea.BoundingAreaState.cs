//=======================================================================
// FixedBoundArea.BoundingAreaState.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp.Geometry;

/// <content>BoundingAreaState declaration for FixedBoundArea.</content>
public partial struct FixedBoundArea
{
    /// <summary>
    /// Represents the normalized serializable state of a two-dimensional axis-aligned bounding area.
    /// </summary>
    [Serializable]
    [MemoryPackable]
    public readonly partial struct BoundingAreaState
    {
        /// <inheritdoc cref="FixedBoundArea.Min"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector2d Min;

        /// <inheritdoc cref="FixedBoundArea.Max"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector2d Max;

        /// <summary>
        /// Initializes a normalized state from minimum and maximum corners.
        /// </summary>
        [JsonConstructor]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingAreaState(Vector2d min, Vector2d max)
        {
            Min = ComponentMin(min, max);
            Max = ComponentMax(min, max);
        }
    }
}
