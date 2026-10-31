//=======================================================================
// FixedBoundCircle.BoundingCircleState.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp.Geometry;

/// <content>BoundingCircleState declaration for FixedBoundCircle.</content>
public partial struct FixedBoundCircle
{
    /// <summary>
    /// Represents the normalized serializable state of a two-dimensional circular bound.
    /// </summary>
    [Serializable]
    [MemoryPackable]
    public readonly partial struct BoundingCircleState
    {
        /// <inheritdoc cref="FixedBoundCircle.Center"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector2d Center;

        /// <inheritdoc cref="FixedBoundCircle.Radius"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Fixed64 Radius;

        /// <summary>
        /// Initializes a normalized state from center and radius.
        /// </summary>
        [JsonConstructor]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingCircleState(Vector2d center, Fixed64 radius)
        {
            Center = center;
            Radius = NormalizeRadius(radius);
        }
    }
}
