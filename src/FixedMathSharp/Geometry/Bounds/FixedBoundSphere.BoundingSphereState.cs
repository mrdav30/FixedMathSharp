//=======================================================================
// FixedBoundSphere.BoundingSphereState.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp.Geometry;

/// <content>BoundingSphereState declaration for FixedBoundSphere.</content>
public partial struct FixedBoundSphere
{
    /// <summary>
    /// Represents the normalized serializable state of a three-dimensional spherical bound.
    /// </summary>
    [Serializable]
    [MemoryPackable]
    public readonly partial struct BoundingSphereState
    {
        /// <inheritdoc cref="FixedBoundSphere.Center"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector3d Center;

        /// <inheritdoc cref="FixedBoundSphere.Radius"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Fixed64 Radius;

        /// <summary>
        /// Initializes a normalized state from center and radius.
        /// </summary>
        [JsonConstructor]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingSphereState(Vector3d center, Fixed64 radius)
        {
            Center = center;
            Radius = NormalizeRadius(radius);
        }
    }
}
