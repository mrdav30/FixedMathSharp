//=======================================================================
// WideConvexPrismRelations.CylinderCylinderPenetration.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>CylinderCylinderPenetration declaration for WideConvexPrismRelations.</content>
internal static partial class WideConvexPrismRelations
{
    internal readonly struct CylinderCylinderPenetration
    {
        internal readonly Axis3 Axis;
        internal readonly bool Negate;
        internal readonly CylinderPairDepth Depth;

        internal CylinderCylinderPenetration(
            Axis3 axis,
            bool negate,
            CylinderPairDepth depth)
        {
            Axis = axis;
            Negate = negate;
            Depth = depth;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }
}
