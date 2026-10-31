//=======================================================================
// WideConvexPrismRelations.ProjectionDepth.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>ProjectionDepth declaration for WideConvexPrismRelations.</content>
internal static partial class WideConvexPrismRelations
{
    internal readonly struct ProjectionDepth
    {
        internal readonly Signed704 Rational;
        internal readonly Signed192 Common;
        internal readonly Fixed64 Radius;
        internal readonly RadialKind RadialKind;
        internal readonly Signed576 AxisSquared;
        internal readonly Signed320 ShapeAxisSquared;

        internal ProjectionDepth(
            Signed704 rational,
            Signed192 common,
            Fixed64 radius,
            RadialKind radialKind,
            Signed576 axisSquared,
            Signed320 shapeAxisSquared)
        {
            Rational = rational;
            Common = common;
            Radius = radius;
            RadialKind = radialKind;
            AxisSquared = axisSquared;
            ShapeAxisSquared = shapeAxisSquared;
        }
    }
}
