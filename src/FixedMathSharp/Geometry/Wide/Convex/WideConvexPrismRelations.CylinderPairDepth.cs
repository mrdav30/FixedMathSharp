//=======================================================================
// WideConvexPrismRelations.CylinderPairDepth.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>CylinderPairDepth declaration for WideConvexPrismRelations.</content>
internal static partial class WideConvexPrismRelations
{
    /// <summary>
    /// Parallel cap/perpendicular depth R / (C sqrt(A)) + radiusRaw.
    /// Authored axes need not have an exact unit squared length.
    /// </summary>
    internal readonly struct CylinderPairDepth
    {
        internal readonly Signed704 Rational;
        internal readonly Signed192 Common;
        internal readonly Signed576 AxisSquared;
        internal readonly ulong RadiusRaw;

        internal CylinderPairDepth(
            Signed704 rational,
            Signed192 common,
            Signed576 axisSquared,
            ulong radiusRaw)
        {
            Rational = rational;
            Common = common;
            AxisSquared = axisSquared;
            RadiusRaw = radiusRaw;
        }

        internal Signed320 RadialCoefficient => WideArithmetic.MultiplySigned192(
            Common, new Signed192(0UL, 0UL, RadiusRaw));
    }
}
