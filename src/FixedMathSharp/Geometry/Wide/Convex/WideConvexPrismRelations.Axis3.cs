//=======================================================================
// WideConvexPrismRelations.Axis3.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Axis3 declaration for WideConvexPrismRelations.</content>
internal static partial class WideConvexPrismRelations
{
    internal readonly struct Axis3
    {
        internal readonly Signed320 X;
        internal readonly Signed320 Y;
        internal readonly Signed320 Z;

        internal Axis3(Signed320 x, Signed320 y, Signed320 z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        internal bool IsZero => X.IsZero && Y.IsZero && Z.IsZero;
    }
}
