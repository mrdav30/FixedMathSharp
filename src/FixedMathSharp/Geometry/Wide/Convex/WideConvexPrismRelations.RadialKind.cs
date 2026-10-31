//=======================================================================
// WideConvexPrismRelations.RadialKind.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>RadialKind declaration for WideConvexPrismRelations.</content>
internal static partial class WideConvexPrismRelations
{
    internal enum RadialKind
    {
        None,
        Capsule,
    }
}
