//=======================================================================
// FiniteSlabRayExtrema.NativePrism.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;



/// <content>Feeds exact native line inequalities into the existing slab active sets.</content>
internal static partial class FiniteSlabRayExtrema
{
    /// <remarks>Source coordinates are A+lambda*E, with the existing solver dummy mu fixed at zero.
    /// Selections must be materialized with those same A/E values, not Triangle metadata.</remarks>
    internal static void AccumulateNativeLine(in FiniteSlabRayFrame frame, in WideAxis3 a,
        in WideAxis3 e, ReadOnlySpan<Signed832> rows,
        ref FiniteSlabRaySelection maximum)
    {
        ReadOnlySpan<WideAxis3> source = stackalloc WideAxis3[3] { a, e, default };
        AccumulateDomain(frame, default, ReadOnlySpan<Vector2d>.Empty, 1, true, false,
            ref maximum, null, ReadOnlySpan<RationalSegmentInterval>.Empty, nativeSource: source, nativeRows: rows);
    }
}
