//=======================================================================
// ConeSurfaceFamilyWitness.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

// Slots 0..2 are a cone-local unnormalized normal; 3..5 and 6..8 are
// centered raw p/q numerators over common slot 9; 10..11 retain segment t.
internal readonly ref struct ConeSurfaceFamilyWitness
{
    internal readonly Span<ulong> Values, Root;
    internal readonly Span<int> Signs;
    internal int FieldWords => Values.Length / 24;
    internal ContactQuadratic this[int index] => ContactQuadratic.At(Values, Signs, index, FieldWords);
    internal ConeSurfaceFamilyWitness(Span<ulong> values, Span<int> signs, Span<ulong> root)
    { Values = values; Signs = signs; Root = root; values.Clear(); signs.Clear(); root.Clear(); }
}
