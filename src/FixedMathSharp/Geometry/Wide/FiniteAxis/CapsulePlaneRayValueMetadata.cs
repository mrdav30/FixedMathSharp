//=======================================================================
// CapsulePlaneRayValueMetadata.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>Copied metadata for a borrowed positive capsule boundary-ray value.</summary>
internal readonly struct CapsulePlaneRayValueMetadata
{
    internal readonly int ValueShift, Ordinal, RootCount, PolynomialCount, DenominatorShift;
    internal readonly bool IsRational, HasCompactCell, HasValue;

    internal CapsulePlaneRayValueMetadata(int valueShift, int ordinal, int rootCount, int polynomialCount,
        int denominatorShift, bool rational, bool compact)
    {
        ValueShift = valueShift; Ordinal = ordinal; RootCount = rootCount; PolynomialCount = polynomialCount;
        DenominatorShift = denominatorShift; IsRational = rational; HasCompactCell = compact; HasValue = true;
    }
}
