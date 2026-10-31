//=======================================================================
// FiniteSlabRaySelectionMetadata.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

internal readonly struct FiniteSlabRaySelectionMetadata
{
    internal readonly bool HasValue, ParameterIsDepth, HasCompactCell, IsRational;
    internal readonly int Ordinal, RootCount, CoefficientCount, Orientation, DenominatorShift;
    /// <summary>
    /// Bit i marks an active authored domain edge i that generated this event;
    /// clipped segment queries additionally use bits 3/4 for their rational cuts.
    /// Zero marks a shape-intrinsic event, which remains intrinsic when its
    /// source point happens to coincide with a covered mesh seam.
    /// </summary>
    internal readonly int DomainEdgeMask;
    internal readonly FixedTriangle Triangle;
    internal FiniteSlabRaySelectionMetadata(bool hasValue, bool parameterIsDepth, int ordinal,
        int rootCount, int coefficientCount, int orientation, FixedTriangle triangle, int domainEdgeMask,
        bool hasCompactCell, bool isRational, int denominatorShift)
    {
        HasValue = hasValue; ParameterIsDepth = parameterIsDepth; Ordinal = ordinal; RootCount = rootCount;
        CoefficientCount = coefficientCount; Orientation = orientation; Triangle = triangle;
        DomainEdgeMask = domainEdgeMask;
        HasCompactCell = hasCompactCell; IsRational = isRational; DenominatorShift = denominatorShift;
    }
}
