//=======================================================================
// CylinderPairValueDerivative.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Caller-owned storage for one exact directional derivative of the rim value
/// polynomial. The six input slots are rho', tau', (c²)', (c.u)', (c.w)',
/// and (b.c)'; axes and perpendicular basis vectors remain fixed.
/// </summary>
/// <remarks>
/// Each slot uses the value builder's word width and canonical signed-magnitude
/// encoding. Output has nine slots. All storage is disjoint from the value
/// construction and from each other. The default value disables derivatives;
/// a supplied six-slot zero direction is still evaluated and cleared exactly.
/// The derivative preserves the value polynomial's raw factor of three.
/// </remarks>
internal readonly ref struct CylinderPairValueDerivative
{
    internal const int InvariantCount = 6;
    internal const int ScratchCoefficientCount = 72;

    internal readonly ReadOnlySpan<ulong> Invariants;
    internal readonly ReadOnlySpan<sbyte> InvariantSigns;
    internal readonly Span<ulong> Coefficients;
    internal readonly Span<sbyte> Signs;
    internal readonly Span<ulong> Scratch;
    internal readonly Span<sbyte> ScratchSigns;

    internal CylinderPairValueDerivative(ReadOnlySpan<ulong> invariants,
        ReadOnlySpan<sbyte> invariantSigns, Span<ulong> coefficients,
        Span<sbyte> signs, Span<ulong> scratch, Span<sbyte> scratchSigns)
    {
        Invariants = invariants;
        InvariantSigns = invariantSigns;
        Coefficients = coefficients;
        Signs = signs;
        Scratch = scratch;
        ScratchSigns = scratchSigns;
    }
}
