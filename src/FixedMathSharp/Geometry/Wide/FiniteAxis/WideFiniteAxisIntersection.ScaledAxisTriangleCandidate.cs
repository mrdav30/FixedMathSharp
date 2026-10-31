//=======================================================================
// WideFiniteAxisIntersection.ScaledAxisTriangleCandidate.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>ScaledAxisTriangleCandidate declaration for WideFiniteAxisIntersection.</content>
internal static partial class WideFiniteAxisIntersection
{
    internal ref struct ScaledAxisTriangleCandidate
    {
        internal const int StorageWords = 18 * ScaledAxisWords, SignCount = 18;
        internal ContactQuadratic TriangleCoordinate(int axis) => At(axis);
        internal ContactQuadratic AxisCoordinate(int axis) => At(axis + 3);
        internal ContactQuadratic Denominator => At(6);
        private readonly Span<ulong> _values;
        private readonly Span<int> _signs;
        internal bool HasValue;
        internal int EdgeMask, Endpoint;
        internal ScaledAxisTriangleCandidate(Span<ulong> values, Span<int> signs)
        { _values = values; _signs = signs; HasValue = false; EdgeMask = Endpoint = 0; }
        internal ContactQuadratic At(int index) => ContactQuadratic.At(_values, _signs, index, ScaledAxisWords);
        internal void Keep(scoped ScaledAxisTriangleCandidate candidate)
        {
            if (HasValue && ContactQuadratic.CompareRatios(candidate.At(7), candidate.At(8), ReadOnlySpan<ulong>.Empty,
                At(7), At(8), ReadOnlySpan<ulong>.Empty) >= 0) return;
            candidate._values.CopyTo(_values); candidate._signs.CopyTo(_signs);
            HasValue = true; EdgeMask = candidate.EdgeMask; Endpoint = candidate.Endpoint;
        }
        internal void Complete()
        {
            Span<ulong> values = stackalloc ulong[4 * ScaledAxisWords]; Span<int> signs = stackalloc int[4];
            var delta = ContactQuadratic.At(values, signs, 0, ScaledAxisWords);
            var square = ContactQuadratic.At(values, signs, 1, ScaledAxisWords);
            At(7).Clear();
            for (int axis = 0; axis < 3; axis++)
            {
                At(axis + 3).CopyTo(delta); delta.Add(At(axis), -1);
                ContactQuadratic.Multiply(delta, delta, ReadOnlySpan<ulong>.Empty, square); At(7).Add(square);
            }
            ContactQuadratic.Multiply(At(6), At(6), ReadOnlySpan<ulong>.Empty, At(8));
            HasValue = true;
        }
    }
}
