//=======================================================================
// CapsulePlaneRaySelection.CrossFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact finite triangle admission across independently scaled capsule charts.</content>
internal ref partial struct CapsulePlaneRaySelection
{
    /// <summary>Tests this source-frame point against a triangle in the target frame of the same capsule.</summary>
    internal readonly bool ContainsTrianglePoint(in CapsulePlaneRayFrame target, FixedTriangle triangle,
        in CapsulePlaneRayFrame source)
    {
        if (!HasValue) return false;
        Signed320 sourceScale = source.Finite.ShapeFrame.CoordinateDenominator;
        Signed320 targetScale = target.Finite.ShapeFrame.CoordinateDenominator;
        if (sourceScale.Equals(targetScale))
            return target.ContainsTrianglePoint(triangle, X, Y, Z, Denominator, Root);
        // Native coordinates represent 2D times the same capsule-local point.
        // Cross multiplication changes its chart without rounding; positive
        // frame denominators add at most 132 bits (three limbs) per field.
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(X), ContactQuadratic.ActiveWords(Y)),
            Math.Max(ContactQuadratic.ActiveWords(Z), ContactQuadratic.ActiveWords(Denominator))) + 5;
        Span<ulong> values = stackalloc ulong[8 * words];
        Span<int> signs = stackalloc int[8];
        var point = new FiniteSectionPoint(values, signs, words, 0);
        Signed576 scale = Signed576.ExtendValue(targetScale);
        ContactQuadratic.Scale(X, scale, point.Coordinate(0));
        ContactQuadratic.Scale(Y, scale, point.Coordinate(1));
        ContactQuadratic.Scale(Z, scale, point.Coordinate(2));
        ContactQuadratic.Scale(Denominator, Signed576.ExtendValue(sourceScale), point.Denominator);
        return target.ContainsTrianglePoint(triangle, point.Coordinate(0), point.Coordinate(1),
            point.Coordinate(2), point.Denominator, Root);
    }
}
