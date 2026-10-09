//=======================================================================
// ConePlaneRayPoint.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Borrowed homogeneous coordinates in one exact quadratic field.</summary>
internal readonly ref struct ConePlaneRayPoint
{
    internal const int FieldWords = ConePlaneRaySelection.FieldWords;
    internal const int StorageWords = 8 * FieldWords;
    internal const int SignCount = 8;
    private readonly Span<ulong> values;
    private readonly Span<int> signs;
    internal ContactQuadratic X => ContactQuadratic.At(values, signs, 0, FieldWords);
    internal ContactQuadratic Y => ContactQuadratic.At(values, signs, 1, FieldWords);
    internal ContactQuadratic Z => ContactQuadratic.At(values, signs, 2, FieldWords);
    internal ContactQuadratic Denominator => ContactQuadratic.At(values, signs, 3, FieldWords);

    internal ConePlaneRayPoint(Span<ulong> values, Span<int> signs)
    {
        if (values.Length < StorageWords || signs.Length < SignCount)
            throw new ArgumentException("The plane-ray point storage is too small.");
        this.values = values[..StorageWords]; this.signs = signs[..SignCount];
    }

    internal void Set(WideAxis3 point)
    {
        X.Set(Signed576.ExtendValue(point.X)); Y.Set(Signed576.ExtendValue(point.Y));
        Z.Set(Signed576.ExtendValue(point.Z));
        // Signed*.One is Q32 raw unity; a homogeneous denominator needs
        // integer unity instead of the physical fixed-point scale.
        Denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
    }
}
