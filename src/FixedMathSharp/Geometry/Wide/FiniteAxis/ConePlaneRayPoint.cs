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
    private readonly int _fieldWords;
    internal int BankWords => _fieldWords;
    internal ContactQuadratic X => ContactQuadratic.At(values, signs, 0, _fieldWords);
    internal ContactQuadratic Y => ContactQuadratic.At(values, signs, 1, _fieldWords);
    internal ContactQuadratic Z => ContactQuadratic.At(values, signs, 2, _fieldWords);
    internal ContactQuadratic Denominator => ContactQuadratic.At(values, signs, 3, _fieldWords);

    internal ConePlaneRayPoint(Span<ulong> values, Span<int> signs, int fieldWords = FieldWords)
    {
        if (values.Length < 8 * fieldWords || signs.Length < SignCount)
            throw new ArgumentException("The plane-ray point storage is too small.");
        this.values = values[..(8 * fieldWords)]; this.signs = signs[..SignCount];
        _fieldWords = fieldWords;
    }

    internal void CopyTo(scoped ConePlaneRayPoint target)
    {
        X.CopyTo(target.X); Y.CopyTo(target.Y); Z.CopyTo(target.Z); Denominator.CopyTo(target.Denominator);
    }

    /// <summary>Changes only the native scale of coordinates in the same physical cone frame.</summary>
    internal void CopyTo(scoped ConePlaneRayPoint target, Signed320 sourceScale, Signed320 targetScale)
    {
        // Coordinates carry 2D native scale. Cross multiplication adds fewer
        // than 132 bits to certified point fields below 3072 bits.
        Signed576 scale = Signed576.ExtendValue(targetScale);
        ContactQuadratic.Scale(X, scale, target.X);
        ContactQuadratic.Scale(Y, scale, target.Y);
        ContactQuadratic.Scale(Z, scale, target.Z);
        ContactQuadratic.Scale(Denominator, Signed576.ExtendValue(sourceScale), target.Denominator);
    }

    internal int CompareTo(scoped ConePlaneRayPoint other, scoped ReadOnlySpan<ulong> root, scoped ReadOnlySpan<ulong> otherRoot)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            int order = ContactQuadratic.CompareRatios(axis == 0 ? X : axis == 1 ? Y : Z, Denominator, root,
                axis == 0 ? other.X : axis == 1 ? other.Y : other.Z, other.Denominator, otherRoot);
            if (order != 0) return order;
        }
        return 0;
    }

    /// <summary>Orders physical cone-frame points across independently scaled authored frames of the same cone.</summary>
    /// <remarks>Both points have positive homogeneous denominators and the frames describe the same physical cone.</remarks>
    internal int CompareTo(scoped ConePlaneRayPoint other, scoped ReadOnlySpan<ulong> root,
        scoped ReadOnlySpan<ulong> otherRoot, in ConePlaneRayFrame frame, in ConePlaneRayFrame otherFrame)
    {
        Signed320 scale = frame.Finite.ShapeFrame.CoordinateDenominator;
        Signed320 otherScale = otherFrame.Finite.ShapeFrame.CoordinateDenominator;
        if (scale.Equals(otherScale)) return CompareTo(other, root, otherRoot);
        // Each native coordinate is 2D times the physical coordinate. Carry
        // D in the homogeneous denominator; at most three extra limbs keep
        // the comparison exact even when Q32.32 witnesses coincide.
        int words = Math.Max(ContactQuadratic.ActiveWords(Denominator), ContactQuadratic.ActiveWords(other.Denominator)) + 5;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic denominator = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic otherDenominator = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic.Scale(Denominator, Signed576.ExtendValue(scale), denominator);
        ContactQuadratic.Scale(other.Denominator, Signed576.ExtendValue(otherScale), otherDenominator);
        for (int axis = 0; axis < 3; axis++)
        {
            int order = ContactQuadratic.CompareRatios(axis == 0 ? X : axis == 1 ? Y : Z, denominator, root,
                axis == 0 ? other.X : axis == 1 ? other.Y : other.Z, otherDenominator, otherRoot);
            if (order != 0) return order;
        }
        return 0;
    }

    internal void Set(WideAxis3 point)
    {
        X.Set(Signed576.ExtendValue(point.X)); Y.Set(Signed576.ExtendValue(point.Y));
        Z.Set(Signed576.ExtendValue(point.Z));
        // Signed*.One is Q32 raw unity; a homogeneous denominator needs
        // integer unity instead of the physical fixed-point scale.
        Denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
    }

    internal void Set(WideDirection3d direction)
    {
        X.Set(direction.X); Y.Set(direction.Y); Z.Set(direction.Z);
        Denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
    }
}
