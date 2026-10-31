//=======================================================================
// FixedPointAnchorTerm2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Two-dimensional counterpart of <see cref="FixedPointAnchorTerm3d"/>.
/// </summary>
internal readonly struct FixedPointAnchorTerm2d :
    IEquatable<FixedPointAnchorTerm2d>
{
    internal readonly long X;
    internal readonly long Y;

    private FixedPointAnchorTerm2d(long x, long y)
    {
        X = x;
        Y = y;
    }

    internal bool IsZero => X == 0L && Y == 0L;

    internal static FixedPointAnchorTerm2d CreateCenteredAxisSupport(
        Vector2d localAxis,
        Fixed64 signedAxisLength,
        Vector2d localRadialDirection,
        Fixed64 radius,
        Vector2d roundedAxialOffset,
        Vector2d roundedRadialOffset) =>
        new(
            FixedPointAnchorTerm3d.GetResidual(
                localAxis.X,
                signedAxisLength,
                localRadialDirection.X,
                radius,
                roundedAxialOffset.X,
                roundedRadialOffset.X),
            FixedPointAnchorTerm3d.GetResidual(
                localAxis.Y,
                signedAxisLength,
                localRadialDirection.Y,
                radius,
                roundedAxialOffset.Y,
                roundedRadialOffset.Y));

    internal static FixedPointAnchorTerm2d CreateRadialSupport(
        Vector2d localRadialDirection,
        Fixed64 radius,
        Vector2d roundedRadialOffset) =>
        CreateCenteredAxisSupport(
            Vector2d.Zero,
            Fixed64.Zero,
            localRadialDirection,
            radius,
            Vector2d.Zero,
            roundedRadialOffset);

    public bool Equals(FixedPointAnchorTerm2d other) =>
        X == other.X && Y == other.Y;

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + X.GetHashCode();
            hash = (hash * 31) + Y.GetHashCode();
            return hash;
        }
    }
}
