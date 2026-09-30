//=======================================================================
// FixedPointAnchorTerms.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Retains the residual between rounded local feature components and their
/// exact centered-axis construction over the shared 2*Q32.32 denominator.
/// </summary>
internal readonly struct FixedPointAnchorTerm3d : IEquatable<FixedPointAnchorTerm3d>
{
    internal readonly long X;
    internal readonly long Y;
    internal readonly long Z;

    private FixedPointAnchorTerm3d(
        long x,
        long y,
        long z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    internal bool IsZero => X == 0L && Y == 0L && Z == 0L;

    internal static FixedPointAnchorTerm3d CreateCenteredAxisSupport(
        Vector3d localAxis,
        Fixed64 signedAxisLength,
        Vector3d localRadialDirection,
        Fixed64 radius,
        Vector3d roundedAxialOffset,
        Vector3d roundedRadialOffset) =>
        new(
            GetResidual(
                localAxis.X,
                signedAxisLength,
                localRadialDirection.X,
                radius,
                roundedAxialOffset.X,
                roundedRadialOffset.X),
            GetResidual(
                localAxis.Y,
                signedAxisLength,
                localRadialDirection.Y,
                radius,
                roundedAxialOffset.Y,
                roundedRadialOffset.Y),
            GetResidual(
                localAxis.Z,
                signedAxisLength,
                localRadialDirection.Z,
                radius,
                roundedAxialOffset.Z,
                roundedRadialOffset.Z));

    internal static FixedPointAnchorTerm3d CreateRadialSupport(
        Vector3d localRadialDirection,
        Fixed64 radius,
        Vector3d roundedRadialOffset) =>
        CreateCenteredAxisSupport(
            Vector3d.Zero,
            Fixed64.Zero,
            localRadialDirection,
            radius,
            Vector3d.Zero,
            roundedRadialOffset);

    internal static FixedPointAnchorTerm3d LiftPlanarXZ(
        FixedPointAnchorTerm2d planarTerm) =>
        new(planarTerm.X, 0L, planarTerm.Y);

    internal static long GetResidual(
        Fixed64 localAxis,
        Fixed64 signedAxisLength,
        Fixed64 localRadialDirection,
        Fixed64 radius,
        Fixed64 roundedAxialOffset,
        Fixed64 roundedRadialOffset)
    {
        // Retain the rounding residual of the local feature construction.
        // The axial term can include two rounding steps (length/2, then
        // multiplication by the axis), not just a single half-raw error.
        // Only the low word is retained. Unchecked raw arithmetic preserves
        // that word through cancellation even when intermediate products or
        // the rounded-offset sum overflow; Fixed64 operators would not.
        return unchecked(
            localAxis.m_rawValue * signedAxisLength.m_rawValue
            + 2L * localRadialDirection.m_rawValue * radius.m_rawValue
            - Fixed64.Two.m_rawValue
                * (roundedAxialOffset.m_rawValue + roundedRadialOffset.m_rawValue));
    }

    internal static Signed192 Denominator =>
        Signed192.Signed(Fixed64.Two.m_rawValue);

    public bool Equals(FixedPointAnchorTerm3d other) =>
        X == other.X && Y == other.Y && Z == other.Z;

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + X.GetHashCode();
            hash = (hash * 31) + Y.GetHashCode();
            hash = (hash * 31) + Z.GetHashCode();
            return hash;
        }
    }
}

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
