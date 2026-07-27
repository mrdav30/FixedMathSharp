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
internal readonly struct FixedPointAnchorTerm3d :
    IEquatable<FixedPointAnchorTerm3d>
{
    internal const long MaximumResidualMagnitude = 1L << 33;

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

    private static long GetResidual(
        Fixed64 localAxis,
        Fixed64 signedAxisLength,
        Fixed64 localRadialDirection,
        Fixed64 radius,
        Fixed64 roundedAxialOffset,
        Fixed64 roundedRadialOffset)
    {
        Signed320 exact = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(localAxis),
                Signed192.Raw(signedAxisLength)),
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localRadialDirection),
                    Signed192.Raw(radius)),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localRadialDirection),
                    Signed192.Raw(radius))));
        Signed192 rounded = WideArithmetic.AddSigned192(
            Signed192.Raw(roundedAxialOffset),
            Signed192.Raw(roundedRadialOffset));
        Signed320 residual = WideArithmetic.SubtractSigned320(
            exact,
            WideArithmetic.MultiplySigned192(
                rounded,
                Denominator));
        // Each rounded term is within half of its source denominator. The
        // axial full-length/2 residual is therefore at most one Q32 scale
        // unit, as is the doubled radial product residual. Their sum is
        // bounded inclusively by 2*Q32 (2^33), so the exact numerator fits in
        // one signed word even at a pair of round-to-even ties.
        return unchecked((long)residual.Word0);
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

    public override bool Equals(object? obj) =>
        obj is FixedPointAnchorTerm3d other && Equals(other);
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

    private static long GetResidual(
        Fixed64 localAxis,
        Fixed64 signedAxisLength,
        Fixed64 localRadialDirection,
        Fixed64 radius,
        Fixed64 roundedAxialOffset,
        Fixed64 roundedRadialOffset)
    {
        Signed320 exact = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(localAxis),
                Signed192.Raw(signedAxisLength)),
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localRadialDirection),
                    Signed192.Raw(radius)),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localRadialDirection),
                    Signed192.Raw(radius))));
        Signed192 rounded = WideArithmetic.AddSigned192(
            Signed192.Raw(roundedAxialOffset),
            Signed192.Raw(roundedRadialOffset));
        Signed320 residual = WideArithmetic.SubtractSigned320(
            exact,
            WideArithmetic.MultiplySigned192(
                rounded,
                FixedPointAnchorTerm3d.Denominator));
        return unchecked((long)residual.Word0);
    }

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

    public override bool Equals(object? obj) =>
        obj is FixedPointAnchorTerm2d other && Equals(other);
}
