//=======================================================================
// FixedConvex2dRelations.AnchorProjection.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp.Geometry;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvex2dAnchorProjectionTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void ExactSupportOutsideAClosedFeature_ShouldNotBeAdmittedAtItsRoundedEndpoint(bool reverse, bool worldLimit)
    {
        Vector2d direction = reverse ? -Vector2d.Right : Vector2d.Right;
        Vector2d origin = worldLimit ? new(reverse ? Fixed64.MinValue : Fixed64.MaxValue, Fixed64.MinValue) : Vector2d.Zero;
        FixedPointAnchor2d anchor = WideFiniteAxisIntersection.GetCenteredCapsuleSupportAnchor(
            origin, Fixed64.Zero, Vector2d.Right, Fixed64.FromRaw(3), Fixed64.Half, direction);
        Fixed64 start = Fixed64.Half + Fixed64.FromRaw(2), end = start + Fixed64.FromRaw(1);
        Vector2d a = new(reverse ? -start : start, Fixed64.Zero), b = new(reverse ? -end : end, Fixed64.Zero);
        Assert.False(anchor.ExactLocalTerm.IsZero);
        Assert.False(WideConvex2dRelations.TryProjectAnchorOntoFeature(anchor, origin, Fixed64.Zero,
            a, b, requireInteriorProjection: true, out _));
        Assert.False(WideConvex2dRelations.TryProjectAnchorOntoFeature(anchor, origin, Fixed64.Zero,
            b, a, requireInteriorProjection: true, out _));
    }

    [Theory]
    [InlineData(17)]
    [InlineData(37)]
    [InlineData(53)]
    [InlineData(73)]
    [InlineData(89)]
    public void RotatedSweptAnchor_ShouldProjectItsResidualBeforeFinalRounding(int degrees)
    {
        Fixed64 angle = FixedMath.DegToRad((Fixed64)degrees);
        FixedPointAnchor2d anchor = WideFiniteAxisIntersection.GetCenteredCapsuleSupportAnchor(
            Vector2d.Zero, angle, Vector2d.Right, Fixed64.FromRaw(1), Fixed64.Half, Vector2d.Right);
        Vector2d[] square = { Vector2d.Zero, Vector2d.Right, Vector2d.One, Vector2d.Forward };
        Vector2d point = WideConvex2dRelations.GetSweptAnchorTargetContactOffset(anchor,
            new Vector2d(Fixed64.Quarter, Fixed64.Zero), Vector2d.Zero, Fixed64.Zero, square, Vector2d.Forward);
        // Exact source X = cos(angle)*(radius + 0.5 raw) + translation.
        // BigInteger is a test oracle only; runtime retains fixed-width ratios.
        BigInteger denominator = BigInteger.One << 33;
        BigInteger numerator = (BigInteger)FixedMath.Cos(angle).m_rawValue * ((BigInteger.One << 32) + 1)
            + (BigInteger)Fixed64.Quarter.m_rawValue * denominator;
        BigInteger expected = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        int half = (remainder * 2).CompareTo(denominator);
        if (half > 0 || half == 0 && !expected.IsEven) expected++;
        Assert.Equal((long)expected, point.X.m_rawValue);
        Assert.Equal(Fixed64.One, point.Y);
    }
}
