//=======================================================================
// FixedConvexPrismRoundingContracts.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismRoundingContractsTests
{
    private static readonly Vector2d[] UnitSquare =
    {
        new(-1, -1),
        new(1, -1),
        new(1, 1),
        new(-1, 1),
    };

    private static readonly Vector2d[] RawSkewSquare =
    {
        new(
            -Fixed64.MinIncrement * (Fixed64)3,
            -Fixed64.MinIncrement),
        new(
            Fixed64.MinIncrement,
            -Fixed64.MinIncrement * (Fixed64)3),
        new(
            Fixed64.MinIncrement * (Fixed64)3,
            Fixed64.MinIncrement),
        new(
            -Fixed64.MinIncrement,
            Fixed64.MinIncrement * (Fixed64)3),
    };
    private static readonly Vector2d RawSkewFaceNormal =
        new Vector2d(-Fixed64.One, -Fixed64.Two).Normalized;
    private static readonly Vector2d RawSkewFaceTangent =
        new Vector2d(Fixed64.Two, -Fixed64.One).Normalized;
    private static readonly Vector2d[] RawObliquePrism =
    {
        FromRaw(82_634_498_995L, 0L),
        FromRaw(22_634_498_995L, -90_000_000_000L),
        FromRaw(22_634_498_998L, -90_000_000_002L),
        FromRaw(82_634_498_998L, -2L),
    };

    [Fact]
    public void CapsuleContact_NearMaximumRotatedSupportRetainsShallowDepth()
    {
        Fixed64 prismRotation = Fixed64.PiOver4;
        Vector2d normal = Vector2d.Rotate(
            Vector2d.Right,
            prismRotation);
        Fixed64 separation = Fixed64.MaxValue - Fixed64.One;
        Vector3d center = new(
            normal.X * separation,
            Fixed64.Zero,
            normal.Y * separation);

        Assert.True(FixedConvexPrismRelations
            .TryGetCenteredCapsuleContact(
                center,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.MinIncrement,
                Fixed64.MaxValue,
                Vector3d.Zero,
                prismRotation,
                UnitSquare,
                Fixed64.One,
                out FixedContactAnchors contact));

        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Depth <= (Fixed64)4);
        Assert.False(contact.DepthIsClamped);

        Assert.True(FixedConvexPrismRelations
            .TryGetCenteredCapsuleContact(
                center,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.MinIncrement,
                Fixed64.MaxValue - Fixed64.One,
                Vector3d.Zero,
                prismRotation,
                UnitSquare,
                Fixed64.One,
                out FixedContactAnchors tangent));
        Assert.True(tangent.Depth >= Fixed64.Zero);
        Assert.True(tangent.Depth <= (Fixed64)4);
        Assert.True(contact.Depth > tangent.Depth);
        Assert.False(tangent.DepthIsClamped);
    }

    [Fact]
    public void CapsuleContact_RawSkewPrismRetainsExtremeCancellationDepth()
    {
        Assert.True(TryGetRawSkewContact(
            Fixed64.MaxValue,
            out FixedContactAnchors contact));

        Assert.True(contact.Normal.IsNormalized());
        Assert.Equal(
            new Vector3d(
                RawSkewFaceNormal.X,
                Fixed64.Zero,
                RawSkewFaceNormal.Y),
            contact.Normal);
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Depth <= Fixed64.Two);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void CapsuleContact_RawObliquePrismRoundsLargeSupportExactly()
    {
        // The limiting face has normal (2, 3) and relative support
        // 165,268,997,990, whose exact distance rounds to 45,837,372,807 raw.
        Assert.True(FixedConvexPrismRelations
            .TryGetCenteredCapsuleContact(
                new Vector3d(
                    Fixed64.FromRaw(57_208_499_304L),
                    Fixed64.Zero,
                    Fixed64.FromRaw(-38_138_999_536L)),
                FixedQuaternion.Identity,
                new Vector3d(3, 0, -2).Normalized,
                Fixed64.FromRaw(100_000_000_000L),
                Fixed64.Zero,
                Vector3d.Zero,
                Fixed64.Zero,
                RawObliquePrism,
                Fixed64.FromRaw(200_000_000_000L),
                out FixedContactAnchors contact));

        Assert.Equal(Fixed64.FromRaw(45_837_372_807L), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void CapsuleContact_RawSkewPrismRetainsHalfDomainCancellationDepth()
    {
        Assert.True(TryGetRawSkewContact(
            Fixed64.MaxValue / Fixed64.Two,
            out FixedContactAnchors contact));

        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Depth <= Fixed64.Two);
        Assert.False(contact.DepthIsClamped);
    }

    private static bool TryGetRawSkewContact(
        Fixed64 radius,
        out FixedContactAnchors contact)
    {
        Fixed64 separation = radius - Fixed64.One;
        Vector2d faceCenter = new(
            RawSkewFaceNormal.X * separation,
            RawSkewFaceNormal.Y * separation);
        Span<Vector2d> translatedPrism =
            stackalloc Vector2d[RawSkewSquare.Length];
        for (int index = 0; index < translatedPrism.Length; index++)
            translatedPrism[index] = faceCenter + RawSkewSquare[index];

        return FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                RawSkewFaceTangent.X,
                Fixed64.Zero,
                RawSkewFaceTangent.Y),
            Fixed64.One,
            radius,
            Vector3d.Zero,
            Fixed64.Zero,
            translatedPrism,
            Fixed64.MinIncrement,
            out contact);
    }

    private static Vector2d FromRaw(long x, long y) =>
        new(Fixed64.FromRaw(x), Fixed64.FromRaw(y));
}
