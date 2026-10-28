// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public class CylinderPrismBoundaryTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CutSideWindowRetainsTheQuadraticRimWitness(bool translated)
    {
        Vector3d origin = translated ? new(Fixed64.MaxValue - (Fixed64)4, Fixed64.Zero, Fixed64.Zero) : Vector3d.Zero;
        Vector3d center = origin + new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter);
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, origin, Fixed64.Zero, footprint, Fixed64.Two, false, vertices);
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(13)), Signed320.ExtendValue(Signed192.Signed(32)),
            Signed320.ExtendValue(Signed192.Signed(7)), Signed320.ExtendValue(Signed192.Signed(16)));
        // Full support lies at x=0, outside this retained x=[1/4,3/8] window.
        Assert.False(CurvedPrismContact.TryGetFaceSupportContact(geometry, 2, clip, Fixed64.Zero, Fixed64.Zero,
            center, FixedQuaternion.Identity, origin, false, out _, out _));
        Assert.True(CurvedPrismContact.TryGetCylinderBoundaryContact(geometry, 2, clip, Fixed64.Zero, Fixed64.Zero,
            center, FixedQuaternion.Identity, origin, out FixedContactAnchors contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(origin.X + Fixed64.Quarter, source.X);
        Assert.Equal(Fixed64.Zero, source.Z);
        Assert.Equal(Fixed64.Zero, source.Y);
        Assert.Equal(source.X, exit.X); Assert.Equal(source.Y, exit.Y);
        // At the retained x=1/4 rim, z=1/4-sqrt(3)/4. This exercises the
        // defining quadratic root and its affine lambda/mu witness replay.
        Fixed64 expectedDepth = FixedMath.Sqrt((Fixed64)3) / (Fixed64)4 - Fixed64.Quarter;
        Assert.True(FixedMath.Abs(contact.Depth - expectedDepth) <= Fixed64.FromRaw(2));
        Assert.True(FixedMath.Abs(exit.Z + expectedDepth) <= Fixed64.FromRaw(2));
        Assert.Equal(Vector3d.Backward, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void TiltedCylinderReplaysTheAlgebraicHeightAtTheCapRim()
    {
        Vector3d center = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Two).Normalized;
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, false, vertices);
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(7)), Signed320.ExtendValue(Signed192.Signed(16)),
            Signed320.ExtendValue(Signed192.Signed(7)), Signed320.ExtendValue(Signed192.Signed(16)));
        Assert.True(CurvedPrismContact.TryGetCylinderBoundaryContact(geometry, 2, clip, -Fixed64.Half, Fixed64.Half,
            center, rotation, Vector3d.Zero, out FixedContactAnchors contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d exit));
        Fixed64 rim = FixedMath.Sqrt((Fixed64)3) / (Fixed64)4;
        // The ideal authored 1:2 quaternion has cos=3/5, sin=4/5.
        // Its Q32.32 normalization perturbs those ratios by less than 8 raw.
        Fixed64 expectedY = -Fixed64.FromFraction(3, 5) + Fixed64.FromFraction(4, 5) * rim;
        Fixed64 expectedDepth = Fixed64.FromFraction(4, 5) + Fixed64.FromFraction(3, 5) * rim - Fixed64.Quarter;
        Assert.Equal(Fixed64.Quarter, source.X); Assert.Equal(Fixed64.Zero, source.Z);
        Assert.True(FixedMath.Abs(source.Y - expectedY) <= Fixed64.FromRaw(8));
        Assert.Equal(source.X, exit.X); Assert.Equal(source.Y, exit.Y);
        Assert.True(FixedMath.Abs(contact.Depth - expectedDepth) <= Fixed64.FromRaw(8));
        Assert.True(FixedMath.Abs(exit.Z + expectedDepth) <= Fixed64.FromRaw(8));
        Assert.Equal(Vector3d.Backward, contact.Normal);
    }

    [Fact]
    public void WideRationalLineKeepsTheExactSectionWhileReplayingTheNativeQuadraticMaximum()
    {
        Vector3d center = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Two + Fixed64.FromRaw(1)).Normalized;
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, false, vertices);
        // lambda=7/16+2^-260 is irreducible. Even after converting scalar
        // units to raw coordinates, this cut and the authored
        // quaternion it needs a >192-bit native coordinate denominator,
        // while the one-dimensional active sets still use 64-word banks.
        Signed320 n = new(7, 0, 0, 0, 1), d = new(16, 0, 0, 0, 0);
        var cut = new RationalSegmentInterval(n, d, n, d);
        CurvedPrismContact.GetNativeBoundarySegment(geometry, 2, cut, -Fixed64.Half, Fixed64.Half,
            out _, out _, out Signed320 coordinateD, out _);
        Span<ulong> magnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(Signed576.ExtendValue(coordinateD), magnitude);
        Assert.True(WideArithmetic.GetMagnitudeBitLength(magnitude) > 192);
        Assert.True(CurvedPrismContact.TryGetCylinderBoundaryContact(geometry, 2, cut, -Fixed64.Half, Fixed64.Half,
            center, rotation, Vector3d.Zero, out FixedContactAnchors contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d exit));
        Fixed64 rim = FixedMath.Sqrt((Fixed64)3) / (Fixed64)4;
        Fixed64 expectedY = -Fixed64.FromFraction(3, 5) + Fixed64.FromFraction(4, 5) * rim;
        Fixed64 expectedDepth = Fixed64.FromFraction(4, 5) + Fixed64.FromFraction(3, 5) * rim - Fixed64.Quarter;
        // The exact x=1/4-2^-258 lies strictly on the selected line and
        // rounds once to 1/4. Both final anchors retain that same slice.
        Assert.Equal(Fixed64.Quarter, source.X); Assert.Equal(Fixed64.Zero, source.Z);
        Assert.Equal(source.X, exit.X); Assert.Equal(source.Y, exit.Y);
        Assert.True(FixedMath.Abs(source.Y - expectedY) <= Fixed64.FromRaw(12));
        Assert.True(FixedMath.Abs(contact.Depth - expectedDepth) <= Fixed64.FromRaw(12));
        Assert.True(FixedMath.Abs(exit.Z + expectedDepth) <= Fixed64.FromRaw(12));
        Assert.Equal(Vector3d.Backward, contact.Normal); Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CapWindowUsesItsOriginalFootprintInequalities(int sign)
    {
        Vector3d center = new(Fixed64.Zero, (Fixed64)sign * Fixed64.Quarter, Fixed64.Zero);
        Vector3d origin = new(Fixed64.Zero, (Fixed64)(-2 * sign), Fixed64.Zero);
        Vector2d[] footprint = { new(Fixed64.Quarter, -Fixed64.Quarter), new(Fixed64.FromFraction(3, 8), -Fixed64.Quarter),
            new(Fixed64.FromFraction(3, 8), Fixed64.Quarter), new(Fixed64.Quarter, Fixed64.Quarter) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, origin, Fixed64.Zero, footprint, Fixed64.Two, false, vertices);
        Assert.True(CurvedPrismContact.TryGetCylinderBoundaryContact(geometry, sign > 0 ? -2 : -1, default,
            -Fixed64.Two, Fixed64.Two, center, FixedQuaternion.Identity, origin, out FixedContactAnchors contact, boundaryEdge: 3));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d exit));
        Assert.True(source.X >= Fixed64.Quarter && source.X <= Fixed64.FromFraction(3, 8));
        Assert.True(source.Z >= -Fixed64.Quarter && source.Z <= Fixed64.Quarter);
        Assert.Equal(Fixed64.Zero, source.Y);
        Assert.Equal(source.X, exit.X); Assert.Equal(source.Z, exit.Z);
        Assert.Equal((Fixed64)(-sign) * Fixed64.FromFraction(3, 4), exit.Y);
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.Equal((Fixed64)(-sign) * Vector3d.Up, contact.Normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapBoundaryLineSelectsTheFinitePoleFamilyInsteadOfItsUnclippedCenter(bool cone)
    {
        Vector3d center = new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero);
        Vector3d origin = new(Fixed64.Zero, -Fixed64.One, Fixed64.Zero);
        Vector2d[] footprint = { new(-2, -2), new(0, -2), new(0, 2), new(-2, 2) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, origin, Fixed64.Zero, footprint, Fixed64.Half, cone, vertices);
        FixedContactAnchors contact;
        bool found = cone ? new ConePrismBoundaryContacts().TryGetContact(geometry, -2, default, Fixed64.Half, Fixed64.Half,
            center, FixedQuaternion.Identity, origin, out contact, boundaryEdge: 1)
            : CurvedPrismContact.TryGetCylinderBoundaryContact(geometry, -2, default, Fixed64.Half, Fixed64.Half,
                center, FixedQuaternion.Identity, origin, out contact, boundaryEdge: 1);
        Assert.True(found); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(-Fixed64.Half, source.Y);
        Assert.InRange(source.Z.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
        Assert.Equal(Fixed64.Half, contact.Depth); Assert.Equal(Vector3d.Down, contact.Normal);
    }

    [Fact]
    public void FiniteWindowOutsideInitialCylinderDoesNotBecomeAContactWhileTranslating()
    {
        Vector3d center = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter);
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, false, vertices);
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(4)),
            Signed320.ExtendValue(Signed192.Signed(5)), Signed320.ExtendValue(Signed192.Signed(16)));
        Assert.False(CurvedPrismContact.TryGetCylinderBoundaryContact(geometry, 2, clip, Fixed64.Zero, Fixed64.Zero,
            center, FixedQuaternion.Identity, Vector3d.Zero, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }
}

