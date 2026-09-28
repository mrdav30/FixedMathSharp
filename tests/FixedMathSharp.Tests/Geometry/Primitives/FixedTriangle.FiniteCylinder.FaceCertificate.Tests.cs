//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FixedTriangleFiniteCylinderTests
{
    [Theory]
    [InlineData(0, -1)]
    [InlineData(0, 1)]
    [InlineData(2, -1)]
    [InlineData(2, 1)]
    public void Cylinder_LocalRadialAxisMinimum_RoundsRationalDepthForEitherSign(int axis, int sign)
    {
        Fixed64 coordinate = sign * Fixed64.FromFraction(3, 4);
        var triangle = new FixedTriangle(Point(-2, -2), Point(2, -2), Point(0, 2));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(axis == 0 ? new Vector3d(-sign, 0, 0) : new Vector3d(0, 0, -sign), contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(coordinate, axis == 0 ? first.X : first.Z);
        Assert.Equal((Fixed64)sign, axis == 0 ? second.X : second.Z);
        Assert.Equal(Fixed64.Zero, axis == 0 ? first.Z : first.X);
        Assert.Equal(Fixed64.Zero, axis == 0 ? second.Z : second.X);
        Assert.Equal(first.Y, second.Y);

        Vector3d Point(int y, int transverse) => axis == 0
            ? new Vector3d(coordinate, (Fixed64)y, (Fixed64)transverse)
            : new Vector3d((Fixed64)transverse, (Fixed64)y, coordinate);
    }

    [Theory]
    [InlineData(0L)]
    [InlineData(1L)]
    public void CircleSlab_AxialDepthAtScalarLimit_ClampsOnlyExactOverflow(long rawTilt)
    {
        Fixed64 tilt = Fixed64.FromRaw(rawTilt);
        var triangle = new FixedTriangle(new Vector3d((Fixed64)(-2), -tilt, (Fixed64)(-2)),
            new Vector3d((Fixed64)2, -tilt, (Fixed64)(-2)), new Vector3d(Fixed64.Zero, tilt, (Fixed64)2));
        // The shallowest cap gap is exactly Max+tilt. The XZ projection
        // contains a radius1/2 disk; its nonzero radial margin and the
        // cylinder's huge radius keep the other support gaps larger.
        Assert.True(triangle.TryGetCircleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Fixed64.MaxValue, Fixed64.MaxValue, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.Equal(rawTilt != 0, contact.DepthIsClamped);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(tilt, first.Y);
        Assert.Equal(-Fixed64.MaxValue, second.Y);
    }

    [Theory]
    [InlineData(3, false)]
    [InlineData(-3, false)]
    [InlineData(3, true)]
    [InlineData(4, false)]
    [InlineData(0, false)]
    public void Cylinder_HorizontalFaceWithContainedProjection_RetainsExactCapMinimum(int quarterHeight, bool boundary)
    {
        Fixed64 y = Fixed64.FromFraction(quarterHeight, 4);
        var triangle = boundary
            ? new FixedTriangle(new Vector3d(Fixed64.Zero, y, Fixed64.Zero),
                new Vector3d((Fixed64)2, y, Fixed64.Zero), new Vector3d(Fixed64.Zero, y, (Fixed64)2))
            : new FixedTriangle(new Vector3d((Fixed64)(-2), y, (Fixed64)(-2)),
                new Vector3d((Fixed64)2, y, (Fixed64)(-2)), new Vector3d(Fixed64.Zero, y, (Fixed64)2));

        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            out FixedContactAnchors contact, out bool capFace));
        Assert.True(capFace);
        Assert.Equal(Fixed64.One - FixedMath.Abs(y), contact.Depth);
        Assert.Equal(quarterHeight > 0 ? Vector3d.Down : Vector3d.Up, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.Zero, y, Fixed64.Zero), first);
        Assert.Equal(new Vector3d(Fixed64.Zero, quarterHeight > 0 ? Fixed64.One : -Fixed64.One, Fixed64.Zero), second);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Cylinder_VerticalFaceWithContainedProjection_RetainsExactSideMinimum(bool boundary)
    {
        Fixed64 x = Fixed64.FromFraction(3, 4);
        var triangle = boundary
            ? new FixedTriangle(new Vector3d(x, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(x, (Fixed64)2, Fixed64.Zero), new Vector3d(x, Fixed64.Zero, (Fixed64)2))
            : new FixedTriangle(new Vector3d(x, (Fixed64)(-2), (Fixed64)(-2)),
                new Vector3d(x, (Fixed64)2, (Fixed64)(-2)), new Vector3d(x, Fixed64.Zero, (Fixed64)2));

        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            out FixedContactAnchors contact, out bool capFace));
        Assert.False(capFace);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(x, first.X);
        Assert.Equal(Fixed64.One, second.X);
        Assert.Equal(Fixed64.Zero, first.Z);
        Assert.Equal(first.Y, second.Y);
        Assert.Equal(Fixed64.Zero, second.Z);
        Assert.InRange(first.Y.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Cylinder_HorizontalFaceWithOutsideProjection_StillFindsRadialTouch(int order)
    {
        Fixed64 y = Fixed64.Half;
        var triangle = new FixedTriangle(new Vector3d(Fixed64.One, y, -Fixed64.One),
            new Vector3d((Fixed64)3, y, -Fixed64.One), new Vector3d(Fixed64.One, y, Fixed64.One));
        for (int index = 0; index < order; index++)
            triangle = new FixedTriangle(triangle.B, triangle.C, triangle.A);
        // The axis projection is outside, although its infinite-plane cap
        // gap is 1/2. The actual triangle only touches the radial side at X=1.
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.One, y, Fixed64.Zero), first);
        Assert.Equal(first, second);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Cylinder_FaceProjectionWithInsufficientBallMargin_RetainsSmallerBoundaryMinimum(bool vertical)
    {
        var triangle = vertical
            ? new FixedTriangle(Vector3d.Zero, new Vector3d(0, 4, 0), new Vector3d(0, 0, 4))
            : new FixedTriangle(Vector3d.Zero, new Vector3d(4, 0, 0), new Vector3d(0, 0, 4));
        Fixed64 height = vertical ? Fixed64.Two : (Fixed64)4;
        Fixed64 radius = vertical ? Fixed64.Two : Fixed64.One;
        // Horizontal: cap depth2 exceeds radial depth1. Vertical: side
        // depth2 exceeds axial depth1. Merely containing the projection
        // cannot certify the triangle face as the global minimum.
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, height, radius, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.Equal(vertical ? Vector3d.Down : Vector3d.Backward, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Zero, first);
        Assert.Equal(vertical ? Vector3d.Up : Vector3d.Forward, second);
    }
}
