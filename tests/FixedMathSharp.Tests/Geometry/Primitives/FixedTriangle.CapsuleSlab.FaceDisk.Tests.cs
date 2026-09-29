//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleCapsuleSlabFaceDiskTests
{
    [Theory]
    [InlineData(-1L)]
    [InlineData(0L)]
    [InlineData(1L)]
    public void CoreParallelFace_DiskBoundaryPreservesExactFaceContact(long edgeOffset)
    {
        Fixed64 bottom = -Fixed64.FromFraction(3, 2) + Fixed64.FromRaw(edgeOffset);
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Zero, bottom, (Fixed64)(-8)),
            new Vector3d(Fixed64.Zero, bottom, (Fixed64)8), new Vector3d(0, 8, 0));
        // The lower edge clearance is exactly d, one raw inside, or one raw
        // outside the sufficient face-disk certificate. The broad slab still
        // gives the same face minimum in all three cases.
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), Fixed64.Zero,
            Vector2d.Right, Fixed64.Two, Fixed64.One, (Fixed64)4, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(3, 2), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.Zero, first.X);
        Assert.Equal(-Fixed64.FromFraction(3, 2), second.X);
        Assert.Equal(first.Y, second.Y);
        Assert.Equal(first.Z, second.Z);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 0)]
    public void CoreParallelFace_ProjectionOnOrOutsideTriangleRetainsCapEdgeMinimum(int bottom, int depth)
    {
        var triangle = new FixedTriangle(new Vector3d(0, bottom, -8),
            new Vector3d(0, bottom, 8), new Vector3d(0, 8, 0));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), Fixed64.Zero,
            Vector2d.Right, Fixed64.Two, Fixed64.One, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal((Fixed64)depth, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal((Fixed64)bottom, first.Y);
        Assert.Equal(Fixed64.One, second.Y);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Z, second.Z);
    }
}
