//=======================================================================
// CenteredFiniteSurfaceHardening.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredFiniteSurfaceHardeningTests
{
    [Fact]
    public void CylinderSurfaceAnchor_PreservesFullDomainRadialCancellation()
    {
        Assert.True(FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(-1L), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.MaxValue,
            Vector3d.Right,
            out FixedPointAnchor anchor,
            out Vector3d normal,
            out Fixed64 distance));
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(long.MaxValue - 1L, point.X.m_rawValue);
        Assert.Equal(Vector3d.Right, normal);
        Assert.Equal(Fixed64.FromRaw(1L), distance);
    }

    [Fact]
    public void ConeSphereOverlap_PreservesFullDomainBaseRimTangency()
    {
        Vector3d center = new(
            Fixed64.FromRaw(-1L),
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d sphereCenter = new(
            Fixed64.MaxValue,
            -Fixed64.One,
            Fixed64.Zero);
        Assert.True(FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
            sphereCenter,
            center,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.MaxValue,
            Vector3d.Right,
            out FixedPointAnchor anchor,
            out _,
            out Fixed64 distance));
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(
            Fixed64.MaxValue.m_rawValue,
            anchor.LocalDisplacement.X.m_rawValue);
        Assert.Equal(
            (-Fixed64.One).m_rawValue,
            anchor.LocalPoint.Y.m_rawValue);
        Assert.Equal(long.MaxValue - 1L, point.X.m_rawValue);
        Assert.Equal((-Fixed64.One).m_rawValue, point.Y.m_rawValue);
        Assert.Equal(Fixed64.FromRaw(1L), distance);

        Assert.True(FixedSegment.DoesCenteredFiniteConeOverlapSphere(
            center,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.MaxValue,
            sphereCenter,
            Fixed64.FromRaw(1L)));
    }

    [Fact]
    public void ConeSurfaceAnchor_PreservesInteriorSideSelectionNearTheBase()
    {
        var point = new Vector3d(
            Fixed64.Zero,
            Fixed64.FromRaw(-1L),
            Fixed64.FromRaw(2L));

        Assert.True(FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
            point,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.FromRaw(3L),
            Fixed64.FromRaw(1L),
            Vector3d.Forward,
            out FixedPointAnchor anchor,
            out Vector3d normal,
            out Fixed64 distance));
        Assert.True(anchor.TryGetPoint(out Vector3d surface));
        Assert.Equal(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromRaw(-2L),
                Fixed64.FromRaw(1L)),
            surface);
        Assert.True(normal.IsNormalized());
        Assert.Equal(Fixed64.FromRaw(1L), distance);
    }

    [Fact]
    public void TriangleSupportContact_RejectsZeroAreaFaces()
    {
        FixedTriangle[] triangles =
        {
            new(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero),
            new(Vector3d.Zero, Vector3d.Right, Vector3d.Right * Fixed64.Two),
        };

        foreach (FixedTriangle triangle in triangles)
        {
            Assert.False(triangle.TryGetCenteredFiniteCylinderSupportContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(10, 1, 0),
                FixedQuaternion.Identity,
                Fixed64.Two,
                Fixed64.Zero,
                Vector3d.Down,
                Vector3d.Up,
                out FixedContactAnchors contact));
            Assert.Equal(default, contact);
        }
    }

    [Fact]
    public void TriangleSupportContact_PreservesTolerantlyNormalizedPlaneScale()
    {
        Fixed64 tiny = Fixed64.FromRaw(1L << 20);
        var normal = new Vector3d(Fixed64.One, tiny, Fixed64.Zero);
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(tiny, -Fixed64.One, Fixed64.Zero),
            Vector3d.Forward);
        var supportPoint = new Vector3d(
            Fixed64.MinValue,
            Fixed64.FromRaw(-(1L << 51)),
            Fixed64.Zero);
        var cylinderCenter = new Vector3d(
            supportPoint.X,
            supportPoint.Y + Fixed64.One,
            Fixed64.Zero);

        Assert.True(triangle.TryGetCenteredFiniteCylinderSupportContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            cylinderCenter,
            FixedQuaternion.Identity,
            Fixed64.Two,
            Fixed64.Zero,
            Vector3d.Down,
            normal,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void CenteredCapsuleContact_RoundsExactDistanceBeforeDepth()
    {
        const long k = (1L << 32) - 1L;
        const long m = 1L << 16;

        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.FromRaw(k + 2L),
            new Vector3d(Fixed64.FromRaw(k), Fixed64.FromRaw(m), Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.Zero,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(1L), contact.Depth);

        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.FromRaw(2L),
            new Vector3d(
                Fixed64.FromRaw(1L),
                Fixed64.FromRaw(1L),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.Zero,
            Vector3d.Right,
            out contact));
        Assert.Equal(Fixed64.FromRaw(1L), contact.Depth);

        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.FromRaw((1L << 32) + 1L),
            new Vector3d(
                Fixed64.FromRaw(1L << 32),
                Fixed64.FromRaw(1L),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.Zero,
            Vector3d.Right,
            out contact));
        Assert.Equal(Fixed64.FromRaw(1L), contact.Depth);
    }

    [Fact]
    public void CenteredCapsuleTriangleContact_RoundsExactDistanceBeforeDepth()
    {
        const long k = (1L << 32) - 1L;
        const long m = 1L << 16;
        var point = new Vector3d(
            Fixed64.FromRaw(k),
            Fixed64.FromRaw(m),
            Fixed64.Zero);
        var triangle = new FixedTriangle(point, point, point);

        Assert.True(triangle.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.FromRaw(k + 2L),
            Vector3d.Right,
            out _,
            out _,
            out _,
            out Fixed64 depth));
        Assert.Equal(Fixed64.FromRaw(1L), depth);
    }
}
