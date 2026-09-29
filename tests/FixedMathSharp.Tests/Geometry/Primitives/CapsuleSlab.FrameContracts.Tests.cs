//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleCapsuleSlabFrameContractTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PositiveCoreContact_PreservesDirtyCallerHeadroomOnOneMiBStack(bool largeRotated)
    {
        // The endpoint cylinder at +X owns this exact interior quartic root.
        // Its inward normal is (-3,-12,-4)/13 and depth is 13/256.
        Fixed64 radius = largeRotated ? Fixed64.FromRaw(long.MaxValue / 4) : (Fixed64)5;
        Fixed64 length = largeRotated ? Fixed64.MaxValue : Fixed64.Two;
        Fixed64 yaw = largeRotated ? Fixed64.FromFraction(7, 13) : Fixed64.Zero;
        Vector2d axis = largeRotated
            ? new Vector2d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)) : Vector2d.Right;
        Vector3d center = largeRotated
            ? new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue) : Vector3d.Zero;
        Vector3d origin = largeRotated ? center : Vector3d.Right;
        FixedQuaternion rotation = largeRotated
            ? new FixedQuaternion((Fixed64)(-1), (Fixed64)(-9), (Fixed64)7, Fixed64.Zero).Normalized
            : FixedQuaternion.Identity;
        FixedTriangle triangle = largeRotated
            ? new FixedTriangle(new Vector3d(-radius, Fixed64.Zero, -radius),
                new Vector3d(radius, Fixed64.Zero, -radius), new Vector3d(Fixed64.Zero, Fixed64.Zero, radius))
            : new FixedTriangle(
                new Vector3d(Fixed64.FromFraction(1021, 256), Fixed64.FromFraction(309, 64), Fixed64.FromFraction(231, 64)),
                new Vector3d(Fixed64.FromFraction(509, 256), Fixed64.FromFraction(325, 64), Fixed64.FromFraction(279, 64)),
                new Vector3d(Fixed64.FromFraction(813, 256), Fixed64.FromFraction(365, 64), Fixed64.FromFraction(271, 64)));
        var contacts = new FixedContactAnchors[2];
        var hits = new bool[2];
        Exception? failure = null;
        var worker = new Thread(() =>
        {
            try
            {
                for (int index = 0; index < contacts.Length; index++)
                    hits[index] = GetContactWithLiveCallerBuffer(triangle, origin, rotation, center, yaw, axis, length, radius,
                        index == 0 ? 0xA55A_0FF0_1234_5678UL : 0x5AA5_F00F_FEDC_BA98UL, out contacts[index]);
            }
            catch (Exception exception) { failure = exception; }
        }, 1024 * 1024);
        worker.Start(); worker.Join();
        Assert.Null(failure);
        for (int index = 0; index < contacts.Length; index++)
        {
            Assert.True(hits[index]);
            Assert.False(contacts[index].DepthIsClamped);
            Assert.Equal(origin, contacts[index].FirstAnchor.Origin);
            Assert.Equal(center, contacts[index].SecondAnchor.Origin);
            Assert.Equal(rotation, contacts[index].FirstAnchor.Rotation);
            Assert.Equal(FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw), contacts[index].SecondAnchor.Rotation);
            if (largeRotated)
            {
                // Weights (1/4,1/4,1/2) put the shared center in the triangle,
                // so the stadium's central radius-R ball is a lower bound.
                // With R=H<Max/4, L=Max and |axis|<sqrt(2), every slab point
                // has norm <Max*sqrt((sqrt(2)/2+1/4)^2+1/16)<Max.
                // Its face support therefore bounds depth strictly below Max.
                // Shared extreme origin cancels translation: this stresses
                // large extents and distinct frames, not opposite-origin width.
                Assert.InRange(contacts[index].Depth.m_rawValue, radius.m_rawValue, long.MaxValue - 1);
                Assert.True(contacts[index].Normal.IsNormalized());
            }
            else
            {
                Assert.Equal(Fixed64.FromFraction(13, 256), contacts[index].Depth);
                Assert.Equal(new Vector3d(-Fixed64.FromFraction(3, 13), -Fixed64.FromFraction(12, 13),
                    -Fixed64.FromFraction(4, 13)), contacts[index].Normal);
                Assert.True(contacts[index].SecondAnchor.TryGetPoint(out Vector3d point));
                Assert.Equal(new Vector3d(4, 5, 4), point);
            }
        }
        Assert.Equal(contacts[0], contacts[1]);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool GetContactWithLiveCallerBuffer(FixedTriangle triangle, Vector3d origin,
        FixedQuaternion rotation, Vector3d center, Fixed64 yaw, Vector2d axis, Fixed64 length,
        Fixed64 radius, ulong seed, out FixedContactAnchors contact)
    {
        Span<ulong> caller = stackalloc ulong[8192];
        for (int index = 0; index < caller.Length; index++)
            caller[index] = seed ^ unchecked(0x9E37_79B9_7F4A_7C15UL * (ulong)(index + 1));
        bool hit = triangle.TryGetCenteredCapsuleSlabContact(origin, rotation,
            center, yaw, axis, length, radius, radius, out contact);
        for (int index = 0; index < caller.Length; index++)
            if (caller[index] != (seed ^ unchecked(0x9E37_79B9_7F4A_7C15UL * (ulong)(index + 1))))
                throw new InvalidOperationException("Capsule-slab contact changed its caller's stack buffer.");
        return hit;
    }

    [Fact]
    public void AuthoredYawAndNoncardinalCore_RetainLocalFaceDepthAndPairedFrames()
    {
        // In the common authored frame, the positive endpoint is a=(3/5,0,4/5)
        // (its exact Q32 components). The +X support is a+Right. A plane 1/4
        // inward contains its orthogonal foot and a radius-1/4 ball fits in
        // that endpoint cylinder. Rotating the entire fixture cannot alter depth.
        Vector2d axis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Fixed64 yaw = Fixed64.FromFraction(7, 13);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw);
        Vector3d origin = new(17, -23, 31);
        Fixed64 faceX = Fixed64.One + axis.X - Fixed64.FromFraction(1, 4);
        var triangle = new FixedTriangle(new Vector3d(faceX, (Fixed64)(-4), (Fixed64)(-4)),
            new Vector3d(faceX, (Fixed64)4, (Fixed64)(-4)), new Vector3d(faceX, Fixed64.Zero, (Fixed64)4));

        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(origin, rotation, origin, yaw,
            axis, Fixed64.Two, Fixed64.One, Fixed64.Two, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(origin, contact.FirstAnchor.Origin);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.Equal(rotation, contact.FirstAnchor.Rotation);
        Assert.Equal(rotation, contact.SecondAnchor.Rotation);
        var normal = new FixedPointAnchor(Vector3d.Zero, rotation, Vector3d.Left);
        Assert.True(normal.TryGetPoint(out Vector3d expectedNormal));
        Assert.True(contact.FirstAnchor.TryGetLocalPointIn(origin, rotation, out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetLocalPointIn(origin, rotation, out Vector3d second));
        Assert.Equal(faceX, first.X);
        Assert.Equal(Fixed64.One + axis.X, second.X);
        Assert.Equal(axis.Y, first.Z);
        Assert.Equal(axis.Y, second.Z);
        // Axial Y is free on the supporting barrel; require a paired point,
        // not an arbitrary choice of the barrel's midpoint.
        Assert.Equal(first.Y, second.Y);
        Assert.InRange(first.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
        Assert.Equal(expectedNormal, contact.Normal);
    }

    [Fact]
    public void OddRawCore_RetainsHalfRawEndpointAfterDepthRoundsToZero()
    {
        // The endpoint cylinder extends to X=2+1/2 raw. Its plane at X=2
        // therefore overlaps by exactly 1/2 raw: ties-to-even depth is zero,
        // but the supporting feature must still retain the residual.
        var triangle = new FixedTriangle(new Vector3d(2, -4, -4),
            new Vector3d(2, 4, -4), new Vector3d(2, 0, 4));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.Two + Fixed64.MinIncrement,
            Fixed64.One, Fixed64.Two, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.Two, first.X);
        Assert.Equal(Fixed64.Zero, first.Z);
        Assert.Equal(first, second);
        Assert.InRange(first.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
        FixedPointAnchor anchor = contact.SecondAnchor;
        var roundedOnly = new FixedPointAnchor(anchor.Origin, anchor.Rotation,
            anchor.LocalPoint, anchor.LocalDisplacement);
        Assert.NotEqual(0, anchor.CompareLocalFeature(roundedOnly));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void FullDomainCap_RetainsMirroredUnmaterializedWitnesses(int sign)
    {
        // Local face and cap lie 1/4 apart; both are outside the scalar world
        // range after translation. Core length and half-height are full-domain,
        // while the central endpoint-free cap point certifies depth 1/4.
        Fixed64 faceY = sign * (Fixed64.MaxValue - Fixed64.FromFraction(1, 4));
        var origin = new Vector3d(Fixed64.Zero, sign > 0 ? Fixed64.MaxValue : Fixed64.MinValue, Fixed64.Zero);
        var triangle = new FixedTriangle(new Vector3d((Fixed64)(-16), faceY, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, faceY, (Fixed64)(-16)), new Vector3d(Fixed64.Zero, faceY, (Fixed64)16));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(origin, FixedQuaternion.Identity,
            origin, Fixed64.Zero, Vector2d.Right, Fixed64.MaxValue, (Fixed64)5,
            Fixed64.MaxValue, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)(-sign), Fixed64.Zero), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(origin, contact.FirstAnchor.Origin);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
    }

    [Fact]
    public void ZeroRadiusLongCore_CoplanarTriangleHasZeroDepthMatchedWitness()
    {
        // The slab degenerates to the X/Y rectangle, not two isolated endpoint
        // segments. This triangle intersects only its middle and admits escape
        // by an arbitrarily small Z translation.
        var triangle = new FixedTriangle(new Vector3d(-2, -2, 0),
            new Vector3d(2, -2, 0), new Vector3d(0, 2, 0));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.MaxValue,
            Fixed64.Zero, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.Zero, contact.Normal.X);
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.Equal(Fixed64.One, contact.Normal.Z.Abs());
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(first, second);
        Assert.Equal(Fixed64.Zero, first.Z);
        Assert.InRange(first.Y.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
        Assert.InRange(first.X.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
    }
}
