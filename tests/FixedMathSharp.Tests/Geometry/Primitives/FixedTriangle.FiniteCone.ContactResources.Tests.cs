using System;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FixedTriangleFiniteConeContactTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Contact_PreservesDirtyCallerHeadroomOnOneMiBStack(bool largeRotated)
    {
        Fixed64 radius = largeRotated ? Fixed64.FromRaw(long.MaxValue / 4) : (Fixed64)5;
        Fixed64 height = largeRotated ? Fixed64.FromRaw(long.MaxValue / 2) : (Fixed64)10;
        var origin = largeRotated ? new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue) : Vector3d.Zero;
        FixedTriangle triangle = largeRotated
            ? new FixedTriangle(new Vector3d(-radius, Fixed64.Zero, -radius),
                new Vector3d(radius, Fixed64.Zero, -radius), new Vector3d(Fixed64.Zero, Fixed64.Zero, radius))
            : InteriorRimTriangle();
        FixedQuaternion triangleRotation = largeRotated
            ? new FixedQuaternion((Fixed64)(-1), (Fixed64)(-9), (Fixed64)7, Fixed64.Zero).Normalized
            : FixedQuaternion.Identity;
        FixedQuaternion coneRotation = largeRotated
            ? new FixedQuaternion((Fixed64)3, (Fixed64)(-2), (Fixed64)5, (Fixed64)7).Normalized
            : FixedQuaternion.Identity;
        var contacts = new FixedContactAnchors[2];
        var hits = new bool[2];
        Exception? failure = null;
        var worker = new Thread(() =>
        {
            try
            {
                for (int index = 0; index < contacts.Length; index++)
                    hits[index] = ContactWithLiveCallerBuffer(triangle, origin, triangleRotation, coneRotation,
                        height, radius, index == 0 ? 0xA55A_0FF0_1234_5678UL : 0x5AA5_F00F_FEDC_BA98UL,
                        out contacts[index]);
            }
            catch (Exception exception)
            {
                failure = exception;
            }
        }, 1024 * 1024);
        worker.Start();
        worker.Join();
        Assert.Null(failure);
        for (int index = 0; index < contacts.Length; index++)
        {
            Assert.True(hits[index]);
            Assert.False(contacts[index].DepthIsClamped);
            Assert.Equal(origin, contacts[index].FirstAnchor.Origin);
            Assert.Equal(origin, contacts[index].SecondAnchor.Origin);
            if (largeRotated)
            {
                // The triangle contains its origin. With H approximately 2R,
                // the cone contains the origin-centered radius-R/4 ball; its
                // face support is bounded above by sqrt((H/2)^2+R^2) < Max.
                Assert.InRange(contacts[index].Depth.m_rawValue, radius.m_rawValue / 4, long.MaxValue - 1);
                Assert.True(contacts[index].Normal.IsNormalized());
            }
            else
                Assert.Equal(Fixed64.FromFraction(13, 256), contacts[index].Depth);
        }
        Assert.Equal(contacts[0], contacts[1]);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WarmedRimContact_AllocatesNothingAndRetainsItsExactResult(bool interiorRoot)
    {
        FixedTriangle triangle = interiorRoot ? InteriorRimTriangle() : ObliqueRimTriangle();
        bool hit = false;
        FixedContactAnchors contact = default;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
            hit = triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out contact));
        Assert.Equal(0, allocated);
        Assert.True(hit);
        Assert.Equal(interiorRoot ? Fixed64.FromFraction(13, 256) : Fixed64.FromFraction(5, 16), contact.Depth);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool ContactWithLiveCallerBuffer(FixedTriangle triangle, Vector3d origin,
        FixedQuaternion triangleRotation, FixedQuaternion coneRotation, Fixed64 height,
        Fixed64 radius, ulong seed, out FixedContactAnchors contact)
    {
        Span<ulong> caller = stackalloc ulong[8192];
        for (int index = 0; index < caller.Length; index++)
            caller[index] = seed ^ unchecked(0x9E37_79B9_7F4A_7C15UL * (ulong)(index + 1));
        bool hit = triangle.TryGetCenteredFiniteConeContact(origin, triangleRotation,
            origin, coneRotation, height, radius, out contact);
        for (int index = 0; index < caller.Length; index++)
            if (caller[index] != (seed ^ unchecked(0x9E37_79B9_7F4A_7C15UL * (ulong)(index + 1))))
                throw new InvalidOperationException("Triangle-cone contact changed its caller's stack buffer.");
        return hit;
    }
}
