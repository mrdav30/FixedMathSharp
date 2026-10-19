using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedConvexPrismCurvedResourceTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PositiveOddLeadingHalfTie_RoundsTheCompleteAnchorCoordinateToEven(bool cone)
    {
        const long scale = 1L << 32;
        const long k = (scale - 1) / 5;
        const long lengthRaw = long.MaxValue - (scale - 1);
        const long radiusRaw = 7 * (1L << 60) + 9 * (1L << 28) - 1;
        Assert.Equal(0L, radiusRaw % 5);
        Assert.Equal(BigInteger.Pow(scale - 1, 2), 25 * BigInteger.Pow(k, 2));
        // The actual authored norm is (S-1)/S. The perpendicular unit
        // radial direction is (4,-3)/5 for the cylinder and (4,3)/5 for
        // the cone's base. Both support X coordinates are MaxRaw+1/2.
        BigInteger twiceSupportX = 3 * (BigInteger)k * lengthRaw / scale + 8 * (BigInteger)radiusRaw / 5;
        Assert.Equal(2 * (BigInteger)long.MaxValue + 1, twiceSupportX);
        BigInteger supportY = 2 * (BigInteger)k * ((1L << 31) - 1) - 3 * (BigInteger)radiusRaw / 5;
        long supportYRaw = checked((long)(cone ? -supportY : supportY));
        Vector3d axis = new(Fixed64.FromRaw((cone ? -3 : 3) * k), Fixed64.FromRaw(4 * k), Fixed64.Zero);
        Assert.True(axis.IsNormalized());
        Vector3d center = -Vector3d.Right;
        Vector3d prismOrigin = new(Fixed64.FromRaw(long.MaxValue - scale),
            Fixed64.FromRaw(supportYRaw), Fixed64.Zero);
        Vector2d[] footprint = { new(0, -1), new(1, -1), new(1, 1), new(0, 1) };
        FixedContactAnchors contact;
        bool hit = cone
            ? FixedConvexPrismRelations.TryGetCenteredConeContact(center, FixedQuaternion.Identity, axis,
                Fixed64.FromRaw(lengthRaw), Fixed64.FromRaw(radiusRaw), prismOrigin, Fixed64.Zero,
                footprint, Fixed64.One, out contact)
            : FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity, axis,
                Fixed64.FromRaw(lengthRaw), Fixed64.FromRaw(radiusRaw), prismOrigin, Fixed64.Zero,
                footprint, Fixed64.One, out contact);
        Assert.True(hit);
        // The prism contains the radius-half-raw ball centered at this exact
        // shape support: all other target faces are at least one unit away.
        // Thus the complete Minkowski difference contains that same ball,
        // while the +X support plane supplies the matching half-raw upper
        // bound. Depth rounds to zero, independently of witness rounding.
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.MaxValue, contact.FirstAnchor.LocalPoint.X);
        Assert.Equal(Fixed64.FromRaw(1), contact.FirstAnchor.LocalDisplacement.X);
        Assert.Equal(BigInteger.One << 63,
            (BigInteger)contact.FirstAnchor.LocalPoint.X.m_rawValue + contact.FirstAnchor.LocalDisplacement.X.m_rawValue);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(long.MaxValue - scale + 1L),
            Fixed64.FromRaw(supportYRaw), Fixed64.Zero), first);
        Assert.Equal(prismOrigin, second);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BelowMinimumLocalCoordinate_RetainsTheNegativeAdditiveAnchor(bool cone)
    {
        const long scale = 1L << 32;
        const long k = (scale - 1) / 5;
        const long lengthRaw = long.MaxValue - (scale - 1);
        const long originalRadiusRaw = 7 * (1L << 60) + 9 * (1L << 28) - 1;
        const long radiusRaw = originalRadiusRaw + 1;
        // Mirroring the half-tie support and increasing R by one raw gives
        // local X=MinRaw-3/10. Its exact positive penetration is 3/10 raw.
        BigInteger tenSupportMagnitude = 15 * (BigInteger)k * lengthRaw / scale + 8 * (BigInteger)radiusRaw;
        Assert.Equal(10 * (BigInteger.One << 63) + 3, tenSupportMagnitude);
        BigInteger originalY = 2 * (BigInteger)k * ((1L << 31) - 1) - 3 * (BigInteger)originalRadiusRaw / 5;
        // The cylinder Y becomes originalY-3/5 raw, with the cone's Y its
        // negative. Both round away from originalY by exactly one raw.
        long roundedY = checked((long)(cone ? -originalY + 1 : originalY - 1));
        Vector3d axis = new(Fixed64.FromRaw((cone ? 3 : -3) * k), Fixed64.FromRaw(4 * k), Fixed64.Zero);
        Assert.True(axis.IsNormalized());
        Vector3d center = Vector3d.Right;
        Vector3d prismOrigin = new(Fixed64.FromRaw(long.MinValue + scale), Fixed64.FromRaw(roundedY), Fixed64.Zero);
        Vector2d[] footprint = { new(-1, -1), new(0, -1), new(0, 1), new(-1, 1) };
        FixedContactAnchors contact;
        bool hit = cone
            ? FixedConvexPrismRelations.TryGetCenteredConeContact(center, FixedQuaternion.Identity, axis,
                Fixed64.FromRaw(lengthRaw), Fixed64.FromRaw(radiusRaw), prismOrigin, Fixed64.Zero,
                footprint, Fixed64.One, out contact)
            : FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity, axis,
                Fixed64.FromRaw(lengthRaw), Fixed64.FromRaw(radiusRaw), prismOrigin, Fixed64.Zero,
                footprint, Fixed64.One, out contact);
        Assert.True(hit);
        // The exact support is 3/10 raw inside the target's right face and
        // less than half a raw from its Y center. Its radius-3/10-raw ball
        // lies in the prism, proving the matching global minimum as above.
        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.MinValue, contact.FirstAnchor.LocalPoint.X);
        Assert.Equal(Fixed64.Zero, contact.FirstAnchor.LocalDisplacement.X);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(prismOrigin, first);
        Assert.Equal(prismOrigin, second);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WarmedFiniteFeatureContact_AllocatesNothingAndRepeatsItsWitnesses(bool cone)
    {
        Vector2d[] footprint = CreateFootprint(cone);
        Assert.True(Query(cone, footprint, out FixedContactAnchors expected));
        AssertContact(cone, expected);
        var hits = new bool[2];
        var contacts = new FixedContactAnchors[2];
        Action operation = () =>
        {
            for (int index = 0; index < contacts.Length; index++)
                hits[index] = Query(cone, footprint, out contacts[index]);
        };
        Assert.Equal(0L, FixedMathTestHelper.MeasureWarmedAllocations(operation));
        for (int index = 0; index < contacts.Length; index++)
        {
            Assert.True(hits[index]);
            Assert.Equal(expected, contacts[index]);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FiniteFeatureContact_PreservesDirtyCallerHeadroomOnOneMiBStack(bool cone)
    {
        Vector2d[] footprint = CreateFootprint(cone);
        var hits = new bool[2];
        var contacts = new FixedContactAnchors[2];
        Exception? failure = null;
        var worker = new Thread(() =>
        {
            try
            {
                for (int index = 0; index < contacts.Length; index++)
                    hits[index] = QueryWithLiveCallerBuffer(cone, footprint,
                        index == 0 ? 0xA55A_0FF0_1234_5678UL : 0x5AA5_F00F_FEDC_BA98UL, out contacts[index]);
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
            AssertContact(cone, contacts[index]);
        }
        Assert.Equal(contacts[0], contacts[1]);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool QueryWithLiveCallerBuffer(bool cone, Vector2d[] footprint, ulong seed,
        out FixedContactAnchors contact)
    {
        Span<ulong> caller = stackalloc ulong[8192];
        for (int index = 0; index < caller.Length; index++)
            caller[index] = seed ^ unchecked(0x9E37_79B9_7F4A_7C15UL * (ulong)(index + 1));
        bool hit = Query(cone, footprint, out contact);
        for (int index = 0; index < caller.Length; index++)
            if (caller[index] != (seed ^ unchecked(0x9E37_79B9_7F4A_7C15UL * (ulong)(index + 1))))
                throw new InvalidOperationException("Curved prism contact changed its caller's stack buffer.");
        return hit;
    }

    private static bool Query(bool cone, Vector2d[] footprint, out FixedContactAnchors contact)
    {
        if (cone)
        {
            Fixed64 delta = Fixed64.FromFraction(1, 64);
            return FixedConvexPrismRelations.TryGetCenteredConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Up, Fixed64.Two, Fixed64.One,
                new Vector3d(Fixed64.Half - Fixed64.Two * delta, Fixed64.One - delta, Fixed64.Zero),
                Fixed64.Zero, footprint, Fixed64.One, out contact);
        }
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        Vector3d center = new((Fixed64)(-5), Fixed64.FromFraction(31, 2) - Fixed64.FromFraction(15, 1024),
            Fixed64.FromFraction(41, 2) - Fixed64.FromFraction(16, 1024));
        return FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, out contact);
    }

    private static Vector2d[] CreateFootprint(bool cone) => cone
        ? new Vector2d[] { new(0, -1), new(1, -1), new(1, 1), new(0, 1) }
        : new Vector2d[] { new((Fixed64)(-10), -Fixed64.Half), new((Fixed64)10, -Fixed64.Half),
            new((Fixed64)10, Fixed64.Half), new((Fixed64)(-10), Fixed64.Half) };

    private static void AssertContact(bool cone, FixedContactAnchors contact)
    {
        // The cone's 2x+y supporting plane gives sqrt(5)/64. The cylinder
        // fixture has exact axis (4,3,0)/5 and rim support direction
        // (0,-15,-16)/sqrt(481), giving sqrt(481)/1024.
        Assert.Equal(Fixed64.FromRaw(cone ? 150_059_982L : 91_988_268L), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        if (cone)
        {
            Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), first);
            Assert.Equal(new Vector3d(Fixed64.Half - Fixed64.FromFraction(1, 32),
                -Fixed64.FromFraction(1, 64), Fixed64.Zero), second);
        }
        else
        {
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Half - Fixed64.FromFraction(15, 1024),
                Fixed64.Half - Fixed64.FromFraction(16, 1024)), first);
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Half), second);
        }
    }
}
