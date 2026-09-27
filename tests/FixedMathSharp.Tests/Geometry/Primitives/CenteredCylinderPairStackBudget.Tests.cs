using System;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderPairStackBudgetTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Contact_PreservesSixtyFourKiBOfCallerHeadroomOnOneMiBStack(int fixture)
    {
        // Ordinary and repeated irrational contacts use two different dirty
        // patterns. The full-width skew fixture needs only one public call.
        int count = fixture == 1 ? 1 : 2;
        var contacts = new FixedContactAnchors[count];
        var hits = new bool[count];
        Exception? failure = null;
        var worker = new Thread(() =>
        {
            try
            {
                for (int index = 0; index < count; index++)
                    hits[index] = GetContactWithLiveCallerBuffer(fixture,
                        index == 0 ? 0xA55A_0FF0_1234_5678UL : 0x5AA5_F00F_FEDC_BA98UL,
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
        for (int index = 0; index < count; index++)
        {
            Assert.True(hits[index]);
            FixedContactAnchors contact = contacts[index];
            Assert.False(contact.DepthIsClamped);
            Assert.Equal(Origin(), contact.FirstAnchor.Origin);
            if (fixture == 0)
            {
                Assert.Equal(Origin() + new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, (Fixed64)5 / 4),
                    contact.SecondAnchor.Origin);
                Assert.InRange(contact.Depth.m_rawValue, 1L, Fixed64.FromFraction(1, 20).m_rawValue);
                Assert.Equal(contact.Normal.X, contact.Normal.Y);
                Assert.True(contact.Normal.X > Fixed64.Zero);
                Assert.True(contact.Normal.Z > Fixed64.Zero);
            }
            else if (fixture == 1)
            {
                // Half-length is r+half a raw unit, so each cylinder contains
                // a ball of radius r. The common-perpendicular normal attains
                // the lower bound 2r exactly, independently of axis skew.
                Assert.Equal(Origin(), contact.SecondAnchor.Origin);
                Assert.Equal(long.MaxValue - 1, contact.Depth.m_rawValue);
                Assert.Equal(Fixed64.Zero, contact.Normal.X);
                Assert.Equal(Fixed64.Zero, contact.Normal.Y);
                Assert.True(contact.Normal.Z == Fixed64.One || contact.Normal.Z == -Fixed64.One);
            }
            else
            {
                // Independent exact certificate: depth²=351/400 and normal
                // component squares are 75/208 and 133/208, respectively.
                Assert.Equal(Origin() + new Vector3d(0, 1, 8), contact.SecondAnchor.Origin);
                Assert.Equal(4_023_309_325L, contact.Depth.m_rawValue);
                Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(2_579_044_439L),
                    Fixed64.FromRaw(3_434_424_822L)), contact.Normal);
            }
        }
        if (count == 2)
        {
            Assert.Equal(contacts[0].Depth, contacts[1].Depth);
            Assert.Equal(contacts[0].Normal, contacts[1].Normal);
            Assert.Equal(contacts[0].FirstAnchor.Origin, contacts[1].FirstAnchor.Origin);
            Assert.Equal(contacts[0].SecondAnchor.Origin, contacts[1].SecondAnchor.Origin);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool GetContactWithLiveCallerBuffer(int fixture, ulong seed, out FixedContactAnchors contact)
    {
        Span<ulong> caller = stackalloc ulong[8192];
        for (int index = 0; index < caller.Length; index++)
            caller[index] = DirtyWord(seed, index);
        bool hit = GetContact(fixture, out contact);
        // Every word is consumed after contact returns. This makes the full
        // 64 KiB a live containing frame, not a prior stack-pollution helper.
        for (int index = 0; index < caller.Length; index++)
            if (caller[index] != DirtyWord(seed, index))
                throw new InvalidOperationException("Cylinder contact changed its caller's stack buffer.");
        return hit;
    }

    private static ulong DirtyWord(ulong seed, int index) =>
        seed ^ unchecked(0x9E37_79B9_7F4A_7C15UL * (ulong)(index + 1));

    private static Vector3d Origin() => new(
        Fixed64.FromRaw(long.MaxValue - (16L << 32)),
        Fixed64.FromRaw(long.MinValue + (16L << 32)),
        Fixed64.FromRaw(long.MaxValue - (16L << 32)));

    private static bool GetContact(int fixture, out FixedContactAnchors contact)
    {
        Vector3d origin = Origin();
        if (fixture == 0)
            return FixedSegment.TryGetCenteredFiniteCylindersContact(
                origin, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One,
                origin + new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, (Fixed64)5 / 4),
                FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One, out contact);

        Fixed64 component = Fixed64.FromRaw(1_920_767_767L);
        if (fixture == 1)
        {
            var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, component, component + component);
            Fixed64 radius = Fixed64.FromRaw(long.MaxValue / 2);
            return FixedSegment.TryGetCenteredFiniteCylindersContact(
                origin, FixedQuaternion.Identity, Vector3d.Up, Fixed64.MaxValue, radius,
                origin, rotation, Vector3d.Right, Fixed64.MaxValue, radius, out contact);
        }

        var firstRotation = new FixedQuaternion(Fixed64.Zero, -component, Fixed64.Zero, component + component);
        var secondRotation = new FixedQuaternion(Fixed64.Zero, component, Fixed64.Zero, component + component);
        return FixedSegment.TryGetCenteredFiniteCylindersContact(
            origin, firstRotation, Vector3d.Right, (Fixed64)10, Fixed64.One,
            origin + new Vector3d(0, 1, 8), secondRotation, Vector3d.Left,
            (Fixed64)10, Fixed64.One, out contact);
    }
}
