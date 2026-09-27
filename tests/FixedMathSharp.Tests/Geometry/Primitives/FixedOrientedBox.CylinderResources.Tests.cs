using System;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedOrientedBoxCylinderResourceTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Contact_PreservesSixtyFourKiBOfDirtyCallerHeadroomOnOneMiBStack(bool fullWidth)
    {
        CreateFixture(fullWidth, out FixedOrientedBox box, out Vector3d center,
            out FixedQuaternion rotation, out Fixed64 length, out Fixed64 radius);
        var contacts = new FixedContactAnchors[2];
        var hits = new bool[2];
        Exception? failure = null;
        var worker = new Thread(() =>
        {
            try
            {
                for (int index = 0; index < contacts.Length; index++)
                    hits[index] = GetContactWithLiveCallerBuffer(box, center, rotation, length, radius,
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
        for (int index = 0; index < contacts.Length; index++)
        {
            Assert.True(hits[index]);
            FixedContactAnchors contact = contacts[index];
            Assert.False(contact.DepthIsClamped);
            Assert.Equal(box.Center, contact.FirstAnchor.Origin);
            Assert.Equal(center, contact.SecondAnchor.Origin);
            if (fullWidth)
            {
                // Each centered shape contains a ball of radius r, so their
                // Minkowski sum contains a ball of radius 2r. The cylinder's
                // cap axis gives an upper bound sqrt(3)*r+length/2 < MaxValue.
                // This proves positive, unclamped depth without asking this
                // contact solver to manufacture its own expected answer.
                Assert.InRange(contact.Depth.m_rawValue, 2 * radius.m_rawValue, long.MaxValue - 1);
                Assert.InRange(contact.Normal.MagnitudeSquared.m_rawValue,
                    Fixed64.One.m_rawValue - 8, Fixed64.One.m_rawValue + 8);
            }
            else
            {
                AssertPositiveEdgeContact(contact);
            }
        }
        Assert.Equal(contacts[0], contacts[1]);
    }

    [Fact]
    public void PositiveEdgeRimContact_AllocatesNothingAndRepeatsExactOutput()
    {
        CreateFixture(false, out FixedOrientedBox box, out Vector3d center,
            out FixedQuaternion rotation, out Fixed64 length, out Fixed64 radius);
        Assert.True(box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            length, radius, out FixedContactAnchors expected));
        AssertPositiveEdgeContact(expected);
        var contacts = new FixedContactAnchors[2];
        var hits = new bool[2];
        // All closures, arrays, authored geometry and assertions are outside
        // the measured operation. Two calls exercise repeatability without
        // turning this allocation regression into a throughput workload.
        Action operation = () =>
        {
            for (int index = 0; index < contacts.Length; index++)
                hits[index] = box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
                    length, radius, out contacts[index]);
        };
        Assert.Equal(0L, FixedMathTestHelper.MeasureWarmedAllocations(operation));
        for (int index = 0; index < contacts.Length; index++)
        {
            Assert.True(hits[index]);
            Assert.Equal(expected, contacts[index]);
        }
    }

    private static void AssertPositiveEdgeContact(FixedContactAnchors contact)
    {
        // At R=25 the unique support normal is (0,15,16)/sqrt(481).
        // Increasing R by one raw unit gives depth rounding to one raw unit;
        // its normal remains strictly inside the +Y/+Z edge cone.
        Assert.Equal(Fixed64.FromRaw(1), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.Zero, contact.Normal.X);
        Assert.True(contact.Normal.Y > Fixed64.Zero);
        Assert.True(contact.Normal.Z > Fixed64.Zero);
        Assert.Equal(new Vector3d((Fixed64)(-10), Fixed64.Half, Fixed64.Half), contact.FirstAnchor.LocalPoint);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool GetContactWithLiveCallerBuffer(FixedOrientedBox box, Vector3d center,
        FixedQuaternion rotation, Fixed64 length, Fixed64 radius, ulong seed,
        out FixedContactAnchors contact)
    {
        Span<ulong> caller = stackalloc ulong[8192];
        for (int index = 0; index < caller.Length; index++)
            caller[index] = DirtyWord(seed, index);
        bool hit = box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            length, radius, out contact);
        // Reading every word after return keeps all 64 KiB live across the
        // contact's deepest frame and catches caller-buffer corruption.
        for (int index = 0; index < caller.Length; index++)
            if (caller[index] != DirtyWord(seed, index))
                throw new InvalidOperationException("Box-cylinder contact changed its caller's stack buffer.");
        return hit;
    }

    private static ulong DirtyWord(ulong seed, int index) =>
        seed ^ unchecked(0x9E37_79B9_7F4A_7C15UL * (ulong)(index + 1));

    private static void CreateFixture(bool fullWidth, out FixedOrientedBox box,
        out Vector3d center, out FixedQuaternion rotation, out Fixed64 length, out Fixed64 radius)
    {
        if (fullWidth)
        {
            center = new Vector3d(Fixed64.FromRaw(long.MaxValue - (32L << 32)),
                Fixed64.FromRaw(long.MinValue + (32L << 32)),
                Fixed64.FromRaw(long.MaxValue - (32L << 32)));
            radius = Fixed64.FromRaw(long.MaxValue / 4);
            box = new FixedOrientedBox(center, new FixedQuaternion((Fixed64)(-1), (Fixed64)(-9), (Fixed64)7, Fixed64.Zero).Normalized,
                new Vector3d(radius, radius, radius));
            rotation = new FixedQuaternion((Fixed64)3, (Fixed64)(-2), (Fixed64)5, (Fixed64)7).Normalized;
            length = Fixed64.MaxValue;
            return;
        }
        const long quaternionScale = 1_920_767_767L;
        box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d((Fixed64)10, Fixed64.Half, Fixed64.Half));
        center = new Vector3d((Fixed64)(-5), Fixed64.FromFraction(31, 2), Fixed64.FromFraction(41, 2));
        rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        length = (Fixed64)10;
        radius = (Fixed64)25 + Fixed64.FromRaw(1);
    }
}
