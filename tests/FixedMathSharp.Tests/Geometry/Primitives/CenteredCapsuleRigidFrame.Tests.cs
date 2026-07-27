using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class CenteredCapsuleRigidFrameTests
{
    [Fact]
    public void RigidFrameContact_ClassifiesEveryFiniteAxisFeaturePair()
    {
        var cases = new[]
        {
            new CapsulePairCase(
                "interior-interior",
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Zero),
            new CapsulePairCase(
                "lower-interior",
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One),
            new CapsulePairCase(
                "upper-interior",
                new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One),
            new CapsulePairCase(
                "interior-lower",
                Vector3d.Zero,
                Vector3d.Right,
                new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One),
            new CapsulePairCase(
                "interior-upper",
                Vector3d.Zero,
                Vector3d.Right,
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One),
            new CapsulePairCase(
                "lower-lower",
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Two),
            new CapsulePairCase(
                "upper-upper",
                new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Two),
            new CapsulePairCase(
                "lower-upper",
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Two),
            new CapsulePairCase(
                "upper-lower",
                new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Two),
        };

        foreach (CapsulePairCase value in cases)
        {
            if (value.Distance > Fixed64.Zero)
            {
                Assert.False(TryGetContact(
                    value,
                    value.Distance - Fixed64.FromRaw(1),
                    out _));
            }

            Assert.True(
                TryGetContact(value, value.Distance, out FixedContactAnchors contact),
                value.Name);
            Assert.True(contact.Depth >= Fixed64.Zero, value.Name);
        }
    }

    [Fact]
    public void RigidFrameContact_DoesNotRoundParallelAxesBeforeClassification()
    {
        var rotation = new FixedQuaternion(
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(1_191_209_601L));
        Assert.True(rotation.IsNormalized());

        Vector3d firstCenter = new(
            Fixed64.MaxValue - (Fixed64)4,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d secondCenter = new(
            firstCenter.X + Fixed64.One,
            Fixed64.Two,
            (Fixed64)3);
        Vector3d roundedAxis = rotation.Rotate(Vector3d.Up).Normalized;
        Fixed64 roundedFalsePositiveRadius =
            Fixed64.FromRaw(14_929_469_565L);

        Assert.True(FixedSegment.DoCenteredCapsulesOverlap(
            firstCenter,
            roundedAxis,
            Fixed64.MaxValue,
            roundedFalsePositiveRadius,
            secondCenter,
            roundedAxis,
            Fixed64.MaxValue,
            Fixed64.Zero));

        Assert.False(FixedSegment.TryGetCenteredCapsulesContact(
            firstCenter,
            rotation,
            Vector3d.Up,
            Fixed64.MaxValue,
            roundedFalsePositiveRadius,
            secondCenter,
            rotation,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Vector3d.Right,
            out _));
        Assert.False(FixedSegment.TryGetCenteredCapsulesContact(
            firstCenter,
            rotation,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.FromRaw(14_929_469_566L),
            secondCenter,
            rotation,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Vector3d.Right,
            out _));
        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            firstCenter,
            rotation,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.FromRaw(14_929_469_567L),
            secondCenter,
            rotation,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth >= Fixed64.Zero);
    }

    private static bool TryGetContact(
        CapsulePairCase value,
        Fixed64 radius,
        out FixedContactAnchors contact) =>
        FixedSegment.TryGetCenteredCapsulesContact(
            value.FirstCenter,
            FixedQuaternion.Identity,
            value.FirstAxis,
            value.AxisLength,
            radius,
            value.SecondCenter,
            FixedQuaternion.Identity,
            value.SecondAxis,
            value.AxisLength,
            Fixed64.Zero,
            Vector3d.Forward,
            out contact);

    private readonly struct CapsulePairCase
    {
        internal CapsulePairCase(
            string name,
            Vector3d firstCenter,
            Vector3d firstAxis,
            Vector3d secondCenter,
            Vector3d secondAxis,
            Fixed64 axisLength,
            Fixed64 distance)
        {
            Name = name;
            FirstCenter = firstCenter;
            FirstAxis = firstAxis;
            SecondCenter = secondCenter;
            SecondAxis = secondAxis;
            AxisLength = axisLength;
            Distance = distance;
        }

        internal string Name { get; }
        internal Vector3d FirstCenter { get; }
        internal Vector3d FirstAxis { get; }
        internal Vector3d SecondCenter { get; }
        internal Vector3d SecondAxis { get; }
        internal Fixed64 AxisLength { get; }
        internal Fixed64 Distance { get; }
    }
}
