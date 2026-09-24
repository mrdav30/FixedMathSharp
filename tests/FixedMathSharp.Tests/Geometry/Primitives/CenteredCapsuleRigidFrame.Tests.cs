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
            Assert.False(OverlapsStrict(value, value.Distance), value.Name);
            Assert.True(OverlapsStrict(value, value.Distance + Fixed64.MinIncrement), value.Name);
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

        Assert.False(FixedSegment.DoCenteredCapsulesOverlapStrict(
            firstCenter, rotation, Vector3d.Up, Fixed64.MaxValue, Fixed64.FromRaw(14_929_469_566L),
            secondCenter, rotation, Vector3d.Up, Fixed64.MaxValue, Fixed64.Zero));
        Assert.True(FixedSegment.DoCenteredCapsulesOverlapStrict(
            firstCenter, rotation, Vector3d.Up, Fixed64.MaxValue, Fixed64.FromRaw(14_929_469_567L),
            secondCenter, rotation, Vector3d.Up, Fixed64.MaxValue, Fixed64.Zero));
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, false)]
    [InlineData(1, false)]
    public void StrictRigidFrame_QuaternionSignPreservesRationalTangency(long rawStep, bool overlap)
    {
        // Raw components have exact ratio (-2,-2,-2,1), so local Up rotates
        // to (12/13,-3/13,4/13). Length 26 puts the upper endpoint at
        // (12,-3,4); the opposing point is exactly one unit beyond it in Z.
        FixedQuaternion rotation = new(
            Fixed64.FromRaw(-2_382_419_202L), Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(-2_382_419_202L), Fixed64.FromRaw(1_191_209_601L));
        Vector3d second = new((Fixed64)12, (Fixed64)(-3), Fixed64.FromRaw(((Fixed64)5).m_rawValue + rawStep));
        Assert.Equal(overlap, FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, rotation, Vector3d.Up, (Fixed64)26, Fixed64.One,
            second, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Zero));
        FixedQuaternion negated = new(-rotation.X, -rotation.Y, -rotation.Z, -rotation.W);
        Assert.Equal(overlap, FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, negated, Vector3d.Up, (Fixed64)26, Fixed64.One,
            second, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void StrictRigidFrame_DegenerateAxisAndHalfRawEndpointRemainExact()
    {
        Assert.True(FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Fixed64.MinIncrement, Fixed64.MinIncrement,
            new(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Zero));
        Assert.False(FixedSegment.DoCenteredCapsulesOverlapStrict(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Fixed64.Zero, Fixed64.Zero,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Zero));
    }

    private static bool OverlapsStrict(CapsulePairCase value, Fixed64 radius) =>
        FixedSegment.DoCenteredCapsulesOverlapStrict(
            value.FirstCenter, FixedQuaternion.Identity, value.FirstAxis, value.AxisLength, radius,
            value.SecondCenter, FixedQuaternion.Identity, value.SecondAxis, value.AxisLength, Fixed64.Zero);

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
