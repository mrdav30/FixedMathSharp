using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedSlabProjectionTests
{
    private static readonly FixedRange WideSlab = new((Fixed64)(-10), (Fixed64)10);

    [Fact]
    public void CapsuleSupport_ProjectsOrdinaryVerticalShape()
    {
        bool found = FixedSlabProjection.TryGetCapsuleSupport(
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)3),
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            new FixedRange(-Fixed64.Half, Fixed64.Half),
            Vector2d.Right,
            out Vector2d support);

        Assert.True(found);
        Assert.Equal(new Vector2d((Fixed64)3, (Fixed64)3), support);
    }

    [Fact]
    public void CylinderSupport_ProjectsOrdinaryHorizontalShape()
    {
        bool found = FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)2,
            Fixed64.One,
            new FixedRange(-Fixed64.Half, Fixed64.Half),
            Vector2d.Right,
            out Vector2d support);

        Assert.True(found);
        Assert.Equal((Fixed64)2, support.X);
        Assert.Equal(-FixedMath.Sqrt(Fixed64.FromFraction(3, 4)), support.Y);
    }

    [Fact]
    public void UnclippedWinnerAdmission_PreservesEndpointTiesAndShapeKinds()
    {
        Assert.True(FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, Vector3d.Forward, (Fixed64)2, Fixed64.One,
            WideSlab, Vector2d.Right, out Vector2d capsule));
        Assert.Equal(new Vector2d(Fixed64.One, (Fixed64)(-2)), capsule);

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero, Vector3d.Right, (Fixed64)2, Fixed64.One,
            WideSlab, Vector2d.Forward, out Vector2d cylinder));
        Assert.Equal(new Vector2d((Fixed64)(-2), Fixed64.One), cylinder);

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Right, (Fixed64)4, (Fixed64)2,
            WideSlab, Vector2d.Forward, out Vector2d cone));
        Assert.Equal(new Vector2d((Fixed64)(-2), (Fixed64)2), cone);
    }

    [Fact]
    public void UnclippedWinnerAdmission_SelectsNegativeAxisEndpoint()
    {
        Assert.True(FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, Vector3d.Left, (Fixed64)2, Fixed64.One,
            WideSlab, Vector2d.Right, out Vector2d capsule));
        Assert.Equal(new Vector2d((Fixed64)3, Fixed64.Zero), capsule);

        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Assert.True(FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero, new Vector3d(-diagonal, Fixed64.Zero, diagonal), (Fixed64)2, Fixed64.One,
            WideSlab, Vector2d.Right, out Vector2d cylinder));
        Assert.True(cylinder.X > Fixed64.Zero);

        Assert.True(FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, Vector3d.Backward, (Fixed64)2, Fixed64.One,
            WideSlab, Vector2d.Right, out Vector2d reversedTie));
        Assert.Equal(new Vector2d(Fixed64.One, (Fixed64)(-2)), reversedTie);

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero, Vector3d.Left, (Fixed64)2, Fixed64.One,
            WideSlab, Vector2d.Forward, out Vector2d reversedCylinderTie));
        Assert.Equal(new Vector2d((Fixed64)(-2), Fixed64.One), reversedCylinderTie);
    }

    [Fact]
    public void UnclippedWinnerOutsideSlab_FallsBackToPartialClip()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d axis = new(diagonal, diagonal, Fixed64.Zero);
        FixedRange plane = new(Fixed64.Zero, Fixed64.Zero);

        Assert.True(FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, axis, (Fixed64)2, Fixed64.One,
            plane, Vector2d.Right, out Vector2d capsule));
        Assert.True(capsule.X < ((Fixed64)2 * diagonal) + Fixed64.One);

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero, axis, (Fixed64)2, Fixed64.Zero,
            plane, Vector2d.Right, out Vector2d cylinder));
        Assert.Equal(Vector2d.Zero, cylinder);

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, axis, (Fixed64)4, (Fixed64)2,
            plane, Vector2d.Right, out Vector2d cone));
        Assert.True(cone.X < (Fixed64)2 * diagonal);
    }

    [Fact]
    public void ConeSupport_ProjectsVerticalCrossSection()
    {
        bool found = FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            new FixedRange((Fixed64)(-1), (Fixed64)(-1)),
            Vector2d.Right,
            out Vector2d support);

        Assert.True(found);
        Assert.Equal(new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero), support);
    }

    [Fact]
    public void ConeSupport_ProjectsOppositeVerticalAxis()
    {
        bool found = FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero,
            Vector3d.Down,
            (Fixed64)2,
            Fixed64.One,
            new FixedRange(Fixed64.Zero, Fixed64.Zero),
            Vector2d.Right,
            out Vector2d support);

        Assert.True(found);
        Assert.Equal(new Vector2d(Fixed64.Half, Fixed64.Zero), support);
    }

    [Fact]
    public void ConeSupport_ProjectsRotatedLateralCrossSection()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        bool found = FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, diagonal, diagonal),
            (Fixed64)4,
            (Fixed64)2,
            new FixedRange(Fixed64.Zero, Fixed64.Zero),
            Vector2d.Forward,
            out Vector2d support);

        Assert.True(found);
        Assert.InRange(support.X.m_rawValue, -1L, 1L);
        FixedMathTestHelper.AssertWithinRelativeTolerance(
            Fixed64.FromDouble(0.94280904158),
            support.Y,
            Fixed64.FromDouble(0.00001));

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, diagonal, diagonal),
            (Fixed64)4,
            (Fixed64)2,
            new FixedRange(-Fixed64.FromFraction(1, 10), Fixed64.FromFraction(1, 10)),
            Vector2d.Forward,
            out Vector2d intervalSupport));
        Assert.True(intervalSupport.Y > Fixed64.Zero);
    }

    [Fact]
    public void ConeSupport_ProjectsRotatedCapAndApexCandidates()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d axis = new(Fixed64.Zero, diagonal, diagonal);

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, axis, (Fixed64)4, (Fixed64)2,
            WideSlab, Vector2d.Forward, out Vector2d wholeCone));
        FixedMathTestHelper.AssertWithinRelativeTolerance(
            (Fixed64)2 * diagonal,
            wholeCone.Y,
            Fixed64.FromDouble(0.00001));

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, axis, (Fixed64)4, (Fixed64)2,
            new FixedRange((Fixed64)(-1), (Fixed64)(-1)),
            -Vector2d.Forward,
            out Vector2d baseSlice));
        Assert.True(baseSlice.Y < Fixed64.Zero);
    }

    [Fact]
    public void ConeSupport_HandlesLinearStationaryEquation()
    {
        const long fifthRaw = 858993459L;
        bool found = FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero,
            new Vector3d(Fixed64.FromRaw(4L * fifthRaw), Fixed64.FromRaw(3L * fifthRaw), Fixed64.Zero),
            (Fixed64)4,
            Fixed64.FromRaw(15L * fifthRaw),
            new FixedRange(Fixed64.Zero, Fixed64.Zero),
            Vector2d.Right,
            out Vector2d support);

        Assert.True(found);
        Assert.Equal(Fixed64.FromRaw(5368709119L), support.X);
        Assert.Equal(Fixed64.Zero, support.Y);

        found = FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero,
            new Vector3d(Fixed64.FromRaw(-4L * fifthRaw), Fixed64.FromRaw(-3L * fifthRaw), Fixed64.Zero),
            (Fixed64)4,
            Fixed64.FromRaw(15L * fifthRaw),
            new FixedRange(Fixed64.Zero, Fixed64.Zero),
            Vector2d.Left,
            out support);

        Assert.True(found);
        Assert.Equal(Fixed64.FromRaw(-5368709119L), support.X);
        Assert.Equal(Fixed64.Zero, support.Y);
    }

    [Fact]
    public void ConeSupport_HandlesNegativeQuadraticAndRepeatedRoot()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d axis = new(Fixed64.Zero, diagonal, diagonal);

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, axis, Fixed64.One, (Fixed64)4,
            new FixedRange(Fixed64.Zero, Fixed64.Zero),
            Vector2d.Forward, out Vector2d wideCone));
        Assert.True(wideCone.Y > Fixed64.Zero);

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, axis, (Fixed64)4, Fixed64.Zero,
            new FixedRange(Fixed64.Zero, Fixed64.Zero),
            Vector2d.Forward, out Vector2d segment));
        Assert.Equal(Vector2d.Zero, segment);
    }

    [Fact]
    public void ConeSupport_ChoosesPositiveBaseChordTangent()
    {
        const long fifthRaw = 858993459L;
        Fixed64 radius = Fixed64.FromRaw(15L * fifthRaw);
        Vector3d axis = new(
            Fixed64.FromRaw(4L * fifthRaw),
            Fixed64.FromRaw(3L * fifthRaw),
            Fixed64.Zero);
        Fixed64 baseY = Fixed64.FromRaw(-6L * fifthRaw);

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, axis, (Fixed64)4, radius,
            new FixedRange(baseY, baseY),
            Vector2d.Forward, out Vector2d support));
        Assert.Equal(Fixed64.FromRaw(-8L * fifthRaw), support.X);
        Assert.Equal(radius, support.Y);

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, axis, (Fixed64)4, radius,
            new FixedRange(baseY, baseY),
            Vector2d.Backward, out support));
        Assert.Equal(Fixed64.FromRaw(-8L * fifthRaw), support.X);
        Assert.Equal(-radius, support.Y);
    }

    [Fact]
    public void Supports_PreserveFullDomainCenteredAxisArithmetic()
    {
        Vector3d center = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);
        Vector2d expected = new(Fixed64.FromRaw(-1L), Fixed64.Zero);

        Assert.True(FixedSlabProjection.TryGetCapsuleSupport(
            center, Vector3d.Right, Fixed64.MaxValue, Fixed64.Zero,
            WideSlab, Vector2d.Right, out Vector2d capsule));
        Assert.Equal(expected, capsule);

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(
            center, Vector3d.Right, Fixed64.MaxValue, Fixed64.Zero,
            WideSlab, Vector2d.Right, out Vector2d cylinder));
        Assert.Equal(expected, cylinder);

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            center, Vector3d.Right, Fixed64.MaxValue, Fixed64.Zero,
            WideSlab, Vector2d.Right, out Vector2d cone));
        Assert.Equal(Fixed64.FromRaw(-4611686018427387904L), cone.X);
        Assert.Equal(Fixed64.Zero, cone.Y);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AxisShapeSupport_HandlesLargeRadiusCrossSection(bool cylinder)
    {
        Vector3d center = Vector3d.Zero;
        FixedRange slab = new((Fixed64)60000, (Fixed64)60000);
        bool found = cylinder
            ? FixedSlabProjection.TryGetCylinderSupport(
                center, Vector3d.Right, Fixed64.One, (Fixed64)100000,
                slab, Vector2d.Forward, out Vector2d support)
            : FixedSlabProjection.TryGetCapsuleSupport(
                center, Vector3d.Right, Fixed64.One, (Fixed64)100000,
                slab, Vector2d.Forward, out support);

        Assert.True(found);
        Assert.Equal((Fixed64)80000, support.Y);
        Assert.True(support.X >= -Fixed64.One && support.X <= Fixed64.One);

        found = cylinder
            ? FixedSlabProjection.TryGetCylinderSupport(
                center, Vector3d.Right, Fixed64.One, (Fixed64)100000,
                slab, Vector2d.Right, out support)
            : FixedSlabProjection.TryGetCapsuleSupport(
                center, Vector3d.Right, Fixed64.One, (Fixed64)100000,
                slab, Vector2d.Right, out support);

        Assert.True(found);
        Assert.Equal(cylinder ? Fixed64.One : (Fixed64)80001, support.X);
    }

    [Fact]
    public void ConeSupport_HandlesLargeRadiusBaseCrossSection()
    {
        bool found = FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)2,
            (Fixed64)100000,
            new FixedRange((Fixed64)60000, (Fixed64)60000),
            Vector2d.Forward,
            out Vector2d support);

        Assert.True(found);
        Assert.Equal(new Vector2d(-Fixed64.One, (Fixed64)80000), support);
    }

    [Fact]
    public void Supports_RetainSlabBoundaryTangencyAndRejectEmptyClips()
    {
        FixedRange tangent = new(Fixed64.One, Fixed64.One);
        FixedRange miss = new((Fixed64)2, (Fixed64)3);

        Assert.True(FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, Vector3d.Right, Fixed64.Zero, Fixed64.One,
            tangent, Vector2d.Right, out Vector2d capsule));
        Assert.Equal(Vector2d.Zero, capsule);
        Assert.False(FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, Vector3d.Right, Fixed64.Zero, Fixed64.One,
            miss, Vector2d.Right, out _));

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            new FixedRange(Fixed64.One, Fixed64.One), Vector2d.Right, out Vector2d cylinder));
        Assert.Equal(new Vector2d(Fixed64.One, Fixed64.Zero), cylinder);
        Assert.False(FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            miss, Vector2d.Right, out _));

        Assert.True(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Up, (Fixed64)2, Fixed64.One,
            tangent, Vector2d.Right, out Vector2d cone));
        Assert.Equal(Vector2d.Zero, cone);
        Assert.False(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Up, (Fixed64)2, Fixed64.One,
            miss, Vector2d.Right, out _));
        Assert.False(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Up, (Fixed64)2, Fixed64.One,
            new FixedRange((Fixed64)(-3), (Fixed64)(-2)), Vector2d.Right, out _));
        Assert.False(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Down, (Fixed64)2, Fixed64.One,
            miss, Vector2d.Right, out _));
    }

    [Fact]
    public void CapsuleSupport_HandlesNegativeAxisSideCandidate()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Assert.True(FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero,
            new Vector3d(diagonal, -diagonal, Fixed64.Zero),
            (Fixed64)2,
            Fixed64.One,
            new FixedRange(Fixed64.Zero, Fixed64.Zero),
            Vector2d.Right,
            out Vector2d support));
        Assert.True(support.X > Fixed64.Zero);
    }

    [Fact]
    public void Supports_ReturnFalseWhenWinningPlanarPointIsUnrepresentable()
    {
        Vector3d center = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);

        Assert.False(FixedSlabProjection.TryGetCapsuleSupport(
            center, Vector3d.Up, Fixed64.One, Fixed64.One,
            WideSlab, Vector2d.Right, out Vector2d capsule));
        Assert.Equal(Vector2d.Zero, capsule);

        Assert.False(FixedSlabProjection.TryGetCylinderSupport(
            center, Vector3d.Up, Fixed64.One, Fixed64.One,
            WideSlab, Vector2d.Right, out Vector2d cylinder));
        Assert.Equal(Vector2d.Zero, cylinder);

        Assert.False(FixedSlabProjection.TryGetCylinderSupport(
            center, Vector3d.Up, Fixed64.One, Fixed64.One,
            new FixedRange(Fixed64.One, Fixed64.One), Vector2d.Right, out cylinder));
        Assert.Equal(Vector2d.Zero, cylinder);

        Assert.False(FixedSlabProjection.TryGetConeSupport(
            center, Vector3d.Up, (Fixed64)2, Fixed64.One,
            WideSlab, Vector2d.Right, out Vector2d cone));
        Assert.Equal(Vector2d.Zero, cone);
    }

    [Fact]
    public void ConeSupport_RejectsEmptyRotatedHorizontalClip()
    {
        Assert.False(FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Right, (Fixed64)2, Fixed64.One,
            new FixedRange((Fixed64)2, (Fixed64)3), Vector2d.Right, out Vector2d support));
        Assert.Equal(Vector2d.Zero, support);
    }

    [Fact]
    public void Supports_RejectInvalidShapeAndDirectionContracts()
    {
        FixedRange inverted = new(Fixed64.One, Fixed64.Zero, enforceOrder: false);

        Assert.Throws<ArgumentException>(() => FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One,
            WideSlab, Vector2d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, Vector3d.Up, -Fixed64.One, Fixed64.One,
            WideSlab, Vector2d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One,
            WideSlab, Vector2d.Right, out _));

        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One,
            WideSlab, Vector2d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One,
            WideSlab, Vector2d.Right, out _));
        Assert.Throws<ArgumentException>(() => FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            inverted, Vector2d.Right, out _));
        Assert.Throws<ArgumentException>(() => FixedSlabProjection.TryGetConeSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            WideSlab, Vector2d.Zero, out _));
    }
}
