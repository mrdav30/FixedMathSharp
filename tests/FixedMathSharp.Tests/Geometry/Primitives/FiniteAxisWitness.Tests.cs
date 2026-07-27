using FixedMathSharp.Bounds;
using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CenteredAxisEndpoint_MirroredScalarFacesRejectOnlyTheOutwardEndpoint(bool positiveFace)
    {
        Vector3d center = new(
            positiveFace ? Fixed64.MaxValue : Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredAxisEndpoint(
            center,
            Vector3d.Right,
            Fixed64.Two,
            positive: !positiveFace,
            out Vector3d inward));
        Assert.Equal(
            positiveFace ? Fixed64.MaxValue - Fixed64.One : Fixed64.MinValue + Fixed64.One,
            inward.X);

        Assert.False(FixedSegment.TryGetCenteredAxisEndpoint(
            center,
            Vector3d.Right,
            Fixed64.Two,
            positive: positiveFace,
            out Vector3d outward));
        Assert.Equal(Vector3d.Zero, outward);
    }

    [Fact]
    public void CenteredAxisEndpoint_PreservesOddRawLengthAndOrdinaryParity()
    {
        Vector3d oddCenter = new(Fixed64.FromRaw(1L), Fixed64.Zero, Fixed64.Zero);
        Fixed64 oddLength = Fixed64.FromRaw(1L);

        Assert.True(FixedSegment.TryGetCenteredAxisEndpoint(
            oddCenter, Vector3d.Right, oddLength, positive: false, out Vector3d negativeOdd));
        Assert.True(FixedSegment.TryGetCenteredAxisEndpoint(
            oddCenter, Vector3d.Right, oddLength, positive: true, out Vector3d positiveOdd));
        Assert.Equal(Fixed64.Zero, negativeOdd.X);
        Assert.Equal(Fixed64.FromRaw(2L), positiveOdd.X);

        Assert.True(FixedSegment.TryGetCenteredAxisEndpoint(
            new Vector3d(1, 2, 3),
            Vector3d.Up,
            (Fixed64)4,
            positive: false,
            out Vector3d ordinaryNegative));
        Assert.True(FixedSegment.TryGetCenteredAxisEndpoint(
            new Vector3d(1, 2, 3),
            Vector3d.Up,
            (Fixed64)4,
            positive: true,
            out Vector3d ordinaryPositive));
        Assert.Equal(new Vector3d(1, 0, 3), ordinaryNegative);
        Assert.Equal(new Vector3d(1, 4, 3), ordinaryPositive);

        Assert.True(FixedSegment2d.TryGetCenteredAxisEndpoint(
            new Vector2d(1, 2),
            Vector2d.Forward,
            (Fixed64)4,
            positive: true,
            out Vector2d planar));
        Assert.Equal(new Vector2d(1, 4), planar);
    }

    [Fact]
    public void CenteredCapsuleSupport_UsesStableAxisCenterForAxialTies()
    {
        Assert.True(FixedSegment.TryGetCenteredCapsuleSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d radial));
        Assert.Equal(Vector3d.Right, radial);

        Assert.True(FixedSegment.TryGetCenteredCapsuleSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Up,
            out Vector3d axial));
        Assert.Equal(new Vector3d(0, 2, 0), axial);

        Assert.True(FixedSegment.TryGetCenteredCapsuleSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Zero,
            out Vector3d zeroDirection));
        Assert.Equal(Vector3d.Zero, zeroDirection);

        Assert.True(FixedSegment2d.TryGetCenteredCapsuleSupport(
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Right,
            out Vector2d planar));
        Assert.Equal(Vector2d.Right, planar);
        Assert.True(FixedSegment2d.TryGetCenteredCapsuleSupport(
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Forward,
            out Vector2d planarAxial));
        Assert.Equal(new Vector2d(0, 2), planarAxial);
    }

    [Fact]
    public void CenteredSupport_NonCardinalExtremeRadiusRoundsOnlyTheFinalWitness()
    {
        var direction = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero);
        long expectedRaw = RoundRadicalRatio(
            (BigInteger)Fixed64.MaxValue.m_rawValue * Fixed64.One.m_rawValue,
            (BigInteger)Fixed64.One.m_rawValue * Fixed64.One.m_rawValue * 2);
        var expected = new Vector3d(
            Fixed64.FromRaw(expectedRaw),
            Fixed64.FromRaw(expectedRaw),
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredCapsuleSupport(
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.MaxValue,
            direction,
            out Vector3d capsule));
        Assert.Equal(expected, capsule);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One,
            Fixed64.MaxValue,
            direction,
            out Vector3d cylinder));
        Assert.Equal(
            new Vector3d(expected.X, expected.Y, -Fixed64.Half),
            cylinder);
    }

    [Fact]
    public void CenteredSupport_ExactRadicalCorrectsBothSidesOfNearestEvenMidpoint()
    {
        const long pellRadiusRaw = 723_573_111_879_672L;
        Fixed64 radius = Fixed64.FromRaw(pellRadiusRaw);
        var positiveDirection = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero);
        var negativeDirection = -positiveDirection;
        long positiveRaw = RoundRadicalRatio(
            (BigInteger)pellRadiusRaw * Fixed64.One.m_rawValue,
            (BigInteger)Fixed64.One.m_rawValue * Fixed64.One.m_rawValue * 2);

        Assert.True(FixedSegment.TryGetCenteredCapsuleSupport(
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.Zero,
            radius,
            positiveDirection,
            out Vector3d positive));
        Assert.Equal(Fixed64.FromRaw(positiveRaw), positive.X);
        Assert.True(FixedSegment.TryGetCenteredCapsuleSupport(
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.Zero,
            radius,
            negativeDirection,
            out Vector3d negative));
        Assert.Equal(Fixed64.FromRaw(-positiveRaw), negative.X);
    }

    [Fact]
    public void CenteredSupport_ExactRadicalKeepsMirroredScalarFaceResultsRepresentable()
    {
        const long pellRadiusRaw = 723_573_111_879_672L;
        Fixed64 radius = Fixed64.FromRaw(pellRadiusRaw);
        var direction = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero);
        long radialRaw = RoundRadicalRatio(
            (BigInteger)pellRadiusRaw * Fixed64.One.m_rawValue,
            (BigInteger)Fixed64.One.m_rawValue * Fixed64.One.m_rawValue * 2);

        Assert.True(FixedSegment.TryGetCenteredCapsuleSupport(
            new Vector3d(Fixed64.FromRaw(long.MaxValue - radialRaw), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Forward,
            Fixed64.Zero,
            radius,
            direction,
            out Vector3d maximum));
        Assert.Equal(Fixed64.MaxValue, maximum.X);

        Assert.True(FixedSegment.TryGetCenteredCapsuleSupport(
            new Vector3d(Fixed64.FromRaw(long.MinValue + radialRaw), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Forward,
            Fixed64.Zero,
            radius,
            -direction,
            out Vector3d minimum));
        Assert.Equal(Fixed64.MinValue, minimum.X);
    }

    [Fact]
    public void CenteredSupport_ExactHalfBaseAndDegenerateConeRetainStablePolicies()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.FromRaw(1L),
            Fixed64.One,
            Vector3d.Up,
            out Vector3d halfBase));
        Assert.Equal(Fixed64.Zero, halfBase.X);

        Assert.True(FixedSegment.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Zero,
            Vector3d.Right + Vector3d.Up,
            out Vector3d degenerateCone));
        Assert.Equal(Vector3d.Up, degenerateCone);
    }

    [Fact]
    public void CenteredCylinderSupportAnchor_UsesCapCenterForParallelDirection()
    {
        FixedPointAnchor anchor =
            FixedSegment.GetCenteredFiniteCylinderSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.Two,
                Fixed64.One,
                Vector3d.Up);

        Assert.Equal(Vector3d.Up, anchor.LocalPoint);
        Assert.Equal(Vector3d.Zero, anchor.LocalDisplacement);
        Assert.True(anchor.TryGetPoint(out Vector3d support));
        Assert.Equal(Vector3d.Up, support);
    }

    [Fact]
    public void CenteredCylinderSupportAnchor_ValidatesRigidFrameAndGeometry()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.GetCenteredFiniteCylinderSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Up));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.GetCenteredFiniteCylinderSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Up));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.GetCenteredFiniteCylinderSupportAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Up));
    }

    [Fact]
    public void CenteredSupportOffsets_PreserveFeaturesWhenWorldWitnessesOverflow()
    {
        Vector3d center = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);

        Assert.False(FixedSegment.TryGetCenteredCapsuleSupport(
            center,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _));
        Assert.True(FixedSegment.TryGetCenteredCapsuleSupportOffset(
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d capsuleOffset));
        Assert.Equal(Vector3d.Right, capsuleOffset);

        Assert.False(FixedSegment.TryGetCenteredFiniteCylinderSupport(
            center,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderSupportOffset(
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d cylinderOffset));
        Assert.Equal(new Vector3d(1, -1, 0), cylinderOffset);

        Assert.False(FixedSegment.TryGetCenteredFiniteConeSupport(
            center,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _));
        Assert.True(FixedSegment.TryGetCenteredFiniteConeSupportOffset(
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d coneOffset));
        Assert.Equal(new Vector3d(1, -1, 0), coneOffset);
    }

    [Fact]
    public void CenteredCapsuleSurfaceOffset_RemainsAvailablePastScalarFace()
    {
        Vector3d center = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);

        Assert.False(FixedSegment.TryGetSurfacePointOnCenteredCapsule(
            center,
            center,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _));
        Assert.True(FixedSegment.TryGetSurfaceOffsetOnCenteredCapsule(
            center,
            center,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d offset));
        Assert.Equal(new Vector3d(1, 0, 0), offset);

        Vector3d rotatedNormal = new(
            Fixed64.FromFraction(1, 10),
            Fixed64.Zero,
            -FixedMath.Sqrt(Fixed64.One - Fixed64.FromFraction(1, 100)));
        rotatedNormal = rotatedNormal.Normalized;
        Assert.True(FixedSegment.TryGetSurfaceOffsetOnCenteredCapsule(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            rotatedNormal,
            out Vector3d rotatedOffset));
        Assert.Equal(rotatedNormal * Fixed64.Half, rotatedOffset);

        Assert.False(FixedSegment.TryGetSurfaceOffsetOnCenteredCapsule(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector3d.Up,
            out Vector3d unrepresentableOffset));
        Assert.Equal(Vector3d.Zero, unrepresentableOffset);

        Vector2d center2d = new(Fixed64.MaxValue, Fixed64.Zero);
        Assert.False(FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            center2d,
            center2d,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Right,
            out _));
        Assert.True(FixedSegment2d.TryGetSurfaceOffsetOnCenteredCapsule(
            center2d,
            center2d,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Right,
            out Vector2d offset2d));
        Assert.Equal(Vector2d.Right, offset2d);

        Assert.False(FixedSegment2d.TryGetSurfaceOffsetOnCenteredCapsule(
            Vector2d.Forward,
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector2d.Forward,
            out Vector2d unrepresentableOffset2d));
        Assert.Equal(Vector2d.Zero, unrepresentableOffset2d);
    }

    [Fact]
    public void CenteredFiniteCylinderSupport_UsesCapCenterWhenRadialProjectionIsZero()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d radial));
        Assert.Equal(new Vector3d(1, -1, 0), radial);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderSupport(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Up,
            out Vector3d axial));
        Assert.Equal(new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.Zero), axial);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Zero,
            out Vector3d zeroDirection));
        Assert.Equal(new Vector3d(0, -1, 0), zeroDirection);
    }

    [Fact]
    public void CenteredFiniteConeSupport_SelectsBaseOnEqualProjectionAndZeroDirection()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(2, 1, 0),
            out Vector3d tied));
        Assert.Equal(new Vector3d(1, -1, 0), tied);

        Assert.True(FixedSegment.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Up,
            out Vector3d apex));
        Assert.Equal(new Vector3d(0, 1, 0), apex);

        Assert.True(FixedSegment.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Zero,
            out Vector3d zeroDirection));
        Assert.Equal(new Vector3d(0, -1, 0), zeroDirection);
    }

    [Fact]
    public void CenteredFiniteAxisWitnesses_RejectInvalidGeometry()
    {
        Assert.Throws<ArgumentException>(() => FixedSegment.TryGetCenteredAxisEndpoint(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, true, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredAxisEndpoint(
            Vector3d.Zero, Vector3d.Up, -Fixed64.One, true, out _));
        Assert.Throws<ArgumentException>(() => FixedSegment2d.TryGetCenteredAxisEndpoint(
            Vector2d.Zero, Vector2d.Zero, Fixed64.One, true, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment2d.TryGetCenteredCapsuleSupport(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, -Fixed64.One, Vector2d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredCapsuleSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentException>(() => FixedSegment.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentException>(() => FixedSegment.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredCapsuleSupportOffset(
            Vector3d.Up, Fixed64.One, -Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentException>(() => FixedSegment.TryGetCenteredFiniteCylinderSupportOffset(
            Vector3d.Zero, Fixed64.One, Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredFiniteCylinderSupportOffset(
            Vector3d.Up, Fixed64.Zero, Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredFiniteCylinderSupportOffset(
            Vector3d.Up, Fixed64.One, -Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentException>(() => FixedSegment.TryGetCenteredFiniteConeSupportOffset(
            Vector3d.Zero, Fixed64.One, Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredFiniteConeSupportOffset(
            Vector3d.Up, Fixed64.Zero, Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetCenteredFiniteConeSupportOffset(
            Vector3d.Up, Fixed64.One, -Fixed64.One, Vector3d.Right, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedSegment.TryGetSurfaceOffsetOnCenteredCapsule(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            -Fixed64.One,
            Vector3d.Right,
            out _));
        Assert.Throws<ArgumentException>(() => FixedSegment.TryGetSurfaceOffsetOnCenteredCapsule(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Vector3d.Zero,
            out _));
    }

    [Fact]
    public void CenteredFiniteAxisWitnesses_FailAtomicallyWhenAnyCoordinateIsUnrepresentable()
    {
        Vector3d center = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);

        Assert.False(FixedSegment.TryGetCenteredCapsuleSupport(
            center, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out Vector3d capsule));
        Assert.Equal(Vector3d.Zero, capsule);
        Assert.False(FixedSegment.TryGetCenteredFiniteCylinderSupport(
            center, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out Vector3d cylinder));
        Assert.Equal(Vector3d.Zero, cylinder);
        Assert.False(FixedSegment.TryGetCenteredFiniteConeSupport(
            center, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out Vector3d cone));
        Assert.Equal(Vector3d.Zero, cone);
        Assert.False(FixedSegment2d.TryGetCenteredCapsuleSupport(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Right,
            out Vector2d planar));
        Assert.Equal(Vector2d.Zero, planar);
        Assert.False(FixedSegment.TryGetCenteredCapsuleSupport(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Left,
            out capsule));
        Assert.Equal(Vector3d.Zero, capsule);
    }

    [Fact]
    public void CenteredFiniteAxisWitnesses_AllocateZeroAfterWarmup()
    {
        _ = FixedSegment.TryGetCenteredAxisEndpoint(
            Vector3d.Zero, Vector3d.Up, Fixed64.Two, true, out _);
        _ = FixedSegment.TryGetCenteredCapsuleSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out _);
        _ = FixedSegment.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out _);
        _ = FixedSegment.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out _);

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++)
        {
            _ = FixedSegment.TryGetCenteredAxisEndpoint(
                Vector3d.Zero, Vector3d.Up, Fixed64.Two, true, out _);
            _ = FixedSegment.TryGetCenteredCapsuleSupport(
                Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out _);
            _ = FixedSegment.TryGetCenteredFiniteCylinderSupport(
                Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out _);
            _ = FixedSegment.TryGetCenteredFiniteConeSupport(
                Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One, Vector3d.Right, out _);
        }

        Assert.Equal(0L, GC.GetAllocatedBytesForCurrentThread() - before);
    }

    private static long RoundRadicalRatio(BigInteger numerator, BigInteger radicand)
    {
        bool negative = numerator.Sign < 0;
        numerator = BigInteger.Abs(numerator);
        BigInteger squaredNumerator = numerator * numerator;
        BigInteger low = BigInteger.Zero;
        BigInteger high = numerator + BigInteger.One;
        while (low + BigInteger.One < high)
        {
            BigInteger middle = (low + high) >> 1;
            if (middle * middle * radicand <= squaredNumerator)
                low = middle;
            else
                high = middle;
        }

        BigInteger doubledMidpoint = (low << 1) + BigInteger.One;
        int midpointComparison = (squaredNumerator << 2).CompareTo(
            doubledMidpoint * doubledMidpoint * radicand);
        if (midpointComparison > 0 || (midpointComparison == 0 && !low.IsEven))
            low++;
        return (long)(negative ? -low : low);
    }
}
