using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void CenteredCapsule_RepresentableAxisMatchesEndpointContract()
    {
        Vector3d axisDirection = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        Fixed64 endpointOffset = (Fixed64)2;
        Fixed64 axisLength = (Fixed64)4;
        var center = new Vector3d((Fixed64)3, (Fixed64)(-2), Fixed64.One);
        var capsuleAxis = new FixedSegment(
            center - axisDirection * endpointOffset,
            center + axisDirection * endpointOffset);
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-2), (Fixed64)(-2), Fixed64.One),
            new Vector3d((Fixed64)8, (Fixed64)(-2), Fixed64.One));

        bool endpointHit = query.TryGetCapsuleIntersectionInterval(
            capsuleAxis,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 endpointEntry,
            out Fixed64 endpointExit,
            out bool endpointStartContained,
            out bool endpointEndContainedStrict);
        bool centeredHit = query.TryGetCapsuleIntersectionInterval(
            center,
            axisDirection,
            axisLength,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit,
            out bool centeredStartContained,
            out bool centeredEndContainedStrict);

        Assert.Equal(endpointHit, centeredHit);
        Assert.Equal(endpointEntry, centeredEntry);
        Assert.Equal(endpointExit, centeredExit);
        Assert.Equal(endpointStartContained, centeredStartContained);
        Assert.Equal(endpointEndContainedStrict, centeredEndContainedStrict);
    }

    [Fact]
    public void CenteredCapsule2d_RepresentableAxisMatchesEndpointContract()
    {
        Vector2d axisDirection = new Vector2d(Fixed64.One, Fixed64.One).Normalized;
        Fixed64 endpointOffset = (Fixed64)2;
        Fixed64 axisLength = (Fixed64)4;
        var center = new Vector2d((Fixed64)3, (Fixed64)(-2));
        var capsuleAxis = new FixedSegment2d(
            center - axisDirection * endpointOffset,
            center + axisDirection * endpointOffset);
        var query = new FixedSegment2d(
            new Vector2d((Fixed64)(-2), (Fixed64)(-2)),
            new Vector2d((Fixed64)8, (Fixed64)(-2)));

        bool endpointHit = query.TryGetCapsuleIntersectionInterval(
            capsuleAxis,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 endpointEntry,
            out Fixed64 endpointExit,
            out bool endpointStartContained,
            out bool endpointEndContainedStrict);
        bool centeredHit = query.TryGetCapsuleIntersectionInterval(
            center,
            axisDirection,
            axisLength,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit,
            out bool centeredStartContained,
            out bool centeredEndContainedStrict);

        Assert.Equal(endpointHit, centeredHit);
        Assert.Equal(endpointEntry, centeredEntry);
        Assert.Equal(endpointExit, centeredExit);
        Assert.Equal(endpointStartContained, centeredStartContained);
        Assert.Equal(endpointEndContainedStrict, centeredEndContainedStrict);
    }

    [Fact]
    public void CenteredCapsule_ZeroLengthMatchesCenteredSphereContract()
    {
        var center = new Vector3d((Fixed64)2, (Fixed64)(-1), (Fixed64)3);
        var query = new FixedSegment(center + Vector3d.Left * (Fixed64)2, center + Vector3d.Right * (Fixed64)2);
        var sphereAxis = new FixedSegment(center, center);

        bool sphereHit = query.TryGetCapsuleIntersectionInterval(
            sphereAxis,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 sphereEntry,
            out Fixed64 sphereExit,
            out bool sphereStartContained,
            out bool sphereEndContainedStrict);
        bool centeredHit = query.TryGetCapsuleIntersectionInterval(
            center,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit,
            out bool centeredStartContained,
            out bool centeredEndContainedStrict);

        Assert.Equal(sphereHit, centeredHit);
        Assert.Equal(sphereEntry, centeredEntry);
        Assert.Equal(sphereExit, centeredExit);
        Assert.Equal(sphereStartContained, centeredStartContained);
        Assert.Equal(sphereEndContainedStrict, centeredEndContainedStrict);
    }

    [Fact]
    public void CenteredCapsule2d_ZeroLengthMatchesCenteredCircleContract()
    {
        var center = new Vector2d((Fixed64)2, (Fixed64)(-1));
        var query = new FixedSegment2d(center + Vector2d.Left * (Fixed64)2, center + Vector2d.Right * (Fixed64)2);
        var circleAxis = new FixedSegment2d(center, center);

        bool circleHit = query.TryGetCapsuleIntersectionInterval(
            circleAxis,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 circleEntry,
            out Fixed64 circleExit,
            out bool circleStartContained,
            out bool circleEndContainedStrict);
        bool centeredHit = query.TryGetCapsuleIntersectionInterval(
            center,
            Vector2d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit,
            out bool centeredStartContained,
            out bool centeredEndContainedStrict);

        Assert.Equal(circleHit, centeredHit);
        Assert.Equal(circleEntry, centeredEntry);
        Assert.Equal(circleExit, centeredExit);
        Assert.Equal(circleStartContained, centeredStartContained);
        Assert.Equal(circleEndContainedStrict, centeredEndContainedStrict);
    }

    [Fact]
    public void CenteredCapsule_RejectsInvalidAuthoredParameters()
    {
        var query = new FixedSegment(Vector3d.Zero, Vector3d.One);

        Assert.Throws<ArgumentException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.One, Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, -Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            -Fixed64.One,
            out _,
            out _));
    }

    [Fact]
    public void CenteredCapsule_FullDomainConceptualAxisDoesNotBendAtScalarBoundary()
    {
        Vector3d axisDirection = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        var center = new Vector3d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        var point = new Vector3d(Fixed64.MaxValue, (Fixed64)7, Fixed64.Zero);
        var query = new FixedSegment(point, point);
        var narrowedAxis = new FixedSegment(
            center - axisDirection * (Fixed64)10,
            center + axisDirection * (Fixed64)10);

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            narrowedAxis,
            Fixed64.One,
            Fixed64.Zero,
            out _,
            out _,
            out bool narrowedStartContained,
            out _));
        Assert.True(narrowedStartContained);

        Assert.False(query.TryGetCapsuleIntersectionInterval(
            center,
            axisDirection,
            (Fixed64)20,
            Fixed64.One,
            Fixed64.Zero,
            out _,
            out _,
            out bool startContained,
            out bool endContainedStrict));
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void CenteredCapsule2d_FullDomainConceptualAxisDoesNotBendAtScalarBoundary()
    {
        Vector2d axisDirection = new Vector2d(Fixed64.One, Fixed64.One).Normalized;
        var center = new Vector2d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero);
        var point = new Vector2d(Fixed64.MaxValue, (Fixed64)7);
        var query = new FixedSegment2d(point, point);
        var narrowedAxis = new FixedSegment2d(
            center - axisDirection * (Fixed64)10,
            center + axisDirection * (Fixed64)10);

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            narrowedAxis,
            Fixed64.One,
            Fixed64.Zero,
            out _,
            out _,
            out bool narrowedStartContained,
            out _));
        Assert.True(narrowedStartContained);

        Assert.False(query.TryGetCapsuleIntersectionInterval(
            center,
            axisDirection,
            (Fixed64)20,
            Fixed64.One,
            Fixed64.Zero,
            out _,
            out _,
            out bool startContained,
            out bool endContainedStrict));
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void CenteredAxisDirection_FullDomainSideFeatureDoesNotUseNarrowedEndpoints()
    {
        Vector3d axisDirection = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        var center = new Vector3d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        var point = new Vector3d(Fixed64.MaxValue, (Fixed64)7, Fixed64.Zero);

        Vector3d direction = FixedSegment.GetDirectionFromCenteredAxis(
            point,
            center,
            axisDirection,
            (Fixed64)20);

        Assert.True(direction.IsNormalized());
        Assert.True(direction.X < Fixed64.Zero);
        Assert.True(direction.Y > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, direction.Z);
    }

    [Fact]
    public void CenteredAxisDirection2d_FullDomainCapFeatureDoesNotUseNarrowedEndpoints()
    {
        Vector2d axisDirection = new Vector2d(Fixed64.One, Fixed64.One).Normalized;
        var center = new Vector2d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero);
        var point = new Vector2d(Fixed64.MaxValue, (Fixed64)20);

        Vector2d direction = FixedSegment2d.GetDirectionFromCenteredAxis(
            point,
            center,
            axisDirection,
            (Fixed64)20);

        Assert.True(direction.IsNormalized());
        Assert.True(direction.X < Fixed64.Zero);
        Assert.True(direction.Y > Fixed64.Zero);
    }

    [Fact]
    public void CenteredAxisDirection_PointOnAxisReturnsZero()
    {
        Assert.Equal(Vector3d.Zero, FixedSegment.GetDirectionFromCenteredAxis(
                Vector3d.Up,
                Vector3d.Zero,
                Vector3d.Up,
                (Fixed64)4));
        Assert.Equal(Vector2d.Zero, FixedSegment2d.GetDirectionFromCenteredAxis(
                Vector2d.Forward,
                Vector2d.Zero,
                Vector2d.Forward,
                (Fixed64)4));
    }

    [Fact]
    public void CenteredCapsuleSurfacePoint_FusesScalarTranslationBeforeFinalNarrowing()
    {
        Vector2d axis = new Vector2d(Fixed64.One, Fixed64.One).Normalized;
        var moderateCenter = new Vector2d((Fixed64)(-5), Fixed64.Zero);
        var moderatePoint = new Vector2d((Fixed64)(-1), (Fixed64)8);
        var extremeCenter = new Vector2d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero);
        var extremePoint = new Vector2d(Fixed64.MaxValue - Fixed64.One, (Fixed64)8);
        Vector2d moderateNormal = FixedSegment2d.GetDirectionFromCenteredAxis(
            moderatePoint, moderateCenter, axis, (Fixed64)20);
        Vector2d extremeNormal = FixedSegment2d.GetDirectionFromCenteredAxis(
            extremePoint, extremeCenter, axis, (Fixed64)20);

        Assert.False(FixedSegment2d.ContainsPointInCenteredCapsule(
            extremePoint, extremeCenter, axis, (Fixed64)20, (Fixed64)2));
        Assert.True(FixedSegment2d.ContainsPointInCenteredCapsule(
            extremePoint, extremeCenter, axis, (Fixed64)20, (Fixed64)2, Fixed64.One));
        Assert.Equal(moderateNormal, extremeNormal);
        Assert.True(FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            moderatePoint,
            moderateCenter,
            axis,
            (Fixed64)20,
            (Fixed64)2,
            moderateNormal,
            out Vector2d moderateSurface));
        Assert.True(FixedSegment2d.TryGetSurfacePointOnCenteredCapsule(
            extremePoint,
            extremeCenter,
            axis,
            (Fixed64)20,
            (Fixed64)2,
            extremeNormal,
            out Vector2d extremeSurface));
        Assert.True(Fixed64.TryAdd(Fixed64.MaxValue, moderateSurface.X, out Fixed64 translatedX));
        Assert.Equal(new Vector2d(translatedX, moderateSurface.Y), extremeSurface);

        Vector3d axis3D = new(axis.X, axis.Y, Fixed64.Zero);
        Vector3d moderateNormal3D = FixedSegment.GetDirectionFromCenteredAxis(
            new Vector3d(moderatePoint.X, moderatePoint.Y, Fixed64.Zero),
            new Vector3d(moderateCenter.X, moderateCenter.Y, Fixed64.Zero),
            axis3D,
            (Fixed64)20);
        Assert.True(FixedSegment.TryGetSurfacePointOnCenteredCapsule(
            new Vector3d(extremePoint.X, extremePoint.Y, Fixed64.Zero),
            new Vector3d(extremeCenter.X, extremeCenter.Y, Fixed64.Zero),
            axis3D,
            (Fixed64)20,
            (Fixed64)2,
            new Vector3d(extremeNormal.X, extremeNormal.Y, Fixed64.Zero),
            out Vector3d extremeSurface3D));
        Assert.Equal(new Vector3d(translatedX, moderateSurface.Y, Fixed64.Zero), extremeSurface3D);
        Assert.Equal(new Vector3d(moderateNormal.X, moderateNormal.Y, Fixed64.Zero), moderateNormal3D);

        Assert.True(FixedSegment2d.TryGetDistanceToCenteredCapsule(
            moderatePoint, moderateCenter, axis, (Fixed64)20, (Fixed64)2, out Fixed64 moderateDistance));
        Assert.True(FixedSegment2d.TryGetDistanceToCenteredCapsule(
            extremePoint, extremeCenter, axis, (Fixed64)20, (Fixed64)2, out Fixed64 extremeDistance));
        Assert.Equal(moderateDistance, extremeDistance);
        Assert.True(moderateDistance > Fixed64.Zero);
        Assert.True(FixedSegment.TryGetDistanceToCenteredCapsule(
            new Vector3d(extremePoint.X, extremePoint.Y, Fixed64.Zero),
            new Vector3d(extremeCenter.X, extremeCenter.Y, Fixed64.Zero),
            axis3D,
            (Fixed64)20,
            (Fixed64)2,
            out Fixed64 extremeDistance3D));
        Assert.Equal(extremeDistance, extremeDistance3D);
    }

    [Fact]
    public void CenteredCapsuleDistance_LargeObliqueResidualMatchesExactIntegerOracle()
    {
        Vector2d axis = new Vector2d(Fixed64.One, (Fixed64)2).Normalized;
        var point = new Vector2d((Fixed64)1_500_000_000, (Fixed64)(-500_000_000));
        Fixed64 radius = Fixed64.One;

        Assert.True(FixedSegment2d.TryGetDistanceToCenteredCapsule(
            point,
            Vector2d.Zero,
            axis,
            (Fixed64)2_000_000_000,
            radius,
            out Fixed64 actual));

        BigInteger x = point.X.m_rawValue;
        BigInteger y = point.Y.m_rawValue;
        BigInteger dx = axis.X.m_rawValue;
        BigInteger dy = axis.Y.m_rawValue;
        BigInteger q = dx * dx + dy * dy;
        BigInteger projection = x * dx + y * dy;
        BigInteger residualX = q * x - dx * projection;
        BigInteger residualY = q * y - dy * projection;
        BigInteger squaredResidual = residualX * residualX + residualY * residualY;
        BigInteger coreFloor = IntegerSquareRoot(squaredResidual) / q;
        BigInteger gapFloor = coreFloor - radius.m_rawValue;
        BigInteger midpointTwice = coreFloor * 2 + 1;
        int midpointComparison = (squaredResidual * 4).CompareTo(q * q * midpointTwice * midpointTwice);
        if (midpointComparison > 0 || (midpointComparison == 0 && !gapFloor.IsEven))
            gapFloor++;

        Assert.Equal((long)gapFloor, actual.m_rawValue);
    }

    [Fact]
    public void CenteredCapsuleDistance_RoundsFinalGapAcrossOddRadiusMidpoints()
    {
        BigInteger evenGap = BigInteger.One << 31;
        BigInteger doubledCoreMidpoint = 2 * (BigInteger.One + evenGap) + 1;
        BigInteger exactMidpointSquare = doubledCoreMidpoint * doubledCoreMidpoint;
        Signed192 denominator = ToPositiveSigned192(2);
        Fixed64 oddRawRadius = Fixed64.FromRaw(1);

        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(exactMidpointSquare),
            denominator,
            oddRawRadius,
            out Fixed64 exactTie));
        Assert.Equal((long)evenGap, exactTie.m_rawValue);

        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(exactMidpointSquare - BigInteger.One),
            denominator,
            oddRawRadius,
            out Fixed64 justBelowTie));
        Assert.Equal((long)evenGap, justBelowTie.m_rawValue);

        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(exactMidpointSquare + BigInteger.One),
            denominator,
            oddRawRadius,
            out Fixed64 justAboveTie));
        Assert.Equal((long)evenGap + 1L, justAboveTie.m_rawValue);

        BigInteger oddGap = evenGap + BigInteger.One;
        BigInteger oddGapMidpoint = 2 * (BigInteger.One + oddGap) + 1;
        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(oddGapMidpoint * oddGapMidpoint),
            denominator,
            oddRawRadius,
            out Fixed64 oddTie));
        Assert.Equal((long)oddGap + 1L, oddTie.m_rawValue);

        BigInteger evenRadius = 2;
        BigInteger evenRadiusGap = 4;
        BigInteger evenRadiusMidpoint = 2 * (evenRadius + evenRadiusGap) + 1;
        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(evenRadiusMidpoint * evenRadiusMidpoint),
            denominator,
            Fixed64.FromRaw((long)evenRadius),
            out Fixed64 evenRadiusTie));
        Assert.Equal((long)evenRadiusGap, evenRadiusTie.m_rawValue);
    }

    [Fact]
    public void CenteredCapsuleDistance_HandlesInteriorSubRawAndPositiveLimitBoundaries()
    {
        Signed192 denominator = ToPositiveSigned192(3);
        Fixed64 radius = Fixed64.FromRaw(5);
        BigInteger axisSurface = 15;

        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(axisSurface * axisSurface),
            denominator,
            radius,
            out Fixed64 onSurface));
        Assert.Equal(Fixed64.Zero, onSurface);

        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576((axisSurface - 1) * (axisSurface - 1)),
            denominator,
            radius,
            out Fixed64 inside));
        Assert.Equal(Fixed64.Zero, inside);

        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576((axisSurface + 1) * (axisSurface + 1)),
            denominator,
            radius,
            out Fixed64 positiveSubRaw));
        Assert.Equal(Fixed64.Zero, positiveSubRaw);

        BigInteger rawScale = BigInteger.One << FixedMath.SHIFT_AMOUNT_I;
        BigInteger largeRadiusAxis = 3 * rawScale;
        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(largeRadiusAxis * largeRadiusAxis + BigInteger.One),
            denominator,
            Fixed64.One,
            out Fixed64 restoringSubRaw));
        Assert.Equal(Fixed64.Zero, restoringSubRaw);

        BigInteger maximum = long.MaxValue;
        BigInteger doubledMaximumMidpoint = 2 * maximum + 1;
        BigInteger maximumMidpointSquare = doubledMaximumMidpoint * doubledMaximumMidpoint;
        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(maximumMidpointSquare - BigInteger.One),
            ToPositiveSigned192(2),
            Fixed64.Zero,
            out Fixed64 justBelowPositiveLimit));
        Assert.Equal(Fixed64.MaxValue, justBelowPositiveLimit);

        Assert.False(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(maximumMidpointSquare),
            ToPositiveSigned192(2),
            Fixed64.Zero,
            out Fixed64 midpointOverflow));
        Assert.Equal(Fixed64.MaxValue, midpointOverflow);
        Assert.False(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(maximumMidpointSquare + BigInteger.One),
            ToPositiveSigned192(2),
            Fixed64.Zero,
            out Fixed64 aboveMidpointOverflow));
        Assert.Equal(Fixed64.MaxValue, aboveMidpointOverflow);

        BigInteger exactMaximumAxisDistance = 2 * maximum;
        Assert.True(WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            ToPositiveSigned576(exactMaximumAxisDistance * exactMaximumAxisDistance),
            ToPositiveSigned192(2),
            Fixed64.Zero,
            out Fixed64 representableMaximum));
        Assert.Equal(Fixed64.MaxValue, representableMaximum);

        var farPoint2D = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
        var farCenter2D = new Vector2d(Fixed64.MinValue, Fixed64.Zero);
        Assert.False(FixedSegment2d.TryGetDistanceToCenteredCapsule(
            farPoint2D,
            farCenter2D,
            Vector2d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 saturatedDistance));
        Assert.Equal(Fixed64.MaxValue, saturatedDistance);
        Assert.Equal(
            Fixed64.MaxValue,
            FixedSegment2d.GetDistanceToCenteredCapsule(
                farPoint2D,
                farCenter2D,
                Vector2d.Forward,
                Fixed64.Zero,
                Fixed64.Zero));

        Assert.Equal(
            Fixed64.MaxValue,
            FixedSegment.GetDistanceToCenteredCapsule(
                new Vector3d(farPoint2D.X, farPoint2D.Y, Fixed64.Zero),
                new Vector3d(farCenter2D.X, farCenter2D.Y, Fixed64.Zero),
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.Zero));
    }

    [Fact]
    public void CenteredCapsulePointContainment_RemainsExactAtScalarBoundary()
    {
        Vector3d axis3D = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        var center3D = new Vector3d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        var point3D = new Vector3d(Fixed64.MaxValue, (Fixed64)7, Fixed64.Zero);
        Assert.False(FixedSegment.ContainsPointInCenteredCapsule(
            point3D,
            center3D,
            axis3D,
            (Fixed64)20,
            Fixed64.One,
            Fixed64.Zero));

        Vector2d axis2D = new Vector2d(Fixed64.One, Fixed64.One).Normalized;
        var center2D = new Vector2d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero);
        var point2D = new Vector2d(Fixed64.MaxValue, (Fixed64)7);
        Assert.False(FixedSegment2d.ContainsPointInCenteredCapsule(
            point2D,
            center2D,
            axis2D,
            (Fixed64)20,
            Fixed64.One,
            Fixed64.Zero));

        Assert.True(FixedSegment2d.ContainsPointInCenteredCapsule(
            new Vector2d(Fixed64.One, Fixed64.Zero),
            Vector2d.Zero,
            Vector2d.Forward,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.Zero));
    }

    [Fact]
    public void Capsule_AxialCrossingMergesOuterEndpointIntervals()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, (Fixed64)(-1), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.Half,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 4), entry);
        Assert.Equal(Fixed64.FromFraction(3, 4), exit);
    }

    [Fact]
    public void Capsule2d_CapOnlyIntersectionDoesNotRequireASideProjection()
    {
        var query = new FixedSegment2d(
            new Vector2d((Fixed64)(-2), (Fixed64)(-2)),
            new Vector2d((Fixed64)2, (Fixed64)(-2)));
        var axis = new FixedSegment2d(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One));

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(entry, exit);
    }

    [Fact]
    public void Capsule_AdvancedClassification_ReportsBoundaryStartAsContained()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            UnitCylinderAxis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out _,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.True(startContained);
        Assert.False(endContainedStrict);
    }

    [Fact]
    public void Capsule_AdvancedClassification_KeepsWideStrictInteriorEndExact()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)1_500_000_000, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)1_300_000_000, Fixed64.Zero, Fixed64.Zero));

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            UnitCylinderAxis,
            (Fixed64)1_400_000_000,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Half, entry);
        Assert.Equal(Fixed64.One, exit);
        Assert.False(startContained);
        Assert.True(endContainedStrict);
    }

    [Fact]
    public void Capsule_AdvancedClassification_TreatsZeroAxisAsExactSphere()
    {
        var query = new FixedSegment(Vector3d.Right, Vector3d.Zero);
        var axis = new FixedSegment(Vector3d.Zero, Vector3d.Zero);

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Zero,
            out _,
            out _,
            out bool startContained,
            out bool endContainedStrict));
        Assert.True(startContained);
        Assert.True(endContainedStrict);
    }

    [Fact]
    public void Capsule_DegenerateAxisReducesToSphere()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        var axis = new FixedSegment(Vector3d.Zero, Vector3d.Zero);

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 4), entry);
        Assert.Equal(Fixed64.FromFraction(3, 4), exit);
    }

    [Fact]
    public void Capsule2d_FullDomainAxisMatchesEmbedded3dContract()
    {
        var query = new FixedSegment2d(
            new Vector2d(Fixed64.Zero, (Fixed64)(-2)),
            new Vector2d(Fixed64.Zero, (Fixed64)2));
        var axis = new FixedSegment2d(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero));

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 4), entry);
        Assert.Equal(Fixed64.FromFraction(3, 4), exit);
    }

    [Fact]
    public void Capsule2d_DegenerateAxisReducesToCircle()
    {
        var query = new FixedSegment2d(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero));
        var axis = new FixedSegment2d(Vector2d.Zero, Vector2d.Zero);

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.Equal(Fixed64.FromFraction(1, 4), entry);
        Assert.Equal(Fixed64.FromFraction(3, 4), exit);
    }

    [Fact]
    public void Capsule2d_AdvancedClassification_MatchesExactEndpointContract()
    {
        Fixed64 radius = Fixed64.Half;
        var query = new FixedSegment2d(
            new Vector2d(radius + Fixed64.FromRaw(1), Fixed64.Zero),
            new Vector2d((Fixed64)(-3), Fixed64.Zero));
        var axis = new FixedSegment2d(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One));

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            axis,
            radius,
            Fixed64.Zero,
            out Fixed64 entry,
            out _,
            out bool startContained,
            out bool endContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.False(startContained);
        Assert.False(endContainedStrict);
    }

    private static BigInteger IntegerSquareRoot(BigInteger value)
    {
        if (value <= BigInteger.Zero)
            return BigInteger.Zero;

        BigInteger estimate = BigInteger.One << ((GetBitLength(value) + 1) / 2);
        while (true)
        {
            BigInteger next = (estimate + value / estimate) >> 1;
            if (next >= estimate)
                return estimate;
            estimate = next;
        }
    }

    private static int GetBitLength(BigInteger value)
    {
        int bits = 0;
        while (value > BigInteger.Zero)
        {
            value >>= 1;
            bits++;
        }
        return bits;
    }

    private static Signed192 ToPositiveSigned192(BigInteger value)
    {
        ulong low = (ulong)(value & ulong.MaxValue);
        ulong middle = (ulong)((value >> 64) & ulong.MaxValue);
        ulong high = (ulong)((value >> 128) & ulong.MaxValue);
        return new Signed192(high, middle, low);
    }

    private static Signed576 ToPositiveSigned576(BigInteger value)
    {
        var words = new ulong[9];
        for (int index = 0; index < words.Length; index++)
        {
            words[index] = (ulong)(value & ulong.MaxValue);
            value >>= 64;
        }

        return new Signed576(
            words[8],
            words[7],
            words[6],
            words[5],
            words[4],
            words[3],
            words[2],
            words[1],
            words[0]);
    }
}
