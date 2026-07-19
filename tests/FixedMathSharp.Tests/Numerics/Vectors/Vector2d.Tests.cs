using MemoryPack;
using System;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

public class Vector2dTests
{
    [Fact]
    public void Vector2d_CanonicalPlaneForward_IsPositiveY()
    {
        Assert.Equal(new Vector2d(0, 1), Vector2d.Forward);
        Assert.Equal(new Vector2d(1, 0), Vector2d.Right);
        Assert.Equal(new Vector2d(0, -1), Vector2d.Backward);
        Assert.Equal(new Vector2d(-1, 0), Vector2d.Left);
    }

    [Fact]
    public void Vector2d_ForwardDirectionZero_IsPolarPositiveX()
    {
        Assert.Equal(Vector2d.Right, Vector2d.ForwardDirection(Fixed64.Zero));
        Assert.NotEqual(Vector2d.Forward, Vector2d.ForwardDirection(Fixed64.Zero));
    }

    [Fact]
    public void RotatedRight_Rotates90DegreesClockwise()
    {
        var vector = new Vector2d(1, 0);
        var result = vector.RotatedRight;

        Assert.Equal(new Vector2d(0, -1), result); // (1, 0) rotated 90° clockwise becomes (0, -1)
    }

    [Fact]
    public void RotatedLeft_Rotates90DegreesCounterclockwise()
    {
        var vector = new Vector2d(1, 0);
        var result = vector.RotatedLeft;

        Assert.Equal(new Vector2d(0, 1), result); // (1, 0) rotated 90° counterclockwise becomes (0, 1)
    }

    [Fact]
    public void RightHandNormal_ReturnsCorrectNormalVector()
    {
        var vector = new Vector2d(1, 0);
        var result = vector.RightHandNormal;

        Assert.Equal(new Vector2d(0, 1), result); // The right-hand normal of (1, 0) is (0, 1)
    }

    [Fact]
    public void LeftHandNormal_ReturnsCorrectNormalVector()
    {
        var vector = new Vector2d(1, 0);
        var result = vector.LeftHandNormal;

        Assert.Equal(new Vector2d(0, -1), result); // The left-hand normal of (1, 0) is (0, -1)
    }

    [Fact]
    public void Magnitude_CalculatesCorrectMagnitude()
    {
        var vector = new Vector2d(3, 4);
        var result = vector.Magnitude;

        Assert.Equal(new Fixed64(5), result); // The magnitude of (3, 4) is 5 (3-4-5 triangle)
    }

    [Fact]
    public void Magnitude_WhenSquaresSaturate_ReturnsRepresentableLength()
    {
        var vector = new Vector2d(60000, 80000);

        Assert.Equal(new Fixed64(100000), vector.Magnitude);
        Assert.True(Vector2d.TryGetMagnitude(vector, out Fixed64 magnitude));
        Assert.Equal(new Fixed64(100000), magnitude);
        Assert.Equal(new Fixed64(100000), Vector2d.Distance(Vector2d.Zero, vector));
    }

    [Fact]
    public void Normalize_WhenLengthExceedsScalarRange_ReturnsUnitDirectionAndReportsSaturation()
    {
        var vector = new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue);

        Assert.False(Vector2d.TryGetMagnitude(vector, out Fixed64 magnitude));
        Assert.Equal(Fixed64.MaxValue, magnitude);

        Vector2d normalized = vector.Normalized;
        Assert.Equal(normalized.X, normalized.Y);
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, normalized.Magnitude);

        Vector2d inPlace = vector;
        Assert.Equal(normalized, inPlace.NormalizeInPlace(out Fixed64 originalMagnitude));
        Assert.Equal(Fixed64.MaxValue, originalMagnitude);
    }

    [Fact]
    public void TryGetMagnitude_MaximumAxisLength_IsRepresentable()
    {
        Assert.True(Vector2d.TryGetMagnitude(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            out Fixed64 magnitude));
        Assert.Equal(Fixed64.MaxValue, magnitude);
    }

    [Fact]
    public void TryGetMagnitude_DetectsExactScalarRangeBoundary()
    {
        Assert.False(Vector2d.TryGetMagnitude(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            out Fixed64 minimumAxisMagnitude));
        Assert.Equal(Fixed64.MaxValue, minimumAxisMagnitude);

        Fixed64 justBelowMaximum = Fixed64.FromRaw(Fixed64.MaxValue.m_rawValue - 1);
        Assert.False(Vector2d.TryGetMagnitude(
            new Vector2d(justBelowMaximum, Fixed64.Two),
            out Fixed64 overflowMagnitude));
        Assert.Equal(Fixed64.MaxValue, overflowMagnitude);

        Assert.True(Vector2d.TryGetMagnitude(
            new Vector2d(justBelowMaximum, Fixed64.Half),
            out Fixed64 representableMagnitude));
        Assert.Equal(justBelowMaximum, representableMagnitude);

        Assert.False(Vector2d.TryGetMagnitude(
            new Vector2d(Fixed64.MaxValue, Fixed64.FromRaw(1)),
            out Fixed64 oneRawBeyondBoundary));
        Assert.Equal(Fixed64.MaxValue, oneRawBeyondBoundary);

        Fixed64 carryComponent = Fixed64.FromRaw((1L << 62) + 1);
        Assert.True(Vector2d.TryGetMagnitude(
            new Vector2d(carryComponent, carryComponent),
            out _));
    }

    [Fact]
    public void Normalize_MinimumAxisLength_ReturnsNegativeUnitDirection()
    {
        Assert.Equal(
            Vector2d.Left,
            new Vector2d(Fixed64.MinValue, Fixed64.Zero).Normalized);
    }

    [Fact]
    public void GetDirection_NormalizesFullDomainEndpointDifference()
    {
        var start = new Vector2d(Fixed64.MinValue, Fixed64.MinValue);
        var end = new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue);

        Assert.Equal(new Vector2d(1, 1).Normalized, Vector2d.GetDirection(start, end));
        Assert.Equal(Vector2d.Right, Vector2d.GetDirection(start, new Vector2d(Fixed64.MaxValue, start.Y)));
        Assert.Equal(Vector2d.Zero, Vector2d.GetDirection(start, start));
    }

    [Fact]
    public void TryGetDistance_PreservesRoundedAndFullDomainEndpointContracts()
    {
        Fixed64 oneRaw = Fixed64.MinIncrement;
        Fixed64 smallestComponentWhoseRawSquareExceedsMaxRaw = Fixed64.FromRaw(3_037_000_500L);
        Vector2d nearUnit = new(Fixed64.One, Fixed64.FromFraction(1, 65536));

        Assert.True(Vector2d.TryGetDistance(Vector2d.Zero, nearUnit, out Fixed64 rounded));
        Assert.Equal(Fixed64.One, rounded);
        Assert.False(Vector2d.TryGetDistance(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            out Fixed64 unrepresentable));
        Assert.Equal(Fixed64.MaxValue, unrepresentable);
        Assert.False(Vector2d.TryGetDistance(
            Vector2d.Zero,
            new Vector2d(Fixed64.MaxValue, smallestComponentWhoseRawSquareExceedsMaxRaw),
            out Fixed64 roundsPastMaximum));
        Assert.Equal(Fixed64.MaxValue, roundsPastMaximum);
        Assert.True(Vector2d.TryGetDistance(Vector2d.Zero, new Vector2d(oneRaw, oneRaw), out Fixed64 minimum));
        Assert.Equal(oneRaw, minimum);
    }

    [Fact]
    public void Lerp_InterpolatesAcrossTheFullScalarDomain()
    {
        var start = new Vector2d(Fixed64.MinValue, Fixed64.MaxValue);
        var end = new Vector2d(Fixed64.MaxValue, Fixed64.MinValue);

        Assert.Equal(Vector2d.Zero, Vector2d.Lerp(start, end, Fixed64.Half));
        Assert.Equal(Vector2d.Zero, start.Lerp(end, Fixed64.Half));
    }

    [Fact]
    public void CompareMagnitudeSquared_OrdersVectorsAcrossScalarMagnitudeRange()
    {
        var shorter = new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue - Fixed64.One);
        var longer = new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue);

        Assert.True(Vector2d.CompareMagnitudeSquared(shorter, longer) < 0);
        Assert.True(Vector2d.CompareMagnitudeSquared(longer, shorter) > 0);
        Assert.Equal(0, Vector2d.CompareMagnitudeSquared(longer, -longer));

        var oneRaw = new Vector2d(Fixed64.FromRaw(1), Fixed64.Zero);
        var twoRaw = new Vector2d(Fixed64.FromRaw(2), Fixed64.Zero);
        Assert.True(Vector2d.CompareMagnitudeSquared(oneRaw, twoRaw) < 0);
        Assert.True(Vector2d.CompareMagnitudeSquared(twoRaw, oneRaw) > 0);
    }

    [Fact]
    public void CompareDistanceSquared_OrdersFullDomainDifferencesWithoutSaturation()
    {
        Vector2d query = new(Fixed64.MaxValue, Fixed64.MaxValue);
        Vector2d nearer = Vector2d.One;
        Vector2d farther = new(Fixed64.MinValue, Fixed64.MinValue);

        Assert.True(Vector2d.CompareDistanceSquared(query, nearer, query, farther) < 0);
        Assert.True(Vector2d.CompareDistanceSquared(query, farther, query, nearer) > 0);
        Assert.Equal(0, Vector2d.CompareDistanceSquared(query, nearer, nearer, query));
    }

    [Fact]
    public void OrientationSign_ClassifiesFullDomainTurnsExactly()
    {
        Vector2d origin = new(Fixed64.MinValue, Fixed64.MinValue);
        Vector2d right = new(Fixed64.MaxValue, Fixed64.MinValue);
        Vector2d up = new(Fixed64.MinValue, Fixed64.MaxValue);

        Assert.Equal(1, Vector2d.OrientationSign(origin, right, up));
        Assert.Equal(-1, Vector2d.OrientationSign(origin, up, right));
        Assert.Equal(0, Vector2d.OrientationSign(origin, right, right));
    }

    [Fact]
    public void Distance_NearUnit_PreservesOrdinarySquareRootResult()
    {
        Fixed64 nearUnit = Fixed64.FromRaw(Fixed64.One.m_rawValue + 1);

        Assert.Equal(nearUnit, Vector2d.Distance(Vector2d.Zero, new Vector2d(nearUnit, Fixed64.Zero)));
    }

    [Fact]
    public void Distance_WhenSquaresSaturate_ReturnsRepresentableSignedEndpointLength()
    {
        var start = new Vector2d(-30000, -40000);
        var end = new Vector2d(30000, 40000);

        Assert.Equal(new Fixed64(100000), Vector2d.Distance(start, end));
    }

    [Fact]
    public void MagnitudeSquared_CalculatesCorrectSquareMagnitude()
    {
        var vector = new Vector2d(3, 4);
        var result = vector.MagnitudeSquared;

        Assert.Equal(new Fixed64(25), result); // The squared magnitude of (3, 4) is 25 (3^2 + 4^2)
    }

    [Fact]
    public void FromDouble_UsesCheckedFixed64Conversion()
    {
        var vector = Vector2d.FromDouble(1.25, -2.5);

        Assert.Equal(Fixed64.FromDouble(1.25), vector.X);
        Assert.Equal(Fixed64.FromDouble(-2.5), vector.Y);

        Assert.Throws<ArgumentOutOfRangeException>(() => Vector2d.FromDouble(double.NaN, 0));
        Assert.Throws<OverflowException>(() => Vector2d.FromDouble(0, 2147483648d));
    }


    [Fact]
    public void NormalizeInPlace_NormalizesVectorCorrectly()
    {
        var vector = new Vector2d(3, 4);
        vector.NormalizeInPlace();

        var expected = new Vector2d(Fixed64.FromDouble(0.6), Fixed64.FromDouble(0.8)); // Normalized vector (0.6, 0.8)
        Assert.True(vector.FuzzyEqual(expected, Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void IsNormalized_UsesNonzeroSquaredMagnitudeEpsilonContract()
    {
        Assert.True(Vector2d.Right.IsNormalized());
        Assert.True(Vector2d.Forward.IsNormalized());
        Assert.False(Vector2d.Zero.IsNormalized());
        Assert.False(new Vector2d(2, 0).IsNormalized());
        Assert.True(new Vector2d(3, 4).Normalized.IsNormalized());

        var oneRawInside = new Vector2d(Fixed64.One, Fixed64.FromRaw(1_045_500));
        var oneRawOutside = new Vector2d(Fixed64.One, Fixed64.FromRaw(1_049_600));
        Assert.Equal(Fixed64.One + Fixed64.Epsilon - Fixed64.MinIncrement, oneRawInside.MagnitudeSquared);
        Assert.Equal(Fixed64.One + Fixed64.Epsilon + Fixed64.MinIncrement, oneRawOutside.MagnitudeSquared);
        Assert.True(oneRawInside.IsNormalized());
        Assert.False(oneRawOutside.IsNormalized());
    }

    [Fact]
    public void NormalizedResults_AreNormalizedAcrossTinyOrdinaryAndExtremeVectorDomains()
    {
        Vector2d[] vectors2d =
        {
            new(Fixed64.FromRaw(1), Fixed64.FromRaw(2)),
            new(3, 4),
            new(Fixed64.MaxValue, Fixed64.MinValue),
        };
        Vector3d[] vectors3d =
        {
            new(Fixed64.FromRaw(1), Fixed64.FromRaw(2), Fixed64.FromRaw(3)),
            new(2, -3, 6),
            new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue),
        };
        Vector4d[] vectors4d =
        {
            new(Fixed64.FromRaw(1), Fixed64.FromRaw(2), Fixed64.FromRaw(3), Fixed64.FromRaw(4)),
            new(Fixed64.One, -Fixed64.Two, Fixed64.Three, -Fixed64.One),
            new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue),
        };

        foreach (Vector2d vector in vectors2d)
            Assert.True(vector.Normalized.IsNormalized(), $"2D normalization failed for {vector}.");
        foreach (Vector3d vector in vectors3d)
            Assert.True(vector.Normalized.IsNormalized(), $"3D normalization failed for {vector}.");
        foreach (Vector4d vector in vectors4d)
            Assert.True(vector.Normalized.IsNormalized(), $"4D normalization failed for {vector}.");
    }

    [Fact]
    public void LerpInPlace_InterpolatesBetweenVectorsCorrectly()
    {
        var start = new Vector2d(0, 0);
        var end = new Vector2d(10, 10);
        var amount = Fixed64.FromDouble(0.5); // 50% interpolation

        start.LerpInPlace(end, amount);
        Assert.Equal(new Vector2d(5, 5), start); // Should be halfway between (0, 0) and (10, 10)
    }

    [Fact]
    public void RotateInPlace_RotatesVectorCorrectly()
    {
        var vector = new Vector2d(1, 0);
        var cos = FixedMath.Cos(Fixed64.HalfPi); // 90° cosine
        var sin = FixedMath.Sin(Fixed64.HalfPi); // 90° sine

        vector.RotateInPlace(cos, sin);
        Assert.True(vector.FuzzyEqual(new Vector2d(0, 1), Fixed64.FromDouble(0.0001))); // (1, 0) rotated 90° becomes (0, 1)
    }

    [Fact]
    public void RotateInverse_RotatesVectorInOppositeDirection()
    {
        var vector = new Vector2d(1, 0);
        var cos = FixedMath.Cos(Fixed64.HalfPi); // 90° cosine
        var sin = FixedMath.Sin(Fixed64.HalfPi); // 90° sine

        vector.RotateInverse(cos, sin);
        Assert.True(vector.FuzzyEqual(new Vector2d(0, -1), Fixed64.FromDouble(0.0001))); // Should rotate -90° to (0, -1)
    }

    [Fact]
    public void RotateRightInPlace_RotatesVector90DegreesClockwise()
    {
        var vector = new Vector2d(1, 0);
        vector.RotateRightInPlace();

        Assert.Equal(new Vector2d(0, -1), vector); // (1, 0) rotated 90° clockwise becomes (0, -1)
    }

    [Fact]
    public void RotateLeftInPlace_RotatesVector90DegreesCounterclockwise()
    {
        var vector = new Vector2d(1, 0);
        vector.RotateLeftInPlace();

        Assert.Equal(new Vector2d(0, 1), vector); // (1, 0) rotated 90° counterclockwise becomes (0, 1)
    }

    [Fact]
    public void MultiplyInPlace_ScalarOverload_MultipliesVectorCorrectly()
    {
        var vector = new Vector2d(2, 3);
        var factor = new Fixed64(2);
        vector.MultiplyInPlace(factor);

        Assert.Equal(new Vector2d(4, 6), vector);
    }

    [Fact]
    public void Dot_ComputesDotProductCorrectly()
    {
        var vector1 = new Vector2d(1, 2);
        var vector2 = new Vector2d(3, 4);
        var result = vector1.Dot(vector2);

        Assert.Equal(new Fixed64(11), result); // Dot product of (1, 2) and (3, 4) is 1*3 + 2*4 = 11
    }

    [Fact]
    public void Cross_ComputesCrossProductCorrectly()
    {
        var vector1 = new Vector2d(1, 2);
        var vector2 = new Vector2d(3, 4);
        var result = vector1.CrossProduct(vector2);

        Assert.Equal(new Fixed64(-2), result); // Cross product of (1, 2) and (3, 4) is 1*4 - 2*3 = -2
        Assert.Equal(result, Vector2d.CrossProduct(vector1, vector2));
    }

    [Fact]
    public void Distance_ComputesDistanceCorrectly()
    {
        var vector1 = new Vector2d(1, 1);
        var vector2 = new Vector2d(4, 5);
        var result = vector1.Distance(vector2);

        Assert.Equal(new Fixed64(5), result); // Distance between (1, 1) and (4, 5) is 5 (3-4-5 triangle)
    }

    [Fact]
    public void DistanceSquared_ComputesSquareDistanceCorrectly()
    {
        var vector1 = new Vector2d(1, 1);
        var vector2 = new Vector2d(4, 5);
        var result = vector1.DistanceSquared(vector2);

        Assert.Equal(new Fixed64(25), result); // Squared distance between (1, 1) and (4, 5) is 25
    }

    [Fact]
    public void ReflectInPlace_ReflectsVectorCorrectly()
    {
        var vector = new Vector2d(1, 1);
        var axisX = new Fixed64(0);
        var axisY = new Fixed64(1); // Reflect across the Y-axis

        vector.ReflectInPlace(axisX, axisY);
        Assert.Equal(new Vector2d(-1, 1), vector); // Reflecting (1, 1) across Y-axis becomes (-1, 1)
    }

    [Fact]
    public void AddInPlace_AddsToVectorCorrectly()
    {
        var vector = new Vector2d(1, 1);
        var amount = new Fixed64(2);
        vector.AddInPlace(amount);

        Assert.Equal(new Vector2d(3, 3), vector); // (1, 1) + 2 becomes (3, 3)
    }

    [Fact]
    public void SubtractInPlace_SubtractsFromVectorCorrectly()
    {
        var vector = new Vector2d(3, 3);
        var amount = new Fixed64(1);
        vector.SubtractInPlace(amount);

        Assert.Equal(new Vector2d(2, 2), vector); // (3, 3) - 1 becomes (2, 2)
    }

    [Fact]
    public void Indexer_GetAndSet_RoundTripsComponents()
    {
        var vector = new Vector2d(1, 2);

        Assert.Equal(Fixed64.One, vector[0]);
        Assert.Equal(new Fixed64(2), vector[1]);

        vector[0] = new Fixed64(3);
        vector[1] = new Fixed64(4);

        Assert.Equal(new Vector2d(3, 4), vector);
    }

    [Fact]
    public void Indexer_InvalidIndex_Throws()
    {
        var vector = new Vector2d(1, 2);

        Assert.Throws<IndexOutOfRangeException>(() => _ = vector[-1]);
        Assert.Throws<IndexOutOfRangeException>(() => _ = vector[2]);
        Assert.Throws<IndexOutOfRangeException>(() => vector[-1] = Fixed64.One);
        Assert.Throws<IndexOutOfRangeException>(() => vector[2] = Fixed64.One);
    }

    [Fact]
    public void Set_UpdatesComponents()
    {
        var vector = new Vector2d(1, 2);

        vector.Set(new Fixed64(5), new Fixed64(6));

        Assert.Equal(new Vector2d(5, 6), vector);
    }

    [Fact]
    public void AddInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector2d(1, 2);

        Assert.Equal(new Vector2d(4, 5), vector.AddInPlace(new Fixed64(3)));
        Assert.Equal(new Vector2d(5, 7), vector.AddInPlace(Fixed64.One, new Fixed64(2)));
        Assert.Equal(new Vector2d(7, 10), vector.AddInPlace(new Vector2d(2, 3)));
    }

    [Fact]
    public void SubtractInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector2d(10, 12);

        Assert.Equal(new Vector2d(9, 11), vector.SubtractInPlace(Fixed64.One));
        Assert.Equal(new Vector2d(7, 8), vector.SubtractInPlace(new Fixed64(2), new Fixed64(3)));
        Assert.Equal(new Vector2d(6, 6), vector.SubtractInPlace(new Vector2d(1, 2)));
    }

    [Fact]
    public void StaticArithmeticHelpers_ReturnExpectedValues()
    {
        var left = new Vector2d(12, 18);
        var right = new Vector2d(3, 6);

        Assert.Equal(new Vector2d(15, 24), Vector2d.Add(left, right));
        Assert.Equal(new Vector2d(9, 12), Vector2d.Subtract(left, right));
        Assert.Equal(new Vector2d(36, 108), Vector2d.Multiply(left, right));
        Assert.Equal(new Vector2d(24, 36), Vector2d.Multiply(left, new Fixed64(2)));
        Assert.Equal(new Vector2d(24, 36), new Fixed64(2) * left);
        Assert.Equal(new Vector2d(4, 3), Vector2d.Divide(left, right));
        Assert.Equal(new Vector2d(6, 9), Vector2d.Divide(left, new Fixed64(2)));
    }

    [Fact]
    public void TryAddAndTrySubtract_ExactResultsIncludingRepresentableLimits_Succeed()
    {
        var left = new Vector2d(12, 18);
        var right = new Vector2d(3, 6);

        Assert.True(Vector2d.TryAdd(left, right, out Vector2d sum));
        Assert.Equal(new Vector2d(15, 24), sum);
        Assert.True(Vector2d.TrySubtract(left, right, out Vector2d difference));
        Assert.Equal(new Vector2d(9, 12), difference);

        var boundaryLeft = new Vector2d(
            Fixed64.FromRaw(long.MaxValue - 1),
            Fixed64.FromRaw(long.MinValue + 1));
        Assert.True(Vector2d.TryAdd(
            boundaryLeft,
            new Vector2d(Fixed64.MinIncrement, Fixed64.FromRaw(-1)),
            out Vector2d boundarySum));
        Assert.Equal(new Vector2d(Fixed64.MaxValue, Fixed64.MinValue), boundarySum);

        Assert.True(Vector2d.TrySubtract(
            boundaryLeft,
            new Vector2d(Fixed64.FromRaw(-1), Fixed64.MinIncrement),
            out Vector2d boundaryDifference));
        Assert.Equal(new Vector2d(Fixed64.MaxValue, Fixed64.MinValue), boundaryDifference);
    }

    [Fact]
    public void TryAdd_ComponentOverflow_ReturnsFalseAndDefaultAtomically()
    {
        var firstLeft = new Vector2d(Fixed64.MaxValue, Fixed64.One);
        var firstRight = new Vector2d(Fixed64.MinIncrement, Fixed64.One);
        Assert.False(Vector2d.TryAdd(firstLeft, firstRight, out Vector2d firstResult));
        Assert.Equal(default, firstResult);
        Assert.Equal(new Vector2d(Fixed64.MaxValue, Fixed64.Two), firstLeft + firstRight);

        var finalLeft = new Vector2d(Fixed64.One, Fixed64.MinValue);
        var finalRight = new Vector2d(Fixed64.One, Fixed64.FromRaw(-1));
        Assert.False(Vector2d.TryAdd(finalLeft, finalRight, out Vector2d finalResult));
        Assert.Equal(default, finalResult);
        Assert.Equal(new Vector2d(Fixed64.Two, Fixed64.MinValue), finalLeft + finalRight);
    }

    [Fact]
    public void TrySubtract_ComponentOverflow_ReturnsFalseAndDefaultAtomically()
    {
        var firstLeft = new Vector2d(Fixed64.MinValue, Fixed64.Three);
        var firstRight = new Vector2d(Fixed64.MinIncrement, Fixed64.One);
        Assert.False(Vector2d.TrySubtract(firstLeft, firstRight, out Vector2d firstResult));
        Assert.Equal(default, firstResult);
        Assert.Equal(new Vector2d(Fixed64.MinValue, Fixed64.Two), firstLeft - firstRight);

        var finalLeft = new Vector2d(Fixed64.Three, Fixed64.MaxValue);
        var finalRight = new Vector2d(Fixed64.One, Fixed64.FromRaw(-1));
        Assert.False(Vector2d.TrySubtract(finalLeft, finalRight, out Vector2d finalResult));
        Assert.Equal(default, finalResult);
        Assert.Equal(new Vector2d(Fixed64.Two, Fixed64.MaxValue), finalLeft - finalRight);
    }

    [Fact]
    public void CompareProjection_FullDomain_ReturnsExactSignAndCancellation()
    {
        var maximum = new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue);
        var minimum = new Vector2d(Fixed64.MinValue, Fixed64.MinValue);
        var direction = new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue);

        Assert.Equal(1, Vector2d.CompareProjection(maximum, minimum, direction));
        Assert.Equal(-1, Vector2d.CompareProjection(minimum, maximum, direction));
        Assert.Equal(0, Vector2d.CompareProjection(
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue),
            direction));
        Assert.Equal(-1, Vector2d.CompareProjection(
            maximum,
            minimum,
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue)));
        Assert.Equal(0, Vector2d.CompareProjection(maximum, minimum, Vector2d.Zero));
    }

    [Fact]
    public void CompareProjection_OpposingLowProducts_PropagatesCarryAcrossWords()
    {
        int result = Vector2d.CompareProjection(
            new Vector2d(Fixed64.FromRaw(2), Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.MinIncrement),
            new Vector2d(Fixed64.MinIncrement, Fixed64.MinIncrement));

        Assert.Equal(1, result);
    }

    [Fact]
    public void BarycentricCoordinates_WeightsSecondAndThirdVertices()
    {
        var value1 = new Vector2d(10, 100);
        var value2 = new Vector2d(20, 200);
        var value3 = new Vector2d(30, 300);

        Assert.Equal(value1, Vector2d.BarycentricCoordinates(value1, value2, value3, Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(value2, Vector2d.BarycentricCoordinates(value1, value2, value3, Fixed64.One, Fixed64.Zero));
        Assert.Equal(value3, Vector2d.BarycentricCoordinates(value1, value2, value3, Fixed64.Zero, Fixed64.One));
        Assert.Equal(new Vector2d(25, 250), Vector2d.BarycentricCoordinates(value1, value2, value3, Fixed64.Half, Fixed64.Half));
    }

    [Fact]
    public void MultiplyInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector2d(2, 3);

        Assert.Equal(new Vector2d(4, 6), vector.MultiplyInPlace(new Fixed64(2)));
        Assert.Equal(new Vector2d(12, 24), vector.MultiplyInPlace(new Fixed64(3), new Fixed64(4)));
        Assert.Equal(new Vector2d(24, 72), vector.MultiplyInPlace(new Vector2d(2, 3)));
    }

    [Fact]
    public void DivideInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector2d(24, 72);

        Assert.Equal(new Vector2d(12, 36), vector.DivideInPlace(new Fixed64(2)));
        Assert.Equal(new Vector2d(4, 9), vector.DivideInPlace(new Fixed64(3), new Fixed64(4)));
        Assert.Equal(new Vector2d(2, 3), vector.DivideInPlace(new Vector2d(2, 3)));
    }

    [Fact]
    public void InPlaceHelpers_CanChainWhenAssigned()
    {
        var vector = new Vector2d(2, 4);

        vector = vector.AddInPlace(new Vector2d(2, 2))
            .MultiplyInPlace(new Fixed64(3))
            .DivideInPlace(new Vector2d(2, 3))
            .SubtractInPlace(Fixed64.One);

        Assert.Equal(new Vector2d(5, 5), vector);
    }

    [Fact]
    public void MultiplyInPlace_VectorOverload_MultipliesPerComponent()
    {
        var vector = new Vector2d(2, 3);

        Assert.Equal(new Vector2d(4, 12), vector.MultiplyInPlace(new Vector2d(2, 4)));
    }

    [Fact]
    public void NormalizeOut_ReturnsOriginalMagnitudeAndNormalizedVector()
    {
        var vector = new Vector2d(3, 4);

        var normalized = vector.NormalizeInPlace(out var magnitude);

        Assert.Equal(new Fixed64(5), magnitude);
        Assert.Equal(vector, normalized);
        Assert.True(vector.FuzzyEqual(new Vector2d(Fixed64.FromDouble(0.6), Fixed64.FromDouble(0.8)), Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void NormalizeOut_ZeroVector_ReturnsZeroMagnitude()
    {
        var vector = Vector2d.Zero;

        var normalized = vector.NormalizeInPlace(out var magnitude);

        Assert.Equal(Fixed64.Zero, magnitude);
        Assert.Equal(Vector2d.Zero, normalized);
    }

    [Fact]
    public void NormalizeOut_UnitVector_LeavesVectorUnchanged()
    {
        var vector = Vector2d.Right;

        var normalized = vector.NormalizeInPlace(out var magnitude);

        Assert.Equal(Fixed64.One, magnitude);
        Assert.Equal(Vector2d.Right, normalized);
    }

    [Theory]
    [InlineData(3, 4)]
    [InlineData(-3, 4)]
    [InlineData(5, -12)]
    [InlineData(123, 456)]
    public void Normalize_MatchesComponentDivisionByMagnitude(int x, int y)
    {
        var source = new Vector2d(x, y);

        AssertNormalizeMatchesComponentDivision(source);
    }

    [Fact]
    public void Normalize_MatchesComponentDivisionByMagnitude_ForFractionalAndHugeValues()
    {
        AssertNormalizeMatchesComponentDivision(Vector2d.FromDouble(1.5, -2.25));
        AssertNormalizeMatchesComponentDivision(new Vector2d(10000, -20000));
    }

    [Fact]
    public void NormalizeInPlace_DoesNothingForZeroVector()
    {
        var vector = new Vector2d(0, 0);
        vector.NormalizeInPlace();

        Assert.Equal(new Vector2d(0, 0), vector); // A zero vector remains zero after normalization
    }

    private static void AssertNormalizeMatchesComponentDivision(Vector2d source)
    {
        Fixed64 magnitude = source.Magnitude;
        if (magnitude == Fixed64.Zero)
        {
            Assert.Equal(Vector2d.Zero, source.Normalized);

            var zeroLength = source;
            Assert.Equal(Vector2d.Zero, zeroLength.NormalizeInPlace());
            return;
        }

        Vector2d expected = source / magnitude;
        Assert.Equal(expected, source.Normalized);

        var inPlace = source;
        Assert.Equal(expected, inPlace.NormalizeInPlace(out Fixed64 originalMagnitude));
        Assert.Equal(magnitude, originalMagnitude);
        Assert.Equal(expected, inPlace);
    }

    [Fact]
    public void AllComponentsGreaterThanEpsilon_ReturnsTrue_WhenAllComponentsExceedEpsilon()
    {
        var vector = new Vector2d(Fixed64.Epsilon + Fixed64.One, Fixed64.Epsilon + Fixed64.One);

        Assert.True(vector.AllComponentsGreaterThanEpsilon());
    }

    [Fact]
    public void AllComponentsGreaterThanEpsilon_ReturnsFalse_WhenAComponentIsAtOrBelowEpsilon()
    {
        var vector = new Vector2d(Fixed64.Epsilon, Fixed64.Epsilon + Fixed64.One);

        Assert.False(vector.AllComponentsGreaterThanEpsilon());
    }

    [Fact]
    public void V2ClampExtensions_MatchCanonicalImplementations()
    {
        var vector = new Vector2d(2, -3);

        Assert.Equal(new Vector2d(1, -1), vector.ClampOne());
        Assert.Equal(new Vector2d(1, -1), Vector2d.Clamp(vector, Vector2d.Negative, Vector2d.One));
        Assert.Equal(Vector2d.Clamp(vector, Vector2d.Negative, Vector2d.One), vector.Clamp(Vector2d.Negative, Vector2d.One));
    }

    [Fact]
    public void V2AbsAndSign_ExtensionsReturnComponentWiseResults()
    {
        var vector = Vector2d.FromDouble(-2, 0.5);

        Assert.Equal(Vector2d.FromDouble(2, 0.5), vector.Abs());
        Assert.Equal(Vector2d.FromDouble(-1, 1), vector.Sign());
    }

    [Fact]
    public void V2ToDegrees_ConvertsCorrectly()
    {
        var radians = new Vector2d(Fixed64.HalfPi, Fixed64.Pi); // (90°, 180°)
        var result = radians.ToDegrees();

        Assert.True(result.FuzzyEqual(new Vector2d(90, 180))); // Converts radians to degrees
    }

    [Fact]
    public void V2ToRadians_ConvertsCorrectly()
    {
        var degrees = new Vector2d(90, 180);
        var result = degrees.ToRadians();

        Assert.True(result.FuzzyEqual(new Vector2d(Fixed64.HalfPi, Fixed64.Pi))); // Converts degrees to radians
    }

    [Fact]
    public void V2FuzzyEqualAbsolute_ComparesCorrectly_WithAllowedDifference()
    {
        var vector1 = new Vector2d(2, 2);
        var vector2 = Vector2d.FromDouble(2.1, 2.1);
        var allowedDifference = Fixed64.FromDouble(0.15);

        Assert.True(vector1.FuzzyEqualAbsolute(vector2, allowedDifference)); // Approximate equality with a 0.15 difference
    }

    [Fact]
    public void V2FuzzyEqual_ComparesCorrectly_WithDefaultTolerance()
    {
        var vector1 = new Vector2d(100, 100);
        var vector2 = Vector2d.FromDouble(100.0000008537, 100.0000008537); // Small difference

        // Use FuzzyEqual with the default tolerance, which is small (e.g., 0.01% difference)
        Assert.True(vector1.FuzzyEqual(vector2)); // The difference should be within the default tolerance

        vector2 = Vector2d.FromDouble(100.0001, 100.0001); // big difference
        Assert.False(vector1.FuzzyEqual(vector2)); // The difference should be outside the default tolerance
    }


    [Fact]
    public void V2FuzzyEqual_ComparesCorrectly_WithCustomPercentage()
    {
        var vector1 = new Vector2d(100, 100);
        var vector2 = new Vector2d(102, 102);
        var percentage = Fixed64.FromDouble(0.02); // Allow a 2% difference

        Assert.True(vector1.FuzzyEqual(vector2, percentage)); // Should be approximately equal within 2% difference
    }

    [Fact]
    public void V2CheckDistance_VerifiesDistanceCorrectly()
    {
        var vector1 = new Vector2d(0, 0);
        var vector2 = new Vector2d(3, 4); // Distance is 5 (3-4-5 triangle)
        var factor = new Fixed64(5);

        Assert.True(vector1.CheckDistance(vector2, factor)); // Distance is 5, so should return true
    }

    [Fact]
    public void V2CheckDistance_NegativeFactor_ReturnsFalse()
    {
        var vector1 = new Vector2d(0, 0);
        var vector2 = new Vector2d(1, 0);

        Assert.False(vector1.CheckDistance(vector2, -Fixed64.One));
    }

    [Fact]
    public void V2DistanceSquared_CalculatesCorrectly()
    {
        var vector1 = new Vector2d(0, 0);
        var vector2 = new Vector2d(3, 4); // Squared distance should be 25
        var result = vector1.DistanceSquared(vector2);

        Assert.Equal(new Fixed64(25), result); // 3^2 + 4^2 = 25
    }

    [Fact]
    public void V2Rotate_RotatesVectorCorrectly()
    {
        var vector = new Vector2d(1, 0);
        var angle = Fixed64.HalfPi; // Rotate by 90° (π/2 radians)
        var result = vector.Rotate(angle);

        Assert.True(result.FuzzyEqual(new Vector2d(0, 1), Fixed64.FromDouble(0.0001))); // Should rotate to (0, 1)
    }

    [Fact]
    public void Lerp_ReturnsInterpolatedCopyWithoutMutatingOriginal()
    {
        var start = new Vector2d(0, 0);

        var result = start.Lerp(new Vector2d(10, 20), Fixed64.FromDouble(0.25));

        Assert.Equal(new Vector2d(Fixed64.FromDouble(2.5), new Fixed64(5)), result);
        Assert.Equal(Vector2d.Zero, start);
    }

    [Fact]
    public void Rotated_Overloads_ReturnRotatedCopiesWithoutMutatingOriginal()
    {
        var vector = new Vector2d(1, 0);
        var rotation = Vector2d.CreateRotation(Fixed64.HalfPi);

        var rotatedByComponents = vector.Rotated(rotation.X, rotation.Y);
        var rotatedByVector = vector.Rotated(rotation);

        Assert.True(rotatedByComponents.FuzzyEqual(Vector2d.Forward, Fixed64.FromDouble(0.0001)));
        Assert.True(rotatedByVector.FuzzyEqual(Vector2d.Forward, Fixed64.FromDouble(0.0001)));
        Assert.Equal(Vector2d.Right, vector);
    }

    [Fact]
    public void Reflected_Overloads_ReturnReflectedCopiesWithoutMutatingOriginal()
    {
        var vector = new Vector2d(1, 1);
        var axis = Vector2d.Forward;

        var reflectedByComponents = vector.Reflected(axis.X, axis.Y);
        var reflectedByVector = vector.Reflected(axis);

        Assert.Equal(new Vector2d(-1, 1), reflectedByComponents);
        Assert.Equal(new Vector2d(-1, 1), reflectedByVector);
        Assert.Equal(new Vector2d(1, 1), vector);
    }

    [Fact]
    public void ReflectInPlace_WithPrecomputedProjection_ReflectsCorrectly()
    {
        var vector = new Vector2d(1, 1);
        var axis = Vector2d.Forward;
        var projection = vector.Dot(axis);

        var reflected = vector.ReflectInPlace(axis.X, axis.Y, projection);

        Assert.Equal(new Vector2d(-1, 1), reflected);
    }

    [Fact]
    public void ForwardDirection_ReturnsExpectedUnitVector()
    {
        var result = Vector2d.ForwardDirection(Fixed64.HalfPi);

        Assert.True(result.FuzzyEqual(Vector2d.Forward, Fixed64.FromDouble(0.0001)));
    }

    [Fact]
    public void Vector2d_PropertiesAndHashes_ReturnExpectedValues()
    {
        var vector = new Vector2d(new Fixed64(3), new Fixed64(4));

        Assert.Equal(new Vector2d(Fixed64.FromDouble(0.6), Fixed64.FromDouble(0.8)), vector.Normalized);
        Assert.Equal(new Fixed64(5), vector.Magnitude);
        Assert.Equal(new Fixed64(25), vector.MagnitudeSquared);
        Assert.Equal(vector.X.m_rawValue * 31 + vector.Y.m_rawValue * 7, vector.LongStateHash);
        Assert.Equal((int)(vector.LongStateHash % int.MaxValue), vector.StateHash);
        Assert.Equal(vector.StateHash, vector.GetHashCode());
    }

    [Fact]
    public void Vector2d_LerpHelpers_CoverClampAndCopyBranches()
    {
        var vector = new Vector2d(1, 2);

        Assert.Equal(new Vector2d(10, 20), vector.LerpInPlace(new Vector2d(10, 20), new Fixed64(2)));
        Assert.Equal(new Vector2d(10, 20), vector.LerpInPlace(new Vector2d(30, 40), Fixed64.Zero));
        Assert.Equal(Vector2d.Zero, Vector2d.Lerp(Vector2d.Zero, new Vector2d(10, 20), -Fixed64.One));
        Assert.Equal(new Vector2d(10, 20), Vector2d.Lerp(Vector2d.Zero, new Vector2d(10, 20), new Fixed64(2)));
    }

    [Fact]
    public void Vector2d_ReflectionAndMagnitudeHelpers_CoverOverloadsAndNormalizationBranches()
    {
        var vector = new Vector2d(1, 1);

        Assert.Equal(new Vector2d(-1, 1), vector.ReflectInPlace(Vector2d.Forward));
        Assert.Equal(Vector2d.Right, Vector2d.GetNormalized(Vector2d.Right));

        var slightlyAboveUnit = new Vector2d(Fixed64.One, Fixed64.FromRaw(65536));
        Assert.Equal(Fixed64.One, Vector2d.GetMagnitude(slightlyAboveUnit));
    }

    [Fact]
    public void Vector2d_StaticHelpersAndConversions_WorkCorrectly()
    {
        var vector = new Vector2d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(-2.5));

        Assert.Equal(new Vector2d(0, -1), Vector2d.Backward);
        Assert.Equal(new Vector2d(-1, 0), Vector2d.Left);
        Assert.Equal(new Fixed64(5), Vector2d.Distance(Vector2d.Zero, new Vector2d(3, 4)));
        Assert.Equal(new Fixed64(25), Vector2d.DistanceSquared(Vector2d.Zero, new Vector2d(3, 4)));
        Assert.Equal(new Fixed64(11), Vector2d.Dot(new Vector2d(1, 2), new Vector2d(3, 4)));
        Assert.Equal(new Vector2d(8, 15), Vector2d.Multiply(new Vector2d(2, 3), new Vector2d(4, 5)));
        Assert.Equal("(1.25, -2.5)", vector.ToString());
        Assert.Equal(new Vector3d(Fixed64.FromDouble(1.25), new Fixed64(7), Fixed64.FromDouble(-2.5)), vector.ToVector3d(new Fixed64(7)));

        vector.Deconstruct(out Fixed64 x, out Fixed64 y);
        vector.Deconstruct(out long lx, out long ly);
        vector.Deconstruct(out double fx, out double fy);
        vector.Deconstruct(out int ix, out int iy);

        Assert.Equal(Fixed64.FromDouble(1.25), x);
        Assert.Equal(Fixed64.FromDouble(-2.5), y);
        Assert.Equal(vector.X.m_rawValue, lx);
        Assert.Equal(vector.Y.m_rawValue, ly);
        Assert.Equal(1.25f, fx);
        Assert.Equal(-2.5f, fy);
        Assert.Equal(1, ix);
        Assert.Equal(-2, iy);
    }

    [Fact]
    public void ClosestPointOnLineSegment_ProjectsAndClampsToSegment()
    {
        var segmentStart = new Vector2d(0, 0);
        var segmentEnd = new Vector2d(8, 0);

        Assert.Equal(new Vector2d(4, 0), Vector2d.ClosestPointOnLineSegment(new Vector2d(4, 3), segmentStart, segmentEnd));
        Assert.Equal(segmentStart, Vector2d.ClosestPointOnLineSegment(new Vector2d(-2, 5), segmentStart, segmentEnd));
        Assert.Equal(segmentEnd, Vector2d.ClosestPointOnLineSegment(new Vector2d(12, -5), segmentStart, segmentEnd));
    }

    [Fact]
    public void ClosestPointOnLineSegment_ReturnsStartForZeroLengthSegment()
    {
        var start = new Vector2d(2, 3);

        Assert.Equal(start, Vector2d.ClosestPointOnLineSegment(new Vector2d(9, 9), start, start));
    }

    [Fact]
    public void ClosestPointOnLineSegment_FullDomainDelta_ProjectsAndPreservesEndpoints()
    {
        var start = new Vector2d(Fixed64.MinValue, Fixed64.Zero);
        var end = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
        var midpointProbe = new Vector2d(Fixed64.Zero, Fixed64.MaxValue);

        Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.Zero),
            Vector2d.ClosestPointOnLineSegment(midpointProbe, start, end));
        Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.Zero),
            Vector2d.ClosestPointOnLineSegment(midpointProbe, end, start));
        Assert.Equal(start, Vector2d.ClosestPointOnLineSegment(start, start, end));
        Assert.Equal(end, Vector2d.ClosestPointOnLineSegment(end, start, end));
    }

    [Fact]
    public void Vector2d_EqualityAndComparisonHelpers_WorkCorrectly()
    {
        var a = new Vector2d(1, 2);
        var b = new Vector2d(1, 2);
        var c = new Vector2d(2, 3);

        Assert.Equal(new Vector2d(3, 5), a + c);
        Assert.Equal(new Vector2d(1, 1), c - a);
        Assert.Equal(new Vector2d(0, 1), a - Fixed64.One);
        Assert.Equal(new Vector2d(0, -1), Fixed64.One - a);
        Assert.True(a == b);
        Assert.False(a != b);
        Assert.True(Vector2d.Zero.EqualsZero());
        Assert.True(a.NotZero());
        Assert.True(a.Equals(a, b));
        Assert.False(a.Equals(a, c));
        Assert.False(a.Equals("not-a-vector"));
        Assert.True(c.CompareTo(a) > 0);
        Assert.Equal(a.GetHashCode(), a.GetHashCode(a));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void Vector2d_FuzzyEqualAbsolute_ReturnsFalse_WhenAnyComponentExceedsTolerance(int componentIndex)
    {
        var actual = new Vector2d(1, 2);
        var expected = actual;
        expected[componentIndex] += Fixed64.FromDouble(0.2);

        Assert.False(actual.FuzzyEqualAbsolute(expected, Fixed64.FromDouble(0.1)));
    }

    [Fact]
    public void OperatorOverloads_WithTuplesFloatsAndUnaryMinus_WorkCorrectly()
    {
        var vector = new Vector2d(1, 2);

        Assert.Equal(new Vector2d(2, 3), vector + Fixed64.One);
        Assert.Equal(new Vector2d(2, 3), Fixed64.One + vector);
        Assert.Equal(new Vector2d(4, 6), vector + (3, 4));
        Assert.Equal(new Vector2d(4, 6), (3, 4) + vector);
        Assert.Equal(new Vector2d(-2, -2), vector - (3, 4));
        Assert.Equal(new Vector2d(2, 2), (3, 4) - vector);
        Assert.Equal(new Vector2d(-1, -2), -vector);
        Assert.Equal(new Vector2d(2, 4), vector * new Fixed64(2));
        Assert.Equal(new Vector2d(2, 4), new Fixed64(2) * vector);
        Assert.Equal(new Vector2d(2, 6), vector * new Vector2d(2, 3));
        Assert.Equal(new Vector2d(Fixed64.FromDouble(0.5), Fixed64.One), vector / new Fixed64(2));
    }

    [Fact]
    public void Normalized_WithSmallRepresentableComponents_ShouldRemainUnitLength()
    {
        var vector = new Vector2d(Fixed64.FromRaw(21_011_293), Fixed64.FromRaw(3_311_656));

        Vector2d normalized = vector.Normalized;

        Assert.True(FixedMath.Abs(normalized.Magnitude - Fixed64.One) <= Fixed64.Epsilon);
    }

    [Fact]
    public void MagnitudeAndNormalized_WithMinimumRepresentableAxis_ShouldPreserveDirection()
    {
        var vector = new Vector2d(Fixed64.MinIncrement, Fixed64.Zero);

        Assert.True(Vector2d.TryGetMagnitude(vector, out Fixed64 magnitude));
        Assert.Equal(Fixed64.MinIncrement, magnitude);
        Assert.Equal(Vector2d.Right, vector.Normalized);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(1, 2)]
    public void Normalized_WithSubSquareResolutionComponents_ShouldPreserveRatio(long xRaw, long yRaw)
    {
        var vector = new Vector2d(Fixed64.FromRaw(xRaw), Fixed64.FromRaw(yRaw));
        Vector2d expected = new Vector2d((Fixed64)xRaw, (Fixed64)yRaw).Normalized;

        Vector2d normalized = vector.Normalized;
        var inPlace = vector;
        Vector2d inPlaceResult = inPlace.NormalizeInPlace(out Fixed64 inPlaceMagnitude);

        Assert.True(FixedMath.Abs(normalized.X - expected.X) <= Fixed64.Epsilon);
        Assert.True(FixedMath.Abs(normalized.Y - expected.Y) <= Fixed64.Epsilon);
        Assert.True(FixedMath.Abs(normalized.Magnitude - Fixed64.One) <= Fixed64.Epsilon);
        Assert.True(Vector2d.TryGetMagnitude(vector, out Fixed64 expectedMagnitude));
        Assert.Equal(expectedMagnitude, inPlaceMagnitude);
        Assert.True(FixedMath.Abs(inPlaceResult.X - expected.X) <= Fixed64.Epsilon);
        Assert.True(FixedMath.Abs(inPlaceResult.Y - expected.Y) <= Fixed64.Epsilon);
        Assert.Equal(inPlaceResult, inPlace);
    }

    #region Test: Serialization

    [Fact]
    public void Vector2d_NetSerialization_RoundTripMaintainsData()
    {
        var originalValue = new Vector2d(Fixed64.Pi, Fixed64.HalfPi);

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<Vector2d>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void Vector2d_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Vector2d originalValue = new(Fixed64.Pi, Fixed64.HalfPi);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Vector2d deserializedValue = MemoryPackSerializer.Deserialize<Vector2d>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion
}
