using System;
using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

public class Vector2dTests
{
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
    public void MyMagnitude_CalculatesCorrectMagnitude()
    {
        var vector = new Vector2d(3, 4);
        var result = vector.Magnitude;

        Assert.Equal(new Fixed64(5), result); // The magnitude of (3, 4) is 5 (3-4-5 triangle)
    }

    [Fact]
    public void SqrMagnitude_CalculatesCorrectSquareMagnitude()
    {
        var vector = new Vector2d(3, 4);
        var result = vector.SqrMagnitude;

        Assert.Equal(new Fixed64(25), result); // The squared magnitude of (3, 4) is 25 (3^2 + 4^2)
    }


    [Fact]
    public void NormalizeInPlace_NormalizesVectorCorrectly()
    {
        var vector = new Vector2d(3, 4);
        vector.Normalize();

        var expected = new Vector2d(new Fixed64(0.6), new Fixed64(0.8)); // Normalized vector (0.6, 0.8)
        Assert.True(vector.FuzzyEqual(expected, new Fixed64(0.0001)));
    }

    [Fact]
    public void LerpInPlace_InterpolatesBetweenVectorsCorrectly()
    {
        var start = new Vector2d(0, 0);
        var end = new Vector2d(10, 10);
        var amount = new Fixed64(0.5); // 50% interpolation

        start.LerpInPlace(end, amount);
        Assert.Equal(new Vector2d(5, 5), start); // Should be halfway between (0, 0) and (10, 10)
    }

    [Fact]
    public void RotateInPlace_RotatesVectorCorrectly()
    {
        var vector = new Vector2d(1, 0);
        var cos = FixedMath.Cos(FixedMath.PiOver2); // 90° cosine
        var sin = FixedMath.Sin(FixedMath.PiOver2); // 90° sine

        vector.RotateInPlace(cos, sin);
        Assert.True(vector.FuzzyEqual(new Vector2d(0, 1), new Fixed64(0.0001))); // (1, 0) rotated 90° becomes (0, 1)
    }

    [Fact]
    public void RotateInverse_RotatesVectorInOppositeDirection()
    {
        var vector = new Vector2d(1, 0);
        var cos = FixedMath.Cos(FixedMath.PiOver2); // 90° cosine
        var sin = FixedMath.Sin(FixedMath.PiOver2); // 90° sine

        vector.RotateInverse(cos, sin);
        Assert.True(vector.FuzzyEqual(new Vector2d(0, -1), new Fixed64(0.0001))); // Should rotate -90° to (0, -1)
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
    public void ScaleInPlace_ScalesVectorCorrectly()
    {
        var vector = new Vector2d(2, 3);
        var scaleFactor = new Fixed64(2);
        vector.ScaleInPlace(scaleFactor);

        Assert.Equal(new Vector2d(4, 6), vector); // (2, 3) scaled by 2 becomes (4, 6)
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
    public void SqrDistance_ComputesSquareDistanceCorrectly()
    {
        var vector1 = new Vector2d(1, 1);
        var vector2 = new Vector2d(4, 5);
        var result = vector1.SqrDistance(vector2);

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
    public void ScaleInPlace_VectorOverload_ScalesPerComponent()
    {
        var vector = new Vector2d(2, 3);

        Assert.Equal(new Vector2d(4, 12), vector.ScaleInPlace(new Vector2d(2, 4)));
    }

    [Fact]
    public void NormalizeOut_ReturnsOriginalMagnitudeAndNormalizedVector()
    {
        var vector = new Vector2d(3, 4);

        var normalized = vector.Normalize(out var magnitude);

        Assert.Equal(new Fixed64(5), magnitude);
        Assert.Equal(vector, normalized);
        Assert.True(vector.FuzzyEqual(new Vector2d(new Fixed64(0.6), new Fixed64(0.8)), new Fixed64(0.0001)));
    }

    [Fact]
    public void NormalizeOut_ZeroVector_ReturnsZeroMagnitude()
    {
        var vector = Vector2d.Zero;

        var normalized = vector.Normalize(out var magnitude);

        Assert.Equal(Fixed64.Zero, magnitude);
        Assert.Equal(Vector2d.Zero, normalized);
    }

    [Fact]
    public void NormalizeOut_UnitVector_LeavesVectorUnchanged()
    {
        var vector = Vector2d.Right;

        var normalized = vector.Normalize(out var magnitude);

        Assert.Equal(Fixed64.One, magnitude);
        Assert.Equal(Vector2d.Right, normalized);
    }

    [Fact]
    public void NormalizeInPlace_DoesNothingForZeroVector()
    {
        var vector = new Vector2d(0, 0);
        vector.Normalize();

        Assert.Equal(new Vector2d(0, 0), vector); // A zero vector remains zero after normalization
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
    public void V2ClampOneInPlace_ClampsCorrectly()
    {
        var vector = new Vector2d(2, -3);
        var result = vector.ClampOneInPlace();

        Assert.Equal(new Vector2d(1, -1), result); // Clamps x and y to [-1, 1]
    }

    [Fact]
    public void V2AbsAndSign_ExtensionsReturnComponentWiseResults()
    {
        var vector = new Vector2d(-2, 0.5);

        Assert.Equal(new Vector2d(2, 0.5), vector.Abs());
        Assert.Equal(new Vector2d(-1, 1), vector.Sign());
    }

    [Fact]
    public void V2ToDegrees_ConvertsCorrectly()
    {
        var radians = new Vector2d(FixedMath.PiOver2, FixedMath.PI); // (90°, 180°)
        var result = radians.ToDegrees();

        Assert.True(result.FuzzyEqual(new Vector2d(90, 180))); // Converts radians to degrees
    }

    [Fact]
    public void V2ToRadians_ConvertsCorrectly()
    {
        var degrees = new Vector2d(90, 180);
        var result = degrees.ToRadians();

        Assert.True(result.FuzzyEqual(new Vector2d(FixedMath.PiOver2, FixedMath.PI))); // Converts degrees to radians
    }

    [Fact]
    public void V2FuzzyEqualAbsolute_ComparesCorrectly_WithAllowedDifference()
    {
        var vector1 = new Vector2d(2, 2);
        var vector2 = new Vector2d(2.1, 2.1);
        var allowedDifference = new Fixed64(0.15);

        Assert.True(vector1.FuzzyEqualAbsolute(vector2, allowedDifference)); // Approximate equality with a 0.15 difference
    }

    [Fact]
    public void V2FuzzyEqual_ComparesCorrectly_WithDefaultTolerance()
    {
        var vector1 = new Vector2d(100, 100);
        var vector2 = new Vector2d(100.0000008537, 100.0000008537); // Small difference

        // Use FuzzyEqual with the default tolerance, which is small (e.g., 0.01% difference)
        Assert.True(vector1.FuzzyEqual(vector2)); // The difference should be within the default tolerance

        vector2 = new Vector2d(100.0001, 100.0001); // big difference
        Assert.False(vector1.FuzzyEqual(vector2)); // The difference should be outside the default tolerance
    }


    [Fact]
    public void V2FuzzyEqual_ComparesCorrectly_WithCustomPercentage()
    {
        var vector1 = new Vector2d(100, 100);
        var vector2 = new Vector2d(102, 102);
        var percentage = new Fixed64(0.02); // Allow a 2% difference

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
    public void V2SqrDistance_CalculatesCorrectly()
    {
        var vector1 = new Vector2d(0, 0);
        var vector2 = new Vector2d(3, 4); // Squared distance should be 25
        var result = vector1.SqrDistance(vector2);

        Assert.Equal(new Fixed64(25), result); // 3^2 + 4^2 = 25
    }

    [Fact]
    public void V2Rotate_RotatesVectorCorrectly()
    {
        var vector = new Vector2d(1, 0);
        var angle = FixedMath.PiOver2; // Rotate by 90° (π/2 radians)
        var result = vector.Rotate(angle);

        Assert.True(result.FuzzyEqual(new Vector2d(0, 1), new Fixed64(0.0001))); // Should rotate to (0, 1)
    }

    [Fact]
    public void Lerped_ReturnsInterpolatedCopyWithoutMutatingOriginal()
    {
        var start = new Vector2d(0, 0);

        var result = start.Lerped(new Vector2d(10, 20), new Fixed64(0.25));

        Assert.Equal(new Vector2d(new Fixed64(2.5), new Fixed64(5)), result);
        Assert.Equal(Vector2d.Zero, start);
    }

    [Fact]
    public void Rotated_Overloads_ReturnRotatedCopiesWithoutMutatingOriginal()
    {
        var vector = new Vector2d(1, 0);
        var rotation = Vector2d.CreateRotation(FixedMath.PiOver2);

        var rotatedByComponents = vector.Rotated(rotation.x, rotation.y);
        var rotatedByVector = vector.Rotated(rotation);

        Assert.True(rotatedByComponents.FuzzyEqual(Vector2d.Forward, new Fixed64(0.0001)));
        Assert.True(rotatedByVector.FuzzyEqual(Vector2d.Forward, new Fixed64(0.0001)));
        Assert.Equal(Vector2d.Right, vector);
    }

    [Fact]
    public void Reflected_Overloads_ReturnReflectedCopiesWithoutMutatingOriginal()
    {
        var vector = new Vector2d(1, 1);
        var axis = Vector2d.Forward;

        var reflectedByComponents = vector.Reflected(axis.x, axis.y);
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

        var reflected = vector.ReflectInPlace(axis.x, axis.y, projection);

        Assert.Equal(new Vector2d(-1, 1), reflected);
    }

    [Fact]
    public void ForwardDirection_ReturnsExpectedUnitVector()
    {
        var result = Vector2d.ForwardDirection(FixedMath.PiOver2);

        Assert.True(result.FuzzyEqual(Vector2d.Forward, new Fixed64(0.0001)));
    }

    [Fact]
    public void Vector2d_PropertiesAndHashes_ReturnExpectedValues()
    {
        var vector = new Vector2d(new Fixed64(3), new Fixed64(4));

        Assert.Equal(new Vector2d(new Fixed64(0.6), new Fixed64(0.8)), vector.Normal);
        Assert.Equal(new Fixed64(5), vector.Magnitude);
        Assert.Equal(new Fixed64(25), vector.SqrMagnitude);
        Assert.Equal(vector.x.m_rawValue * 31 + vector.y.m_rawValue * 7, vector.LongStateHash);
        Assert.Equal((int)(vector.LongStateHash % int.MaxValue), vector.StateHash);
        Assert.Equal(vector.StateHash, vector.GetHashCode());
    }

    [Fact]
    public void Vector2d_LerpHelpers_CoverClampAndCopyBranches()
    {
        var vector = new Vector2d(1, 2);

        Assert.Equal(new Vector2d(10, 20), vector.LerpInPlace(new Vector2d(10, 20), new Fixed64(2)));
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
        var vector = new Vector2d(new Fixed64(1.25), new Fixed64(-2.5));

        Assert.Equal(new Fixed64(25), Vector2d.SqrDistance(Vector2d.Zero, new Vector2d(3, 4)));
        Assert.Equal(new Fixed64(11), Vector2d.Dot(new Vector2d(1, 2), new Vector2d(3, 4)));
        Assert.Equal(new Vector2d(8, 15), Vector2d.Scale(new Vector2d(2, 3), new Vector2d(4, 5)));
        Assert.Equal("(1.25, -2.5)", vector.ToString());
        Assert.Equal(new Vector3d(new Fixed64(1.25), new Fixed64(7), new Fixed64(-2.5)), vector.ToVector3d(new Fixed64(7)));

        vector.Deconstruct(out float fx, out float fy);
        vector.Deconstruct(out int ix, out int iy);

        Assert.Equal(1.25f, fx);
        Assert.Equal(-2.5f, fy);
        Assert.Equal(1, ix);
        Assert.Equal(-2, iy);
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
        Assert.Equal(new Vector2d(2, 6), vector * new Vector2d(2, 3));
        Assert.Equal(new Vector2d(new Fixed64(0.5), Fixed64.One), vector / new Fixed64(2));
    }

    #region Test: Serialization

    [Fact]
    public void Vector2d_NetSerialization_RoundTripMaintainsData()
    {
        var originalValue = new Vector2d(FixedMath.PI, FixedMath.PiOver2);

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

    [Fact]
    public void Vector2d_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Vector2d originalValue = new Vector2d(FixedMath.PI, FixedMath.PiOver2);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Vector2d deserializedValue = MemoryPackSerializer.Deserialize<Vector2d>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    #endregion
}
