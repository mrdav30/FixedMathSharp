using System;
using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;

using Xunit;

namespace FixedMathSharp.Tests;

public class Vector3dTests
{
    #region Test: Constructors

    [Fact]
    public void Constructor_Default_InitializesToZero()
    {
        var vector = new Vector3d();
        Assert.Equal(Fixed64.Zero, vector.x);
        Assert.Equal(Fixed64.Zero, vector.y);
        Assert.Equal(Fixed64.Zero, vector.z);
    }

    [Fact]
    public void Constructor_Fixed64_InitializesComponents()
    {
        var x = new Fixed64(1);
        var y = new Fixed64(2);
        var z = new Fixed64(3);
        var vector = new Vector3d(x, y, z);
        Assert.Equal(x, vector.x);
        Assert.Equal(y, vector.y);
        Assert.Equal(z, vector.z);
    }

    [Fact]
    public void Constructor_Float_InitializesComponents()
    {
        var vector = new Vector3d(1.5f, -2.5f, 3.0f);
        Assert.Equal(new Fixed64(1.5), vector.x);
        Assert.Equal(new Fixed64(-2.5), vector.y);
        Assert.Equal(new Fixed64(3.0), vector.z);
    }

    #endregion

    #region Test: Magnitude and Normalization

    [Fact]
    public void MyMagnitude_CalculatesCorrectly()
    {
        var vector = new Vector3d(3, 4, 0); // Magnitude should be 5 (3-4-5 triangle).
        Assert.Equal(new Fixed64(5), vector.Magnitude);
    }

    [Fact]
    public void Normalize_ProducesUnitVector()
    {
        var vector = new Vector3d(3, 4, 0);
        var normalized = vector.Normal;
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, normalized.Magnitude);
    }

    [Fact]
    public void NormalizeInPlace_ProducesUnitVector()
    {
        var vector = new Vector3d(0, 0, 5);
        vector.Normalize();
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, vector.Magnitude);
    }

    [Fact]
    public void IsNormalized_IdentifiesNormalizedVector()
    {
        var normalizedVector = new Vector3d(1, 0, 0).Normalize();
        Assert.True(normalizedVector.IsNormalized());
    }

    [Fact]
    public void IsNormalized_IdentifiesNonNormalizedVector()
    {
        var nonNormalizedVector = new Vector3d(10, 0, 0);
        Assert.False(nonNormalizedVector.IsNormalized());
    }

    [Fact]
    public void IsNormalized_ZeroVector_ReturnsFalse()
    {
        var zeroVector = new Vector3d(0, 0, 0);
        Assert.False(zeroVector.IsNormalized());
    }

    [Fact]
    public void AllComponentsGreaterThanEpsilon_ReturnsTrue_WhenAllComponentsExceedEpsilon()
    {
        var vector = new Vector3d(
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon + Fixed64.One);

        Assert.True(vector.AllComponentsGreaterThanEpsilon());
    }

    [Fact]
    public void AllComponentsGreaterThanEpsilon_ReturnsFalse_WhenAComponentIsAtOrBelowEpsilon()
    {
        var vector = new Vector3d(
            Fixed64.Epsilon + Fixed64.One,
            Fixed64.Epsilon,
            Fixed64.Epsilon + Fixed64.One);

        Assert.False(vector.AllComponentsGreaterThanEpsilon());
    }

    #endregion

    #region Test: Arithmetic 

    [Fact]
    public void Addition_AddsComponentsCorrectly()
    {
        var v1 = new Vector3d(1, 2, 3);
        var v2 = new Vector3d(4, 5, 6);
        var result = v1 + v2;
        Assert.Equal(new Vector3d(5, 7, 9), result);
    }

    [Fact]
    public void Subtraction_SubtractsComponentsCorrectly()
    {
        var v1 = new Vector3d(5, 7, 9);
        var v2 = new Vector3d(1, 2, 3);
        var result = v1 - v2;
        Assert.Equal(new Vector3d(4, 5, 6), result);
    }

    [Fact]
    public void Scaling_ScalesComponentsCorrectly()
    {
        var vector = new Vector3d(1, -2, 3);
        var result = vector * new Fixed64(2);
        Assert.Equal(new Vector3d(2, -4, 6), result);
    }

    [Fact]
    public void Division_DividesComponentsCorrectly()
    {
        var vector = new Vector3d(2, -4, 6);
        var result = vector / new Fixed64(2);
        Assert.Equal(new Vector3d(1, -2, 3), result);
    }

    [Fact]
    public void Indexer_GetAndSet_RoundTripsComponents()
    {
        var vector = new Vector3d(1, 2, 3);

        Assert.Equal(Fixed64.One, vector[0]);
        Assert.Equal(new Fixed64(2), vector[1]);
        Assert.Equal(new Fixed64(3), vector[2]);

        vector[0] = new Fixed64(4);
        vector[1] = new Fixed64(5);
        vector[2] = new Fixed64(6);

        Assert.Equal(new Vector3d(4, 5, 6), vector);
    }

    [Fact]
    public void Indexer_InvalidIndex_Throws()
    {
        var vector = new Vector3d(1, 2, 3);

        Assert.Throws<IndexOutOfRangeException>(() => _ = vector[-1]);
        Assert.Throws<IndexOutOfRangeException>(() => _ = vector[3]);
        Assert.Throws<IndexOutOfRangeException>(() => vector[-1] = Fixed64.One);
        Assert.Throws<IndexOutOfRangeException>(() => vector[3] = Fixed64.One);
    }

    [Fact]
    public void Set_UpdatesComponents()
    {
        var vector = new Vector3d(1, 2, 3);

        var result = vector.Set(new Fixed64(7), new Fixed64(8), new Fixed64(9));

        Assert.Equal(new Vector3d(7, 8, 9), result);
        Assert.Equal(result, vector);
    }

    [Fact]
    public void AddInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector3d(1, 2, 3);

        Assert.Equal(new Vector3d(4, 5, 6), vector.AddInPlace(new Fixed64(3)));
        Assert.Equal(new Vector3d(5, 7, 9), vector.AddInPlace(Fixed64.One, new Fixed64(2), new Fixed64(3)));
        Assert.Equal(new Vector3d(7, 10, 13), vector.AddInPlace(new Vector3d(2, 3, 4)));
    }

    [Fact]
    public void SubtractInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector3d(10, 12, 14);

        Assert.Equal(new Vector3d(9, 11, 13), vector.SubtractInPlace(Fixed64.One));
        Assert.Equal(new Vector3d(7, 8, 9), vector.SubtractInPlace(new Fixed64(2), new Fixed64(3), new Fixed64(4)));
        Assert.Equal(new Vector3d(6, 6, 6), vector.SubtractInPlace(new Vector3d(1, 2, 3)));
    }

    [Fact]
    public void ScaleInPlace_Overloads_ModifyVectorCorrectly()
    {
        var vector = new Vector3d(2, 3, 4);

        Assert.Equal(new Vector3d(4, 6, 8), vector.ScaleInPlace(new Fixed64(2)));
        Assert.Equal(new Vector3d(8, 18, 32), vector.ScaleInPlace(new Vector3d(2, 3, 4)));
    }

    [Fact]
    public void NormalizeOut_ReturnsOriginalMagnitudeAndNormalizedVector()
    {
        var vector = new Vector3d(0, 3, 4);

        var normalized = vector.Normalize(out var magnitude);

        Assert.Equal(new Fixed64(5), magnitude);
        Assert.Equal(vector, normalized);
        Assert.True(vector.FuzzyEqual(new Vector3d(Fixed64.Zero, new Fixed64(0.6), new Fixed64(0.8)), new Fixed64(0.0001)));
    }

    [Fact]
    public void NormalizeOut_ZeroVector_ReturnsZeroMagnitude()
    {
        var vector = Vector3d.Zero;

        var normalized = vector.Normalize(out var magnitude);

        Assert.Equal(Fixed64.Zero, magnitude);
        Assert.Equal(Vector3d.Zero, normalized);
    }

    [Fact]
    public void NormalizeOut_UnitVector_LeavesVectorUnchanged()
    {
        var vector = Vector3d.Right;

        var normalized = vector.Normalize(out var magnitude);

        Assert.Equal(Fixed64.One, magnitude);
        Assert.Equal(Vector3d.Right, normalized);
    }

    #endregion

    #region Test: Dot and Cross Product

    [Fact]
    public void Dot_ProductCalculatesCorrectly()
    {
        var v1 = new Vector3d(1, 0, 0);
        var v2 = new Vector3d(0, 1, 0);
        Assert.Equal(Fixed64.Zero, Vector3d.Dot(v1, v2)); // Perpendicular vectors, dot product should be zero.
    }

    [Fact]
    public void Cross_ProductCalculatesCorrectly()
    {
        var v1 = new Vector3d(1, 0, 0);
        var v2 = new Vector3d(0, 1, 0);
        var result = Vector3d.Cross(v1, v2);
        Assert.Equal(new Vector3d(0, 0, 1), result); // Standard right-hand rule cross product.
    }

    #endregion

    #region Test: Distance and SqrDistance

    [Fact]
    public void Distance_CalculatesCorrectly()
    {
        var v1 = new Vector3d(0, 0, 0);
        var v2 = new Vector3d(0, 3, 4);
        Assert.Equal(new Fixed64(5), Vector3d.Distance(v1, v2)); // 3-4-5 triangle.
    }

    [Fact]
    public void SqrDistance_CalculatesCorrectly()
    {
        var v1 = new Vector3d(0, 0, 0);
        var v2 = new Vector3d(0, 3, 4);
        Assert.Equal(new Fixed64(25), Vector3d.SqrDistance(v1, v2)); // 3^2 + 4^2 = 25.
    }

    #endregion

    #region Test: Projection 

    [Fact]
    public void Project_ReturnsCorrectProjection()
    {
        var v1 = new Vector3d(2, 3, 4);
        var v2 = new Vector3d(1, 0, 0); // Project onto the X-axis.
        var result = Vector3d.Project(v1, v2);
        Assert.Equal(new Vector3d(2, 0, 0), result);
    }

    [Fact]
    public void ProjectOnPlane_ReturnsCorrectProjection()
    {
        var v1 = new Vector3d(1, 1, 1);
        var planeNormal = new Vector3d(0, 1, 0); // Y-plane.
        var result = Vector3d.ProjectOnPlane(v1, planeNormal);
        Assert.Equal(new Vector3d(1, 0, 1), result);
    }

    #endregion

    #region Test: Rotation 

    [Fact]
    public void RightHandNormal_RotatesCorrectly()
    {
        var vector = new Vector3d(1, 0, 0);
        var result = vector.RightHandNormal;
        Assert.Equal(new Vector3d(0, 0, -1), result);
    }

    [Fact]
    public void LeftHandNormal_RotatesCorrectly()
    {
        var vector = new Vector3d(1, 0, 0);
        var result = vector.LeftHandNormal;
        Assert.Equal(new Vector3d(0, 0, 1), result);
    }

    [Fact]
    public void Direction_ZeroAngles_ReturnsForward()
    {
        var angles = new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        Assert.Equal(Vector3d.Forward, angles.Direction);
    }

    #endregion

    #region Test: Equality 

    [Fact]
    public void Equality_TwoIdenticalVectors_AreEqual()
    {
        var v1 = new Vector3d(1, 2, 3);
        var v2 = new Vector3d(1, 2, 3);
        Assert.True(v1 == v2);
    }

    [Fact]
    public void Equality_TwoDifferentVectors_AreNotEqual()
    {
        var v1 = new Vector3d(1, 2, 3);
        var v2 = new Vector3d(3, 2, 1);
        Assert.True(v1 != v2);
    }

    #endregion

    #region Test: Static Math

    [Fact]
    public void Lerp_ReturnsCorrectInterpolation()
    {
        var start = new Vector3d(1, 1, 1);
        var end = new Vector3d(3, 3, 3);

        // Lerp at t = 0 (should return the start vector)
        var result = Vector3d.Lerp(start, end, Fixed64.Zero);
        Assert.Equal(start, result);

        // Lerp at t = 1 (should return the end vector)
        result = Vector3d.Lerp(start, end, Fixed64.One);
        Assert.Equal(end, result);

        // Lerp at t = 0.5 (should be the midpoint)
        var midpoint = new Vector3d(2, 2, 2);
        result = Vector3d.Lerp(start, end, new Fixed64(0.5));
        Assert.Equal(midpoint, result);
    }

    [Fact]
    public void UnclampedLerp_AllowsInterpolationOutsideRange()
    {
        var start = new Vector3d(1, 1, 1);
        var end = new Vector3d(3, 3, 3);

        // Unclamped Lerp at t = -1 (should go past start)
        var result = Vector3d.UnclampedLerp(start, end, new Fixed64(-1));
        var expected = new Vector3d(-1, -1, -1);
        Assert.Equal(expected, result);

        // Unclamped Lerp at t = 2 (should go past end)
        result = Vector3d.UnclampedLerp(start, end, new Fixed64(2));
        expected = new Vector3d(5, 5, 5);
        Assert.Equal(expected, result);
    }

    [Fact]
    public void SpeedLerp_DoesNotOvershoot()
    {
        var start = new Vector3d(0, 0, 0);
        var end = new Vector3d(10, 0, 0);
        var speed = new Fixed64(5);
        var dt = Fixed64.One; // Simulating 1 second of delta time

        // Speed lerp should move the vector but not overshoot the target
        var result = Vector3d.SpeedLerp(start, end, speed, dt);
        var expected = new Vector3d(5, 0, 0);
        Assert.Equal(expected, result);

        // Test where it shouldn't overshoot
        result = Vector3d.SpeedLerp(result, end, speed, dt);
        Assert.Equal(end, result);
    }

    [Fact]
    public void Slerp_InterpolatesOnSphere()
    {
        var start = new Vector3d(1, 0, 0);
        var end = new Vector3d(0, 1, 0);

        // Slerp at t = 0 (should return the start vector)
        var result = Vector3d.Slerp(start, end, Fixed64.Zero);
        Assert.Equal(start, result);

        // Slerp at t = 1 (should return the end vector)
        result = Vector3d.Slerp(start, end, Fixed64.One);
        Assert.Equal(end, result);

        // Slerp at t = 0.5 (should return a midpoint vector on the unit circle)
        result = Vector3d.Slerp(start, end, new Fixed64(0.5));
        // Since the midpoint on a unit circle between (1, 0, 0) and (0, 1, 0) should be (0.707, 0.707, 0)
        var expected = new Vector3d(new Fixed64(0.70710678118), new Fixed64(0.70710678118), Fixed64.Zero);
        Assert.True(Vector3d.AreAlmostParallel(result, expected, new Fixed64(0.999)));
    }

    [Fact]
    public void Slerp_ClampsDotProductBeforeAcos()
    {
        var start = Vector3d.Right;
        var end = new Vector3d(Fixed64.FromRaw(Fixed64.One.m_rawValue + 1), Fixed64.Zero, Fixed64.Zero);

        var result = Vector3d.Slerp(start, end, Fixed64.Half);

        Assert.Equal(start, result);
    }

    [Fact]
    public void StaticMagnitude_CalculatesCorrectly()
    {
        var vector = new Vector3d(3, 4, 0); // 3-4-5 triangle
        var result = Vector3d.GetMagnitude(vector);
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void Dot_CalculatesCorrectly()
    {
        var v1 = new Vector3d(1, 0, 0);
        var v2 = new Vector3d(0, 1, 0);

        // Perpendicular vectors should have dot product of 0
        var result = Vector3d.Dot(v1, v2);
        Assert.Equal(Fixed64.Zero, result);

        // Parallel vectors (should be 1 * 1)
        v2 = new Vector3d(1, 0, 0);
        result = Vector3d.Dot(v1, v2);
        Assert.Equal(Fixed64.One, result);
    }

    [Fact]
    public void StaticDistance_CalculatesCorrectly()
    {
        var v1 = new Vector3d(0, 0, 0);
        var v2 = new Vector3d(3, 4, 0); // Distance should be 5 (3-4-5 triangle)
        var result = Vector3d.Distance(v1, v2);
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void StaticSqrDistance_CalculatesCorrectly()
    {
        var v1 = new Vector3d(0, 0, 0);
        var v2 = new Vector3d(3, 4, 0); // Squared distance should be 25
        var result = Vector3d.SqrDistance(v1, v2);
        Assert.Equal(new Fixed64(25), result);
    }

    [Fact]
    public void Project_CalculatesCorrectly()
    {
        var vector = new Vector3d(2, 2, 0);
        var onNormal = new Vector3d(1, 0, 0); // Project onto X-axis
        var result = Vector3d.Project(vector, onNormal);
        var expected = new Vector3d(2, 0, 0);
        Assert.Equal(expected, result);
    }

    [Fact]
    public void Clamp_CalculatesCorrectly()
    {
        var value = new Vector3d(2, 2, 2);
        var min = new Vector3d(1, 1, 1);
        var max = new Vector3d(3, 3, 3);
        var result = Vector3d.Clamp(value, min, max);
        Assert.Equal(value, result);

        // Test clamping below the min
        value = new Vector3d(0, 0, 0);
        result = Vector3d.Clamp(value, min, max);
        Assert.Equal(min, result);

        // Test clamping above the max
        value = new Vector3d(4, 4, 4);
        result = Vector3d.Clamp(value, min, max);
        Assert.Equal(max, result);
    }

    [Fact]
    public void Angle_CalculatesCorrectly()
    {
        var v1 = new Vector3d(1, 0, 0);
        var v2 = new Vector3d(0, 1, 0);

        // The angle between (1, 0, 0) and (0, 1, 0) should be 90 degrees
        var result = Vector3d.Angle(v1, v2);
        var expected = FixedMath.RadToDeg(FixedMath.PiOver2);
        FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
    }

    [Fact]
    public void Midpoint_CalculatesCorrectly()
    {
        var v1 = new Vector3d(1, 1, 1);
        var v2 = new Vector3d(3, 3, 3);
        var result = Vector3d.Midpoint(v1, v2);
        Assert.Equal(new Vector3d(2, 2, 2), result);
    }

    [Fact]
    public void Midpoint_WithNegativeValues_CalculatesCorrectly()
    {
        var v1 = new Vector3d(-1, -1, -1);
        var v2 = new Vector3d(1, 1, 1);
        var result = Vector3d.Midpoint(v1, v2);
        Assert.Equal(new Vector3d(0, 0, 0), result);
    }

    [Fact]
    public void ClosestPointsOnTwoLines_NonParallelSegments()
    {
        var line1Start = new Vector3d(0, 0, 0);
        var line1End = new Vector3d(1, 1, 0);
        var line2Start = new Vector3d(1, 0, 0);
        var line2End = new Vector3d(0, 1, 0);

        var (pointOnLine1, pointOnLine2) = Vector3d.ClosestPointsOnTwoLines(line1Start, line1End, line2Start, line2End);

        // These points should be the same since the lines intersect at (0.5, 0.5, 0)
        var expectedIntersection = new Vector3d(0.5, 0.5, 0);
        Assert.Equal(expectedIntersection, pointOnLine1);
        Assert.Equal(expectedIntersection, pointOnLine2);
    }


    [Fact]
    public void ClosestPointsOnTwoLines_ParallelSegments()
    {
        var line1Start = new Vector3d(0, 0, 0);
        var line1End = new Vector3d(1, 1, 0);
        var line2Start = new Vector3d(0, 0, 1);
        var line2End = new Vector3d(1, 1, 1);
        var (pointOnLine1, pointOnLine2) = Vector3d.ClosestPointsOnTwoLines(line1Start, line1End, line2Start, line2End);

        // Points on the same relative positions on the parallel lines
        Assert.Equal(new Vector3d(0, 0, 0), pointOnLine1);
        Assert.Equal(new Vector3d(0, 0, 1), pointOnLine2);
    }

    [Fact]
    public void ClosestPointsOnTwoLines_ClampsToFirstSegmentEnd_WhenIntersectionFallsBeyondSegment()
    {
        var line1Start = new Vector3d(0, 0, 0);
        var line1End = new Vector3d(1, 0, 0);
        var line2Start = new Vector3d(2, -1, 0);
        var line2End = new Vector3d(2, 1, 0);

        var (pointOnLine1, pointOnLine2) = Vector3d.ClosestPointsOnTwoLines(line1Start, line1End, line2Start, line2End);

        Assert.Equal(new Vector3d(1, 0, 0), pointOnLine1);
        Assert.Equal(new Vector3d(2, 0, 0), pointOnLine2);
    }

    [Fact]
    public void ClosestPointsOnTwoLines_ClampsToFirstSegmentStart_WhenIntersectionFallsBeforeSegment()
    {
        var line1Start = new Vector3d(0, 0, 0);
        var line1End = new Vector3d(1, 0, 0);
        var line2Start = new Vector3d(-1, -1, 0);
        var line2End = new Vector3d(-1, 1, 0);

        var (pointOnLine1, pointOnLine2) = Vector3d.ClosestPointsOnTwoLines(line1Start, line1End, line2Start, line2End);

        Assert.Equal(new Vector3d(0, 0, 0), pointOnLine1);
        Assert.Equal(new Vector3d(-1, 0, 0), pointOnLine2);
    }

    [Fact]
    public void ClosestPointOnLineSegment_InsideSegment()
    {
        var a = new Vector3d(0, 0, 0);
        var b = new Vector3d(2, 0, 0);
        var p = new Vector3d(1, 0, 1); // Above the midpoint
        var result = Vector3d.ClosestPointOnLineSegment(a, b, p);
        Assert.Equal(new Vector3d(1, 0, 0), result); // Should project to (1, 0, 0)
    }

    [Fact]
    public void ClosestPointOnLineSegment_OutsideSegment()
    {
        var a = new Vector3d(0, 0, 0);
        var b = new Vector3d(2, 0, 0);
        var p = new Vector3d(3, 0, 0); // Outside segment
        var result = Vector3d.ClosestPointOnLineSegment(a, b, p);
        Assert.Equal(b, result); // Should clamp to the endpoint b
    }

    [Fact]
    public void Max_CalculatesCorrectly()
    {
        var v1 = new Vector3d(1, 5, 3);
        var v2 = new Vector3d(2, 4, 3);
        var result = Vector3d.Max(v1, v2);
        Assert.Equal(new Vector3d(2, 5, 3), result);
    }

    [Fact]
    public void Min_CalculatesCorrectly()
    {
        var v1 = new Vector3d(1, 5, 3);
        var v2 = new Vector3d(2, 4, 3);
        var result = Vector3d.Min(v1, v2);
        Assert.Equal(new Vector3d(1, 4, 3), result);
    }

    [Fact]
    public void MatrixMultiplication_WithFixed3x3_TransformsVectorFromBothSides()
    {
        var matrix = new Fixed3x3(
            new Fixed64(2), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, new Fixed64(3), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, new Fixed64(4)
        );
        var vector = new Vector3d(1, 2, 3);
        var expected = new Vector3d(2, 6, 12);

        Assert.Equal(expected, matrix * vector);
        Assert.Equal(expected, vector * matrix);
    }

    [Fact]
    public void MatrixMultiplication_WithAffineFixed4x4_TransformsPointFromBothSides()
    {
        var matrix = Fixed4x4.Identity;
        matrix.m00 = new Fixed64(2);
        matrix.m11 = new Fixed64(3);
        matrix.m22 = new Fixed64(4);
        matrix.m30 = new Fixed64(5);
        matrix.m31 = new Fixed64(6);
        matrix.m32 = new Fixed64(7);

        var vector = new Vector3d(1, 2, 3);
        var expected = new Vector3d(7, 12, 19);

        Assert.Equal(expected, matrix * vector);
        Assert.Equal(expected, vector * matrix);
    }

    [Fact]
    public void MatrixMultiplication_WithNonAffineFixed4x4_UsesPerspectiveDivision()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );
        var vector = new Vector3d(1, 2, 3);
        var expected = new Vector3d(Fixed64.One, Fixed64.One, new Fixed64(1.5));

        Assert.Equal(expected, matrix * vector);
        Assert.Equal(expected, vector * matrix);
    }

    [Fact]
    public void MatrixMultiplication_WithNonAffineFixed4x4_ZeroWFallsBackToOne()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, -Fixed64.One
        );
        var vector = new Vector3d(1, 2, 3);
        var expected = new Vector3d(2, 2, 3);

        Assert.Equal(expected, matrix * vector);
        Assert.Equal(expected, vector * matrix);
    }

    [Fact]
    public void OperatorOverloads_WithScalarsTuplesFloatsAndComponentArithmetic_WorkCorrectly()
    {
        var vector = new Vector3d(1, 2, 3);

        Assert.Equal(new Vector3d(2, 3, 4), vector + Fixed64.One);
        Assert.Equal(new Vector3d(2, 3, 4), Fixed64.One + vector);
        Assert.Equal(new Vector3d(4, 6, 8), vector + (3, 4, 5));
        Assert.Equal(new Vector3d(4, 6, 8), (3, 4, 5) + vector);
        Assert.Equal(new Vector3d(new Fixed64(1.5), new Fixed64(3.5), new Fixed64(5.5)), vector + (0.5f, 1.5f, 2.5f));
        Assert.Equal(new Vector3d(new Fixed64(1.5), new Fixed64(3.5), new Fixed64(5.5)), (0.5f, 1.5f, 2.5f) + vector);
        Assert.Equal(new Vector3d(-2, -2, -2), vector - (3, 4, 5));
        Assert.Equal(new Vector3d(2, 2, 2), (3, 4, 5) - vector);
        Assert.Equal(new Vector3d(new Fixed64(0.5), new Fixed64(0.5), new Fixed64(0.5)), vector - (0.5f, 1.5f, 2.5f));
        Assert.Equal(new Vector3d(new Fixed64(-0.5), new Fixed64(-0.5), new Fixed64(-0.5)), (0.5f, 1.5f, 2.5f) - vector);
        Assert.Equal(new Vector3d(-1, -2, -3), -vector);
        Assert.Equal(new Vector3d(2, 4, 6), vector * new Fixed64(2));
        Assert.Equal(new Vector3d(2, 4, 6), new Fixed64(2) * vector);
        Assert.Equal(new Vector3d(3, 6, 9), 3 * vector);
        Assert.Equal(new Vector3d(2, 6, 12), vector * new Vector3d(2, 3, 4));
        Assert.Equal(new Vector3d(new Fixed64(0.5), Fixed64.One, new Fixed64(1.5)), vector / new Fixed64(2));
        Assert.Equal(Vector3d.Zero, vector / Fixed64.Zero);
        Assert.Equal(new Vector3d(new Fixed64(0.5), Fixed64.One, new Fixed64(1.5)), vector / 2);
        Assert.Equal(new Vector3d(Fixed64.Zero, new Fixed64(2), Fixed64.Zero), new Vector3d(4, 4, 4) / new Vector3d(0, 2, 0));
    }

    #endregion

    #region Test: Extensions

    [Fact]
    public void V3ClampOneInPlace_ClampsCorrectly()
    {
        var vector = new Vector3d(2, -3, 0.5);
        var result = vector.ClampOneInPlace();

        Assert.Equal(new Vector3d(1, -1, 0.5), result); // Clamps x and y, z stays the same
    }

    [Fact]
    public void V3ClampMagnitude_AbsAndSign_ExtensionsReturnExpectedResults()
    {
        var vector = new Vector3d(6, 8, 0);

        Assert.True(vector.ClampMagnitude(new Fixed64(5)).FuzzyEqual(new Vector3d(3, 4, 0), new Fixed64(0.0001)));
        Assert.Equal(new Vector3d(1, 2, 3), new Vector3d(-1, -2, -3).Abs());
        Assert.Equal(new Vector3d(-1, 1, 0), Vector3dExtensions.Sign(new Vector3d(-2, 4, 0)));
    }

    [Fact]
    public void V3ToDegrees_ConvertsCorrectly_WithTolerance()
    {
        var radians = new Vector3d(FixedMath.PiOver2, FixedMath.PI, Fixed64.Zero);
        var result = radians.ToDegrees();
        var expected = new Vector3d(90, 180, 0);

        // Use FuzzyEqual with a small allowed difference
        Assert.True(result.FuzzyEqualAbsolute(expected, new Fixed64(0.0001)));
    }

    [Fact]
    public void V3ToRadians_ConvertsCorrectly_WithTolerance()
    {
        var degrees = new Vector3d(90, 180, 0);
        var result = degrees.ToRadians();
        var expected = new Vector3d(FixedMath.PiOver2, FixedMath.PI, Fixed64.Zero);

        // Use FuzzyEqual with a small allowed difference
        Assert.True(result.FuzzyEqualAbsolute(expected, new Fixed64(0.0001)));
    }

    [Fact]
    public void V3FuzzyEqualAbsolute_ComparesCorrectly_WithAllowedDifference()
    {
        var vector1 = new Vector3d(2, 2, 2);
        var vector2 = new Vector3d(2.1, 2.1, 2.1);
        var allowedDifference = new Fixed64(0.15);

        // Should be approximately equal given the allowed difference of 0.15
        Assert.True(vector1.FuzzyEqualAbsolute(vector2, allowedDifference));
    }

    [Fact]
    public void V3FuzzyEqual_ComparesCorrectly_WithPercentage()
    {
        var vector1 = new Vector3d(100, 100, 100);
        var vector2 = new Vector3d(101, 101, 101);
        var percentage = new Fixed64(0.02); // Allow a 2% difference

        // Should be approximately equal within the percentage
        Assert.True(vector1.FuzzyEqual(vector2, percentage));
    }

    [Fact]
    public void V3CheckDistance_VerifiesDistanceCorrectly()
    {
        var vector1 = new Vector3d(0, 0, 0);
        var vector2 = new Vector3d(3, 4, 0); // Distance is 5 (3-4-5 triangle)
        var factor = new Fixed64(5);

        Assert.True(vector1.CheckDistance(vector2, factor)); // Distance is 5, so should return true
    }

    [Fact]
    public void V3SqrDistance_CalculatesCorrectly()
    {
        var vector1 = new Vector3d(0, 0, 0);
        var vector2 = new Vector3d(3, 4, 0); // Squared distance should be 25
        var result = Vector3d.SqrDistance(vector1, vector2);

        Assert.Equal(new Fixed64(25), result); // 3^2 + 4^2 = 25
    }

    [Fact]
    public void V3RotateVector_RotatesCorrectly_WithFuzzyEqual()
    {
        var vector = new Vector3d(1, 0, 0);
        var position = new Vector3d(0, 0, 0);
        var quaternion = FixedQuaternion.FromEulerAngles(new Fixed64(0), new Fixed64(0), FixedMath.PiOver2); // 90° rotation around Z-axis

        var result = vector.Rotate(position, quaternion);
        Assert.True(new Vector3d(0, 1, 0).FuzzyEqual(result, new Fixed64(0.0001))); // Allow small error tolerance
    }

    [Fact]
    public void V3InverseRotateVector_AppliesInverseRotationCorrectly_WithFuzzyEqual()
    {
        var vector = new Vector3d(0, 1, 0);
        var position = new Vector3d(0, 0, 0);
        var quaternion = FixedQuaternion.FromEulerAngles(new Fixed64(0), new Fixed64(0), FixedMath.PiOver2); // 90° rotation around Z-axis

        var result = vector.InverseRotate(position, quaternion);
        Assert.True(result.FuzzyEqual(new Vector3d(1, 0, 0), new Fixed64(0.0001))); // Allow small error tolerance
    }

    [Fact]
    public void Vector3d_StaticHelpersAndProperties_CoverRemainingBranches()
    {
        var vector = new Vector3d(3, 4, 0);

        Assert.True(Vector3d.Zero.IsZero);
        Assert.False(vector.IsZero);
        Assert.Equal(Fixed64.Zero, new Vector3d(1, 2, 3).CrossProduct(new Fixed64(4), new Fixed64(5), new Fixed64(6)));
        Assert.Equal(new Vector3d(2, 2, 2), Vector3d.SpeedLerp(Vector3d.Zero, new Vector3d(2, 2, 2), new Fixed64(10), Fixed64.One));
        Assert.Equal(vector, Vector3d.ClampMagnitude(vector, new Fixed64(10)));
        Assert.True(Vector3d.AreParallel(new Vector3d(1, 0, 0), new Vector3d(2, 0, 0)));
        Assert.False(Vector3d.AreParallel(new Vector3d(1, 0, 0), new Vector3d(0, 1, 0)));
        Assert.False(Vector3d.AreAlmostParallel(Vector3d.Right, Vector3d.Up, new Fixed64(0.5)));
        Assert.Equal(Vector3d.Zero, Vector3d.Project(new Vector3d(1, 2, 3), Vector3d.Zero));
        Assert.Equal(new Vector3d(1, 2, 3), Vector3d.ProjectOnPlane(new Vector3d(1, 2, 3), Vector3d.Zero));
        Assert.Equal(Fixed64.Zero, Vector3d.Angle(Vector3d.Zero, Vector3d.Right));
        Assert.Equal(new Vector3d(2, 3, 4), Vector3d.Scale(new Vector3d(1, 1, 1), new Vector3d(2, 3, 4)));
        Assert.Equal(new Vector3d(0, 0, 1), Vector3d.Cross(Vector3d.Right, Vector3d.Up));
        Assert.Equal(new Fixed64(1), Vector3d.CrossProduct(Vector3d.Right, Vector3d.Up));
        Assert.Equal(new Vector3d(2, 5, 4), Vector3d.Max(new Vector3d(2, 1, 4), new Vector3d(1, 5, 0)));
        Assert.Equal(new Vector3d(1, 1, 0), Vector3d.Min(new Vector3d(2, 1, 4), new Vector3d(1, 5, 0)));
    }

    [Fact]
    public void Vector3d_ClosestPointsOnTwoLines_ClampSecondSegmentEndpoints()
    {
        var line1Start = new Vector3d(0, 0, 0);
        var line1End = new Vector3d(10, 0, 0);

        var (pointOnLine1StartClamp, pointOnLine2StartClamp) = Vector3d.ClosestPointsOnTwoLines(
            line1Start,
            line1End,
            new Vector3d(0, 2, 0),
            new Vector3d(0, 3, 0));

        var (pointOnLine1EndClamp, pointOnLine2EndClamp) = Vector3d.ClosestPointsOnTwoLines(
            line1Start,
            line1End,
            new Vector3d(0, -3, 0),
            new Vector3d(0, -2, 0));

        Assert.Equal(Vector3d.Zero, pointOnLine1StartClamp);
        Assert.Equal(new Vector3d(0, 2, 0), pointOnLine2StartClamp);
        Assert.Equal(Vector3d.Zero, pointOnLine1EndClamp);
        Assert.Equal(new Vector3d(0, -2, 0), pointOnLine2EndClamp);
    }

    [Fact]
    public void Vector3d_OperatorsConversionsAndComparers_WorkCorrectly()
    {
        var vector = new Vector3d(new Fixed64(1.25), new Fixed64(2.5), new Fixed64(3.75));
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);

        Assert.Equal(
            new Vector3d(new Fixed64(1.25) / vector.x, new Fixed64(1.25) / vector.y, new Fixed64(1.25) / vector.z),
            new Fixed64(1.25) / vector);
        Assert.True((Vector3d.Right * rotation).FuzzyEqual(rotation * Vector3d.Right, new Fixed64(0.0001)));
        Assert.True(new Vector3d(2, 2, 2) > Vector3d.One);
        Assert.True(Vector3d.One < new Vector3d(2, 2, 2));
        Assert.True(new Vector3d(2, 2, 2) >= new Vector3d(2, 1, 2));
        Assert.True(Vector3d.One <= new Vector3d(1, 2, 3));
        Assert.Equal("(1.25, 2.5, 3.75)", vector.ToString());
        Assert.Equal(new Vector2d(new Fixed64(1.25), new Fixed64(3.75)), vector.ToVector2d());

        vector.Deconstruct(out float fx, out float fy, out float fz);
        vector.Deconstruct(out int ix, out int iy, out int iz);

        Assert.Equal(1.25f, fx);
        Assert.Equal(2.5f, fy);
        Assert.Equal(3.75f, fz);
        Assert.Equal(1, ix);
        Assert.Equal(2, iy);
        Assert.Equal(4, iz);

        Assert.False(vector.Equals("not-a-vector"));
        Assert.True(vector.Equals(vector, new Vector3d(new Fixed64(1.25), new Fixed64(2.5), new Fixed64(3.75))));
        Assert.False(vector.Equals(vector, Vector3d.Zero));
        Assert.Equal(vector.GetHashCode(), vector.GetHashCode(vector));
        Assert.True(vector.CompareTo(Vector3d.One) > 0);
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void Vector3d_NetSerialization_RoundTripMaintainsData()
    {
        var originalValue = new Vector3d(FixedMath.PI, FixedMath.PiOver2, FixedMath.TwoPI);

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<Vector3d>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    [Fact]
    public void Vector3d_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Vector3d originalValue = new Vector3d(FixedMath.PI, FixedMath.PiOver2, FixedMath.TwoPI);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Vector3d deserializedValue = MemoryPackSerializer.Deserialize<Vector3d>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    #endregion
}
